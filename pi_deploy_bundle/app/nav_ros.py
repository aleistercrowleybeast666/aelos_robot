"""显式任务期间的只读ROS缓存；不启动相机/服务、不打开串口。"""
import collections,threading,time
from fall_diagnostics import FallDiag_Assess, FallDiag_ImageLuma
from magnetic_capture import MagneticRebindResult
from magnetic_ros import make_stream,accept_message
from r3_scan import decode,header_info
from visual_capture import decode_info,camera_status

_REBINDABLE_MAGNETIC_FAULTS = frozenset((
    'capture_stream_outage', 'capture_acquisition_outage', 'capture_transport_delay', 'producer_restarted',
    'producer_fault_since_binding', 'unobserved_failed_attempt',
    'invalid_acquisition:no_data_timeout', 'invalid_acquisition:data_ready_not_cleared',
    'invalid_acquisition:overflow_or_overrun', 'invalid_acquisition:ambiguous_or_overrun_burst',
    'invalid_acquisition:transaction_time_bound_exceeded', 'invalid_acquisition:conversion_time_bound_exceeded',
    'invalid_acquisition:read_exception:OSError',
))

class RosNavigationIO:
    def __init__(self,cfg,now=None,subscribe=None,ros_now_ns=None,get_param=None,boot_id=None):
        self.cfg=cfg;self.clock=now or time.monotonic;self.lock=threading.RLock();self.changed=threading.Event()
        self.latest_camera_info=None;self.latest_image_metadata=None
        self.capture_stream=make_stream(cfg['magnetic'],self.clock,boot_id)
        self.magnetic_load=None;self.magnetic_load_epoch=0;self._magnetic_capture_after=self.clock()
        self.rows=collections.deque(maxlen=128);self.mags=collections.deque(maxlen=64);self.subs=[];self.closed=False
        self.detector_ids=collections.deque(maxlen=256)
        self.fall_image_samples=collections.deque(maxlen=16);self.fall_image_stamp=0
        self.last_image_stamp=0;self.last_image_receipt=0;self.last_stamp=0;self.last_seq=None;self.restart_barrier=0;self.rejected=0;self.publisher_ok=False
        if subscribe is None:
            import rospy
            from ar_track_alvar_msgs.msg import AlvarMarkers
            from sensor_msgs.msg import Image, CameraInfo
            if cfg['magnetic']['enabled']:
                if self.capture_stream is not None:
                    from std_msgs.msg import String as magnetic_message
                else:
                    from aelos_smart_ros.msg import sensor_topic as magnetic_message
            ros_now_ns=lambda:rospy.Time.now().to_nsec();get_param=rospy.get_param
            subscribe=lambda kind,callback:rospy.Subscriber(cfg['visual_micro']['camera_info']['topic'] if kind=='camera_info' else cfg['camera_health']['topic'] if kind=='image' else cfg['topic'] if kind=='ar' else '/aelos/magnetic_capture' if self.capture_stream is not None else '/sensor_info',CameraInfo if kind=='camera_info' else Image if kind=='image' else AlvarMarkers if kind=='ar' else magnetic_message,callback,queue_size=8)
        self.ros_now_ns=ros_now_ns
        try:
            self.parameters_ok=get_param('/chest/ar_track_alvar/output_frame',None)==cfg['pose_contract']['frame'] and float(get_param('/chest/ar_track_alvar/marker_size',-1))==cfg['pose_contract']['marker_size_cm']
        except Exception:self.parameters_ok=False
        # The stock solver and the optional raw-pixel solver share a CameraInfo.
        # Read their private switch once per navigation IO, outside the frame callback.
        try:
            mode=get_param('/chest/ar_track_alvar/use_raw_chest_distortion',False)
            self.use_raw_chest_distortion=mode if type(mode) is bool else None
        except Exception:
            self.use_raw_chest_distortion=None
        try:
            self.subs.append(subscribe('ar',self.on_ar))
            health=cfg.get('camera_health',{})
            if health.get('topic_confirmed') and health.get('topic'):self.subs.append(subscribe('image',self.on_image))
            if cfg.get('visual_micro',{}).get('enabled'):self.subs.append(subscribe('camera_info',self.on_camera_info))
            if cfg['magnetic']['enabled']:self.subs.append(subscribe('mag',self.on_mag))
        except Exception:
            self.close();raise
    @property
    def motion_capture_verified(self):
        with self.lock:
            s=self.capture_stream
            return bool(self.cfg['magnetic'].get('capture_freshness_verified') and s is not None and not s.fault and s.window(0) and s.last_outcome is not None and 0<=self.clock()-s.last_outcome['received_monotonic']<=s.policy.publication_bound_s+s.policy.transport_bound_s)
    def begin_task_observation_scope(self):
        """New task epoch; raw camera buffers may persist, magnetic load may not."""
        with self.lock:
            self.magnetic_load=None
            self.magnetic_load_epoch+=1
            self._magnetic_capture_after=self.clock()
            self.mags.clear()
            self.fall_image_samples.clear();self.fall_image_stamp=0
            if self.capture_stream is not None:self.capture_stream.rows=[]
        self.changed.set()

    def set_magnetic_load(self,load):
        if load not in ('empty','single','double'):raise ValueError('unknown expected load')
        with self.lock:
            if load!=self.magnetic_load:
                self.magnetic_load=load;self.magnetic_load_epoch+=1;self._magnetic_capture_after=self.clock()
                self.fall_image_samples.clear();self.fall_image_stamp=0
                if self.capture_stream is not None:
                    self.mags.clear();self.capture_stream.rows=[]
    def on_camera_info(self,msg):
        try:info=decode_info(msg,self.clock())
        except (TypeError,ValueError,AttributeError):info=None
        with self.lock:self.latest_camera_info=info
        self.changed.set()
    def visual_camera_status(self):
        with self.lock:
            c=self.cfg.get('visual_micro',{}).get('camera_info')
            status=camera_status(self.latest_camera_info,self.latest_image_metadata,c,self.clock()) if c else {'ready':False,'reason':'not configured'}
            return dict(status,use_raw_chest_distortion=self.use_raw_chest_distortion)

    def preflight_health_snapshot(self):
        """Read source and stream health together at a pre-motion boundary."""
        with self.lock:
            now_ros=self.ros_now_ns();now=self.clock()
            healthy=self.parameters_ok and self._camera_healthy(now_ros,now,self.restart_barrier)
            return {'camera_healthy':healthy,'camera_info':self.visual_camera_status(),'checked_monotonic':now}

    def magnetic_capture_health(self):
        with self.lock:
            stream = self.capture_stream
            if stream is None:
                return {'configured': False}
            rows = stream.window(self._magnetic_capture_after)
            last = stream.last_outcome
            report = {
                'configured': True,
                'fault': stream.fault,
                'recoverable_interruption': stream.fault in _REBINDABLE_MAGNETIC_FAULTS,
                'fresh_rows': len(rows),
                'required_rows': self.cfg['magnetic']['min_samples'],
                'last_verified_receipt_age_s': self.clock() - last['received_monotonic'] if last else None,
            }
            if stream.fault_detail is not None:
                report['fault_detail']=dict(stream.fault_detail)
            return report

    def retry_magnetic_preflight(self):
        """Rebind a recoverable acquisition interruption before the first action."""
        return self._rebind_magnetic_capture(same_session=False) is MagneticRebindResult.REBOUND

    def retry_magnetic_after_action(self) -> MagneticRebindResult:
        """Rebind at a completed boundary without changing the load or producer identity."""
        return self._rebind_magnetic_capture(same_session=True)

    def _rebind_magnetic_capture(self, *, same_session):
        with self.lock:
            stream = self.capture_stream
            if self.closed or stream is None or stream.fault not in _REBINDABLE_MAGNETIC_FAULTS:
                return MagneticRebindResult.UNAVAILABLE
            if same_session:
                if stream.fault == 'producer_restarted' or stream.session is None:
                    return MagneticRebindResult.UNAVAILABLE
                self.capture_stream = stream.rebind_same_session()
            else:
                self.capture_stream = make_stream(self.cfg['magnetic'], self.clock, stream.boot_id)
                self.magnetic_load_epoch += 1
            self._magnetic_capture_after = self.clock()
            self.fall_image_samples.clear()
            self.fall_image_stamp = 0
            return MagneticRebindResult.REBOUND

    def _camera_healthy(self,now_ns,now,barrier):
        health=self.cfg['camera_health']
        return bool(health['topic_confirmed'] and self.publisher_ok
            and self.last_image_stamp>barrier and self.last_stamp>barrier
            and 0<=now_ns-self.last_image_stamp<=health['max_age_s']*1e9
            and 0<=now_ns-self.last_stamp<=self.cfg['max_age_s']*1e9
            and 0<=now-self.last_image_receipt<=health['max_age_s'])

    def _camera_window_status(self, now_ns, now, barrier):
        return {
            'parameters_match': self.parameters_ok,
            'ar_source_matches': self.publisher_ok if self.last_stamp else None,
            'restart_barrier_ns': self.restart_barrier,
            'image_after_barrier': self.last_image_stamp > barrier,
            'ar_after_barrier': self.last_stamp > barrier,
            'image_stamp_age_s': (now_ns-self.last_image_stamp)/1e9 if self.last_image_stamp else None,
            'ar_stamp_age_s': (now_ns-self.last_stamp)/1e9 if self.last_stamp else None,
            'image_receipt_age_s': now-self.last_image_receipt if self.last_image_stamp else None,
        }

    def visual_motion_ready(self,stamp_ns):
        with self.lock:
            now_ns = self.ros_now_ns()
            return bool(not self.closed and self.parameters_ok and type(stamp_ns) is int
                and self.restart_barrier < stamp_ns <= self.last_stamp
                and 0 <= now_ns - stamp_ns <= self.cfg['max_age_s'] * 1e9
                and self._camera_healthy(now_ns, self.clock(), self.restart_barrier)
                and self.visual_camera_status().get('ready'))

    def on_image(self,msg):
        h=header_info(msg) or {};stamp=h.get('stamp_ns',0);now=self.ros_now_ns()
        if type(stamp) is not int or not 0<=now-stamp<=self.cfg['camera_health']['max_age_s']*1e9:return
        if not getattr(msg,'data',None) or getattr(msg,'width',0)<=0 or getattr(msg,'height',0)<=0:return
        receipt=self.clock()
        caller=(getattr(msg,'_connection_header',None) or {}).get('callerid','')
        camera=self.cfg.get('visual_micro',{}).get('camera_info') or {}
        with self.lock:
            if self.closed or stamp<=self.last_image_stamp:return
            self.last_image_stamp=stamp;self.last_image_receipt=receipt
            self.latest_image_metadata=dict(stamp_ns=stamp,frame_id=h.get('frame_id',''),width=msg.width,height=msg.height,received_monotonic=receipt,publisher=caller)
            epoch=self.magnetic_load_epoch
            collect=(self.magnetic_load is not None and self.capture_stream is not None
                     and not self.capture_stream.fault and self.cfg['magnetic'].get('capture_freshness_verified')
                     and camera.get('source_confirmed') is True and caller==camera.get('publisher')
                     and str(h.get('frame_id','')).lstrip('/')==str(camera.get('frame_id') or '').lstrip('/')
                     and stamp-self.fall_image_stamp>=300_000_000)
            if collect:self.fall_image_stamp=stamp
        self.changed.set()
        if collect:
            luma=FallDiag_ImageLuma(msg)
            if luma is not None:
                with self.lock:
                    if not self.closed and self.magnetic_load_epoch==epoch and self.fall_image_stamp==stamp:
                        self.fall_image_samples.append(dict(stamp_ns=stamp, received_monotonic=receipt, luma=luma,
                                                            size=(msg.width,msg.height)))

    def fall_diagnostic(self, action):
        """Nonblocking diagnostic snapshot from the existing scoped sensor caches."""
        with self.lock:
            now = self.clock()
            stream = self.capture_stream
            mag = []
            if (not self.closed and self.cfg['magnetic'].get('capture_freshness_verified')
                    and stream is not None and not stream.fault and stream.last_outcome is not None
                    and 0 <= now-stream.last_outcome['received_monotonic']
                    <= stream.policy.publication_bound_s + stream.policy.transport_bound_s):
                mag = list(stream.rows)
            images = []
            if self.fall_image_samples:
                camera = self.visual_camera_status()
                if camera.get('ready'):
                    size = (camera.get('width'), camera.get('height'))
                    images = [row for row in self.fall_image_samples if row['size'] == size]
        return FallDiag_Assess(action, mag, images, now)

    def on_ar(self,msg):
        now=self.ros_now_ns();h=header_info(msg) or {};stamp=h.get('stamp_ns',0);seq=h.get('seq')
        caller=(getattr(msg,'_connection_header',None) or {}).get('callerid','')
        with self.lock:
            if stamp<=0 or not 0<=now-stamp<=self.cfg['max_age_s']*1e9:
                self.rejected+=1;return
            if stamp<self.last_stamp or seq is not None and self.last_seq is not None and seq<self.last_seq:
                self.rows.clear();self.detector_ids.clear();self.restart_barrier=now
            self.last_stamp=stamp;self.last_seq=seq
            self.publisher_ok=caller==self.cfg['pose_contract']['publisher']
            for marker in getattr(msg,'markers',[]):
                marker_id=getattr(marker,'id',None)
                if self.publisher_ok and type(marker_id) is int:
                    self.detector_ids.append((stamp,marker_id))
                try:
                    row=decode(msg,marker,now,self.restart_barrier,self.cfg['max_age_s'])
                    row.update(source_topic=self.cfg['topic'],source_publisher=caller)
                    self.rows.append(row)
                except (ValueError,TypeError,AttributeError):self.rejected+=1
        self.changed.set()
    def on_mag(self,msg):
        if self.capture_stream is not None:
            with self.lock:accept_message(self.capture_stream,msg)
            self.changed.set()
            return
        try:
            row={'heading_deg':float(msg.mag),'xyz':[float(msg.mag_x),float(msg.mag_y),float(msg.mag_z)],'received_monotonic':self.clock()}
        except (TypeError,ValueError,AttributeError):return
        with self.lock:self.mags.append(row)
        self.changed.set()
    def boundary_visual_snapshot(self,after_ns,check):
        """Zero-wait post-completion AR window; prior sightings are diagnostic only."""
        check()
        with self.lock:
            now_ns=self.ros_now_ns();now=self.clock()
            valid=type(after_ns) is int and 0<after_ns<=now_ns
            barrier=max(after_ns,self.restart_barrier) if valid else now_ns
            rows=[dict(r) for r in self.rows if r['stamp_ns']>barrier and
                  0<=now_ns-r['stamp_ns']<=self.cfg['max_age_s']*1e9] if valid else []
            recent=[r for r in self.rows if 0<=now_ns-r['stamp_ns']<=self.cfg['max_age_s']*1e9]
            healthy=bool(valid and self._camera_healthy(now_ns,now,barrier))
            result=dict(observations=rows,now_ns=now_ns,after_ns=barrier,
                camera_restart_ns=self.restart_barrier,
                contract_match=self.parameters_ok and self.publisher_ok,rejected=self.rejected,
                camera_info=self.visual_camera_status(),camera_healthy=healthy,
                boundary_valid=valid,read_only=True,waited_s=0.,
                recent_decoded_ids=sorted({r['id'] for r in recent}),
                recent_ids_are_diagnostic_only=True,buffered_rows=len(self.rows),
                current_rows=len(rows),stale_rows=sum(now_ns-r['stamp_ns']>self.cfg['max_age_s']*1e9 for r in self.rows),
                pre_boundary_rows=sum(r['stamp_ns']<=barrier for r in self.rows))
        check()
        return result

    def observe_checkpoint_id(self, tag_id, after_ns, timeout, check):
        return self.observe(tag_id, after_ns, timeout, check, quick_id=True)

    def observe(self,tag_id,after_ns,timeout,check,*,quick_id=False):
        start_ros=self.ros_now_ns()
        reuse=self.cfg.get('reuse_post_completion_observations',False) is True
        barrier=after_ns if reuse and type(after_ns) is int and 0<after_ns<=start_ros else max(start_ros,after_ns)
        deadline=self.clock()+timeout
        contract=self.cfg['pose_contract']
        required=self.cfg['min_frames'] if quick_id else self.cfg['window_frames']
        require_healthy_streams=quick_id or self.cfg['camera_health']['topic_confirmed']

        def eligible(row,floor,now_ns):
            return row['stamp_ns']>floor and 0<=now_ns-row['stamp_ns']<=self.cfg['max_age_s']*1e9

        def valid_for_id(row):
            return (row.get('source_publisher') == contract['publisher']
                    and row.get('source_topic') == self.cfg['topic']
                    and row['frame_id'].lstrip('/') == contract['frame'].lstrip('/')
                    and not row.get('headers_disagree'))

        # 每次动作完成后重新开窗口，旧帧/其他相机TF不参与。
        while self.clock()<deadline:
            check()
            with self.lock:
                now_ns=self.ros_now_ns();floor=max(barrier,self.restart_barrier)
                unique={(r['stamp_ns'], None if quick_id else r['frame_id']) for r in self.rows
                        if r['id']==tag_id and eligible(r,floor,now_ns) and (not quick_id or valid_for_id(r))}
                if len(unique)>=required and (not require_healthy_streams or
                        (self.parameters_ok and self._camera_healthy(now_ns,self.clock(),floor))):
                    break
            self.changed.wait(min(.05,max(0,deadline-self.clock())));self.changed.clear()
        check()
        with self.lock:
            now_ns=self.ros_now_ns();floor=max(barrier,self.restart_barrier);now=self.clock()
            window = {'observations':[dict(r) for r in self.rows if eligible(r,floor,now_ns)],
                    'now_ns':now_ns,'after_ns':floor,'contract_match':self.parameters_ok and self.publisher_ok,
                    'camera_restart_ns':self.restart_barrier,
                    'rejected':self.rejected,'camera_info':self.visual_camera_status(),
                    'camera_healthy':self._camera_healthy(now_ns,now,floor),
                    'camera_window_status':self._camera_window_status(now_ns,now,floor)}
            if quick_id:
                detector_stamps={stamp for stamp,marker_id in self.detector_ids
                                 if marker_id==tag_id and floor<stamp<=now_ns}
                window['detector_expected_frames']=sum(now_ns-stamp<=self.cfg['max_age_s']*1e9
                                                       for stamp in detector_stamps)
                window['detector_observed_expected_frames']=len(detector_stamps)
            return window
    def settle(self,seconds,check):
        end=self.clock()+seconds
        while self.clock()<end:check();self.changed.wait(min(.05,end-self.clock()));self.changed.clear()
        check()
    def magnetic_samples(self,after,timeout,check):
        if self.capture_stream is not None:
            end=self.clock()+timeout
            while self.clock()<end:
                check()
                with self.lock:
                    if self.capture_stream.fault:return []
                    rows=self.capture_stream.window(max(after,self._magnetic_capture_after))
                    rows=[dict(r,expected_load=self.magnetic_load,load_epoch=self.magnetic_load_epoch) for r in rows if type(r.get('captured_monotonic')) in (int,float) and 0<=self.clock()-r['captured_monotonic']<=self.cfg['magnetic']['max_age_s']]
                if len(rows)>=self.cfg['magnetic']['min_samples']:return rows
                self.changed.wait(min(.05,max(0,end-self.clock())));self.changed.clear()
            check()
            with self.lock:
                return [dict(r,expected_load=self.magnetic_load,load_epoch=self.magnetic_load_epoch) for r in self.capture_stream.window(max(after,self._magnetic_capture_after)) if type(r.get('captured_monotonic')) in (int,float) and 0<=self.clock()-r['captured_monotonic']<=self.cfg['magnetic']['max_age_s']]
        end=self.clock()+timeout
        while self.clock()<end:
            check()
            with self.lock:rows=[dict(r) for r in self.mags if r['received_monotonic']>after]
            if len({r['received_monotonic'] for r in rows})>=self.cfg['magnetic']['min_samples']:return rows
            self.changed.wait(min(.05,max(0,end-self.clock())));self.changed.clear()
        with self.lock:return [dict(r) for r in self.mags if r['received_monotonic']>after]
    def close(self):
        if self.closed:return
        self.closed=True
        for sub in self.subs:
            try:sub.unregister()
            except Exception:pass
        self.subs=[]
        with self.lock:self.fall_image_samples.clear()
