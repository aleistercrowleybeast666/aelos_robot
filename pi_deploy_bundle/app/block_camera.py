"""Read-only, scoped ROS BGR cache. No action execution or hardware startup.
Capture timestamps AND receipt timestamps must be after completion + settle.
Callbacks never decide motion. Closing invalidates data before unregistering.
"""
import collections
import math
import threading
import time
from block_vision import BlockDetector, ConfirmedWindow, validate_config, unknown


class BlockFrameUnavailable(RuntimeError):
    """The next capture window can recover this dropout."""


class BlockCameraContractLost(BlockFrameUnavailable):
    """Reopen the chest subscription and verify its source before using it again."""


class BlockEvidenceExpired(RuntimeError):
    """The capture expired before the grab request was submitted."""


class BlockCamera:
    def __init__(self,cfg,scope,clock=None,subscribe=None,convert=None,ros_now_ns=None,*,
                 staged_publisher=None,trial_alignment=False):
        self.staged_publisher=staged_publisher
        self.trial_alignment=trial_alignment
        staged_trial=staged_publisher is not None
        self.cfg=validate_config(cfg,live=True,staged_trial=staged_trial);self.scope=scope
        self.clock=clock or time.monotonic;self.lock=threading.RLock()
        self.changed=threading.Event();self.frames=collections.deque(maxlen=16)
        self.closed=False;self.fault=None;self.last_stamp=0;self.load='empty'
        self.generation=0;self.sub=None
        self.detector=BlockDetector(cfg,staged_trial=staged_trial,trial_alignment=trial_alignment)
        if subscribe is None:
            import rospy
            from sensor_msgs.msg import Image
            from cv_bridge import CvBridge
            if not rospy.core.is_initialized(): raise RuntimeError('ROS must be initialized by owner')
            bridge=CvBridge();convert=lambda msg:bridge.imgmsg_to_cv2(msg,'bgr8')
            ros_now_ns=lambda:rospy.Time.now().to_nsec()
            subscribe=lambda callback:rospy.Subscriber(cfg['topic'],Image,callback,queue_size=2)
        if convert is None or ros_now_ns is None:raise ValueError('camera clock/converter required')
        self.convert=convert;self.ros_now_ns=ros_now_ns
        try:self.sub=subscribe(self.on_image)
        except BaseException:self.close();raise

    def set_scope(self,scope,load):
        if load not in ('empty','single','double'):raise ValueError('invalid expected load')
        with self.lock:
            if scope!=self.scope or load!=self.load:
                self.scope=scope;self.load=load;self.generation+=1;self.frames.clear()
        self.changed.set()

    def on_image(self,msg):
        with self.lock:
            if self.closed:return
            generation=self.generation;scope=self.scope;load=self.load
        try:
            stamp=msg.header.stamp.to_nsec();received=self.clock();now=self.ros_now_ns()
            if type(stamp) is not int or not 0<stamp<=now or now-stamp>self.cfg['max_age_s']*1e9:return
            if msg.header.frame_id.lstrip('/')!=self.cfg['frame_id'].lstrip('/'):raise ValueError('unexpected camera frame')
            if [msg.width,msg.height]!=self.cfg['resolution']:raise ValueError('camera resolution changed')
            if self.staged_publisher is not None:
                publisher=(getattr(msg,'_connection_header',None) or {}).get('callerid')
                if publisher!=self.staged_publisher:raise ValueError('unexpected staged block camera publisher')
            row={'message':msg,'stamp_ns':stamp,'received_monotonic':received,
                 'scope':scope,'load':load,'width':msg.width,'height':msg.height}
            with self.lock:
                if self.closed or generation!=self.generation or stamp<=self.last_stamp:return
                self.last_stamp=stamp;self.frames.append(row)
        except Exception as exc:
            with self.lock:
                if not self.closed and generation==self.generation:self.fault=str(exc)
        self.changed.set()

    def observe(self,after_monotonic,after_ros_ns,check,timeout_s=None,expected_load='empty',scope=None,*,count_only=False):
        if self.staged_publisher is not None and not (count_only or self.trial_alignment):
            raise ValueError('trial HSV is only scoped to the operator-staged block count')
        if scope is None:scope=self.scope
        if expected_load!='empty':raise RuntimeError('block acquisition requires empty expected load')
        if after_ros_ns is None or type(after_ros_ns) is not int or after_ros_ns<0:raise ValueError('capture-time completion boundary required')
        if type(after_monotonic) not in (int,float) or not math.isfinite(after_monotonic) or after_monotonic<0:raise ValueError('monotonic completion boundary required')
        timeout=self.cfg['timeout_s'] if timeout_s is None else timeout_s
        if type(timeout) not in (int,float) or not 0<timeout<=20:raise ValueError('bounded timeout required')
        settle=self.cfg['settle_s'];mono_boundary=after_monotonic+settle
        ros_boundary=after_ros_ns+int(settle*1e9)
        deadline=self.clock()+timeout;window=ConfirmedWindow(self.cfg,count_only=count_only)
        seen=0;last_eligible=None;invalid_frames=0
        reason='no fresh post-settle images'
        options={'count_only':True} if count_only else {}
        while self.clock()<deadline:
            check()
            with self.lock:
                if self.closed:raise RuntimeError('block camera closed')
                if self.fault:raise BlockCameraContractLost('方块相机订阅失配，将重新核对图像来源：'+self.fault)
                if self.scope!=scope or self.load!=expected_load:raise RuntimeError('block camera scope/load changed')
                frames=list(self.frames)
            for row in frames:
                if row['stamp_ns']<=seen:continue
                seen=row['stamp_ns'];now=self.clock();ros_now=self.ros_now_ns()
                if row['scope']!=scope or row['load']!=expected_load:continue
                if row['stamp_ns']<=ros_boundary or row['received_monotonic']<=mono_boundary:continue
                if not 0<=now-row['received_monotonic']<=self.cfg['max_age_s'] or not 0<=ros_now-row['stamp_ns']<=self.cfg['max_age_s']*1e9:continue
                try:
                    # ROS callbacks keep a small buffer. Decode only eligible frames on the route thread.
                    image=self.convert(row['message'])
                except Exception:
                    detected=None
                else:
                    detected=self.detector.detect(image,**options)
                if detected is None or detected.get('frame_valid') is False:
                    invalid_frames+=1
                    window=ConfirmedWindow(self.cfg,count_only=count_only)
                    last_eligible=None
                    continue
                if not 0<=self.clock()-row['received_monotonic']<=self.cfg['max_age_s'] or not 0<=self.ros_now_ns()-row['stamp_ns']<=self.cfg['max_age_s']*1e9:continue
                last_eligible=row;result=window.add(detected,row)
                reason=result['reason']
                if result['status'] in ('target','empty'):
                    check();result['camera_healthy']=True
                    if invalid_frames:result['discarded_invalid_images']=invalid_frames
                    return result
            self.changed.wait(min(.03,max(0,deadline-self.clock())));self.changed.clear()
        check()
        if last_eligible is None:
            if invalid_frames:
                raise BlockFrameUnavailable(f'方块相机有 {invalid_frames} 帧无法解码，等待新图像。')
            raise BlockFrameUnavailable('block camera has no fresh post-settle stream')
        if not 0<=self.clock()-last_eligible['received_monotonic']<=self.cfg['max_age_s'] or not 0<=self.ros_now_ns()-last_eligible['stamp_ns']<=self.cfg['max_age_s']*1e9:
            raise BlockFrameUnavailable('block camera stopped delivering fresh images during observation')
        result=unknown(reason);result['camera_healthy']=True
        if invalid_frames:result['discarded_invalid_images']=invalid_frames
        if count_only:
            result['staged_count_unresolved']=True
        result.update({k:last_eligible[k] for k in ('stamp_ns','received_monotonic','scope','load','width','height')})
        return result

    def verify(self,row,scope,load):
        """Immediately before native submission; does not itself authorize grasp."""
        with self.lock:
            now=self.clock();ros_now=self.ros_now_ns()
            if self.fault:
                raise BlockEvidenceExpired('方块相机订阅发生变化，待重新采集后再提交动作。')
            if self.closed or self.scope!=scope or self.load!=load:
                raise RuntimeError('block observation scope/camera invalidated')
            staged_pair = (row.get('status')=='unknown' and row.get('staged_count_unresolved') is True
                           and row.get('staged_default_pair') is True and row.get('grip_count')==2 and load=='empty')
            if (row.get('scope')!=scope or row.get('load')!=load or row.get('camera_healthy') is not True
                    or (row.get('status')!='target' and not staged_pair)):
                raise RuntimeError('not a confirmed target for this load/session')
            if not 0<=now-row.get('received_monotonic',-1)<=self.cfg['max_age_s'] or not 0<=ros_now-row.get('stamp_ns',-1)<=self.cfg['max_age_s']*1e9:
                raise BlockEvidenceExpired('block observation expired before action')
        return True

    def close(self):
        with self.lock:self.closed=True;self.generation+=1;self.frames.clear()
        self.changed.set()
        if self.sub is not None:self.sub.unregister();self.sub=None
