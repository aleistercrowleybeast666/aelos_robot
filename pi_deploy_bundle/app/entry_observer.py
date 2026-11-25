"""有限视觉窗口：只订阅AR/Image/CameraInfo/TF，不启动服务，不访问UART。"""
import collections,threading,time
from entry_camera import CameraUnavailable,pixel_observation,transformed_pose,camera_fingerprint,frame
from r3_scan import Collector,header_info

def image_meta(msg):
    h=header_info(msg) or {};return {'frame_id':h.get('frame_id',''),'stamp_ns':h.get('stamp_ns',0),'source_seq':h.get('seq'),'width':int(msg.width),'height':int(msg.height),'encoding':str(getattr(msg,'encoding',''))}
def info_meta(msg):
    h=header_info(msg) or {};roi=getattr(msg,'roi',None)
    return {'frame_id':h.get('frame_id',''),'stamp_ns':h.get('stamp_ns',0),'width':int(msg.width),'height':int(msg.height),
        'K':list(msg.K),'D':list(msg.D),'R':list(msg.R),'P':list(msg.P),'distortion_model':str(msg.distortion_model),
        'binning_x':int(getattr(msg,'binning_x',0)),'binning_y':int(getattr(msg,'binning_y',0)),
        'roi':{k:int(getattr(roi,k,0)) for k in ('x_offset','y_offset','width','height')}}

class Assembler:
    def __init__(self,profile,options,barrier,lookup,save_images=False):
        self.profile=profile;self.options=dict(options,min_fresh_frames=profile['min_frames']);self.options['expected_ids']=[] if profile['target_id'] is None else [profile['target_id']]
        self.collector=Collector(self.options,barrier);self.lookup=lookup;self.images=collections.deque(maxlen=16);self.infos=collections.deque(maxlen=8)
        self.raw_images={};self.save_images=save_images;self.poses={};self.pixels={};self.reasons=set();self.target_lost=False;self.had_target=False;self.restarted=False
    def on_ar(self,msg,now,strict=False):
        ids={m.id for m in getattr(msg,'markers',[])};target=self.profile['target_id']
        if target is not None and target not in ids and (strict or self.had_target):self.target_lost=True
        if target in ids:self.had_target=True
        previous=dict(self.collector.last_seq)
        self.collector.observe(msg,now)
        if any(k in previous and v is not None and previous[k] is not None and v<previous[k] for k,v in self.collector.last_seq.items()):
            self.restarted=True;self.poses.clear();self.pixels.clear()
    def on_image(self,msg):
        row=image_meta(msg);self.images.append(row)
        if self.save_images:
            self.raw_images[row['stamp_ns']]=msg
            while len(self.raw_images)>16:self.raw_images.pop(next(iter(self.raw_images)))
    def on_info(self,msg):self.infos.append(info_meta(msg))
    def process(self,now):
        cfg=self.profile;cam=cfg['camera'];camera_reasons=set();pose_reasons=set()
        for rows in self.collector.history.values():
            for row in rows:
                if now-row['stamp_ns']>cfg['max_age_s']*1e9:continue
                key=(row['id'],row['stamp_ns'],row['frame_id'])
                if row.get('headers_disagree'):
                    self.reasons.add('AR headers disagree; cannot select pose frame');continue
                target=cfg['planar']['frame']
                if target and key not in self.poses:
                    try:
                        transform=self.lookup(target,row['frame_id'],row['stamp_ns']);name,p,q=transformed_pose(row,transform)
                        self.poses[key]={'id':row['id'],'stamp_ns':row['stamp_ns'],'source_seq':row.get('source_seq'),'frame_id':target,
                            'position':p,'quaternion':q,'headers_conflict':False,
                            'geometry_verified':all(cfg['planar'][k] is True for k in ('axes_verified','capture_time_verified','tag_scale_verified')),
                            'raw_observation':row}
                    except Exception as exc:pose_reasons.add(str(exc))
                if key not in self.pixels:
                    try:
                        if not self.images or not self.infos:raise CameraUnavailable('Image/CameraInfo not received')
                        image=min(self.images,key=lambda r:abs(r['stamp_ns']-row['stamp_ns']))
                        candidates=[r for r in self.infos if frame(r['frame_id'])==frame(image['frame_id'])]
                        if not candidates:raise CameraUnavailable('no CameraInfo for image frame')
                        info=min(candidates,key=lambda r:abs(r['stamp_ns']-image['stamp_ns']))
                        transform=self.lookup(image['frame_id'],row['frame_id'],row['stamp_ns'])
                        self.pixels[key]=pixel_observation(row,image,info,transform,cam)
                    except Exception as exc:camera_reasons.add(str(exc))
        self.reasons.update(pose_reasons);self.reasons.update(camera_reasons)
    def snapshot(self,now):
        raw=self.collector.result or {'status':'timeout','type':'AR_TAG_ALVAR','observations':[],'ids':[],
            'reason':self.collector.reason,'observed_ids':sorted(self.collector.observed_ids),'messages':self.collector.messages,'rejected':self.collector.rejected}
        raw=dict(raw,capture_history=[r for rows in self.collector.history.values() for r in rows],window_after_ns=self.collector.after_ns,window_now_ns=now)
        fresh=lambda r:0<=now-r['stamp_ns']<=self.profile['max_age_s']*1e9
        poses=sorted([r for r in self.poses.values() if fresh(r)],key=lambda r:r['stamp_ns'])
        pixels=sorted([r for r in self.pixels.values() if fresh(r)],key=lambda r:r['stamp_ns'])
        return {'raw_scan':raw,'after_ns':self.collector.after_ns,'now_ns':now,'pose_samples':poses,'pixel_samples':pixels,
            'camera_fingerprint':pixels[-1]['camera_fingerprint'] if pixels else camera_fingerprint(self.infos[-1],self.profile['camera']['image_kind']) if self.infos else None,
            'camera_info':self.infos[-1] if self.infos else None,'image_header':self.images[-1] if self.images else None,
            'target_lost':self.target_lost,'publisher_restarted':self.restarted,'diagnostic_reasons':sorted(self.reasons),'image_y_center_required':False}

def save_debug_image(msg,pixel,path):
    """只有用户明确传入保存路径才落图；依赖缺失返回诊断，不伪造像素。"""
    from cv_bridge import CvBridge
    import cv2
    image=CvBridge().imgmsg_to_cv2(msg,desired_encoding='bgr8');w=int(msg.width);h=int(msg.height)
    cv2.line(image,(int((w-1)/2),0),(int((w-1)/2),h-1),(0,255,0),1)
    if pixel:
        u,v=pixel['tag_center_px'];cv2.circle(image,(round(u),round(v)),7,(0,0,255),2)
    cv2.putText(image,'projection: CHECK MANUALLY',(8,20),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,0,255),1)
    if not cv2.imwrite(str(path),image):raise CameraUnavailable('failed to save requested debug image')

def observe_entry_window(profile,options,check=lambda:None,after_ns=None,timeout_s=5.,strict_target=False,save_image=None,
        subscribe=None,now_ns=None,lookup=None,tf_frames=None):
    """默认用真实ROS；依赖仅在显式扫码/命令0段末按需导入。TF不回退最新时刻。"""
    check();diagnostic=[];tf_state={};initialize_tf=None
    if subscribe is None:
        import rospy
        from ar_track_alvar_msgs.msg import AlvarMarkers
        from sensor_msgs.msg import Image,CameraInfo
        now_ns=lambda:rospy.Time.now().to_nsec()
        try:
            import tf2_ros
            def initialize_tf():
                try:
                    buffer=tf2_ros.Buffer(debug=False);listener=tf2_ros.TransformListener(buffer)
                    tf_state.update(buffer=buffer,listener=listener)
                    if stop.is_set() and hasattr(listener,'unregister'):listener.unregister()
                except Exception as exc:tf_state['error']=str(exc)
            def lookup(target,source,stamp):
                if 'buffer' not in tf_state:raise CameraUnavailable('TF listener not ready: '+tf_state.get('error','pending'))
                value=tf_state['buffer'].lookup_transform(frame(target),frame(source),rospy.Time(stamp//10**9,stamp%10**9),rospy.Duration(0))
                t=value.transform.translation;q=value.transform.rotation
                return {'target_frame':value.header.frame_id,'source_frame':value.child_frame_id,'requested_stamp_ns':stamp,
                    'translation':[t.x,t.y,t.z],'rotation':[q.x,q.y,q.z,q.w]}
            tf_frames=lambda:tf_state['buffer'].all_frames_as_yaml()[:8192] if 'buffer' in tf_state else 'TF listener pending'
        except ImportError:
            def lookup(*a):raise CameraUnavailable('tf2_ros unavailable; no coordinates for control')
        kinds={'ar':(options['topic'],AlvarMarkers),'image':(profile['camera']['image_topic'],Image),'info':(profile['camera']['info_topic'],CameraInfo)}
        subscribe=lambda kind,cb:rospy.Subscriber(kinds[kind][0],kinds[kind][1],cb,queue_size=3)
    if now_ns is None or lookup is None:raise CameraUnavailable('explicit ROS time and transform provider required')
    start_stamp=now_ns();barrier=start_stamp if after_ns is None else max(start_stamp,after_ns)
    if type(barrier) is not int or barrier<=0:raise CameraUnavailable('ROS time not ready')
    assembler=Assembler(profile,options,barrier,lookup,save_image is not None);lock=threading.RLock();changed=threading.Event();stop=threading.Event();subscriptions=[]
    def callback(kind,msg):
        if stop.is_set():return
        with lock:
            try:
                if kind=='ar':assembler.on_ar(msg,now_ns(),strict_target)
                elif kind=='image':assembler.on_image(msg)
                else:assembler.on_info(msg)
            except Exception as exc:assembler.reasons.add(str(exc))
        changed.set()
    def setup():
        for kind in ('ar','image','info'):
            try:
                sub=subscribe(kind,lambda msg,k=kind:callback(k,msg))
                with lock:subscriptions.append(sub)
                if stop.is_set():sub.unregister()
            except Exception as exc:
                with lock:assembler.reasons.add(kind+' subscribe: '+str(exc))
                changed.set()
    if initialize_tf is not None:threading.Thread(target=initialize_tf,daemon=True,name='r3-entry-tf').start()
    threading.Thread(target=setup,daemon=True,name='r3-entry-vision').start();deadline=time.monotonic()+timeout_s
    try:
        while time.monotonic()<deadline:
            check()
            with lock:
                now=now_ns()
                if now<barrier:raise CameraUnavailable('ROS clock moved backwards')
                if assembler.restarted:raise CameraUnavailable('AR publisher restarted; discard old window')
                assembler.process(now);result=assembler.snapshot(now)
                if result['target_lost']:raise CameraUnavailable('target lost; no search/retry motion')
                enough_pose=len(result['pose_samples'])>=profile['min_frames'];enough_pixels=len(result['pixel_samples'])>=profile['min_frames']
                if enough_pixels or enough_pose and profile['mode']=='execute_calibrated' and save_image is None:break
            changed.wait(min(.03,max(0,deadline-time.monotonic())));changed.clear()
        with lock:result=assembler.snapshot(now_ns())
        if tf_frames is not None:
            try:result['available_tf_frames']=tf_frames()
            except Exception as exc:result['tf_diagnostic']=str(exc)
        if save_image is not None:
            try:
                pixel=result['pixel_samples'][-1] if result['pixel_samples'] else None
                stamp=pixel['image_stamp_ns'] if pixel else max(assembler.raw_images)
                save_debug_image(assembler.raw_images[stamp],pixel,save_image);result['requested_debug_image']=str(save_image)
            except Exception as exc:result['debug_image_error']=str(exc)
        return result
    finally:
        stop.set()
        listener=tf_state.get('listener')
        if listener is not None and hasattr(listener,'unregister'):threading.Thread(target=listener.unregister,daemon=True).start()
        for sub in subscriptions:
            threading.Thread(target=sub.unregister,daemon=True,name='r3-entry-unsubscribe').start()
