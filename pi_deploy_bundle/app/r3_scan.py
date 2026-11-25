"""Bounded terminal AR Tag observation. No ROS/camera imports at module load.

Uses the senior program's /chest/ar_pose_marker AlvarMarkers interface, not QR
text decoding. Raw pose/frame/yaw are reported, never used to move the robot.
"""
import math
import re
import threading
import time

def validate(options):
    expected={'enabled','topic','settle_s','timeout_s','max_age_s','min_fresh_frames','expected_ids'}
    if not isinstance(options,dict) or set(options)!=expected:raise ValueError('terminal_scan schema invalid')
    if type(options['enabled']) is not bool:raise ValueError('scan enabled must be bool')
    if not isinstance(options['topic'],str) or not re.fullmatch(r'/[A-Za-z0-9_/]{1,127}',options['topic']):raise ValueError('scan ROS topic invalid')
    for key,low,high in (('settle_s',0,5),('timeout_s',.1,20),('max_age_s',.05,5)):
        v=options[key]
        if type(v) not in (int,float) or not math.isfinite(v) or not low<=v<=high:raise ValueError('scan '+key+' invalid')
    if type(options['min_fresh_frames']) is not int or not 1<=options['min_fresh_frames']<=5:raise ValueError('scan min_fresh_frames invalid')
    ids=options['expected_ids']
    if not isinstance(ids,list) or len(ids)>32 or any(type(i) is not int or i<0 for i in ids) or len(ids)!=len(set(ids)):raise ValueError('scan expected_ids invalid')

def header_info(obj):
    h=getattr(obj,'header',None)
    if h is None:return None
    stamp=getattr(h,'stamp',None);sec=getattr(stamp,'secs',None);ns=getattr(stamp,'nsecs',None)
    valid=type(sec) is int and sec>=0 and type(ns) is int and 0<=ns<10**9
    seq=getattr(h,'seq',None)
    return {'stamp_ns':sec*10**9+ns if valid else 0,'secs':sec if type(sec) is int else None,
        'nsecs':ns if type(ns) is int else None,'stamp_valid':valid,'frame_id':str(getattr(h,'frame_id','')),
        'seq':seq if type(seq) is int and seq>=0 else None}

def decode(message,marker,now_ns,after_ns,max_age_s):
    if type(getattr(marker,'id',None)) is not int or marker.id<0:raise ValueError('invalid Tag ID')
    headers={name:header_info(obj) for name,obj in (('marker',marker),('pose',marker.pose),('array',message))}
    headers={k:v for k,v in headers.items() if v is not None}
    populated=[(k,v) for k,v in headers.items() if v['stamp_ns']>0]
    if not populated:raise ValueError('no declared capture timestamp')
    origin,h=populated[0];stamp=h['stamp_ns']
    if stamp<=after_ns or not 0<=now_ns-stamp<=max_age_s*1e9:raise ValueError('stale/future/pre-scan Tag timestamp')
    pose=marker.pose.pose;p=pose.position;q=pose.orientation
    xyz=[float(p.x),float(p.y),float(p.z)];quat=[float(q.x),float(q.y),float(q.z),float(q.w)]
    if not all(math.isfinite(v) for v in xyz+quat):raise ValueError('nonfinite Tag pose')
    norm=math.sqrt(sum(v*v for v in quat))
    if norm<1e-8 or abs(norm-1)>0.1:raise ValueError('invalid Tag quaternion')
    x,y,z,w=[v/norm for v in quat]
    yaw=math.degrees(math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))
    frames={v['frame_id'].lstrip('/') for v in headers.values() if v['frame_id']}
    stamps={v['stamp_ns'] for _,v in populated}
    frame=h['frame_id'] or (next(iter(frames)) if len(frames)==1 else '')
    confidence=getattr(marker,'confidence',None)
    if type(confidence) not in (float,int) or not math.isfinite(confidence):confidence=None
    return {'id':marker.id,'position_raw_m':dict(zip(('x','y','z'),xyz)),
        'quaternion_raw':dict(zip(('x','y','z','w'),quat)),'yaw_raw_deg':yaw,
        'frame_id':frame,'stamp_ns':stamp,'stamp_source':origin+'.header','source_seq':h['seq'],
        'headers':headers,'headers_disagree':len(frames)>1 or len(stamps)>1,
        'confidence_raw':confidence,'confidence_semantics':'publisher specific/unknown; not used as a probability',
        'pose_for_control_verified':False,'timestamp_capture_semantics_field_verified':False}

class Collector:
    def __init__(self,options,after_ns):
        self.options=options;self.after_ns=after_ns;self.seen=set();self.counts={};self.last_seq={}
        self.reason='no Tag messages';self.result=None;self.messages=0;self.rejected=0;self.observed_ids=set();self.history={};self.last_now_ns=after_ns
    def observe(self,message,now_ns):
        self.messages+=1;rows=[];self.last_now_ns=now_ns
        for marker in getattr(message,'markers',[]):
            try:row=decode(message,marker,now_ns,self.after_ns,self.options['max_age_s'])
            except (ValueError,AttributeError,TypeError) as exc:self.reason=str(exc);self.rejected+=1;continue
            self.observed_ids.add(row['id'])
            if self.options['expected_ids'] and row['id'] not in self.options['expected_ids']:
                self.reason='different Tag IDs observed';continue
            key=(row['id'],row['stamp_ns'],row['frame_id'])
            if key in self.seen:self.reason='duplicate declared capture';continue
            previous=self.last_seq.get(row['id']);seq=row['source_seq']
            if previous is not None and seq is not None and seq<previous:self.counts[row['id']]=0;self.history[row['id']]=[]
            self.last_seq[row['id']]=seq;self.seen.add(key);self.counts[row['id']]=self.counts.get(row['id'],0)+1
            self.history.setdefault(row['id'],[]).append(row);self.history[row['id']]=self.history[row['id']][-30:]
            if self.counts[row['id']]>=self.options['min_fresh_frames']:rows.append(row)
        if rows:
            self.result={'status':'recognized','type':'AR_TAG_ALVAR','topic':self.options['topic'],
                'ids':sorted({r['id'] for r in rows}),'observations':rows,'distinct_declared_captures':len(self.seen),
                'capture_history':[r for records in self.history.values() for r in records],
                'window_after_ns':self.after_ns,'window_now_ns':self.last_now_ns}
        elif not getattr(message,'markers',[]):self.reason='fresh stream reported no Tags'

def scan_tags(options,check,timeout_s,subscribe=None,ros_now_ns=None):
    validate(options);started=time.monotonic();stop=threading.Event();changed=threading.Event();lock=threading.Lock();holder={}
    if not options['enabled']:return {'status':'disabled','observations':[],'ids':[]}
    try:check()
    except Exception as exc:return {'status':'cancelled','type':'AR_TAG_ALVAR','reason':str(exc),'observations':[],'ids':[]}
    try:
        if subscribe is None:
            import rospy
            from ar_track_alvar_msgs.msg import AlvarMarkers
            subscribe=lambda cb:rospy.Subscriber(options['topic'],AlvarMarkers,cb,queue_size=1)
            ros_now_ns=lambda:rospy.Time.now().to_nsec()
        if ros_now_ns is None:raise RuntimeError('ROS time source unavailable')
        barrier=ros_now_ns()
        if type(barrier) is not int or barrier<=0:raise RuntimeError('ROS time is not initialized')
        collector=Collector(options,barrier)
    except Exception as exc:return {'status':'unavailable','type':'AR_TAG_ALVAR','reason':str(exc),'observations':[],'ids':[]}
    def callback(message):
        if stop.is_set():return
        try:
            with lock:collector.observe(message,ros_now_ns())
        except Exception as exc:
            with lock:collector.reason=str(exc);collector.rejected+=1
        changed.set()
    def setup():
        try:
            sub=subscribe(callback)
            with lock:holder['sub']=sub
            if stop.is_set():sub.unregister()
        except Exception as exc:
            with lock:holder['error']=str(exc)
            changed.set()
    # Constructor/unregister can wait for a missing ROS master. Keep the
    # terminal wait bounded; late callbacks are ignored after stop is set.
    threading.Thread(target=setup,daemon=True,name='r3-terminal-tag-subscribe').start()
    deadline=started+timeout_s
    try:
        while time.monotonic()<deadline:
            try:check()
            except Exception as exc:return {'status':'cancelled','type':'AR_TAG_ALVAR','reason':str(exc),'observations':[],'ids':[]}
            with lock:
                if collector.result:return dict(collector.result,elapsed_s=time.monotonic()-started)
                if 'error' in holder:return {'status':'unavailable','type':'AR_TAG_ALVAR','reason':holder['error'],'observations':[],'ids':[]}
            changed.wait(min(.05,max(0,deadline-time.monotonic())));changed.clear()
        with lock:return {'status':'timeout','type':'AR_TAG_ALVAR','reason':collector.reason,
            'topic':options['topic'],'observations':[],'ids':[],'observed_ids':sorted(collector.observed_ids),
            'messages':collector.messages,'rejected':collector.rejected,'elapsed_s':time.monotonic()-started,
            'capture_history':[r for records in collector.history.values() for r in records],
            'window_after_ns':collector.after_ns,'window_now_ns':collector.last_now_ns}
    finally:
        stop.set()
        with lock:sub=holder.get('sub')
        if sub is not None:threading.Thread(target=sub.unregister,daemon=True,name='r3-terminal-tag-unsubscribe').start()
