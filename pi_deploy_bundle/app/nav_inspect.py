#!/usr/bin/env python3
"""供后续现场运行的只读合同检查；本轮不在机器人执行，不启动任何服务。"""
import hashlib,json,time
from pathlib import Path

def candidate_geometry(msg,now_ns,cfg):
    """供人核对轴方向的候选读数；不授予控制可信标志，不动作。"""
    from r3_scan import decode
    from nav_observation import ground_pose
    result=[]
    for marker in msg.markers:
        try:
            row=decode(msg,marker,now_ns,0,cfg['max_age_s'])
            if row['headers_disagree']:raise ValueError('AR header frame/time conflict')
            result.append(dict(ground_pose(row,cfg['pose_contract']),verified=False,diagnostic_only=True))
        except (ValueError,TypeError,KeyError) as exc:result.append({'id':getattr(marker,'id',None),'diagnostic_error':str(exc),'verified':False})
    return result

def main():
    import rospy,rosgraph,rospkg
    from ar_track_alvar_msgs.msg import AlvarMarkers
    from tf2_msgs.msg import TFMessage
    rospy.init_node('r3_navigation_readonly_inspect',anonymous=True,disable_signals=True)
    from checkpoint_nav import load
    cfg=load(Path(__file__).resolve().parent)
    result={'body_commands':0,'services_started':False,'source_binary_correspondence_verified':False,'params':{},'ar_samples':[],'marker_tf_sources':{}}
    for key in ('/chest/ar_track_alvar/output_frame','/chest/ar_track_alvar/marker_size','/head/ar_track_alvar/output_frame','/head/ar_track_alvar/marker_size'):
        result['params'][key]=rospy.get_param(key,None)
    try:
        path=Path(rospkg.RosPack().get_path('ar_track_alvar'));result['package_path']=str(path)
        import xml.etree.ElementTree as ET
        result['package_version']=ET.parse(path/'package.xml').getroot().findtext('version')
        source=path/'nodes/IndividualMarkersNoKinect.cpp'
        if source.is_file() and source.stat().st_size<500000:
            data=source.read_bytes();text=data.decode('utf-8',errors='replace')
            result['source_sha256']=hashlib.sha256(data).hexdigest()
            result['source_family_checks']={k:k in text for k in ('translation[0] / 100.0','ar_pose_marker.header.frame_id = output_frame','ar_marker_')}
    except Exception as exc:result['package_inspection_error']=str(exc)
    def ar(msg):
        if len(result['ar_samples'])>=5:return
        result['ar_samples'].append({'caller':getattr(msg,'_connection_header',{}).get('callerid'),'frame':msg.header.frame_id,
            'stamp_ns':msg.header.stamp.to_nsec(),'ids':[m.id for m in msg.markers],
            'marker_headers':[{'id':m.id,'frame':m.header.frame_id,'pose_frame':m.pose.header.frame_id,'stamp_ns':m.header.stamp.to_nsec()} for m in msg.markers],
            'candidate_geometry':candidate_geometry(msg,rospy.Time.now().to_nsec(),cfg)})
    def tf(msg):
        caller=getattr(msg,'_connection_header',{}).get('callerid','unknown')
        for t in msg.transforms:
            if t.child_frame_id.lstrip('/').startswith('ar_marker_'):
                key=t.child_frame_id.lstrip('/');pair=[caller,t.header.frame_id]
                if pair not in result['marker_tf_sources'].setdefault(key,[]):result['marker_tf_sources'][key].append(pair)
    subs=[]
    try:
        subs=[rospy.Subscriber('/chest/ar_pose_marker',AlvarMarkers,ar,queue_size=5),rospy.Subscriber('/tf',TFMessage,tf,queue_size=10),rospy.Subscriber('/tf_static',TFMessage,tf,queue_size=10)]
        end=time.monotonic()+2
        while time.monotonic()<end and not rospy.is_shutdown():time.sleep(.05)
        pub,_,_=rosgraph.Master(rospy.get_name()).getSystemState()
        result['publishers']={topic:nodes for topic,nodes in pub if topic in ('/chest/ar_pose_marker','/head/ar_pose_marker','/sensor_info')}
        result['duplicate_marker_tf_children']={k:v for k,v in result['marker_tf_sources'].items() if len(v)>1}
    finally:
        for s in subs:s.unregister()
    print(json.dumps(result,ensure_ascii=False));return 0

if __name__=='__main__':raise SystemExit(main())
