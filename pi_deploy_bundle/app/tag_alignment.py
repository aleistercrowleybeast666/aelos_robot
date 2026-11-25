"""独立AR粗对齐工具：默认仅报告，不被路线/预热导入，不直接调用机器人。

观测回调须提供同一ROS时钟的now_ns/after_ns和多帧r3_scan格式数据；
真实执行还需要实测动作方向/步幅、已验证相对坐标和时间语义。
"""
import json,math,statistics
from pathlib import Path

class AlignmentStopped(ValueError):pass
AXES=('x_m','y_m','z_m','yaw_deg')
ACTIONS=frozenset(('left_once','right_once'))
def angle(value):return (value+180)%360-180
def finite(value):return type(value) in (int,float) and math.isfinite(value)

def validate_profile(cfg):
    if cfg.get('schema')!='r3.coarse-tag.v1' or cfg.get('mode') not in ('report_only','execute_calibrated'):raise AlignmentStopped('invalid profile')
    if cfg.get('target_id') is not None and (type(cfg['target_id']) is not int or cfg['target_id']<0):raise AlignmentStopped('invalid target ID')
    if cfg.get('expected_load') not in ('empty','single','stacked'):raise AlignmentStopped('expected load required')
    if type(cfg.get('min_frames')) is not int or not 3<=cfg['min_frames']<=30:raise AlignmentStopped('need at least three frames')
    if not finite(cfg.get('max_age_s')) or not 0<cfg['max_age_s']<=5:raise AlignmentStopped('invalid age budget')
    if type(cfg.get('max_actions')) is not int or not 1<=cfg['max_actions']<=10:raise AlignmentStopped('invalid action budget')
    limits=cfg.get('stability_limits',{})
    if set(limits)!=set(AXES) or any(v is not None and (not finite(v) or v<0) for v in limits.values()):raise AlignmentStopped('invalid stability limits')
    return cfg

def summarize_window(window,cfg,after_ns=None):
    """去重、检查丢失/过期/发布者重启及抖动；不把接收次数当新图。"""
    validate_profile(cfg);target=cfg['target_id']
    if target is None:raise AlignmentStopped('select target ID explicitly')
    now=window.get('now_ns');barrier=window.get('after_ns') if after_ns is None else after_ns
    if type(now) is not int or type(barrier) is not int or now<=0 or barrier<0 or barrier>=now:raise AlignmentStopped('same-clock window timestamps required')
    rows=[r for r in window.get('observations',[]) if r.get('id')==target]
    if not rows:raise AlignmentStopped('target lost or absent')
    seen=set();valid=[];seq=None;frame=None;last_stamp=0
    for r in rows:
        stamp=r.get('stamp_ns');f=r.get('frame_id')
        if type(stamp) is not int or stamp<=barrier or stamp>now or now-stamp>cfg['max_age_s']*1e9:raise AlignmentStopped('stale/future/pre-barrier target')
        if not isinstance(f,str) or not f or r.get('headers_disagree'):raise AlignmentStopped('missing/conflicting frame headers')
        if frame is not None and f!=frame:raise AlignmentStopped('reference frame changed')
        frame=f;identity=(target,stamp,f)
        if identity in seen:continue
        if stamp<last_stamp:raise AlignmentStopped('capture time moved backwards')
        incoming=r.get('source_seq')
        if incoming is not None and (type(incoming) is not int or incoming<0):raise AlignmentStopped('invalid publisher sequence')
        if seq is not None and incoming is not None and incoming<seq:raise AlignmentStopped('publisher restarted')
        seq=incoming;last_stamp=stamp;seen.add(identity)
        p=r.get('position_raw_m',{});values=[p.get(k) for k in ('x','y','z')]+[r.get('yaw_raw_deg')]
        if not all(finite(x) for x in values):raise AlignmentStopped('invalid pose')
        valid.append((r,values))
    if len(valid)<cfg['min_frames']:raise AlignmentStopped('insufficient distinct declared captures')
    yaw0=valid[0][1][3];series=list(zip(*[v[:3]+[yaw0+angle(v[3]-yaw0)] for r,v in valid]))
    median={k:statistics.median(v) for k,v in zip(AXES,series)};median['yaw_deg']=angle(median['yaw_deg'])
    spans={k:max(v)-min(v) for k,v in zip(AXES,series)};limits=cfg['stability_limits']
    stable=all(limits[k] is not None for k in AXES)
    if any(limits[k] is not None and spans[k]>limits[k] for k in AXES):raise AlignmentStopped('target window unstable')
    return {'target_id':target,'frame_id':frame,'expected_load':cfg['expected_load'],'median':median,'spans':spans,
        'declared_fresh_frames':len(valid),'stable_under_configured_limits':True if stable else None,
        'latest_stamp_ns':max(r['stamp_ns'] for r,v in valid),'now_ns':now,
        'relative_frame_verified':all(r.get('pose_for_control_verified') is True for r,v in valid),
        'capture_timing_verified':all(r.get('timestamp_capture_semantics_field_verified') is True for r,v in valid),
        'physical_load_sensor_confirmation':False}

def teach_tag_reference(window,cfg,operator_confirmed=False):
    """保存前先由人确认机器人在正确位置/朝向；单帧ID1不会自动变成基准。"""
    if operator_confirmed is not True:raise AlignmentStopped('operator confirmation required')
    summary=summarize_window(window,cfg)
    return dict(summary,schema='r3.taught-tag.v1',operator_confirmed=True,
        control_reference_verified=bool(summary['relative_frame_verified'] and summary['capture_timing_verified'] and summary['stable_under_configured_limits']),
        note='人工确认的多帧参考；未经坐标/时间/稳定性验证时仅为报告用草稿')

def save_taught_reference(path,reference,operator_confirmed=False):
    """仅显式调用才写文件；不从扫码结果自动写入，也不改路线参数。"""
    if operator_confirmed is not True or reference.get('operator_confirmed') is not True:raise AlignmentStopped('confirmation required before saving')
    p=Path(path);p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(reference,ensure_ascii=False,indent=2),encoding='utf-8')

def report_taught_tag_error(window,cfg,reference,after_ns=None):
    current=summarize_window(window,cfg,after_ns)
    if reference.get('schema')!='r3.taught-tag.v1' or not reference.get('operator_confirmed'):raise AlignmentStopped('explicitly taught reference required')
    for key in ('target_id','frame_id','expected_load'):
        if current[key]!=reference.get(key):raise AlignmentStopped('reference '+key+' mismatch')
    goal=reference.get('median',{})
    if any(not finite(goal.get(k)) for k in AXES):raise AlignmentStopped('invalid taught pose')
    delta={k:current['median'][k]-goal[k] for k in AXES};delta['yaw_deg']=angle(delta['yaw_deg'])
    return {'status':'report_only','current':current,'error_in_declared_frame':delta,
        'route_alignment_verified':False,'image_center_known':False,'motion_requested':False}

def report_image_center(observation,image=None,camera_info=None):
    """只有匹配图像的真实像素中心才能判断画面偏差；不投影未验证/map位姿。"""
    unknown={'status':'unknown','reason':'need verified Tag pixels and matching image header; map pose is insufficient'}
    if not image or observation.get('pixel_observation_verified') is not True:return unknown
    uv=observation.get('tag_center_px');w=image.get('width');h=image.get('height')
    if type(w) is not int or type(h) is not int or min(w,h)<=0 or not isinstance(uv,(list,tuple)) or len(uv)!=2 or not all(finite(v) for v in uv):return unknown
    if observation.get('pixel_stamp_ns')!=image.get('stamp_ns') or not image.get('frame_id') or not 0<=uv[0]<w or not 0<=uv[1]<h:return unknown
    cx,cy=(w-1)/2,(h-1)/2
    result={'status':'pixel_error_report','image_frame':image['frame_id'],'image_center_px':[cx,cy],
        'tag_center_px':list(uv),'offset_px':[uv[0]-cx,uv[1]-cy],'centered':None,'reason':'centering tolerance not measured/configured','motion_requested':False}
    if camera_info and camera_info.get('frame_id')==image['frame_id']:
        k=camera_info.get('K')
        if isinstance(k,(list,tuple)) and len(k)==9 and all(finite(v) for v in k):result['camera_principal_point_px']=[k[2],k[5]]
    return result

def execution_blockers(cfg,reference,current):
    """任何缺项都只报告；死区必须大于实测最小一次动作效果，单位为度。"""
    c=cfg.get('calibration',{});reasons=[]
    if cfg['mode']!='execute_calibrated':reasons.append('report_only mode')
    if reference.get('control_reference_verified') is not True:reasons.append('reference pose/time/stability unverified')
    if not current['relative_frame_verified'] or not current['capture_timing_verified'] or current['stable_under_configured_limits'] is not True:reasons.append('current pose/time/stability unverified')
    step=c.get('minimum_yaw_change_deg');band=cfg.get('deadband_deg');improvement=cfg.get('minimum_improvement_deg')
    if c.get('verified') is not True or not finite(step) or step<=0:reasons.append('single-action effect uncalibrated')
    if not finite(band) or not finite(step) or not band>step:reasons.append('deadband must exceed measured minimum step')
    if not finite(improvement) or improvement<0:reasons.append('improvement threshold unconfigured')
    if c.get('positive_error_action') not in ACTIONS or c.get('negative_error_action') not in ACTIONS or c.get('positive_error_action')==c.get('negative_error_action'):reasons.append('turn direction mapping uncalibrated')
    return reasons

def coarse_align_to_taught_tag(cfg,reference,observe_window,execute_one=None,check_cancel=lambda:None):
    """一次离散转向后必须读取动作结束之后的新窗口；有界、无反复摆动。

    默认无动作。将来执行回调必须返回protocol_completed=True及同ROS时钟
    after_ns；当前机器人接口没有自动接入本函数，不能用True/None冒充完成。
    """
    validate_profile(cfg);history=[];barrier=None;previous=None;previous_sign=None
    try:
        for attempt in range(cfg['max_actions']+1):
            check_cancel();window=observe_window(after_ns=barrier)
            report=report_taught_tag_error(window,cfg,reference,barrier);history.append(report)
            blockers=execution_blockers(cfg,reference,report['current'])
            if blockers or execute_one is None:return {'status':'report_only','blockers':blockers or ['no action adapter'],'history':history,'actions':attempt,'motion_requested':attempt>0}
            error=report['error_in_declared_frame']['yaw_deg'];magnitude=abs(error)
            if magnitude<=cfg['deadband_deg']:return {'status':'within_taught_deadband','history':history,'actions':attempt,'physical_pose_confirmed':False}
            sign=1 if error>0 else -1
            if previous is not None and (previous-magnitude<=cfg['minimum_improvement_deg'] or sign!=previous_sign):raise AlignmentStopped('no improvement or overshoot; no reversal')
            if attempt==cfg['max_actions']:raise AlignmentStopped('maximum coarse actions reached')
            action=cfg['calibration']['positive_error_action' if error>0 else 'negative_error_action'];check_cancel()
            result=execute_one(action)
            if not isinstance(result,dict) or result.get('protocol_completed') is not True or type(result.get('after_ns')) is not int or result['after_ns']<window['now_ns']:raise AlignmentStopped('action completion/time unproven; do not continue')
            barrier=result['after_ns'];previous=magnitude;previous_sign=sign
        raise AlignmentStopped('unreachable budget end')
    except Exception as exc:
        return {'status':'stopped','reason':str(exc),'history':history,'automatic_retry':False,'physical_pose_confirmed':False}

def main(argv=None):
    """只处理用户保存的扫码JSON；保存参考必须明确--confirm，绝不连接机器人。"""
    import argparse
    p=argparse.ArgumentParser(description='人工确认保存AR参考，或只比较原始同frame位置；不控制机器人。')
    p.add_argument('mode',choices=['teach','compare']);p.add_argument('observation',type=Path);p.add_argument('reference',type=Path)
    p.add_argument('--id',type=int,required=True);p.add_argument('--load',choices=['empty','single','stacked'],required=True)
    p.add_argument('--confirm',action='store_true');p.add_argument('--profile',type=Path,default=Path(__file__).parent/'config/tag_alignment.json')
    a=p.parse_args(argv)
    try:
        cfg=json.loads(a.profile.read_text(encoding='utf-8'));cfg['target_id']=a.id;cfg['expected_load']=a.load
        raw=json.loads(a.observation.read_text(encoding='utf-8'));raw=raw.get('scan_result',raw)
        window={'observations':raw.get('capture_history',raw.get('observations',[])),'now_ns':raw.get('window_now_ns'),'after_ns':raw.get('window_after_ns')}
        if a.mode=='teach':
            ref=teach_tag_reference(window,cfg,a.confirm);save_taught_reference(a.reference,ref,a.confirm)
            print(json.dumps({'status':'manually_taught_report_reference_saved','path':str(a.reference),'control_reference_verified':ref['control_reference_verified'],'motion_requested':False},ensure_ascii=False))
        else:
            reference=json.loads(a.reference.read_text(encoding='utf-8'));print(json.dumps(report_taught_tag_error(window,cfg,reference),ensure_ascii=False))
        return 0
    except (ValueError,OSError,KeyError) as exc:
        print(json.dumps({'status':'reference_not_saved_or_compared','reason':str(exc),'motion_requested':False},ensure_ascii=False));return 1
if __name__=='__main__':raise SystemExit(main())
