"""入口示教参考系：实际示教位姿定义误差x=0/角度=0，y记录且只监测。

所有坐标必须先经过同时间TF进入人工核对过的平面frame。/map的x/y/yaw
不能直接作为误差。Tag不必位于画面中心，像素中心偏差仅作诊断。
默认只报告；离散动作采用实测耦合效果，恶化即停。
"""
import copy,json,math,statistics,time
from pathlib import Path
from entry_camera import CameraUnavailable,finite,unit,multiply,rotate
from tag_alignment import angle
class EntryNotReady(ValueError):pass
ACTIONS={'strafe_left_once':'strafe_left_once','strafe_right_once':'strafe_right_once','turn_left_once':'left_turn','turn_right_once':'right_turn'}
SIGNATURE='R3:v6:single:strafe-full-cycle5:turn-full-segment3:keep-hands'
def axis(spec):
    if not isinstance(spec,dict) or spec.get('axis') not in ('x','y','z') or type(spec.get('sign')) is not int or spec['sign'] not in (-1,1):raise EntryNotReady('planar axes not verified/configured')
    result=[0.,0.,0.];result['xyz'.index(spec['axis'])]=spec['sign'];return result
def dot(a,b):return sum(x*y for x,y in zip(a,b))
def cross(a,b):return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
def basis(cfg):
    axes=cfg['planar'];x=axis(axes['right']);y=axis(axes['forward']);up=axis(axes['up'])
    if cross(x,y)!=up:raise EntryNotReady('right/forward/up axes must be orthogonal right-handed')
    return x,y,up
def load_bundle(project):
    root=Path(project);p=root/'config/entry_alignment.json'
    if not p.exists():return {'profile':{'enabled':False,'mode':'report_only'},'reference':None,'effects':None}
    cfg=json.loads(p.read_text(encoding='utf-8'));validate(cfg)
    result={'profile':cfg,'reference':None,'effects':None}
    for key in ('reference','effects'):
        name=cfg[key+'_path']
        if name is None:continue
        f=Path(name)
        if f.is_absolute() or '..' in f.parts or f.parts[:1]!=('config',) or f.suffix!='.json':raise EntryNotReady(key+' must be project/config JSON')
        result[key]=json.loads((root/f).read_text(encoding='utf-8'))
    if cfg['enabled'] and cfg['mode']=='execute_calibrated':
        reasons=readiness(result)
        if reasons:raise EntryNotReady('entry alignment not ready BEFORE motion: '+'; '.join(reasons))
    return result
def validate(cfg):
    if cfg.get('schema')!='r3.entry-align.v1' or type(cfg.get('enabled')) is not bool or cfg.get('mode') not in ('report_only','execute_calibrated'):raise EntryNotReady('invalid entry profile')
    if cfg.get('target_id') is not None and (type(cfg['target_id']) is not int or cfg['target_id']<0):raise EntryNotReady('invalid Tag ID')
    if cfg.get('expected_load')!='single':raise EntryNotReady('current correction exports support single load only')
    for name,lo,hi in [('window_timeout_s',.1,10),('total_timeout_s',1,120),('max_age_s',.05,5),('settle_s',0,5)]:
        if not finite(cfg.get(name)) or not lo<=cfg[name]<=hi:raise EntryNotReady('invalid '+name)
    if type(cfg.get('min_frames')) is not int or not 3<=cfg['min_frames']<=10:raise EntryNotReady('at least 3 distinct frames required')
    if type(cfg.get('max_actions')) is not int or not 1<=cfg['max_actions']<=8:raise EntryNotReady('invalid action budget')
    if not isinstance(cfg.get('actions'),list) or not cfg['actions'] or len(set(cfg['actions']))!=len(cfg['actions']) or any(a not in ACTIONS for a in cfg['actions']):raise EntryNotReady('invalid finite action whitelist')
    for group in ('deadband','stability'):
        if set(cfg[group])!={'x_m','angle_deg'}:raise EntryNotReady('invalid '+group)
        if any(v is not None and (not finite(v) or v<=0) for v in cfg[group].values()):raise EntryNotReady('invalid '+group+' values')
    for name in ('y_abort_m','min_improvement','working_angle_deg','working_x_m'):
        if cfg.get(name) is not None and (not finite(cfg[name]) or cfg[name]<=0):raise EntryNotReady('invalid '+name)
    if type(cfg.get('max_axis_switches',1)) is not int or not 0<=cfg.get('max_axis_switches',1)<=2:raise EntryNotReady('invalid axis switch budget')
    return cfg
def planar_signature(cfg):return json.dumps(cfg['planar'],sort_keys=True,separators=(',',':'))
def pose_window(window,cfg,after_ns=None):
    """输入为真实TF后的完整Tag位姿，不把图像接收计数当新帧。"""
    if window.get('target_lost'):raise EntryNotReady('target lost; no correction')
    barrier=window['after_ns'] if after_ns is None else after_ns;now=window['now_ns'];samples=window.get('pose_samples',[])
    if type(now) is not int or type(barrier) is not int or now<=barrier:raise EntryNotReady('invalid ROS capture window')
    records=[];seen=set();last_stamp=0;last_seq=None
    for row in samples:
        if row.get('id')!=cfg['target_id']:continue
        stamp=row.get('stamp_ns')
        if type(stamp) is not int or not barrier<stamp<=now or now-stamp>cfg['max_age_s']*1e9:raise EntryNotReady('stale/future/pre-action target')
        if row.get('frame_id')!=cfg['planar']['frame'] or row.get('headers_conflict'):raise EntryNotReady('planar frame/header mismatch')
        key=(row['id'],stamp,row['frame_id'])
        if key in seen:continue
        seq=row.get('source_seq')
        if stamp<last_stamp or seq is not None and last_seq is not None and seq<last_seq:raise EntryNotReady('publisher/time restarted')
        if not all(finite(x) for x in row.get('position',[])) or len(row.get('position',[]))!=3:raise EntryNotReady('invalid relative position')
        row=dict(row,quaternion=unit(row['quaternion']));seen.add(key);records.append(row);last_stamp=stamp;last_seq=seq
    if len(records)<cfg['min_frames']:raise EntryNotReady('target missing/insufficient fresh planar poses')
    return records
def relative_robot_error(reference,row,cfg):
    """固定Tag假设：T_ref_robot = T_ref_tag * inverse(T_current_tag)。"""
    right,forward,up=basis(cfg);q0=unit(reference['tag_pose']['quaternion']);q=unit(row['quaternion']);delta=unit(multiply(q0,[-q[0],-q[1],-q[2],q[3]]))
    rotated=rotate(delta,row['position']);offset=[a-b for a,b in zip(reference['tag_pose']['position'],rotated)]
    f=rotate(delta,forward);theta=math.degrees(math.atan2(-dot(f,right),dot(f,forward)))
    return {'x_m':dot(offset,right),'angle_deg':angle(theta),'y_offset_m':dot(offset,forward),
        'reference_tag_y_m':reference['tag_y_m'],'observed_tag_y_m':dot(row['position'],forward),'y_controlled':False}
def teach_reference(window,cfg,operator_confirmed=False):
    if operator_confirmed is not True:raise EntryNotReady('confirm ideal physical entry pose before teaching')
    basis(cfg);rows=pose_window(window,cfg);first=rows[0]
    # 用一帧完整旋转作原点，全部样本需满足人工配置的稳定界；不平均冲突旋转。
    reference={'schema':'r3.entry-reference.v1','operator_confirmed':True,'id':cfg['target_id'],'expected_load':cfg['expected_load'],
        'planar_frame':cfg['planar']['frame'],'planar_signature':planar_signature(cfg),'action_signature':SIGNATURE,
        'tag_pose':{'position':list(first['position']),'quaternion':list(first['quaternion'])},'tag_y_m':dot(first['position'],axis(cfg['planar']['forward'])),
        'target_x_m':0.,'target_angle_deg':0.,'y_controlled':False,'alignment_target':'operator_taught_pose','image_center_required':False,
        'capture_stamps_ns':[r['stamp_ns'] for r in rows]}
    checked=summarize(window,reference,cfg)
    if checked['stable'] is not True:raise EntryNotReady('configure/measure stability tolerances before teaching control reference')
    reference['verified']=all(r.get('geometry_verified') is True for r in rows)
    pixels=window.get('pixel_samples',[])
    # 允许任意可见的示教位置；像素中心偏差不作为示教准入或控制目标。
    reference['taught_image']=None if not pixels else {'center_error_x_px':statistics.median(r['center_error_x_px'] for r in pixels),'projection_verified':all(r['projection_verified'] for r in pixels),'image_y_center_required':False,'diagnostic_only':True}
    reference['camera_fingerprint']=window.get('camera_fingerprint');reference['sample_error_ranges']=checked['spans']
    return reference
def summarize(window,reference,cfg,after_ns=None):
    if reference.get('schema')!='r3.entry-reference.v1' or not reference.get('operator_confirmed'):raise EntryNotReady('taught reference missing')
    if (reference.get('id'),reference.get('expected_load'),reference.get('planar_signature'))!=(cfg['target_id'],cfg['expected_load'],planar_signature(cfg)):raise EntryNotReady('reference ID/load/coordinate definition mismatch')
    if reference.get('camera_fingerprint') is not None and window.get('camera_fingerprint')!=reference['camera_fingerprint']:raise EntryNotReady('camera calibration fingerprint changed; reteach/recalibrate')
    rows=pose_window(window,cfg,after_ns);errors=[relative_robot_error(reference,r,cfg) for r in rows]
    med={k:statistics.median(e[k] for e in errors) for k in ('x_m','angle_deg','y_offset_m','observed_tag_y_m')}
    spans={k:max(e[k] for e in errors)-min(e[k] for e in errors) for k in ('x_m','angle_deg')}
    stable=all(cfg['stability'][k] is not None for k in spans)
    if any(cfg['stability'][k] is not None and spans[k]>cfg['stability'][k] for k in spans):raise EntryNotReady('target poses unstable')
    if cfg['y_abort_m'] is not None and abs(med['y_offset_m'])>cfg['y_abort_m']:raise EntryNotReady('y drift outside taught safety region; no forward/back correction')
    if cfg['working_angle_deg'] is not None and abs(med['angle_deg'])>cfg['working_angle_deg']:raise EntryNotReady('outside calibrated angle region')
    if cfg['working_x_m'] is not None and abs(med['x_m'])>cfg['working_x_m']:raise EntryNotReady('outside calibrated lateral region')
    return dict(med,reference_tag_y_m=reference['tag_y_m'],target_x_m=0.,target_angle_deg=0.,y_controlled=False,
        alignment_target='operator_taught_pose',image_center_required=False,
        spans=spans,stable=True if stable else None,verified=reference.get('verified') is True and all(r.get('geometry_verified') is True for r in rows),
        latest_stamp_ns=max(r['stamp_ns'] for r in rows),frame_id=cfg['planar']['frame'],samples=len(rows))
def readiness(bundle):
    cfg=bundle['profile'];ref=bundle.get('reference');effects=bundle.get('effects');reasons=[]
    if cfg.get('target_id') is None:reasons.append('choose Tag ID explicitly')
    try:basis(cfg)
    except (EntryNotReady,KeyError):reasons.append('planar frame axes not configured')
    if not cfg['planar'].get('frame') or not all(cfg['planar'].get(k) is True for k in ('axes_verified','capture_time_verified','tag_scale_verified')):reasons.append('planar frame/time/tag scale unverified')
    if not ref or ref.get('verified') is not True or ref.get('action_signature')!=SIGNATURE:reasons.append('reference not taught/verified for this body/load')
    if ref and (ref.get('id'),ref.get('expected_load'),ref.get('planar_signature'))!=(cfg.get('target_id'),cfg.get('expected_load'),planar_signature(cfg)):reasons.append('reference ID/load/axes do not match profile')
    if not effects or effects.get('schema')!='r3.entry-effects.v1' or effects.get('operator_confirmed') is not True or effects.get('action_signature')!=SIGNATURE or ref and effects.get('reference_stamps')!=ref.get('capture_stamps_ns'):reasons.append('action calibration missing or belongs to another reference')
    for k in ('x_m','angle_deg'):
        if cfg['deadband'][k] is None or cfg['stability'][k] is None:reasons.append(k+' deadband/stability not measured')
    for key in ('y_abort_m','min_improvement','working_angle_deg','working_x_m'):
        if cfg.get(key) is None:reasons.append(key+' unconfigured')
    actions={} if not effects else effects.get('actions',{})
    for name in cfg['actions']:
        record=actions.get(name,{})
        if record.get('verified') is not True or record.get('trials',0)<2 or any(not finite(record.get(k)) for k in ('x_m','angle_deg','y_m')):reasons.append(name+' needs at least two matching measured trials')
        for key in ('x_m','angle_deg'):
            spread=record.get('spans',{}).get(key)
            if spread is not None and cfg['deadband'][key] is not None and spread>=cfg['deadband'][key]:reasons.append(name+' trial spread exceeds deadband')
    for key,prefix in [('x_m','strafe_'),('angle_deg','turn_')]:
        values=[abs(actions.get(a,{}).get(key,0)) for a in cfg['actions'] if a.startswith(prefix) and finite(actions.get(a,{}).get(key)) and abs(actions[a][key])>0]
        if not values or cfg['deadband'][key] is None or cfg['deadband'][key]<=min(values):reasons.append(key+' deadband must exceed measured minimum complete action')
    return reasons
def choose_action(error,cfg,effects):
    """按实测二维耦合效果选一步；不凭左右字样猜符号，不主动改y。"""
    band=cfg['deadband'];score=sum((error[k]/band[k])**2 for k in ('x_m','angle_deg'))
    if all(abs(error[k])<=band[k] for k in ('x_m','angle_deg')):return None
    choices=[]
    for name in cfg['actions']:
        effect=effects['actions'][name];pred={'x_m':error['x_m']+effect['x_m'],'angle_deg':angle(error['angle_deg']+effect['angle_deg'])}
        if any(abs(error[k])<=band[k] and abs(pred[k])>band[k] for k in pred):continue
        if any(abs(pred[k])>max(abs(error[k])+cfg['stability'][k],band[k]) for k in pred):continue
        if abs(error['y_offset_m']+effect['y_m'])>cfg['y_abort_m']:continue
        after=sum((pred[k]/band[k])**2 for k in pred)
        if score-after>=cfg['min_improvement']:choices.append((after,name))
    if not choices:raise EntryNotReady('no calibrated coarse step safely improves both x/angle; use smaller action or manual reposition')
    return min(choices)[1]
def align_to_entry(bundle,observe,execute_one=None,check=lambda:None,clock=time.monotonic):
    cfg=bundle['profile'];history=[];actions=[];attempted=[];barrier=None;deadline=clock()+cfg.get('total_timeout_s',1);prior_score=None;last_by_axis={};last_axis=None;switches=0;previous_error=None
    try:
        if not cfg.get('enabled'):return {'status':'report_only','reason':'entry alignment disabled','actions':[],'ready_for_obstacle':False}
        check();window=observe(after_ns=None,timeout_s=min(cfg['window_timeout_s'],deadline-clock()))
        ref=bundle.get('reference');reasons=readiness(bundle)
        if ref:
            report=summarize(window,ref,cfg);history.append(report)
        else:report=None
        if cfg['mode']=='report_only' or reasons or execute_one is None:
            return {'status':'report_only','reason':'; '.join(reasons) or 'configured report-only/no action adapter','history':history,'observation':window,'actions':[],'ready_for_obstacle':False}
        while True:
            check()
            if clock()>=deadline:raise EntryNotReady('alignment total deadline exceeded')
            if not report['verified'] or report['stable'] is not True:raise EntryNotReady('live coordinate/time/stability unverified')
            score=sum((report[k]/cfg['deadband'][k])**2 for k in ('x_m','angle_deg'))
            if all(abs(report[k])<=cfg['deadband'][k] for k in ('x_m','angle_deg')):return {'status':'within_taught_deadband','history':history,'actions':actions,'y_controlled':False,'ready_for_obstacle':True,'physical_pose_confirmed':False}
            if previous_error is not None and any(abs(report[k])>max(abs(previous_error[k])+cfg['stability'][k],cfg['deadband'][k]) for k in ('x_m','angle_deg')):raise EntryNotReady('observed axis worsened; stop')
            if prior_score is not None and prior_score-score<cfg['min_improvement']:raise EntryNotReady('no measured improvement; no retry/reversal')
            if len(actions)>=cfg['max_actions']:raise EntryNotReady('alignment action budget reached')
            name=choose_action(report,cfg,bundle['effects'])
            axis_name='strafe' if name.startswith('strafe') else 'turn'
            if axis_name in last_by_axis and last_by_axis[axis_name]!=name:raise EntryNotReady('would reverse coarse action; stop for manual check')
            if last_axis is not None and last_axis!=axis_name:
                switches+=1
                if switches>cfg.get('max_axis_switches',1):raise EntryNotReady('axis switch budget reached; avoid oscillation')
            remaining=deadline-clock()
            if remaining<2:raise EntryNotReady('insufficient remaining action time')
            check();attempted.append(name);result=execute_one(name,remaining)
            if not isinstance(result,dict) or result.get('protocol_completed') is not True or type(result.get('after_ns')) is not int or result['after_ns']<window['now_ns']:raise EntryNotReady('action completion/time uncertain; no further command')
            actions.append(name);barrier=result['after_ns'];last_by_axis[axis_name]=name;last_axis=axis_name;prior_score=score;previous_error=report;check()
            remaining=deadline-clock()
            if remaining<=0:raise EntryNotReady('deadline after action; no next step')
            window=observe(after_ns=barrier,timeout_s=min(cfg['window_timeout_s'],remaining));report=summarize(window,ref,cfg,barrier);history.append(report)
    except Exception as exc:
        if cfg.get('mode')=='report_only' and not attempted and isinstance(exc,(CameraUnavailable,EntryNotReady,ImportError)):
            return {'status':'report_only','reason':str(exc),'history':history,'actions':[],'ready_for_obstacle':False,'automatic_retry':False}
        return {'status':'stopped','reason':str(exc),'history':history,'actions':actions,'attempted_actions':attempted,'ready_for_obstacle':False,'automatic_retry':False,'physical_pose_confirmed':False}
