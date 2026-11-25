"""只处理显式保存的视觉JSON：示教、测量动作效果；不发任何机器人命令。"""
import argparse,json,math,statistics
from pathlib import Path
from entry_alignment import validate,pose_window,basis,relative_robot_error,teach_reference,summarize,SIGNATURE,ACTIONS,EntryNotReady
ROOT=Path(__file__).resolve().parent
def read_window(path):
    row=json.loads(Path(path).read_text(encoding='utf-8'));row=row.get('scan_result',row)
    if 'entry_observation' in row:return row['entry_observation']
    if 'pose_samples' in row:return row
    raise EntryNotReady('need scan --entry --json result; raw /map pose is not a calibrated window')
def effect_trial(before,after,reference,cfg):
    b=summarize(before,reference,cfg);a=summarize(after,reference,cfg)
    if not b['verified'] or not a['verified'] or not b['stable'] or not a['stable']:raise EntryNotReady('verify frame/time/scale and multi-frame stability before measuring action')
    if min(r['stamp_ns'] for r in pose_window(after,cfg))<=max(r['stamp_ns'] for r in pose_window(before,cfg)):raise EntryNotReady('before/after capture windows overlap or are reversed')
    from tag_alignment import angle
    return {'x_m':a['x_m']-b['x_m'],'angle_deg':angle(a['angle_deg']-b['angle_deg']),'y_m':a['y_offset_m']-b['y_offset_m'],
        'noise_x_m':b['spans']['x_m']+a['spans']['x_m'],'noise_angle_deg':b['spans']['angle_deg']+a['spans']['angle_deg'],
        'before_stamps':[r['stamp_ns'] for r in pose_window(before,cfg)],'after_stamps':[r['stamp_ns'] for r in pose_window(after,cfg)]}
def record_effect(current,name,trial,reference,operator_confirmed=False):
    if operator_confirmed is not True or name not in ACTIONS:raise EntryNotReady('confirm exactly one matching manual/native action was performed')
    if current is None:current={'schema':'r3.entry-effects.v1','operator_confirmed':True,'action_signature':SIGNATURE,'reference_stamps':reference['capture_stamps_ns'],'actions':{},'raw_trials':{}}
    if current.get('reference_stamps')!=reference['capture_stamps_ns'] or current.get('action_signature')!=SIGNATURE:raise EntryNotReady('effect file belongs to another reference/action version; do not merge')
    trials=current.setdefault('raw_trials',{}).setdefault(name,[]);trials.append(trial);current['raw_trials'][name]=trials[-8:];trials=current['raw_trials'][name]
    key='x_m' if name.startswith('strafe') else 'angle_deg';noise='noise_x_m' if name.startswith('strafe') else 'noise_angle_deg'
    significant=all(abs(r[key])>r[noise] and r[key]!=0 for r in trials)
    same_sign=len({1 if r[key]>0 else -1 for r in trials})==1
    current['actions'][name]={k:statistics.median(t[k] for t in trials) for k in ('x_m','angle_deg','y_m')}
    current['actions'][name].update(trials=len(trials),verified=significant and same_sign and len(trials)>=2,
        spans={k:max(t[k] for t in trials)-min(t[k] for t in trials) for k in ('x_m','angle_deg','y_m')})
    return current
def inspect_window(window,cfg):
    right,forward,up=basis(cfg);rows=pose_window(window,cfg);first=rows[0]
    ref={'tag_pose':first,'tag_y_m':sum(a*b for a,b in zip(first['position'],forward))}
    errors=[relative_robot_error(ref,r,cfg) for r in rows]
    return {'status':'diagnostic_only','samples':len(rows),'frame':cfg['planar']['frame'],'no_reference_saved':True,
        'observed_reference_tag_y_m':ref['tag_y_m'],'relative_spans':{k:max(e[k] for e in errors)-min(e[k] for e in errors) for k in ('x_m','angle_deg','y_offset_m')},
        'geometry_verified':all(r['geometry_verified'] for r in rows),'image_header':window.get('image_header'),'camera_info':window.get('camera_info'),
        'pixel_samples':window.get('pixel_samples',[]),'available_tf_frames':window.get('available_tf_frames'),'motion_requested':False}
def main(argv=None):
    p=argparse.ArgumentParser(description='离线示教入口、测量真实动作效果；没有任何动作执行接口。')
    p.add_argument('--profile',type=Path,default=ROOT/'config/entry_alignment.json');sub=p.add_subparsers(dest='op',required=True)
    a=sub.add_parser('inspect');a.add_argument('sample',type=Path)
    a=sub.add_parser('teach');a.add_argument('sample',type=Path);a.add_argument('out',type=Path);a.add_argument('--confirm',action='store_true')
    a=sub.add_parser('measure');a.add_argument('action',choices=list(ACTIONS));a.add_argument('before',type=Path);a.add_argument('after',type=Path);a.add_argument('out',type=Path);a.add_argument('--confirm',action='store_true')
    a=sub.add_parser('suggest');a.add_argument('effects',type=Path)
    args=p.parse_args(argv)
    try:
        cfg=json.loads(args.profile.read_text(encoding='utf-8'));validate(cfg)
        if args.op=='inspect':result=inspect_window(read_window(args.sample),cfg)
        elif args.op=='teach':
            result=teach_reference(read_window(args.sample),cfg,args.confirm);args.out.parent.mkdir(parents=True,exist_ok=True);args.out.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
            result={'status':'reference_saved','verified':result['verified'],'target_x_m':0,'target_angle_deg':0,'recorded_y_m':result['tag_y_m'],'path':str(args.out),'motion_requested':False}
        elif args.op=='measure':
            if not cfg['reference_path']:raise EntryNotReady('configure reference_path before measuring effects')
            ref=json.loads((ROOT/cfg['reference_path']).read_text(encoding='utf-8'))
            current=json.loads(args.out.read_text(encoding='utf-8')) if args.out.exists() else None
            result=record_effect(current,args.action,effect_trial(read_window(args.before),read_window(args.after),ref,cfg),ref,args.confirm)
            args.out.parent.mkdir(parents=True,exist_ok=True);args.out.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
            result={'status':'effect_trial_recorded','action':args.action,'measured':result['actions'][args.action],'motion_requested':False}
        else:
            effects=json.loads(args.effects.read_text(encoding='utf-8'));actions=effects['actions']
            result={'status':'measured_lower_bounds_only_not_auto_configured','deadband_x_m_must_exceed':max([abs(v['x_m'])+v['spans']['x_m'] for k,v in actions.items() if k.startswith('strafe')] or [0]),
                'deadband_angle_deg_must_exceed':max([abs(v['angle_deg'])+v['spans']['angle_deg'] for k,v in actions.items() if k.startswith('turn')] or [0]),
                'note':'按实测完整动作计算；若死区太宽/丢码/动作不改善则保持report_only，不能把大动作缩小当作已验证微调','motion_requested':False}
        print(json.dumps(result,ensure_ascii=False));return 0
    except Exception as exc:print(json.dumps({'status':'setup_not_completed','reason':str(exc),'motion_requested':False},ensure_ascii=False));return 1
if __name__=='__main__':raise SystemExit(main())
