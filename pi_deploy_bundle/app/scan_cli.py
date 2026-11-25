"""独立有限扫码：只订阅现有AR topic，不启动相机服务、不碰UART/动作。"""
import argparse,contextlib,json,shlex,subprocess,sys
from pathlib import Path
import task_runner
from tag_reporting import enrich_for_project,human_lines,reporting_options
ROOT=Path(__file__).resolve().parent
SETUPS=('/opt/ros/noetic/setup.bash','/home/lemon/ros_catkin_ws/install_isolated/setup.bash','/home/lemon/catkin_ws/devel/setup.bash')

def scan_worker(cfg,project=ROOT,entry=False,save_image=None):
    from run_auto import identity
    identity()
    import rospy
    from r3_scan import scan_tags
    with contextlib.redirect_stdout(sys.stderr):rospy.init_node('r3_scan_only',anonymous=True,disable_signals=True)
    try:
        if entry:
            from entry_alignment import validate
            from entry_observer import observe_entry_window
            profile=json.loads((Path(project)/'config/entry_alignment.json').read_text(encoding='utf-8'));validate(profile);profile['mode']='report_only'
            window=observe_entry_window(profile,cfg['terminal_scan'],timeout_s=profile['window_timeout_s'],save_image=save_image)
            return dict(enrich_for_project(window['raw_scan'],project),entry_observation=window)
        options=dict(cfg['terminal_scan']);options['enabled']=True;options['settle_s']=0
        options['min_fresh_frames']=reporting_options(project)['min_frames']
        result=scan_tags(options,lambda:None,options['timeout_s'])
        return enrich_for_project(result,project)
    finally:rospy.signal_shutdown('read-only scan complete')

def main(argv=None,worker=scan_worker):
    p=argparse.ArgumentParser(description='只读胸前AR扫码；不移动，不begin，不连接动作owner。')
    p.add_argument('--entry',action='store_true',help='只读采集Image/CameraInfo/同时间TF及入口位姿');p.add_argument('--save-image',type=Path,help='仅显式请求时保存带中心线的诊断图');
    p.add_argument('--config',type=Path,default=ROOT/'config/runtime.json');p.add_argument('--json',action='store_true')
    p.add_argument('--check',action='store_true');p.add_argument('--simulate',action='store_true');p.add_argument('--worker',action='store_true',help=argparse.SUPPRESS)
    a=p.parse_args(argv)
    try:
        cfg=task_runner.load(a.config);reporting_options(ROOT)
        if a.save_image and not a.entry:raise RuntimeError('--save-image requires --entry')
        profile=None
        if a.entry:
            from entry_alignment import validate
            profile=json.loads((ROOT/'config/entry_alignment.json').read_text(encoding='utf-8'));validate(profile)
        if a.check:print(json.dumps({'status':'scan_config_valid','body_commands':0,'UART_used':False}));return 0
        if a.simulate:result=enrich_for_project({'status':'simulation_only','observations':[],'ids':[]},ROOT)
        elif a.worker:result=worker(cfg,entry=a.entry,save_image=a.save_image)
        else:
            # 外层超时也覆盖ROS节点初始化/环境加载，缺master不会无限卡住。
            for name in SETUPS:
                if not Path(name).is_file():raise RuntimeError('ROS setup missing: '+name)
            args=['python3','-B',str(Path(__file__).resolve()),'--worker','--json','--config',str(a.config.resolve())]
            if a.entry:args+=['--entry']
            if a.save_image:args+=['--save-image',str(a.save_image.expanduser().resolve())]
            command=' && '.join(['. '+shlex.quote(s) for s in SETUPS]+['exec '+' '.join(shlex.quote(s) for s in args)])
            try:completed=subprocess.run(['bash','-c',command],capture_output=True,text=True,timeout=max(cfg['terminal_scan']['timeout_s'],profile['window_timeout_s'] if profile else 0)+6)
            except subprocess.TimeoutExpired:raise RuntimeError('ROS startup/scan exceeded bounded budget; no automatic retry')
            if not completed.stdout.strip():raise RuntimeError('scan worker returned no JSON: '+completed.stderr[-300:])
            result=json.loads(completed.stdout.strip().splitlines()[-1])
        result=dict(result,body_commands=0,UART_used=False,begin_sent=False,scan_only=True)
        if not a.json:
            for line in human_lines(result):print(line)
        print(json.dumps(result,ensure_ascii=False))
        return 0 if result['status'] in ('recognized','simulation_only') else 1
    except (Exception,KeyboardInterrupt) as exc:
        print(json.dumps({'status':'scan_stopped','reason':str(exc),'body_commands':0,'UART_used':False,'automatic_retry':False},ensure_ascii=False),file=sys.stderr);return 1
if __name__=='__main__':raise SystemExit(main())
