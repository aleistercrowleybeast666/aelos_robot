#!/usr/bin/env python3
import argparse,json,re,sys
from pathlib import Path
from r3_chain import Simulation,RosMacros
ROOT=Path(__file__).resolve().parent

def identity():
    row=json.loads((ROOT/'deployment.json').read_text(encoding='utf-8'))
    cpu=Path('/proc/cpuinfo').read_text();m=re.search(r'^Serial\s*:\s*(\S+)',cpu,re.M)
    if row.get('robot_id')!=3 or not m or row.get('cpu_serial')!=m[1] or row.get('machine_id')!=Path('/etc/machine-id').read_text().strip():raise RuntimeError('Robot3 CPU+machine identity mismatch')

def main(argv=None):
    # The warm SSH client imports identity() for every check and start.
    # Only the standalone planner needs to load the route engine here.
    import task_runner
    p=argparse.ArgumentParser(description='R3区域路线：0到障碍区前扫码；stand松手恢复空载站姿；status只读诊断；1/2缺项时动作前拒绝。')
    p.add_argument('mode',nargs='?',choices=['0','1','2','0-full','stage1','stand','stop','status','scan',*task_runner.CALIBRATION_STEPS,*task_runner.TURN_TEST_STEPS,'cooling-check','cooling-confirm'],default='0');p.add_argument('legacy_after_home',nargs='?',choices=['0'])
    p.add_argument('--config',type=Path,default=ROOT/'config/runtime.json');p.add_argument('--json',action='store_true',help='scan仅输出JSON');p.add_argument('--entry',action='store_true',help='scan采集入口坐标');p.add_argument('--save-image',type=Path);p.add_argument('--one-step',action='store_true',help='明确测试一次完整离散动作，不是精确微小步')
    from transfer_test import BOUNDARIES
    p.add_argument('--stop-after',choices=BOUNDARIES)
    p.add_argument('--drop-yaw-deg', type=int, help='进大本营后放物前的转角，负左正右，范围 −30 至 +30 度')
    g=p.add_mutually_exclusive_group();g.add_argument('--check',action='store_true');g.add_argument('--simulate',action='store_true');g.add_argument('--device-check',action='store_true')
    a=p.parse_args(argv);backend=None;lock=None
    if a.drop_yaw_deg is not None:
        from base_placement import BasePlacement_ValidateYaw
        try:
            BasePlacement_ValidateYaw(a.mode, a.drop_yaw_deg)
        except ValueError as exc:
            p.error(str(exc))
    try:
        if a.mode in ('cooling-check','cooling-confirm'):
            from warm_client import main as warm
            return warm([a.mode]+(['--check'] if a.check else ['--simulate'] if a.simulate else []))
        if a.mode=='scan':
            if a.device_check:raise RuntimeError('scan does not use MCU device-check')
            from scan_cli import main as scan
            return scan((['--config',str(a.config)])+(['--check'] if a.check else ['--simulate'] if a.simulate else [])+(['--json'] if a.json else [])+(['--entry'] if a.entry else [])+(['--save-image',str(a.save_image)] if a.save_image else []))
        if a.json or a.entry or a.save_image:raise RuntimeError('--json/--entry/--save-image only for scan')
        if a.mode in ('stop','status','scan'):
            if a.check or a.simulate:print(json.dumps({'status':a.mode+'_syntax_valid','body_commands':0}));return 0
            from warm_client import main as warm
            return warm([a.mode])
        if a.stop_after and a.mode not in ('0','stage1'):
            raise RuntimeError('--stop-after only applies to command 0/stage1; never silently ignore a requested stop boundary')
        cfg=task_runner.load(a.config)
        if a.drop_yaw_deg is not None:cfg=dict(cfg,drop_yaw_deg=a.drop_yaw_deg)
        if a.stop_after:cfg=dict(cfg,transfer_test=dict(cfg.get('transfer_test',{}),stop_after=a.stop_after))
        plan=task_runner.prepare(a.mode,cfg,ROOT,simulation=a.simulate) # 在创建SDK/ROS/硬件接口之前检查
        if a.check:
            from route_status import inspect_project
            checks={'status':'config_and_route_ready','command':a.mode,'body_commands':0,
                'navigation_readiness':inspect_project(ROOT,a.mode,plan.get('navigation',{'enabled':False}))}
            if 'route_steps' in plan['config']:checks['route_steps']=plan['config']['route_steps']
            print(json.dumps(checks));return 0
        if a.simulate and a.mode in ('1','2'):
            from outbound_pickup_trial import OutboundPickup_Simulate
            print(json.dumps(OutboundPickup_Simulate(plan)));return 0
        if a.simulate and a.mode=='0-full':
            from simulate_v5 import run
            print(json.dumps(run(plan)));return 0
        if a.simulate:
            if a.mode not in ('0','stage1','stand') and a.mode not in task_runner.CALIBRATION_STEPS and a.mode not in task_runner.TURN_TEST_STEPS:
                print(json.dumps({'status':'function_plan_only','command':a.mode,'body_commands':0,'continuous':a.mode=='2','executed':False}));return 0
            backend=Simulation();result=task_runner.execute(plan,backend)
            print(json.dumps({'status':'simulation_only','calls':backend.calls,'result':result,'body_commands':0}));return 0
        if not a.device_check:
            raise RuntimeError('Direct ROS body execution is disabled in this candidate. Use aelos-auto-r3 / warm_client.py with the current-session recovery gate; no body backend constructed.')
        if a.mode in task_runner.CALIBRATION_STEPS and not a.one_step:raise RuntimeError('calibration movement requires explicit --one-step; do not start whole route')
        identity()
        import rospy
        sys.path.insert(0,'/home/lemon/catkin_ws/src/aelos_smart_ros')
        rospy.init_node('r3_function_tasks',anonymous=True,disable_signals=True)
        backend=RosMacros()
        if a.device_check:
            state=backend.wait_ready();print(json.dumps({'status':'matching_idle_bridge','owner':state,'body_commands':0}));return 0
        import fcntl
        state=Path.home()/'.local/state/r3_simple';state.mkdir(parents=True,exist_ok=True)
        lock=(state/'run.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        result=task_runner.execute(plan,backend);print(json.dumps(result));return 0 if result['status'] in ('task_complete','stage1_complete','stage1_complete_blind','stage2_checkpoint_reached','stage3_checkpoint_reached','stand_request_completed','one_calibration_step_request_completed','transfer_boundary_complete') else 1
    except (Exception,KeyboardInterrupt) as exc:
        if backend:backend.cancel()
        result={'status':'not_ready_or_stopped','reason':str(exc),'native_immediate_stop':False}
        result.update(getattr(exc,'error_details',{}))
        print(json.dumps(result),file=sys.stderr);return 1
    finally:
        if backend:backend.close()
        if lock:lock.close()
if __name__=='__main__':raise SystemExit(main())
