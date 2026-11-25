#!/usr/bin/env python3
"""轻量本机命令入口：热路径仅用标准库，不重复初始化ROS/SDK。"""
import argparse
import json
from pathlib import Path
import socket
import sys
import time

PROTOCOL='r3.warm.v1'
ROOT=Path(__file__).resolve().parent
SOCKET=Path('/home/lemon/.local/state/r3_simple/control.sock')

def main(argv=None,connector=None):
    p=argparse.ArgumentParser(description='R3预热入口：0到障碍区前扫码；stand松手恢复空载站姿；status只读诊断；stop取消余链。')
    p.add_argument('mode',nargs='?',choices=['0','1','2','0-full','stage1','stand','stop','status','scan','test-left','test-right','test-small-left','test-small-right','test-turn-left','test-turn-right','test-micro-left','test-micro-right','test-medium-left','test-medium-right','cooling-check','cooling-confirm','micro-confirm','readiness'],default='0');p.add_argument('legacy_after_home',nargs='?',choices=['0'])
    p.add_argument('--config',type=Path);p.add_argument('--json',action='store_true',help='scan仅输出JSON');p.add_argument('--entry',action='store_true',help='scan采集入口坐标');p.add_argument('--save-image',type=Path);p.add_argument('--one-step',action='store_true',help='明确测试一次完整离散动作，不是精确微小步')
    p.add_argument('--idle-only',action='store_true',help='stand 只在空闲时执行')
    from transfer_test import BOUNDARIES
    p.add_argument('--stop-after',choices=BOUNDARIES)
    p.add_argument('--owner-session');p.add_argument('--owner-epoch',type=int);p.add_argument('--operator')
    for flag in ('cooled','mechanism-checked','single-target','guard-reload-checked','guarded-single-trial','stop-limits-understood'):p.add_argument('--'+flag,action='store_true')
    g=p.add_mutually_exclusive_group();g.add_argument('--check',action='store_true');g.add_argument('--simulate',action='store_true');g.add_argument('--device-check',action='store_true')
    a=p.parse_args(argv)
    if a.idle_only and (a.mode!='stand' or a.check or a.simulate or a.device_check):
        p.error('--idle-only 只用于实机 stand')
    if a.mode=='scan':
        if a.device_check:p.error('scan不使用MCU device-check')
        from scan_cli import main as scan
        return scan((['--config',str(a.config)] if a.config else [])+(['--check'] if a.check else ['--simulate'] if a.simulate else [])+(['--json'] if a.json else [])+(['--entry'] if a.entry else [])+(['--save-image',str(a.save_image)] if a.save_image else []))
    if a.json or a.entry or a.save_image:p.error('--json/--entry/--save-image只用于scan')
    if a.mode in ('cooling-check','cooling-confirm','micro-confirm') and (a.check or a.simulate):
        print(json.dumps({'status':'recovery_command_syntax_only','body_commands':0,'confirmation_recorded':False}));return 0
    if a.check or a.simulate:
        from run_auto import main as offline
        args=[a.mode]+([a.legacy_after_home] if a.legacy_after_home else [])+(['--check'] if a.check else ['--simulate'])
        if a.config:args+=['--config',str(a.config)]
        if a.stop_after:args+=['--stop-after',a.stop_after]
        return offline(args)
    conn=None;stream=None;accepted=False;last_status=None;started=time.monotonic()
    try:
        from run_auto import identity
        identity()
        conn=connector() if connector else socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
        conn.settimeout(5)
        if connector is None:conn.connect(str(SOCKET))
        request={'protocol':PROTOCOL,'op':'status' if a.device_check or a.mode in ('status','cooling-check','readiness') else ('stop' if a.mode=='stop' else 'start'),'mode':a.mode,'legacy_after_home':a.legacy_after_home}
        if a.idle_only:request['idle_only']=True
        if a.mode=='cooling-confirm':
            if not a.owner_session or a.owner_epoch is None or not a.operator or not all((a.cooled,a.mechanism_checked,a.single_target,a.guard_reload_checked)):
                raise RuntimeError('explicit current session/epoch, named operator and all four physical inspection confirmations required; no command sent')
            request['op']='confirm_recovery'
            request['recovery']={'session':a.owner_session,'epoch':a.owner_epoch,'operator':a.operator,
                'cooled':a.cooled,'mechanism_checked':a.mechanism_checked,'single_target':a.single_target,'guard_reload_checked':a.guard_reload_checked}
        if a.mode=='micro-confirm':
            if not a.owner_session or a.owner_epoch is None or not a.operator or not a.guarded_single_trial or not a.stop_limits_understood:
                raise RuntimeError('explicit current session/epoch, operator, guarded single trial and stop limits required; no command sent')
            request['op']='confirm_micro'
            request['micro_trial']={'session':a.owner_session,'epoch':a.owner_epoch,'operator':a.operator,'guarded_single_trial':True,'stop_limits_understood':True}
        if a.stop_after:
            if a.mode not in ('0','stage1'):raise RuntimeError('--stop-after only applies to command 0/stage1')
            request['stop_after']=a.stop_after
        if a.mode.startswith('test-'):
            if not a.one_step:raise RuntimeError('explicit --one-step required for calibration movement')
            request['one_step_confirmed']=True
        if request['op']=='start':request['expected_owner_version']='r3-visual-pickup-20261109-v5'
        if a.config:request['config']=str(a.config)
        conn.sendall((json.dumps(request)+'\n').encode())
        stream=conn.makefile('r',encoding='utf-8')
        for line in stream:
            row=json.loads(line);event=row.get('event')
            if event=='accepted':
                conn.settimeout(None)
                accepted=True;print('R3 command accepted; Ctrl+C cancels remaining actions. '+str(row.get('command','legacy')),flush=True)
            elif event=='status':
                last_status=row
                if row.get('protocol')!=PROTOCOL:raise RuntimeError('unknown status protocol')
                if a.mode in ('status','cooling-check','readiness') and not a.device_check:
                    if a.mode=='readiness':
                        from micro_readiness import report
                        row=report(ROOT,row)
                    print(json.dumps(dict(row,body_commands=0,read_only=True)));return 0
                r=row.get('owner',{})
                if row.get('protocol')!=PROTOCOL or not row.get('prepared') or row.get('task_active') or r.get('robot_id')!=3 or r.get('owner_version')!='r3-visual-pickup-20261109-v5' or not r.get('mcu_ready') or r.get('mode')!='manual' or r.get('in_flight') or not 0<=time.time()-r.get('published_at_s',0)<=2:raise RuntimeError('warm owner/MCU is not ready and idle')
                print(json.dumps(dict(row,body_commands=0,client_elapsed_s=time.monotonic()-started)));return 0
            elif event in ('recovery_confirmed','micro_confirmed'):
                print(json.dumps(row));return 0
            elif event=='done':
                if row.get('status')=='single_medium_response_observed':
                    print('中转实测 · 磁力计 {:+.1f}° · Tag {:+.1f}°'.format(
                        row['magnetic_delta_deg'],row['visual_delta_deg']),flush=True)
                if 'scan_result' in row:
                    from tag_reporting import human_lines
                    for line in human_lines(row['scan_result']):print(line,flush=True)
                    y=row['scan_result'].get('pixel_y_diagnostic')
                    if y:
                        for sample in y.get('samples',[]):
                            print('图像y诊断（非到位判据）：'+json.dumps(sample,ensure_ascii=False),flush=True)
                        if not y.get('samples'):print('图像y诊断不可用：'+str(y.get('diagnostic_reasons',[]))+'；不据此移动。',flush=True)
                if 'entry_alignment' in row:
                    arow=row['entry_alignment'];print(('入口视觉航向：' if arow.get('visual_micro') else '入口示教校正：')+str(arow.get('status'))+'；'+str(arow.get('reason_zh') or arow.get('reason','')),flush=True)
                    if arow.get('history'):
                        last=arow['history'][-1];print('示教参考：x偏差={:.4f}m，相对角度={:.2f}deg；y参考={:.4f}m，前向偏移={:.4f}m（仅监测）。'.format(last['x_m'],last['angle_deg'],last['reference_tag_y_m'],last['y_offset_m']),flush=True)
                if 'next_segment' in row:
                    segment=row['next_segment']
                    if segment['detected']:
                        print('已扫到 Tag 2，入口试跑段完成。',flush=True)
                    elif not segment.get('continue_estimated'):
                        print('Tag 2 试跑待确认。',flush=True)
                if 'corridor_trial' in row:
                    corridor=row['corridor_trial']
                    if corridor.get('release_protocol_completed'):
                        delivery=corridor.get('delivery',{})
                        steps=delivery.get('approach_steps')
                        advance='持物前进 {} 步'.format(steps) if type(steps) is int else '持物前进'
                        operation=delivery.get('release_operation')
                        pair=(operation=='place_double' if operation else row.get('pickup',{}).get('grip_count')==2)
                        release='双抱放物并回站姿' if pair else '单块放物'
                        print(f'已扫到 Tag 5，{advance}和{release}动作已完成。',flush=True)
                    elif corridor['detected']:
                        print('已扫到 Tag 5，当前保持持物。',flush=True)
                    else:
                        print('通道试跑待确认。最近确认的码为 Tag {}。'.format(corridor['checkpoint']),flush=True)
                if row.get('status')=='return_trial_unconfirmed':
                    checkpoint=row.get('checkpoint')
                    message='返程待确认。'
                    if checkpoint is not None:message+='最近确认 Tag {}。'.format(checkpoint)
                    print(message,flush=True)
                print(json.dumps(row));return 0 if row.get('status') in ('task_complete','chain_finished','stage1_complete','stage1_complete_blind','stage2_checkpoint_reached','stage3_checkpoint_reached','stand_request_completed','one_calibration_step_request_completed','single_micro_response_observed','single_medium_response_observed','transfer_boundary_complete') else 1
            elif event=='stopped':print(json.dumps(row));return 0
            elif event in ('error','cancelled'):
                last_status=row
                raise RuntimeError(row.get('reason',event))
            else:raise RuntimeError('unknown local owner response')
        raise RuntimeError('local owner disconnected; no automatic resume')
    except KeyboardInterrupt:
        if conn:
            try:conn.sendall(b'{"op":"cancel"}\n')
            except OSError:pass
        print('Remaining automatic macros cancelled. In-flight native/grab may continue; immediate stop and physical safety are NOT guaranteed. This is not an emergency stop.',file=sys.stderr);return 130
    except Exception as exc:
        print(json.dumps({'status':'not_started_or_stopped','reason':str(exc),'accepted':accepted,'automatic_retry':False,'observed_status':last_status}),file=sys.stderr);return 1
    finally:
        if stream:stream.close()
        if conn:conn.close()

if __name__=='__main__':raise SystemExit(main())
