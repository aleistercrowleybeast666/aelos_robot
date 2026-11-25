"""兼容原宏调用方式；只有真实构造ROS后端时才导入硬件依赖。"""
import json
import math
from pathlib import Path
import time
import uuid

VERSION='r3-visual-pickup-20261109-v5'
MACROS={'free_forward':('empty','empty'),'grab_single':('empty','single'),
        'carry_forward':('single','single'),'left_turn':('single','single'),
        'right_turn':('single','single'),'carry_forward2':('single','single'),'carry_forward4':('single','single')}


class Stopped(RuntimeError):pass

def scan_module():
    # 按本文件位置加载，兼容工作目录为CMDcontrol_node的现有串口owner。
    import importlib.util
    spec=importlib.util.spec_from_file_location('r3_terminal_scan',Path(__file__).with_name('r3_scan.py'))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def load(path):
    cfg=json.loads(Path(path).read_text(encoding='utf-8'))
    if cfg.get('schema')!='r3.simple-chain.v1' or cfg.get('robot_id')!=3:raise ValueError('Robot3 chain identity required')
    if type(cfg.get('request_timeout_s')) is not int or not 2<=cfg['request_timeout_s']<=120:raise ValueError('request timeout must be integer 2..120')
    if type(cfg.get('max_body_macros')) is not int or not 1<=cfg['max_body_macros']<=80:raise ValueError('macro budget invalid')
    if type(cfg.get('task_timeout_s')) not in (int,float) or not 10<=cfg['task_timeout_s']<=900:raise ValueError('task timeout invalid')
    rows=cfg.get('chain');state='empty';count=0
    if not isinstance(rows,list) or not 1<=len(rows)<=20:raise ValueError('chain length invalid')
    for row in rows:
        if not isinstance(row,dict) or set(row)!={'macro','repeat','pause_s'} or row['macro'] not in MACROS:raise ValueError('unknown macro/schema')
        if type(row['repeat']) is not int or not 1<=row['repeat']<=20:raise ValueError('repeat must be integer 1..20')
        if type(row['pause_s']) not in (float,int) or not math.isfinite(row['pause_s']) or not 0<=row['pause_s']<=10:raise ValueError('pause invalid')
        before,after=MACROS[row['macro']]
        for i in range(row['repeat']):
            if state!=before:raise ValueError('macro/load order mismatch: '+row['macro'])
            state=after;count+=1
    if state!='single' or count>cfg['max_body_macros']:raise ValueError('chain load/budget invalid')
    if 'terminal_scan' in cfg:
        scan_module().validate(cfg['terminal_scan'])
    return cfg


def run(cfg,backend,sleep=time.sleep,clock=time.monotonic,scanner=None):
    deadline=clock()+cfg['task_timeout_s'];events=[]
    backend.call('begin',cfg['request_timeout_s'])
    for row in cfg['chain']:
        for i in range(row['repeat']):
            backend.check()
            remaining=deadline-clock()
            if remaining<2:raise Stopped('chain time budget exhausted')
            timeout=min(cfg['request_timeout_s'],int(remaining))
            backend.call(row['macro'],timeout)
            events.append({'macro':row['macro'],'iteration':i+1,'physical_success':'unobserved'})
            until=clock()+row['pause_s']
            while clock()<until:
                backend.check();sleep(min(.05,max(0,until-clock())))
    options=cfg.get('terminal_scan',{})
    if options.get('enabled'):
        settle=clock()+options['settle_s']
        while clock()<settle:
            backend.check();sleep(min(.05,max(0,settle-clock())))
        remaining=deadline-clock()
        if remaining<=0:result={'status':'timeout','reason':'task budget exhausted before scan','observations':[]}
        else:
            if scanner is None:scanner=getattr(backend,'scan',None)
            if scanner is None:
                scanner=scan_module().scan_tags
            result=scanner(options,backend.check,min(options['timeout_s'],remaining))
        events.append({'stage':1,'terminal_scan_result':result,'stage2_started':False,'stage3_started':False})
        if result.get('status')=='cancelled':backend.cancel();return events
    backend.check();backend.call('finish',cfg['request_timeout_s'])
    return events


def stage_summary(events):
    scan=next((e['terminal_scan_result'] for e in events if 'terminal_scan_result' in e),None)
    if scan is None:return {'status':'chain_finished','events':events}
    status='stage1_complete' if scan.get('status')=='recognized' else 'stage1_ended_'+str(scan.get('status','scan_error'))
    return {'status':status,'stage':1,'scan_result':scan,'events':events,'next_stage_started':False,
        'expected_load':'unknown_after_cancel' if scan.get('status')=='cancelled' else 'single','physical_load_sensor_confirmation':False}


class Simulation:
    is_simulation=True
    def __init__(self):self.calls=[]
    def check(self):pass
    def call(self,op,timeout):self.calls.append(op) # 仅在显式模拟后端记录，不发送真实动作
    def scan(self,options,check,timeout):return {'status':'simulation_only','type':'AR_TAG_ALVAR','observations':[],'ids':[],'reason':'No sensor subscribed during simulation'}
    def cancel(self):pass
    def close(self):pass


class RosMacros:
    """沿用base_action.action；None是正常返回，完成还须检查唯一owner状态。

    本类不直接访问串口或写姿态；有效遥控/取消会阻止后续动作。
    """
    def __init__(self):
        import rospy
        from std_msgs.msg import String
        from leju import base_action
        self.rospy=rospy;self.action=base_action.action;self.row=None;self.epoch=None;self.session=None
        self.sub=rospy.Subscriber('/r3_auto/status',String,self.receive,queue_size=1)

    def receive(self,msg):
        try:self.row=json.loads(msg.data)
        except Exception:self.row=None

    def fresh(self):
        r=self.row
        if not r or r.get('schema')!='r3.simple.v1' or r.get('robot_id')!=3 or r.get('owner_version')!=VERSION or not 0<=time.time()-r.get('published_at_s',0)<=2:
            raise Stopped('matching R3 Pi owner status missing/stale; no command sent')
        if self.session is not None and (r.get('session')!=self.session or r.get('epoch')!=self.epoch):
            raise Stopped('manual takeover/cancel or owner restart; no automatic resume')
        return r

    def wait_ready(self,timeout=4):
        deadline=time.monotonic()+timeout;last=None
        while time.monotonic()<deadline:
            try:
                r=self.fresh()
                if not r.get('mcu_ready'):raise Stopped('matching R3 MCU bridge not observed; install/reload repository overlay first')
                if r.get('mode')!='manual' or r.get('in_flight'):raise Stopped('owner busy or old cancelled native action not drained')
                self.session,self.epoch=r['session'],r['epoch'];return r
            except Stopped as exc:last=exc;time.sleep(.05)
        raise Stopped(str(last))

    def check_manual_idle(self):
        r=self.fresh() # fresh preserves the original session/epoch; never adopts takeover.
        if r.get('mode')!='manual' or r.get('in_flight') or not r.get('mcu_ready'):raise Stopped('manual-idle observation interrupted')

    def check(self):
        r=self.fresh()
        if r.get('mode')!='auto' or not r.get('mcu_ready'):raise Stopped('automatic chain cancelled or MCU unavailable')

    def call(self,op,timeout):
        if op in ('begin','begin_align','stand','capability'):
            if self.session is None:self.wait_ready()
            else:
                r=self.fresh() # 初始stand后不重新绑定取消纪元，防止接管后又begin。
                if not r.get('mcu_ready') or r['mode']!='manual' or r['in_flight']:raise Stopped('matching manual idle required before '+op)
        else:self.check()
        before=self.fresh();sequence=before.get('operation_seq',0);nonce=uuid.uuid4().hex
        request='R3AUTO|{}|{}|{}|{}'.format(self.session,self.epoch,op,nonce)
        started=time.monotonic()
        # 原SDK同步调用正常返回None；完成需结合owner状态，不能只看布尔返回。
        returned=self.action(request,int(timeout))
        if returned is False:raise Stopped('owner explicitly rejected operation '+op+'; no completion accepted')
        elapsed=time.monotonic()-started
        if elapsed>=timeout-.25:raise Stopped('operation {} returned near timeout ({:.3f}s / {}s); completion uncertain; stop remaining macros'.format(op,elapsed,timeout))
        expected='manual' if op in ('finish','stand','capability') else 'auto';deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            row=self.fresh()
            rejection=row.get('last_rejection') or {}
            if rejection.get('request_nonce')==nonce:raise Stopped('owner rejected '+op+': '+str(rejection.get('code')))
            result=row.get('last_result') or {}
            if (row.get('mode')==expected and not row.get('in_flight') and
                    result.get('request_nonce')==nonce and result.get('op')==op and
                    type(result.get('operation_seq')) is int and result['operation_seq']>sequence and
                    result.get('protocol_completed') is True):
                if op in ('capability','single_micro_left','single_micro_right') and (not all(result.get(k) is True for k in ('dispatched','started','complete','drained')) or result.get('cancelled') is not False):raise Stopped('fresh micro START+COMPLETE evidence missing')
                return
            if row.get('mode')=='manual' and op not in ('finish','stand','capability'):raise Stopped('manual mode; remaining macros cancelled')
            time.sleep(.02)
        raise Stopped('owner completion/cancel state did not settle')

    def verify_micro(self,timeout):
        self.check_manual_idle();self.call('capability',timeout)
        if not self.fresh().get('micro_trial',{}).get('ready'):raise Stopped('micro capability/recovery/guarded trial not ready')

    def check_micro(self):
        self.check()
        if not self.fresh().get('micro_trial',{}).get('ready'):raise Stopped('micro current session capability or physical inspection expired')

    def cancel(self):
        try:
            self.fresh()
            if self.session is not None:self.action('R3AUTO|{}|{}|cancel'.format(self.session,self.epoch),1)
        except Exception:pass # 取消纪元已阻止迟到请求，不能借异常自动重启

    def close(self):self.sub.unregister()
