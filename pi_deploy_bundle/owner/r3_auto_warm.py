"""在原UART owner内预热和接收本机触发，不增加独立守护进程。

启动只加载代码、参数、身份并开放本地socket；不执行stand/begin或恢复旧任务。
每次运动必须由用户显式发新命令，当前任务持有独立参数/源码快照。
"""
import importlib.util
import json
import os
from pathlib import Path
import select
import sys
import socket
import stat
import struct
import tempfile
import threading
import time
import uuid

from r3_micro_gate import (CMD0_ROUTE_PROBE, EMPTY_RECOVERY_PROBE, MEDIUM_TURN_PROBE, MicroGate_IsRobot3Identity, PAIR_PROBE, PICKUP_BACKWARD4_PROBE,
                           PICKUP_BACKWARD6_OPS, PROBE_OPS, RIGHT5_ROUTE_PROBE, SMALL_PROFILE_PROBE, SMALL_STRAFE_PROBE)

PROTOCOL='r3.warm.v1'
DEFAULT_PROJECT=Path('/home/lemon/aelos_auto_r3_python')
DEFAULT_STATE=Path('/home/lemon/.local/state/r3_simple')
MCU_IDLE_CONTACT_GRACE_S=1.25
PAIR_PROBE_TIMEOUT_S=4.


class EmptyRecoveryUnavailable(RuntimeError):
    """Unknown optional alias returned ERROR before START and then ordered idle."""


class PairProbeUnavailable(RuntimeError):
    """The read-only probe returned to idle without a completion receipt."""

def effective_uid():return os.geteuid() if hasattr(os,'geteuid') else 1000

def exclusive_lock(handle):
    if os.name=='nt': # 只用于Windows离线验证环境
        import msvcrt
        if os.fstat(handle.fileno()).st_size==0:handle.write('\0');handle.flush()
        handle.seek(0);msvcrt.locking(handle.fileno(),msvcrt.LK_NBLCK,1)
    else:
        import fcntl
        fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)


def _write_runtime_report(project, result):
    reports=project/'field_reports'/'runtime'
    reports.mkdir(parents=True,exist_ok=True)
    name=time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8]+'.json'
    pending=None
    try:
        with tempfile.NamedTemporaryFile(mode='w',encoding='utf-8',dir=reports,
                                         prefix='.'+name+'.',suffix='.tmp',delete=False) as output:
            pending=Path(output.name)
            json.dump(result,output,ensure_ascii=False,indent=2)
            output.flush()
            os.fsync(output.fileno())
        os.replace(pending,reports/name)
        pending=None
        try:
            directory=os.open(reports,os.O_RDONLY|getattr(os,'O_DIRECTORY',0))
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
        except OSError:
            pass  # Some filesystems and Windows do not support directory fsync.
    finally:
        if pending is not None:
            try:
                pending.unlink()
            except OSError:
                pass


class GateBackend:
    def __init__(self,gate):
        self.gate=gate;self.session=None;self.epoch=None
        self.cancelled=threading.Event();self.progress=None
        self.empty_recovery_attempt=None
    def snapshot(self, *, require_mcu=True):
        r=self.gate.status()
        if r.get('robot_id')!=3 or r.get('owner_version')!='r3-visual-pickup-20261109-v5':raise RuntimeError('R3 owner mismatch')
        if not 0<=time.time()-r.get('published_at_s',0)<=2:raise RuntimeError('owner state missing/stale')
        if require_mcu and not r.get('mcu_ready'):raise RuntimeError('MCU handshake missing/stale')
        return r
    def ready(self):
        r=self.snapshot()
        if r.get('mode')!='manual' or r.get('in_flight'):raise RuntimeError('manual idle required; native action may still be in flight')
        self.session,self.epoch=r['session'],r['epoch'];return r
    def current(self, *, require_mcu=True):
        if self.cancelled.is_set():raise RuntimeError('local task cancelled; no automatic resume')
        r=self.snapshot(require_mcu=require_mcu)
        if r['session']!=self.session or r['epoch']!=self.epoch:raise RuntimeError('manual takeover/cancel; no automatic resume')
        return r
    def check_manual_idle(self):
        r=self.current() # keep the original session and cancellation epoch
        if r['mode']!='manual' or r.get('in_flight'):raise RuntimeError('manual-idle observation interrupted')
    def check(self):
        state=self.current(require_mcu=False)
        if state['mode']!='auto':raise RuntimeError('automatic mode cancelled')
        if not state.get('mcu_ready'):
            self._wait_for_idle_mcu_request(state)
    def _wait_for_idle_mcu_request(self,state):
        # Only a drained action boundary can wait for the next native REQ.
        sequence=state['operation_seq']
        deadline=time.monotonic()+MCU_IDLE_CONTACT_GRACE_S
        while True:
            last=state.get('last_result') or {}
            boundary=(state['mode']=='auto' and state.get('in_flight') is False
                      and state['operation_seq']==sequence and last.get('operation_seq')==sequence
                      and last.get('cancelled') is False
                      and all(last.get(key) is True for key in
                              ('protocol_completed','dispatched','started','complete','drained')))
            if not boundary:
                raise RuntimeError('MCU handshake missing/stale')
            if state.get('mcu_ready') and state.get('mcu_last_contact_source')=='request':
                return
            remaining=deadline-time.monotonic()
            if remaining<=0:
                raise RuntimeError('MCU handshake missing/stale')
            self.cancelled.wait(min(0.05,remaining))
            state=self.current(require_mcu=False)
    def call(self,op,timeout):
        manual_only=op in ('begin','begin_align','stand') or (op in PROBE_OPS and op != EMPTY_RECOVERY_PROBE)
        if manual_only:
            if self.session is None:self.ready()
            else:
                r=self.current()
                if r['mode']!='manual' or r['in_flight']:raise RuntimeError('owner became busy before begin')
        else:self.check()
        if op in ('begin', 'begin_align'):
            self._verify_small_strafe_profile(timeout)
        before=self.current(require_mcu=manual_only);sequence=before.get('operation_seq',0);nonce=uuid.uuid4().hex
        text='R3AUTO|{}|{}|{}|{}'.format(self.session,self.epoch,op,nonce)
        if op=='grab_single' and getattr(self,'usage_file',None) is not None:
            self.usage_file.write_text(json.dumps(dict(boot_id=self.gate.micro.boot_id,session=self.session,attempted=True,evidence='conservatively consumed before grab submission; not a physical success claim')))
        ok=self.gate.submit(text,timeout)
        if not ok and not manual_only:
            detail=self.gate.status();rejection=detail.get('last_rejection') or {}
            if (detail.get('session')==self.session and detail.get('epoch')==self.epoch
                    and detail.get('operation_seq')==sequence and detail.get('in_flight') is False
                    and rejection.get('request_nonce')==nonce and rejection.get('op')==op
                    and rejection.get('code')=='MCU_handshake_missing_or_stale'):
                # The first request was rejected before queuing. Only this same nonce can be retried.
                self._wait_for_idle_mcu_request(detail)
                ok=self.gate.submit(text,timeout)
        self.gate.publish()
        if not ok:
            detail=self.gate.status()
            result=detail.get('last_result') or {}
            if (op in PICKUP_BACKWARD6_OPS and result.get('op')==op and result.get('request_nonce')==nonce
                    and result.get('retreat_unavailable') is True and result.get('started') is False
                    and result.get('cancelled') is False and result.get('drained') is True
                    and not detail.get('in_flight')):
                self.check()
                from cmd0_motion import PickupRetreatUnavailable
                raise PickupRetreatUnavailable('当前 MCU 未注册连续后退六步动作')
            if (op in (PAIR_PROBE, EMPTY_RECOVERY_PROBE, SMALL_PROFILE_PROBE) and result.get('op')==op and result.get('request_nonce')==nonce
                    and result.get('probe_unavailable') is True and result.get('drained') is True
                    and result.get('started') is False and result.get('cancelled') is False
                    and not detail.get('in_flight')):
                self.current()
                if op == EMPTY_RECOVERY_PROBE:
                    raise EmptyRecoveryUnavailable('MCU 未提供空载回程微调能力')
                if op == SMALL_PROFILE_PROBE:
                    return  # The ordered idle receipt selected the legacy gait.
                raise PairProbeUnavailable('MCU 未确认双抱能力')
            raise RuntimeError('operation {} failed; reason={}; last_result={}; last_rejection={}; in_flight={}'.format(op,detail.get('reason'),detail.get('last_result'),detail.get('last_rejection'),detail.get('in_flight')))
        r=self.current(require_mcu=False);expected='manual' if op in ('finish','stand') or (op in PROBE_OPS and op != EMPTY_RECOVERY_PROBE) else 'auto'
        result=r.get('last_result') or {}
        if (result.get('request_nonce')!=nonce or result.get('op')!=op or
                type(result.get('operation_seq')) is not int or result['operation_seq']<=sequence or
                result.get('protocol_completed') is not True or
                not all(result.get(k) is True for k in ('dispatched','started','complete','drained')) or
                result.get('cancelled') is not False):raise RuntimeError('fresh nonce/sequence START+COMPLETE evidence missing')
        if r['mode']!=expected or r['in_flight']:raise RuntimeError('operation state did not settle')
    def verify_micro(self,timeout):
        self.check_manual_idle()
        micro=self.current().get('micro_trial',{})
        if micro.get('pickup_backward4_deployment_readback_ready') is True:
            probe=PICKUP_BACKWARD4_PROBE
        elif micro.get('pair_deployment_readback_ready') is True:
            probe=PAIR_PROBE
        elif micro.get('cmd0_right5_deployment_readback_ready') is True:
            probe=RIGHT5_ROUTE_PROBE
        elif micro.get('medium_turn_deployment_readback_ready') is True:
            probe=MEDIUM_TURN_PROBE
        elif micro.get('cmd0_route_deployment_readback_ready') is True:
            probe=CMD0_ROUTE_PROBE
        elif micro.get('small_strafe_deployment_readback_ready') is True:
            probe=SMALL_STRAFE_PROBE
        else:
            probe='capability'
        self.call(probe,timeout)
        state=self.current()
        if not state.get('micro_trial',{}).get('ready'):
            raise RuntimeError('micro capability/guarded trial/recovery not ready BEFORE cleanup: '+json.dumps({k:state.get(k) for k in ('session','epoch','recovery_gate','micro_trial')}))
    def small_strafe_profile(self):
        return self.current(require_mcu=False).get('micro_trial', {}).get('small_strafe_profile')

    def _verify_small_strafe_profile(self,timeout):
        micro = self.current().get('micro_trial',{})
        if ('small_strafe_profile' in micro and micro['small_strafe_profile'] is None
                and micro.get('small_strafe_ready') is True):
            self.call(SMALL_PROFILE_PROBE, min(timeout,PAIR_PROBE_TIMEOUT_S))

    def verify_mission_v5(self,timeout):
        self.verify_micro(timeout)
        if self.current().get('mission_v5_ready') is not True:
            raise RuntimeError('V5 MCU ABI/probe/readback/repeated-single gate not ready BEFORE motion')
    def verify_pickup_capabilities(self,timeout):
        self.check_manual_idle()
        micro=self.current()['micro_trial']
        if (micro['cmd0_route_deployment_readback_ready']
                and not micro['pair_deployment_readback_ready']):
            try:
                self.call(PAIR_PROBE,min(timeout,PAIR_PROBE_TIMEOUT_S))
            except PairProbeUnavailable:
                pass
            else:
                if self.current().get('mission_v5_pair_ready') is True:
                    return
        self.verify_cmd0_continuous(timeout)
    def verify_small_strafe(self,timeout):
        self.check_manual_idle()
        if self.current().get('micro_trial',{}).get('small_strafe_deployment_readback_ready') is not True:
            raise RuntimeError('small strafe MCU installation not verified BEFORE motion')
        self.verify_micro(timeout)
        if self.current().get('micro_trial',{}).get('small_strafe_ready') is not True:
            raise RuntimeError('small strafe current-session capability not verified BEFORE motion')
    def verify_medium_turn(self,timeout):
        self.check_manual_idle()
        if self.current().get('micro_trial',{}).get('medium_turn_deployment_readback_ready') is not True:
            raise RuntimeError('medium turn MCU installation not verified BEFORE motion')
        self.verify_micro(timeout)
        if self.current().get('micro_trial',{}).get('medium_turn_ready') is not True:
            raise RuntimeError('medium turn current-session capability not verified BEFORE motion')
    def verify_cmd0_continuous(self,timeout):
        self.check_manual_idle()
        if self.current().get('micro_trial',{}).get('cmd0_route_deployment_readback_ready') is not True:
            raise RuntimeError('continuous route MCU installation not verified BEFORE motion')
        self.verify_micro(timeout)
        if self.current().get('micro_trial',{}).get('cmd0_route_ready') is not True:
            raise RuntimeError('continuous route current-session capability not verified BEFORE motion')
    def check_mission_v5(self):
        self.check()
        if self.current().get('mission_v5_ready') is not True:
            raise RuntimeError('V5 MCU evidence/permit expired')
    def mission_pair_ops(self):
        state=self.current(require_mcu=False)
        if state.get('mission_v5_pair_ready') is not True:
            return frozenset()
        return frozenset(state.get('mission_v5_pair_ops',()))
    def report_progress(self,progress):
        self.progress=dict(progress)
    def check_micro(self):
        self.check()
        if not self.current().get('micro_trial',{}).get('ready'):raise RuntimeError('micro current session capability or physical inspection expired')
    def small_strafe_ready(self):
        state=self.current()
        return state.get('mode')=='auto' and state.get('micro_trial',{}).get('small_strafe_ready') is True
    def cmd0_continuous_ready(self):
        state=self.current()
        return state.get('mode')=='auto' and state.get('micro_trial',{}).get('cmd0_route_ready') is True
    def empty_recovery_ready(self):
        state = self.current(require_mcu=False)
        return state['mode'] == 'auto' and state.get('micro_trial', {}).get('empty_recovery_ready') is True

    def ensure_empty_recovery(self):
        if self.empty_recovery_ready():
            return True
        state = self.current(require_mcu=False)
        if (state['mode'] != 'auto' or state.get('mission_v5_ready') is not True
                or 'empty_recovery_ready' not in state.get('micro_trial', {})):
            return False
        scope = (self.session, self.epoch)
        if self.empty_recovery_attempt == scope:
            return False
        self.empty_recovery_attempt = scope
        try:
            self.call(EMPTY_RECOVERY_PROBE, PAIR_PROBE_TIMEOUT_S)
        except EmptyRecoveryUnavailable:
            return False
        return self.empty_recovery_ready()

    def medium_turn_ready(self):
        state=self.current()
        return state.get('mode')=='auto' and state.get('micro_trial',{}).get('medium_turn_ready') is True
    def cmd0_right5_ready(self):
        state=self.current()
        return state.get('mode')=='auto' and state.get('micro_trial',{}).get('cmd0_right5_ready') is True
    def _pickup_backward_ready(self, name):
        state=self.current(require_mcu=False)
        if state.get('mode')!='auto':return False
        if not state.get('mcu_ready'):
            self._wait_for_idle_mcu_request(state)
            state=self.current()
        return state.get('micro_trial',{}).get(name) is True
    def pickup_backward4_ready(self):
        return self._pickup_backward_ready('pickup_backward4_ready')
    def pickup_backward6_ready(self):
        return self._pickup_backward_ready('pickup_backward6_ready')
    def Pickup_CanTryBackward6(self):
        state=self.current(require_mcu=False)
        return state.get('mode')=='auto' and state.get('mission_v5_pair_ready') is True
    def pickup_backward8_ready(self):
        return self._pickup_backward_ready('pickup_backward8_ready')
    def cmd0_legacy_ready(self):
        state=self.current()
        return state.get('mode')=='auto' and state.get('mission_v5_ready') is True
    def cancel(self):
        self.cancelled.set()
        self.gate.cancel('local trigger/client cancelled');self.gate.publish()


class WarmController:
    def __init__(self,gate,project=DEFAULT_PROJECT,state=DEFAULT_STATE,identity=None,listener_factory=None):
        self.gate=gate;self.project=Path(project);self.state=Path(state)
        self.identity=identity or self.check_identity
        self.listener_factory=listener_factory
        self.busy=threading.Lock();self.closed=threading.Event();self.listener=None;self.lockfile=None
        self.threads=[];self.active=None;self.prepared=False;self.prepare_error=None
        self.navigation_cache=None;self.navigation_cache_error=None
        self.socket_path=self.state/'control.sock'
        started=time.monotonic()
        try:
            self.identity()
            # 固定工程经身份核对后作为模块搜索目录，避免owner工作目录下找不到新助手模块。
            if str(self.project) not in sys.path:sys.path.append(str(self.project))
            spec=importlib.util.spec_from_file_location('r3_warmed_tasks',self.project/'task_runner.py')
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
            self.engine=module
            runtime_cfg=self.engine.load(self.project/'config/runtime.json')
            micro_record=self.project/'config/micro_mcu_deployment.json'
            self.gate.micro.configure(json.loads(micro_record.read_text(encoding='utf-8')) if micro_record.exists() else None)
            self.prepared=True
            self.prewarm_navigation(runtime_cfg)
        except Exception as exc:self.prepare_error=str(exc)
        self.prepare_s=time.monotonic()-started

    def prewarm_navigation(self,runtime_cfg):
        if runtime_cfg.get('persistent_navigation_cache') is not True:return
        try:
            import rospy
            if not rospy.core.is_initialized():raise RuntimeError('ROS not initialized; sensor prewarm deferred')
            from navigation_cache import PersistentNavigationCache
            from nav_ros import RosNavigationIO
            from checkpoint_nav import load as load_navigation
            cfg=load_navigation(self.project)
            if not cfg.get('enabled'):return
            if self.navigation_cache is None:self.navigation_cache=PersistentNavigationCache(RosNavigationIO)
            self.navigation_cache.prewarm(cfg);self.navigation_cache_error=None
        except Exception as exc:
            self.navigation_cache_error=str(exc) # Not a sensor-ready claim; explicit start retries.

    def check_identity(self):
        import re
        record=json.loads((self.project/'deployment.json').read_text(encoding='utf-8'))
        cpu=re.search(r'^Serial\s*:\s*(\S+)',Path('/proc/cpuinfo').read_text(),re.M)
        machine_id=Path('/etc/machine-id').read_text().strip()
        if (record.get('robot_id')!=3 or not cpu or record.get('cpu_serial')!=cpu[1]
                or record.get('machine_id')!=machine_id or not MicroGate_IsRobot3Identity(cpu[1],machine_id)):
            raise RuntimeError('Robot3 CPU+machine identity mismatch')

    def status(self):
        active=self.active
        progress=getattr(active.get('backend'),'progress',None) if active else None
        return {'protocol':PROTOCOL,'prepared':self.prepared,'prepare_error':self.prepare_error,
            'prepare_s':self.prepare_s,'navigation_cache_prewarm_error':self.navigation_cache_error,'navigation_cache':self.navigation_cache.status() if self.navigation_cache else {'subscriptions_prepared':False,'error':self.navigation_cache_error,'sensor_health_verified':False},'task_active':self.busy.locked(),'mission_progress':progress,'owner':self.gate.status()}

    def config(self,value=None):
        path=(self.project/'config/runtime.json') if value is None else Path(value)
        if not path.is_absolute():path=self.project/path
        path=path.resolve();root=(self.project/'config').resolve()
        if path.parent!=root or path.suffix!='.json':raise RuntimeError('config must be a JSON file directly in project/config')
        return self.engine.load(path)

    def start_job(self,config=None,mode='stage1',stop_after=None,*,idle_only=False):
        if not self.prepared:raise RuntimeError('owner preparation failed: '+str(self.prepare_error))
        if mode=='stand' and (self.busy.locked() or self.gate.status().get('mode')=='auto'):
            if idle_only:raise RuntimeError('empty reset requires idle; no active task cancelled')
            # 只取消余链；绝不把站姿排在未知的在途动作后自动执行。
            active=self.active
            if active:self.cancel_job(active)
            else:self.gate.cancel('stand requested: cancel existing auto before retry')
            raise RuntimeError('stand cancelled remaining auto task; wait for native completion and manual safety inspection, then explicitly retry stand; no stand queued')
        if not self.busy.acquire(False):raise RuntimeError('another local task is active')
        backend=None
        try:
            self.identity() # 每次任务仍检查实际机体身份
            if mode in ('0','1','2','stage1','0-full'):
                self.gate.require_empty_start()
            cfg=self.config(config) # 每次任务重读参数；不保存或恢复旧队列
            if stop_after is not None:
                cfg=dict(cfg,transfer_test=dict(cfg.get('transfer_test',{}),stop_after=stop_after))
            plan=self.engine.prepare(mode,cfg,self.project) # 未实现区域在动作前拒绝
            backend=GateBackend(self.gate);backend.ready()
            from turn_estimator import MICRO_FILTER_ALPHA, MICRO_INITIAL_DEG, TurnEstimator
            from micro_trial import response_limit
            response_bound=response_limit(plan.get('navigation',{}).get('post_action_heading',{}))
            magnetic=plan.get('navigation',{}).get('magnetic',{})
            filtering=plan['config'].get('route_steps',{}).get('micro_turn_filter',{})
            estimator_options=dict(max_age_s=magnetic.get('max_age_s',.8),
                max_norm_change=magnetic.get('max_norm_fraction_change',.35),
                max_micro_deg=response_bound,allow_provisional=response_bound>8,
                micro_initial_deg=filtering.get('initial_deg',MICRO_INITIAL_DEG),
                micro_alpha=filtering.get('alpha',MICRO_FILTER_ALPHA))
            if getattr(self,'_turn_estimator_options',None)!=estimator_options:
                self.turn_estimator=TurnEstimator(**estimator_options)
                self._turn_estimator_options=estimator_options
            backend.turn_estimator=self.turn_estimator
            if cfg.get('persistent_navigation_cache') is True and plan.get('navigation',{}).get('enabled'):
                self.prewarm_navigation(cfg)
                if self.navigation_cache is None or self.navigation_cache_error:
                    raise RuntimeError('navigation cache unavailable: '+str(self.navigation_cache_error))
                backend.navigation_lease=self.navigation_cache.acquire(plan['navigation'])
                backend.navigation_io=backend.navigation_lease.require_config
            if cfg.get('direct_operator_start') is True:
                usage=self.state/'single_attempt.json'
                repeated=self.gate.micro.repeated_single_ready()
                if mode in ('0','stage1','1','2','0-full') and not repeated and usage.exists():
                    used=json.loads(usage.read_text())
                    if used.get('boot_id')==self.gate.micro.boot_id:
                        raise RuntimeError('MCU single-grab allowance already consumed this boot; no stand or movement sent. Inspect and fully restart robot, or install repeated-single MCU update.')
                backend.usage_file=None if repeated else usage
                self.gate.authorize_operator_start()

            job={'done':threading.Event(),'cancel':threading.Event(),'result':None,'backend':backend,'cfg':cfg,'plan':plan,'thread':None}
            self.active=job
            return job
        except BaseException:
            if backend is not None and getattr(backend,'navigation_lease',None) is not None:backend.navigation_lease.close()
            self.busy.release();raise

    def execute(self,job):
        started=time.monotonic()
        try:
            if job['cancel'].is_set() or self.closed.is_set():raise RuntimeError('trigger closed before execution')
            result=self.engine.execute(job['plan'],job['backend'])
            job['result']=dict(result,event='done',elapsed_s=time.monotonic()-started)
        except BaseException as exc:
            job['backend'].cancel()
            job['result']={'event':'done','status':'stopped_no_further_auto_macros','reason':str(exc),'events':getattr(exc,'route_events',[]),'native_immediate_stop':False,'elapsed_s':time.monotonic()-started,'diagnostic':self.gate.status()}
            job['result'].update(getattr(exc,'error_details',{}))
            if 'outbound_pickup' in job['plan']:
                job['result'].update(command=job['plan']['mode'],deliveries=getattr(exc,'completed_deliveries',0),
                    expected_blocks_delivered=getattr(exc,'expected_blocks_delivered',0),
                    completed_returns=getattr(exc,'completed_returns',0),return_started=getattr(exc,'return_started',False))
        finally:
            if getattr(job['backend'],'navigation_lease',None) is not None:job['backend'].navigation_lease.close()
            try:
                _write_runtime_report(self.project,job['result'])
            except Exception:pass  # Persistence failures must not suppress the terminal receipt.
            job['done'].set();self.active=None;self.busy.release()

    def cancel_job(self,job):
        if not job['done'].is_set():job['cancel'].set();job['backend'].cancel()

    def start_endpoint(self):
        if not self.prepared:raise RuntimeError('owner preparation failed: '+str(self.prepare_error))
        if self.state.is_symlink():raise RuntimeError('state directory is a symlink')
        self.state.mkdir(parents=True,exist_ok=True)
        lock=self.state/'warm.lock'
        if lock.is_symlink():raise RuntimeError('warm lock is a symlink')
        self.lockfile=lock.open('a')
        try:exclusive_lock(self.lockfile)
        except BaseException:self.lockfile.close();self.lockfile=None;raise RuntimeError('another warm endpoint is active')
        if os.path.lexists(str(self.socket_path)):
            if self.socket_path.is_symlink() or not stat.S_ISSOCK(self.socket_path.lstat().st_mode):raise RuntimeError('unexpected control socket path')
            self.socket_path.unlink() # 独占锁确认没有本代码的活跃端点
        self.listener=self.listener_factory() if self.listener_factory else socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
        self.listener.bind(str(self.socket_path));self.listener.listen(4);self.listener.settimeout(.2)
        if os.name!='nt':
            if effective_uid()==0:os.chown(str(self.socket_path),0,1000)
            os.chmod(str(self.socket_path),0o660)
        thread=threading.Thread(target=self.serve,name='r3-local-trigger',daemon=True);thread.start();self.threads.append(thread)

    def serve(self):
        while not self.closed.is_set():
            try:conn,_=self.listener.accept()
            except socket.timeout:continue
            except OSError:break
            thread=threading.Thread(target=self.handle,args=(conn,),daemon=True);thread.start();self.threads.append(thread)

    @staticmethod
    def send(conn,row):conn.sendall((json.dumps(row,ensure_ascii=False)+'\n').encode())

    def handle(self,conn):
        job=None
        try:
            if hasattr(socket,'SO_PEERCRED'):
                _,uid,_=struct.unpack('3i',conn.getsockopt(socket.SOL_SOCKET,socket.SO_PEERCRED,12))
                if uid not in (0,1000,effective_uid()):raise RuntimeError('local peer uid rejected')
            conn.settimeout(2);data=b''
            while b'\n' not in data:
                part=conn.recv(4096)
                if not part:return
                data+=part
                if len(data)>8192:raise RuntimeError('request too large')
            first,extra=data.split(b'\n',1)
            if extra:raise RuntimeError('one trigger request at a time')
            request=json.loads(first)
            if request.get('protocol')!=PROTOCOL or set(request)-{'protocol','op','mode','legacy_after_home','config','expected_owner_version','one_step_confirmed','recovery','stop_after','micro_trial','idle_only'}:raise RuntimeError('unknown trigger schema')
            if request.get('op')=='confirm_recovery':
                if self.busy.locked():raise RuntimeError('task active; cannot acknowledge recovery')
                self.identity()
                record=json.loads((self.project/'config/mcu_guard_deployment.json').read_text(encoding='utf-8'))
                recovery=self.gate.confirm_recovery(request.get('recovery'),record)
                self.gate.publish()
                self.send(conn,{'event':'recovery_confirmed','recovery_gate':recovery,'body_commands':0});return
            if request.get('op')=='confirm_micro':
                if self.busy.locked():raise RuntimeError('task active; cannot acknowledge micro trial')
                self.identity()
                result=self.gate.confirm_micro(request.get('micro_trial'))
                self.gate.publish()
                self.send(conn,{'event':'micro_confirmed','micro_trial':result,'body_commands':0});return
            if request.get('op')=='status':self.send(conn,dict(self.status(),event='status'));return
            if request.get('op')=='probe_pair':
                if not self.busy.acquire(False):raise RuntimeError('task active; cannot probe MCU')
                try:
                    self.identity()
                    backend=GateBackend(self.gate);backend.ready()
                    try:
                        backend.call(PAIR_PROBE,PAIR_PROBE_TIMEOUT_S)
                    except PairProbeUnavailable:
                        pass
                    owner=self.gate.status()
                    self.send(conn,{'event':'pair_probe','body_commands':0,
                        'pair_supported':owner['micro_trial']['pair_live_capability_verified'],
                        'receipt':owner['last_result']})
                finally:self.busy.release()
                return
            if request.get('op')=='stop':
                active=self.active
                if active:self.cancel_job(active)
                else:self.gate.cancel('explicit stop while idle; recovery acknowledgment invalidated');self.gate.publish()
                self.send(conn,{'event':'stopped','status':'stop_requested' if active else 'already_idle','native_immediate_stop':False});return
            if request.get('op')!='start' or request.get('mode') not in ('0','1','2','0-full','stage1','stand','test-left','test-right','test-small-left','test-small-right','test-turn-left','test-turn-right','test-micro-left','test-micro-right','test-medium-left','test-medium-right') or request.get('legacy_after_home') not in (None,'0'):raise RuntimeError('unknown explicit command')
            if request.get('mode') in ('test-medium-left','test-medium-right') and request.get('legacy_after_home') is not None:raise RuntimeError('medium turn calibration requires a standalone command')
            if request.get('expected_owner_version')!=self.gate.status()['owner_version']:raise RuntimeError('client/owner version mismatch; no body command sent')
            if request['mode'].startswith('test-') and request.get('one_step_confirmed') is not True:raise RuntimeError('calibration movement requires explicit --one-step')
            idle_only=request.get('idle_only',False)
            if type(idle_only) is not bool or idle_only and request['mode']!='stand':
                raise RuntimeError('idle-only applies only to an explicit stand request')
            job=self.start_job(request.get('config'),request['mode'],request.get('stop_after'),idle_only=idle_only)
            source='action_chains_v5.py' if 'mission_v5' in job['plan'] else 'routes.py'
            self.send(conn,{'event':'accepted','protocol':PROTOCOL,'command':request['mode'],'route_source':source})
            thread=threading.Thread(target=self.execute,args=(job,),daemon=True);job['thread']=thread
            try:thread.start()
            except BaseException:job['thread']=None;raise
            self.threads.append(thread)
            conn.settimeout(None)
            while not job['done'].is_set():
                if self.closed.is_set():self.cancel_job(job);break
                readable,_,_=select.select([conn],[],[],.05)
                if readable:
                    part=conn.recv(4096)
                    # 断连、Ctrl+C或异常后续输入都取消余链；
                    # 不把输入偷偷转为另一个排队任务。
                    self.cancel_job(job)
                    if not part:return
                    self.send(conn,{'event':'cancelled','native_immediate_stop':False});return
            if job['done'].is_set():self.send(conn,job['result'])
        except BaseException as exc:
            if job:
                self.cancel_job(job)
                if job['thread'] is None:
                    if getattr(job['backend'],'navigation_lease',None) is not None:job['backend'].navigation_lease.close()
                    job['done'].set();self.active=None;self.busy.release()
            try:self.send(conn,{'event':'error','reason':str(exc),'diagnostic':self.gate.status()})
            except OSError:pass
        finally:conn.close()

    def close(self):
        self.closed.set()
        if self.active:self.cancel_job(self.active)
        if self.navigation_cache is not None:self.navigation_cache.close()
        if self.listener:
            self.listener.close();self.listener=None
            try:self.socket_path.unlink()
            except FileNotFoundError:pass
        if self.lockfile:self.lockfile.close();self.lockfile=None


def install(gate):
    controller=WarmController(gate)
    try:controller.start_endpoint()
    except Exception:
        controller.close();raise
    import rospy
    rospy.on_shutdown(controller.close)
    return controller
