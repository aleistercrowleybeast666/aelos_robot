"""现有唯一UART owner内的手动优先门禁。

保留旧服务；自动请求携带会话和取消纪元，迟到请求/COMPLETE不能恢复旧队列。
stand也使用同一互斥与完成合同，不能绕过未知在途状态。
"""
import hashlib
import json
import threading
import time
import uuid
import re
from r3_recovery_gate import RecoveryGate
from r3_micro_gate import MicroGate, NAMES as MICRO_NAMES, PAIR_BODY_OPS, PAIR_PROBE, PICKUP_BACKWARD6_OPS, PROBE_OPS

WIRE_BASE=0x2A00
CANCELLED_BOUNDARY=WIRE_BASE+0xFE
_MCU_ERROR='MCU ERROR; automatic requests blocked'
VALID_KEYS=frozenset([0xC1,0xC2,0xC3,0xC4,0xC5,0xC6,0xC7,0xC8,
 0xC9,0xCA,0xCB,0xCD,0xCE,0xCF,0xDC,0xDE,0xDF,0xE9,0xEA,0xEB,
 0xED,0xEF,0xF1,0xF2,0xF3,0xF4,0xF5,0xF6])
NAMES={op:'r3_simple_'+op for op in ('begin','begin_align','free_forward','grab_single','carry_forward','left_turn','right_turn','carry_forward2','carry_forward4','strafe_left_once','strafe_right_once','stand','finish')}
NAMES.update({op:'r3_simple_'+op for op in ('empty_forward2', 'empty_left_turn', 'empty_right_turn', 'empty_strafe_left', 'empty_strafe_right', 'place_single', 'carry_forward7', 'carry_forward9')})
MISSION_OPS=frozenset(('empty_forward2', 'empty_left_turn', 'empty_right_turn', 'empty_strafe_left', 'empty_strafe_right', 'place_single', 'carry_forward7', 'carry_forward9'))
NAMES.update(MICRO_NAMES)
HASHES={op:'leju_'+hashlib.md5(name.encode()).hexdigest() for op,name in NAMES.items()}
OWNED=frozenset(HASHES.values())
_ACQUIRE_LOAD=frozenset(('grab_single','grab_double','begin_align'))
_RELEASE_LOAD=frozenset(('stand','place_single','cmd0_place_single','place_double'))
_RESET_REQUIRED='previous automatic task may still hold a block; explicitly restore empty stand before starting again'


class Gate:
    def __init__(self,queue,clock=time.monotonic,boot_id=None):
        self.queue=queue;self.clock=clock;self.cv=threading.Condition()
        self.session=uuid.uuid4().hex;self.epoch=0;self.mode='manual'
        self.current=None;self.last_hello=float('-inf');self.seen_bridge=False;self.reason='manual default'
        self.last_hello_source=None
        self.last_key=0
        self.last_wire_key=0
        self.last_rejection=None;self.last_result=None;self.operation_seq=0
        self._may_hold_load=False
        self.recovery=RecoveryGate(self.session,self.clock)
        self.micro=MicroGate(self.session,self.clock,boot_id)

    def status(self):
        with self.cv:
            contact_age=self.clock()-self.last_hello if self.seen_bridge else None
            recovery=self.recovery.status(self.epoch)
            pair_ready=self.micro.pair_ready(self.epoch,recovery['ready'])
            return {'schema':'r3.simple.v1','robot_id':3,'session':self.session,'epoch':self.epoch,
                'mode':self.mode,'reason':self.reason,'last_key':self.last_key,
                'mcu_ready':contact_age is not None and 0<=contact_age<=2.5,
                'mcu_last_contact_age_s':contact_age,
                'mcu_last_contact_source':self.last_hello_source,
                'in_flight':self.current is not None,'published_at_s':time.time(),
                'restart_requires_empty_reset':self.mode=='manual' and self._may_hold_load,
                'operation_seq':self.operation_seq,'native_immediate_cancel':False,'last_rejection':self.last_rejection,'last_result':self.last_result,
                'current_operation':None if not self.current else {k:self.current.get(k) for k in ('op','dispatched','started','complete','cancelled','drained')},
                'recovery_gate':recovery,
                'micro_trial':self.micro.status(self.epoch,recovery['ready']),
                'mission_v5_ready':self.micro.mission_ready(self.epoch,recovery['ready']),
                'mission_v5_pair_ready':pair_ready,
                'mission_v5_pair_ops':sorted(PAIR_BODY_OPS) if pair_ready else [],
                'expected_wire_abi':WIRE_BASE//256,'owner_version':'r3-visual-pickup-20261109-v5'}

    def cancel(self,reason,key=0,force_epoch=False):
        with self.cv:
            c=self.current
            if (reason==_MCU_ERROR and c and (c['op']==PAIR_PROBE or c['op'] in PICKUP_BACKWARD6_OPS)
                    and c['dispatched'] and not c['started'] and not c['cancelled']):
                # Installed UART owners report unknown aliases through cancel.
                # Defer classification until the next ordered idle REQ.
                c['mcu_error_before_start']=True
                return
            # 长按/重复键不重新开放会话，也不反复增加取消纪元。
            if force_epoch or self.mode=='auto' or self.current and not self.current['cancelled']:
                self.epoch+=1
            self.mode='manual';self.reason=reason
            self.recovery.clear('cancel/takeover invalidated recovery acknowledgment')
            self.micro.clear()
            if key in VALID_KEYS:self.last_key=key
            self.queue[:]=[n for n in self.queue if n not in OWNED]
            if self.current:self.current['cancelled']=True
            self.cv.notify_all()

    def authorize_operator_start(self):
        """Explicit local task command, not an assertion of measured physical health."""
        with self.cv:
            if self.mode!='manual' or self.current or self.queue or not self.status()['mcu_ready']:
                raise RuntimeError('direct operator start requires live manual idle and drained MCU')
            if not self.micro.deployment_ready() or self.micro.deployment.get('policy_single_enabled') is not True:
                raise RuntimeError('verified single-enabled installation required')
            now=self.clock()
            self.recovery.permit=dict(session=self.session,epoch=self.epoch,operator='explicit-local-operator-command',confirmed_monotonic=now,expires_monotonic=now+900.,evidence='user requested direct task start; NOT cooling or physical inspection verification')
            self.recovery.last_reason='operator direct-start authorization; physical condition not measured'
            self.micro.permit=dict(session=self.session,epoch=self.epoch,boot_id=self.micro.boot_id,operator='explicit-local-operator-command',confirmed_monotonic=now,evidence='direct start policy requested by user; not new physical confirmation')

    def require_empty_start(self):
        with self.cv:
            if self._may_hold_load:
                raise RuntimeError(_RESET_REQUIRED)

    def confirm_recovery(self, payload, deployment):
        with self.cv:
            if self.mode != 'manual' or self.current or self.queue:
                raise RuntimeError('manual idle and empty queue required for human recovery acknowledgment')
            if not self.status()['mcu_ready']:
                raise RuntimeError('matching live bridge required; this does not verify MCU guard installation')
            if self.micro.deployment_ready():
                expected={k:v for k,v in self.micro.deployment['verified_files_sha256'].items() if k!='lua/craic_single_micro.lua'}
                if deployment.get('verified_files_sha256')!=expected or deployment.get('policy_single_enabled')!=self.micro.deployment.get('policy_single_enabled'):
                    raise RuntimeError('thermal guard record must match same-boot complete micro readback, including policy')
            return self.recovery.confirm(payload, self.epoch, deployment)

    def confirm_micro(self,payload):
        with self.cv:
            if self.mode!='manual' or self.current or self.queue:raise RuntimeError('manual idle required for micro acknowledgment')
            return self.micro.confirm(payload,self.epoch,self.recovery.status(self.epoch)['ready'])

    def decoded_key(self,params):
        if not isinstance(params,list) or len(params)!=1 or type(params[0]) is not int:return 0
        raw=params[0]
        if raw//256==WIRE_BASE//256:
            self.seen_bridge=True;self.last_hello=self.clock();self.last_hello_source='request'
            return raw%256
        return raw if 0<=raw<=255 else 0

    def request(self,params):
        with self.cv:
            key=self.decoded_key(params)
            if params==[CANCELLED_BOUNDARY]:
                # v5的有序REQ在MCU函数返回后报告取消；不是COMPLETE/身体成功。
                self.cancel('MCU cancelled at returned native boundary')
                if self.current and self.current['dispatched']:self.current['drained']=True
                self.cv.notify_all()
                return 'default',0
            if key in VALID_KEYS:
                new_press=key!=self.last_wire_key
                self.last_wire_key=key
                self.cancel('valid remote key: manual takeover',key,force_epoch=new_press)
                return 'default',key
            if key in (0,0xDA):self.last_wire_key=0
            if self.current and self.current['dispatched']:
                c=self.current
                if c['op']==PAIR_PROBE and params==[WIRE_BASE] and not c['complete']:
                    # This read-only alias never polls from inside its body. The
                    # next REQ proves it returned without a completion receipt.
                    if not c['cancelled']:
                        if c['started']:
                            self.cancel('MCU pair capability probe returned without COMPLETE')
                        else:
                            c['probe_unavailable']=True
                    c['drained']=True
                    self.cv.notify_all()
                elif (c['op'] in PICKUP_BACKWARD6_OPS and params==[WIRE_BASE]
                      and c.get('mcu_error_before_start') and not c['started'] and not c['cancelled']):
                    c.update(retreat_unavailable=True,drained=True)
                    self.cv.notify_all()
                return 'default',0  # native在途时禁止嵌套分派
            while self.queue:
                name=self.queue.pop(0)
                if name in OWNED:
                    c=self.current
                    if not c or c['native']!=name or c['cancelled'] or c['epoch']!=self.epoch:continue
                    c['dispatched']=True
                    if c['op'] in _ACQUIRE_LOAD:self._may_hold_load=True
                return name,0
            return 'default',0

    def started(self):
        with self.cv:
            c=self.current
            if not c:return # 保留旧手动/服务的完成语义
            if c['cancelled']:return
            if not c['dispatched'] or c['started']:
                self.cancel('unexpected or duplicate START');return
            c['started']=True

    def completed(self):
        with self.cv:
            if self.seen_bridge:
                self.last_hello=self.clock()
                self.last_hello_source='complete'
            c=self.current
            if not c:return
            if c['cancelled']:
                c['drained']=True;self.cv.notify_all();return
            if not c['started'] or c['complete']:
                self.cancel('unexpected COMPLETE');return
            c['complete']=True;c['drained']=True;self.cv.notify_all()

    def submit(self,text,timeout):
        try:
            parts=text.split('|')
            if len(parts) not in (4,5):return False
            prefix,session,epoch,op=parts[:4]
            nonce=parts[4] if len(parts)==5 else None
            if nonce is not None and not re.fullmatch('[0-9a-f]{32}',nonce):return False
            epoch=int(epoch)
        except (ValueError,TypeError):return False
        if prefix!='R3AUTO' or op not in set(NAMES)|{'cancel'}:return False
        with self.cv:
            def reject(code):
                self.last_rejection={'op':op,'request_nonce':nonce,'code':code,'published_at_s':time.time()};return False
            if session!=self.session or epoch!=self.epoch:return reject('session_or_epoch_mismatch')
            if op=='cancel':self.cancel('client cancelled');return True
            if not self.status()['mcu_ready']:return reject('MCU_handshake_missing_or_stale')
            if self.current:return reject('native_in_flight_or_not_drained')
            if self.queue:return reject('existing_queue_not_empty')
            if type(timeout) not in (float,int) or not 1<=timeout<=120:return reject('invalid_timeout')
            if op=='begin' and self._may_hold_load:return reject('previous_load_requires_explicit_empty_reset')
            if op in PROBE_OPS:
                if self.mode!='manual':return reject('manual_idle_required_for_capability')
            denial=self.micro.admit(op,nonce,self.epoch,self.recovery.status(self.epoch)['ready'])
            if denial:return reject(denial)
            if op in MISSION_OPS and not self.micro.mission_ready(self.epoch,self.recovery.status(self.epoch)['ready']):return reject('mission_v5_verified_installation_and_live_capability_required')
            if op in ('begin','begin_align','stand'):
                if self.mode!='manual':return reject('manual_idle_required')
                if not self.recovery.status(self.epoch)['ready']:return reject('current_session_recovery_inspection_required')
                self.mode='auto';self.last_key=0;self.reason='explicit stand' if op=='stand' else 'explicit automatic start'
            elif op not in PROBE_OPS and self.mode!='auto':return reject('automatic_mode_required')
            self.operation_seq+=1
            c={'operation_seq':self.operation_seq,'request_nonce':nonce,'native':HASHES[op],'epoch':self.epoch,'op':op,'dispatched':False,
                'started':False,'complete':False,'cancelled':False,'drained':False}
            self.current=c;self.queue.append(c['native']);deadline=self.clock()+timeout
            while not (c['complete'] or c['cancelled'] or c.get('probe_unavailable') or c.get('retreat_unavailable')):
                left=deadline-self.clock()
                if left<=0:self.cancel('automatic request timed out; no further queue');break
                self.cv.wait(left)
            ok=(c['complete'] and not c['cancelled'] and not c.get('probe_unavailable')
                and not c.get('retreat_unavailable') and c['epoch']==self.epoch)
            self.last_result={'op':op,'request_nonce':nonce,'operation_seq':c['operation_seq'],'protocol_completed':bool(ok),'reason':self.reason,
                **{k:c[k] for k in ('dispatched','started','complete','cancelled','drained')}}
            if c.get('probe_unavailable'):
                self.micro.capability=None
                self.last_result['probe_unavailable']=True
            if c.get('retreat_unavailable'):
                self.last_result['retreat_unavailable']=True
            if ok:
                if op in PROBE_OPS:self.micro.complete_probe(c,self.epoch)
                if op in _RELEASE_LOAD:self._may_hold_load=False
                self.current=None
                if op=='finish':
                    self.recovery.clear('task ended; new physical inspection acknowledgment required')
                    self.micro.clear()
                if op in ('finish','stand'):
                    self.mode='manual';self.reason='normal stand request finished; MCU load set empty; physical pose unverified' if op=='stand' else 'chain finished'
            elif not c['dispatched'] or c['drained']:
                self.current=None
            # 已下发但被取消的动作保留到迟到COMPLETE排空；
            # 不能先解锁，让旧完成帧被当作新动作完成。
            return ok

    def clear_drained(self):
        with self.cv:
            if self.current and self.current['cancelled'] and self.current['drained']:
                self.current=None


def install(queue):
    import rospy
    from std_msgs.msg import String
    gate=Gate(queue)
    pub=rospy.Publisher('/r3_auto/status',String,queue_size=1,latch=True)
    def publish(event=None):
        gate.clear_drained();pub.publish(String(data=json.dumps(gate.status())))
    gate.publish=publish
    gate.timer=rospy.Timer(rospy.Duration(.25),publish)
    # 开机只预热代码与接口；不排队站姿、begin，也不恢复旧任务。
    try:
        from r3_auto_warm import install as install_warm
        gate.warm=install_warm(gate)
    except Exception as exc:
        gate.warm=None
        rospy.logwarn('R3 warm trigger unavailable; manual/legacy service retained: %s',str(exc))
    publish();return gate
