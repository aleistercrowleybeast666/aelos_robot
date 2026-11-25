"""区域函数执行器：复用原有有限动作；独立站姿不启动运输路线。"""
import collections,json,math,time,types,threading
from enum import Enum
from pathlib import Path

from transfer_test import BoundaryReached, MACRO_BOUNDARIES
from cmd0_motion import (COMMAND0_TRANSFER_FORWARD_STEPS, TRANSFER_FORWARD9_ACTION, Cmd0Motion_ContinuousReady,
                         Cmd0Motion_ForwardActions, Cmd0Motion_ForwardStepCount, Cmd0Motion_LegacyReady, Cmd0Motion_LoadRouteSteps,
                         FORWARD_ACTIONS as CMD0_FORWARD_ACTIONS, SIDE_ACTIONS as CMD0_SIDE_ACTIONS,
                         PLACE_ACTION as CMD0_PLACE_ACTION, PICKUP_BACKWARD4_ACTION, PICKUP_BACKWARD6_ACTION,
                         PICKUP_BACKWARD8_ACTION, PICKUP_EXIT_ACTIONS)
from heading_guard import (CARRY_HEADING_ACTIONS, CORRIDOR_HEADING_ACTIONS,
                           MagneticDataUnavailable, MagneticLoss_IsRecoverable, MicroTrialStopped)
from micro_step_test import TURN_TEST_STEPS
from nav_observation import MAGNETIC_INSUFFICIENT_SAMPLES

class NotReady(RuntimeError):pass


class RouteTagHeadingRebaseResult(Enum):
    APPLIED = 'applied'
    UNAVAILABLE = 'unavailable'


class _Tag1ReacquireResult(Enum):
    ADVANCE_UNSEEN = 'advance_unseen'
    ADVANCE_FAR = 'advance_far'
    RETREAT_NEAR = 'retreat_near'
    WAIT_CAMERA = 'wait_camera'
    WAIT_AMBIGUOUS = 'wait_ambiguous'
    WAIT_NEAR = 'wait_near'


class MagneticPreflightError(RuntimeError):
    def __init__(self, reason, health):
        super().__init__(reason)
        fault = health.get('fault')
        if fault in ('producer_restarted', 'producer_fault_since_binding', 'unobserved_failed_attempt'):
            message = '磁力计采集重启或中途出错，启动检查未通过。'
        elif health.get('recoverable_interruption'):
            message = '磁力计采样中断，启动检查未通过。'
        elif fault:
            message = '磁力计采样校验失败，启动检查未通过。'
        elif MAGNETIC_INSUFFICIENT_SAMPLES in reason:
            message = '启动检查未收到足够的磁力计新样本。'
        else:
            message = '磁力计数据未通过启动检查。'
        self.error_details = {
            'error_code': 'magnetic_preflight_unavailable',
            'stage': 'sensor_preflight',
            'reason_zh': message + '未发送搬运动作。',
            'magnetic_capture': health,
        }


CALIBRATION_STEPS={'test-left':'strafe_left_once','test-right':'strafe_right_once','test-turn-left':'left_turn','test-turn-right':'right_turn'}
SMALL_STRAFE_TEST_STEPS={'test-small-left':'single_small_strafe_left','test-small-right':'single_small_strafe_right'}
CALIBRATION_STEPS.update(SMALL_STRAFE_TEST_STEPS)
BODY_ACTIONS=frozenset(('free_forward','grab_single','carry_forward','carry_forward2','carry_forward4','left_turn','right_turn','strafe_left_once','strafe_right_once',*SMALL_STRAFE_TEST_STEPS.values()))
BODY_ACTIONS=BODY_ACTIONS.union(CMD0_FORWARD_ACTIONS.values(), CMD0_SIDE_ACTIONS.values(),
                              ('carry_forward7', TRANSFER_FORWARD9_ACTION, CMD0_PLACE_ACTION), PICKUP_EXIT_ACTIONS)
_COMMAND0_PHASES=('cmd0_prepare','cmd0_approach','cmd0_pickup','cmd0_carry','cmd0_turn','cmd0_scan','cmd0_entry_reacquire')
_TAG1_APPROACH_GROUPS = 3
_TAG1_QUICK_SCAN_S = 2.
_TAG1_IDLE_REOBSERVATIONS = 3
_TAG1_REOBSERVE_SCANS = ('no_tag', 'id_seen_pose_untrusted', 'unreliable')
_TAG1_PROBE_DEFAULT_MAX_TRAVEL_M = .125
_TAG1_UNCONFIRMED_REASON_ZH = '入口已完成找码与原地复核，仍未确认 Tag 1，请检查机器人位置和胸部相机。'
_TAG1_POSE_UNCONFIRMED_REASON_ZH = '已识别 Tag 1，当前位姿未能验证，无法确认进场方向。'
_TAG3_VISUAL_MICRO_TRIGGER_DEG = 7.

def load(path):
    cfg=json.loads(Path(path).read_text(encoding='utf-8'))
    if cfg.get('schema')!='r3.function-tasks.v1' or cfg.get('robot_id')!=3:raise ValueError('function runtime config required')
    for key,lo,hi in (('request_timeout_s',2,120),('journey_timeout_s',10,900),('max_body_macros_per_journey',1,200)):
        if type(cfg.get(key)) is not int or not lo<=cfg[key]<=hi:raise ValueError(key+' invalid')
    if type(cfg.get('empty_retry_s')) not in (int,float) or not math.isfinite(cfg['empty_retry_s']) or not .1<=cfg['empty_retry_s']<=10:raise ValueError('empty retry invalid')
    import importlib.util
    spec=importlib.util.spec_from_file_location('r3_scan_settings',Path(__file__).with_name('r3_scan.py'))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);module.validate(cfg['terminal_scan'])
    from transfer_test import validate
    validate(cfg)
    return cfg

def load_routes(project):
    path=Path(project)/'routes.py'
    module=types.ModuleType('r3_editable_routes');module.__file__=str(path)
    # 每次任务直接编译源码，避免同一时间戳内等长修改误用旧pyc；文件路径固定。
    exec(compile(path.read_text(encoding='utf-8'),str(path),'exec'),module.__dict__)
    return module

def prepare(mode,cfg,project,capabilities=(),simulation=False):
    if mode=='0' and cfg.get('transfer_test',{}).get('stop_after','terminal_scan')=='terminal_scan':
        cfg=dict(cfg,route_steps=Cmd0Motion_LoadRouteSteps(project))
    if mode in ('1','2'):
        from outbound_pickup_trial import OutboundPickup_Prepare
        return OutboundPickup_Prepare(mode,cfg,project,simulation)
    if mode=='0-full':
        from mission_v5 import prepare as prepare_v5
        return prepare_v5(mode,cfg,project,simulation)
    from transfer_test import validate
    validate(cfg,mode)
    if mode not in ('0','1','2','stage1','stand') and mode not in CALIBRATION_STEPS and mode not in TURN_TEST_STEPS:raise ValueError('unknown command')
    if mode in CALIBRATION_STEPS or mode in TURN_TEST_STEPS:
        from checkpoint_nav import load as load_navigation
        return {'mode':mode,'config':cfg,'routes':None,'project':str(project),'navigation':load_navigation(project)}
    # 固定站姿不依赖运输路线，人工临时改坏routes.py也不会阻止单独站姿。
    if mode=='stand':
        from checkpoint_nav import load as load_navigation
        return {'mode':mode,'config':cfg,'routes':None,'project':str(project),'navigation':load_navigation(project)}
    if mode in ('0','stage1') and cfg['terminal_scan']['enabled']:
        from tag_reporting import reporting_options
        reporting_options(project) # 输出/参考配置非法也必须在第一身体动作前拒绝。
    from checkpoint_nav import load as load_navigation
    navigation=load_navigation(project)
    target=cfg.get('transfer_test',{}).get('target_id')
    if target is not None and navigation.get('enabled') and target != navigation['legs']['outbound'][0]['id']:
        raise ValueError('explicit target ID must be the first outbound checkpoint in command 0')
    if navigation.get('visual_micro',{}).get('execute') and mode in ('0','stage1') and (target is None or target!=navigation['visual_micro'].get('confirmed_target_id')):
        raise ValueError('visual micro requires an explicitly confirmed matching target ID BEFORE motion')
    from entry_alignment import load_bundle
    entry=load_bundle(project) if mode in ('0','stage1') and not navigation.get('enabled') else None
    tuning=cfg.get('route_steps',{})
    chain=tuning.get('command0_transfer')
    routes=load_routes(project) if chain is None else None
    missing=[f.__name__+': '+f.pending_reason for f in routes.required_sections(mode) if getattr(f,'pending_reason',None)] if routes is not None else []
    if chain is not None:
        transfer_steps=tuning['transfer_forward_steps']
        body=1+(1 if transfer_steps==9 else len(segments(transfer_steps,'single')))
        body+=sum(len(segments(step['steps'],'empty')) for step in chain if step['op']=='free_forward')
        body+=sum(step['repeat'] for step in chain if step['op'] in ('left_turn','right_turn'))
        if body>cfg['max_body_macros_per_journey']:
            raise NotReady('中转区动作链超出任务动作上限，未发送动作')
    # 尚未导出叠块抓放、障碍及空载返程动作；缺项必须在开始前拒绝。
    if mode in ('1','2') and 'stacked' not in capabilities:missing+=['native capabilities: stacked pickup/motion not exported']
    if mode in ('1','2') and 'placement' not in capabilities:missing+=['native capabilities: placement not exported']
    if mode=='2' and 'empty_return' not in capabilities:missing+=['native capabilities: unloaded return motion not exported']
    if missing:raise NotReady('task '+mode+' not ready BEFORE motion: '+'; '.join(missing))
    return {'mode':mode,'config':cfg,'routes':routes,'project':str(project),'entry_alignment':entry,'navigation':navigation}

def segments(steps,load_kind):
    if type(steps) is not int or not 1<=steps<=80:raise ValueError('steps must be an integer 1..80')
    if load_kind=='empty':
        if steps%5:raise ValueError('empty forward currently supports groups of 5 only')
        return ['free_forward']*(steps//5)
    if load_kind!='single':raise NotReady('forward for this load has no validated finite export')
    if steps%2:raise ValueError('carried forward currently supports even groups only; not arbitrary continuous steps')
    result=[]
    for size,name in ((8,'carry_forward'),(4,'carry_forward4'),(2,'carry_forward2')):
        count,steps=divmod(steps,size);result.extend([name]*count)
    return result

class Robot:
    def __init__(self,backend,cfg,project,clock=time.monotonic,sleep=time.sleep):
        self.backend=backend;self.cfg=cfg;self.project=Path(project);self.clock=clock;self.sleep=sleep
        self.magnetic_route_only=False
        self.load_kind='empty';self.events=collections.deque(maxlen=128);self.empty_retry_s=cfg['empty_retry_s'];self.new_journey()
    def new_journey(self):
        self.deadline=self.clock()+self.cfg['journey_timeout_s'];self.count=0;self.nav_count=0;self.heading_guard=None;self.transfer_index=0;self.shared_micro_budget=None;self.last_action_completed_monotonic=None;self.last_action_completed_ros_ns=None;self.last_heading_report=None
        self._managed_transfer_group_done=False
        if getattr(self,'navigator',None) is not None:self.navigator.mag.anchor=None
    def report_command0_phase(self,phase,*,reason_zh=None):
        if not getattr(self,'boundary_recovery_command0',False):
            return
        index=_COMMAND0_PHASES.index(phase)
        previous=getattr(self,'_command0_phase_index',-1)
        if index<previous or (index==previous and reason_zh is None):
            return
        self._command0_phase_index=index
        report=getattr(self.backend,'report_progress',None)
        if report is not None:
            try:
                command=getattr(self,'route_command','0')
                progress={'command':command,'phase':phase,'direction':'outbound'}
                if reason_zh:
                    progress['reason_zh']=reason_zh
                if command in ('1','2'):
                    progress.update(outbound_only=getattr(self,'outbound_only',command=='1'),shared_outbound=True,
                                    journey=getattr(self,'deliveries',0)+1)
                if getattr(self,'command0_scan_after_turn',False):
                    progress.update(trial_target_tag=5,delivery_planned=Cmd0Motion_ContinuousReady(self.backend))
                report(progress)
            except Exception:
                # The status display must never change motion execution.
                pass

    def completed_call(self,name,timeout_s):
        """Backend return means protocol COMPLETE, never physical posture confirmation."""
        started=self.clock()
        try:result=self.backend.call(name,timeout_s)
        except BaseException:
            self.events.append({'stage_timing':{'stage':'protocol_action','action':name,'started_monotonic':started,'elapsed_s':self.clock()-started,'protocol_completed':False}})
            raise
        completed=self.clock()
        self.last_action_completed_monotonic=completed
        nav=getattr(self,'navigator',None)
        ros_now=getattr(getattr(nav,'io',None),'ros_now_ns',None)
        self.last_action_completed_ros_ns=ros_now() if ros_now is not None else None
        self.events.append({'stage_timing':{'stage':'protocol_action','action':name,'started_monotonic':started,'completed_monotonic':completed,'completed_ros_ns':self.last_action_completed_ros_ns,'elapsed_s':completed-started,'protocol_completed':True,'physical_success':'unobserved'}})
        if name=='stand':
            # Preflight samples/anchors are never a post-stand baseline.
            self.heading_guard=None;self.last_heading_report=None
            if nav is not None:nav.mag.anchor=None
        return result

    def check(self):
        self.backend.check()
        if self.clock()>self.deadline:raise RuntimeError('current journey time budget exceeded')
    def wait(self,seconds):
        if type(seconds) not in (int,float) or not math.isfinite(seconds) or not 0<=seconds<=10:raise ValueError('wait must be 0..10 seconds')
        end=self.clock()+seconds
        while self.clock()<end:self.check();self.sleep(min(.05,max(0,end-self.clock())))
        self.check()
    def settle_since_last_completed(self,seconds):
        """Minimum post-COMPLETE dwell; observation time already counts as rest.

        Unlike wait(), this is only for explicit route settling, never retries.
        The latest completion may be a corrective turn, not the original macro.
        Invalid/unknown boundaries conservatively require the full dwell.
        """
        if type(seconds) not in (int,float) or not math.isfinite(seconds) or not 0<=seconds<=10:
            raise ValueError('settle must be 0..10 seconds')
        self.check()
        now=self.clock();completed=getattr(self,'last_action_completed_monotonic',None)
        valid=(type(completed) in (int,float) and math.isfinite(completed) and
               type(now) in (int,float) and math.isfinite(now) and 0<=completed<=now)
        remaining=max(0.,seconds-(now-completed)) if valid else seconds
        self.wait(remaining) # includes cancellation/deadline checks even at zero

    def _allows_missing_magnetic(self):
        return (getattr(self,'command0_scan_after_turn',False) and
                getattr(self,'boundary_recovery_command0',False))

    def _enter_magnetic_route_fallback(self, health, stage, sampling_attempts=None):
        if not self._allows_missing_magnetic() or not MagneticLoss_IsRecoverable(health):return False
        if not self.magnetic_route_only:
            self.magnetic_route_only=True
            nav=self.ensure_navigation_io()
            nav.mag.anchor=None
            if self.heading_guard is not None:
                self.heading_guard.last_sample=None
                self.heading_guard.pending_turn_observation=None
            self.last_heading_report=None
            self.events.append({'magnetic_route_fallback':{
                'stage':stage,'magnetic_capture':health,'sampling_attempts':sampling_attempts or [],
                'follow_configured_actions':True,
                'reason_zh':'磁力计暂时缺少有效样本，按已配置动作走到下一处扫码点。'}})
        return True

    def defer_magnetic_route_fault(self, exc):
        if not isinstance(exc,MagneticDataUnavailable):return False
        return self._enter_magnetic_route_fallback(exc.health,exc.error_details['stage'],
                                                   exc.error_details['magnetic_sampling_attempts'])

    def _heading_controller(self, nav):
        if self.heading_guard is None:
            from heading_guard import HeadingGuard
            self.heading_guard=HeadingGuard(nav.cfg,nav.io,nav.mag,self.check,self.clock,
                budget=self.micro_budget(),estimator=getattr(self.backend,'turn_estimator',None))
        self.heading_guard.allow_missing_magnetic=self._allows_missing_magnetic()
        self.heading_guard.observation_scope=(getattr(self.backend,'session',''),getattr(self.backend,'epoch',-1))
        return self.heading_guard

    def route_checkpoint_observed(self, tag):
        if not self.magnetic_route_only:return
        nav=self.ensure_navigation_io()
        health=nav.io.magnetic_capture_health()
        if not MagneticLoss_IsRecoverable(health):return
        if not nav.io.motion_capture_verified and not nav.io.retry_magnetic_preflight():return
        guard=self._heading_controller(nav)
        try:report=guard.before_straight(self.load_kind)
        except MagneticDataUnavailable:return
        if report.get('status')=='relative_reference_established' and nav.io.motion_capture_verified:
            self.magnetic_route_only=False
            self.events.append({'magnetic_route_restored':{'tag_id':tag,'heading_reference':report}})

    def revalidate_route_heading(self, reserve_s):
        if self.magnetic_route_only or self.heading_guard is None:return None
        self.check_magnetic_before_motion(configured=True)
        if self.magnetic_route_only:return None
        try:return self.heading_guard.revalidate_stationary_target(self.load_kind,reserve_s)
        except MagneticDataUnavailable as exc:
            if not self.defer_magnetic_route_fault(exc):raise

    def check_magnetic_before_motion(self,*,configured=False,heading_feedback=False):
        c=getattr(self,'navigation_bundle',None) or {}
        from micro_trial import active
        if active(c) and not getattr(self.backend,'is_simulation',False):
            nav=self.ensure_navigation_io()
            if nav is not None and getattr(nav.io,'motion_capture_verified',False) and (configured or not self.magnetic_route_only):return
            health=getattr(getattr(nav,'io',None),'magnetic_capture_health',lambda:{})()
            if configured and self._enter_magnetic_route_fallback(health,'before_configured_action'):return
            if heading_feedback and self._allows_missing_magnetic() and MagneticLoss_IsRecoverable(health):
                raise MagneticDataUnavailable('magnetic stream interrupted before optional Tag sweep',health,
                                              stage='visual_tag_search')
            raise RuntimeError('micro trial capture unhealthy BEFORE new body macro; no motion dispatched')

    def record_fall_diagnostic(self,name):
        io=getattr(getattr(self,'navigator',None),'io',None)
        diagnostic=getattr(io,'fall_diagnostic',None)
        if diagnostic is None:return
        try:report=diagnostic(name)
        except Exception:return  # Advisory sensor data must not mask the original motion fault.
        self.events.append({'fall_diagnostic':report})

    def _complete_body_action(self,name,timeout):
        try:self.completed_call(name,timeout)
        except Exception:
            self.record_fall_diagnostic(name)
            raise
        self.count+=1;self.events.append({'action':name,'physical_success':'unobserved'})

    def action(self,name,timeout_s=None,*,monitor_only_as=None):
        if name not in BODY_ACTIONS:raise ValueError('unknown/unsupported body action: '+str(name))
        self.check()
        if self.count>=self.cfg['max_body_macros_per_journey']:raise RuntimeError('journey macro budget exceeded')
        timeout=self.cfg['request_timeout_s'] if timeout_s is None else timeout_s
        if type(timeout) is not int or not 2<=timeout<=self.cfg['request_timeout_s']:raise ValueError('bounded action timeout invalid')
        self.check_magnetic_before_motion(configured=name not in SMALL_STRAFE_TEST_STEPS.values())
        guard=getattr(self,'heading_guard',None)
        if guard is not None:
            guard.observation_scope=(getattr(self.backend,'session',''),getattr(self.backend,'epoch',-1))
        if guard is not None and name in ('left_turn','right_turn'):
            guard.begin_turn_observation(name,self.load_kind,self.clock(),getattr(self.backend,'session',''),getattr(self.backend,'epoch',-1))
        self._complete_body_action(name,timeout)
        entry_context=getattr(self,'_entry_forward_context',None)
        if entry_context is not None:entry_context[1].append(name)
        if guard is not None and name in ('left_turn','right_turn'):guard.finish_turn_observation(self.last_action_completed_monotonic)
        if name==CMD0_PLACE_ACTION:
            self.load_kind='empty' # native completion is not a physical release sensor
            return
        if name=='grab_single':self.load_kind='single' # protocol completion changes expected load before sampling
        try:self.monitor_heading(monitor_only_as or name,
                                 allow_correction=monitor_only_as is None and not self.at_selected_boundary())
        except Exception:
            self.record_fall_diagnostic(name)
            self.boundary_visual_observation(name,fault_diagnostic=True)
            raise
        followup=(self.last_heading_report or {}).get('entry_followup',{})
        boundary=getattr(self,'last_boundary_visual',None) or {}
        cached_target = 2 if entry_context is not None else getattr(self,'_route_end_expected_tag',None)
        if (cached_target is None or followup.get('fresh_tag')!=cached_target
                or boundary.get('target_id')!=cached_target
                or boundary.get('stance_completed_ns')!=self.last_action_completed_ros_ns):
            self.boundary_visual_observation(name)
        self.transfer_index+=1
        if self.selected_boundary_reached():raise BoundaryReached(MACRO_BOUNDARIES[self.transfer_index-1])

    def entry_forward_action(self, name, *, next_action, completed_actions):
        self._entry_forward_context=(next_action,completed_actions)
        try:self.action(name)
        finally:self._entry_forward_context=None

    def route_checkpoint_action(self, name, expected_tag):
        self._route_end_expected_tag=expected_tag
        try:self.action(name)
        finally:self._route_end_expected_tag=None

    def corridor_side_action(self, name):
        """End the side step with a fresh reading before any recovery motion."""
        self.last_heading_report=None
        self.action(name,monitor_only_as='corridor_side_boundary')

    def Route_TurnFromTagHeading(self, tag_id, error, observed_at_ns, *, reserved_actions=0):
        """Submit one camera-guided micro turn without relying on a magnetic reference."""
        from micro_trial import OPS
        from turn_estimator import MICRO_INITIAL_DEG
        paired_route = self.load_kind == 'double' and getattr(self,'pair_route_enabled',False)
        if (tag_id not in (1,2,3,4) or (self.load_kind!='single' and not paired_route)
                or not self._allows_missing_magnetic()
                or getattr(self.backend,'is_simulation',False)
                or type(error) not in (int,float) or not math.isfinite(error)):
            return None
        nav=self.ensure_navigation_io()
        cfg=nav.cfg
        visual=cfg.get('visual_micro',{})
        goal=next((point for point in cfg['legs']['outbound'] if point['id']==tag_id),{})
        if (visual.get('execute') is not True or visual.get('target_id_confirmed') is not True
                or visual.get('direction_evidence')!='measured_r3_tag' or goal.get('allow_turn') is not True
                or abs(error)>visual['max_error_deg']):
            return None
        trigger = _TAG3_VISUAL_MICRO_TRIGGER_DEG if tag_id == 3 else visual['deadband_deg']
        if abs(error) <= trigger:
            return None
        gains=visual['signed_nominal_delta_deg']
        name=next((op for op in OPS if gains[op]*error<0),None)
        bound=visual['max_observed_step_deg']
        improvement=visual['minimum_improvement_deg']
        if name is None:
            return None
        selection_response = bound
        if tag_id in (2, 3, 4):
            # Each corridor correction is followed by a new observation.
            # Use the estimated turn for selection and the full bound for the motion budget.
            initial = self.cfg.get('route_steps', {}).get('micro_turn_filter', {}).get('initial_deg', MICRO_INITIAL_DEG)
            selection_response = min(bound, initial)
            estimator = getattr(self.heading_guard, 'turn_estimator', None)
            anchor = nav.mag.anchor or {}
            direction = 'left' if gains[name] < 0 else 'right'
            measured = estimator.control_estimate(direction, 'micro', self.load_kind,
                getattr(self.backend,'session',''), getattr(self.backend,'epoch',-1),
                anchor.get('calibration_id',''), now_s=self.clock()) if estimator is not None else None
            if measured is not None:
                selection_response = abs(measured['degrees'])
        if abs(abs(error)-selection_response)>abs(error)-improvement:
            return None
        timeout=self.cfg['request_timeout_s']
        if (self.count+reserved_actions+1>self.cfg['max_body_macros_per_journey']
                or self.nav_count>=cfg['max_extra_actions_per_journey']
                or self.deadline-self.clock()<=timeout+cfg['settle_s']+cfg['observe_s']):
            return None
        self.check()
        checker=getattr(self.backend,'check_micro',None)
        live=getattr(nav.io,'visual_motion_ready',None)
        if (checker is None or live is None or type(observed_at_ns) is not int
                or (tag_id in (2, 3, 4) and type(self.last_action_completed_ros_ns) is int
                    and observed_at_ns <= self.last_action_completed_ros_ns)):
            return None
        checker()
        if not live(observed_at_ns) or not self.micro_budget().reserve('visual',name,gains[name],conservative_bound=bound):
            return None
        self._complete_body_action(name,timeout)
        self.nav_count+=1
        self.last_heading_report=None
        self.events.append({'visual_micro_action':name,'tag_id':tag_id,'feedback_source':'tag_only',
                            'angle_evidence':'source_nominal','physical_success':'unobserved'})
        turn = {'action':name,'before_error_deg':error}
        if tag_id in (2, 3, 4):
            turn['selected_response_deg'] = selection_response
        return turn

    def Route_RebaseTagHeading(self, tag_id, error_deg, observed_at_ns, *, direction=None):
        """Keep a fresh expected Tag's yaw error in the magnetic target for this segment."""
        unavailable = RouteTagHeadingRebaseResult.UNAVAILABLE
        bundle = getattr(self,'navigation_bundle',None)
        empty_return = (self.load_kind == 'empty' and getattr(self,'route_command',None) == '2'
                        and ((direction == 'return' and tag_id in (5,6,7,1))
                             or (direction == 'outbound' and tag_id == 1)))
        loaded_route = direction is None and self.load_kind in ('single','double') and tag_id in (1,2,3,4)
        if (not (empty_return or loaded_route) or not self._allows_missing_magnetic()
                or getattr(self.backend,'is_simulation',False)
                or not bundle or not bundle['post_action_heading'].get('enabled')
                or type(error_deg) not in (int,float) or not math.isfinite(error_deg) or abs(error_deg)>180
                or type(observed_at_ns) is not int or observed_at_ns<=0):
            return unavailable
        completed_ns = self.last_action_completed_ros_ns
        if type(completed_ns) is int and observed_at_ns<=completed_ns:
            return unavailable
        self.check()
        nav = self.ensure_navigation_io()
        goal = next((point for point in nav.cfg['legs'][direction or 'outbound'] if point['id']==tag_id),None)
        live = getattr(nav.io,'visual_motion_ready',None)
        if goal is None or live is None or not live(observed_at_ns):
            return unavailable
        heading = {'tag_id':tag_id,'route_heading_deg':goal['route_heading_deg'],
                   'error_deg':error_deg,'observed_at_ns':observed_at_ns}
        if self.magnetic_route_only:
            health = nav.io.magnetic_capture_health()
            if (not MagneticLoss_IsRecoverable(health)
                    or (not nav.io.motion_capture_verified and not nav.io.retry_magnetic_preflight())
                    or not live(observed_at_ns)):
                self.events.append({'segment_heading_reference':{'status':'reference_unavailable',
                                                                 'tag_departure':heading}})
                return unavailable
        guard = self._heading_controller(nav)
        try:
            report = guard.before_straight(self.load_kind,self.last_action_completed_monotonic,
                                           tag_departure=heading)
        except MagneticDataUnavailable as exc:
            if not self.defer_magnetic_route_fault(exc):
                raise
            self.events.append({'segment_heading_reference':{'status':'reference_unavailable',
                                                             'tag_departure':heading}})
            return unavailable
        self.events.append({'segment_heading_reference':dict(report,tag_departure=heading)})
        if report['status']=='tag_departure_reference_established':
            self.magnetic_route_only=False
            self.last_heading_report=None
            return RouteTagHeadingRebaseResult.APPLIED
        return unavailable

    def Corridor_TurnAtTag5(self, name, observed_at_ns):
        """One visual-selected micro turn at the gate, followed by a fresh camera check."""
        if name not in ('single_micro_left','single_micro_right') or self.load_kind not in ('single','double'):
            raise ValueError('Tag 5 requires a loaded micro turn')
        self.check()
        nav = self.ensure_navigation_io()
        timeout = self.cfg['request_timeout_s']
        goal = next((point for point in nav.cfg['legs']['outbound'] if point['id'] == 5), {})
        passage = self.cfg['route_steps']['tag5_entry_lateral']
        if goal.get('allow_turn') is not True or passage.get('verify_passage') is not True:
            return False
        if (self.count >= self.cfg['max_body_macros_per_journey']
                or self.deadline-self.clock() <= timeout + nav.cfg['observe_s']):
            return False
        self.backend.check_micro()
        if not nav.io.visual_motion_ready(observed_at_ns):
            return False
        self.last_heading_report = None
        self._complete_body_action(name,timeout)
        self.monitor_heading('corridor_side_boundary',allow_correction=False)
        return True

    def corridor_recovery_translate(self,name,*,sampled_at):
        """Complete one verified two-step clearance, with no in-place turn hook."""
        if name not in ('carry_forward2','cmd0_carry_backward2'):
            raise ValueError('corridor clearance only supports a two-step move')
        self.check()
        cfg=getattr(self,'navigation_bundle',None)
        guard=self.heading_guard
        report=self.last_heading_report or {}
        if (not cfg or not cfg['post_action_heading'].get('execute') or guard is None
                or self.load_kind!='single' or self.magnetic_route_only
                or report.get('action')!='corridor_side_boundary' or report.get('correction_actions')!=0):
            return False
        if name=='cmd0_carry_backward2' and not Cmd0Motion_ContinuousReady(self.backend):
            return False
        timeout=self.cfg['request_timeout_s']
        settle=cfg['post_action_heading']['settle_s']+cfg['magnetic']['collect_s']
        if (self.count+1>=self.cfg['max_body_macros_per_journey']
                or self.deadline-self.clock()<=2*(timeout+settle)):
            return False
        scope=(getattr(self.backend,'session',''),getattr(self.backend,'epoch',-1))
        if guard.observation_scope!=scope or not guard.corridor_motion_sample_ready(
                self.load_kind,sampled_at,self.last_action_completed_monotonic):
            return False
        self.last_heading_report=None
        self._complete_body_action(name,timeout)
        return True

    def Pickup_RetreatStep(self, name):
        """Complete one loaded retreat macro, leaving the heading check to its group boundary."""
        if (name not in ('cmd0_carry_backward2', PICKUP_BACKWARD4_ACTION, PICKUP_BACKWARD6_ACTION, PICKUP_BACKWARD8_ACTION)
                or self.load_kind not in ('single', 'double')):
            raise ValueError('pickup retreat requires a supported loaded backward operation')
        self.check()
        if self.count >= self.cfg['max_body_macros_per_journey']:
            raise RuntimeError('journey macro budget exceeded')
        self.check_magnetic_before_motion(configured=True)
        self._complete_body_action(name, self.cfg['request_timeout_s'])

    def corridor_recover_tag2_heading(self,expected_tag=None):
        self.corridor_recover_heading(expected_tag,tag2_clearance_on_worsening=True)

    def corridor_recover_heading(self,expected_tag=None,*,boundary='corridor_side_boundary',
                                 tag2_clearance_on_worsening=False):
        """Sample after the last completed motion and correct toward the saved target."""
        self._route_end_expected_tag=expected_tag
        self.last_heading_report=None
        try:
            options={'tag2_clearance_on_worsening':True} if tag2_clearance_on_worsening else {}
            self.monitor_heading(boundary,**options)
        except Exception:
            self.record_fall_diagnostic(boundary)
            self.boundary_visual_observation(boundary,fault_diagnostic=True)
            raise
        finally:
            self._route_end_expected_tag=None

    def _entry_followup(self, name, next_action):
        if next_action is None:
            return {'next_action':'scan_tag2', 'position_confirmed':False}
        if next_action not in ('carry_forward4','carry_forward2','cmd0_carry_forward5'):
            return None
        report=self.boundary_visual_observation(name)
        if not report or not all(report.get(key) for key in ('boundary_valid','camera_healthy','camera_info_ready','contract_match')):
            return None
        scan=report['scan']
        if scan.get('ids')==[2]:
            return {'next_action':next_action, 'fresh_tag':2, 'distinct_frames':scan['samples'],
                    'position_confirmed':False}
        return None

    def _entry_visual_before_magnetic_turn(self,name,next_action,nav,settings):
        from entrance_trial import EntranceTrial_BoundaryCheckpoint, _tag2_boundary_replan
        report=self.boundary_visual_observation(name)
        checkpoint=EntranceTrial_BoundaryCheckpoint(self,nav,2,action=name,report=report)
        if checkpoint is None:
            return None
        remaining=Cmd0Motion_ForwardStepCount([next_action]) if next_action else 0
        decision=_tag2_boundary_replan(checkpoint,nav,settings,remaining)
        if decision is None:
            return None
        _,plan,_=decision
        return {'next_action':next_action or 'scan_tag2','fresh_tag':2,
                'visual_heading_deg':plan['heading_deg'],'position_confirmed':False}

    def _tag4_visual_before_magnetic_turn(self, name, nav, settings):
        """Give the next Tag 4 approach decision the pose before magnetic correction moves it."""
        from entrance_trial import EntranceTrial_BoundaryCheckpoint
        from route_pose import RoutePose_PlanTag4Longitudinal
        report = self.boundary_visual_observation(name, expected_tag=4)
        checkpoint = EntranceTrial_BoundaryCheckpoint(self,nav,4,action=name,report=report)
        if (checkpoint is None and report and report.get('boundary_valid') and report.get('camera_healthy')
                and report.get('contract_match') and report.get('camera_info_ready')):
            # The first stationary frames may still be arriving when the magnetometer completes.
            observe = getattr(nav.io,'observe_checkpoint_id',None)
            if observe is not None:
                try:
                    observe(4,report['after_ns'],min(.35,nav.cfg['observe_s']),self.check)
                except OSError:
                    self.check()
                    return None
                report = self.boundary_visual_observation(name,expected_tag=4)
                checkpoint = EntranceTrial_BoundaryCheckpoint(self,nav,4,action=name,report=report)
        if checkpoint is None:
            return None
        pose = RoutePose_PlanTag4Longitudinal(checkpoint,nav.cfg,settings,approach_steps=4)
        depth=pose.get('distance_bounds_m')
        lateral=pose.get('lateral_bounds_m')
        if (not depth or not lateral or pose['projection_bounds'][0]<=0 or depth[0]<=0
                or depth[1]-depth[0]>.025 or lateral[1]-lateral[0]>.025
                or pose['max_heading_deviation_deg']>3.):
            return None
        return {'next_action':'scan_tag4','fresh_tag':4,'visual_heading_deg':pose['heading_deg'],
                'position_confirmed':False}
    def at_selected_boundary(self):
        c=self.cfg.get('transfer_test',{})
        return bool(getattr(self,'transfer_mode',False) and c.get('enabled') and
                    self.transfer_index<len(MACRO_BOUNDARIES) and
                    c['stop_after']==MACRO_BOUNDARIES[self.transfer_index])
    def selected_boundary_reached(self):
        c=self.cfg.get('transfer_test',{})
        return bool(getattr(self,'transfer_mode',False) and c.get('enabled') and
                    0<self.transfer_index<=len(MACRO_BOUNDARIES) and
                    c['stop_after']==MACRO_BOUNDARIES[self.transfer_index-1])
    def micro_budget(self):
        if self.shared_micro_budget is None:
            from micro_trial import MicroBudget, response_limit
            limits=(getattr(self,'navigation_bundle',None) or {}).get('post_action_heading',{})
            self.shared_micro_budget=MicroBudget(limits.get('max_actions_per_journey',1),max_step_deg=response_limit(limits),
                limit_total_angle=not getattr(self,'command0_scan_after_turn',False))
        return self.shared_micro_budget

    def prepare_heading_segment(self,*,fresh=False,pickup_heading=None):
        cfg=getattr(self,'navigation_bundle',None)
        if (self.magnetic_route_only or not cfg or not cfg.get('post_action_heading',{}).get('enabled')
                or getattr(self.backend,'is_simulation',False)):return
        nav=self.ensure_navigation_io()
        guard=self._heading_controller(nav)
        started=self.clock()
        last_completed=None if fresh else self.last_action_completed_monotonic
        options={'pickup_heading':pickup_heading} if pickup_heading is not None else {}
        try:self.events.append({'segment_heading_reference':guard.before_straight(self.load_kind,action_completed_monotonic=last_completed,**options)})
        except MagneticDataUnavailable as exc:
            if not self.defer_magnetic_route_fault(exc):raise
        finally:self.events.append({'stage_timing':{'stage':'heading_reference','elapsed_s':self.clock()-started}})
    def sensor_preflight(self):
        if getattr(self.backend,'is_simulation',False):return
        nav=self.ensure_navigation_io()
        if nav is None:raise RuntimeError('transfer sensor monitoring must be configured before first stand')
        checker=getattr(self.backend,'check_manual_idle',None)
        if checker is None:raise RuntimeError('manual-idle sensor preflight unavailable')
        from micro_trial import active
        from heading_guard import HeadingGuard, MicroTrialStopped
        from nav_observation import MagneticReference
        from visual_micro import VisualCamera_DiagnoseWindow
        started=self.clock();wall_started=time.monotonic();cancel=threading.Event()
        results={};errors={};timings={};threads=[];magnetic_attempts=[];camera_attempts=[]
        camera_s=min(nav.cfg['observe_s'],nav.cfg.get('preflight_observe_s',nav.cfg['observe_s']))
        magnetic=active(nav.cfg)
        magnetic_window=nav.cfg['magnetic']['collect_s']+nav.cfg['post_action_heading']['settle_s'] if magnetic else 0
        retry_magnetic=getattr(nav.io,'retry_magnetic_preflight',None) if magnetic else None
        magnetic_health=getattr(nav.io,'magnetic_capture_health',lambda:{}) if magnetic else None
        visual=nav.cfg.get('visual_micro',{})
        optional_magnetic=self._allows_missing_magnetic()
        retry_camera=optional_magnetic and visual.get('execute')
        wall_limit=max(camera_s*(2 if retry_camera else 1),magnetic_window*(2 if retry_magnetic is not None else 1))+1.
        def reader_check():
            # No owner/backend access from readers; only the task thread checks owner state.
            if cancel.is_set():raise RuntimeError('sensor preflight reader cancelled')
            if time.monotonic()-wall_started>wall_limit:raise RuntimeError('sensor preflight reader deadline exceeded')
        def camera():
            target=nav.cfg['legs']['outbound'][0]['id']
            w=nav.io.observe(target,0,camera_s,reader_check)
            if retry_camera:
                expected=visual['camera_info']['expected_fingerprint']
                issue=VisualCamera_DiagnoseWindow(w,w.get('camera_info',{}),expected)
                if issue is not None and issue[2]:
                    camera_attempts.append({'reason':issue[0],'reason_zh':issue[1]})
                    reader_check()
                    w=nav.io.observe(target,nav.io.ros_now_ns(),camera_s,reader_check)
                    issue=VisualCamera_DiagnoseWindow(w,w.get('camera_info',{}),expected)
                if issue is not None:
                    raise RuntimeError(issue[0]+' BEFORE stand')
            if not w.get('camera_healthy',False):raise RuntimeError('camera or AR stream lacks new data BEFORE stand')
            if nav.cfg.get('visual_micro',{}).get('execute') and not w.get('camera_info',{}).get('ready'):
                raise RuntimeError('actual CameraInfo source/calibration unverified BEFORE stand')
            return w
        def magnet():
            for attempt in range(2):
                guard=HeadingGuard(nav.cfg,nav.io,MagneticReference(nav.cfg['magnetic']),reader_check,self.clock)
                guard.allow_missing_magnetic=optional_magnetic
                try:
                    result,sample=guard.sample('empty')
                    if sample is None or not sample.get('capture_verified') or not nav.io.motion_capture_verified:
                        raise MicroTrialStopped('fresh magnetic contract unverified BEFORE stand')
                except MicroTrialStopped as exc:
                    health=magnetic_health()
                    magnetic_attempts.append(dict(health,verified=False))
                    reader_check()
                    reason=getattr(exc,'guard_reason',str(exc))
                    needs_samples=(isinstance(exc,MagneticDataUnavailable) or MAGNETIC_INSUFFICIENT_SAMPLES in reason)
                    if attempt == 0 and retry_magnetic is not None:
                        waiting_for_samples=(health.get('configured') and not health.get('fault') and needs_samples)
                        if waiting_for_samples or retry_magnetic():
                            continue
                    if (optional_magnetic and MagneticLoss_IsRecoverable(health)
                            and (needs_samples or health.get('recoverable_interruption'))):
                        return None  # Only the task thread can enter the camera-guided route state.
                    raise MagneticPreflightError(reason,health) from exc
                magnetic_attempts.append(dict(magnetic_health(),verified=True))
                return result
        def read(name,fn):
            t=time.monotonic()
            try:results[name]=fn()
            except BaseException as exc:errors[name]=exc;cancel.set()
            finally:timings[name]=time.monotonic()-t
        jobs=[('camera',camera)]+([('magnetic',magnet)] if magnetic else [])
        try:
            checker()
            for name,fn in jobs:
                thread=threading.Thread(target=read,args=(name,fn),name='r3-preflight-'+name,daemon=True)
                thread.start();threads.append(thread)
            while any(thread.is_alive() for thread in threads):
                checker()
                if errors:raise next(iter(errors.values()))
                if time.monotonic()-wall_started>wall_limit:raise RuntimeError('sensor preflight timed out; no motion permitted')
                for thread in threads:thread.join(.01)
            if errors:raise next(iter(errors.values()))
            checker()
            snapshot=getattr(nav.io,'preflight_health_snapshot',None)
            if snapshot is not None:
                health=snapshot()
                if not health.get('camera_healthy',False):raise RuntimeError('camera or AR stream stale after parallel preflight; no motion permitted')
                if nav.cfg.get('visual_micro',{}).get('execute') and not health.get('camera_info',{}).get('ready'):
                    raise RuntimeError('CameraInfo stale after parallel preflight; no motion permitted')
            if magnetic and (results.get('magnetic') is None or not nav.io.motion_capture_verified):
                health=magnetic_health()
                if not self._enter_magnetic_route_fallback(health,'sensor_preflight',list(magnetic_attempts)):
                    raise MagneticPreflightError('magnetic capture unhealthy after parallel preflight',health)
        finally:
            cancel.set()
            join_deadline=time.monotonic()+.5
            for thread in threads:thread.join(max(0,join_deadline-time.monotonic()))
            unfinished=[thread.name for thread in threads if thread.is_alive()]
            timing={'stage':'sensor_preflight','elapsed_s':self.clock()-started,'wall_elapsed_s':time.monotonic()-wall_started,'readers_s':timings,'parallel_read_only':True,'unjoined_readers':unfinished}
            if magnetic:timing['magnetic_attempts']=list(magnetic_attempts)
            if camera_attempts:timing['camera_rechecks']=list(camera_attempts)
            self.events.append({'stage_timing':timing})
            if unfinished:
                reason='sensor preflight reader did not stop; no motion permitted: '+','.join(unfinished)
                poison=getattr(nav.io,'poison',None)
                if poison is not None:poison(reason)
                else:
                    close=getattr(nav.io,'close',None)
                    if close is not None:close()
                raise RuntimeError(reason)
        self.events.append({'sensor_preflight':'fresh camera and AR stream',
                            'magnetic_capture_ready':magnetic and not self.magnetic_route_only,
                            'mcu_guard_online_status_available':False,'body_commands':0,'parallel_read_only':True,'pre_stand_sample_not_route_baseline':True})

    def ensure_navigation_io(self):
        cfg=getattr(self,'navigation_bundle',None)
        if not cfg or getattr(self.backend,'is_simulation',False):return None
        if getattr(self,'navigator',None) is None:
            from checkpoint_nav import Navigator
            factory=getattr(self.backend,'navigation_io',None)
            if factory is None:
                from nav_ros import RosNavigationIO
                factory=RosNavigationIO
            io=factory(cfg)
            def extra_action(name):
                self.check()
                if self.load_kind!='single' or self.nav_count>=cfg['max_extra_actions_per_journey'] or self.count>=self.cfg['max_body_macros_per_journey'] or self.deadline-self.clock()<self.cfg['request_timeout_s']+cfg['settle_s']+cfg['observe_s']:return None
                if cfg.get('visual_micro',{}).get('enabled'):raise RuntimeError('v10 visual path cannot call legacy large-turn/strafe navigation actions')
                if name not in ('left_turn','right_turn','strafe_left_once','strafe_right_once'):raise ValueError('unknown navigation motion')
                self.completed_call(name,self.cfg['request_timeout_s']);self.nav_count+=1;self.count+=1
                self.events.append({'navigation_action':name,'physical_success':'unobserved'})
                self.monitor_heading(name,allow_correction=False)
                return {'after_ns':io.ros_now_ns()}
            self.navigator=Navigator(cfg,io,extra_action,self.check,self.load_kind,self.clock,
                can_search=lambda n:self.load_kind=='single' and self.nav_count+n<=cfg['max_extra_actions_per_journey'] and self.count+n<=self.cfg['max_body_macros_per_journey'] and self.deadline-self.clock()>n*(self.cfg['request_timeout_s']+cfg['settle_s']+cfg['observe_s'])+self.cfg['request_timeout_s'])
        self.navigator.load_kind=self.load_kind
        return self.navigator

    def monitor_heading(self,name,allow_correction=True,*,tag2_clearance_on_worsening=False):
        name=getattr(self,'heading_turn_phase',None) or name
        cfg=getattr(self,'navigation_bundle',None)
        if (self.magnetic_route_only or not cfg or not cfg.get('post_action_heading',{}).get('enabled')
                or getattr(self.backend,'is_simulation',False)):return
        nav=self.ensure_navigation_io()
        guard=self._heading_controller(nav)
        def correct(turn):
            self.check()
            import micro_trial
            if micro_trial.enabled(cfg['post_action_heading']):
                medium_ok=(turn in micro_trial.MEDIUM_OPS and guard.boundary_recovery_enabled(name,self.load_kind)
                           and guard.medium_turn_ready())
                if ((turn not in micro_trial.OPS and not medium_ok) or not guard.carry_load_supported(self.load_kind)
                        or name not in CARRY_HEADING_ACTIONS|CORRIDOR_HEADING_ACTIONS|{'planned_turn_complete'}):
                    raise RuntimeError('micro correction scope violation')
                checker=getattr(self.backend,'check_micro',None)
                if checker is None:raise RuntimeError('live micro capability check unavailable')
                checker()
            required=self.cfg['request_timeout_s']+cfg['post_action_heading']['settle_s']+cfg['magnetic']['collect_s']
            if self.count>=self.cfg['max_body_macros_per_journey'] or self.nav_count>=cfg['max_extra_actions_per_journey'] or self.deadline-self.clock()<=required:return False
            # Recheck this same decision window after capability/time checks, not just current stream health.
            self.heading_guard.verify_pending_decision()
            # Direct finite backend call: verification is inside the guard, never recurse.
            self.completed_call(turn,self.cfg['request_timeout_s']);self.count+=1;self.nav_count+=1
            self.events.append({'magnetic_action':turn,'physical_success':'unobserved'})
            return True
        self.heading_guard.boundary_recovery_command0=getattr(self,'boundary_recovery_command0',False)
        self.heading_guard.pair_route_enabled=getattr(self,'pair_route_enabled',False)
        self.heading_guard.medium_turn_ready=getattr(self.backend,'medium_turn_ready',lambda:False)
        self.heading_guard.short_transfer_segment_pending=getattr(self,'short_transfer_segment_pending',False)
        def report_carry_wait(report):
            self.events.append({'post_action_heading':report})
            publish=getattr(self.backend,'report_progress',None)
            if publish is not None:
                command=getattr(self,'route_command','0')
                try:publish({'command':command,'phase':'cmd0_carry_wait','direction':'outbound','trial_target_tag':5,
                             'outbound_only':getattr(self,'outbound_only',command=='1'),'shared_outbound':command in ('1','2'),
                             'journey':getattr(self,'deliveries',0)+1,'heading_error_deg':report.get('error_deg')})
                except Exception:pass  # Status reporting is advisory.
        self.heading_guard.carry_boundary_wait_report=(report_carry_wait
            if getattr(self,'command0_scan_after_turn',False) else None)
        started=self.clock()
        try:
            sample_options={}
            if tag2_clearance_on_worsening:
                sample_options['tag2_clearance_on_worsening']=True
            if self.last_action_completed_monotonic is not None:
                sample_options['action_completed_monotonic']=self.last_action_completed_monotonic
            entry_context=getattr(self,'_entry_forward_context',None)
            if entry_context is not None:
                sample_options['entry_followup']=lambda:self._entry_followup(name,entry_context[0])
                trial=self.cfg.get('route_steps',{}).get('tag2_forward_trial') or {}
                if trial.get('enabled') is True and Cmd0Motion_ContinuousReady(self.backend):
                    sample_options['prefer_visual_followup']=lambda:self._entry_visual_before_magnetic_turn(
                        name,entry_context[0],nav,trial)
            else:
                expected_tag=getattr(self,'_route_end_expected_tag',None)
                if expected_tag is not None:
                    sample_options['entry_followup']=lambda:{'next_action':'scan_checkpoint',
                        'expected_tag':expected_tag,'position_confirmed':False}
                    tag4_settings=self.cfg.get('route_steps',{}).get('tag4_forward_trial') or {}
                    if (expected_tag==4 and allow_correction and tag4_settings.get('enabled') is True
                            and name in CARRY_HEADING_ACTIONS):
                        sample_options['prefer_visual_followup']=lambda:self._tag4_visual_before_magnetic_turn(
                            name,nav,tag4_settings)
            if getattr(self,'_transfer_turn_pending',False):
                sample_options['transfer_turn_pending']=True
            result=guard.after_action(name,self.load_kind,correct,allow_correction,**sample_options)
        except Exception as exc:
            report=getattr(exc,'report',None)
            if report is not None:self.events.append({'post_action_heading':report})
            if self.defer_magnetic_route_fault(exc):return
            raise
        finally:self.events.append({'stage_timing':{'stage':'post_action_heading','action':name,'elapsed_s':self.clock()-started}})
        self.last_heading_report=result
        self.events.append({'post_action_heading':result})

    def forward(self,steps):
        """按当前负载选择已有有限段；步数为名义推进相位，不是实测距离。"""
        self.prepare_heading_segment()
        if getattr(self,'_managed_transfer_forward',False) and self.load_kind=='single' and steps==9:
            if not getattr(self.backend,'is_simulation',False) and not Cmd0Motion_LegacyReady(self.backend):
                raise NotReady('中转区连续 9 步需要当前会话的 MCU 能力回执')
            plan=[TRANSFER_FORWARD9_ACTION]
        elif (getattr(self,'command0_scan_after_turn',False) and self.load_kind=='single' and steps==10
                and Cmd0Motion_ContinuousReady(self.backend)):
            plan=list(Cmd0Motion_ForwardActions(steps,continuous=True))
        else:
            plan=segments(steps,self.load_kind)
        if self.count+len(plan)>self.cfg['max_body_macros_per_journey']:raise RuntimeError('forward group exceeds remaining journey macro budget')
        self.report_command0_phase('cmd0_approach' if self.load_kind=='empty' else 'cmd0_carry')
        short_transfer=(getattr(self,'command0_scan_after_turn',False)
                        and plan==['carry_forward','carry_forward2'])
        for index,name in enumerate(plan):
            self.short_transfer_segment_pending=short_transfer and index==0
            self._transfer_turn_pending=getattr(self,'_managed_transfer_forward',False) and index==len(plan)-1
            try:self.action(name)
            finally:
                self.short_transfer_segment_pending=False
                self._transfer_turn_pending=False
    def carry_forward(self,steps):
        """抱1块前进：偶数2–80由8/4/2段组成，每段有完整起收脚。"""
        if self.load_kind!='single':raise RuntimeError('carry_forward requires expected single load')
        route_steps=self.cfg.get('route_steps')
        if (route_steps is not None and getattr(self,'command0_scan_after_turn',False)
                and getattr(self,'route_command','0')=='0' and type(steps) is int):
            phase=getattr(self,'_command0_phase_index',-1)
            if phase==_COMMAND0_PHASES.index('cmd0_pickup') and steps in (8,10):
                tuned=route_steps['transfer_forward_steps']
                self.events.append({'cmd0_transfer_forward':{'route_requested_steps':steps,'planned_steps':tuned}})
                self._managed_transfer_forward=True
                try:self.forward(tuned)
                finally:self._managed_transfer_forward=False
                self._managed_transfer_group_done=True
                return
            if self._managed_transfer_group_done and phase==_COMMAND0_PHASES.index('cmd0_carry') and steps==2:
                # Some preserved routes append the old two steps in a separate call.
                self.check()
                self.events.append({'cmd0_transfer_forward_skip':{'route_requested_steps':steps}})
                return
        self.forward(steps)
    def grab(self,count):
        """只支持抓1块；成功回报后更新期望负载，不伪造抓牢检测。"""
        if self.load_kind!='empty':raise RuntimeError('pickup requires expected empty load')
        if type(count) is not int or count!=1:raise NotReady('only single pickup is currently exported; never guess stack count')
        self.report_command0_phase('cmd0_pickup')
        self.action('grab_single');self.load_kind='single'
    def right(self,count=1):self.turn(count,'right_turn')
    def left(self,count=1):self.turn(count,'left_turn')
    def turn(self,count,name,*,target_delta_deg=None,nominal_step_deg=None):
        """完整转向宏按次数调用，不能把次数当作精确角度。"""
        paired = (self.load_kind == 'double' and getattr(self, 'pair_route_enabled', False)
                  and name in ('left_turn', 'right_turn'))
        if self.load_kind != 'single' and not paired:
            raise NotReady('current finite turn requires an expected supported load')
        if type(count) is not int or not 1<=count<=20:raise ValueError('turn repeat must be integer 1..20')
        self.report_command0_phase('cmd0_turn')
        cfg=getattr(self,'navigation_bundle',None) or {}
        target=cfg.get('planned_turn_groups',{}).get(name+':'+str(count))
        if target_delta_deg is not None and cfg.get('post_action_heading',{}).get('execute') is True:
            target=target_delta_deg
        if target is not None and not getattr(self.backend,'is_simulation',False):
            self.prepare_heading_segment()
        if target is not None and not self.magnetic_route_only and not getattr(self.backend,'is_simulation',False):
            self.heading_guard.plan_turn(target)
            early=cfg.get('early_visual_at_action_boundaries',False) is True
            diagnostic_boundary=self.cfg.get('transfer_test',{}).get('stop_after','terminal_scan')!='terminal_scan'
            self.heading_guard.planned_turn_recovery_scope=(
                getattr(self,'boundary_recovery_command0',False) is True and not diagnostic_boundary
                and cfg.get('adaptive_coarse_turn_selection',False) is True)
            self.heading_guard.terminal_stationary_scan_scope=(
                getattr(self,'command0_scan_after_turn',False) is True and not diagnostic_boundary)
            prediction_inputs={}
            if nominal_step_deg is not None:
                prediction_inputs['nominal_delta_deg']=nominal_step_deg if name=='right_turn' else -nominal_step_deg
            try:
                for i in range(count):
                    self.check_magnetic_before_motion(configured=True)
                    if self.magnetic_route_only:
                        self.action(name)
                        continue
                    if self.last_action_completed_monotonic is not None:
                        prediction_inputs['action_completed_monotonic']=self.last_action_completed_monotonic
                    try:
                        prediction=({'use_original_macro':True,'reason':'preserve explicitly selected diagnostic macro boundary'} if diagnostic_boundary else self.heading_guard.coarse_turn_prediction(name,self.load_kind,**prediction_inputs))
                    except MagneticDataUnavailable as exc:
                        if not self.defer_magnetic_route_fault(exc):raise
                        self.action(name)
                        continue
                    prediction['applied_to_route']=cfg.get('adaptive_coarse_turn_selection',False) is True
                    self.events.append({'coarse_turn_prediction':prediction})
                    if prediction['applied_to_route'] and not prediction['use_original_macro']:
                        if early:break
                        self.heading_turn_phase='planned_turn_complete'
                        self.monitor_heading(name)
                        break
                    self.heading_turn_phase='planned_turn_intermediate' if early else ('planned_turn_complete' if i==count-1 else 'planned_turn_intermediate')
                    self.action(name)
                    if early and not getattr(self,'command0_scan_after_turn',False) and self.try_early_visual_turn():return
                if early and not self.magnetic_route_only:
                    self.heading_turn_phase='planned_turn_complete'
                    self.monitor_heading(name)
            except Exception:
                self.boundary_visual_observation(name,fault_diagnostic=True)
                raise
            finally:
                self.heading_turn_phase=None
                self.heading_guard.planned_turn_recovery_scope=False
                self.heading_guard.terminal_stationary_scan_scope=False
            self.stationary_scan_after_turn()
        else:
            for _ in range(count):self.action(name)

    def boundary_visual_observation(self,name,fault_diagnostic=False,*,expected_tag=None):
        """No wait or motion; expose decoded IDs at every completed-action boundary."""
        entry_context=getattr(self,'_entry_forward_context',None)
        route_expected = expected_tag if expected_tag is not None else getattr(self,'_route_end_expected_tag',None)
        if (getattr(self,'command0_scan_after_turn',False) and not fault_diagnostic
                and entry_context is None and route_expected is None):
            return None
        cfg=getattr(self,'navigation_bundle',None) or {}
        if (cfg.get('early_visual_at_action_boundaries',False) is not True
                and (fault_diagnostic or entry_context is None and route_expected is None)):
            return None
        if getattr(self.backend,'is_simulation',False):return None
        try:
            nav=self.ensure_navigation_io();snapshot=getattr(nav.io,'boundary_visual_snapshot',None)
            completed=getattr(self,'last_action_completed_ros_ns',None)
            if snapshot is None or type(completed) is not int or completed<=0:return None
            settle=max(cfg.get('settle_s',0),cfg.get('post_action_heading',{}).get('settle_s',0))
            barrier=completed+int(settle*1e9)
            window=snapshot(barrier,self.check)
            from nav_observation import summarize
            target=2 if entry_context is not None else route_expected
            if target is None:
                target=self.cfg.get('transfer_test',{}).get('target_id')
            obs=summarize(window,target,cfg,barrier)
            camera_info=window.get('camera_info') or {}
            report=dict(action=name,target_id=target,scan=obs,stance_completed_ns=completed,
                camera_healthy=window.get('camera_healthy',False),
                camera_info=camera_info,camera_info_ready=camera_info.get('ready',False),
                contract_match=window.get('contract_match',False),
                boundary_valid=window.get('boundary_valid',False),after_ns=max(barrier,window.get('after_ns',0)),
                now_ns=window.get('now_ns'),camera_restart_ns=window.get('camera_restart_ns',0),waited_s=0.,
                recent_decoded_ids=window.get('recent_decoded_ids',[]),recent_ids_are_diagnostic_only=True,
                row_counts={k:window.get(k,0) for k in ('buffered_rows','current_rows','stale_rows','pre_boundary_rows','rejected')},
                fault_diagnostic=fault_diagnostic,correction_actions=0)
            self.events.append({'boundary_visual_observation':report})
            self.last_boundary_visual=report
            return report
        except Exception as exc:
            if not fault_diagnostic:
                self.check()  # Cancellation or deadline remains authoritative.
            self.events.append({'boundary_visual_observation':{'action':name,'fault_diagnostic':fault_diagnostic,
                'read_error':str(exc),'correction_actions':0}})
            return None

    def try_early_visual_turn(self):
        """Only a verified current target may take priority over further coarse turns."""
        cfg=getattr(self,'navigation_bundle',None) or {};report=getattr(self,'last_boundary_visual',None)
        if not report or report.get('fault_diagnostic') or self.load_kind!='single':return False
        v=cfg.get('visual_micro',{});target=self.cfg.get('transfer_test',{}).get('target_id')
        if not v.get('enabled') or not v.get('execute') or type(target) is not int or target!=v.get('confirmed_target_id'):return False
        if self.cfg.get('transfer_test',{}).get('stop_after','terminal_scan')!='terminal_scan':return False
        if not all(report.get(k) for k in ('boundary_valid','camera_healthy','camera_info_ready')):return False
        obs=report.get('scan',{})
        if target not in obs.get('ids',[]) or not obs.get('pose_usable') or not (obs.get('position_usable') or obs.get('heading_only')):return False
        if obs.get('heading_mad_deg',float('inf'))>v['max_heading_mad_deg']:return False
        goal=next((g for g in cfg['legs']['outbound'] if g['id']==target),None)
        if goal is None or goal.get('allow_turn') is not True:return False
        from nav_observation import wrap
        error=wrap(obs['heading_deg']-goal['tag_heading_deg'])
        if abs(error)>v['max_error_deg']:return False
        nav=self.ensure_navigation_io()
        live=getattr(nav.io,'visual_motion_ready',None)
        if live is None or not live(obs.get('latest_stamp_ns',0)):return False
        import copy
        anchor=copy.deepcopy(nav.mag.anchor)
        result=self.visual_checkpoint(target,'outbound') # rechecks fresh paired data and all motion guards
        self.events.append({'early_visual_turn_checkpoint':result})
        if result.get('heading_aligned') and result.get('magnetic',{}).get('status')=='updated':
            self.last_heading_report=dict(action='planned_turn_complete',status='visual_aligned',
                error_deg=result.get('after_error_deg',result.get('error_deg')),correction_actions=result.get('correction_actions',0))
            return True
        if nav.mag.anchor!=anchor:
            raise RuntimeError('partial early visual checkpoint changed fixed heading target; no further motion')
        if result.get('correction_actions',0):
            # Visual moved but did not reanchor. Refresh against the preserved fixed target.
            self.monitor_heading('planned_turn_intermediate',allow_correction=False)
            self.boundary_visual_observation('early_visual_partial')
        return False

    def stationary_scan_after_turn(self):
        # The optimized command 0 invokes its single terminal scan directly via routes.
        if getattr(self,'command0_scan_after_turn',False):return
        # A successful magnetic turn boundary, not a recovery path after a fault.
        cfg=getattr(self,'navigation_bundle',None) or {}
        if cfg.get('stationary_scan_after_planned_turn',False) is not True:return
        if self.cfg.get('transfer_test',{}).get('stop_after','terminal_scan')!='terminal_scan':return
        target=self.cfg.get('transfer_test',{}).get('target_id')
        report=getattr(self,'last_heading_report',{}) or {}
        residual=report.get('after_error_deg',report.get('error_deg'))
        reason=None
        if not cfg.get('visual_micro',{}).get('enabled'):reason='visual checkpoint disabled'
        elif type(target) is not int or target!=cfg.get('visual_micro',{}).get('confirmed_target_id'):reason='no explicitly confirmed target; no arbitrary ID selected'
        elif report.get('action')!='planned_turn_complete':reason='no successful completed-turn magnetic report'
        elif type(residual) not in (int,float) or not math.isfinite(residual) or abs(residual)>cfg.get('post_action_heading',{}).get('deadband_deg',0):reason='turn direction not verified inside deadband'
        if reason:
            self.events.append({'stationary_turn_checkpoint':{'status':'skipped','reason':reason,'correction_actions':0}})
            return
        self.check()
        scan=self.calibrate_tag(target,'outbound')
        self.events.append({'stationary_turn_checkpoint':{'status':'observed_before_forward','target_id':target,'scan':scan,'position_controlled':False}})

    def scan_ar(self):
        """保留旧路线调用点；显式任务启用新校准时在此插入Tag1检查点。"""
        self.report_command0_phase('cmd0_scan')
        bundle=getattr(self,'navigation_bundle',None)
        if bundle and bundle.get('enabled') and not getattr(self.backend,'is_simulation',False):
            target=self.cfg.get('transfer_test',{}).get('target_id')
            turn=getattr(self,'last_heading_report',None) or {}
            pending=(getattr(self,'command0_scan_after_turn',False) is True and
                     turn.get('terminal_visual_scan_allowed') is True)
            if pending and (target is None or not bundle.get('visual_micro',{}).get('enabled')):
                scan=self.raw_scan_ar()
                scan['navigation_checkpoint']=dict(status='partial',heading_aligned=False,
                    position_controlled=False,ready_for_obstacle=False,correction_actions=0,
                    reason='stationary scan only: turn target unverified; no bounded visual turn available')
            elif target is not None:
                scan=self.calibrate_tag(target,'outbound')
            else:scan=None
            if scan is not None:
                if pending:
                    scan['terminal_turn']={'magnetic_residual_deg':turn['remaining_heading_error_deg'],
                        'magnetic_target_reached':False,'forward_translation_allowed':False}
                return scan
            nav=self.ensure_navigation_io()
            window=nav.io.observe(None,0,bundle['observe_s'],self.check)
            if not window.get('camera_healthy',False):raise RuntimeError('camera or AR stream lacks new data; not ordinary Tag loss')
            scan=self.raw_scan_ar()
            scan['navigation_checkpoint']={'status':'blocked' if bundle.get('visual_micro',{}).get('enabled') else 'report_only','visual_micro':bundle.get('visual_micro',{}).get('enabled',False),'heading_aligned':False,'position_controlled':False,'ready_for_obstacle':False,'reason':'target ID not explicitly verified; no target selected and no correction','correction_actions':0,'search_actions':0,'magnetic':{'status':'not_updated','reason':'no reliable target alignment'}}
            return scan
        return self.raw_scan_ar()
    def raw_scan_ar(self):
        """只在路线显式调用处停稳并读取AR新帧；不把Tag ID当方块数。"""
        if not self.cfg['terminal_scan']['enabled']:return {'status':'disabled','ids':[],'observations':[]}
        self.wait(self.cfg['terminal_scan']['settle_s'])
        scanner=getattr(self.backend,'scan',None)
        if scanner is None:
            import importlib.util
            spec=importlib.util.spec_from_file_location('r3_terminal_ar',self.project/'r3_scan.py')
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);scanner=module.scan_tags
        from tag_reporting import reporting_options,enrich_for_project
        options=dict(self.cfg['terminal_scan']);options['min_fresh_frames']=reporting_options(self.project)['min_frames']
        result=scanner(options,self.check,options['timeout_s'])
        if self.cfg.get('transfer_test',{}).get('preflight_sensor_streams') and not getattr(self.backend,'is_simulation',False):
            nav=self.ensure_navigation_io()
            window=nav.io.observe(None,0,nav.cfg['observe_s'],self.check)
            if not window.get('camera_healthy',False):raise RuntimeError('terminal scan camera/AR stream fault; not ordinary missing Tag')
            result['scan_health']='fresh_camera_and_AR_stream'
        result=enrich_for_project(result,self.project,self.load_kind)
        self.add_pixel_diagnostics(result)
        self.events.append({'AR_scan':result})
        if result.get('status')=='cancelled':raise RuntimeError('AR observation cancelled')
        return result

    def add_pixel_diagnostics(self,scan):
        if not self.cfg.get('transfer_test',{}).get('pixel_diagnostic'):return
        from pixel_diagnostics import load_entrance_reference, summarize_window
        reference=load_entrance_reference(self.project)
        if getattr(self.backend,'is_simulation',False):
            scan['pixel_y_diagnostic']=summarize_window({'diagnostic_reasons':['simulation; no camera subscribed']});return
        from entry_alignment import load_bundle
        profile=dict(load_bundle(self.project)['profile'])
        profile['target_id']=self.cfg.get('transfer_test',{}).get('target_id')
        # The main fresh visual checkpoint already diagnosed ordinary loss. Do not
        # add a second long observation solely for optional image-y reporting.
        checkpoint=scan.get('navigation_checkpoint',{})
        if checkpoint.get('scan',{}).get('status')=='no_tag':
            self.check()
            scan['pixel_y_diagnostic']=summarize_window({'diagnostic_reasons':['ordinary target absent in main checkpoint; optional pixel reread skipped']})
            scan['pixel_y_diagnostic'].update(ordinary_target_loss=True,extra_observation=False)
            return
        injected=getattr(self.backend,'observe_entry',None)
        from entry_camera import CameraUnavailable
        try:
            if injected is not None:window=injected(profile,None,profile['window_timeout_s'])
            else:
                from entry_observer import observe_entry_window
                window=observe_entry_window(profile,self.cfg['terminal_scan'],self.check,timeout_s=profile['window_timeout_s'])
        except CameraUnavailable as exc:
            # Only this exact ordinary-loss event is nonfatal in this read-only
            # diagnostic. Clock/source/camera/transport failures remain faults.
            if str(exc)!='target lost; no search/retry motion':raise
            self.check()
            window={'target_lost':True,'diagnostic_reasons':[str(exc)]}
        self.check()
        scan['pixel_y_diagnostic']=summarize_window(window,reference=reference,load_kind=self.load_kind,
            image_topic=profile.get('camera',{}).get('image_topic'))
        if window.get('target_lost'):scan['pixel_y_diagnostic']['ordinary_target_loss']=True

    def strafe_left(self,count=1):self.strafe(count,'strafe_left_once')
    def strafe_right(self,count=1):self.strafe(count,'strafe_right_once')
    def strafe(self,count,name):
        """面对前方侧跨；一次为已有完整周期，距离未换算成米。"""
        if self.load_kind!='single':raise NotReady('current finite strafe supports single load only')
        if type(count) is not int or not 1<=count<=20:raise ValueError('strafe repeat must be 1..20')
        cfg=getattr(self,'navigation_bundle',None) or {}
        target=cfg.get('planned_turn_groups',{}).get(name+':'+str(count))
        if target is not None and not getattr(self.backend,'is_simulation',False):
            self.prepare_heading_segment();self.heading_guard.plan_turn(target)
            try:
                for i in range(count):
                    self.heading_turn_phase='planned_turn_complete' if i==count-1 else 'planned_turn_intermediate'
                    self.action(name)
            finally:self.heading_turn_phase=None
        else:
            for _ in range(count):self.action(name)
    def close_navigation(self):
        nav=getattr(self,'navigator',None)
        if nav is not None:nav.close();self.navigator=None
    def calibrate_tag(self,tag_id,direction='outbound'):
        """插码校准：普通失码/不可信位姿只返回续走，不吞动作或通信异常。"""
        cfg=getattr(self,'navigation_bundle',None)
        if not cfg or not cfg.get('enabled'):return self.raw_scan_ar()
        self.ensure_navigation_io()
        nav=self.visual_checkpoint(tag_id,direction) if cfg.get('visual_micro',{}).get('enabled') else self.navigator.checkpoint(tag_id,direction)
        obs=nav['scan']
        scan={'status':'recognized' if obs['ids'] else 'timeout','type':'AR_TAG_ALVAR','reason':nav['reason'],
              'topic':cfg['topic'],'ids':obs['ids'],'observed_ids':obs['observed_ids'],'observations':obs['observations'],
              'navigation_checkpoint':nav}
        from tag_reporting import enrich_for_project
        scan=enrich_for_project(scan,self.project,self.load_kind)
        if tag_id==1 and direction=='outbound' and getattr(self,'command0_scan_after_turn',False):
            from route_pose import RoutePose_Load, RoutePose_Measure
            nav['route_pose']=RoutePose_Measure(dict(obs,expected_load=self.load_kind),cfg,
                RoutePose_Load(self.project),tag_id,camera_info=nav.get('camera_info'))
        self.events.append({'AR_checkpoint':nav}) # retain the primary result even if optional reporting faults
        from entry_camera import CameraUnavailable
        try:
            self.add_pixel_diagnostics(scan)
        except CameraUnavailable as exc:
            from entrance_trial import EntranceTrial_IsEntryAccepted
            if (not getattr(self,'command0_scan_after_turn',False)
                    or not EntranceTrial_IsEntryAccepted(nav)):
                raise
            self.check()
            scan['pixel_y_diagnostic']={'status':'unavailable','diagnostic_only':True,
                'diagnostic_reasons':[str(exc)],'body_commands':0}
        return scan

    def visual_checkpoint(self,tag_id,direction='outbound',*,departure_pose=None):
        from visual_micro import VisualMicro, VisualMicroStopped, VisualMicro_ObserveWithoutMagnetic
        nav=self.ensure_navigation_io();cfg=nav.cfg
        if direction!='outbound':raise RuntimeError('v10 dedicated visual micro is only scoped to command-0 outbound endpoint')
        metric_departure=departure_pose is not None
        if metric_departure:
            if (tag_id not in (2,3,4,5) or departure_pose.get('tag_id')!=tag_id
                    or departure_pose.get('status')!='measured' or departure_pose.get('position_usable') is not True
                    or departure_pose.get('goal_allows_turn') is not True or self.load_kind!='single'):
                raise RuntimeError('corridor yaw correction requires a fresh calibrated single-load checkpoint pose')
            cfg=dict(cfg,relative_heading_reference={},visual_micro=dict(cfg['visual_micro'],confirmed_target_id=tag_id))
        elif type(tag_id) is not int or tag_id!=self.cfg.get('transfer_test',{}).get('target_id'):
            raise RuntimeError('visual target must match explicit runtime transfer_test.target_id')
        goal=next((g for g in cfg['legs'][direction] if g['id']==tag_id),None)
        if goal is None:raise RuntimeError('explicit target not in configured route; no arbitrary checkpoint')
        motion_deadline=self.deadline
        pair_route_enabled=getattr(self,'pair_route_enabled',False)
        def one(name,verify_pair):
            self.check()
            if self.load_kind!='single' and not (pair_route_enabled and self.load_kind=='double'):
                raise RuntimeError('visual micro load mismatch')
            checker=getattr(self.backend,'check_micro',None)
            if checker is None:raise RuntimeError('live dedicated micro capability unavailable')
            checker()
            needed=self.cfg['request_timeout_s']+cfg['settle_s']+cfg['observe_s']+cfg['magnetic']['collect_s']
            if self.count>=self.cfg['max_body_macros_per_journey'] or self.nav_count>=cfg['max_extra_actions_per_journey'] or motion_deadline-self.clock()<=needed:return None
            verify_pair()  # after backend checks, immediately before actual native submission
            self.completed_call(name,self.cfg['request_timeout_s']);self.count+=1;self.nav_count+=1
            self.events.append({'visual_micro_action':name,'angle_evidence':'source_nominal','physical_success':'unobserved'})
            return {'after_ns':nav.io.ros_now_ns()}
        if self.heading_guard is None:
            from heading_guard import HeadingGuard
            self.heading_guard=HeadingGuard(cfg,nav.io,nav.mag,self.check,self.clock,budget=self.micro_budget(),estimator=getattr(self.backend,'turn_estimator',None))
        self.heading_guard.observation_scope=(getattr(self.backend,'session',''),getattr(self.backend,'epoch',-1))
        defer_visual_prediction=(getattr(self,'boundary_recovery_command0',False) is True
            and self.cfg.get('transfer_test',{}).get('stop_after','terminal_scan')=='terminal_scan')
        defer_heading_at_deg=None
        if (tag_id==1 and not metric_departure and defer_visual_prediction and self._allows_missing_magnetic()
                and cfg['visual_micro'].get('execute')
                and (getattr(self,'transfer_mode',False) or getattr(self,'route_command',None) in ('1','2'))):
            from entrance_trial import TAG1_CLEARANCE_YAW_DEG
            defer_heading_at_deg=TAG1_CLEARANCE_YAW_DEG
        controller=VisualMicro(cfg,nav.io,nav.mag,self.micro_budget(),one,self.check,self.clock,self.load_kind,
                               motion_model=self.heading_guard,defer_predicted_overshoot=defer_visual_prediction,
                               allow_missing_magnetic=self._allows_missing_magnetic(), allow_double=pair_route_enabled,
                               defer_heading_at_deg=defer_heading_at_deg)
        forward_search=(not metric_departure and defer_visual_prediction and self._tag1_prefers_forward_search(tag_id)
                        and cfg['visual_micro'].get('execute') and cfg['visual_micro'].get('confirmed_target_id')==tag_id)
        approach={'target_id':tag_id,'completed_actions':[],'observations':[]}
        stance_evidence={}
        pickup_departure=(tag_id==1 and pair_route_enabled and not metric_departure
                          and self._allows_missing_magnetic())
        def passive_checkpoint():
            can_recover=self._allows_missing_magnetic() and (tag_id==1 or (metric_departure and tag_id!=5))
            if not can_recover:
                return VisualMicro_ObserveWithoutMagnetic(cfg,nav.io,tag_id,goal,
                    after_ns=nav.io.ros_now_ns(),deadline=self.deadline,check=self.check,clock=self.clock)
            recovery=None
            turns=[]
            retried_camera_gap=False
            idle_windows=0
            while True:
                self.check()
                windows=1 if forward_search else 1+cfg['visual_micro']['max_reobserve_windows']
                deadline=min(self.deadline,self.clock()+cfg['settle_s']+windows*cfg['observe_s'])
                try:
                    result=VisualMicro_ObserveWithoutMagnetic(cfg,nav.io,tag_id,goal,
                        after_ns=nav.io.ros_now_ns(),deadline=deadline,check=self.check,clock=self.clock,
                        accept_trusted_residual=True)
                except VisualMicroStopped as exc:
                    warnings=(exc.report or {}).get('events') or [{}]
                    last_window=warnings[-1].get('passive_camera_window',{})
                    if retried_camera_gap or last_window.get('recoverable') is not True:
                        raise
                    retried_camera_gap=True
                    self.events.append({'visual_camera_gap_recheck':{'tag_id':tag_id,**last_window}})
                    continue
                if (metric_departure or result.get('pose_accepted') or result.get('heading_aligned')
                        or result.get('status')=='blocked' or not result.get('scan_attempted')):
                    if result.get('pose_accepted'):
                        error=result.get('error_deg')
                        if turns:turns[-1]['after_error_deg']=error
                        improving=(not turns or isinstance(error,(int,float))
                                   and abs(error)<=abs(turns[-1]['before_error_deg'])-cfg['visual_micro']['minimum_improvement_deg'])
                        defer_turn=(defer_heading_at_deg is not None and type(error) in (int,float)
                                    and abs(error)>=defer_heading_at_deg)
                        if defer_turn:
                            result.update(correction_deferred=True,route_clearance_required=True,
                                          reason_zh='Tag 1 已确认有较大偏航，交由入口先判断让位余量。')
                        elif improving and not result.get('heading_aligned'):
                            turn=self.Route_TurnFromTagHeading(tag_id,error,result.get('scan',{}).get('latest_stamp_ns'))
                            if turn is not None:
                                turns.append(turn)
                                continue
                    if turns:
                        result.update(correction_actions=len(turns),correction_executed=True,camera_only_turns=turns)
                    return result
                if recovery is None:
                    self.report_command0_phase('cmd0_entry_reacquire',
                        reason_zh='磁力计暂不可用，正在原地重读 Tag 1。')
                    recovery={'target_id':tag_id,'windows':0,
                              'reason_zh':'磁力计暂时不可用，正在原地重新读取 Tag 1。'}
                    self.events.append({'terminal_tag_reacquisition':recovery})
                recovery['windows']+=result['observation_windows']
                recovery['last_scan']=result['scan']
                completed_groups=approach.get('motion_groups',0)
                if forward_search:
                    scan=result.get('scan') or {}
                    result=dict(result,tag_observation={'camera_healthy':scan.get('camera_healthy'),
                                                        'contract_match':scan.get('contract_match')})
                    result=self._advance_to_reacquire_tag1(nav,goal,result,self.deadline,approach,stance_evidence)
                    recovery['last_scan']=result['scan']
                if approach.get('motion_groups',0)>completed_groups:
                    idle_windows=0
                else:
                    idle_windows+=1
                    if idle_windows>=_TAG1_IDLE_REOBSERVATIONS:
                        reason=(_TAG1_POSE_UNCONFIRMED_REASON_ZH if tag_id in result['scan'].get('ids',())
                                else _TAG1_UNCONFIRMED_REASON_ZH)
                        recovery.update(status='stalled',reason_zh=reason)
                        return dict(result,reason=reason,reason_zh=reason)
        def record_checkpoint_failure(exc):
            self.events.append({'visual_checkpoint_failure':{'target_id':tag_id,'reason':str(exc),'report':getattr(exc,'report',None)}})
            self.boundary_visual_observation('visual_checkpoint_failure',fault_diagnostic=True)

        def observe_turned_stance():
            after=nav.io.ros_now_ns()
            deadline=min(self.deadline,self.clock()+cfg['settle_s']+2*cfg['observe_s'])
            try:
                return VisualMicro_ObserveWithoutMagnetic(cfg,nav.io,tag_id,goal,after_ns=after,
                    deadline=deadline,check=self.check,clock=self.clock,accept_trusted_residual=True)
            except VisualMicroStopped as exc:
                checked=exc.report or {}
                windows=checked.get('events') or [{}]
                if windows[-1].get('passive_camera_window',{}).get('recoverable') is not True:
                    raise
                return checked

        started=self.clock()
        try:
            if self.magnetic_route_only:
                result=passive_checkpoint()
            elif metric_departure:
                result=controller.checkpoint(tag_id,goal,after_ns=nav.io.ros_now_ns(),deadline=self.deadline)
            elif getattr(self,'command0_scan_after_turn',False) is True:
                after_ns=nav.io.ros_now_ns()
                if type(after_ns) is not int or after_ns<=0:
                    raise RuntimeError('fresh terminal scan clock unavailable')
                deadline=min(self.deadline,self.clock()+cfg['checkpoint_timeout_s'])
                motion_deadline=deadline
                steps=[]
                keep_searching=(defer_visual_prediction and tag_id==1 and cfg['visual_micro'].get('execute')
                    and cfg['visual_micro'].get('confirmed_target_id')==tag_id)
                def needs_reobserve(result):
                    status=result.get('scan',{}).get('status')
                    return status in _TAG1_REOBSERVE_SCANS if forward_search else status=='no_tag'

                def reobserve_missing(result):
                    nonlocal deadline,motion_deadline
                    if not needs_reobserve(result):return result
                    moved=result.get('correction_executed')
                    if moved:
                        steps.append(result)
                        result=dict(result,correction_executed=False,correction_actions=0)
                    quick_entry=(tag_id==1 and self._allows_missing_magnetic())
                    if not forward_search or moved or not result.get('tag_observation'):
                        result=self._reobserve_terminal_tag(nav,goal,result,deadline,controller=controller,
                            wait_s=_TAG1_QUICK_SCAN_S if forward_search else 5.,quick_id=quick_entry)
                    if forward_search and needs_reobserve(result):
                        result=self._advance_to_reacquire_tag1(nav,goal,result,deadline,approach,stance_evidence,
                                                                 controller=controller)
                    if not keep_searching or not needs_reobserve(result):return result
                    reason = ('检测器读到了 Tag 1，正在原地复核有效帧。'
                              if self._entry_waits_for_decoded_tag(result)
                              else '尚未确认 Tag 1，留在入口处等待新画面。')
                    if approach.get('retreat_action'):
                        reason='已沿刚走过的两步后退，正在复核 Tag 1。'
                    elif approach['completed_actions']:
                        reason=f"已补进 {2*len(approach['completed_actions'])} 步，正在复核 Tag 1。"
                    self.report_command0_phase('cmd0_entry_reacquire',reason_zh=reason)
                    recovery={'target_id':tag_id,'windows':0,'last_scan':result['scan']}
                    self.events.append({'terminal_tag_reacquisition':recovery})
                    idle_windows=0
                    while needs_reobserve(result) and idle_windows<_TAG1_IDLE_REOBSERVATIONS:
                        self.check()
                        if self.clock()>=self.deadline:
                            raise RuntimeError('mission time budget reached while reacquiring entrance Tag 1')
                        deadline=min(self.deadline,self.clock()+cfg['checkpoint_timeout_s'])
                        motion_deadline=deadline
                        if result.get('correction_executed'):
                            steps.append(result)
                            result=dict(result,correction_executed=False,correction_actions=0)
                        completed_groups=approach.get('motion_groups',0)
                        result=self._reobserve_terminal_tag(nav,goal,result,deadline,controller=controller,
                            record=False,wait_s=_TAG1_QUICK_SCAN_S if forward_search else 5.,quick_id=quick_entry,
                            reuse_settled_cache=not forward_search)
                        if forward_search and not result.get('correction_executed'):
                            result=self._advance_to_reacquire_tag1(nav,goal,result,deadline,approach,stance_evidence,
                                                                     controller=controller)
                        recovery['windows']+=1
                        recovery['last_scan']=result['scan']
                        if 'tag_observation' in result:
                            recovery['last_observation']=result['tag_observation']
                        if approach.get('motion_groups',0)>completed_groups or result.get('correction_executed'):
                            idle_windows=0
                        else:
                            idle_windows+=1
                    if needs_reobserve(result):
                        nominal = self._tag1_nominal_entry(nav,result) if forward_search else None
                        if nominal is not None:
                            reason='已识别 Tag 1，画面中的码位置稳定。沿当前磁航向按基准步数进入下一段。'
                            recovery.update(status='nominal_entry_accepted',reason_zh=reason)
                            return dict(result,tag1_nominal_entry=True,nominal_entry_evidence=nominal,
                                        reason=reason,reason_zh=reason)
                        reason=(_TAG1_POSE_UNCONFIRMED_REASON_ZH if 1 in result['scan'].get('ids',())
                                else _TAG1_UNCONFIRMED_REASON_ZH)
                        recovery.update(status='stalled',reason_zh=reason)
                        result=dict(result,reason=reason,reason_zh=reason)
                    return result
                if forward_search:
                    entry_wait=min(_TAG1_QUICK_SCAN_S,self.cfg['terminal_scan']['timeout_s'])
                    result=self._reobserve_terminal_tag(nav,goal,{'scan':{'status':'no_tag'}},
                        deadline,controller=controller,wait_s=entry_wait,quick_id=True)
                else:
                    result=controller.checkpoint(tag_id,goal,after_ns=after_ns,deadline=deadline)
                if keep_searching and not forward_search and goal['allow_turn'] and not result.get('fresh_target_detected'):
                    self.report_command0_phase('cmd0_entry_reacquire')
                    if not self._entry_waits_for_decoded_tag(result):
                        result=self._reacquire_terminal_tag(nav,controller,goal,result,deadline)
                result=reobserve_missing(result)
                search_actions=result.get('search_actions',0)
                stationary_rechecks=0
                while ((result.get('correction_executed') or (keep_searching and
                        (result.get('reason')=='no safe predicted visual correction'
                         or result.get('stationary_reobserve_required'))
                        and stationary_rechecks < cfg['visual_micro']['max_reobserve_windows'])) and result.get('pose_accepted')
                       and not result.get('heading_aligned') and not result.get('stationary_recheck_inconclusive')
                       and result.get('scan',{}).get('pose_usable')
                       and self.micro_budget().actions < self.micro_budget().max_actions
                       and self.clock()+cfg['settle_s']+cfg['observe_s']+cfg['magnetic']['collect_s']<deadline):
                    self.check()
                    if result.get('correction_executed'):steps.append(result)
                    else:stationary_rechecks+=1
                    result=controller.checkpoint(tag_id,goal,after_ns=nav.io.ros_now_ns(),deadline=deadline)
                    result=reobserve_missing(result)
                if stationary_rechecks:result['stationary_heading_rechecks']=stationary_rechecks
                if (pickup_departure and not result.get('heading_aligned') and not result.get('correction_executed')
                        and result.get('pose_accepted') and result.get('reason')=='no safe predicted visual correction'
                        and result.get('decision_fusion',{}).get('source')=='trusted_tag_heading'):
                    prior = result
                    turn = self.Route_TurnFromTagHeading(tag_id,prior.get('error_deg'),
                        prior.get('scan',{}).get('latest_stamp_ns'),reserved_actions=1)
                    if turn is not None:
                        checked = observe_turned_stance()
                        if checked.get('pose_accepted'):
                            result = checked
                            turn['after_error_deg'] = checked['error_deg']
                        else:
                            result = dict(checked,heading_unknown_after_turn=True,
                                reason_zh='入口转向已完成，正在用转后的胸部相机画面重新定位。')
                        result.update(correction_executed=True,correction_actions=1,
                            camera_only_turns=[turn],camera_only_recheck=checked.get('scan'),
                            stationary_heading_rechecks=stationary_rechecks)
                        self.events.append({'tag1_departure_visual_recovery':dict(turn,
                            heading_reconfirmed=checked.get('pose_accepted') is True)})
                if keep_searching and result.get('heading_unknown_after_turn'):
                    checked=observe_turned_stance()
                    result['post_turn_recheck']=checked
                    for key in ('scan','camera_info','fresh_target_detected','pose_accepted','heading_aligned',
                                'error_deg','decision_fusion','status','reason'):
                        if key in checked:
                            result[key]=checked[key]
                    result['heading_unknown_after_turn']=not checked.get('pose_accepted')
                    if checked.get('pose_accepted'):
                        result['after_error_deg']=checked['error_deg']
                        result.pop('stationary_recheck_inconclusive',None)
                        result['reason_zh']='入口转向后已重新读到 Tag 1，按当前视觉航向进入下一段。'
                    else:
                        result['reason_zh']='入口转向后仍未取得可信的新位姿，已记录复测结果。'
                if steps:
                    steps.append(result)
                    result=dict(result,correction_steps=steps,
                                search_actions=search_actions,
                                correction_actions=sum(r.get('correction_actions',0) for r in steps),
                                correction_executed=any(r.get('correction_executed') for r in steps))
            else:
                result=controller.checkpoint(tag_id,goal)
        except MagneticDataUnavailable as exc:
            if not self.defer_magnetic_route_fault(exc):raise
            if exc.report is not None:self.events.append({'visual_checkpoint_before_magnetic_loss':exc.report})
            try:
                result=passive_checkpoint()
            except Exception as passive_error:
                record_checkpoint_failure(passive_error)
                raise
        except Exception as exc:
            record_checkpoint_failure(exc)
            raise
        finally:self.events.append({'stage_timing':{'stage':'visual_checkpoint','target_id':tag_id,'elapsed_s':self.clock()-started}})
        if (tag_id==1 and not metric_departure and self._allows_missing_magnetic()
                and result.get('pose_accepted') is True and result.get('status')=='partial'
                and result.get('decision_fusion',{}).get('source')=='trusted_tag_heading'
                and result.get('scan',{}).get('pose_usable') is True
                and result.get('reason')!='configured checkpoint forbids turns; no micro budget consumed'):
            # The camera accepted Tag 1. A remaining heading residual stays diagnostic.
            result['heading_residual_advisory']=True
            if not result.get('reason_zh') or result.get('stationary_reobserve_required'):
                result['reason_zh']='Tag 1 已确认，保留实测航向余差并继续原路线。'
        if result.get('heading_aligned') and self.magnetic_route_only:self.route_checkpoint_observed(tag_id)
        result['visual_micro']=True
        if self.heading_guard is not None:self.heading_guard.latched=False
        return result

    def _tag1_reacquire_decision(self,nav,result,approach,stance_evidence):
        """Use partial rows only to choose a bounded change of view."""
        from route_pose import RoutePose_PartialCheckpointGeometry

        decision=_Tag1ReacquireResult
        scan=result.get('scan') or {}
        evidence=result.get('tag_observation') or {}
        stance=self.last_action_completed_ros_ns
        if not stance_evidence or stance_evidence['stance_ns']!=stance:
            stance_evidence.update(stance_ns=stance,rows={},seen=False,other_id=False)
        rows=stance_evidence['rows']
        if (evidence.get('camera_healthy') is not True or evidence.get('contract_match') is not True
                or evidence.get('camera_ready_for_motion') is False):
            rows.clear()
            return decision.WAIT_CAMERA,None
        observed=scan.get('observed_ids',())
        decoded=evidence.get('decoded_expected_frames')
        stance_evidence['other_id'] |= any(tag!=1 for tag in observed)
        stance_evidence['seen'] |= (1 in observed or bool(scan.get('observations'))
                                   or type(decoded) is int and decoded>0)
        if (scan.get('status')!='no_tag' or stance_evidence['other_id']
                or approach.get('retreat_action')):
            return decision.WAIT_AMBIGUOUS,None
        if not stance_evidence['seen']:
            if approach.get('far_probe_done'):
                return decision.WAIT_AMBIGUOUS,None
            return decision.ADVANCE_UNSEEN,None

        after,observed_at=scan.get('after_ns'),scan.get('now_ns')
        now=nav.io.ros_now_ns()
        if (type(stance) is not int or type(now) is not int or now<=stance):
            return decision.WAIT_AMBIGUOUS,None
        restart=(evidence.get('camera_window_status') or {}).get('restart_barrier_ns',0)
        floor=max(stance,restart) if type(restart) is int else stance
        earliest=now-nav.cfg['max_age_s']*1e9
        for stamp in tuple(rows):
            if stamp<=floor or stamp<earliest:
                del rows[stamp]
        if type(after) is int and type(observed_at) is int and floor<=after<=observed_at<=now:
            for row in scan.get('observations',()):
                stamp=row.get('stamp_ns')
                if type(stamp) is int and max(floor,after)<stamp<=observed_at and stamp>=earliest:
                    rows[stamp]=row
        geometry=RoutePose_PartialCheckpointGeometry(rows.values(),nav.cfg,1)
        if geometry is None:
            return decision.WAIT_AMBIGUOUS,None
        near,far=geometry['distance_bounds_m']
        left,right=geometry['lateral_bounds_m']
        if far-near>.025 or max(abs(left),abs(right))>.10 or geometry['max_abs_heading_deg']>12.:
            return decision.WAIT_AMBIGUOUS,geometry
        camera_ready=getattr(nav.io,'visual_motion_ready',None)
        live=camera_ready is not None and camera_ready(geometry['latest_stamp_ns'])
        if .02<=near<=far<=.11:
            if (live and approach.get('last_forward_completed_ros_ns')==stance
                    and Cmd0Motion_ContinuousReady(self.backend)):
                return decision.RETREAT_NEAR,geometry
            return decision.WAIT_NEAR,geometry
        # Use the configured upper travel bound without weakening the default margin.
        settings=self.cfg.get('route_steps',{}).get('tag2_forward_trial',{})
        travel=settings.get('approach_two_step_distance_m',
                            settings.get('two_step_distance_m',(0.,_TAG1_PROBE_DEFAULT_MAX_TRAVEL_M)))
        far_threshold=max(.34 if geometry['samples']==1 else .29, travel[1]+.16)
        if live and near>=far_threshold and far<=.55:
            if not approach.get('far_probe_done'):
                return decision.ADVANCE_FAR,geometry
            previous = approach.get('last_far_pose') or {}
            camera = result.get('camera_info') or {}
            if (geometry['samples'] >= 2 and previous.get('fingerprint')
                    and camera.get('ready') is True and camera.get('fingerprint') == previous['fingerprint']
                    and previous.get('restart_barrier_ns') == restart
                    and approach.get('last_forward_completed_ros_ns') == stance
                    and previous['stamp_ns'] < stance < geometry['latest_stamp_ns']
                    and previous['near_m'] - far >= max(.002, far - near, previous['spread_m'])):
                return decision.ADVANCE_FAR,geometry
        return decision.WAIT_AMBIGUOUS,geometry

    def _advance_to_reacquire_tag1(self,nav,goal,result,deadline,approach,stance_evidence,*,controller=None):
        """Change view by two steps and read a new camera window before proceeding."""
        actions=approach['completed_actions']
        reserve=self.cfg['request_timeout_s']+nav.cfg['settle_s']+_TAG1_QUICK_SCAN_S
        while (self.clock()+reserve<deadline and self.count+1<self.cfg['max_body_macros_per_journey']
               and self.nav_count<nav.cfg['max_extra_actions_per_journey']):
            if (result.get('scan',{}).get('status') not in _TAG1_REOBSERVE_SCANS
                    or result.get('correction_executed')):
                break
            self.check()
            outcome,geometry=self._tag1_reacquire_decision(nav,result,approach,stance_evidence)
            if 'decisions' not in approach:
                self.events.append({'terminal_tag_forward_reacquisition':approach})
                approach['decisions']=[]
            approach['decisions'].append({'status':outcome.value,'partial_geometry':geometry})
            if outcome is _Tag1ReacquireResult.RETREAT_NEAR:
                action='cmd0_carry_backward2'
                reason='Tag 1 有近距迹象，沿刚走过的两步后退，再用新画面确认。'
            elif outcome in (_Tag1ReacquireResult.ADVANCE_UNSEEN,_Tag1ReacquireResult.ADVANCE_FAR):
                if len(actions)>=_TAG1_APPROACH_GROUPS:
                    break
                action='carry_forward2'
                reason=('Tag 1 部分帧显示码仍偏远，前探两步再扫。'
                        if outcome is _Tag1ReacquireResult.ADVANCE_FAR else
                        f'Tag 1 尚未读到，按入口路线前探两步再扫（第 {len(actions)+1} 组）。')
            else:
                break
            self.report_command0_phase('cmd0_entry_reacquire',reason_zh=reason)
            self.action(action,monitor_only_as='carry_forward2')
            self.nav_count+=1
            approach['motion_groups']=approach.get('motion_groups',0)+1
            stance_evidence.clear()
            if outcome is _Tag1ReacquireResult.RETREAT_NEAR:
                approach['retreat_action']=action
            else:
                actions.append(action)
                approach['last_forward_completed_ros_ns']=self.last_action_completed_ros_ns
                if outcome is _Tag1ReacquireResult.ADVANCE_FAR:
                    approach['far_probe_done']=True
                    near,far=geometry['distance_bounds_m']
                    camera=result.get('camera_info') or {}
                    window=(result.get('tag_observation') or {}).get('camera_window_status') or {}
                    approach['last_far_pose']={'near_m':near,'spread_m':far-near,
                        'stamp_ns':geometry['latest_stamp_ns'],
                        'fingerprint':camera.get('fingerprint') if camera.get('ready') is True else None,
                        'restart_barrier_ns':window.get('restart_barrier_ns',0)}
            result=self._reobserve_terminal_tag(nav,goal,result,deadline,controller=controller,
                record=False,wait_s=_TAG1_QUICK_SCAN_S,quick_id=True)
            approach['observations'].append(dict(result.get('tag_observation') or {},
                                                 scan_status=result.get('scan',{}).get('status')))
        if actions:
            result['tag1_reacquire_forward_steps']=2*len(actions)
        if approach.get('retreat_action'):
            result['tag1_reacquire_backward_steps']=2
        return result

    def _tag1_prefers_forward_search(self, tag):
        supported_load = self.load_kind == 'single' or (
            self.load_kind == 'double' and getattr(self,'pair_route_enabled',False))
        return (tag == 1 and getattr(self,'route_command','0') in ('0','1','2')
                and supported_load and self._allows_missing_magnetic())

    def _tag1_nominal_entry(self, nav, result):
        """Accept the known entrance with stable current ID/position and an existing heading reference."""
        from corridor_trial import CorridorTrial_HeadingSnapshot

        scan = result.get('scan') or {}
        evidence = result.get('tag_observation') or {}
        camera = result.get('camera_info') or {}
        contract = nav.cfg['pose_contract']
        pinned = nav.cfg['visual_micro']['camera_info']
        stance, after, now = self.last_action_completed_ros_ns, scan.get('after_ns'), scan.get('now_ns')
        if (scan.get('status') not in ('id_seen_pose_untrusted','unreliable') or scan.get('ids') != [1]
                or scan.get('observed_ids') != [1] or self.magnetic_route_only
                or (nav.mag.anchor or {}).get('load') != self.load_kind
                or evidence.get('camera_healthy') is not True or evidence.get('contract_match') is not True
                or evidence.get('camera_ready_for_motion') is False
                or pinned.get('source_confirmed') is not True or camera.get('ready') is not True
                or not camera.get('fingerprint') or camera['fingerprint'] != pinned.get('expected_fingerprint')
                or contract.get('capture_time_confirmed') is not True
                or (contract.get('body_forward'),contract.get('body_right')) != ('+x','-y')
                or any(type(value) is not int for value in (stance,after,now)) or not 0 < stance <= after < now):
            return None
        heading = CorridorTrial_HeadingSnapshot(self.last_heading_report or {},after=True,load=self.load_kind)
        completed = self.last_action_completed_monotonic
        if (heading is None or type(completed) not in (int,float) or not completed <= heading[1] <= self.clock()
                or abs(heading[0]) > min(16.,2*nav.cfg['post_action_heading']['deadband_deg'])):
            return None
        frame = contract['frame']
        positions = {}
        for row in scan.get('observations', ()):
            stamp = row.get('stamp_ns')
            if (row.get('id') != 1 or row.get('frame_id') != frame or row.get('headers_disagree')
                    or row.get('source_publisher') != contract['publisher'] or row.get('source_topic') != nav.cfg['topic']
                    or type(stamp) is not int or not 0 <= now-stamp <= nav.cfg['max_age_s']*1e9 or stamp <= after):
                continue
            position = row.get('position_raw_m') or {}
            coordinates = tuple(position.get(axis) for axis in ('x','y','z'))
            if any(type(value) not in (int,float) or not math.isfinite(value) for value in coordinates):
                return None
            positions[stamp] = coordinates
        if len(positions) < nav.cfg['min_frames']:
            return None
        latest = max(positions)
        live = getattr(nav.io,'visual_motion_ready',None)
        if live is None or not live(latest):
            return None
        x, y, z = zip(*positions.values())
        if (not .14 <= min(x) <= max(x) <= .22 or max(map(abs,y)) > .06
                or any(max(values)-min(values) > .025 for values in (x,y,z))):
            return None
        return {'accepted_frames':len(positions),'body_forward_bounds_m':(min(x),max(x)),
                'body_side_bounds_m':(min(y),max(y)),'magnetic_heading_error_deg':heading[0],
                'latest_stamp_ns':latest}

    @staticmethod
    def _entry_waits_for_decoded_tag(result):
        evidence=result.get('tag_observation') or {}
        decoded=evidence.get('decoded_expected_frames')
        return (evidence.get('camera_healthy') is True and evidence.get('contract_match') is True
                and type(decoded) is int and decoded>0 and result.get('scan',{}).get('status')=='no_tag')

    def _reacquire_terminal_tag(self,nav,controller,goal,result,deadline):
        """Command 0 stays at its endpoint and retries a missing entry tag by turning."""
        from mission_checkpoints_v5 import Checkpoint_ReadHeading, Checkpoint_TrySweepTurn

        tag=goal['id']
        cfg=nav.cfg
        sweep={'origin':None,'turns':0,'blocked':False}
        measured = self._tag1_prefers_forward_search(tag)
        if measured:
            sweep['target_offsets'] = (-18.,18.,0.)
            telemetry = {'target_id': tag, 'observations': []}
            self.events.append({'terminal_tag_sweep': telemetry})
        view_wait=max(2.,cfg['observe_s']) if measured else cfg['observe_s']
        reserve=self.cfg['request_timeout_s']+view_wait+cfg['settle_s']+2*cfg['magnetic']['collect_s']

        def camera_ready():
            snapshot=getattr(nav.io,'preflight_health_snapshot',None)
            if snapshot is None:
                return False
            health=snapshot()
            return health.get('camera_healthy',False) and health.get('camera_info',{}).get('ready',False)

        def read_heading():
            if self._allows_missing_magnetic():
                _, sample=self._heading_controller(nav).sample(self.load_kind,
                    action_completed_monotonic=self.last_action_completed_monotonic,stationary_retry=True)
                return sample
            return Checkpoint_ReadHeading(self,nav,self.check)

        while (result.get('scan',{}).get('status')=='no_tag' and self.clock()+reserve<deadline
               and not (measured and self._entry_waits_for_decoded_tag(result))):
            moved=Checkpoint_TrySweepTurn(self,nav,sweep,check_action=camera_ready,
                read_heading=read_heading,
                check_capture=lambda:self.check_magnetic_before_motion(heading_feedback=True),
                accept_verified_angle=self._allows_missing_magnetic())
            if measured:
                telemetry.update(turns=sweep['turns'], offset_deg=sweep.get('offset_deg'),
                                 coverage_deg=sweep.get('coverage_deg'), status=sweep.get('status'),
                                 reason=sweep.get('reason'))
            if sweep['blocked']:
                raise RuntimeError('terminal Tag sweep feedback rejected; no further motion')
            if not moved:
                break
            if measured:
                angle=sweep.get('offset_deg')
                if angle is not None:
                    self.report_command0_phase('cmd0_entry_reacquire',
                        reason_zh=f'正扫入口两侧，当前相对起点 {angle:+.0f}°。')
                result=self._reobserve_terminal_tag(nav,goal,result,deadline,controller=controller,
                                                    record=False, wait_s=view_wait, quick_id=True)
                scan=result.get('scan') or {}
                telemetry['observations'].append({'offset_deg':angle,'seen_tags':scan.get('observed_ids',[]),
                                                   'status':scan.get('status'),**result.get('tag_observation',{})})
            else:
                result=controller.checkpoint(tag,goal,after_ns=nav.io.ros_now_ns(),deadline=deadline)
        if sweep['turns']:
            scope='measured two-sided entrance sweep' if measured else 'bounded in-place sweep'
            result=dict(result,search_actions=sweep['turns'],search_scope=scope)
        return result

    def _reobserve_terminal_tag(self,nav,goal,result,deadline,*,controller=None,record=True,wait_s=5.,
                                quick_id=False,reuse_settled_cache=True):
        """Read one fresh stationary Tag window after ordinary loss."""
        previous=result.get('scan',{})
        quick_entry=quick_id and goal['id']==1
        statuses=_TAG1_REOBSERVE_SCANS if quick_entry else ('no_tag',)
        if previous.get('status') not in statuses:
            return result
        self.check()
        if quick_id:
            self.settle_since_last_completed(nav.cfg['settle_s'])
        remaining=min(wait_s,deadline-self.clock())
        if remaining<=0:
            return result
        after_ns=nav.io.ros_now_ns()
        if type(after_ns) is not int or after_ns<=0:
            raise RuntimeError('fresh terminal scan clock unavailable')
        stance=self.last_action_completed_ros_ns
        if (quick_entry and reuse_settled_cache and nav.cfg.get('reuse_post_completion_observations') is True
                and type(stance) is int):
            settled=stance+int(nav.cfg['settle_s']*1e9)
            if 0<stance<=settled<=after_ns:
                # Reuse only fresh frames captured after this stance settled, including partial windows.
                after_ns=settled
        target=goal['id']
        observe=getattr(nav.io,'observe_checkpoint_id',nav.io.observe) if quick_id else nav.io.observe
        window=observe(target,after_ns,remaining,self.check)
        self.check()
        evidence={'camera_healthy':window.get('camera_healthy') is True,
                  'contract_match':window.get('contract_match') is True,
                  'decoded_expected_frames':window.get('detector_observed_expected_frames',
                                                       window.get('detector_expected_frames')),
                  'receiver_rejections_total':window.get('rejected'),
                  'camera_window_status':window.get('camera_window_status')}
        if self._allows_missing_magnetic():
            from visual_micro import VisualCamera_DiagnoseWindow
            camera=window.get('camera_info',{})
            camera_config=nav.cfg['visual_micro']['camera_info']
            issue=VisualCamera_DiagnoseWindow(window,camera,camera_config['expected_fingerprint'])
            if issue is not None:
                evidence['camera_ready_for_motion']=False
                if record:
                    self.events.append({'terminal_stationary_reobserve':{'target_id':target,
                        'body_commands':0,'reason_zh':issue[1]}})
                if not issue[2]:
                    raise RuntimeError(issue[0])
                return dict(result,tag_observation=evidence)
        if not window.get('camera_healthy'):
            raise RuntimeError('terminal stationary retry camera/AR stream fault')
        if nav.cfg['visual_micro'].get('execute') and not window.get('camera_info',{}).get('ready'):
            raise RuntimeError('terminal stationary retry CameraInfo unavailable')
        from nav_observation import summarize
        scan=summarize(window,target,nav.cfg,after_ns)
        if quick_entry:
            scan.update(after_ns=max(after_ns,window.get('after_ns',after_ns)),now_ns=window.get('now_ns'))
        seen=target in scan['observed_ids']
        evidence['accepted_expected_frames']=scan.get('samples')
        if record:
            self.events.append({'terminal_stationary_reobserve':{'target_id':target,
                'max_wait_s':remaining,'prior_ids':previous.get('observed_ids',[]),
                'observed_ids':scan['observed_ids'],'scan_status':scan['status'],'body_commands':0,**evidence}})
        report=dict(result,scan=scan,status='partial',pose_accepted=False,heading_aligned=False,
            position_controlled=False,ready_for_obstacle=False,
            expected_id=target, tag_observation=evidence, camera_info=window.get('camera_info',{}),
            fresh_target_detected=seen or result.get('fresh_target_detected',False),
            reason=('expected Tag seen in final stationary window; no alignment motion'
                    if seen else 'expected Tag absent after final stationary observation'))
        if (controller is None or not seen or not scan.get('pose_usable')
                or not (scan.get('position_usable') or scan.get('heading_only'))
                or goal.get('allow_turn') is not True or not nav.cfg['visual_micro'].get('execute')):
            return report
        budget=self.micro_budget()
        cfg=nav.cfg
        reserve=self.cfg['request_timeout_s']+2*(cfg['settle_s']+cfg['observe_s']+cfg['magnetic']['collect_s'])
        if budget.locked or self.clock()+reserve>=deadline:
            return report
        checkpoint_after_ns=nav.io.ros_now_ns()
        if type(checkpoint_after_ns) is not int or checkpoint_after_ns<after_ns:
            raise RuntimeError('terminal stationary retry ROS clock moved backwards')
        window_after_ns=window.get('after_ns')
        if (quick_id and cfg.get('reuse_post_completion_observations') is True
                and type(window_after_ns) is int and after_ns<=window_after_ns<=checkpoint_after_ns):
            # Same settled stance. The visual controller still checks current age and source.
            checkpoint_after_ns=window_after_ns
            evidence['same_station_window_allowed']=True
        verified=controller.checkpoint(target,goal,after_ns=checkpoint_after_ns,deadline=deadline)
        report=dict(verified,fresh_target_detected=True,search_actions=result.get('search_actions',0),
                    tag_observation=evidence)
        if report['search_actions'] and 'search_scope' in result:
            report['search_scope']=result['search_scope']
        return report

    def align_entry(self,scan):
        """人可调的段末入口：示教参考x/角度归零，y只监测；默认不运动。"""
        bundle=getattr(self,'entry_alignment_bundle',None)
        if not bundle or not bundle['profile'].get('enabled'):return {'status':'report_only','reason':'not configured','ready_for_obstacle':False,'actions':[]}
        if getattr(self.backend,'is_simulation',False):return {'status':'report_only','reason':'simulation: no camera subscribers or correction body','ready_for_obstacle':False,'actions':[]}
        from entry_alignment import align_to_entry,ACTIONS
        profile=bundle['profile'];injected=getattr(self.backend,'observe_entry',None)
        def observe(after_ns=None,timeout_s=5):
            if injected is not None:return injected(profile,after_ns,timeout_s)
            from entry_observer import observe_entry_window
            return observe_entry_window(profile,self.cfg['terminal_scan'],self.check,after_ns,timeout_s,strict_target=profile['mode']=='execute_calibrated')
        def one(action,remaining):
            if self.load_kind!=profile['expected_load']:raise RuntimeError('entry action load mismatch')
            self.action(ACTIONS[action],min(self.cfg['request_timeout_s'],int(remaining)))
            self.wait(profile['settle_s']);self.check()
            time_source=getattr(self.backend,'ros_now_ns',None)
            if time_source is None:
                import rospy
                time_source=lambda:rospy.Time.now().to_nsec()
            end=time_source()
            if type(end) is not int or end<=0:raise RuntimeError('ROS action-end time unavailable')
            return {'protocol_completed':True,'after_ns':end,'physical_success':'unobserved'}
        result=align_to_entry(bundle,observe,one,self.check,clock=self.clock)
        self.events.append({'entry_alignment':result});return result


def _run_configured_transfer(robot, chain):
    for step in chain:
        op=step['op']
        if op=='free_forward':robot.forward(step['steps'])
        elif op=='wait':robot.settle_since_last_completed(step['seconds'])
        elif op=='grab_single':robot.grab(1)
        elif op=='carry_forward_group':robot.carry_forward(COMMAND0_TRANSFER_FORWARD_STEPS)
        else:robot.turn(step['repeat'],op,target_delta_deg=step['target_delta_deg'],
                        nominal_step_deg=step.get('nominal_step_deg'))
    return robot.scan_ar()


def execute(plan,backend,robot_factory=Robot):
    if 'outbound_pickup' in plan:
        from outbound_pickup_trial import OutboundPickup_Run
        return OutboundPickup_Run(plan,backend)
    if 'mission_v5' in plan:
        from mission_v5 import execute as execute_v5
        return execute_v5(plan,backend)
    cfg=plan['config'];mode=plan['mode'];routes=plan['routes']
    if mode in TURN_TEST_STEPS:
        from micro_step_test import execute_micro_step
        return execute_micro_step(plan,backend)
    def diagnostic_monitor(name,load_kind):
        if getattr(backend,'is_simulation',False):return []
        monitor=robot_factory(backend,cfg,plan['project']);monitor.navigation_bundle=plan.get('navigation');monitor.load_kind=load_kind
        if name=='stand':
            def manual_check():
                checker=getattr(backend,'check_manual_idle',None)
                if checker is None:raise RuntimeError('manual-idle observation check unavailable')
                checker()
                if monitor.clock()>monitor.deadline:raise RuntimeError('stand observation timed out')
            monitor.check=manual_check
        try:
            try:
                monitor.monitor_heading(name,allow_correction=False)
            except MicroTrialStopped as exc:
                if name!='stand':raise
                # The MCU already replied COMPLETE. Only the auxiliary magnetic read failed.
                monitor.check()
                monitor.events.append({'stand_observation':{
                    'status':'magnetic_diagnostic_unavailable','protocol_completed':True,
                    'reason_zh':'已收到站姿复位完成回执，磁航向诊断暂不可用。',
                    'diagnostic':getattr(exc,'error_details',None) or getattr(exc,'guard_reason',str(exc))}})
            return list(monitor.events)
        finally:monitor.close_navigation()
    if mode in CALIBRATION_STEPS:
        try:
            if mode in SMALL_STRAFE_TEST_STEPS and not getattr(backend,'is_simulation',False):
                verify=getattr(backend,'verify_small_strafe',None)
                if verify is None:raise NotReady('verified small strafe capability required before single-step trial')
                verify(cfg['request_timeout_s'])
            backend.call('begin_align',cfg['request_timeout_s']) # MCU检查单块负载；不松手、不先stand。
            backend.call(CALIBRATION_STEPS[mode],cfg['request_timeout_s'])
            monitored=diagnostic_monitor(CALIBRATION_STEPS[mode],'single')
            backend.check();backend.call('finish',cfg['request_timeout_s'])
            return {'status':'one_calibration_step_request_completed','post_action_monitor':monitored,'command':mode,'action':CALIBRATION_STEPS[mode],'body_action_requests':1,'expected_load':'single','physical_pose_confirmed':False,'physical_load_sensor_confirmation':False,'next_stage_started':False}
        except BaseException:
            backend.cancel();raise
    if mode=='stand':
        # 独立stand允许松手并请求正常空载站姿；不下蹲放物、不begin/扫描。
        # COMPLETE只证明协议路径返回，不证明关节到位或物体已释放。
        try:
            backend.call('stand',cfg['request_timeout_s'])
            monitored=diagnostic_monitor('stand','empty')
            return {'status':'stand_request_completed','post_action_monitor':monitored,'command':'stand','requested_posture':'normal_stand_arms_down',
                'expected_load':'empty_on_MCU','physical_load_sensor_confirmation':False,
                'physical_posture_confirmed':False,'physical_release_confirmed':False,'may_release_object':True,
                'fall_recovery':False,'next_stage_started':False}
        except BaseException:
            backend.cancel();raise
    robot=robot_factory(backend,cfg,plan['project'])
    robot.entry_alignment_bundle=plan.get('entry_alignment')
    robot.navigation_bundle=plan.get('navigation')
    robot.transfer_mode=mode in ('0','stage1')
    robot.boundary_recovery_command0=(mode=='0')
    robot.command0_scan_after_turn=(mode=='0' and cfg.get('transfer_test',{}).get('stop_after','terminal_scan')=='terminal_scan')
    if mode=='0':
        report=getattr(robot,'report_command0_phase',None)
        if report is not None:report('cmd0_prepare')
    try:
        h=plan.get('navigation',{}).get('post_action_heading',{})
        from micro_trial import active
        micro_active=active(plan.get('navigation',{}))
        transfer_nine=(robot.command0_scan_after_turn
                       and cfg.get('route_steps',{}).get('transfer_forward_steps')==9)
        if (micro_active or transfer_nine) and not getattr(backend,'is_simulation',False):
            import micro_trial
            if micro_active and not micro_trial.enabled(h):raise RuntimeError('real correction requires dedicated micro trial contract; route macros are not small turns')
            verifier=getattr(backend,'verify_mission_v5' if transfer_nine else 'verify_micro',None)
            if verifier is None:
                if transfer_nine:raise NotReady('启动前无法核验中转区连续 9 步的 MCU 能力')
                raise RuntimeError('fresh micro capability backend unavailable BEFORE motion')
            started=robot.clock()
            try:verifier(cfg['request_timeout_s'])
            finally:robot.events.append({'stage_timing':{'stage':'micro_capability','elapsed_s':robot.clock()-started}})
        if mode in ('0','stage1'):
            if cfg.get('transfer_test',{}).get('preflight_sensor_streams'):
                robot.sensor_preflight()
            # 只有显式任务开始才归位；失败/取消/未知完成不begin、不走、不抓。
            robot.completed_call('stand',cfg['request_timeout_s'])
            if not plan.get('navigation',{}).get('skip_initial_stand_monitor',False):
                robot.events.extend(diagnostic_monitor('stand','empty'))
            robot.events.append({'initial_stand':'protocol_completed','may_release_object':True,'physical_posture_confirmed':False})
        backend.call('begin',cfg['request_timeout_s'])
        if mode in ('0','stage1'):
            try:
                chain=cfg.get('route_steps',{}).get('command0_transfer') if robot.command0_scan_after_turn else None
                if chain is None:scan=(routes.cmd0 if mode=='0' else routes.cmd0_transfer_zone)(robot)
                else:scan=_run_configured_transfer(robot,chain)
            except BoundaryReached as boundary:
                scan=robot.raw_scan_ar() # read-only terminal print, never calibrate/move/resume
                robot.check();backend.call('finish',cfg['request_timeout_s'])
                return {'status':'transfer_boundary_complete','boundary':boundary.boundary,
                    'command':mode,'scan_result':scan,'events':list(robot.events),'next_stage_started':False,
                    'automatic_resume':False,'expected_load':robot.load_kind,'extra_release_commands':0,
                    'physical_pose_confirmed':False,'physical_load_sensor_confirmation':False}
            alignment=scan.get('navigation_checkpoint',{'status':'simulation_only','actions':[]}) if plan.get('navigation',{}).get('enabled') else robot.align_entry(scan)
            if alignment['status']=='stopped':
                backend.cancel()
                return {'status':'stage1_alignment_stopped','command':mode,'scan_result':scan,'entry_alignment':alignment,'events':list(robot.events),'next_stage_started':False,'expected_load':robot.load_kind,'physical_load_sensor_confirmation':False}
            from entrance_trial import EntranceTrial_IsEntryAccepted, EntranceTrial_Run
            if (robot.command0_scan_after_turn and not getattr(backend,'is_simulation',False)
                    and EntranceTrial_IsEntryAccepted(alignment)):
                segment=EntranceTrial_Run(robot,camera_info=alignment['camera_info'],entry_alignment=alignment)
                if 'terminal_turn' in scan:
                    scan['terminal_turn']['visual_trial_started']=segment['next_stage_started']
                result={'status':('stage2_checkpoint_reached' if segment['detected'] else 'stage2_trial_unconfirmed'),
                    'command':mode,'stage':2 if segment['next_stage_started'] else 1,
                    'checkpoint':'after_entrance_trial' if segment['next_stage_started'] else 'post_turn_stationary_scan',
                    'entry_alignment':alignment,'scan_result':scan,'next_segment':segment,
                    'reason':segment['reason'],'next_stage_started':segment['next_stage_started'],
                    'expected_load':robot.load_kind,'physical_load_sensor_confirmation':False}
                if segment['detected'] or segment.get('continue_estimated') is True:
                    from corridor_trial import CorridorTrialResult, CorridorTrial_Run
                    corridor=CorridorTrial_Run(robot,segment,entry_alignment=alignment)
                    delivered=corridor['release_protocol_completed']
                    status=('task_complete' if delivered else
                        'stage3_checkpoint_reached'
                        if corridor['detected'] and corridor.get('status')!=CorridorTrialResult.DEPARTURE_UNSAFE.value
                        else 'stage3_trial_unconfirmed')
                    result.update(status=status,stage=4 if delivered else 3,checkpoint=corridor['checkpoint'],
                        reason=corridor['reason'],corridor_trial=corridor,expected_load=robot.load_kind,
                        release_protocol_completed=delivered,physical_release_confirmed=False)
                robot.check();backend.call('finish',cfg['request_timeout_s'])
                result['events']=list(robot.events)
                return result
            robot.check();backend.call('finish',cfg['request_timeout_s'])
            stage_status='stage1_complete' if scan.get('status')=='recognized' else ('stage1_complete_blind' if plan.get('navigation',{}).get('enabled') else 'stage1_ended_'+str(scan.get('status')))
            if 'terminal_turn' in scan:
                stage_status='stage1_partial_alignment'
            elif alignment.get('visual_micro'):
                stage_status='stage1_alignment_blocked' if alignment['status']=='blocked' else 'stage1_partial_alignment'
            return {'entry_alignment':alignment,'status':stage_status,
                'command':mode,'stage':1,'checkpoint':('post_turn_stationary_scan' if robot.command0_scan_after_turn else 'before_obstacles_unconfirmed_reference'),'scan_result':scan,'events':list(robot.events),'next_stage_started':False,'expected_load':robot.load_kind,'physical_load_sensor_confirmation':False}
        getattr(routes,'cmd'+mode)(robot)
        robot.check();backend.call('finish',cfg['request_timeout_s'])
        return {'status':'task_complete','command':mode,'events':list(robot.events)}
    except BaseException as exc:
        exc.route_events=list(robot.events)
        backend.cancel();raise
    finally:
        close=getattr(robot,'close_navigation',None)
        if close is not None:close()
