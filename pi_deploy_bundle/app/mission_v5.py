"""V5 single-runner mission for pickup, delivery and optional unloaded return.

Importing this module never imports ROS, starts threads or sends body commands.
"""
import collections
from contextlib import contextmanager
import copy
import json
import math
from pathlib import Path
import types

from task_runner import Robot, NotReady
from pickup_controller import PickupController, validate as validate_pickup
from block_camera import BlockCameraContractLost, BlockEvidenceExpired, BlockFrameUnavailable
from heading_guard import HeadingGuard, MagneticDataUnavailable
from mission_checkpoints_v5 import (
    CheckpointArrivalResult, Checkpoint_ChooseNudge, Checkpoint_Locate, Checkpoint_ReadHeading,
    Checkpoint_TrySweepTurn, Checkpoint_ValidateArrivals, align_if_configured,
)
from nav_observation import MagneticReference, wrap

VERSION='r3-visual-pickup-20261109-v5'
OPS={
    'free_forward':('empty','empty'), 'empty_forward2':('empty','empty'),
    'empty_left_turn':('empty','empty'), 'empty_right_turn':('empty','empty'),
    'empty_strafe_left':('empty','empty'), 'empty_strafe_right':('empty','empty'),
    'grab_single':('empty','single'), 'place_single':('single','empty'),
    'grab_double':('empty','double'), 'place_double':('double','empty'),
    'carry_forward':('single','single'),'carry_forward2':('single','single'),
    'carry_forward4':('single','single'),'carry_forward7':('single','single'),
    'carry_forward9':('single','single'),'left_turn':('single','single'),
    'right_turn':('single','single'),'strafe_left_once':('single','single'),
    'strafe_right_once':('single','single'),
}
SINGLE_CHAINS=('transfer_to_entry','outbound_obstacles','base_approach','start_to_transfer')
EMPTY_CHAINS=('pickup_search','pickup_left','pickup_right','pickup_advance','base_to_return','return_obstacles','return_to_transfer')
_LOAD_TRANSITIONS=('grab_single','grab_double','place_single','place_double')
PAIR_MOTION_OPS={op:'double_'+op for op,loads in OPS.items() if loads==('single','single')}
_TURN_OPS=('left_turn','right_turn','empty_left_turn','empty_right_turn')
_CARRY_FORWARD_OPS=('carry_forward','carry_forward2','carry_forward4','carry_forward7','carry_forward9')
_TRANSLATION_OPS=_CARRY_FORWARD_OPS+(
    'strafe_left_once','strafe_right_once','free_forward','empty_forward2','empty_strafe_left','empty_strafe_right',
)
_GRAB_OPS=('grab_single','grab_double')
VISUAL_PICKUP_OPS=frozenset((*_GRAB_OPS,'free_forward','empty_forward2','empty_strafe_left','empty_strafe_right'))
_SENSOR_RECHECKS = 4


class _ChestUnavailable(RuntimeError):
    """A missing stream before the next native dispatch."""

class _CheckpointEvidenceExpired(RuntimeError):
    """The tag pose expired while preparing a short recovery step."""


def defaults():
    from block_vision import default_config
    return {'schema':'r3.mission-v5.v1','enabled':False,'field_calibration_verified':False,
            'vision':default_config(),'checkpoint_profiles':{},'checkpoint_forward_windows_m':{},'pickup':{'max_adjustments':8,'confirmation_windows':2,
            'max_lost_windows':2,'target_x':.5,'x_tolerance':.07,'target_y_min':.65,
            'target_y_max':.78,'max_target_jump':.3,'double_pickup_enabled':True},
            'note':'Illustrative grab band, NOT field calibration. Do not enable without measured empty-load camera/grab calibration.'}


def validate_chains(chains,mode):
    names=set(SINGLE_CHAINS+EMPTY_CHAINS)
    if not isinstance(chains,dict) or set(chains)!=names:raise ValueError('V5 action chain names/schema')
    required=set(('pickup_left','pickup_right','pickup_advance','transfer_to_entry','outbound_obstacles','base_approach'))
    if mode=='2':required.update(('base_to_return','return_obstacles','return_to_transfer'))
    if mode=='0-full':required.add('start_to_transfer')
    for name,steps in chains.items():
        if not isinstance(steps,(tuple,list)) or len(steps)>60:raise ValueError('bounded chain length '+name)
        if name in required and not steps:raise NotReady('V5 missing action chain: '+name)
        load='single' if name in SINGLE_CHAINS else 'empty'
        body_count=0
        for step in steps:
            if not isinstance(step,dict):raise ValueError('chain step schema')
            if 'op' in step:
                if set(step)-{'op','repeat','pause_s','target_delta_deg','allow_heading_correction'}:
                    raise ValueError('body step extra fields')
                op=step['op']; repeat=step.get('repeat',1); pause=step.get('pause_s',0)
                if op not in OPS or op in _LOAD_TRANSITIONS or OPS[op]!=(load,load):raise ValueError('chain op/load mismatch '+name)
                if name=='pickup_search' and op not in ('empty_left_turn','empty_right_turn'):
                    raise ValueError('search must remain finite in-place turns, not blind translation')
                if type(pause) not in (int,float) or not math.isfinite(pause) or not 0<=pause<=3:raise ValueError('chain pause')
                if 'allow_heading_correction' in step and (
                        op not in _CARRY_FORWARD_OPS or type(step['allow_heading_correction']) is not bool):
                    raise ValueError('boundary correction is configured only for single-load forward motions')
                if 'target_delta_deg' in step:
                    delta=step['target_delta_deg']
                    if (op not in _TURN_OPS or type(delta) not in (int,float) or not math.isfinite(delta)
                            or not 0<abs(delta)<=180 or delta*(1 if 'right' in op else -1)<=0):
                        raise ValueError('bounded planned turn target')
                if repeat is None:
                    if name in required:raise NotReady('V5 route macro count is unmeasured: '+name+'/'+op)
                    if name=='pickup_search':raise ValueError('optional search counts must be set')
                    continue
                if type(repeat) is not int or not 1<=repeat<=20:raise ValueError('chain repeat')
                body_count+=repeat
            elif set(step)=={'tag','direction'}:
                if name not in ('outbound_obstacles','return_obstacles'):raise ValueError('tag only in obstacle chains')
                if type(step['tag']) is not int or step['direction']!=('outbound' if load=='single' else 'return'):raise ValueError('tag direction/load')
            else:raise ValueError('unknown chain step')
        if body_count>80:raise ValueError('expanded chain macro budget')
        if name=='pickup_search' and steps:
            left=sum(s.get('repeat',1) for s in steps if 'left' in s.get('op',''))
            right=sum(s.get('repeat',1) for s in steps if 'right' in s.get('op',''))
            if left!=right or body_count>6:raise ValueError('bounded nominally balanced search required')
        if name in required and body_count==0:raise ValueError('empty motion chain '+name)
        if name in ('pickup_left','pickup_right','pickup_advance') and body_count!=1:
            raise ValueError('pickup feedback requires one finite motion per observation')
    for name,ids in (('outbound_obstacles',[1,2,3,4,5]),('return_obstacles',[5,6,7,1])):
        if name not in required:continue
        points=[index for index,step in enumerate(chains[name]) if 'tag' in step]
        if [chains[name][index]['tag'] for index in points]!=ids:
            raise ValueError(name+' requires explicit ordered checkpoint IDs '+str(ids))
        if points[0]!=0 or points[-1]!=len(chains[name])-1 or any(b-a<2 for a,b in zip(points,points[1:])):
            raise ValueError(name+' must interleave motion between checkpoints from entry to exit')
    return chains


def prepare(mode,cfg,project,simulation=False):
    from checkpoint_nav import load
    from block_vision import validate_config
    from transfer_test import validate as validate_transfer
    validate_transfer(cfg,mode)
    project=Path(project); path=project/'config/mission_v5.json'
    c=json.loads(path.read_text(encoding='utf-8')) if path.exists() else defaults()
    if c.get('schema')!='r3.mission-v5.v1':raise ValueError('V5 mission schema')
    validate_pickup(c['pickup']); validate_config(c['vision'],live=False)
    module=types.ModuleType('v5_chain_snapshot'); source=project/'action_chains_v5.py'
    exec(compile(source.read_text(encoding='utf-8'),str(source),'exec'),module.__dict__)
    if not simulation and (c.get('enabled') is not True or c.get('field_calibration_verified') is not True):
        raise NotReady('task '+mode+' not ready BEFORE motion: V5 pickup/camera calibration is required')
    validate_config(c['vision'],live=not simulation)
    chains=validate_chains(module.demo_chains() if simulation else module.CHAINS,mode)
    nav=load(project)
    from mission_checkpoints_v5 import validate_profiles
    validate_profiles(c.get('checkpoint_profiles',{}),nav)
    def count(name):return sum(s.get('repeat',1) for s in chains[name] if 'op' in s)
    required=['transfer_to_entry','outbound_obstacles','base_approach']
    if mode=='2':required+=['base_to_return','return_obstacles','return_to_transfer']
    if mode=='0-full':required+=['start_to_transfer']
    worst=(sum(count(n) for n in required)+2+count('pickup_search')+
           c['pickup']['max_adjustments']*max(count(n) for n in ('pickup_left','pickup_right','pickup_advance'))+
           nav.get('max_extra_actions_per_journey',0))
    if worst>cfg['max_body_macros_per_journey']:
        raise NotReady('V5 expanded route/pickup/correction budget exceeds whole-journey limit BEFORE motion')
    if not simulation:
        h=nav.get('post_action_heading',{})
        if (not nav.get('enabled') or not h.get('enabled') or not h.get('execute') or
                not h.get('micro_trial',{}).get('enabled') or not nav['magnetic'].get('capture_freshness_verified')
                or nav['magnetic'].get('heading_sign') not in (-1,1)):
            raise NotReady('V5 live fresh magnetic monitoring required BEFORE motion')
        directions=('outbound','return') if mode=='2' else ('outbound',)
        for direction in directions:
            route_ids=[step['tag'] for step in chains[direction+'_obstacles'] if 'tag' in step]
            if [goal['id'] for goal in nav['legs'][direction]]!=route_ids:
                raise NotReady('V5 navigation checkpoint order differs from the '+direction+' route BEFORE motion')
        Checkpoint_ValidateArrivals(c.get('checkpoint_forward_windows_m',{}),nav,directions)
        if c['vision']['topic']!=nav['camera_health']['topic']:
            raise NotReady('V5 block vision and checkpoint health must use the same chest camera topic BEFORE motion')
    return {'mode':mode,'config':cfg,'project':str(project),'routes':None,'navigation':nav,
            'mission_v5':copy.deepcopy(c),'chains_v5':copy.deepcopy(chains),'simulation_only':simulation,
            'v5_worst_case_body_macros':worst,
            'route_evidence':'field_verified' if module.FIELD_VALIDATED else 'estimated_trial'}


class MissionRobot(Robot):
    def configure(self,plan):
        self.mission=plan['mission_v5']; self.chains=plan['chains_v5']; self.camera=None
        self.navigation_bundle=plan['navigation']; self.transfer_mode=False
        self.pickup=PickupController(self,self.mission['pickup']); self.deliveries=0
        self.last_block=None; self.observation_floor=self.clock()
        self.observation_ros_floor=0
        self.events=collections.deque(self.events,maxlen=512)
        self.last_checkpoint=None
        self.progress=None
        self.blocks_delivered=0
        self.route_evidence=plan['route_evidence']
        load_chains=[name for name in SINGLE_CHAINS if name!='start_to_transfer' or plan['mode']=='0-full']
        self.pair_required={'grab_double','place_double'}
        self.pair_required.update(PAIR_MOTION_OPS[step['op']] for name in load_chains for step in self.chains[name] if 'op' in step)

    def scope(self):return (getattr(self.backend,'session',''),getattr(self.backend,'epoch',-1))

    def supports_pair(self):
        operations=getattr(self.backend,'mission_pair_ops',None)
        return operations is not None and self.pair_required.issubset(operations())

    def _record_progress(self,phase,**detail):
        progress={'phase':phase,'expected_load':self.load_kind,'route_evidence':self.route_evidence,**detail}
        if self.progress==progress:
            return
        self.progress=progress
        self.events.append({'mission_progress':progress})
        report=getattr(self.backend,'report_progress',None)
        if report is not None:
            report(progress)

    @contextmanager
    def _pause_route_timer(self):
        started=self.clock()
        deadline=self.deadline
        self.deadline=float('inf')
        try:
            yield
        finally:
            self.deadline=deadline+self.clock()-started

    def sensor_preflight(self, *, magnetic_optional=False):
        """Read magnetic and chest streams in order on the route's task thread."""
        nav = self.ensure_navigation_io()
        cfg = nav.cfg

        def check():
            self.backend.check_manual_idle()
            if self.clock() >= self.deadline:
                raise RuntimeError('V5 preflight timed out before begin')

        guard=None
        if magnetic_optional:
            if not self._allows_missing_magnetic():
                raise ValueError('optional magnetic preflight requires the configured outbound route')
            guard=HeadingGuard(cfg,nav.io,MagneticReference(cfg['magnetic']),check,self.clock)
            guard.allow_missing_magnetic=True
            guard.observation_scope=self.scope()

        def heading_ready():
            if guard is None:
                return bool(self._read_heading(nav,check))
            if self.magnetic_route_only:
                return True
            try:
                _,sample=guard.sample(self.load_kind,stationary_retry=True)
            except MagneticDataUnavailable as exc:
                if not self.defer_magnetic_route_fault(exc):raise
                return True
            return bool(sample and sample.get('capture_verified') and nav.io.motion_capture_verified)

        for attempt in range(_SENSOR_RECHECKS):
            check()
            after = nav.io.ros_now_ns()
            window = nav.io.observe(cfg['legs']['outbound'][0]['id'], after, cfg['observe_s'], check)
            try:
                self._check_camera(window, 'before begin')
            except _ChestUnavailable as exc:
                reason = '启动前多次重读仍未收到有效的胸部相机、CameraInfo 或扫码数据。'
                self._record_progress('waiting_camera',reason=str(exc))
            else:
                if heading_ready() and nav.io.preflight_health_snapshot().get('camera_healthy'):
                    check()
                    break
                reason = '启动前多次重读仍未取得同时有效的航向与胸部相机数据。'
                self._record_progress('waiting_sensors',reason=reason)
            check()
            if attempt + 1 < _SENSOR_RECHECKS:
                self.sleep(self.empty_retry_s)
        else:
            raise NotReady(reason)
        self.events.append({
            'sensor_preflight': ('fresh chest stream; following visual route after magnetic loss'
                                 if self.magnetic_route_only else 'fresh magnetic and chest streams'),
            'parallel_read_only': False,
            'body_commands': 0,
        })

    def completed_call(self, name, timeout_s):
        if not getattr(self.backend, 'is_simulation', False):
            self._check_camera(self.ensure_navigation_io().io.preflight_health_snapshot(), 'before '+name)
            self.check()
        return super().completed_call(name, timeout_s)

    def _await_camera(self, name):
        io=self.ensure_navigation_io().io
        for attempt in range(_SENSOR_RECHECKS):
            self.check()
            try:
                self._check_camera(io.preflight_health_snapshot(),'before '+name)
            except _ChestUnavailable as exc:
                self._record_progress('waiting_camera',action=name,reason=str(exc))
            else:
                return
            if attempt + 1 < _SENSOR_RECHECKS:
                self.wait(self.empty_retry_s)
        raise _ChestUnavailable('胸部相机多次复查后仍未恢复，待执行的动作尚未发出。')

    @staticmethod
    def _check_camera(window, boundary):
        if not window.get('camera_healthy'):
            raise _ChestUnavailable('V5 chest image/AR stream unavailable '+boundary)
        info = window.get('camera_info', {})
        if not info.get('ready'):
            raise _ChestUnavailable('V5 CameraInfo unavailable {} ({})'.format(boundary, info.get('reason', 'missing')))

    def _open_block_camera(self,count_only):
        from block_camera import BlockCamera
        return BlockCamera(self.mission['vision'],scope=self.scope())

    def observe_blocks(self,*,count_only=False):
        if getattr(self.backend,'is_simulation',False):return self.backend.observe_blocks(self)
        if self.camera is None:
            self.camera=self._open_block_camera(count_only)
        self.camera.set_scope(self.scope(),self.load_kind)
        mono=max(self.last_action_completed_monotonic or 0,self.observation_floor)
        ros=max(self.last_action_completed_ros_ns or 0,self.observation_ros_floor)
        options={'count_only':True} if count_only else {}
        try:
            row=self.camera.observe(mono,ros,self.check,expected_load=self.load_kind,scope=self.scope(),**options)
        except BlockCameraContractLost:
            self.observation_floor=max(mono,self.clock())
            self.observation_ros_floor=max(ros,self.camera.ros_now_ns())
            self.last_block=None
            self.camera.close()
            self.camera=None
            raise
        self.observation_floor=max(mono,row.get('received_monotonic',mono))
        self.observation_ros_floor=max(ros,row.get('stamp_ns',ros))
        self.last_block=row
        return row

    def verify_block_observation(self,row):
        self.check()
        if self.load_kind!='empty':raise RuntimeError('grab load changed')
        if getattr(self.backend,'is_simulation',False):return
        self.camera.verify(row,scope=self.scope(),load=self.load_kind)

    def finite(self,op,observation=None,*,allow_heading_correction=False,checkpoint_follows=False,
               before_dispatch=None,configured=False,visual_pickup=False):
        self.check()
        required_load, next_load=OPS[op]
        if configured and (required_load,next_load)!=('empty','empty'):
            raise ValueError('configured return may only dispatch empty travel macros')
        if visual_pickup and (not self._allows_missing_magnetic() or op not in VISUAL_PICKUP_OPS or observation is None):
            raise ValueError('visual pickup requires a supported macro and a fresh target on the configured outbound route')
        magnetic_optional=configured or visual_pickup
        motion_context={'configured': True} if magnetic_optional else {}
        native_op=op
        if self.load_kind=='double' and op in PAIR_MOTION_OPS:
            required_load=next_load='double'
            native_op=PAIR_MOTION_OPS[op]
        if self.load_kind!=required_load:
            raise RuntimeError('V5 finite op/load mismatch '+op)
        if required_load=='double' or next_load=='double':
            if not self.supports_pair() or native_op not in self.backend.mission_pair_ops():
                raise RuntimeError('V5 double-load native capability unavailable before '+native_op)
        if self.count>=self.cfg['max_body_macros_per_journey'] or self.deadline-self.clock()<=self.cfg['request_timeout_s']:
            raise RuntimeError('V5 journey action/time budget exhausted before dispatch')
        if not getattr(self.backend,'is_simulation',False):
            self.backend.check_mission_v5()
            self.check_magnetic_before_motion(**motion_context)
            self.settle_since_last_completed(self.navigation_bundle['settle_s'])
            self.prepare_heading_segment()
            if self.deadline-self.clock()<=self.cfg['request_timeout_s']:
                raise RuntimeError('V5 journey time budget exhausted after settling')
        guard=getattr(self,'heading_guard',None)
        capture_turn=guard is not None and op in _TURN_OPS and not (magnetic_optional and self.magnetic_route_only)
        if guard is not None:
            guard.observation_scope=self.scope()
            if capture_turn:
                guard.begin_turn_observation(op,self.load_kind,self.clock(),*self.scope())
        if op in _GRAB_OPS:
            if observation is None:raise RuntimeError('grab requires bound fresh visual evidence')
            if observation.get('grip_count',observation.get('count')) != (2 if op=='grab_double' else 1):
                raise RuntimeError('grab macro does not match observed ground-row target')
            self.verify_block_observation(observation)
        if not getattr(self.backend,'is_simulation',False) and not (magnetic_optional and self.magnetic_route_only) and (
                guard is None or guard.blocked or not guard.reference.anchor):
            raise RuntimeError('V5 heading reference unavailable before '+op)
        self.check()
        for attempt in range(_SENSOR_RECHECKS):
            if before_dispatch is not None:
                before_dispatch()
            if visual_pickup:
                self.verify_block_observation(observation)
                if ((self.last_action_completed_monotonic is not None
                     and observation.get('received_monotonic',-1)<=self.last_action_completed_monotonic)
                        or (self.last_action_completed_ros_ns is not None
                            and observation.get('stamp_ns',-1)<=self.last_action_completed_ros_ns)):
                    raise BlockEvidenceExpired('block observation predates the last completed action')
            try:
                self.completed_call(native_op,self.cfg['request_timeout_s'])
                break
            except _ChestUnavailable as exc:
                # This exception is raised before submitting the native request.
                if op in _GRAB_OPS:
                    raise BlockEvidenceExpired(str(exc)) from exc
                if attempt + 1 >= _SENSOR_RECHECKS:
                    raise _ChestUnavailable('胸部相机反复在动作提交前掉线，待执行的动作尚未发出。') from exc
                self._await_camera(native_op)
                self.backend.check_mission_v5()
                self.check_magnetic_before_motion(**motion_context)
            except Exception:
                self.record_fall_diagnostic(native_op)
                raise
        self.count+=1
        if capture_turn and not (magnetic_optional and self.magnetic_route_only):
            guard.finish_turn_observation(self.last_action_completed_monotonic)
        self.load_kind=next_load
        if self.camera is not None:self.camera.set_scope(self.scope(),self.load_kind)
        self.events.append({'v5_action':native_op,'expected_load':self.load_kind,'physical_success':'unobserved'})
        alias={'empty_left_turn':'left_turn','empty_right_turn':'right_turn','place_single':'stand',
               'place_double':'stand','grab_double':'grab_single',
               'carry_forward7':'carry_forward','carry_forward9':'carry_forward'}.get(op,op)
        try:
            self.monitor_heading(alias,allow_correction=(
                (allow_heading_correction and op in _CARRY_FORWARD_OPS)
                or getattr(self,'heading_turn_phase',None)=='planned_turn_complete'))
        except _ChestUnavailable:
            # The requested macro is complete. A dropped frame can only cancel
            # the pending correction here, never replay the completed macro.
            self.last_heading_report=None
        except Exception:
            self.record_fall_diagnostic(native_op)
            raise
        if (guard is not None and op in _TRANSLATION_OPS and not checkpoint_follows
                and not (magnetic_optional and self.magnetic_route_only)):
            report=self.last_heading_report or {}
            residual=report.get('after_error_deg',report.get('error_deg'))
            if (guard.blocked or type(residual) not in (int,float) or not math.isfinite(residual)
                    or abs(residual)>guard.c['deadband_deg']):
                self._await_heading(op,**motion_context)

    def chain_step(self,step,*,checkpoint_follows=False,configured=False):
        self.check()
        if 'tag' in step:return self.checkpoint(step['tag'],step['direction'])
        motion_context={'configured': True} if configured else {}
        if 'target_delta_deg' in step and not getattr(self.backend,'is_simulation',False):
            self._planned_turn(step,**motion_context)
            return
        repeat=step.get('repeat',1)
        for index in range(repeat):
            self.finite(step['op'],allow_heading_correction=step.get('allow_heading_correction',False),
                        checkpoint_follows=checkpoint_follows and index==repeat-1,**motion_context)
            self.wait(step.get('pause_s',0))

    def _planned_turn(self, step, *, configured=False):
        motion_context={'configured': True} if configured else {}
        if configured:
            self.check_magnetic_before_motion(configured=True)
        self.prepare_heading_segment()
        guard=self.heading_guard
        op=step['op']
        original=None
        if not (configured and self.magnetic_route_only):
            guard.plan_turn(step['target_delta_deg'])
            original=guard.c
            anchor=guard.reference.anchor
            sign=self.navigation_bundle['magnetic']['heading_sign']
            # Measured overshoot is handled by recovery using the same target.
            guard.c=dict(original,stop_on_possible_overshoot=False)
        self.heading_turn_phase='planned_turn_intermediate'
        try:
            for _ in range(step.get('repeat',1)):
                before=guard.last_sample if guard is not None else None
                self.finite(op,**motion_context)
                self.wait(step.get('pause_s',0))
                if configured and self.magnetic_route_only:
                    continue  # Complete only the remaining configured turns.
                after=guard.last_sample
                if not before or not after:
                    break
                actual=wrap(sign*(after['raw_heading_deg']-before['raw_heading_deg']))
                error=wrap(sign*after['raw_heading_deg']+anchor['offset_deg']-anchor['route_heading_deg'])
                self.events.append({'mission_planned_turn':op,'observed_deg':actual,'residual_deg':error})
                if abs(error)<=original['deadband_deg']:
                    return
                if actual*step['target_delta_deg']<=0 or error*step['target_delta_deg']>=0:
                    break
                if abs(error)<=abs(actual)/2+3.:
                    break
        finally:
            if original is not None:
                guard.c=original
            self.heading_turn_phase=None
        if not (configured and self.magnetic_route_only):
            self._await_heading(op,**motion_context)

    def run_chain(self,name):
        self.events.append({'v5_chain':name})
        self._record_progress(name)
        steps=self.chains[name]
        checkpoint_indices={step['tag']:index for index,step in enumerate(steps) if 'tag' in step}
        index=0
        while index<len(steps):
            step=steps[index]
            if 'tag' in step:
                last_motion=steps[index-1]['op'] if index else ''
                arrived=self.checkpoint(step['tag'],step['direction'],last_motion=last_motion)
                index=checkpoint_indices[arrived]+1
            else:
                follows=index+1<len(steps) and 'tag' in steps[index+1]
                self.chain_step(step,checkpoint_follows=follows)
                index+=1

    def _read_heading(self,nav,check,*,configured=False):
        if configured:
            _,sample=self.heading_guard.sample(self.load_kind,
                action_completed_monotonic=self.last_action_completed_monotonic,stationary_retry=True)
            return sample if sample and sample.get('capture_verified') and nav.io.motion_capture_verified else None
        return Checkpoint_ReadHeading(self,nav,check)

    def _await_heading(self,op,*,configured=False):
        nav=self.ensure_navigation_io()
        anchor=nav.mag.anchor
        motion_context={'configured': True} if configured else {}
        advisory_heading=configured and self._allows_missing_magnetic()
        sweep={'origin':None,'turns':0,'blocked':False}
        idle_windows=0
        with self._pause_route_timer():
            while idle_windows<_SENSOR_RECHECKS:
                self.check()
                sample=None
                if configured:
                    self.check_magnetic_before_motion(configured=True)
                    if self.magnetic_route_only:
                        return
                self._record_progress('recovering_heading',after_action=op)
                try:
                    sample=self._read_heading(nav,self.check,**motion_context)
                    if anchor and sample and sample.get('calibration_id')==anchor.get('calibration_id'):
                        yaw=nav.cfg['magnetic']['heading_sign']*sample['raw_heading_deg']
                        error=wrap(yaw+anchor['offset_deg']-anchor['route_heading_deg'])
                        if abs(error)<=nav.cfg['post_action_heading']['deadband_deg']:
                            nav.mag.anchor=dict(anchor,received_monotonic=sample['received_monotonic'])
                            return
                        moved=self._search_turn(nav,sweep,error=error,**motion_context)
                        if moved:
                            if advisory_heading and sweep['blocked']:
                                self.events.append({'return_heading_recheck': {
                                    'after_action': op, 'search_turns': sweep['turns'],
                                    'reason_zh': '空载纠偏后航向反馈待确认，复测后按原路线续走。'}})
                            idle_windows=0
                            continue
                        if advisory_heading:
                            latest=self.heading_guard.last_sample or {}
                            if self.heading_guard.corridor_motion_sample_ready(self.load_kind,
                                    latest.get('received_monotonic'),self.last_action_completed_monotonic):
                                yaw=nav.cfg['magnetic']['heading_sign']*latest['raw_heading_deg']
                                residual=wrap(yaw+anchor['offset_deg']-anchor['route_heading_deg'])
                                nav.mag.anchor=dict(anchor,stationary_validated_monotonic=latest['received_monotonic'])
                                self.events.append({'return_heading_advisory':{
                                    'after_action':op,'remaining_heading_error_deg':residual,
                                    'heading_aligned':abs(residual)<=nav.cfg['post_action_heading']['deadband_deg'],
                                    'search_turns':sweep['turns'],
                                    'sweep_limited':sweep['blocked'] or sweep.get('limit_reached',False)}})
                                return
                except _ChestUnavailable:
                    pass
                except MagneticDataUnavailable as exc:
                    if not configured or not self.defer_magnetic_route_fault(exc):
                        raise
                    return
                idle_windows+=1
                if idle_windows<_SENSOR_RECHECKS:
                    self.wait(self.empty_retry_s)
        if advisory_heading:
            capture_health=getattr(nav.io,'magnetic_capture_health',lambda:{})()
            if self._enter_magnetic_route_fallback(capture_health,'return_heading_recheck'):
                return
        raise NotReady('动作已完成，但航向连续复查无进展，未找到可用的下一段航向参考。')

    def _extra_action_available(self,nav):
        return (self.nav_count<nav.cfg['max_extra_actions_per_journey']
                and self.count<self.cfg['max_body_macros_per_journey'])

    def _search_turn(self,nav,sweep,*,error=None,configured=False):
        pair_ops=self.backend.mission_pair_ops() if self.load_kind=='double' and self.supports_pair() else ()
        motion_context={'configured': True} if configured else {}

        def check_action():
            capability=self.backend.check_mission_v5()
            if capability is False or not configured:
                return capability
            self.check_magnetic_before_motion(configured=True)
            return not self.magnetic_route_only

        return Checkpoint_TrySweepTurn(self,nav,sweep,
            read_heading=lambda:self._read_heading(nav,self.check,**motion_context),
            error=error,pair_ops=pair_ops,check_action=check_action,
            accept_verified_angle=configured and self._allows_missing_magnetic())

    def checkpoint(self,tag,direction,*,last_motion=''):
        """Poll until recovered arrival, including a bounded in-place sweep."""
        if getattr(self.backend,'is_simulation',False):
            self.events.append({'v5_checkpoint':{'id':tag,'direction':direction,'status':'simulation_only'}})
            return tag
        nav = self.ensure_navigation_io()
        cfg = nav.cfg
        goal = next(g for g in cfg['legs'][direction] if g['id'] == tag)
        windows = self.mission['checkpoint_forward_windows_m'][direction]
        forward_window = windows[str(tag)]
        boundary = 'at checkpoint {}:{}'.format(direction, tag)
        passed=getattr(self,'last_checkpoint',None)
        previous=passed[1] if passed and passed[0]==direction else None
        sweep={'origin':None,'turns':0,'blocked':False}
        alignment=None
        nudged=False
        adjacent=None
        misses=0
        with self._pause_route_timer():
            recover_until=self.clock()+cfg['checkpoint_timeout_s']
            while self.clock()<recover_until:
                self.settle_since_last_completed(cfg['settle_s'])
                after=nav.io.ros_now_ns()
                if type(after) is not int or after<=0:
                    raise RuntimeError('V5 checkpoint clock unavailable')
                window=nav.io.observe(tag,after,cfg['observe_s'],self.check)
                try:
                    self._check_camera(window,boundary)
                    row,result,location=Checkpoint_Locate(window,cfg,windows,direction,tag,previous,after)
                    next_seen=location.get('adjacent_at_arrival')
                    if result is CheckpointArrivalResult.ARRIVED or (next_seen is not None and next_seen==adjacent):
                        arrived=tag if result is CheckpointArrivalResult.ARRIVED else next_seen
                        break
                    adjacent=next_seen
                    self._record_progress('recovering_checkpoint',reason=result.value,
                                          search_turns=sweep['turns'],**location)
                    if adjacent is not None:
                        continue  # Confirm the adjacent pose in a new stationary window.
                    if result is CheckpointArrivalResult.POSE_UNAVAILABLE:
                        misses+=1
                        if misses>1 and goal['allow_turn'] and self._search_turn(nav,sweep):
                            continue
                    elif result is CheckpointArrivalResult.HEADING_OUTSIDE and goal['allow_turn']:
                        key='{}:{}:{}'.format(direction,self.load_kind,tag)
                        profile=self.mission.get('checkpoint_profiles',{}).get(key,{})
                        if alignment is None and profile.get('execute'):
                            alignment=align_if_configured(self,tag,direction,deadline=self.clock()+cfg['checkpoint_timeout_s'])
                            if alignment and alignment.get('correction_executed'):
                                continue
                        error=wrap(row['heading_deg']-goal['tag_heading_deg'])
                        if self._search_turn(nav,sweep,error=error):
                            continue
                    elif result is CheckpointArrivalResult.POSITION_OUTSIDE and not nudged and self._extra_action_available(nav):
                        op=Checkpoint_ChooseNudge(row,goal,forward_window,cfg,last_motion)
                        if op:
                            stamp=row['latest_stamp_ns']

                            def verify_pose():
                                self.check()
                                if not nav.io.visual_motion_ready(stamp):
                                    raise _CheckpointEvidenceExpired('checkpoint pose expired before '+op)

                            self.finite(op,checkpoint_follows=True,before_dispatch=verify_pose)
                            self.nav_count+=1
                            nudged=True
                            continue
                except _ChestUnavailable as exc:
                    adjacent=None
                    self._record_progress('waiting_camera',direction=direction,expected_tag=tag,reason=str(exc))
                except _CheckpointEvidenceExpired:
                    self._record_progress('recovering_checkpoint',reason='pose_expired',**location)
                self.wait(self.empty_retry_s)
            else:
                reason=f'Tag {tag} 在配置的恢复时限内未取得可定位的位姿，已搜索转向 {sweep["turns"]} 次。'
                self._record_progress('checkpoint_unconfirmed',direction=direction,expected_tag=tag,reason_zh=reason)
                raise NotReady(reason)
        self.last_checkpoint=(direction,arrived)
        nav.mag.anchor=None
        self.last_heading_report=None
        self.events.append({'v5_checkpoint':{'id':arrived,'direction':direction,
            'status':'arrived' if arrived==tag else 'reanchored_adjacent','location':location,
            'scan':row,'search_turns':sweep['turns'],'heading_alignment':alignment,'nudged':nudged}})
        self._record_progress('checkpoint_passed',accepted_tag=arrived,**location)
        return arrived

    def deliver(self):
        self.run_chain('transfer_to_entry'); self.run_chain('outbound_obstacles')
        self.run_chain('base_approach')
        count=2 if self.load_kind=='double' else 1
        self.finite('place_double' if count==2 else 'place_single')
        self.deliveries+=1
        self.blocks_delivered+=count
        self.events.append({'delivery_protocol_completed':self.deliveries,'expected_blocks_delivered':self.blocks_delivered,
                            'physical_release_confirmed':False})

    def close(self):
        if self.camera is not None:self.camera.close(); self.camera=None
        self.close_navigation()


def execute(plan,backend,robot_factory=MissionRobot):
    if plan.get('simulation_only') and not getattr(backend,'is_simulation',False):raise RuntimeError('demo plan cannot use live backend')
    r=robot_factory(backend,plan['config'],plan['project']); r.configure(plan); mode=plan['mode']
    try:
        if not getattr(backend,'is_simulation',False):
            backend.verify_mission_v5(r.cfg['request_timeout_s'])
            r.sensor_preflight()  # read-only before first body action
        backend.call('begin',r.cfg['request_timeout_s'])
        while True:
            r.check()
            # new_journey resets V4's completion fields. Preserve completion and
            # introduce a new observation floor before clearing any old episode.
            r.observation_floor=max(r.observation_floor,r.last_action_completed_monotonic or 0,r.clock())
            r.observation_ros_floor=max(r.observation_ros_floor,r.last_action_completed_ros_ns or 0)
            r.new_journey()
            # Establish slow magnetic reference BEFORE observing the final pickup
            # windows. finite() still rechecks after any later preflight delay.
            r.prepare_heading_segment()
            phase='start_wall_pickup' if mode=='2' and r.deliveries else 'pickup'
            r._record_progress(phase)
            with r._pause_route_timer():
                acquire_until=r.clock()+r.navigation_bundle['checkpoint_timeout_s']
                while r.clock()<acquire_until:
                    try:
                        acquired=r.pickup.acquire()
                    except BlockFrameUnavailable as exc:
                        acquired={'status':'no_pickup','reason':str(exc)}
                    r.events.append({'pickup':acquired})
                    if acquired['status']!='no_pickup':
                        break
                    r._record_progress(phase,reason=acquired['reason'],adjustments=r.pickup.adjustments)
                    r.wait(r.empty_retry_s)
                else:
                    reason='已完成取物搜索和复查，仍无可抓取的目标。'
                    r._record_progress('pickup_unconfirmed',reason_zh=reason,adjustments=r.pickup.adjustments)
                    r.check()
                    backend.call('finish',r.cfg['request_timeout_s'])
                    return {'status':'pickup_unconfirmed','command':mode,'reason':reason,'events':list(r.events),
                            'expected_load':r.load_kind,'deliveries':r.deliveries,
                            'expected_blocks_delivered':r.blocks_delivered}
            if mode=='0-full':r.run_chain('start_to_transfer')
            r.deliver()
            if mode!='2':
                r.check(); backend.call('finish',r.cfg['request_timeout_s'])
                return {'status':'task_complete','command':mode,'deliveries':r.deliveries,'expected_blocks_delivered':r.blocks_delivered,'events':list(r.events),
                        'expected_load':r.load_kind,'physical_grip_confirmed':False,'physical_release_confirmed':False}
            r.run_chain('base_to_return'); r.run_chain('return_obstacles'); r.run_chain('return_to_transfer')
            r.pickup.reset_after_delivery()
            # No production trip cap. Simulator has its own explicit cancellation event.
            boundary=getattr(backend,'cycle_boundary',None)
            if boundary is not None:boundary(r)
    except BaseException as exc:
        exc.route_events=list(r.events); backend.cancel(); raise
    finally:r.close()
