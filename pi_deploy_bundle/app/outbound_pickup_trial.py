"""Visual pickup, shared outbound delivery, and the configured command 2 endpoint."""

import json
from collections import deque
from itertools import groupby, takewhile
from pathlib import Path
from types import SimpleNamespace

from action_chains_v5 import CHAINS, demo_chains
from block_camera import BlockCamera, BlockEvidenceExpired, BlockFrameUnavailable
from block_vision import validate_config
from checkpoint_nav import load as load_navigation
from cmd0_motion import (Cmd0Motion_ContinuousReady, Cmd0Motion_LoadRouteSteps, FORWARD_ACTIONS,
                         PICKUP_BACKWARD4_ACTION, PICKUP_BACKWARD6_ACTION, PICKUP_BACKWARD8_ACTION, PICKUP_EXIT_ACTIONS,
                         PICKUP_MATCH_OUTBOUND_STRAFES, PLACE_ACTION,
                          PickupRetreatUnavailable, SIDE_ACTIONS, SMALL_SIDE_ACTIONS)
from corridor_trial import (CorridorTrial_CheckLateralHeading, CorridorTrial_DefaultChain,
                            CorridorTrial_GroupStrafes, CorridorTrial_HeadingSnapshot,
                            CorridorTrial_MaxBodyMacros, CorridorTrial_MirroredPickupStrafes, CorridorTrial_Run)
from checkpoint_alignment import Checkpoint_TrackEntryHeading
from checkpoint_scan import Checkpoint_FromBoundary, Checkpoint_Observe, CheckpointScanResult
from entrance_trial import EntranceTrial_IsEntryAccepted, EntranceTrial_PlanForward, EntranceTrial_Run
from mission_v5 import MissionRobot, PAIR_MOTION_OPS, VISUAL_PICKUP_OPS, defaults, validate_chains
from micro_trial import MEDIUM_OPS, OPS as MICRO_OPS
from pickup_controller import PickupObservationUnavailable, validate as validate_pickup
from route_pose import RoutePose_TrialLateralHint
from task_runner import NotReady, Robot
from transfer_test import validate as validate_transfer
from checkpoint_errors import VisualCamera_IsFresh, VisualCamera_Read, VisualCameraUnavailable


_PICKUP_SIDE_ACTIONS = frozenset((*SIDE_ACTIONS.values(), *SMALL_SIDE_ACTIONS, *SMALL_SIDE_ACTIONS.values()))
_PICKUP_RECHECKS = 4
_TAG1_PICKUP_RECHECKS = 8
PAIR_ROUTE_OPS = {**PAIR_MOTION_OPS, **{name: 'double_'+name for name in (
    'cmd0_carry_backward2', *FORWARD_ACTIONS.values(), *SIDE_ACTIONS.values(),
    *SMALL_SIDE_ACTIONS.values(), *MICRO_OPS, *MEDIUM_OPS)}, PLACE_ACTION: 'place_double',
    PICKUP_BACKWARD4_ACTION: 'double_' + PICKUP_BACKWARD4_ACTION,
    PICKUP_BACKWARD6_ACTION: 'double_' + PICKUP_BACKWARD6_ACTION,
    PICKUP_BACKWARD8_ACTION: 'double_' + PICKUP_BACKWARD8_ACTION}
PAIR_REQUIRED_OPS = frozenset(physical for logical, physical in PAIR_ROUTE_OPS.items()
                              if logical not in (PICKUP_BACKWARD4_ACTION, PICKUP_BACKWARD6_ACTION,
                                                 PICKUP_BACKWARD8_ACTION)) | {'grab_double'}
_PICKUP_ADJUSTMENTS = {'pickup_left': 'empty_strafe_left', 'pickup_right': 'empty_strafe_right',
                      'pickup_advance': 'empty_forward2'}


def _pickup_side_groups(actions):
    groups = []
    for direction, motions in groupby(actions):
        if direction not in SMALL_SIDE_ACTIONS:
            groups.extend(motions)
            continue
        groups.extend(CorridorTrial_GroupStrafes(direction, sum(1 for _ in motions),
                                                 continuous=True, monitor_every=3))
    return tuple(groups)


def _pickup_exit_actions(steps, corridor, *, small_ready, command=None):
    actions = steps.get('pickup_exit', PICKUP_EXIT_ACTIONS)
    if command in ('1', '2'):
        actions = steps.get('command1_pickup_exit', actions)
    return _pickup_side_groups(move for action in actions for move in (
        CorridorTrial_MirroredPickupStrafes(corridor, small_ready=small_ready)
        if action == PICKUP_MATCH_OUTBOUND_STRAFES else (action,)))


def _pickup_exit_macro_budget(actions):
    # A recent drift can split a three- or four-step left group in two.
    return len(actions) + actions.count('cmd0_strafe_left3') + actions.count('cmd0_strafe_left4')


def _pickup_retreat_groups(backend, remaining, *, allow_native_six=True):
    groups = []
    for native_steps, operation in ((8, PICKUP_BACKWARD8_ACTION),
                                    (6, PICKUP_BACKWARD6_ACTION), (4, PICKUP_BACKWARD4_ACTION)):
        if remaining < native_steps or (native_steps == 6 and not allow_native_six):
            continue
        # Older boards export an eight-step macro with extra backward pitch.
        # Only the gait revision verified by the owner can run all eight together.
        capability = 'pickup_level_backward8_ready' if native_steps == 8 else f'pickup_backward{native_steps}_ready'
        ready = getattr(backend, capability, lambda: False)
        live_six = (native_steps == 6
                    and getattr(backend, 'Pickup_CanTryBackward6', lambda: False)() is True)
        if ready() is True or live_six:
            repeats, remaining = divmod(remaining, native_steps)
            groups.extend([(operation, native_steps)] * repeats)
    groups.extend([('cmd0_carry_backward2', 2)] * (remaining // 2))
    return groups


def _pickup_turns_since(robot, marker):
    actions = []
    for event in reversed(robot.events):
        if event is marker:
            break
        action = event.get('magnetic_action') or event.get('visual_micro_action')
        if action:
            actions.append(action)
    return list(reversed(actions))


def _pickup_settled_heading(robot, heading):
    """A completed turn invalidates its pre-turn sample, even when feedback drops out."""
    last = (heading.get('correction_steps') or [heading])[-1]
    if last.get('correction_actions') and ('after_error_deg' not in last or not last.get('verification')):
        return None
    settled = CorridorTrial_HeadingSnapshot(heading,after=True,load=robot.load_kind)
    if settled and last.get('correction_actions'):
        before = CorridorTrial_HeadingSnapshot(last,after=False,load=robot.load_kind)
        if not before or settled[2]!=before[2] or settled[1]<=before[1]:
            return None
    completed = getattr(robot,'last_action_completed_monotonic',None)
    if settled and type(completed) in (int,float) and settled[1] <= completed:
        return None
    return settled


def _pickup_recover_magnetic_residual(robot, boundary, boundary_name='pickup_retreat_complete'):
    marker = robot.events[-1] if robot.events else None
    robot.corridor_recover_heading(boundary=boundary_name)
    report = robot.last_heading_report or {}
    settled = _pickup_settled_heading(robot,report)
    boundary['correction_actions'] += report.get('correction_actions',0)
    boundary['heading_status'] = report.get('status','heading_unavailable')
    if settled:
        boundary['residual_deg'] = settled[0]
    elif report.get('correction_actions') or _pickup_turns_since(robot,marker):
        boundary['residual_deg'] = None


def _pickup_use_retreat_tag(robot, nav, boundary, view, reserved):
    yaw, stamp = view['heading_deg'], view['latest_stamp_ns']
    deadband = nav.cfg['post_action_heading']['deadband_deg']
    if stamp <= robot.last_action_completed_ros_ns or not VisualCamera_IsFresh(nav.io.visual_motion_ready,stamp):
        # The guard yielded without moving for this observation. A camera
        # restart or expired frame must give that decision back to the magnet.
        before = boundary['residual_deg']
        if before is not None and abs(before)>deadband:
            _pickup_recover_magnetic_residual(robot,boundary)
        return
    applied = robot.Route_RebaseTagHeading(1,yaw,stamp).value
    boundary.update(heading_source='tag1', heading_before_correction_deg=yaw, residual_deg=yaw,
                    heading_status='tag1_verified', tag1_reference=applied)
    if abs(yaw)<=deadband:
        return
    if applied=='applied':
        _pickup_recover_magnetic_residual(robot,boundary)
        return
    turn = robot.Route_TurnFromTagHeading(1,yaw,stamp,reserved_actions=reserved)
    if not turn:
        return
    boundary['correction_actions'] += 1
    boundary['residual_deg'] = None
    fresh = {'expected_id':1,'detected':False,'observation_windows':0}
    status,_ = Checkpoint_Observe(robot,nav,fresh,lambda:None,single_window=True,quick_id=True)
    if status is CheckpointScanResult.CHECKPOINT_DETECTED:
        after = RoutePose_TrialLateralHint(fresh,nav.cfg,1)
        if (after and after['latest_stamp_ns'] > max(stamp,robot.last_action_completed_ros_ns)
                and VisualCamera_IsFresh(nav.io.visual_motion_ready,after['latest_stamp_ns'])
                and max(map(abs,after['lateral_bounds_m'])) <= .10):
            boundary['residual_deg'] = after['heading_deg']
            boundary['tag1_reference'] = robot.Route_RebaseTagHeading(1,after['heading_deg'],after['latest_stamp_ns']).value


def _run_startup_retreat(robot, nav, pending, report, *, pickup_heading=None):
    """Complete the shared first-pickup retreat, then realign before the first strafe."""
    retreat_count = sum(1 for _ in takewhile(lambda op: op == 'cmd0_carry_backward2', pending))
    if not retreat_count:
        return {}
    for _ in range(retreat_count):
        pending.popleft()
    unrun = deque(_pickup_retreat_groups(robot.backend, 2 * retreat_count))
    boundary = {'planned_steps': 2 * retreat_count, 'groups': [steps for _, steps in unrun],
                'continuous': len(unrun) == 1, 'completed_actions': [], 'heading_status': 'pending'}
    if 'retreat' in report:
        report.setdefault('additional_retreats', []).append(boundary)
    else:
        report['retreat'] = boundary
    saved_anchor=dict(getattr(getattr(nav,'mag',None),'anchor',None) or {})
    completed_groups = []
    while unrun:
        operation, strides = unrun.popleft()
        try:
            robot.Pickup_RetreatStep(operation)
        except PickupRetreatUnavailable:
            if operation != PICKUP_BACKWARD6_ACTION:
                raise
            # The MCU explicitly rejected this request before starting. Replan
            # every still-unexecuted stride, so an older four-step board uses 4+4.
            remaining_steps = strides + sum(count for _, count in unrun)
            unrun = deque(_pickup_retreat_groups(robot.backend, remaining_steps, allow_native_six=False))
            boundary.update(continuous=False, native_six_unavailable=True,
                            groups=completed_groups + [group for _,group in unrun])
            continue
        completed_groups.append(strides)
        boundary['completed_actions'].append(operation)
        report['completed_actions'].append(operation)

    read_tag = getattr(robot,'Pickup_ReadRetreatHeading',None)
    event_start = robot.events[-1] if robot.events else None
    if getattr(robot,'magnetic_route_only',False):
        if saved_anchor:
            Checkpoint_TrackEntryHeading(robot,nav,saved_anchor)
        if pickup_heading and robot.magnetic_route_only:
            robot.prepare_heading_segment(fresh=True, pickup_heading=pickup_heading)
    anchor = getattr(getattr(nav,'mag',None),'anchor',None) or {}
    if read_tag is not None:
        # The guard reads the settled camera buffer during magnetic sampling,
        # before a turn can invalidate either observation.
        robot.corridor_recover_heading(1,boundary='pickup_retreat_complete')
    else:
        robot.corridor_recover_heading(boundary='pickup_retreat_complete')
    heading = robot.last_heading_report or {}
    measured = CorridorTrial_HeadingSnapshot(heading, after=False, load=robot.load_kind)
    settled = _pickup_settled_heading(robot,heading)
    boundary.update(heading_status=heading.get('status', 'heading_unavailable'),
                    heading_source=('pickup_approach' if anchor.get('pickup_approach_heading')
                                    else 'loaded_reference') if measured else 'unavailable',
                    heading_before_correction_deg=measured[0] if measured else None,
                    residual_deg=settled[0] if settled else None,
                    correction_actions=heading.get('correction_actions', 0))
    view = heading.get('entry_followup') or {}
    first_turns = _pickup_turns_since(robot,event_start)
    if not measured and read_tag is not None and not boundary['correction_actions'] and not first_turns:
        view = read_tag(nav) or {}
    if view.get('fresh_tag') == 1:
        _pickup_use_retreat_tag(robot,nav,boundary,view,len(pending))
    severe_heading = max(25., 3 * nav.cfg['post_action_heading']['deadband_deg'])
    if boundary['residual_deg'] is not None and abs(boundary['residual_deg']) >= severe_heading:
        _recover_startup_side_heading(robot, nav, boundary, severe_heading)
    boundary['heading_actions'] = _pickup_turns_since(robot,event_start)
    if boundary['heading_actions']:
        boundary['correction_actions'] = len(boundary['heading_actions'])
    if boundary['correction_actions'] and boundary['residual_deg'] is None:
        boundary['heading_status'] = 'post_turn_unverified'
    if boundary['residual_deg'] is None and (boundary['correction_actions']
            or (callable(read_tag) and not getattr(robot.backend,'is_simulation',False))):
        boundary['next_group_max'] = 2
    return boundary


def _recover_startup_side_heading(robot, nav, boundary, severe_heading, *, side_group=False):
    """Recover once more from a verified large residual before taking another side stride."""
    event_start = robot.events[-1] if robot.events else None
    robot.corridor_recover_heading(boundary='pickup_side_boundary' if side_group else 'pickup_retreat_complete')
    heading = robot.last_heading_report or {}
    residual = _pickup_settled_heading(robot,heading)
    check = {'heading_status': heading.get('status','heading_unavailable'),
             'residual_deg': residual[0] if residual else None,
             'correction_actions': max(heading.get('correction_actions', 0),len(_pickup_turns_since(robot,event_start)))}
    boundary['heading_recheck'] = check
    boundary['correction_actions'] += check['correction_actions']
    if check['correction_actions'] and residual is None:
        boundary['residual_deg'] = None
    if residual is not None:
        boundary['residual_deg'] = residual[0]
        if abs(residual[0]) < severe_heading:
            return

    if getattr(getattr(nav, 'io', None), 'visual_motion_ready', None) is not None:
        view = {'expected_id':1, 'detected':False, 'observation_windows':0}
        observed, reason = Checkpoint_Observe(robot, nav, view, lambda: None,
            single_window=True, quick_id=True, reuse_settled_cache=True)
        hint = RoutePose_TrialLateralHint(view, nav.cfg, 1) if observed is CheckpointScanResult.CHECKPOINT_DETECTED else None
        check['tag1'] = {'status':observed.value, 'reason':reason, 'pose':hint}
        if (hint is not None and VisualCamera_IsFresh(nav.io.visual_motion_ready, hint['latest_stamp_ns'])
                and max(map(abs, hint['lateral_bounds_m'])) <= .10
                and abs(hint['heading_deg']) < severe_heading):
            boundary.update(residual_deg=hint['heading_deg'], heading_status='tag1_verified')
            rebase = getattr(robot, 'Route_RebaseTagHeading', None)
            if rebase is not None:
                check['heading_reference'] = rebase(1,hint['heading_deg'],hint['latest_stamp_ns']).value
            return

    if robot.cfg.get('route_steps', {}).get('continue_estimated_route') is True:
        boundary.update(side_start_estimated=True, reason_zh='侧跨前已纠偏，余差留给下一组侧跨和 Tag 1。')
        return
    residual = boundary.get('residual_deg')
    detail = f"航向余差 {residual:+.1f}°" if residual is not None else '转后航向待测'
    boundary.update(side_start_authorized=False,
        reason_zh=f"侧跨前{detail}，再次纠偏和扫码后仍未确认下一跨方向。")


def _pickup_side_tag1(robot, nav):
    cached = getattr(robot, 'boundary_visual_observation', None)
    view = None
    if cached is not None:
        sample = cached('pickup_side_boundary', expected_tag=1)
        if sample:
            view = Checkpoint_FromBoundary(robot, nav, 1, report=sample)
    if view is None:
        view = {'expected_id':1, 'detected':False, 'observation_windows':0}
        observed, _ = Checkpoint_Observe(robot, nav, view, lambda: None,
            single_window=True, quick_id=True, reuse_settled_cache=True)
        if observed is not CheckpointScanResult.CHECKPOINT_DETECTED:
            return None
    hint = RoutePose_TrialLateralHint(view, nav.cfg, 1)
    if (hint and .10 <= hint['distance_m'] <= .35 and max(map(abs,hint['lateral_bounds_m'])) <= .12
            and VisualCamera_IsFresh(nav.io.visual_motion_ready, hint['latest_stamp_ns'])):
        return hint
    return None


def _pickup_side_restore_heading(robot, nav, saved_anchor, check, remaining):
    """Try one Tag measurement or restore the same loaded compass reference at this stance."""
    turn = getattr(robot, 'Route_TurnFromTagHeading', None)
    rebase = getattr(robot, 'Route_RebaseTagHeading', None)
    camera = callable(getattr(nav.io, 'visual_motion_ready', None))
    hint = _pickup_side_tag1(robot, nav) if camera else None
    if hint is not None:
        observed = check['tag1_heading'] = {'before_deg':hint['heading_deg']}
        deadband = nav.cfg['post_action_heading']['deadband_deg']
        if turn and abs(hint['heading_deg']) > deadband:
            action = turn(1, hint['heading_deg'], hint['latest_stamp_ns'], reserved_actions=remaining)
            if action and action.get('action'):
                check['correction_actions'] += 1
                check['residual_deg'] = None
                observed['turn'] = action['action']
                old_stamp = hint['latest_stamp_ns']
                hint = _pickup_side_tag1(robot, nav)
                if hint and hint['latest_stamp_ns'] <= max(old_stamp, robot.last_action_completed_ros_ns):
                    hint = None
        if hint is not None:
            observed['residual_deg'] = hint['heading_deg']
            check.update(residual_deg=hint['heading_deg'], heading_status='pickup_side_tag1')
            if rebase is not None:
                observed['magnetic_reference'] = rebase(1,hint['heading_deg'],hint['latest_stamp_ns']).value
                if ('turn' not in observed and observed['magnetic_reference'] == 'applied'
                        and abs(hint['heading_deg']) > deadband):
                    _pickup_recover_magnetic_residual(robot, check, 'pickup_side_boundary')
            return
    if (saved_anchor and robot.magnetic_route_only and
            Checkpoint_TrackEntryHeading(robot, nav, saved_anchor).value == 'applied'):
        robot.corridor_recover_heading(boundary='pickup_side_boundary')
        recovered = robot.last_heading_report or {}
        estimate = _pickup_settled_heading(robot, recovered)
        check['correction_actions'] += recovered.get('correction_actions', 0)
        if estimate:
            check.update(residual_deg=estimate[0], heading_status='pickup_side_magnetic_restored')
            return
    check['next_group_max'] = 2


def _run_pickup_exit(robot, nav, *, actions=None, pickup_heading=None):
    steps = robot.cfg.get('route_steps', {})
    corridor = steps.get('corridor') or CorridorTrial_DefaultChain()
    ready = getattr(robot.backend, 'small_strafe_ready', lambda: False)
    command = getattr(robot, 'route_command', None)
    if actions is None:
        actions = _pickup_exit_actions(steps, corridor, small_ready=ready() and not robot.magnetic_route_only,
                                      command=command)
    else:
        actions = _pickup_side_groups(actions)
    report = {'completed_actions': [], 'lateral_heading_checks': []}
    robot.events.append({'pickup_exit': report})
    pending = deque(actions)
    deadband = nav.cfg['post_action_heading']['deadband_deg']
    startup = command in ('1', '2')
    live_pickup = (startup and getattr(robot, 'deliveries', 0) == 0 and getattr(nav, 'io', None) is not None
                   and nav.cfg['post_action_heading'].get('execute') and not getattr(robot.backend,'is_simulation',False))
    saved_anchor = dict(getattr(getattr(nav, 'mag', None), 'anchor', None) or {}) if live_pickup else {}
    previous = _run_startup_retreat(robot, nav, pending, report, pickup_heading=pickup_heading) if startup else {}
    if not saved_anchor and live_pickup:
        saved_anchor = dict(getattr(getattr(nav,'mag',None),'anchor',None) or {})
    if previous.get('side_start_authorized') is False:
        report.update(status='side_start_heading_unconfirmed', reason_zh=previous['reason_zh'])
        return report
    while pending:
        if startup and pending[0] == 'cmd0_carry_backward2':
            previous = _run_startup_retreat(robot, nav, pending, report)
            if previous.get('side_start_authorized') is False:
                report.update(status='side_start_heading_unconfirmed', reason_zh=previous['reason_zh'])
                return report
            continue
        action = pending.popleft()
        residual, drift = previous.get('residual_deg'), previous.get('drift_deg')
        side_count = {'cmd0_strafe_left3':3,'cmd0_strafe_left4':4}.get(action)
        # A small accepted residual with no worsening trend does not need an extra restart.
        limit = 2*deadband if startup and side_count==3 else deadband
        if (live_pickup and previous.get('heading_status') == 'corridor_heading_followup'
                and previous.get('correction_actions') == 0
                and (drift is None or residual is not None and drift*residual <= 0)):
            limit = max(limit,min(1.5*deadband,12.))
        if (side_count is not None and (previous.get('next_group_max') == 2 or residual is not None
                and (abs(residual) > limit
                     or (previous.get('correction_actions') == 0 and drift is not None
                         and abs(drift) > limit and drift * residual > 0)))):
            previous['next_group_split'] = {'action': action, 'groups': [2,side_count-2]}
            pending.appendleft('cmd0_strafe_left2' if side_count==4 else 'strafe_left_once')
            action = 'cmd0_strafe_left2'
        robot.check_magnetic_before_motion(configured=True)
        if action == SMALL_SIDE_ACTIONS['strafe_left_once'] and (robot.magnetic_route_only or not ready()):
            action = 'strafe_left_once'
        if action not in _PICKUP_SIDE_ACTIONS:
            robot.action(action)
            report['completed_actions'].append(action)
            previous = {}
            continue
        check_startup = live_pickup and 'left' in action
        if check_startup and not robot.magnetic_route_only and getattr(getattr(nav,'mag',None),'anchor',None):
            saved_anchor = dict(nav.mag.anchor)
        before = CorridorTrial_HeadingSnapshot(robot.last_heading_report or {}, after=True, load=robot.load_kind)
        robot.corridor_side_action(action)
        report['completed_actions'].append(action)
        event_start = robot.events[-1] if robot.events else None
        CorridorTrial_CheckLateralHeading(robot, nav, action, before, robot.last_heading_report or {},
            report, len(pending), expected_tag=None if pending else 1, allow_clearance_translation=False,
            reuse_side_sample=bool(check_startup))
        previous = report['lateral_heading_checks'][-1]
        if check_startup:
            # A receipt can precede a failed post-turn sample or a deactivated heading guard.
            turns = _pickup_turns_since(robot,event_start)
            if turns or previous['correction_actions']:
                heading = robot.last_heading_report or {}
                settled = _pickup_settled_heading(robot,heading) if heading.get('correction_actions') else None
                previous.update(residual_deg=settled[0] if settled else None,
                                correction_actions=max(len(turns),previous['correction_actions']))
        if check_startup and pending:
            left = previous.get('residual_deg')
            drift = previous.get('drift_deg')
            if (left is None or (not previous['correction_actions'] and abs(left) > deadband
                                 and drift is not None and abs(drift) >= 3. and left*drift > 0)):
                _pickup_side_restore_heading(robot, nav, saved_anchor, previous, len(pending))
        if startup and pending:
            residual = previous.get('residual_deg')
            if residual is None and not previous['correction_actions']:
                residual = previous.get('error_after_strafe_deg')
            severe_heading = max(25., 3 * deadband)
            if residual is not None and abs(residual) >= severe_heading:
                previous['residual_deg'] = residual
                _recover_startup_side_heading(robot, nav, previous, severe_heading, side_group=check_startup)
        if check_startup:
            previous['heading_actions'] = _pickup_turns_since(robot, event_start)
            previous['correction_actions'] = max(previous['correction_actions'],len(previous['heading_actions']))
            previous['heading_check_pending'] = False
            if previous['residual_deg'] is None:
                previous['heading_feedback'] = 'turn_heading_unverified' if previous['correction_actions'] else 'unavailable'
            else:
                previous['heading_feedback'] = 'turn_completed' if previous['correction_actions'] else 'measured_only'
        if previous.get('side_start_authorized') is False:
            report.update(status='side_start_heading_unconfirmed', reason_zh=previous['reason_zh'])
            return report
    return report


def _tag1_pickup_adjustment_limits(steps):
    return steps.get('wall_pickup_advance_steps',2)//2, steps.get('wall_pickup_side_limit',2)


def _tag1_pickup_exit_actions(approach):
    # The turn precedes pickup. Undo the actual walk in the same body frame, in reverse order.
    reverse = {'empty_forward2': ('back', 2), 'empty_forward4': ('back', 4), 'free_forward': ('back', 5),
               'empty_strafe_left': ('strafe_right_once', 1), 'empty_strafe_right': ('strafe_left_once', 1)}
    for count in (2, 3):
        reverse[f'empty_strafe_left{count}'] = ('strafe_right_once', count)
        reverse[f'empty_strafe_right{count}'] = ('strafe_left_once', count)
    actions = []
    for direction, group in groupby((reverse[name] for name in reversed(approach)), key=lambda motion: motion[0]):
        count = sum(strides for _, strides in group)
        if direction == 'back':
            if count % 2:
                raise ValueError('接驳取物区同一段连续前进须为偶数步，以便按后退动作的两步单位原路返回')
            actions.extend(['cmd0_carry_backward2'] * (count // 2))
        else:
            actions.extend(CorridorTrial_GroupStrafes(direction, count, continuous=True, monitor_every=4))
    return tuple(actions)


def OutboundPickup_Prepare(mode, cfg, project, simulation=False):
    """Validate pickup calibration independently of the old return-route windows."""
    if mode not in ('1', '2'):
        raise ValueError('single-outbound pickup only supports command 1 or 2')
    validate_transfer(cfg, mode)
    project = Path(project)
    cfg = dict(cfg, route_steps=Cmd0Motion_LoadRouteSteps(project))
    path = project/'config'/'mission_v5.json'
    profile = json.loads(path.read_text(encoding='utf-8')) if path.exists() else defaults()
    if profile.get('schema') != 'r3.mission-v5.v1':
        raise ValueError('V5 pickup configuration schema')
    validate_pickup(profile['pickup'])
    validate_config(profile['vision'], live=False)
    if not simulation:
        staged = profile.get('staged_block_count')
        if profile.get('enabled') is True and profile.get('field_calibration_verified') is True:
            validate_config(profile['vision'], live=True)
        elif not staged:
            raise NotReady('task {} not ready BEFORE motion: field-verified block pickup calibration required'.format(mode))
        else:
            vision = validate_config(dict(profile['vision'], **staged))
            if set(vision['hsv_ranges']) != {'green','orange'}:
                raise ValueError('staged_block_count 需要绿色和橙色的 HSV 范围')
            profile['staged_vision'] = vision

    chains = validate_chains(demo_chains() if simulation else CHAINS, '1')
    nav = load_navigation(project)
    if not simulation:
        visual = nav.get('visual_micro', {})
        heading = nav.get('post_action_heading', {})
        if (nav.get('enabled') is not True
                or cfg.get('transfer_test', {}).get('target_id') != 1
                or visual.get('enabled') is not True or visual.get('execute') is not True
                or visual.get('confirmed_target_id') != 1
                or [goal['id'] for goal in nav['legs']['outbound']] != [1, 2, 3, 4, 5]):
            raise NotReady(f'task {mode} not ready BEFORE motion: shared corridor requires confirmed Tag 1–5 and live Tag 1 alignment')
        magnetic = nav.get('magnetic', {})
        if (heading.get('enabled') is not True or heading.get('execute') is not True
                or heading.get('micro_trial', {}).get('enabled') is not True
                or magnetic.get('capture_freshness_verified') is not True
                or magnetic.get('heading_sign') not in (-1, 1)):
            raise NotReady('shared corridor requires verified magnetic monitoring BEFORE motion')
        if profile.get('staged_vision',profile['vision'])['topic'] != nav['camera_health']['topic']:
            raise NotReady('pickup and shared corridor require the same calibrated chest image topic')

    steps = cfg['route_steps']
    corridor = steps.get('corridor', CorridorTrial_DefaultChain())
    exit_cost = max(_pickup_exit_macro_budget(_pickup_exit_actions(steps, corridor, small_ready=ready,command=mode))
                    for ready in (False, True))
    entry_ops, _ = EntranceTrial_PlanForward(steps['tag1_to_tag2_forward_steps'],
                                             continuous=True, legacy=False)
    entry_reserve = 2 if (steps.get('tag2_forward_trial') or {}).get('enabled') else 0
    guard_tag4 = bool(steps.get('continue_estimated_route') and (steps.get('tag4_forward_trial') or {}).get('enabled'))
    tag5_passage = (steps.get('tag5_entry_lateral') or {}).get('verify_passage') is True
    extra_home_steps = steps.get('command2_before_base_turn_extra_steps', 0) if mode == '2' else 0
    outbound_cost = (1 + len(entry_ops) + entry_reserve
                     + CorridorTrial_MaxBodyMacros(corridor, guard_tag4=guard_tag4, tag5_passage=tag5_passage,
                                                  extra_home_steps=extra_home_steps)
                     + nav['max_extra_actions_per_journey'])
    worst = outbound_cost + exit_cost
    if mode == '2':
        from return_trial import ReturnTrial_DefaultChain, ReturnTrial_MaxBodyMacros
        if steps.get('command2_continue_return') is True:
            if [goal['id'] for goal in nav['legs']['return']] != [5,6,7,1]:
                raise NotReady('command 2 requires the return checkpoint order 5, 6, 7, 1 BEFORE motion')
            return_chain = steps.get('return_chain') or ReturnTrial_DefaultChain()
            if any(abs(return_chain[name]['target_delta_deg']) != 180 for name in ('base_turn','transfer_turn')):
                raise ValueError('循环返程需在大本营和中转区掉头 180°')
            max_adjustments = sum(_tag1_pickup_adjustment_limits(steps))
            station = [step['op'] for step in return_chain['station_side'] for _ in range(step.get('repeat',1))]
            retrace = _tag1_pickup_exit_actions(station)
            worst = max(worst, outbound_cost + _pickup_exit_macro_budget(retrace) + max_adjustments)
            transfer = sum(step.get('repeat',1) for step in return_chain['transfer_approach'])
            empty_cost = (ReturnTrial_MaxBodyMacros(return_chain) + transfer + return_chain['transfer_turn']['repeat']
                          + len(station) + max_adjustments + 1 + nav['max_extra_actions_per_journey'])
            if empty_cost > cfg['max_body_macros_per_journey']:
                raise NotReady('empty return exceeds journey macro budget BEFORE motion')
            worst = max(worst, empty_cost)
        else:
            return_chain = steps.get('return_chain') or ReturnTrial_DefaultChain()
            worst += return_chain['base_turn']['repeat']
    if worst > cfg['max_body_macros_per_journey']:
        raise NotReady('pickup and shared corridor exceed journey macro budget BEFORE motion')

    return {'outbound_pickup': True, 'mode': mode, 'config': cfg, 'project': str(project),
            'routes': None, 'navigation': nav, 'mission_v5': profile, 'chains_v5': chains,
            'simulation_only': simulation, 'v5_worst_case_body_macros': worst,
            'route_evidence': 'estimated_trial'}


class _PickupCamera(BlockCamera):
    def observe(self, after_monotonic, after_ros_ns, check, **options):
        read=super().observe
        try:
            return VisualCamera_Read(lambda guard:read(after_monotonic, after_ros_ns, guard, **options), check)
        except VisualCameraUnavailable as exc:
            raise BlockFrameUnavailable(str(exc)) from exc


class _OutboundPickupRobot(MissionRobot):
    def configure(self, plan):
        super().configure(plan)
        self.route_command = plan['mode']
        self.outbound_only = (self.route_command == '1'
                              or self.cfg.get('route_steps', {}).get('command2_continue_return') is not True)
        self.boundary_recovery_command0 = True
        self.command0_scan_after_turn = True
        self.pair_route_enabled = True
        self.pickup_active = True

    def sensor_preflight(self):
        return super().sensor_preflight(magnetic_optional=True)

    def finite(self, op, observation=None, **options):
        if (self.pickup_active and not getattr(self.backend, 'is_simulation', False)
                and not options.get('configured', False)
                and options.get('pickup_station') is None
                and op in VISUAL_PICKUP_OPS):
            if observation is None and op not in ('grab_single','grab_double'):
                observation = self.last_block
            options['visual_pickup'] = True
        return super().finite(op, observation, **options)

    def _record_progress(self, phase, **detail):
        detail.setdefault('direction', 'outbound')
        detail.setdefault('journey', self.deliveries+int(detail['direction']=='outbound'))
        super()._record_progress(phase, command=self.route_command, outbound_only=self.outbound_only,
                                 shared_outbound=detail['direction']=='outbound', delivery_planned=True, **detail)

    def supports_pair(self):
        operations = getattr(self.backend, 'mission_pair_ops', None)
        return operations is not None and PAIR_REQUIRED_OPS.issubset(operations())

    def _open_block_camera(self,count_only):
        try:
            return self._create_pickup_camera(count_only)
        except OSError as exc:
            raise BlockFrameUnavailable('胸部相机暂时无法连接：'+str(exc)) from exc

    def _create_pickup_camera(self,count_only):
        vision = self.mission.get('staged_vision')
        trial_alignment = not self.outbound_only and self.deliveries>0
        if vision is None or not (count_only or trial_alignment):
            return _PickupCamera(self.mission['vision'],scope=self.scope())
        snapshot = self.ensure_navigation_io().io.preflight_health_snapshot()
        info = snapshot.get('camera_info', {})
        if (snapshot.get('camera_healthy') is not True or info.get('ready') is not True
                or not info.get('frame_id') or not info.get('publisher')):
            raise BlockFrameUnavailable('等待已核验的胸部相机图像')
        config = dict(vision, frame_id=info['frame_id'], topic_confirmed=True)
        if trial_alignment:
            left, top, right, bottom = vision['roi']
            # The corner target may enter the sides of the chest camera view.
            # Keep the staged vertical crop and use the same confirmed grasp band.
            config['roi'] = [min(left, .08), top, max(right, .92), bottom]
            config['tag1_handoff'] = True
        if [info.get('width'), info.get('height')] != config['resolution']:
            raise BlockFrameUnavailable('方块识别图像尺寸与胸部相机不一致')
        return _PickupCamera(config, scope=self.scope(), staged_publisher=info['publisher'],
                             trial_alignment=trial_alignment)

    def finish_pickup(self):
        self.last_block = None
        if self.camera is not None:
            self.camera.close()
            self.camera = None
        self.pickup_active = False

    def completed_call(self, name, timeout_s):
        if self.load_kind == 'double' and name in PAIR_ROUTE_OPS:
            if not self.supports_pair():
                raise NotReady('双抱动作能力未验证，无法发出 '+name)
            name = PAIR_ROUTE_OPS[name]
        if self.pickup_active:
            return super().completed_call(name, timeout_s)
        # The shared worker retains command 0's existing marker-boundary checks.
        return Robot.completed_call(self, name, timeout_s)


def _restart_leg_budget(robot):
    completed = (robot.last_action_completed_monotonic, robot.last_action_completed_ros_ns)
    robot.new_journey()
    robot.last_action_completed_monotonic, robot.last_action_completed_ros_ns = completed


def _acquire_near_tag1(robot):
    """Locate the block or its floor row in the empty chest view from Tag 1."""
    forward_limit, lateral_limit = _tag1_pickup_adjustment_limits(robot.cfg.get('route_steps', {}))
    moves = []
    event = {'status': 'watching', 'anchor_tag': 1, 'adjustments': moves, 'approach_steps': 0, 'checks': 0,
             'position_basis': 'normalized_image_floor_row', 'block_presence_required': True}
    robot.events.append({'tag1_pickup_standby': event})
    robot._record_progress('waiting_blocks', direction='return', reason_zh='正在 Tag 1 附近观察接驳方块。')

    def adjust(name,row):
        op = _PICKUP_ADJUSTMENTS[name]
        forward = moves.count('empty_forward2')
        lateral = len(moves)-forward
        if (op=='empty_forward2' and forward>=forward_limit) or (op!='empty_forward2' and lateral>=lateral_limit):
            return False
        robot._record_progress('start_wall_pickup', direction='return', reason_zh='已定位接驳方块，开始短步进近。')
        # Revalidate the same image at submission. After motion, pickup requires
        # fresh frames from the new stance before another step or a grab.
        robot.finite(op, observation=row, checkpoint_follows=True,
                     before_dispatch=lambda:robot.verify_block_observation(row))
        moves.append(op)
        event['approach_steps'] = 2*moves.count('empty_forward2')
        return True

    with robot._pause_route_timer():
        stalled = 0
        unresolved_checks = 0
        blocked_pair = None
        for attempt in range(_TAG1_PICKUP_RECHECKS):
            robot.check()
            event['checks'] = attempt + 1
            live = not getattr(robot.backend,'is_simulation',False)
            completed_moves = len(moves)
            if live and robot.backend.current(require_mcu=False).get('mission_v5_ready') is not True:
                acquired = {'status':'no_pickup', 'reason':'mcu_pickup_permit_unavailable',
                            'reason_zh':'Tag 1 接驳取物等待当前会话的 MCU 自动动作许可。'}
            else:
                try:
                    acquired = robot.pickup.acquire(adjust=adjust,allow_stacked=True)
                except (BlockFrameUnavailable,BlockEvidenceExpired,PickupObservationUnavailable) as exc:
                    if robot.pickup.attempted:
                        raise
                    acquired = {'status':'no_pickup','reason':str(exc),'vision_unresolved':True}
                if (robot.cfg.get('route_steps',{}).get('continue_estimated_route') is True
                        and acquired['status']=='no_pickup'
                        and acquired.get('reason') in ('target_changed','too_close','outside_handoff_adjustment_range','adjustment_budget')
                        and acquired.get('reason_code')!='double_load_capability_unavailable'):
                    unresolved_checks = unresolved_checks + 1 if len(moves)==completed_moves else 0
                    if (unresolved_checks>=3 or attempt+1==_TAG1_PICKUP_RECHECKS
                            or (len(moves)==completed_moves and stalled+1>=_PICKUP_RECHECKS)):
                        acquired=robot.pickup.Pickup_AcquireAtHandoff(acquired)
                else:
                    unresolved_checks=0
            robot.events.append({'pickup': acquired})
            event.update(status=acquired['status'],reason=acquired.get('reason'))
            if acquired['status']!='no_pickup':
                return acquired,moves
            if (acquired.get('scene_empty') or acquired.get('reason') == 'empty scene visibility not validated'
                    or len(moves) > completed_moves):
                stalled = 0
                blocked_pair = None
            else:
                stalled += 1
                if acquired.get('reason_code') == 'double_load_capability_unavailable':
                    blocked_pair = acquired
            diagnosis = blocked_pair or acquired
            if stalled >= _PICKUP_RECHECKS or attempt + 1 == _TAG1_PICKUP_RECHECKS:
                event.update(status='unconfirmed', reason=diagnosis['reason'])
                if diagnosis['reason']=='mcu_pickup_permit_unavailable':
                    return dict(diagnosis, reason_zh='Tag 1 接驳区已多次复查，当前会话的 MCU 取物许可仍不可用。'), moves
                reason = ('Tag 1 附近连续未看到方块，本轮已完成一次搬运。'
                          if diagnosis.get('scene_empty') else
                          'Tag 1 附近多次观察仍未找到可抱取的方块，请检查相机和方块摆放。')
                return dict(diagnosis, reason_zh=diagnosis.get('reason_zh') or reason), moves
            robot._record_progress('waiting_blocks', direction='return', reason=diagnosis['reason'],
                                   reason_zh=diagnosis.get('reason_zh'))
            robot.wait(robot.empty_retry_s)


def OutboundPickup_Run(plan, backend, robot_factory=_OutboundPickupRobot):
    """Use one automatic session for delivery and the configured endpoint."""
    simulation = getattr(backend, 'is_simulation', False)
    if bool(plan.get('simulation_only')) != bool(simulation):
        raise RuntimeError('first-outbound plan/backend simulation mode mismatch')
    robot = robot_factory(backend, plan['config'], plan['project'])
    robot.configure(plan)
    timeout = robot.cfg['request_timeout_s']
    steps = robot.cfg.get('route_steps', {})
    repeat = plan['mode']=='2' and steps.get('command2_continue_return') is True
    result = {'command': plan['mode'], 'outbound_only': not repeat, 'return_started': False,
              'deliveries': 0, 'expected_blocks_delivered': 0,
              'completed_returns': 0,
              'release_protocol_completed': False, 'physical_grip_confirmed': False,
              'physical_release_confirmed': False, 'physical_load_sensor_confirmation': False}
    try:
        if not simulation:
            verify_pickup = getattr(backend, 'verify_pickup_capabilities', None) or backend.verify_cmd0_continuous
            verify_pickup(timeout)
            if backend.current().get('mission_v5_ready') is not True:
                raise NotReady('取物需要当前会话的 MCU 抓取和路线能力回执')
            robot.sensor_preflight()

        backend.call('begin', timeout)
        if not Cmd0Motion_ContinuousReady(backend):
            raise NotReady('continuous backward/lateral/outbound capability unavailable before pickup')
        while True:
            robot.prepare_heading_segment()
            guard = robot.heading_guard
            second_trip = result['completed_returns'] > 0
            pickup_heading = dict(guard.reference.anchor or {}) if guard else None
            exit_actions = None
            if second_trip:
                acquired,moves = _acquire_near_tag1(robot)
                if acquired['status']=='no_pickup':
                    result.update(status='pickup_unconfirmed', pickup=acquired, reason=acquired['reason_zh'])
                    break
                exit_actions = _tag1_pickup_exit_actions(result['return_trial']['station_side_actions'] + moves)
                _restart_leg_budget(robot)
            else:
                robot._record_progress('pickup')
                blocked_pair = None
                for attempt in range(_PICKUP_RECHECKS):
                    robot.check()
                    try:
                        acquired = robot.pickup.Pickup_AcquireStaged(allow_stacked=True, default_pair=True)
                    except (BlockFrameUnavailable,BlockEvidenceExpired,PickupObservationUnavailable) as exc:
                        acquired = {'status': 'no_pickup', 'reason': str(exc)}
                    if (steps.get('continue_estimated_route') is True and acquired['status']=='no_pickup'
                            and acquired.get('reason_code')!='double_load_capability_unavailable'
                            and blocked_pair is None):
                        acquired=robot.pickup.Pickup_AcquireDefaultPair('operator_staged',acquired['reason'])
                    robot.events.append({'pickup': acquired})
                    if acquired['status']!='no_pickup':
                        break
                    if acquired.get('reason_code') == 'double_load_capability_unavailable':
                        blocked_pair = acquired
                    diagnosis = blocked_pair or acquired
                    robot._record_progress('pickup',reason=diagnosis['reason'],
                                           observed_count=diagnosis.get('observed_count'),
                                           reason_code=diagnosis.get('reason_code'),reason_zh=diagnosis.get('reason_zh'))
                    if attempt + 1 < _PICKUP_RECHECKS:
                        robot.wait(robot.empty_retry_s)
                else:
                    result.update(status='pickup_unconfirmed', pickup=diagnosis,
                                  reason=(blocked_pair['reason_zh'] if blocked_pair
                                          else '多次复核仍无法确认取物布局，请重新摆放方块后再启动。'))
                    break

            if (robot.load_kind not in ('single','double') or acquired['expected_load'] != robot.load_kind
                    or acquired['status'] != robot.load_kind+'_pickup_protocol_completed'):
                raise RuntimeError('shared outbound requires a confirmed pickup response for the selected load')
            robot.finish_pickup()
            result['pickup'] = acquired
            nav = robot.ensure_navigation_io()
            nav.mag.anchor = None
            robot.prepare_heading_segment(fresh=True, pickup_heading=pickup_heading, before_pickup_retreat=True)
            robot._record_progress('transfer_to_entry',observed_count=acquired['observed_count'],
                                   grip_count=acquired['grip_count'],planned_count=acquired['planned_count'],
                                   stacked_triple=acquired['stacked_triple'])
            result['pickup_exit'] = _run_pickup_exit(robot, nav, actions=exit_actions, pickup_heading=pickup_heading)

            for key in ('next_segment','corridor_trial','checkpoint'):
                result.pop(key,None)
            if result['pickup_exit'].get('status') == 'side_start_heading_unconfirmed':
                result.update(status='pickup_exit_heading_unconfirmed', stage=1, next_stage_started=False,
                              reason=result['pickup_exit']['reason_zh'])
                break
            robot._record_progress('outbound_entry_scan', expected_tag=1)
            entry_check = {'tag_id': 1, 'confirmed_for_forward': False, 'status': 'scanning'}
            robot.events.append({'pickup_exit_entry': entry_check})
            scan = robot.scan_ar()
            alignment = scan.get('navigation_checkpoint', {})
            entry_accepted = EntranceTrial_IsEntryAccepted(alignment)
            entry_scan = alignment.get('scan') or {}
            entry_check.update(status='checked', confirmed_for_forward=entry_accepted,
                seen_ids=entry_scan.get('observed_ids', entry_scan.get('ids', [])),
                heading_error_deg=(alignment.get('decision_fusion') or {}).get('error_deg'))
            result.update(status='stage1_partial_alignment', stage=1, scan_result=scan,
                          entry_alignment=alignment, next_stage_started=False, release_protocol_completed=False,
                          reason=alignment.get('reason') or 'Tag 1 entry alignment unconfirmed')
            if entry_accepted or steps.get('continue_estimated_route') is True:
                segment = EntranceTrial_Run(robot, camera_info=alignment.get('camera_info'), entry_alignment=alignment)
                result.update(status='stage2_trial_unconfirmed', stage=2, next_segment=segment,
                              next_stage_started=segment['next_stage_started'], reason=segment['reason'])
                if segment['detected'] or segment.get('continue_estimated') is True:
                    turns_at_base = plan['mode'] == '2' and result['completed_returns'] == 0
                    extra_home_steps = steps.get('command2_before_base_turn_extra_steps', 0) if turns_at_base else 0
                    corridor = CorridorTrial_Run(robot, segment, entry_alignment=alignment,
                                                  extra_home_steps=extra_home_steps)
                    delivered = corridor['release_protocol_completed']
                    result.update(status='task_complete' if delivered else 'stage3_trial_unconfirmed',
                                  stage=4 if delivered else 3, checkpoint=corridor['checkpoint'],
                                  corridor_trial=corridor, reason=corridor['reason'], release_protocol_completed=delivered)
            if not result['release_protocol_completed']:
                break
            robot.deliveries+=1
            robot.blocks_delivered+=acquired['planned_count']
            result.update(deliveries=robot.deliveries,expected_blocks_delivered=robot.blocks_delivered)
            if result['completed_returns']:
                result['reason'] = '已完成两次放物，机器人在大本营结束本轮任务。'
                break
            if not repeat:
                if plan['mode'] == '2':
                    from return_trial import ReturnTrial_DefaultChain, ReturnTrial_TurnAtBase
                    turn = (steps.get('return_chain') or ReturnTrial_DefaultChain())['base_turn']
                    end = {'target_delta_deg': turn['target_delta_deg'], 'protocol_completed': False,
                           'stand_after_release': ('included_in_place_double' if acquired['grip_count'] == 2
                                                   else 'included_in_cmd0_place_single')}
                    result['base_turn_trial'] = end
                    robot.events.append({'base_turn_trial': end})
                    robot._record_progress('cmd2_base_turn', expected_tag=5, last_passed_tag=result.get('checkpoint'))
                    ReturnTrial_TurnAtBase(robot, turn)
                    end['protocol_completed'] = True
                    result['reason'] = '大本营已完成放物、起立和空载掉头，本轮结束。'
                break
            if simulation:
                backend.cycle_boundary(robot)
            from return_trial import ReturnTrial_DefaultChain, ReturnTrial_ReachPickupStation, ReturnTrial_Run
            _restart_leg_budget(robot)
            result['return_started']=True
            returned=ReturnTrial_Run(robot)
            result['return_trial']=returned
            if not returned['station_reached']:
                result.update(status='return_trial_unconfirmed', checkpoint=returned.get('checkpoint'), reason=returned['reason'])
                break
            result['completed_returns']+=1
            station = ReturnTrial_ReachPickupStation(robot, steps.get('return_chain') or ReturnTrial_DefaultChain(), returned)
            if not station['ready']:
                result.update(status='return_trial_unconfirmed', checkpoint=1, reason=station['reason_zh'])
                break
            robot.pickup.reset_after_delivery()
            robot.pickup_active=True

        robot.check()
        backend.call('finish', timeout)
        result.update(events=list(robot.events), expected_load=robot.load_kind)
        return result
    except BaseException as exc:
        exc.route_events = list(robot.events)
        exc.completed_deliveries = result['deliveries']
        exc.expected_blocks_delivered = result['expected_blocks_delivered']
        exc.completed_returns = result['completed_returns']
        exc.return_started = result['return_started']
        backend.cancel()
        raise
    finally:
        robot.close()


class _SyntheticNavigation:
    """Supply ordered placeholder IDs without importing ROS or image libraries."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.io = self
        self.mag = SimpleNamespace(anchor=None)
        self.stamp = 1

    def ros_now_ns(self):
        self.stamp += 1
        return self.stamp

    def preflight_health_snapshot(self):
        return {'camera_healthy': True, 'camera_info': {'ready': True, 'fingerprint': 'synthetic'}}

    def observe(self, tag, after, timeout, check):
        check()
        rows = [{'id': tag, 'stamp_ns': self.ros_now_ns(),
                 'source_topic': self.cfg['topic'], 'frame_id': self.cfg['pose_contract']['frame']}
                for _ in range(self.cfg['min_frames'])]
        return dict(self.preflight_health_snapshot(), observations=rows, after_ns=after,
                    now_ns=self.ros_now_ns(), contract_match=True)

    def close(self):
        pass


class _SyntheticOutboundRobot(_OutboundPickupRobot):
    def configure(self, plan):
        super().configure(plan)
        self.navigator = _SyntheticNavigation(plan['navigation'])

    def ensure_navigation_io(self):
        return self.navigator

    def prepare_heading_segment(self, *, fresh=False, pickup_heading=None, before_pickup_retreat=False):
        self.navigator.mag.anchor = {'load': self.load_kind}

    def settle_since_last_completed(self, seconds):
        self.check()

    def chain_step(self, step, **options):
        super().chain_step(step, **options)
        if self.load_kind == 'empty' and abs(step.get('target_delta_deg', 0)) == 180:
            # The dry-run backend models completed turns at the commanded angle.
            self.navigator.mag.anchor = {'load':'empty', 'planned_group_target':True}
            self.last_heading_report = {'error_deg':0., 'simulation_only':True, 'sample':{'reference':{
                'load':'empty', 'capture_verified':True, 'usable_for_heading':True,
                'received_monotonic':self.clock(), 'capture_session':'simulation',
                'load_epoch':0, 'calibration_id':'synthetic_empty_turn'}}}

    def scan_ar(self):
        camera = self.navigator.preflight_health_snapshot()['camera_info']
        return {'status': 'recognized', 'ids': [1], 'navigation_checkpoint': {
            'status': 'synthetic', 'expected_id': 1, 'fresh_target_detected': True,
            'pose_accepted': True, 'heading_aligned': True,
            'decision_fusion': {'source': 'trusted_tag_heading', 'target_heading_deg': 0},
            'camera_info': camera, 'scan': {'ids': [1], 'pose_usable': True}}}


def OutboundPickup_Simulate(plan, backend=None):
    """Run the same orchestration with synthetic block and Tag observations."""
    from simulate_v5 import ScenarioBackend, SimulationStop

    class _SyntheticBridge:
        def __init__(self, backend):
            self.backend = backend

        def __getattr__(self, name):
            return getattr(self.backend, name)

        def cmd0_continuous_ready(self):
            return True

    backend = backend if backend is not None else ScenarioBackend()
    if not plan.get('simulation_only') or not getattr(backend, 'is_simulation', False):
        raise RuntimeError('synthetic pickup rehearsal requires an explicit simulation plan and backend')
    try:
        result = OutboundPickup_Run(plan, _SyntheticBridge(backend), _SyntheticOutboundRobot)
    except SimulationStop as exc:
        repeat = plan['mode']=='2' and plan['config'].get('route_steps', {}).get('command2_continue_return') is True
        result = {'status': 'simulation_cancelled', 'command': plan['mode'], 'outbound_only': not repeat,
                  'reason': str(exc), 'events': getattr(exc, 'route_events', []),
                  'deliveries':getattr(exc,'completed_deliveries',0),
                  'expected_blocks_delivered':getattr(exc,'expected_blocks_delivered',0),
                  'completed_returns':getattr(exc,'completed_returns',0),
                  'return_started':getattr(exc,'return_started',False)}
    return {'status': 'simulation_only', 'calls': backend.calls, 'result': result,
            'body_commands': 0, 'field_readiness_proven': False,
            'route_evidence': plan['route_evidence']}
