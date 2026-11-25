"""Continue command 0 from Tag 2 to Tag 5 in its existing automatic session."""

from enum import Enum
from math import isfinite, radians, sin
from statistics import median

from action_chains_v5 import COMMAND0_OUTBOUND, COMMAND0_DELIVERY
from cmd0_motion import (Cmd0Motion_ContinuousReady, Cmd0Motion_ForwardStepCount, Cmd0Motion_LegacyReady,
                         PLACE_ACTION, SIDE_ACTIONS, SMALL_SIDE_ACTIONS)
from entrance_trial import (EntranceTrial_BoundaryCheckpoint, EntranceTrial_ObserveCheckpoint,
                            EntranceTrial_PlanForward, EntranceTrialResult)
from nav_observation import wrap
from route_pose import (TAG2_APPROACH_COUNTS, RoutePose_AdviseDeparture, RoutePose_Load, RoutePose_Measure,
                        RoutePose_PartialCheckpointGeometry,
                        RoutePose_PlanSideCount, RoutePose_PlanSmallSide, RoutePose_PlanTag2ForwardTrim,
                        RoutePose_PlanTag2ForwardTrial, RoutePose_PlanTag4Longitudinal, RoutePose_PlanTag5EntryLateral,
                        RoutePose_PlanTag5Passage, RoutePose_Tag3HeadingRecovery, RoutePose_TrialLateralHint)


_CHECKPOINTS = (2, 3, 4, 5)
_LEGS = tuple((start, end, f'tag{start}_to_tag{end}') for start, end in zip(_CHECKPOINTS, _CHECKPOINTS[1:]))
_TAG5_LEG = _LEGS[-1][2]
_DROP = 'tag5_to_drop'
_SIDE_ACTIONS = ('strafe_left_once', 'strafe_right_once')
_FORWARD_GROUP = 'carry_forward_group'
_DEPARTURE_REOBSERVATIONS = 3
_TAG2_DEPARTURE_WINDOWS = 2
_TAG2_BOUNDARY_HINT_MAX_AGE_NS = 8_000_000_000
_TAG4_APPROACH_GROUPS = 4
_TAG4_STRAIGHT_LATERAL_M = .16
_TAG4_CLEARANCE_YAW_DEG = 22.
_TAG4_RECOVERY_CENTER_STRAFES = 2
_TAG4_NEAR_LEFT_MARGIN_M = .015
_TAG4_LEFT_PROBE_DEPTH_M = .15
_TAG4_LEFT_CLEARANCE_FLOOR_M = .125
_TAG4_RECOVERY_STRAFE_UPPER_M = .10  # A field cycle shifted roughly 9 cm.
_LARGE_STRAFE_TRAVEL_UPPER_M = .075
_TAG3_RIGHT_OFFSET_M = .075
_TAG3_TRIAL_MAX_RIGHT_STRIDES = 7
_TAG2_CARRIED_LATERAL_MARGIN_M = .02
_TAG2_CARRIED_MAX_YAW_DEG = 12.
_TAG2_CAUTION_LATERAL_M = .05
_TAG2_APPROACH_NEAR_MARGIN_M = .025
_TAG2_DEPARTURE_LATERAL_M = .16
_TAG2_CLEARANCE_YAW_DEG = 22.
_TAG3_CLEARANCE_YAW_DEG = 20.
_LATERAL_LARGE_DRIFT_DEG = 30.
_LATERAL_CLEARANCE_MAX_GROUPS = 3
_LATERAL_CLEARANCE_HEADING_DEG = 45.
_TAG5_ENTRY_TRIMS = 2
_TAG5_STEP_UPPER_M = .05
_TAG5_OUTSIDE_RECOVERY_DEPTH_M = .24
_TAG5_OUTSIDE_SIDE_DEPTH_M = .18
_TAG5_TIGHT_SIDE_DEPTH_M = .14
_TAG5_TIGHT_TURN_MARGIN_M = .025
_TAG5_SMALL_SIDE_TRAVEL_UPPER_M = .035
_TAG5_CLEARANCE_YAW_DEG = 22.
_TAG5_RETREAT_TRAVEL_UPPER_M = .125
_TAG5_SIDE_PROGRESS_M = .005
_TAG5_TURN_PROGRESS_DEG = 1.5
_TAG5_IDLE_WINDOWS = 4


class CorridorTrialResult(Enum):
    ROUTE_UNAVAILABLE = 'route_unavailable'
    MOVING_TO_CHECKPOINT = 'moving_to_checkpoint'
    WAITING_FOR_CHECKPOINT = 'waiting_for_checkpoint'
    CAMERA_UNAVAILABLE = 'camera_unavailable'
    CHECKPOINT_NOT_SEEN = 'checkpoint_not_seen'
    DEPARTURE_UNSAFE = 'departure_unsafe'
    CHECKPOINT_DETECTED = 'checkpoint_detected'
    DELIVERY_COMPLETED = 'delivery_protocol_completed'


class CorridorLateralHeadingResult(Enum):
    PROCEED = 'proceed'
    RECHECK_CLEARANCE = 'recheck_clearance'
    DEFERRED_TO_SMALL_STRAFE = 'deferred_to_small_strafe'


def CorridorTrial_DefaultChain():
    suffix = COMMAND0_OUTBOUND[2:]
    markers = {step['tag']: i for i, step in enumerate(suffix) if 'tag' in step}
    chain = {name: [dict(step) for step in suffix[markers[start]+1:markers[end]]]
             for start, end, name in _LEGS}
    chain[_DROP] = [dict(step) for step in COMMAND0_DELIVERY]
    return chain


def CorridorTrial_ValidateChain(chain, *, extra_home_steps=0):
    """Normalize the four legs, with an empty Tag 4 to Tag 5 leg scanning in place."""
    names = (*(name for _, _, name in _LEGS), _DROP)
    if not isinstance(chain, dict) or chain.keys() != set(names):
        raise ValueError('通道动作链须包含这四段 ' + '、'.join(names))
    normalized = {}
    for name in names:
        steps = chain[name]
        if not isinstance(steps, list):
            raise ValueError(f'{name} 须填写动作列表')
        if not steps and name != _TAG5_LEG:
            raise ValueError(f'{name} 须填写非空动作列表')
        if name == _DROP and (len(steps) < 2 or steps[-1] != {'op': 'place_single'}):
            raise ValueError(f'{name} 须先前进，再以一次 place_single 结束')
        if name == _DROP and extra_home_steps:
            steps = [*steps[:-1], {'op': _FORWARD_GROUP, 'steps': extra_home_steps}, steps[-1]]
        normalized[name] = []
        for index, step in enumerate(steps, 1):
            label = f'{name} 的第 {index} 个动作'
            if not isinstance(step, dict):
                raise ValueError(f'{label} 须填写动作对象')
            op = step.get('op')
            if name == _DROP and index == len(steps):
                normalized[name].append({'op': 'place_single'})
            elif op == _FORWARD_GROUP:
                distance = step.get('steps')
                if (step.keys() != {'op', 'steps'} or type(distance) is not int
                        or not 2 <= distance <= 80 or (distance % 2 and distance not in (5, 15))):
                    raise ValueError(f'{label} 的 steps 须为 2 至 80 的偶数，或 MCU 支持的 5、15 步')
                normalized[name].append({'op': op, 'steps': distance})
            elif name != _DROP and op in _SIDE_ACTIONS:
                count = step.get('repeat', 1)
                if (step.keys() - {'op', 'repeat', 'unverified_repeat', 'unverified_last_small', 'monitor_every',
                                    'left_bias_stride_m', 'nominal_last_small'}
                        or type(count) is not int or not 1 <= count <= 6):
                    raise ValueError(f'{label} 的 repeat 须为 1 至 6 的整数')
                motion = {'op': op, 'repeat': count}
                if 'monitor_every' in step:
                    group = step['monitor_every']
                    if type(group) is not int or not 1 <= group <= 6:
                        raise ValueError(f'{label} 的 monitor_every 须为 1 至 6 的整数')
                    motion['monitor_every'] = group
                if 'unverified_repeat' in step:
                    alternate = step['unverified_repeat']
                    if index != 1 or type(alternate) is not int or not 1 <= alternate <= 6:
                        raise ValueError(f'{label} 的 unverified_repeat 仅适用于本段首个动作，须为 1 至 6 的整数')
                    motion['unverified_repeat'] = alternate
                if 'unverified_last_small' in step:
                    flag = step['unverified_last_small']
                    if type(flag) is not bool or motion.get('unverified_repeat', 0) < 2:
                        raise ValueError(f'{label} 的 unverified_last_small 需搭配至少 2 步的 unverified_repeat')
                    motion['unverified_last_small'] = flag
                if 'nominal_last_small' in step:
                    flag = step['nominal_last_small']
                    if (type(flag) is not bool or count < 2 or 'unverified_repeat' in step
                            or name != 'tag3_to_tag4' or index != 1 or op != 'strafe_right_once'):
                        raise ValueError(f'{label} 的 nominal_last_small 仅用于 Tag 3 出发的右跨，repeat 至少为 2')
                    motion['nominal_last_small'] = flag
                if 'left_bias_stride_m' in step:
                    stride = step['left_bias_stride_m']
                    if (name != 'tag3_to_tag4' or index != 1 or op != 'strafe_right_once'
                            or type(stride) not in (int, float) or not .03 <= stride <= .15):
                        raise ValueError(f'{label} 的 left_bias_stride_m 须为 Tag 3 出发大右跨的试跑步幅，单位米')
                    motion['left_bias_stride_m'] = stride
                normalized[name].append(motion)
            else:
                raise ValueError(f'{label} 包含该路段不支持的操作')
    return normalized


def _forward_actions(steps, *, continuous, legacy):
    actions, planned_steps = EntranceTrial_PlanForward(steps, continuous=continuous, legacy=legacy)
    return actions if planned_steps == steps else ()


def CorridorTrial_GroupStrafes(op, count, *, continuous, right5_ready=False, monitor_every=None):
    """Group a strafe count into supported native actions at the requested check interval."""
    if not continuous:
        return (op,) * count
    if monitor_every and count > monitor_every:
        return tuple(action for offset in range(0, count, monitor_every)
                     for action in CorridorTrial_GroupStrafes(op, min(monitor_every, count - offset),
                                                continuous=True, right5_ready=right5_ready))
    group = SIDE_ACTIONS.get((op, count))
    if group and (group != 'cmd0_strafe_right5' or right5_ready):
        return (group,)
    # Keep exact stride counts while using the established multi-stride macros.
    if count == 5:
        return SIDE_ACTIONS[(op, 3)], SIDE_ACTIONS[(op, 2)]
    if count in (6, 7):
        return SIDE_ACTIONS[(op, count-3)], SIDE_ACTIONS[(op, 3)]
    return (op,) * count


def _side_macro_count(step, *, continuous):
    motion = {'continuous': continuous, 'monitor_every': step.get('monitor_every')}
    nominal = len(CorridorTrial_GroupStrafes(step['op'], step['repeat'], **motion))
    if step.get('nominal_last_small'):
        nominal = max(nominal, 1 + len(CorridorTrial_GroupStrafes(step['op'], step['repeat']-1, **motion)))
    if 'left_bias_stride_m' in step:
        nominal = max(nominal, *(len(CorridorTrial_GroupStrafes(step['op'], count, **motion))
                                  for count in range(1, _TAG3_TRIAL_MAX_RIGHT_STRIDES + 1)))
    alternative = step.get('unverified_repeat')
    if alternative is None:
        return nominal
    actions = len(CorridorTrial_GroupStrafes(step['op'], alternative, **motion))
    if step.get('unverified_last_small'):
        actions = max(actions, len(CorridorTrial_GroupStrafes(step['op'], alternative-1, **motion)) + 1)
    return max(nominal, actions)


def CorridorTrial_MirroredPickupStrafes(chain, *, small_ready):
    """Plan pickup exit left strides from the configured side portions of the first two legs."""
    moves = []
    for _, _, name in _LEGS[:2]:
        for step in chain[name]:
            if step['op'] not in _SIDE_ACTIONS:
                continue
            count = step.get('unverified_repeat', step['repeat'])
            last_small = small_ready and (step.get('unverified_last_small') is True
                                          or step.get('nominal_last_small') is True)
            moves.extend(CorridorTrial_GroupStrafes('strafe_left_once', count-int(last_small), continuous=True,
                                       monitor_every=min(3, step.get('monitor_every', 3))))
            if last_small:
                moves.append(SMALL_SIDE_ACTIONS['strafe_left_once'])
    return tuple(moves)


def _nominal_macro_count(steps, *, continuous, legacy):
    total = 0
    for step in steps:
        op = step['op']
        if op == _FORWARD_GROUP:
            total += len(_forward_actions(step['steps'], continuous=continuous, legacy=legacy))
        elif op in _SIDE_ACTIONS:
            # Budget for the grouped fallback even if native right5 is available.
            total += _side_macro_count(step, continuous=continuous)
        else:
            total += 1
    return total


def CorridorTrial_MaxBodyMacros(chain, *, continuous=True, legacy=False, tag5_passage=False, extra_home_steps=0):
    """Count planned actions and possible stride fusion, apart from the shared navigation budget."""
    chain = CorridorTrial_ValidateChain(chain, extra_home_steps=extra_home_steps)
    if not continuous:
        raise ValueError('Tag 5 放物需要已验证的连续路线 MCU 能力')
    total = 1  # The single-object release.
    total += max(max((count + 1) // 2 for count in TAG2_APPROACH_COUNTS),
                 max(len(_forward_actions(count, continuous=continuous, legacy=legacy))
                     for count in TAG2_APPROACH_COUNTS))
    total += 1  # Tag 2 can need one short forward or reverse group before a new scan.
    total += _TAG4_APPROACH_GROUPS  # Includes the optional Tag 4 trim.
    if not tag5_passage:
        total += _TAG5_ENTRY_TRIMS
    else:
        total += 1  # A partial Tag 5 approach can split its first forward group.
        total += 2  # A close-gate retreat and the compensating forward group.
    for name, steps in chain.items():
        if name not in ('tag2_to_tag3', _TAG5_LEG) and any(step['op'] in _SIDE_ACTIONS for step in steps):
            total += _LATERAL_CLEARANCE_MAX_GROUPS
        for index, step in enumerate(steps):
            op = step['op']
            if op == _FORWARD_GROUP:
                actions = _forward_actions(step['steps'], continuous=continuous, legacy=legacy)
                if not actions:
                    raise ValueError(f'{name} 缺少 {step["steps"]} 步的前进动作')
                total += len(actions)
            elif op in _SIDE_ACTIONS:
                count = step['repeat']
                cost = _side_macro_count(step, continuous=continuous)
                if index == 0:
                    # At most one small step or a one-cycle shift from the departure pose.
                    motion = {'continuous': continuous, 'monitor_every': step.get('monitor_every')}
                    cost = max(cost, len(CorridorTrial_GroupStrafes(op, count, **motion))+1,
                               len(CorridorTrial_GroupStrafes(op, count+1, **motion)),
                               len(CorridorTrial_GroupStrafes(op, count-1, **motion)))
                if name == 'tag2_to_tag3':
                    short_interval = min(2, step.get('monitor_every', 2))
                    short_cost = _side_macro_count(dict(step, monitor_every=short_interval), continuous=continuous)
                    # A cautious traverse can add boundaries. Each can request two clearance groups.
                    cost = 3 * max(cost, short_cost)
                total += cost
    return total


def _report_progress(robot, phase, previous_id, expected_id, *, leg=None, reason_zh=None):
    report = getattr(robot.backend, 'report_progress', None)
    if report is not None:
        try:
            command = getattr(robot, 'route_command', '0')
            progress = {'command': command, 'phase': phase, 'direction': 'outbound',
                        'outbound_only': getattr(robot,'outbound_only',command=='1'), 'shared_outbound': command in ('1','2'),
                        'journey': getattr(robot,'deliveries',0)+1,
                        'expected_tag': expected_id, 'last_passed_tag': previous_id,
                        'trial_target_tag': _CHECKPOINTS[-1], 'delivery_planned': Cmd0Motion_ContinuousReady(robot.backend),
                        'route_evidence': 'estimated_trial'}
            if reason_zh:
                progress['reason_zh'] = reason_zh
            if leg is not None:
                scan = leg['scan'] or {}
                progress.update(completed_actions=list(leg['completed_actions']),
                                last_passed_tag=leg.get('last_confirmed_tag', previous_id),
                                departure_tag_confirmed=leg.get('departure_tag_confirmed'),
                                seen_tags=list(scan.get('observed_ids', ())),
                                expected_fresh_frames=scan.get('samples', 0),
                                observation_windows=leg['observation_windows'])
                if leg.get('next_motion_axis') == 'stationary_scan':
                    progress['stationary_scan'] = True
                if leg.get('stationary_wait_reason_zh'):
                    progress['reason_zh']=leg['stationary_wait_reason_zh']
                if leg.get('checkpoint_not_seen'):
                    progress['checkpoint_not_seen']=leg['checkpoint_not_seen']
                if leg.get('previous_tag_pose_raw') is not None:
                    progress['previous_tag_pose_raw'] = leg['previous_tag_pose_raw']
                if leg.get('entry_completed_actions'):
                    progress.update(entry_completed_actions=list(leg['entry_completed_actions']),
                                    entry_target_steps=leg.get('entry_target_steps'),
                                    entry_planned_steps=leg.get('entry_planned_steps'))
            report(progress)
        except Exception:
            pass  # Status reporting is advisory.


def _checkpoint_raw_pose(scan, tag_id, min_frames):
    """Summarize the previous accepted scan for diagnostics during the next leg."""
    if not scan or not scan.get('camera_healthy') or not scan.get('contract_match') or scan.get('ids') != [tag_id]:
        return None
    try:
        rows = scan['observations']
        frames = {row['frame_id'] for row in rows}
        stamps = {row['stamp_ns'] for row in rows if row['id'] == tag_id}
        if len(frames) != 1 or len(stamps) < min_frames or len(stamps) != len(rows):
            return None
        coordinates = {axis: [row['position_raw_m'][axis] for row in rows] for axis in ('x', 'y', 'z')}
        if not all(isfinite(value) for axis in coordinates.values() for value in axis):
            return None
        return {'id': tag_id, 'frame_id': next(iter(frames)),
                'position_raw_m': {axis: median(values) for axis, values in coordinates.items()},
                'distinct_frames': len(stamps), 'diagnostic_only': True}
    except (KeyError, TypeError, ValueError):
        return None


def _measure(robot, nav, profile, checkpoint):
    observation = dict(checkpoint.get('scan') or {}, expected_load=robot.load_kind)
    return RoutePose_Measure(observation, nav.cfg, profile, checkpoint['expected_id'],
                             camera_info=checkpoint.get('camera_info', {}))


def _revalidate_heading(robot, nav):
    guard = getattr(robot, 'heading_guard', None)
    if guard is not None:
        reserve = (robot.cfg['request_timeout_s'] + nav.cfg['post_action_heading']['settle_s']
                   + nav.cfg['magnetic']['collect_s'])
        revalidate=getattr(robot,'revalidate_route_heading',None)
        report = revalidate(reserve) if revalidate else guard.revalidate_stationary_target(robot.load_kind,reserve)
        if report is not None:
            robot.events.append({'corridor_heading_reference': report})


def CorridorTrial_HeadingSnapshot(report, *, after, load='single'):
    """Return a verified, same-load reading with its capture scope."""
    steps = report.get('correction_steps')
    boundary = (steps[-1] if after else steps[0]) if steps else report
    post_turn = after and 'after_error_deg' in boundary
    field = 'after_error_deg' if post_turn else 'error_deg'
    error = boundary.get(field)
    reference = (boundary.get('verification' if post_turn else 'sample') or {}).get('reference') or {}
    if (type(error) not in (int, float) or not isfinite(error)
            or reference.get('capture_verified') is not True or not reference.get('usable_for_heading')
            or reference.get('load') != load):
        return None
    receipt = reference.get('received_monotonic')
    scope = tuple(reference.get(key) for key in ('capture_session', 'load_epoch', 'calibration_id'))
    if (type(receipt) not in (int, float) or not isfinite(receipt)
            or not scope[0] or type(scope[1]) is not int or not scope[2]):
        return None
    return error, receipt, scope


def _move_lateral_clearance(robot, operation, initial, deadband, *, allow_extra):
    if not robot.corridor_recovery_translate(operation, sampled_at=initial[1]):
        return [], initial[0]
    actions = [operation]
    last_error = initial[0]
    if allow_extra:
        robot.monitor_heading('corridor_side_boundary', allow_correction=False)
        measured = CorridorTrial_HeadingSnapshot(robot.last_heading_report or {}, after=False, load=robot.load_kind)
        if measured and measured[2] == initial[2] and measured[1] > initial[1]:
            last_error = measured[0]
            if (2 * deadband < abs(last_error) <= _LATERAL_CLEARANCE_HEADING_DEG and last_error * initial[0] > 0
                    and robot.corridor_recovery_translate(operation, sampled_at=measured[1])):
                actions.append(operation)
    return actions, last_error


def CorridorTrial_CheckLateralHeading(robot, nav, action, before, report, leg, pending_actions,
                                     expected_tag=None, *, allow_turn=True, allow_clearance_translation=True,
                                     small_strafe_follows=False):
    """At a completed lateral boundary, check drift and recover heading where allowed."""
    load = robot.load_kind
    initial = CorridorTrial_HeadingSnapshot(report, after=False, load=load)
    verified = (before is not None and initial is not None
                and before[2] == initial[2] and before[1] < initial[1])
    drift = wrap(initial[0] - before[0]) if verified else None
    check = {'action': action, 'error_before_deg': before[0] if before else None,
             'error_after_strafe_deg': initial[0] if initial else None, 'drift_deg': drift,
             'residual_deg': initial[0] if initial else None, 'correction_actions': 0}
    leg.setdefault('lateral_heading_checks', []).append(check)
    if not allow_turn:
        check['compensation_status'] = 'tag5_heading_monitor_only'
        return CorridorLateralHeadingResult.PROCEED
    heading_cfg = nav.cfg['post_action_heading']
    if not heading_cfg.get('execute'):
        check['compensation_status'] = 'correction_disabled'
        return CorridorLateralHeadingResult.PROCEED
    deadband = heading_cfg['deadband_deg']
    used = leg.get('compensation_groups', 0)
    tag2_to_tag3 = (leg.get('from_tag'), leg.get('expected_id')) == (2, 3)
    large_drift = verified and (
        min(abs(drift), abs(initial[0])) >= _LATERAL_LARGE_DRIFT_DEG
        or (tag2_to_tag3 and abs(initial[0]) >= _LATERAL_LARGE_DRIFT_DEG and abs(drift) > 2 * deadband))
    check['large_drift'] = large_drift
    if (small_strafe_follows and before is None and initial is not None
            and deadband < abs(initial[0]) <= 2 * deadband):
        check['compensation_status'] = 'heading_deferred_to_small_strafe'
        return CorridorLateralHeadingResult.DEFERRED_TO_SMALL_STRAFE
    if (tag2_to_tag3 and initial is not None and abs(initial[0]) > _LATERAL_CLEARANCE_HEADING_DEG
            and (drift is None or abs(drift) >= _LATERAL_LARGE_DRIFT_DEG)):
        check['large_drift'] = True
        check['compensation_status'] = 'abrupt_drift_needs_clearance_recheck'
        return CorridorLateralHeadingResult.RECHECK_CLEARANCE
    if not verified:
        status = 'drift_unverified'
    elif abs(drift) <= deadband or abs(initial[0]) <= deadband:
        status = 'within_deadband'
    elif drift * initial[0] <= 0:
        status = 'drift_does_not_match_offset'
    elif leg.get('expected_id') == _CHECKPOINTS[-1]:
        status = 'tag5_scan_before_fore_aft_motion'
    elif not allow_clearance_translation:
        status = 'heading_correction_only'
    elif load == 'double':
        status = 'pair_heading_correction_only'
    elif min(abs(drift), abs(initial[0])) <= 2 * deadband:
        # A modest heading error does not justify walking along the skewed heading.
        status = 'moderate_drift_turn_only'
    elif abs(initial[0]) > _LATERAL_CLEARANCE_HEADING_DEG:
        status = 'heading_correction_only'
    elif not tag2_to_tag3 and (used >= _LATERAL_CLEARANCE_MAX_GROUPS or (used and not large_drift)):
        status = 'leg_compensation_already_used'
    elif drift < 0 and leg.get('tag2_backward_clearance_allowed') is False:
        status = 'tag2_backward_clearance_unconfirmed'
    elif drift < 0 and not Cmd0Motion_ContinuousReady(robot.backend):
        status = 'capability_unavailable'
    elif robot.count + pending_actions >= robot.cfg['max_body_macros_per_journey']:
        status = 'budget_unavailable'
    else:
        # Positive is clockwise. Clear the wall along the current heading before turning.
        operation = 'carry_forward2' if drift > 0 else 'cmd0_carry_backward2'
        check.update(compensation_status='requested', compensation_action=operation,
                     compensation_order='before_heading_correction')
        available = robot.cfg['max_body_macros_per_journey'] - robot.count - pending_actions
        allow_extra = large_drift and available >= 2 and (tag2_to_tag3 or used + 2 <= _LATERAL_CLEARANCE_MAX_GROUPS)
        check['compensation_requested_steps'] = 4 if allow_extra else 2
        actions, latest_error = _move_lateral_clearance(robot, operation, initial, deadband, allow_extra=allow_extra)
        check.update(compensation_steps=2*len(actions), last_verified_error_deg=latest_error)
        if actions:
            leg['completed_actions'].extend(actions)
            leg['compensation_action'] = operation
            leg['compensation_groups'] = used + len(actions)
            status = 'applied'
        else:
            status = 'dispatch_unavailable'
    check['compensation_status'] = status
    if initial is not None and abs(initial[0]) <= deadband:
        return CorridorLateralHeadingResult.PROCEED
    robot.corridor_recover_heading(expected_tag)
    recovery = robot.last_heading_report or {}
    settled = CorridorTrial_HeadingSnapshot(recovery, after=True, load=load)
    check.update(residual_deg=settled[0] if settled else None,
                 correction_actions=recovery.get('correction_actions', 0),
                 heading_status=recovery.get('status'))
    return CorridorLateralHeadingResult.PROCEED


def _tag2_recheck_severe_lateral_drift(robot, nav, profile, leg, *, more_right):
    """Require current route geometry after a side group reports an abrupt heading change."""
    drift = leg['lateral_heading_checks'][-1]
    recovery = {'status':'rechecking','drift_deg':drift['drift_deg'],
                'magnetic_error_deg':drift['error_after_strafe_deg'],'observations':[]}
    leg.setdefault('tag2_severe_side_recoveries', []).append(recovery)
    # A possible wall contact invalidates the old stance and its dead reckoning.
    leg.pop('tag2_side_hint', None)
    leg.pop('tag2_projected_side_hint', None)
    leg['tag2_backward_clearance_allowed'] = False
    _report_progress(robot,'cmd0_departure_reobserve',2,3,leg=leg,
                     reason_zh='侧跨后航向突变，正在重读 Tag 2 和 Tag 3，确认墙边通行余量。')
    stance = robot.last_action_completed_ros_ns
    live = getattr(nav.io,'visual_motion_ready',None)
    rebase = getattr(robot,'Route_RebaseTagHeading',None)
    settings = robot.cfg.get('route_steps', {}).get('tag2_forward_trial') or {}
    near, far = settings.get('distance_window_m',(.08,.26))
    deadband = nav.cfg['post_action_heading']['deadband_deg']
    for tag_id in (3,2):
        robot.check()
        _, observation = _refresh_checkpoint_pose(robot,nav,profile,tag_id,quick_id=True)
        window = {'tag_id':tag_id, 'detected':observation['detected'],
                  'scan_windows':observation.get('quick_scan_windows',[])}
        recovery['observations'].append(window)
        floor = (observation.get('scan') or {}).get('after_ns')
        hint = RoutePose_TrialLateralHint(observation,nav.cfg,tag_id)
        if (hint is None or live is None or type(stance) is not int or type(floor) is not int or floor < stance
                or hint['latest_stamp_ns'] <= stance or robot.last_action_completed_ros_ns != stance
                or not live(hint['latest_stamp_ns'])):
            window['result'] = 'current_pose_unavailable'
            continue
        window['pose'] = hint
        left, right = hint['lateral_bounds_m']
        yaw = hint['heading_deg']
        # The reliable hint's depth spread can be up to 2.5 cm. Keep a further
        # 2 cm clear of the near edge while the last wall contact is uncertain.
        if (not near + .045 <= hint['distance_m'] <= far or max(abs(left),abs(right)) > _TAG2_DEPARTURE_LATERAL_M
                or abs(yaw) > deadband or (tag_id == 3 and more_right and right > _TAG3_RIGHT_OFFSET_M)):
            window['result'] = 'pose_outside_clearance'
            continue
        reference = rebase(tag_id,yaw,hint['latest_stamp_ns']).value if rebase is not None else 'unavailable'
        if reference != 'applied':
            window['result'] = 'magnetic_reference_unavailable'
            if getattr(robot, 'magnetic_route_only', False) and live(hint['latest_stamp_ns']):
                window['result'] = 'resumed_with_visual_pose'
                drift.update(residual_deg=yaw, heading_status='visual_clearance_confirmed')
                recovery.update(status='resumed', reference_tag_id=tag_id, source='visual')
                return hint
            continue
        robot.monitor_heading('corridor_departure',allow_correction=False)
        settled = CorridorTrial_HeadingSnapshot(robot.last_heading_report or {},after=True,load=robot.load_kind)
        if settled is None or abs(settled[0]) > deadband + 4.:
            window['result'] = 'magnetic_residual_unconfirmed'
            continue
        window['result'] = 'resumed_with_new_pose'
        drift.update(residual_deg=settled[0],heading_status='tag_reference_restored')
        recovery.update(status='resumed',reference_tag_id=tag_id,magnetic_residual_deg=settled[0])
        return hint
    recovery['status'] = 'clearance_unconfirmed'
    return None


def _departure_pose(robot, nav, profile, tag_id, pose, evidence):
    """Retry a calibrated departure finitely, retaining any verified hard obstruction."""
    if robot.load_kind == 'double' and getattr(robot, 'pair_route_enabled', False):
        evidence['departure_advice'] = {'status': 'nominal_unverified',
                                       'reason': 'metric departure calibration is for single load'}
        return pose, None
    advisory_heading = getattr(robot, '_allows_missing_magnetic', lambda: False)()
    policy = {'advisory_heading': True} if advisory_heading else {}
    advice = RoutePose_AdviseDeparture(pose, profile, tag_id, **policy)
    evidence['departure_advice'] = advice
    if advice['status'] in ('nominal_unverified', 'fuse_or_continue'):
        return pose, None

    if advice['status'] == 'outside_departure_envelope':
        evidence['departure_hard_obstruction'] = advice
    nudged = False
    _report_progress(robot, 'cmd0_departure_reobserve', tag_id, tag_id)
    for attempt in range(1, _DEPARTURE_REOBSERVATIONS + 1):
        robot.check()
        checkpoint = {'expected_id': tag_id, 'observation_windows': 0, 'detected': False,
                      'completed_actions': []}
        evidence['departure_reobservations'] = attempt
        hold = tag_id == _CHECKPOINTS[-1] or 'departure_hard_obstruction' in evidence
        observed, reason = EntranceTrial_ObserveCheckpoint(robot, nav, checkpoint,
            lambda: _report_progress(robot, 'cmd0_departure_reobserve', tag_id, tag_id, leg=checkpoint),
            miss_policy='hold_load_for_departure' if hold else 'continue_estimated')
        if observed is EntranceTrialResult.CAMERA_UNAVAILABLE:
            return None, (CorridorTrialResult.CAMERA_UNAVAILABLE, reason)
        if observed is EntranceTrialResult.CHECKPOINT_NOT_SEEN:
            evidence['departure_checkpoint_not_seen'] = checkpoint['checkpoint_not_seen']
            break
        pose = _measure(robot, nav, profile, checkpoint)
        advice = RoutePose_AdviseDeparture(pose, profile, tag_id, **policy)
        evidence['departure_advice'] = advice
        evidence['departure_observation'] = checkpoint['scan']
        if advice['status'] == 'fuse_or_continue':
            evidence.pop('departure_hard_obstruction', None)
            return pose, None
        if advice['status'] == 'outside_departure_envelope':
            evidence['departure_hard_obstruction'] = advice
        elif advice['status'] == 'fine_adjustment_needed':
            evidence.pop('departure_hard_obstruction', None)
        elif advice['status'] == 'nominal_unverified' and 'departure_hard_obstruction' not in evidence:
            return pose, None
        if advice.get('heading_advisory_only') and (nudged or not advice['action'] or attempt == _DEPARTURE_REOBSERVATIONS):
            evidence['departure_resolution'] = 'verified_position_continue_with_heading_residual'
            return pose, None
        if nudged or advice['status']!='fine_adjustment_needed':
            continue
        if attempt == _DEPARTURE_REOBSERVATIONS:
            # Leave one fresh observation available after any additional motion.
            continue
        still_fresh = getattr(nav.io, 'visual_motion_ready', None)
        if still_fresh is None or not still_fresh(pose.get('latest_stamp_ns', 0)):
            continue
        if advice['action']=='heading_micro_after_reobserve':
            nudged = True
            prior_heading = nav.mag.anchor
            correction = robot.visual_checkpoint(tag_id, departure_pose=pose)
            evidence['departure_correction'] = correction
            anchor = nav.mag.anchor or {}
            if (correction.get('magnetic', {}).get('status') == 'updated'
                    and anchor.get('source') == 'reliable_tag_alignment' and anchor.get('tag_id') == tag_id
                    and anchor.get('capture_verified') is True and anchor.get('usable_for_heading') is True):
                evidence['departure_heading_source'] = 'trusted_tag_then_relative_magnetic'
            if correction.get('correction_executed') or nav.mag.anchor is not prior_heading:
                nav.mag.anchor = None
                robot.prepare_heading_segment(fresh=True)
        elif advice.get('micro_operation'):
            ready = getattr(robot.backend, 'small_strafe_ready', None)
            if (ready is not None and ready() and not getattr(robot,'magnetic_route_only',False)
                    and robot.nav_count < nav.cfg['max_extra_actions_per_journey']):
                nudged = True
                robot.action(advice['micro_operation'])
                robot.nav_count+=1
                evidence['departure_correction'] = advice['micro_operation']
    if 'departure_hard_obstruction' in evidence:
        return None, (CorridorTrialResult.DEPARTURE_UNSAFE,
                      f'Tag {tag_id} 的起步位姿超出已标定通行范围，复查未确认恢复。')
    if tag_id == _CHECKPOINTS[-1]:
        return None, (CorridorTrialResult.DEPARTURE_UNSAFE,
                      f'Tag {tag_id} 的放物前起步位姿未确认，保留持物。')
    evidence['departure_resolution'] = 'continue_estimated_after_reobserve'
    return {}, None


def _corridor_boundary_checkpoint(robot, nav, tag_id, action=None):
    """Reuse or sample the expected ID with no wait at the current completion."""
    read = getattr(robot, 'boundary_visual_observation', None)
    if read is None:
        return None
    checkpoint = EntranceTrial_BoundaryCheckpoint(robot, nav, tag_id)
    if checkpoint is not None:
        return checkpoint
    report = read(action or 'corridor_checkpoint', expected_tag=tag_id)
    if report is not None:
        return EntranceTrial_BoundaryCheckpoint(robot, nav, tag_id, report=report)
    return None


def _refresh_checkpoint_pose(robot, nav, profile, tag_id, *, reuse_settled_cache=False, quick_id=False,
                             boundary_first=False, advisory_windows=None):
    if boundary_first:
        checkpoint = _corridor_boundary_checkpoint(robot, nav, tag_id)
        if checkpoint is not None:
            return _measure(robot, nav, profile, checkpoint), checkpoint
    observation = {'expected_id': tag_id, 'observation_windows': 0, 'detected': False}
    observed, _ = EntranceTrial_ObserveCheckpoint(robot, nav, observation, lambda: None,
                                                  single_window=advisory_windows is None, advisory_windows=advisory_windows,
                                                  reuse_settled_cache=reuse_settled_cache, quick_id=quick_id)
    pose = _measure(robot, nav, profile, observation) if observed is EntranceTrialResult.CHECKPOINT_DETECTED else {}
    return pose, observation


def _corridor_partial_geometry(robot, nav, checkpoint, tag_id, *, fresh_only=False):
    """Review a weak reading from the current stance without calling it an arrival."""
    scan = checkpoint.get('scan') or {}
    if (checkpoint.get('partial_pose_same_stance_ns') != robot.last_action_completed_ros_ns
            or not scan.get('camera_healthy') or not scan.get('contract_match')):
        return None
    rows = checkpoint.get('partial_pose_rows') or ()
    if fresh_only:
        expected = nav.cfg.get('visual_micro',{}).get('camera_info',{}).get('expected_fingerprint')
        camera = checkpoint.get('camera_info') or {}
        if not expected or camera.get('fingerprint') != expected or camera.get('ready') is not True:
            return None
        now = nav.io.ros_now_ns()
        rows = [row for row in rows if type(row.get('stamp_ns')) is int
                and row['stamp_ns'] > robot.last_action_completed_ros_ns
                and 0 <= now-row['stamp_ns'] <= nav.cfg['max_age_s']*1e9]
    geometry = RoutePose_PartialCheckpointGeometry(rows, nav.cfg, tag_id)
    if geometry is None:
        return None
    near, far = geometry['distance_bounds_m']
    left, right = geometry['lateral_bounds_m']
    geometry['consistent'] = (far-near <= .035 and right-left <= .035
                              and geometry['max_heading_deviation_deg'] <= 5.)
    ready = getattr(nav.io, 'visual_motion_ready', None)
    geometry['latest_fresh'] = bool(ready and ready(geometry['latest_stamp_ns']))
    return geometry


def _tag4_partial_advice(geometry, settings):
    if not geometry['consistent']:
        return 'conflicting_frames'
    if (geometry['max_abs_heading_deg'] > 15. or max(map(abs,geometry['lateral_bounds_m'])) > .16
            or geometry['projection_bounds'][0] <= .85):
        return 'outside_heading_or_lateral_range'
    near, far = geometry['distance_bounds_m']
    target, band = settings['target_distance_m'],settings['deadband_m']
    if .02 <= near <= far <= target + band:
        return 'near'
    travel = max(.10, settings['two_step_distance_m'][1])
    reserve = 0.04 if geometry['samples'] == 1 else 0.02
    if (target + band + travel + reserve <= near <= far <= settings['distance_window_m'][1]
            and geometry['latest_fresh']):
        return 'far'
    return 'depth_uncertain'


def _tag4_centered_forward_projection(pose, settings, steps):
    """Bound a straight group using a centered visual pose, including heading spread."""
    near, far = pose['distance_bounds_m']
    left, right = pose['lateral_bounds_m']
    yaw, spread = pose['heading_deg'], pose['max_heading_deviation_deg']
    projection_min, projection_max = pose['projection_bounds']
    max_yaw = pose.get('max_abs_yaw_deg', pose.get('max_abs_heading_deg', abs(yaw) + spread))
    if (max_yaw > 14. or spread > 3. or far-near > .025 or right-left > .025
            or max(abs(left), abs(right)) > .08 or projection_min < .85):
        return None
    travel = steps / 2 * max(.125, settings['two_step_distance_m'][1])
    shifts = [0., *(travel * sin(radians(angle)) for angle in (yaw-spread, yaw+spread))]
    swept = (left + min(shifts), right + max(shifts))
    floor = max(settings['distance_window_m'][0],
                settings['target_distance_m'] - settings['deadband_m'] + .01)
    landing = (near - travel * projection_max, far)
    if min(landing) >= floor and far <= settings['distance_window_m'][1] and max(map(abs, swept)) <= .12:
        return {'steps':steps, 'projected_depth_bounds_m':landing, 'projected_lateral_bounds_m':swept}
    return None


def _tag4_partial_forward_clearance(robot, nav, observation, settings):
    """Two fresh, coherent frames can bound one short step group, never a turn or a left departure."""
    pose = _corridor_partial_geometry(robot,nav,observation,4,fresh_only=True)
    if (pose is None or pose['samples'] < 2 or not pose['consistent'] or not pose['latest_fresh']
            or abs(pose['heading_deg']) > 25. or pose['max_abs_heading_deg'] > 27.
            or pose['max_heading_deviation_deg'] > 3. or pose['projection_bounds'][0] < .85):
        return {}
    near, far = pose['distance_bounds_m']
    if abs(pose['heading_deg']) <= 15.:
        projection = _tag4_centered_forward_projection(pose, settings, 2)
        if projection is None or projection['projected_depth_bounds_m'][0] < (
                settings['target_distance_m'] + settings['deadband_m'] + .02):
            return {}
        return {'guarded_forward_steps':2, 'guarded_partial_frames':pose['samples'],
                'partial_forward_projection':projection, 'heading_deg':pose['heading_deg'],
                'distance_m':(near+far)/2, 'latest_stamp_ns':pose['latest_stamp_ns']}
    travel = max(.10,settings['two_step_distance_m'][1])
    target = settings['target_distance_m']+settings['deadband_m']
    left, right = pose['lateral_bounds_m']
    if not target+travel+.02 <= near <= far <= settings['distance_window_m'][1] or right-left > .025:
        return {}
    spread = pose['max_heading_deviation_deg']
    yaw = pose['heading_deg']
    shifts = (0.,travel*sin(radians(yaw-spread)),travel*sin(radians(yaw+spread)))
    if left+min(shifts) < -.15 or right+max(shifts) > .15:
        return {}
    return {'guarded_forward_steps':2,'guarded_partial_frames':pose['samples'],'heading_deg':yaw,
            'distance_m':(near+far)/2,'latest_stamp_ns':pose['latest_stamp_ns']}


def _tag4_position_plan(robot, nav, profile, settings, *, approach_steps=None, boundary_first=False,
                        reuse_settled_cache=True, advisory_windows=None):
    _, observation = _refresh_checkpoint_pose(robot, nav, profile, 4, reuse_settled_cache=reuse_settled_cache,
                                              quick_id=True, boundary_first=boundary_first,
                                              advisory_windows=advisory_windows)
    return _tag4_longitudinal_observation(robot, nav, observation, settings, approach_steps=approach_steps)


def _tag4_longitudinal_observation(robot, nav, observation, settings, *, approach_steps=None):
    plan = RoutePose_PlanTag4Longitudinal(observation, nav.cfg, settings, approach_steps=approach_steps)
    if observation.get('reused_boundary_frames'):
        plan['reused_boundary_frames'] = True
    ready = getattr(nav.io, 'visual_motion_ready', None)
    if plan['distance_verified'] and (ready is None or not ready(plan['latest_stamp_ns'])):
        plan.update(distance_verified=False, reason='Tag 4 depth expired before dispatch')
    depth = plan.get('distance_bounds_m')
    if plan.get('depth_confirmed') and depth is not None:
        plan['confirmed_near'] = depth[1] < settings['target_distance_m'] - settings['deadband_m']
    deadband = nav.cfg.get('visual_micro', {}).get('deadband_deg', 8.)
    if (plan['distance_verified'] and approach_steps is not None and plan['steps'] in (2, 4)
            and abs(plan['heading_deg']) > deadband):
        projection = _tag4_centered_forward_projection(plan, settings, plan['steps'])
        if projection is not None:
            plan['heading_fused_into_forward'] = projection
    if not plan['distance_verified']:
        partial = _corridor_partial_geometry(robot, nav, observation, 4, fresh_only=True)
        if partial is not None:
            plan.update(partial_pose=partial, partial_advice=_tag4_partial_advice(partial,settings))
            if approach_steps is not None and 'distance_bounds_m' not in plan:
                plan.update(_tag4_partial_forward_clearance(robot,nav,observation,settings))
        camera = observation.get('camera_window_status') or {}
        if camera.get('parameters_match') is False or camera.get('ar_source_matches') is False:
            plan['camera_contract_fault'] = True
    return plan


def _tag4_forward_disruption(robot, last_event, previous_report, leg):
    """Keep the last severe observed departure even when fallback clears the live report."""
    reports = []
    for event in reversed(robot.events):
        if event is last_event:
            break
        if 'post_action_heading' in event:
            reports.append(event['post_action_heading'])
    current = getattr(robot, 'last_heading_report', None)
    if current is not None and current is not previous_report and not reports:
        reports.append(current)
    for report in reports:
        start = CorridorTrial_HeadingSnapshot(report, after=False, load=robot.load_kind)
        end = CorridorTrial_HeadingSnapshot(report, after=True, load=robot.load_kind)
        if end is not None and abs(end[0]) <= _LATERAL_LARGE_DRIFT_DEG:
            break
        raw = report.get('error_deg')
        field_unusable = report.get('status') == 'magnetic_field_recheck_unusable'
        error = end[0] if end else start[0] if start else raw if field_unusable else None
        if type(error) in (int, float) and abs(error) >= _LATERAL_CLEARANCE_HEADING_DEG:
            recovery = {'heading_error_deg': error, 'heading_status': report.get('status'),
                        'reason_zh': f'Tag 4 进近出现约 {abs(error):.0f}° 偏航，先重扫确认通道。'}
            leg['tag4_forward_disruption'] = recovery
            _report_progress(robot, 'cmd0_corridor_scan', 3, 4, leg=leg, reason_zh=recovery['reason_zh'])
            return recovery
    return None


def _tag4_observed_clearance_problem(plan, nav, *, heading_limit=None):
    if ('distance_bounds_m' not in plan or 'lateral_bounds_m' not in plan
            or plan.get('max_heading_deviation_deg', float('inf')) > 3.):
        return None
    near, far = plan['distance_bounds_m']
    if far - near > .025:
        return None
    left, right = plan['lateral_bounds_m']
    if right-left > .025:
        return None
    heading = plan['heading_deg']
    evidence = {'heading_error_deg': heading, 'evidence_distance_m': plan.get('distance_m'),
                'lateral_bounds_m': (left, right)}
    if left < -_TAG4_STRAIGHT_LATERAL_M or right > _TAG4_STRAIGHT_LATERAL_M:
        return dict(evidence, reason_zh='Tag 4 实测横偏超出直行范围，先恢复通道位姿。')
    if heading_limit is None:
        heading_limit = nav.cfg.get('visual_micro', {}).get('deadband_deg', 8.)
    if abs(heading) > heading_limit and not plan.get('heading_fused_into_forward'):
        return dict(evidence, reason_zh='Tag 4 已读到一致位姿，先按实测航向纠偏，再核对剩余直行。')
    return None


def _tag4_guarded_forward_ready(nav, plan):
    live = getattr(nav.io,'visual_motion_ready',None)
    return (plan.get('guarded_forward_steps') == 2 and not plan.get('camera_contract_fault')
            and abs(plan['heading_deg']) < _TAG4_CLEARANCE_YAW_DEG
            and live is not None and live(plan['latest_stamp_ns']))


def _tag4_recovery_strafe(plan, recovery, settings):
    """Choose a single turn-clearance stride or a measured move back toward the route."""
    near, far = plan['distance_bounds_m']
    left, right = plan['lateral_bounds_m']
    yaw = plan['heading_deg']
    target, band = settings.get('target_distance_m', .175), settings.get('deadband_m', .035)
    min_depth, max_depth = settings.get('distance_window_m', (.045, .55))
    if not min_depth <= near <= far <= max_depth or plan.get('projection_bounds', (0.,))[0] < .75:
        return None
    centers = recovery.get('centering_strafes', ())
    inward = (1 if right < 0 and left < -_TAG4_STRAIGHT_LATERAL_M else
              -1 if left > 0 and right > _TAG4_STRAIGHT_LATERAL_M else 0)
    outside = max(abs(left), abs(right))
    travel = _TAG4_RECOVERY_STRAFE_UPPER_M
    if (_TAG4_CLEARANCE_YAW_DEG <= abs(yaw) <= 35.
            and 'clearance_strafe' not in recovery and not centers and outside <= .22):
        turn_side = -1 if yaw < 0 else 1
        # Keep the whole step closer to the corridor center when the pose proves a side.
        preferred = (1 if right <= -travel / 2 else -1 if left >= travel / 2 else turn_side)
        directions, purpose = (preferred, -preferred), 'clearance_strafe'
    elif (inward and abs(yaw) < _TAG4_CLEARANCE_YAW_DEG
            and len(centers) < _TAG4_RECOVERY_CENTER_STRAFES and outside <= .30
            and (not centers or abs(median((left, right))) < abs(centers[-1]['before_lateral_m']) - .01)):
        directions, purpose = (inward,), 'centering_strafes'
    else:
        return None

    spread = plan['max_heading_deviation_deg']
    lateral_limit = max(outside, _TAG4_STRAIGHT_LATERAL_M)
    for direction in directions:
        shift = direction * travel
        if outside >= travel / 2 and median((left, right)) * direction > 0:
            continue
        if left + min(0., shift) < -lateral_limit or right + max(0., shift) > lateral_limit:
            continue
        depth_shifts = [shift * sin(radians(angle)) for angle in (yaw - spread, yaw + spread)]
        if near + min(0., *depth_shifts) >= max(min_depth, target - band):
            operation = 'strafe_left_once' if direction < 0 else 'strafe_right_once'
            return operation, purpose
    return None


def _tag4_can_reverse_approach(plan, settings, last_action, *, minimum_departure_m=None):
    if last_action not in ('carry_forward', 'carry_forward2', 'carry_forward4'):
        return False
    near, far = plan['distance_bounds_m']
    minimum, maximum = settings.get('distance_window_m', (.045, .55))
    lower_target = settings.get('target_distance_m', .175) - settings.get('deadband_m', .035)
    if minimum_departure_m is not None:
        lower_target = max(lower_target, minimum_departure_m)
    if not max(minimum, .06) <= near <= far < lower_target:
        return False
    yaw, spread = plan['heading_deg'], plan['max_heading_deviation_deg']
    projection_min, projection_max = plan.get('projection_bounds', (0., 0.))
    if abs(yaw) + spread > _TAG4_CLEARANCE_YAW_DEG or projection_min < .75:
        return False
    travel = max(.10, settings.get('two_step_distance_m', (.04, .075))[1])
    shifts = [0., *(-travel * sin(radians(angle)) for angle in (yaw-spread, yaw+spread))]
    left, right = plan['lateral_bounds_m']
    # Reverse only inside the known straight corridor, with the entire sweep accounted for.
    return (far + travel * projection_max <= maximum
            and left + min(shifts) >= -_TAG4_STRAIGHT_LATERAL_M
            and right + max(shifts) <= _TAG4_STRAIGHT_LATERAL_M)


def _tag4_recover_forward_pose(robot, nav, profile, settings, plan, recovery, *, leg, approach_steps, reserved_actions,
                               prior_action=None):
    """Reframe, turn, and verify a fresh corridor pose before any further straight motion."""
    turn_from_tag = getattr(robot, 'Route_TurnFromTagHeading', None)
    strafe = getattr(robot, 'corridor_side_action', None)
    rebase = getattr(robot, 'Route_RebaseTagHeading', None)
    live = getattr(nav.io, 'visual_motion_ready', lambda _: False)
    attempts = recovery.setdefault('reobservations', [])
    turns = recovery.setdefault('visual_turns', [])
    goal = next((point for point in nav.cfg['legs']['outbound'] if point['id'] == 4), {})
    stationary = 0
    motion_evidence_stamp = 0
    collected_after_motion = False
    while True:
        robot.check()
        partial = plan.get('partial_pose') or {}
        if (motion_evidence_stamp and approach_steps is not None and not collected_after_motion
                and not plan['distance_verified'] and not plan.get('camera_contract_fault')
                and partial.get('consistent') and partial.get('latest_fresh')):
            collected_after_motion = True
            collected = _tag4_position_plan(robot, nav, profile, settings, approach_steps=approach_steps,
                                             reuse_settled_cache=False, advisory_windows=2)
            attempts.append(collected)
            if (collected['distance_verified'] or collected.get('camera_contract_fault')
                    or collected.get('partial_pose') or not _tag4_guarded_forward_ready(nav, plan)):
                plan = collected
        stamp = plan.get('latest_stamp_ns', 0)
        fresh = type(stamp) is int and stamp > motion_evidence_stamp and live(stamp)
        if motion_evidence_stamp and plan['distance_verified'] and not fresh:
            plan = dict(plan, distance_verified=False, reason='Tag 4 pose did not match the current stance')
        if fresh and turns and 'after_error_deg' not in turns[-1] and plan.get('heading_deg') is not None:
            turns[-1]['after_error_deg'] = plan['heading_deg']
        issue = _tag4_observed_clearance_problem(plan, nav)
        if plan.get('camera_contract_fault') or (plan['distance_verified'] and issue is None):
            break
        if (motion_evidence_stamp and fresh and plan.get('guarded_partial_frames')
                and _tag4_guarded_forward_ready(nav, plan)):
            recovery['post_turn_partial_forward'] = plan.get('partial_pose')
            break
        moved = False
        if issue is not None and fresh:
            error = issue['heading_error_deg']
            bounds = issue['lateral_bounds_m']
            side = (_tag4_recovery_strafe(plan, recovery, settings)
                    if strafe is not None and goal.get('allow_strafe') is True else None)
            remaining = robot.cfg['max_body_macros_per_journey'] - robot.count - reserved_actions
            can_reverse = ('approach_retreat' not in recovery and remaining >= 2
                           and _tag4_can_reverse_approach(plan, settings,
                               (leg['completed_actions'] or [prior_action])[-1],
                               minimum_departure_m=(_TAG4_LEFT_CLEARANCE_FLOOR_M if
                                   approach_steps is None and settings.get('left_departure_reference') else None)))
            has_side_clearance = 'clearance_strafe' in recovery or recovery.get('centering_strafes')
            direct_turn_room = (side is None and abs(error) <= 30.
                                and plan['distance_bounds_m'][0] >= settings.get('target_distance_m', .175)
                                and max(map(abs, bounds)) <= .12
                                and plan.get('projection_bounds', (0.,))[0] >= .75)
            if side is not None and remaining >= (2 if side[1] == 'clearance_strafe' else 1):
                operation, purpose = side
                label = '左跨' if operation == 'strafe_left_once' else '右跨'
                reason = (f'Tag 4 航向 {error:+.1f}°，先{label}一步让位，再按新位姿纠偏。'
                          if purpose == 'clearance_strafe' else
                          f'Tag 4 横偏 {median(bounds)*100:+.1f} cm，先{label}一步回通道，再按新位姿纠偏。')
                _report_progress(robot, 'cmd0_corridor_scan', 3, 4, leg=leg, reason_zh=reason)
                if live(stamp):
                    strafe(operation)
                    leg['completed_actions'].append(operation)
                    record = {'action': operation, 'before_error_deg': error,
                              'before_lateral_m': median(bounds), 'reason_zh': reason}
                    if purpose == 'clearance_strafe':
                        recovery[purpose] = record
                    else:
                        recovery.setdefault(purpose, []).append(record)
                    moved = True
            elif can_reverse and live(stamp):
                operation = 'cmd0_carry_backward2'
                reason = 'Tag 4 距码过近，沿刚走过的直行后退两步，再按新位姿纠偏。'
                _report_progress(robot, 'cmd0_corridor_scan', 3, 4, leg=leg, reason_zh=reason)
                robot.action(operation, monitor_only_as=operation)
                leg['completed_actions'].append(operation)
                recovery['approach_retreat'] = {'action': operation, 'reason_zh': reason,
                                                'before_distance_m': plan['distance_m']}
                moved = True
            elif (turn_from_tag is not None and abs(error) <= 35.
                    and (abs(error) < _TAG4_CLEARANCE_YAW_DEG or has_side_clearance or direct_turn_room)
                    and (max(map(abs, bounds)) <= _TAG4_STRAIGHT_LATERAL_M or 'clearance_strafe' in recovery)
                    and (not turns or abs(error) <= abs(turns[-1]['before_error_deg']) - 1.)):
                turn = turn_from_tag(4, error, stamp, reserved_actions=reserved_actions)
                if turn is not None:
                    turns.append(turn)
                    recovery.setdefault('visual_turn', turn)
                    if direct_turn_room and abs(error) >= _TAG4_CLEARANCE_YAW_DEG and not has_side_clearance:
                        turn['reason_zh'] = 'Tag 4 直接小转有余量，未确认可用的侧跨。'
                    moved = True
        if (not moved and issue is not None and plan['distance_verified'] and fresh
                and _tag4_observed_clearance_problem(plan, nav, heading_limit=15.) is None):
            recovery['heading_deferred_deg'] = plan['heading_deg']
            break
        if not moved and stationary >= 2:
            break
        if moved:
            motion_evidence_stamp = stamp
            collected_after_motion = False
        plan = _tag4_position_plan(robot, nav, profile, settings, approach_steps=approach_steps,
                                   boundary_first=moved, reuse_settled_cache=moved)
        attempts.append(plan)
        stationary = 0 if moved else stationary + 1
    if (plan['distance_verified'] and 'heading_deferred_deg' not in recovery
            and _tag4_observed_clearance_problem(plan, nav) is not None):
        plan = dict(plan, distance_verified=False)
    recovery['reacquired'] = plan['distance_verified']
    if not recovery['reacquired']:
        remaining_issue = _tag4_observed_clearance_problem(plan, nav)
        if remaining_issue is None:
            recovery['blocked_reason_zh'] = 'Tag 4 纠偏后未取得可信的新位姿，未确认下一步的余量。'
        else:
            offset_cm = 100 * median(remaining_issue['lateral_bounds_m'])
            yaw = remaining_issue['heading_error_deg']
            recovery['blocked_reason_zh'] = (
                f'Tag 4 横偏 {offset_cm:+.1f} cm、航向 {yaw:+.1f}°，侧跨、后退或小转均未确认有余量。')
    if (recovery['reacquired'] and rebase is not None
            and plan.get('max_heading_deviation_deg', float('inf')) <= 3.
            and abs(plan['heading_deg']) <= 15.):
        recovery['heading_reference'] = rebase(4, plan['heading_deg'], plan['latest_stamp_ns']).value
    return plan


def _tag4_recheck_after_short_advance(robot, nav, profile, settings, leg, previous_heading, *,
                                     side_heading_lost=False, reserved_actions=0, partial_far=False,
                                     guarded_forward=False, magnetic_fallback=False):
    """Advance two steps, then check before committing the last two."""
    before = CorridorTrial_HeadingSnapshot(previous_heading, after=True, load=robot.load_kind)
    reason = ('Tag 4 位姿暂不可用，沿路线先走两步，再检查航向和扫码。' if magnetic_fallback else
              'Tag 4 航向仍有偏差，经位姿投影确认后先走计划中的两步。' if guarded_forward else
              'Tag 4 帧数不足但仍偏远，先走计划中的两步再重扫。' if partial_far else
              '侧跨后发生大偏航且航向暂不可用，Tag 4 进近先走两步再复查。' if side_heading_lost
              else '前一段直行有明显偏航，Tag 4 暂未确认，后四步拆成两步一查。')
    split = {'groups': [2,2], 'reason_zh': reason}
    leg['tag4_approach_split'] = split
    _report_progress(robot, 'cmd0_corridor_leg', 3, 4, leg=leg, reason_zh=split['reason_zh'])
    _revalidate_heading(robot, nav)
    previous_report = getattr(robot, 'last_heading_report', None)
    last_event = robot.events[-1] if robot.events else None
    robot.action('carry_forward2')
    leg['completed_actions'].append('carry_forward2')
    leg['completed_forward_steps'] = leg.get('completed_forward_steps',0) + 2
    disrupted = _tag4_forward_disruption(robot, last_event, previous_report, leg)

    report = getattr(robot, 'last_heading_report', None)
    report = report if report is not None and report is not previous_report else {}
    initial = CorridorTrial_HeadingSnapshot(report, after=False, load=robot.load_kind)
    settled = CorridorTrial_HeadingSnapshot(report, after=True, load=robot.load_kind)
    changed = wrap(initial[0] - before[0]) if (
        before and initial and before[2] == initial[2] and before[1] < initial[1]) else None
    deadband = nav.cfg['post_action_heading']['deadband_deg']
    recheck = _tag4_position_plan(robot, nav, profile, settings, approach_steps=2,
                                boundary_first=guarded_forward or magnetic_fallback)
    leg['tag4_approach'].append(recheck)
    if not disrupted:
        disrupted = _tag4_observed_clearance_problem(recheck, nav)
        if disrupted:
            leg['tag4_forward_disruption'] = disrupted
    split.update(heading_change_deg=changed, residual_deg=settled[0] if settled else None,
                 tag4_distance_verified=recheck['distance_verified'])
    if disrupted:
        recheck = _tag4_recover_forward_pose(robot, nav, profile, settings, recheck, disrupted,
                                          leg=leg, approach_steps=2, reserved_actions=reserved_actions)
        split['tag4_distance_verified'] = recheck['distance_verified']
    if recheck['distance_verified']:
        split.update(remaining_steps=recheck['steps'], recheck_reason_zh='已按 Tag 4 新位姿核对最后两步。')
    elif guarded_forward and not disrupted and _tag4_guarded_forward_ready(nav,recheck):
        split.update(remaining_steps=2, recheck_reason_zh='新位姿确认还有通行余量，再完成最后两步。')
    elif guarded_forward or partial_far or recheck.get('partial_advice') == 'near' or recheck.get('camera_contract_fault'):
        split.update(remaining_steps=0, partial_near=recheck.get('partial_advice')=='near',
                     recheck_reason_zh='Tag 4 本组先完成两步，剩余距离交给下一次扫码判断。')
    elif (disrupted or (changed is not None and abs(changed) > deadband)
          or (before and settled and before[2] == settled[2] and before[1] < settled[1]
              and abs(settled[0]) > deadband)):
        split.update(remaining_steps=0, recheck_reason_zh='短前进检测到明显偏航，省略最后两步，继续重扫 Tag 4。')
        _report_progress(robot, 'cmd0_corridor_scan', 3, 4, leg=leg, reason_zh=split['recheck_reason_zh'])
    else:
        split.update(remaining_steps=2, recheck_reason_zh='Tag 4 暂不可见，按估算完成剩余两步。')
    return split['remaining_steps']


def _tag4_departure_settings(settings):
    reference = settings.get('left_departure_reference')
    if reference is None:
        return settings
    return dict(settings, target_distance_m=reference['distance_m'], deadband_m=reference['distance_deadband_m'],
                approach_far_limit_m=settings['target_distance_m'] + settings['deadband_m'])


def _tag4_taught_close(plan, settings):
    bounds = plan.get('distance_bounds_m')
    return bool(settings.get('left_departure_reference') and bounds and bounds[0] < _TAG4_LEFT_PROBE_DEPTH_M)


def _tag4_left_start_ready(plan, settings):
    return (plan.get('depth_confirmed') and plan['distance_verified'] and not plan['steps']
            and not plan.get('confirmed_near') and not plan.get('confirmed_far')
            and not _tag4_taught_close(plan, settings))


def _tag4_authorize_near_left(robot, nav, settings, plan, leg, *, reserved_actions):
    """Approve one monitored left stride outside the nominal fore/aft band."""
    if (not plan.get('depth_confirmed') or not plan['distance_verified']
            or plan['steps'] != 0 or plan.get('camera_contract_fault')):
        return False
    if robot.count + reserved_actions + 1 > robot.cfg['max_body_macros_per_journey']:
        return False

    depth = plan.get('distance_bounds_m')
    adjusted_far = (plan.get('confirmed_far') and depth
                    and depth[1] <= settings.get('approach_far_limit_m', 0.))
    if not (plan.get('confirmed_near') or _tag4_taught_close(plan, settings) or adjusted_far):
        return False

    lateral = plan.get('lateral_bounds_m')
    heading = plan.get('heading_deg')
    spread = plan.get('max_heading_deviation_deg', float('inf'))
    stamp = plan.get('latest_stamp_ns')
    live = getattr(nav.io, 'visual_motion_ready', None)
    deadband = nav.cfg.get('visual_micro', {}).get('deadband_deg', 8.)
    if (depth is None or lateral is None or heading is None or spread > 3.
            or plan.get('max_abs_yaw_deg', abs(heading) + spread) > deadband or depth[1]-depth[0] > .025
            or lateral[1]-lateral[0] > .025 or plan.get('projection_bounds', (0.,))[0] < .90
            or type(stamp) is not int or live is None or not live(stamp)):
        return False

    travel = _TAG4_RECOVERY_STRAFE_UPPER_M
    min_depth, max_depth = settings.get('distance_window_m', (.045, .55))
    floor = (_TAG4_LEFT_CLEARANCE_FLOOR_M if settings.get('left_departure_reference') else
             settings.get('target_distance_m', .175) - settings.get('deadband_m', .035) - _TAG4_NEAR_LEFT_MARGIN_M)
    floor = max(min_depth, floor)
    shifts = [0., *(-travel * sin(radians(yaw)) for yaw in (heading - spread, heading + spread))]
    depth_after = depth[0] + min(shifts), depth[1] + max(shifts)
    swept_lateral = lateral[0] - travel, lateral[1]
    if (not floor <= depth_after[0] <= depth_after[1] <= max_depth
            or max(map(abs, swept_lateral)) > _TAG4_STRAIGHT_LATERAL_M):
        return False

    leg['tag4_near_left_start'] = {
        'latest_stamp_ns': stamp, 'before_distance_m': plan['distance_m'],
        'depth_bounds_m': depth, 'projected_depth_bounds_m': depth_after,
        'heading_deg': heading, 'reason_zh': 'Tag 4 当前位姿预计允许左跨，先走计划内的一步再检查。'}
    plan['approach_confirmed'] = True
    _report_progress(robot, 'cmd0_departure_reobserve', 4, 5, leg=leg,
                     reason_zh=leg['tag4_near_left_start']['reason_zh'])
    return True


def _tag4_retreat_before_left(robot, nav, profile, pose, leg, settings, plan, *,
                              reserved_actions, monitor_only):
    if robot.count + reserved_actions + 1 > robot.cfg['max_body_macros_per_journey']:
        plan['retreat_reason_zh'] = 'Tag 4 距离偏近，但动作余量不足以先后退再左跨。'
        return pose, False
    action = 'cmd0_carry_backward2'
    robot.action(action, **({'monitor_only_as': action} if monitor_only else {}))
    leg['completed_actions'].append(action)
    plan['completed_action'] = action
    rechecks = plan['retreat_rechecks'] = []
    for attempt in range(2):
        robot.check()
        measured, observation = _refresh_checkpoint_pose(robot, nav, profile, 4,
            reuse_settled_cache=attempt == 0, quick_id=True)
        after = _tag4_longitudinal_observation(robot, nav, observation, settings)
        issue = _tag4_observed_clearance_problem(after, nav)
        if issue is not None:
            plan['retreat_heading_recovery'] = issue
            after = _tag4_recover_forward_pose(robot, nav, profile, settings, after, issue,
                                              leg=leg, approach_steps=None, reserved_actions=reserved_actions)
            measured = {}
        rechecks.append(after)
        plan['after_distance_m'] = after.get('distance_m')
        if (_tag4_left_start_ready(after, settings)
                or _tag4_authorize_near_left(robot, nav, settings, after, leg, reserved_actions=reserved_actions)):
            plan['approach_confirmed'] = True
            return measured, True
        if after.get('camera_contract_fault') or after['distance_verified']:
            break
    plan['retreat_reason_zh'] = 'Tag 4 后退两步，仍未确认左跨起点的前后距离。'
    return {}, False


def _trim_tag4_before_left(robot, nav, profile, pose, leg, settings, *, checkpoint,
                          reserved_actions, monitor_only=False):
    plan = _tag4_position_plan(robot, nav, profile, settings)
    partial = plan.get('partial_pose') or {}
    if (not plan['distance_verified'] and not plan.get('camera_contract_fault')
            and partial.get('consistent') and partial.get('latest_fresh') and partial['samples'] < nav.cfg['min_frames']):
        pose, observation = _refresh_checkpoint_pose(robot, nav, profile, 4, quick_id=True, advisory_windows=2)
        collected = _tag4_longitudinal_observation(robot, nav, observation, settings)
        leg['tag4_departure_collection'] = collected
        if collected['distance_verified'] or collected.get('camera_contract_fault') or 'distance_bounds_m' in collected:
            plan = collected
    leg['tag4_forward_trim'] = plan
    advisory = plan.get('partial_advice')
    if plan.get('camera_contract_fault'):
        return pose, False
    issue = _tag4_observed_clearance_problem(plan, nav)
    if issue is not None:
        leg['tag4_forward_disruption'] = issue
        plan = _tag4_recover_forward_pose(robot, nav, profile, settings, plan, issue,
            leg=leg, approach_steps=None, reserved_actions=reserved_actions,
            prior_action=(checkpoint.get('completed_actions') or [None])[-1])
        leg['tag4_forward_trim'] = plan
        pose = {}
        if not plan['distance_verified']:
            plan['retreat_reason_zh'] = issue['blocked_reason_zh']
            return pose, False
        advisory = plan.get('partial_advice')
    if _tag4_authorize_near_left(robot, nav, settings, plan, leg, reserved_actions=reserved_actions):
        return pose, True
    if advisory == 'near':
        partial = plan['partial_pose']
        lower = settings['target_distance_m'] - settings['deadband_m']
        _, far = partial['distance_bounds_m']
        if far < lower:
            # Reverse only the end of a straight we just traversed.
            last_move = (checkpoint.get('completed_actions') or [None])[-1]
            current_heading = CorridorTrial_HeadingSnapshot(getattr(robot,'last_heading_report',None) or {},
                                                            after=True, load=robot.load_kind)
            if (partial['samples'] >= 2 and partial['latest_fresh'] and last_move in ('carry_forward2','carry_forward4')
                    and current_heading is not None and abs(current_heading[0]) <= nav.cfg['post_action_heading']['deadband_deg']
                    and robot.count + reserved_actions + 1 <= robot.cfg['max_body_macros_per_journey']):
                robot.action('cmd0_carry_backward2', **({'monitor_only_as':'cmd0_carry_backward2'} if monitor_only else {}))
                leg['completed_actions'].append('cmd0_carry_backward2')
                plan['partial_reframe_action'] = 'cmd0_carry_backward2'
                recheck = _tag4_position_plan(robot, nav, profile, settings)
                plan['partial_reframe_recheck'] = recheck
                if (_tag4_left_start_ready(recheck, settings)
                        or _tag4_authorize_near_left(robot, nav, settings, recheck, leg,
                                                     reserved_actions=reserved_actions)):
                    plan['approach_confirmed'] = True
                    return {}, True
            return pose, False
    needs_approach = (plan.get('confirmed_far') or plan.get('heading_to_correct_deg') is not None
                      or advisory == 'far')
    if not needs_approach and not plan['distance_verified'] and not plan.get('depth_confirmed'):
        previous = RoutePose_PlanTag4Longitudinal(checkpoint, nav.cfg, settings)
        if previous.get('confirmed_far'):
            plan['last_confirmed_distance_m'] = previous['distance_m']
            needs_approach = True
    if needs_approach:
        return _approach_tag4_before_left(robot, nav, profile, pose, leg, settings, plan,
                                         reserved_actions=reserved_actions, monitor_only=monitor_only)
    if not plan['distance_verified'] and (checkpoint.get('tag4_partial_approach')
            or checkpoint.get('tag4_guarded_forward')
            or advisory in ('conflicting_frames','outside_heading_or_lateral_range')):
        return pose, False
    if plan['distance_verified'] and plan['steps'] == -2:
        return _tag4_retreat_before_left(robot, nav, profile, pose, leg, settings, plan,
                                         reserved_actions=reserved_actions, monitor_only=monitor_only)
    taught_close = _tag4_taught_close(plan, settings)
    if plan.get('confirmed_near') or taught_close:
        last_move = (leg['completed_actions'] or checkpoint.get('completed_actions') or [None])[-1]
        live = getattr(nav.io, 'visual_motion_ready', None)
        if (plan['distance_verified'] and live is not None and live(plan['latest_stamp_ns'])
                and _tag4_can_reverse_approach(plan, settings, last_move,
                    minimum_departure_m=_TAG4_LEFT_PROBE_DEPTH_M if taught_close else None)):
            return _tag4_retreat_before_left(robot, nav, profile, pose, leg, settings, plan,
                                             reserved_actions=reserved_actions, monitor_only=monitor_only)
        plan['retreat_reason_zh'] = (f"Tag 4 距码约 {plan['distance_m']*100:.1f} cm，"
                                    '左跨和沿原路后退均未确认足够余量。')
        return pose, False
    return pose, True


def _approach_tag4_before_left(robot, nav, profile, pose, leg, settings, initial, *, reserved_actions, monitor_only):
    """Prioritize verified two-step advance, then use Tag 4 to check left departure."""
    plan = dict(initial)
    batches = initial['approach_batches'] = []
    turns = initial['heading_recovery'] = []
    turn_from_tag = getattr(robot, 'Route_TurnFromTagHeading', None)
    fresh = getattr(nav.io, 'visual_motion_ready', lambda _: False)
    need_reference = False
    previous_turn = None
    stationary_checks = 0
    _report_progress(robot, 'cmd0_departure_reobserve', 4, 4, leg=leg)
    while True:
        robot.check()
        stamp = plan.get('latest_stamp_ns') or (plan.get('partial_pose') or {}).get('latest_stamp_ns',0)
        ready = (_tag4_left_start_ready(plan, settings)
                 or _tag4_authorize_near_left(robot, nav, settings, plan, leg, reserved_actions=reserved_actions))
        measured_advance = plan['distance_verified'] and plan['steps']==2
        partial_advance = (plan.get('partial_advice') == 'far'
                           and (not batches or (plan.get('partial_pose') or {}).get('samples', 0) >= 2))
        can_advance = ((measured_advance or partial_advance) and len(batches)<_TAG4_APPROACH_GROUPS
                       and robot.count+reserved_actions+1<=robot.cfg['max_body_macros_per_journey'])
        if need_reference and (ready or can_advance):
            rebase = getattr(robot, 'Route_RebaseTagHeading', None)
            if rebase is not None and plan.get('heading_deg') is not None:
                if plan.get('max_heading_deviation_deg', float('inf')) <= 3.:
                    initial['heading_reference'] = rebase(4, plan['heading_deg'], stamp).value
            else:
                nav.mag.anchor = None
                robot.prepare_heading_segment(fresh=True)
            need_reference = False
        if ready and fresh(stamp):
            initial.update(approach_confirmed=True, after_distance_m=plan['distance_m'])
            return pose, True
        taught_close = _tag4_taught_close(plan, settings)
        last_move = (leg['completed_actions'] or [None])[-1]
        if (plan['distance_verified'] and fresh(stamp)
                and (plan['steps'] == -2 or taught_close and _tag4_can_reverse_approach(
                    plan, settings, last_move, minimum_departure_m=_TAG4_LEFT_PROBE_DEPTH_M))):
            return _tag4_retreat_before_left(robot, nav, profile, pose, leg, settings, initial,
                                             reserved_actions=reserved_actions, monitor_only=monitor_only)
        if plan.get('camera_contract_fault'):
            initial.update(approach_confirmed=False, last_approach_reason=plan['reason'])
            return pose, False

        operation = None
        turn = None
        if can_advance and fresh(stamp):
            operation = 'carry_forward2'
            if monitor_only:
                robot.action(operation, monitor_only_as=operation)
            else:
                robot.action(operation)
            leg['completed_actions'].append(operation)
            previous_turn = None
        elif turn_from_tag is not None and plan.get('heading_to_correct_deg') is not None:
            error = plan['heading_to_correct_deg']
            if previous_turn is None or abs(error) <= abs(previous_turn)-1.:
                turn = turn_from_tag(4,error,stamp,reserved_actions=reserved_actions)
                if turn is not None:
                    turns.append(turn)
                    previous_turn = error
                    need_reference = True

        if operation or turn is not None:
            stationary_checks = 0
        elif stationary_checks >= _DEPARTURE_REOBSERVATIONS:
            initial.update(approach_confirmed=False, after_distance_m=plan.get('distance_m'),
                           last_approach_reason=plan['reason'])
            return pose, False
        else:
            stationary_checks += 1
        before = plan.get('distance_m')
        pose, observation = _refresh_checkpoint_pose(robot, nav, profile, 4,
            reuse_settled_cache=operation is not None or turn is not None, quick_id=True)
        plan = _tag4_longitudinal_observation(robot, nav, observation, settings)
        if operation:
            batches.append({'before_distance_m':before,'after_distance_m':plan.get('distance_m'),
                            'partial_pose_probe':partial_advance})
        if turn is not None:
            turn['after_error_deg'] = plan.get('heading_deg')
        issue = _tag4_observed_clearance_problem(plan, nav)
        if issue is not None:
            initial.setdefault('approach_pose_recovery', []).append(issue)
            plan = _tag4_recover_forward_pose(robot, nav, profile, settings, plan, issue,
                                              leg=leg, approach_steps=None, reserved_actions=reserved_actions)
            pose = {}
            if not plan['distance_verified']:
                initial.update(approach_confirmed=False, after_distance_m=plan.get('distance_m'),
                               last_approach_reason=plan['reason'])
                return pose, False


def _trim_tag2_dispatch(robot, nav, leg, pose, plan, *, continuous, reserved_actions):
    operation = plan['operation']
    if operation is None:
        return False
    live = getattr(nav.io, 'visual_motion_ready', None)
    if live is None or not live(plan.get('latest_stamp_ns', pose.get('latest_stamp_ns', 0))):
        plan.update(status='deferred', reason='Tag 2 longitudinal pose expired before dispatch')
        return False
    if ((operation == 'cmd0_carry_backward2' and not continuous)
            or robot.count + reserved_actions + 1 > robot.cfg['max_body_macros_per_journey']):
        plan.update(status='deferred', reason='Tag 2 trim capability or reserved motion budget unavailable')
        return False
    robot.action(operation)
    plan['protocol_completed'] = True
    leg['completed_actions'].append(operation)
    return True


def _observe_tag2_approach(robot, nav, profile, leg, settings, candidate_counts, *, after_turn=False):
    _report_progress(robot, 'cmd0_departure_reobserve', 2, 2, leg=leg)
    # Refresh only before forward planning, including after a heading correction.
    tag2_seen = False
    for attempt in range(2):
        pose, observation = _refresh_checkpoint_pose(robot, nav, profile, 2, quick_id=True,
                                                     reuse_settled_cache=attempt == 0)
        scan = observation.get('scan') or {}
        tag2_seen = (tag2_seen or observation.get('detected') is True or 2 in scan.get('observed_ids', ())
                     or bool(observation.get('partial_pose_rows'))
                     or any(row.get('decoded_since_boundary', row.get('decoded_expected_frames', 0))
                            for row in observation.get('quick_scan_windows', ())))
        checked = RoutePose_PlanTag2ForwardTrial(observation, nav.cfg, settings, candidate_counts=candidate_counts)
        if 'distance_bounds_m' in checked:
            break
        if after_turn:
            partial = _tag2_after_turn_partial_reframe(robot, nav, observation, settings)
            if partial is not None:
                checked['partial_forward_reframe'] = partial
                break
    observation['tag2_seen_during_approach'] = tag2_seen
    leg['tag2_side_hint'] = RoutePose_TrialLateralHint(observation, nav.cfg, 2)
    return pose, observation, checked


def _tag2_departure_distance_ok(plan, settings):
    if plan.get('confirmed_near'):
        return False
    if plan.get('in_departure_window') is True:
        return True
    if 'distance_bounds_m' not in plan:
        return False
    lo, hi = plan['distance_bounds_m']
    near, far = settings['distance_window_m']
    # Keep the near boundary usable when submillimeter variation straddles it.
    return (near <= plan['distance_m'] <= far and lo >= near - .003 and hi <= far
            and hi - lo <= .006)


def _tag2_reframe_for_view(robot, nav, profile, leg, *, operation, distance_m, reserved_actions,
                           fresh_turn_pose=None, monitor_only=False, heading_clearance=False):
    if (leg.get('tag2_close_reframe') or leg.get('tag2_far_reframe') or leg.get('tag2_heading_clearance')
            or not Cmd0Motion_ContinuousReady(robot.backend)
            or robot.count + reserved_actions + 1 > robot.cfg['max_body_macros_per_journey']):
        return None
    backward = operation == 'cmd0_carry_backward2'
    reason = ('Tag 2 太近，先后退两步把码移回画面，再复核右跨起点。' if backward
              else 'Tag 2 的少量新帧表明仍偏远，先前探两步，再复核右跨起点。')
    if heading_clearance:
        reason = 'Tag 2 航向偏差较大，沿已走过的路后退两步让开障碍，再用新帧定转向。'
    robot.check()
    if fresh_turn_pose is not None:
        live = getattr(nav.io,'visual_motion_ready',None)
        if (live is None or fresh_turn_pose['stance_completed_ns'] != robot.last_action_completed_ros_ns
                or not live(fresh_turn_pose['latest_stamp_ns'])):
            return None
    recovery = {'before_distance_m': distance_m, 'operation': operation, 'completed_steps': 0,
                'status': 'rechecking', 'reason_zh': reason}
    if fresh_turn_pose is not None:
        recovery['after_turn_partial_pose'] = fresh_turn_pose
    key = 'tag2_heading_clearance' if heading_clearance else 'tag2_close_reframe' if backward else 'tag2_far_reframe'
    leg[key] = recovery
    _report_progress(robot, 'cmd0_departure_reobserve', 2, 2, leg=leg, reason_zh=recovery['reason_zh'])
    if fresh_turn_pose is not None or monitor_only:
        robot.action(operation,monitor_only_as=operation)
    else:
        robot.action(operation)
    leg['completed_actions'].append(operation)
    recovery['completed_steps'] = 2
    recovery['windows'] = []
    for attempt in range(_TAG2_DEPARTURE_WINDOWS):
        _, observation = _refresh_checkpoint_pose(robot, nav, profile, 2, quick_id=True,
                                                  reuse_settled_cache=attempt == 0)
        recovery['windows'].extend(observation.get('quick_scan_windows', ()))
        if observation['detected']:
            recovery['status'] = 'recovered'
            return observation
    recovery['status'] = 'distance_unconfirmed'
    return None


def _tag2_recover_heading_clearance(robot, nav, profile, leg, plan, settings, *, reserved_actions):
    """Reverse on the approach just traveled when a Tag 2 turn needs wall clearance."""
    stance = robot.last_action_completed_ros_ns
    stamp = plan.get('latest_stamp_ns', 0)
    live = getattr(nav.io, 'visual_motion_ready', None)
    traversed = leg.get('entry_completed_steps', 0) >= 2 or 'carry_forward2' in leg['completed_actions']
    bounds = plan.get('lateral_bounds_m')
    projection = plan.get('projection_bounds')
    spread = plan.get('max_heading_deviation_deg', float('inf'))
    if (not traversed or bounds is None or projection is None or spread > 3.
            or projection[0] < .75 or live is None or type(stance) is not int
            or type(stamp) is not int or stamp <= stance or not live(stamp)):
        return None
    depth = plan['distance_bounds_m']
    near, far = settings['distance_window_m']
    travel = settings.get('approach_two_step_distance_m', settings['two_step_distance_m'])[1]
    yaw = plan['heading_deg']
    shifts = (0., *(-travel * sin(radians(angle)) for angle in (yaw - spread, yaw + spread)))
    if (not near <= depth[0] <= depth[1] <= far - travel * projection[1]
            or bounds[0] + min(shifts) < -_TAG2_DEPARTURE_LATERAL_M
            or bounds[1] + max(shifts) > _TAG2_DEPARTURE_LATERAL_M):
        return None
    return _tag2_reframe_for_view(robot, nav, profile, leg, operation='cmd0_carry_backward2',
        distance_m=plan['distance_m'], reserved_actions=reserved_actions + 1, monitor_only=True,
        heading_clearance=True)


def _tag2_partial_reframe_plan(rows, nav, settings):
    """Use same-stance partial geometry only to choose whether and where to reobserve."""
    geometry = RoutePose_PartialCheckpointGeometry(rows, nav.cfg, 2)
    plan = {'status': 'pose_uncertain', 'operation': None}
    if not geometry:
        return plan
    plan.update(geometry)
    low, high = geometry['distance_bounds_m']
    near, far = settings['distance_window_m']
    left, right = geometry['lateral_bounds_m']
    facing, most_forward = geometry['projection_bounds']
    foremost, _ = geometry['body_forward_bounds_m']
    if (not .02 <= low <= high <= .50 or high-low > .03 or facing < .85 or foremost <= 0.
            or geometry['max_abs_heading_deg'] > 12. or geometry['max_heading_deviation_deg'] > 4.
            or max(abs(left), abs(right)) > .10 or right-left > .035):
        return plan
    plan['distance_m'] = (low+high)/2
    if high <= near+.01:
        return dict(plan, status='too_close', operation='cmd0_carry_backward2')
    # A singleton needs more margin than multiple consistent observations.
    margin = .04 if geometry['samples'] == 1 else .015
    travel = settings.get('approach_two_step_distance_m', settings['two_step_distance_m'])[1]
    if low >= far+margin and low-travel*most_forward >= near+.03:
        return dict(plan, status='too_far', operation='carry_forward2')
    if near+.01 <= low <= high <= far:
        plan['status'] = 'within_window_needs_fresh_pose'
    return plan


def _tag2_after_turn_partial_reframe(robot, nav, observation, settings):
    """Authorize one short probe with consistent post-turn frames from the same stance."""
    scan = observation.get('scan') or {}
    camera = observation.get('camera_info') or {}
    pinned = nav.cfg.get('visual_micro',{}).get('camera_info',{})
    stance = robot.last_action_completed_ros_ns
    now = scan.get('now_ns')
    restart = scan.get('camera_restart_ns', 0)
    live = getattr(nav.io,'visual_motion_ready',None)
    if (type(stance) is not int or observation.get('partial_pose_same_stance_ns') != stance
            or type(now) is not int or type(restart) is not int or live is None
            or scan.get('camera_healthy') is not True or scan.get('contract_match') is not True
            or camera.get('ready') is not True or pinned.get('source_confirmed') is not True
            or not camera.get('fingerprint') or camera['fingerprint'] != pinned.get('expected_fingerprint')):
        return None
    cutoff = max(stance, restart)
    age_limit = nav.cfg['max_age_s']*1e9
    rows = [row for row in observation.get('partial_pose_rows',())
            if type(row.get('stamp_ns')) is int and cutoff < row['stamp_ns'] <= now
            and now-row['stamp_ns'] <= age_limit]
    plan = _tag2_partial_reframe_plan(rows,nav,settings)
    if plan.get('samples',0) < 2 or plan.get('operation') != 'carry_forward2' or not live(plan['latest_stamp_ns']):
        return None
    return dict(plan,stance_completed_ns=stance)


def _tag2_recheck_before_strafe(robot, nav, profile, leg, checkpoint, *, heading_changed,
                               departure_heading, reserved_actions):
    """Return a recent checkpoint, or retain the distinction between lost and unseen."""
    scan = checkpoint.get('scan') or {}
    live = getattr(nav.io, 'visual_motion_ready', None)
    stance = robot.last_action_completed_ros_ns
    stamp = max((row['stamp_ns'] for row in scan.get('observations', ())
                 if row.get('id') == 2 and type(row.get('stamp_ns')) is int), default=0)
    if (checkpoint.get('detected') and not heading_changed and type(stance) is int
            and stamp > stance and live is not None and live(stamp)):
        return checkpoint, True
    current = _corridor_boundary_checkpoint(robot, nav, 2)
    if current is not None:
        leg['tag2_departure_recheck'] = {'status': 'recovered', 'reused_boundary_frames': True}
        return current, True

    miss = checkpoint.get('checkpoint_not_seen') or {}
    sighted = (checkpoint.get('detected') is True or bool(checkpoint.get('tag2_boundary_sighting'))
               or checkpoint.get('tag2_seen_earlier_boundary') is True
               or 2 in miss.get('seen_tags', ()) or 2 in scan.get('observed_ids', ())
               or any(row.get('decoded_since_boundary', row.get('decoded_expected_frames',0))
                      for row in checkpoint.get('quick_scan_windows', ())))
    recheck = {'seen_before': sighted, 'windows': []}
    leg['tag2_departure_recheck'] = recheck
    if checkpoint.get('tag2_boundary_sighting'):
        recheck['boundary_sighting'] = checkpoint['tag2_boundary_sighting']
    forward_plan = checkpoint.get('forward_plan') or {}
    boundary = checkpoint.get('tag2_boundary_sighting') or {}
    near = boundary.get('near_advisory') or {}
    rows = []
    if not heading_changed:
        if checkpoint.get('partial_pose_same_stance_ns') == stance:
            rows.extend(checkpoint.get('partial_pose_rows', ()))
        if boundary.get('stance_completed_ns') == stance:
            rows.extend(boundary.get('partial_pose_rows', ()))
    settings = robot.cfg['route_steps']['tag2_forward_trial']
    camera_status = getattr(nav.io, 'visual_camera_status', None)
    scan_at = scan.get('now_ns')
    cutoff = scan.get('after_ns')
    now = nav.io.ros_now_ns()
    if (not sighted and not heading_changed and miss.get('cause') == 'tag_not_visible'
            and miss.get('healthy_windows', 0) >= 2 and scan.get('camera_healthy') is True
            and scan.get('contract_match') is True and type(cutoff) is int and type(scan_at) is int
            and type(now) is int and type(stance) is int and stance <= cutoff <= scan_at <= now
            and now - scan_at <= nav.cfg['max_age_s'] * 1e9
            and camera_status is not None and camera_status().get('ready') is True):
        recheck.update(status='unseen_continue_estimated', reused_healthy_windows=miss['healthy_windows'])
        return {}, True
    deadband = nav.cfg['post_action_heading'].get('deadband_deg')
    heading_steady = (departure_heading is not None and deadband is not None
                      and abs(departure_heading[0]) <= deadband)
    legacy_near = (not heading_changed and near.get('advisory_only') is True
                   and near.get('stance_completed_ns') == stance)
    _report_progress(robot, 'cmd0_departure_reobserve', 2, 2, leg=leg,
                     reason_zh='复核 Tag 2 的前后距离，再决定是否右跨。')
    healthy = False
    for attempt in range(_TAG2_DEPARTURE_WINDOWS):
        _, observation = _refresh_checkpoint_pose(robot, nav, profile, 2, quick_id=True,
                                                  reuse_settled_cache=attempt == 0)
        observed = observation.get('scan') or {}
        recheck['windows'].extend(observation.get('quick_scan_windows', ()))
        if observation['detected']:
            recheck['status'] = 'recovered'
            return observation, True
        sighted = (sighted or 2 in observed.get('observed_ids', ())
                   or any(row.get('decoded_since_boundary', row.get('decoded_expected_frames',0))
                          for row in observation.get('quick_scan_windows', ())))
        healthy = observed.get('camera_healthy') is True and observed.get('contract_match') is True
        if not healthy or robot.last_action_completed_ros_ns != stance:
            rows.clear()
            legacy_near = False
            continue
        restart = observed.get('camera_restart_ns')
        now = observed.get('now_ns')
        if type(now) is int:
            cutoff = now - _TAG2_BOUNDARY_HINT_MAX_AGE_NS
            rows = [row for row in rows if type(row.get('stamp_ns')) is int and cutoff <= row['stamp_ns'] <= now]
            hint_stamp = near.get('latest_stamp_ns')
            if type(hint_stamp) is int and not cutoff <= hint_stamp <= now:
                legacy_near = False
        if type(restart) is int and restart > 0:
            rows = [row for row in rows if type(row.get('stamp_ns')) is int and row['stamp_ns'] > restart]
            legacy_near = legacy_near and near.get('latest_stamp_ns', 0) > restart
        if observation.get('partial_pose_same_stance_ns') == stance:
            rows.extend(observation.get('partial_pose_rows', ()))
        if rows:
            recheck['partial_pose_advisory'] = plan = _tag2_partial_reframe_plan(rows, nav, settings)
        elif legacy_near:
            plan = {'status': 'too_close', 'operation': 'cmd0_carry_backward2', 'distance_m': near['distance_m']}
        else:
            continue
        sighted = True
        if plan['operation'] is None:
            continue
        if not heading_steady:
            plan['reframe_blocked'] = 'heading_unconfirmed'
            continue
        if camera_status is not None and camera_status().get('ready') is not True:
            plan['reframe_blocked'] = 'camera_unconfirmed'
            continue
        observation = _tag2_reframe_for_view(robot, nav, profile, leg, operation=plan['operation'],
                                             distance_m=plan['distance_m'], reserved_actions=reserved_actions)
        recheck['status'] = 'recovered' if observation else 'distance_unconfirmed'
        return observation or {}, observation is not None
    if sighted or not healthy or forward_plan.get('reused_same_stance_reference') is True:
        recheck['status'] = 'distance_unconfirmed'
        return {}, False
    recheck['status'] = 'unseen_continue_estimated'
    return {}, True


def _tag2_lateral_clearance_issue(plan, live):
    bounds = plan.get('lateral_bounds_m')
    if (bounds is None or bounds[1] - bounds[0] > .025
            or plan.get('max_heading_deviation_deg', float('inf')) > 3.
            or (plan.get('projection_bounds') or (-1.,))[0] < .85
            or live is None or not live(plan['latest_stamp_ns'])):
        return None
    if bounds[1] < -_TAG2_DEPARTURE_LATERAL_M or bounds[0] > _TAG2_DEPARTURE_LATERAL_M:
        return bounds
    return None


def _recover_tag2_lateral_start(robot, nav, profile, leg, plan, settings, *,
                               reserved_actions, candidate_counts):
    """Confirm the effect of one right stride borrowed from the normal corridor plan."""
    def depth_clear(current):
        near, far = settings['distance_window_m']
        low, high = current['distance_bounds_m']
        return near + .02 <= low <= high <= far - .02 and high - low <= .025

    side_action = getattr(robot, 'corridor_side_action', None)
    departure = (leg.get('configured_steps') or [{}])[0]
    if (side_action is None or departure.get('op') != 'strafe_right_once' or departure.get('repeat', 0) < 2
            or plan['lateral_bounds_m'][0] < -.30 or plan['lateral_bounds_m'][1] >= 0.
            or abs(plan['heading_deg']) > 8. or not depth_clear(plan)
            or robot.count + reserved_actions + 1 > robot.cfg['max_body_macros_per_journey']):
        return None
    operation = 'strafe_right_once'
    recovery = {'operation': operation, 'before_lateral_bounds_m': plan['lateral_bounds_m'],
                'confirmed': False}
    leg['tag2_lateral_recovery'] = recovery
    _report_progress(robot, 'cmd0_departure_reobserve', 2, 2, leg=leg,
                     reason_zh='Tag 2 明显偏左，先执行计划内的一步右跨，再确认走廊入口。')
    side_action(operation)
    leg['completed_actions'].append(operation)
    pose, observation = _refresh_checkpoint_pose(robot, nav, profile, 2, reuse_settled_cache=True, quick_id=True,
                                                 boundary_first=True)
    after = RoutePose_PlanTag2ForwardTrial(observation, nav.cfg, settings, candidate_counts=candidate_counts)
    bounds = after.get('lateral_bounds_m')
    recovery['after_lateral_bounds_m'] = bounds
    live = getattr(nav.io, 'visual_motion_ready', None)
    if (bounds is None or bounds[1] - bounds[0] > .025 or bounds[0] < -_TAG2_DEPARTURE_LATERAL_M
            or bounds[1] > _TAG2_DEPARTURE_LATERAL_M or bounds[0] <= plan['lateral_bounds_m'][1] + .02
            or abs(after['heading_deg']) > 8. or after['max_heading_deviation_deg'] > 3.
            or not depth_clear(after)
            or live is None or not live(after['latest_stamp_ns'])):
        return None
    leg['tag2_side_hint'] = RoutePose_TrialLateralHint(observation, nav.cfg, 2)
    recovery.update(confirmed=True, nominal_strides_used=1)
    return pose, after


def _tag2_project_short_approach_lateral(robot, plan, settings):
    """Propagate Tag 2 lateral error across one acknowledged forward pair."""
    bounds = plan.get('lateral_bounds_m')
    yaw = plan.get('heading_deg')
    report = getattr(robot, 'last_heading_report', None) or {}
    if (bounds is None or bounds[1] - bounds[0] > .025 or yaw is None
            or abs(yaw) > _TAG2_CARRIED_MAX_YAW_DEG or report.get('action') != 'carry_forward2'):
        return None
    end = CorridorTrial_HeadingSnapshot(report, after=False, load=robot.load_kind)
    settled = CorridorTrial_HeadingSnapshot(report, after=True, load=robot.load_kind)
    if (end is None or settled is None or end[2] != settled[2] or abs(end[0]) > 25.
            or abs(settled[0]) > _TAG2_CARRIED_MAX_YAW_DEG):
        return None
    travel = settings.get('approach_two_step_distance_m', settings['two_step_distance_m'])
    # The before-correction heading bounds translation. A turn at the end
    # can induce separate translational slip, accounted for by the margin.
    shifts = [distance * sin(radians(angle)) for distance in travel for angle in (yaw, end[0])]
    margin = _TAG2_CARRIED_LATERAL_MARGIN_M
    return {'tag_id': 2, 'basis': 'tag2_short_approach_projection',
            'measured_lateral_bounds_m': bounds,
            'lateral_bounds_m': (bounds[0] + min(shifts) - margin, bounds[1] + max(shifts) + margin),
            'heading_deg': settled[0], 'heading_at_forward_end_deg': end[0],
            'latest_stamp_ns': plan['latest_stamp_ns']}


def _tag2_landing_observation(robot, nav, profile, leg, settings):
    checkpoint = _corridor_boundary_checkpoint(robot, nav, 2)
    if checkpoint is not None:
        plan = _tag2_current_visual_plan(robot, nav, checkpoint, continuous=True, candidate_counts=(2,))
        if plan is not None:
            return _measure(robot, nav, profile, checkpoint), checkpoint, plan
    return _observe_tag2_approach(robot, nav, profile, leg, settings, (2,))


def _tag2_guarded_approach(robot, nav, profile, leg, initial, settings, plan, *, reserved_actions):
    """Use short straight groups near the wall and promote only current pose evidence."""
    near, far = settings['distance_window_m']
    short, long = settings.get('approach_two_step_distance_m', settings['two_step_distance_m'])
    groups = (initial['planned_steps'] + 1) // 2
    live = getattr(nav.io, 'visual_motion_ready', lambda _: False)
    windows = initial['landing_checks'] = []
    turn = getattr(robot, 'Route_TurnFromTagHeading', None)
    rebase = getattr(robot, 'Route_RebaseTagHeading', None)
    leg['tag2_narrow_departure'] = True
    leg.pop('tag2_side_hint', None)
    for index in range(groups):
        robot.check()
        if robot.count + reserved_actions + 1 > robot.cfg['max_body_macros_per_journey']:
            break
        distance = plan['distance_bounds_m']
        projection = plan.get('projection_bounds', (1., 1.))
        estimated = (distance[0] - long * projection[1], distance[1] - short * projection[0])
        _report_progress(robot, 'cmd0_departure_reobserve', 2, 2, leg=leg,
                         reason_zh='Tag 2 进近余量较小，先前进两步，再用新画面复核。')
        robot.last_heading_report = None
        robot.action('carry_forward2', monitor_only_as='carry_forward2')
        leg['completed_actions'].append('carry_forward2')
        initial['planned_actions'].append('carry_forward2')
        initial['completed_steps'] += 2
        pose, checkpoint, current = _tag2_landing_observation(robot, nav, profile, leg, settings)
        check = {'reported_steps': initial['completed_steps'], 'estimated_distance_bounds_m': estimated,
                 'observed': current}
        windows.append(check)
        scan = checkpoint.get('scan') or {}
        if 'distance_bounds_m' not in current:
            partial = _corridor_partial_geometry(robot, nav, checkpoint, 2, fresh_only=True)
            if partial is not None:
                low, high = partial['distance_bounds_m']
                if (partial['samples'] >= 2 and partial['consistent'] and partial['latest_fresh']
                        and near + _TAG2_APPROACH_NEAR_MARGIN_M <= low <= high <= far
                        and partial['max_abs_heading_deg'] <= 8. and partial['projection_bounds'][0] >= .85
                        and max(map(abs, partial['lateral_bounds_m'])) <= .10):
                    check.update(status='partial_visual_cautious_departure', partial_pose=partial)
                    if rebase is not None:
                        initial['heading_reference'] = rebase(2, partial['heading_deg'], partial['latest_stamp_ns']).value
                    initial.update(protocol_completed=True, ready_to_strafe=True, measured_landing_m=(low, high))
                    return {}, True
            compass = CorridorTrial_HeadingSnapshot(getattr(robot, 'last_heading_report', None) or {},
                                                      after=True, load=robot.load_kind)
            camera = checkpoint.get('camera_info') or {}
            pinned = nav.cfg.get('visual_micro', {}).get('camera_info') or {}
            if (not checkpoint.get('tag2_seen_during_approach') and partial is None
                    and not checkpoint.get('partial_pose_rows') and 2 not in scan.get('observed_ids', ())
                    and scan.get('camera_healthy') is True and scan.get('contract_match') is True
                    and pinned.get('source_confirmed') is True and camera.get('ready') is True
                    and camera.get('fingerprint') == pinned.get('expected_fingerprint')
                    and compass is not None and abs(compass[0]) <= 8.
                    and near + _TAG2_APPROACH_NEAR_MARGIN_M <= estimated[0] <= estimated[1] <= far):
                check.update(status='estimated_cautious_departure', magnetic_error_deg=compass[0])
                initial.update(protocol_completed=True, ready_to_strafe=True, estimated_landing_m=estimated)
                return {}, True
            break
        yaw = current.get('heading_to_correct_deg')
        if (yaw is not None and abs(yaw) >= _TAG2_CLEARANCE_YAW_DEG
                and not (leg.get('tag2_heading_clearance') or leg.get('tag2_close_reframe'))):
            recovered = _tag2_recover_heading_clearance(robot, nav, profile, leg, current, settings,
                                                         reserved_actions=reserved_actions)
            if recovered is None:
                check['status'] = 'heading_clearance_unconfirmed'
                break
            checkpoint = recovered
            pose = _measure(robot, nav, profile, checkpoint)
            current = RoutePose_PlanTag2ForwardTrial(checkpoint, nav.cfg, settings, candidate_counts=(2,))
            check['after_heading_clearance'] = current
            yaw = current.get('heading_to_correct_deg')
        if yaw is not None and turn is not None and live(current['latest_stamp_ns']):
            correction = turn(2, yaw, current['latest_stamp_ns'], reserved_actions=reserved_actions)
            if correction is not None:
                initial.setdefault('heading_recovery', []).append(correction)
                pose, checkpoint, current = _tag2_landing_observation(robot, nav, profile, leg, settings)
                correction['after_error_deg'] = current.get('heading_deg')
                check['after_turn'] = current
        if (current.get('confirmed_near') and current.get('reverse_clearance_available', True)
                and live(current['latest_stamp_ns'])):
            recovered = _tag2_reframe_for_view(robot, nav, profile, leg,
                operation='cmd0_carry_backward2', distance_m=current['distance_m'],
                reserved_actions=reserved_actions, monitor_only=True)
            if recovered is not None:
                checkpoint = recovered
                pose = _measure(robot, nav, profile, checkpoint)
                current = RoutePose_PlanTag2ForwardTrial(checkpoint, nav.cfg, settings, candidate_counts=(2,))
                check['after_retreat'] = current
        hint = RoutePose_TrialLateralHint(checkpoint, nav.cfg, 2)
        if (hint is None or not live(hint['latest_stamp_ns'])
                or _tag2_lateral_clearance_issue(current, live) is not None):
            break
        if rebase is not None:
            initial['heading_reference'] = rebase(2, hint['heading_deg'], hint['latest_stamp_ns']).value
        leg['tag2_visual_heading_deg'] = hint['heading_deg']
        landing = current.get('estimated_landing_m')
        if (index + 1 < groups and current['planned_steps'] == 2 and landing is not None
                and landing[0] >= near + _TAG2_APPROACH_NEAR_MARGIN_M and live(hint['latest_stamp_ns'])):
            plan = current
            continue
        if _tag2_departure_distance_ok(current, settings):
            check['status'] = 'visual_departure_confirmed'
            leg['tag2_side_hint'] = hint
            depth = current['distance_bounds_m']
            leg['tag2_backward_clearance_allowed'] = depth[1] + long <= far
            initial.update(protocol_completed=True, ready_to_strafe=True, measured_landing_m=depth)
            return pose, True
        break
    initial['last_approach_reason_zh'] = 'Tag 2 分组进近后，右跨起点或航向仍未确认。'
    return {}, False


def _run_tag2_approach(robot, nav, profile, pose, leg, initial, settings, *,
                       continuous, candidate_counts, reserved_actions, require_verified):
    """Correct heading, select the full forward distance, then follow the plan."""
    latest = dict(initial)
    turn_from_tag = getattr(robot, 'Route_TurnFromTagHeading', None)
    rebase = getattr(robot, 'Route_RebaseTagHeading', None)
    last_turn = None
    refreshed = False
    live = getattr(nav.io, 'visual_motion_ready', None)
    while True:
        yaw = latest.get('heading_to_correct_deg')
        step_count = latest['planned_steps']
        reframed = leg.get('tag2_close_reframe') or leg.get('tag2_far_reframe')
        if (latest.get('confirmed_near') and latest.get('reverse_clearance_available',True)
                and live and live(latest['latest_stamp_ns']) and not reframed):
            observation = _tag2_reframe_for_view(robot, nav, profile, leg, operation='cmd0_carry_backward2',
                                                 distance_m=latest['distance_m'], reserved_actions=reserved_actions)
            if observation is not None:
                pose = _measure(robot, nav, profile, observation)
                latest = RoutePose_PlanTag2ForwardTrial(observation, nav.cfg, settings, candidate_counts=candidate_counts)
                leg['tag2_side_hint'] = RoutePose_TrialLateralHint(observation, nav.cfg, 2)
                continue
            break
        if reframed and _tag2_departure_distance_ok(latest, settings):
            step_count = 0
            latest.update(reason='Tag 2 is inside the strafe window after reframing')
        fresh = bool(live and live(latest.get('latest_stamp_ns', 0))) if yaw is not None or step_count else True
        if yaw is not None and turn_from_tag is not None:
            if (abs(yaw) >= _TAG2_CLEARANCE_YAW_DEG and fresh
                    and not (leg.get('tag2_heading_clearance') or leg.get('tag2_close_reframe'))):
                observation = _tag2_recover_heading_clearance(robot, nav, profile, leg, latest, settings,
                                                               reserved_actions=reserved_actions)
                if observation is None:
                    initial['last_approach_reason_zh'] = 'Tag 2 航向偏差较大，未确认转向前的后退余量。'
                    break
                pose = _measure(robot, nav, profile, observation)
                latest = RoutePose_PlanTag2ForwardTrial(observation, nav.cfg, settings,
                                                        candidate_counts=candidate_counts)
                leg['tag2_side_hint'] = RoutePose_TrialLateralHint(observation, nav.cfg, 2)
                continue
            if last_turn and abs(yaw) > abs(last_turn['before_error_deg'])-1.:
                latest.update(reason='Tag 2 turn did not reduce measured heading error')
                break
            turn = (turn_from_tag(2,yaw,latest['latest_stamp_ns'],reserved_actions=reserved_actions)
                    if fresh else None)
            if turn is not None:
                initial.setdefault('heading_recovery', []).append(turn)
                last_turn = turn
                pose, observation, latest = _observe_tag2_approach(robot, nav, profile, leg, settings,
                                                                candidate_counts,after_turn=True)
                turn['after_error_deg'] = latest.get('heading_deg')
                initial['after_scan'] = observation.get('scan')
                partial = latest.get('partial_forward_reframe')
                if partial is not None:
                    turn['partial_after_error_deg'] = partial['heading_deg']
                    recovered = _tag2_reframe_for_view(robot,nav,profile,leg,operation='carry_forward2',
                        distance_m=partial['distance_m'],reserved_actions=reserved_actions,fresh_turn_pose=partial)
                    if recovered is not None:
                        pose = _measure(robot,nav,profile,recovered)
                        latest = RoutePose_PlanTag2ForwardTrial(recovered,nav.cfg,settings,
                                                                candidate_counts=candidate_counts)
                        leg['tag2_side_hint'] = RoutePose_TrialLateralHint(recovered,nav.cfg,2)
                    elif leg.get('tag2_far_reframe',{}).get('completed_steps') == 2:
                        latest['reason'] = 'Tag 2 转后已前探两步，右跨起点仍待确认。'
                continue
        if not refreshed and (latest.get('heading_reobserve_required') or not fresh):
            refreshed = True
            pose, observation, latest = _observe_tag2_approach(robot, nav, profile, leg, settings, candidate_counts)
            initial['after_scan'] = observation.get('scan')
            continue
        break

    initial.update(approach_distance_m=latest.get('distance_m'), planned_steps=step_count,
                   completed_steps=0, ready_to_strafe=False, last_approach_reason=latest['reason'])
    leg['tag2_backward_clearance_allowed'] = False
    if yaw is not None or latest.get('heading_reobserve_required'):
        return pose, False
    lateral_clearance_recovered = False
    lateral = _tag2_lateral_clearance_issue(latest, live)
    if lateral is not None:
        recovered = _recover_tag2_lateral_start(robot, nav, profile, leg, latest, settings,
                                                reserved_actions=reserved_actions, candidate_counts=candidate_counts)
        if recovered is None:
            initial.update(lateral_clearance_blocked=True, lateral_bounds_m=lateral,
                           last_approach_reason='Tag 2 measured lateral position is outside the corridor departure range')
            return pose, False
        pose, latest = recovered
        lateral_clearance_recovered = True
        step_count = latest['planned_steps']
        fresh = True  # The recovery validates a newly observed, same-stance pose.
        initial.update(planned_steps=step_count, approach_distance_m=latest['distance_m'],
                       last_approach_reason='Tag 2 lateral clearance confirmed after one planned right stride')
    if (rebase is not None and latest.get('max_heading_deviation_deg', float('inf')) <= 3.
            and abs(latest['heading_deg']) <= 12.):
        outcome = rebase(2, latest['heading_deg'], latest['latest_stamp_ns'])
        initial['heading_reference'] = outcome.value
        leg['tag2_visual_heading_deg'] = latest['heading_deg']
    reverse_bound = settings.get('approach_two_step_distance_m', settings['two_step_distance_m'])[1]
    far = settings['distance_window_m'][1]
    if not step_count:
        ready = lateral_clearance_recovered or _tag2_departure_distance_ok(latest, settings)
        initial['ready_to_strafe'] = ready
        if ready:
            leg['tag2_backward_clearance_allowed'] = latest['distance_bounds_m'][1] + reverse_bound <= far
        return pose, ready or (not require_verified and 'distance_bounds_m' not in latest
                               and initial.get('confirmed_far') is not True)
    if not fresh:
        initial['last_approach_reason'] = 'Tag 2 longitudinal pose expired before dispatch'
        return pose, False
    actions = _forward_actions(step_count, continuous=continuous, legacy=False)
    if not actions or robot.count + reserved_actions + len(actions) > robot.cfg['max_body_macros_per_journey']:
        initial['last_approach_reason'] = 'Tag 2 trim capability or reserved motion budget unavailable'
        return pose, False

    landing = latest['estimated_landing_m']
    before_lateral = latest.get('lateral_bounds_m', ())
    if (require_verified and step_count >= 4
            and (landing[0] < settings['distance_window_m'][0] + _TAG2_APPROACH_NEAR_MARGIN_M
                 or any(abs(value) >= _TAG2_CAUTION_LATERAL_M for value in before_lateral))):
        initial.update(planned_actions=[], estimated_landing_m=landing)
        return _tag2_guarded_approach(robot, nav, profile, leg, initial, settings, latest,
                                      reserved_actions=reserved_actions)
    initial.update(planned_actions=list(actions), estimated_landing_m=landing)
    _report_progress(robot, 'cmd0_departure_reobserve', 2, 2, leg=leg,
                     reason_zh=f'Tag 2 按估算前进 {step_count} 步。')
    for operation in actions:
        robot.action(operation)
        leg['completed_actions'].append(operation)
        initial['completed_steps'] += Cmd0Motion_ForwardStepCount((operation,))
    if robot.load_kind == 'double' and landing[0] > settings['target_distance_m'] + settings['deadband_m']:
        # Two-step travel with a double grip was overestimated in the field.
        # Re-use one fresh close pose only when the first plan could not reach the target band.
        _, observation = _refresh_checkpoint_pose(robot, nav, profile, 2, quick_id=True)
        followup = RoutePose_PlanTag2ForwardTrial(observation, nav.cfg, settings, candidate_counts=(2,))
        initial['paired_approach_followup'] = followup
        if (followup.get('planned_steps') == 2 and live is not None and live(followup['latest_stamp_ns'])
                and robot.count + reserved_actions + 1 <= robot.cfg['max_body_macros_per_journey']):
            robot.action('carry_forward2')
            leg['completed_actions'].append('carry_forward2')
            initial['planned_actions'].append('carry_forward2')
            initial['planned_steps'] += 2
            initial['completed_steps'] += 2
            landing = followup['estimated_landing_m']
            initial['estimated_landing_m'] = landing
        elif 'distance_bounds_m' in followup and not _tag2_departure_distance_ok(followup, settings):
            initial['last_approach_reason'] = followup['reason']
            return pose, False
    initial.update(protocol_completed=True, ready_to_strafe=True)
    # The pre-motion pose no longer describes this stance. Keep an estimate
    # separately so only the missed-Tag 3 path may use it.
    leg.pop('tag2_side_hint', None)
    if initial.get('heading_reference') == 'applied' and initial['planned_actions'] == ['carry_forward2']:
        hint = _tag2_project_short_approach_lateral(robot, latest, settings)
        if hint is not None:
            leg['tag2_projected_side_hint'] = hint
    leg['tag2_backward_clearance_allowed'] = landing[1] + reverse_bound <= far
    return {}, True


def _trim_tag2_forward(robot, nav, profile, pose, leg, checkpoint, *, continuous, reserved_actions,
                       require_verified=False, trial_plan=None):
    plan = RoutePose_PlanTag2ForwardTrim(pose, profile)
    settings = robot.cfg.get('route_steps', {}).get('tag2_forward_trial') or {}
    if (settings.get('enabled') is True
            and (plan.get('reason') == 'Tag 2 two-step longitudinal reference unverified'
                 or (robot.load_kind == 'double' and pose.get('position_usable') is not True))):
        candidate_counts = TAG2_APPROACH_COUNTS if continuous else tuple(count for count in TAG2_APPROACH_COUNTS if count % 2 == 0)
        plan = (trial_plan if trial_plan is not None else
                RoutePose_PlanTag2ForwardTrial(checkpoint, nav.cfg, settings, candidate_counts=candidate_counts))
        leg['tag2_forward_trim'] = plan
        return _run_tag2_approach(robot, nav, profile, pose, leg, plan, settings, continuous=continuous,
            candidate_counts=candidate_counts, reserved_actions=reserved_actions, require_verified=require_verified)
    leg['tag2_forward_trim'] = plan
    if _trim_tag2_dispatch(robot, nav, leg, pose, plan, continuous=continuous, reserved_actions=reserved_actions):
        leg.pop('tag2_side_hint', None)
        leg['tag2_backward_clearance_allowed'] = False
        return {}, True
    return pose, not require_verified or (plan['operation'] is None and pose.get('position_usable') is True)


def _tag2_current_visual_plan(robot, nav, checkpoint, *, continuous, candidate_counts=None):
    """Give a pinned, coherent Tag 2 view priority over a magnetic departure turn."""
    scan = checkpoint.get('scan') or {}
    camera = checkpoint.get('camera_info') or {}
    after = scan.get('after_ns')
    stance = robot.last_action_completed_ros_ns
    live = getattr(nav.io, 'visual_motion_ready', None)
    if (checkpoint.get('detected') is not True or camera.get('ready') is not True
            or type(after) is not int or type(stance) is not int or not 0 < stance <= after or live is None):
        return None
    settings = robot.cfg['route_steps']['tag2_forward_trial']
    if candidate_counts is None:
        candidate_counts = TAG2_APPROACH_COUNTS if continuous else tuple(count for count in TAG2_APPROACH_COUNTS if count % 2 == 0)
    plan = RoutePose_PlanTag2ForwardTrial(checkpoint, nav.cfg, settings, candidate_counts=candidate_counts)
    distance = plan.get('distance_bounds_m')
    lateral = plan.get('lateral_bounds_m')
    clearance_needed = abs(plan.get('heading_to_correct_deg', 0.) or 0.) >= _TAG2_CLEARANCE_YAW_DEG
    min_projection = .75 if clearance_needed else .85
    if (distance is None or distance[1] - distance[0] > .025
            or lateral is None or lateral[1] - lateral[0] > .025
            or plan['max_heading_deviation_deg'] > 3. or plan['projection_bounds'][0] < min_projection
            or not live(plan['latest_stamp_ns'])):
        return None
    return plan


def _refresh_tag3_departure(robot, nav, profile, live):
    stance = robot.last_action_completed_ros_ns
    for attempt in range(2):
        pose, observation = _refresh_checkpoint_pose(robot, nav, profile, 3, quick_id=True,
            reuse_settled_cache=attempt == 0, boundary_first=attempt == 0)
        floor = (observation.get('scan') or {}).get('after_ns')
        if type(stance) is not int or type(floor) is not int or floor < stance:
            continue
        hint = RoutePose_TrialLateralHint(observation, nav.cfg, 3)
        if hint is None:
            hint = RoutePose_Tag3HeadingRecovery(observation, nav.cfg)
        if hint is not None and hint['latest_stamp_ns'] > stance and live(hint['latest_stamp_ns']):
            return pose, observation, hint
    return {}, {'detected': False, 'expected_id': 3, 'scan': {}}, None


def _tag3_heading_clearance_action(robot, leg, hint, *, reserved_actions):
    departure = (leg.get('configured_steps') or [{}])[0]
    if (departure.get('op') != 'strafe_right_once' or departure.get('repeat', 0) < 2
            or hint['projection_bounds'][0] < .75
            or robot.count + reserved_actions + 2 > robot.cfg['max_body_macros_per_journey']):
        return None
    left, right = hint['lateral_bounds_m']
    preferred = -1 if left > _TAG3_RIGHT_OFFSET_M else 1
    travel = departure.get('left_bias_stride_m', .08) + .02
    near, far = hint['distance_bounds_m']
    yaw, spread = hint['heading_deg'], hint['max_heading_deviation_deg']
    for direction in (preferred, -preferred):
        shifts = (0., *(direction * travel * sin(radians(angle)) for angle in (yaw - spread, yaw + spread)))
        if (min(left, left + direction * travel) >= -.30 and max(right, right + direction * travel) <= .16
                and near + min(shifts) >= .10 and far + max(shifts) <= .40):
            return 'strafe_right_once' if direction > 0 else 'strafe_left_once'
    return None


def _tag3_visual_departure(robot, nav, profile, previous_leg, leg, *, reserved_actions):
    """Use the current Tag 3 stance as the reference before right-striding toward the wall."""
    rebase = getattr(robot,'Route_RebaseTagHeading',None)
    live = getattr(nav.io,'visual_motion_ready',None)
    stance = robot.last_action_completed_ros_ns
    floor = (previous_leg.get('scan') or {}).get('after_ns')
    if rebase is None or live is None or type(stance) is not int or type(floor) is not int or floor < stance:
        return None, None
    hint = RoutePose_TrialLateralHint(previous_leg,nav.cfg,3)
    if hint is None or abs(hint.get('heading_deg', 0.)) >= _TAG3_CLEARANCE_YAW_DEG:
        hint = RoutePose_Tag3HeadingRecovery(previous_leg, nav.cfg) or hint
    if (hint is None or hint['latest_stamp_ns'] <= robot.last_action_completed_ros_ns
            or not live(hint['latest_stamp_ns'])):
        return None, None
    heading = leg['departure_heading'] = {'status':'prefer_verified_tag3',
                                          'visual_error_deg':hint['heading_deg'],'correction_actions':0}
    turn_from_tag = getattr(robot,'Route_TurnFromTagHeading',None)
    pose = refreshed = None
    direct_turn_fallback = False
    if abs(hint['heading_deg']) >= _TAG3_CLEARANCE_YAW_DEG:
        side_action = getattr(robot, 'corridor_side_action', None)
        goal = next((point for point in nav.cfg['legs']['outbound'] if point['id'] == 3), {})
        operation = (_tag3_heading_clearance_action(robot, leg, hint, reserved_actions=reserved_actions)
                     if (side_action is not None and turn_from_tag is not None
                         and goal.get('allow_strafe') is True and goal.get('allow_turn') is True
                         and 'distance_bounds_m' in hint) else None)
        if operation is not None:
            label = '右跨' if operation == 'strafe_right_once' else '左跨'
            _report_progress(robot, 'cmd0_departure_reobserve', 3, 3, leg=leg,
                             reason_zh=f'Tag 3 航向偏差较大，先{label}一步让位，再用新帧定转向。')
            if not live(hint['latest_stamp_ns']):
                heading.update(status='heading_clearance_expired', clearance_blocked=True)
                return None, None
            side_action(operation)
            leg['completed_actions'].append(operation)
            heading['clearance_action'] = operation
            heading['nominal_right_strides_used'] = int(operation == 'strafe_right_once')
            pose, refreshed, hint = _refresh_tag3_departure(robot, nav, profile, live)
            if hint is None:
                heading.update(status='clearance_pose_unconfirmed', clearance_blocked=True)
                return pose, refreshed
        else:
            # Leave room for the translation that can accompany a small turn.
            near, far = hint.get('distance_bounds_m', (0., 0.))
            left, right = hint.get('lateral_bounds_m', (0., 0.))
            direct_turn_fallback = (turn_from_tag is not None and goal.get('allow_turn') is True
                                    and robot.count + reserved_actions + 1 <= robot.cfg['max_body_macros_per_journey']
                                    and .15 <= near <= far <= .35 and -.24 <= left <= right <= .10
                                    and hint.get('projection_bounds', (0.,))[0] >= .75
                                    and abs(hint['heading_deg']) + hint.get('max_heading_deviation_deg', 30.) <= 26.
                                    and live(hint['latest_stamp_ns']))
            if not direct_turn_fallback:
                heading.update(status='heading_clearance_unavailable', clearance_blocked=True)
                return None, None
            reason = 'Tag 3 当前无法整步侧跨，先小转一次，再确认右跨起点。'
            heading['direct_turn_fallback_reason_zh'] = reason
            _report_progress(robot, 'cmd0_departure_reobserve', 3, 3, leg=leg, reason_zh=reason)
    if turn_from_tag is not None:
        cleared = 'clearance_action' in heading
        for _ in range(3 if cleared else 1):
            yaw = hint['heading_deg']
            turn = turn_from_tag(3, yaw, hint['latest_stamp_ns'], reserved_actions=reserved_actions)
            if turn is None:
                break
            heading['correction_actions'] += 1
            heading.setdefault('visual_turn', turn)
            if cleared:
                heading.setdefault('visual_turns', []).append(turn)
            pose, refreshed, hint = _refresh_tag3_departure(robot, nav, profile, live)
            turn['after_error_deg'] = hint['heading_deg'] if hint is not None else None
            heading['post_turn_pose_confirmed'] = hint is not None
            if hint is None:
                if cleared or direct_turn_fallback:
                    heading.update(status='post_turn_pose_unconfirmed', clearance_blocked=True)
                break
            if abs(hint['heading_deg']) < _TAG3_CLEARANCE_YAW_DEG or abs(yaw) - abs(hint['heading_deg']) < 1.:
                break
    if (heading.get('clearance_action') or direct_turn_fallback) and hint is not None and abs(hint['heading_deg']) >= _TAG3_CLEARANCE_YAW_DEG:
        heading.update(status='yaw_remains_outside_corridor', clearance_blocked=True)
    if (hint is not None and hint['latest_stamp_ns'] > robot.last_action_completed_ros_ns
            and live(hint['latest_stamp_ns'])):
        heading.update(residual_deg=hint['heading_deg'],
            magnetic_reference=rebase(3,hint['heading_deg'],hint['latest_stamp_ns']).value)
    if heading['correction_actions'] and heading.get('magnetic_reference') != 'applied':
        if nav.mag.anchor is not None:
            heading['magnetic_reference'] = 'prior_route_reference_retained'
        else:
            robot.prepare_heading_segment(fresh=True)
    return pose, refreshed


def _tag3_lateral_hint(tag2_leg, nav, deadband, refreshed=None, *, departure_turned=False):
    hint = RoutePose_TrialLateralHint(refreshed if refreshed is not None else tag2_leg, nav.cfg, 3)
    if hint is not None or departure_turned or (refreshed is not None and refreshed.get('detected')):
        return hint
    hint = tag2_leg.get('tag2_side_hint') if refreshed is None else None
    projected = tag2_leg.get('tag2_projected_side_hint')
    checks = tag2_leg.get('lateral_heading_checks') or ()
    right = 'strafe_right_once'
    strides = {native:count for (operation,count),native in SIDE_ACTIONS.items() if operation==right}
    strides[right] = 1
    if not checks or sum(strides.get(check['action'], 0) for check in checks) != 5:
        return None
    if projected is not None and tag2_leg.get('detected') is False:
        # Keep the estimate only across an uninterrupted traverse with
        # verified headings. Dropping a Tag must not hide an unmeasured turn.
        scan = (refreshed if refreshed is not None else tag2_leg).get('scan') or {}
        if (scan.get('camera_healthy') is not True or scan.get('contract_match') is not True
                or any(check.get('drift_deg') is None
                       or check.get('residual_deg') is None or abs(check['residual_deg']) > _TAG2_CARRIED_MAX_YAW_DEG
                       or check.get('correction_actions') or check.get('compensation_status') == 'applied'
                       for check in checks)):
            return None
        hint = projected
    elif (hint is None or deadband is None
          or any(check.get('residual_deg') is None or abs(check['residual_deg']) > deadband
                 or check.get('correction_actions') or check.get('compensation_status') == 'applied'
                 for check in checks)):
        return None
    if any(check['action'] not in strides for check in checks):
        return None
    left, right = hint['lateral_bounds_m']
    return dict(hint, lateral_bounds_m=(left-_TAG2_CARRIED_LATERAL_MARGIN_M,
                                      right+_TAG2_CARRIED_LATERAL_MARGIN_M))


def _tag3_left_bias_count(step, hint):
    """Add full strides to the near-centred route using a provisional right stride length."""
    stride = step.get('left_bias_stride_m')
    if stride is None or hint is None or hint['tag_id'] != 3:
        return None
    _, upper = hint['lateral_bounds_m']
    if upper >= -stride:
        return None
    extra = int(-upper / stride + .5)
    if step.get('nominal_last_small'):
        return min(_TAG3_TRIAL_MAX_RIGHT_STRIDES, step['repeat'] - 1 + extra)
    return min(_TAG3_TRIAL_MAX_RIGHT_STRIDES, 1 + step['repeat'] + extra)


def _tag3_trial_side_plan(step, hint, *, small_available):
    """Use the field-tested geometry when the current lateral offset is unknown."""
    nominal = step['repeat']
    operation = step['op']
    count = nominal - 1
    small = SMALL_SIDE_ACTIONS[operation] if small_available else None
    plan = {'status': 'field_baseline', 'tag_id': 3, 'operation': operation, 'nominal_count': nominal,
            'reason_zh': 'Tag 3 横向位姿未确认，沿用实测基线右跨。'}
    if hint is not None and hint.get('basis') == 'tag2_short_approach_projection':
        stride = step.get('left_bias_stride_m')
        plan['trial_lateral_hint'] = hint
        if stride is not None and hint['lateral_bounds_m'][1] < -stride:
            count, small = nominal, None
            offset_cm = -100 * median(hint['lateral_bounds_m'])
            plan.update(status='tag3_left_bias_fused',
                        reason_zh=f'未扫到 Tag 3，由 Tag 2 推算仍偏左约 {offset_cm:.1f} cm，最后一小跨换成大跨，共 {count} 大跨。')
        else:
            plan['reason_zh'] = 'Tag 2 的横偏估计不足以调整右跨，沿用基线距离。'
    elif hint is not None:
        left_count = _tag3_left_bias_count(step, hint)
        if left_count is not None and left_count >= count:
            count, small = left_count, None
            offset_cm = -100 * median(hint['lateral_bounds_m'])
            plan.update(status='tag3_left_bias_fused', trial_lateral_hint=hint,
                        reason_zh=f'Tag 3 偏左约 {offset_cm:.1f} cm，合入右跨计划，共 {count} 大跨。')
        elif hint['lateral_bounds_m'][0] > _TAG3_RIGHT_OFFSET_M and count > 1:
            if hint['tag_id'] == 2 or not small_available:
                count -= 1
            if hint['tag_id'] == 3:
                small = None
            plan.update(status='tag3_right_bias_fused', trial_lateral_hint=hint,
                        reason_zh=f'Tag 3 已偏右，合并修正后右大跨 {count} 步。')
        else:
            plan['reason_zh'] = 'Tag 3 横偏在基线容差内，采用原右跨距离。'
    return dict(plan, count=count + bool(small), large_count=count, small_operation=small)


def _trim_tag5_entry(robot, nav, profile, settings, checkpoint, delivery, *, reserved_actions):
    alignment = {'expected_id': 5, 'observations': [], 'completed_actions': []}
    delivery['entry_lateral_alignment'] = alignment
    ready = getattr(robot.backend, 'small_strafe_ready', lambda: False)
    live = getattr(nav.io, 'visual_motion_ready', None)
    lateral_action = getattr(robot, 'corridor_side_action', None)
    if (delivery.get('heading_before_advance') or {}).get('correction_actions'):
        _, checkpoint = _refresh_checkpoint_pose(robot, nav, profile, 5)

    previous_error = None
    for index in range(_TAG5_ENTRY_TRIMS + 1):
        _revalidate_heading(robot, nav)
        has_small = not getattr(robot, 'magnetic_route_only', False) and ready()
        plan = RoutePose_PlanTag5EntryLateral(checkpoint, nav.cfg, settings, small_ready=has_small)
        if index == 0 and plan['operation'] and live is not None and not live(plan['latest_stamp_ns']):
            _, checkpoint = _refresh_checkpoint_pose(robot, nav, profile, 5)
            plan = RoutePose_PlanTag5EntryLateral(checkpoint, nav.cfg, settings, small_ready=has_small)
        alignment['observations'].append(plan)
        error = plan.get('map_y_error_m')
        movement = plan['operation']
        if movement is None:
            return
        if previous_error is not None and abs(error) >= abs(previous_error) - 0.005:
            plan.update(status='deferred', reason='last Tag 5 correction did not reduce lateral error')
            return
        if index == _TAG5_ENTRY_TRIMS or robot.count + reserved_actions + 1 > robot.cfg['max_body_macros_per_journey']:
            plan.update(status='deferred', reason='Tag 5 correction budget exhausted')
            return
        if live is None or not live(plan['latest_stamp_ns']):
            plan.update(status='deferred', reason='Tag 5 lateral observation expired')
            return
        if index == 0:
            _report_progress(robot, 'cmd0_departure_reobserve', 5, 5)
        if lateral_action is None:
            robot.action(movement)
            alignment['completed_actions'].append(movement)
        else:
            before = CorridorTrial_HeadingSnapshot(robot.last_heading_report or {}, after=True)
            lateral_action(movement)
            alignment['completed_actions'].append(movement)
            CorridorTrial_CheckLateralHeading(robot, nav, movement, before, robot.last_heading_report or {},
                                   alignment, reserved_actions, 5)
        previous_error = error
        _, checkpoint = _refresh_checkpoint_pose(robot, nav, profile, 5)


def _tag5_passage_plan(robot, nav, settings, checkpoint, *, approach_distance_m=.40):
    small_ready = getattr(robot.backend, 'small_strafe_ready', lambda: False)
    plan = RoutePose_PlanTag5Passage(
        checkpoint, nav.cfg, settings, current_group_distance_m=approach_distance_m,
        small_ready=not getattr(robot, 'magnetic_route_only', False) and small_ready())
    ready = getattr(nav.io, 'visual_motion_ready', None)
    if plan['geometry_valid'] and (ready is None or not ready(plan['latest_stamp_ns'])):
        plan.update(geometry_valid=False, ready_for_forward=False, operation=None,
                    reason='Tag 5 passage observation expired')
    return plan


def _tag5_partial_forward_probe(robot, nav, settings, leg, approach):
    """Borrow two planned steps while clearly outside and lined up with the entrance."""
    if not approach or approach[0]['steps'] < 4:
        return None
    trimmed_groups = (_forward_actions(approach[0]['steps']-2, continuous=True, legacy=False),) + tuple(
        _forward_actions(step['steps'], continuous=True, legacy=False) for step in approach[1:])
    if not all(trimmed_groups) or robot.count + sum(map(len,trimmed_groups)) + 2 > robot.cfg['max_body_macros_per_journey']:
        return None
    geometry = _corridor_partial_geometry(robot, nav, leg, 5, fresh_only=True)
    if geometry is None:
        return None
    leg['tag5_partial_pose'] = geometry
    left, right = geometry['lateral_bounds_m']
    target = settings.get('target_lateral_m',0.)
    near, far = geometry['distance_bounds_m']
    angle = geometry['max_abs_heading_deg']
    limit = settings.get('lateral_clearance_m',.075)
    swept_side = max(abs(left-target),abs(right-target)) + .125*sin(radians(angle))
    if (not geometry['consistent'] or not geometry['latest_fresh'] or geometry['samples'] < 2
            or not .30 <= near <= far <= .50 or angle > 8.
            or geometry['projection_bounds'][0] <= .85 or swept_side > limit - .015):
        return None
    recovery = {'operation':'carry_forward2','completed_steps':0,
                'reason_zh':'Tag 5 有一致的偏远读数，先走进场计划中的两步再重扫。'}
    leg['tag5_partial_probe'] = recovery
    _report_progress(robot, 'cmd0_corridor_wait', 4, 5, leg=leg, reason_zh=recovery['reason_zh'])
    robot.action('carry_forward2', monitor_only_as='carry_forward2')
    recovery['completed_steps'] = 2
    observation = {'expected_id':5,'observation_windows':0,'detected':False}
    verified, reason = EntranceTrial_ObserveCheckpoint(robot, nav, observation, lambda: None,
        quick_id=True, reuse_settled_cache=True, advisory_windows=2, miss_policy='hold_load_for_placement')
    recovery['status'] = verified.value
    return verified, reason, observation


def _tag5_wider_side_recovery(plan, settings, operation):
    """Select one full step that still fits the projected envelope outside the passage."""
    bounds = plan.get('projected_lateral_bounds_m')
    if not plan['geometry_valid'] or plan.get('entering') or bounds is None:
        return None
    limit = settings.get('lateral_clearance_m', .075)
    side = next((full for full, small in SMALL_SIDE_ACTIONS.items() if small == operation), None)
    if operation is None and plan.get('lateral_recovery_allowed') is True:
        if bounds[1] > limit:
            side = 'strafe_left_once'
        elif bounds[0] < -limit:
            side = 'strafe_right_once'
    if side is None:
        return None
    near, far = bounds if side == 'strafe_left_once' else (-bounds[1], -bounds[0])
    travel = _LARGE_STRAFE_TRAVEL_UPPER_M
    max_yaw = plan['max_abs_heading_deg']
    heading_bounds = plan.get('heading_bounds_deg', (-max_yaw, max_yaw))
    direction = -1 if side == 'strafe_left_once' else 1
    # Depth grows when a side step points away from the marker. Also allow
    # zero movement, since a completed cross-step may barely shift the robot.
    depth_shift = min(0., *(direction * travel * sin(radians(yaw)) for yaw in heading_bounds))
    if (far > limit and near - travel >= -limit + .01
            and plan['depth_bounds_m'][0] + depth_shift >= _TAG5_OUTSIDE_SIDE_DEPTH_M):
        return side
    return None


def _tag5_retreat_clearance(plan, settings):
    """Check a single two-step retreat from the existing outside stance."""
    if not plan['geometry_valid']:
        return False
    depth = plan.get('depth_bounds_m')
    lateral = plan.get('lateral_bounds_m')
    yaw = plan.get('max_abs_heading_deg')
    if (depth is None or lateral is None or yaw is None
            or not .09 <= depth[0] <= depth[1] < _TAG5_OUTSIDE_RECOVERY_DEPTH_M or abs(yaw) > 25.):
        return False
    heading_bounds = plan.get('heading_bounds_deg', (-yaw, yaw))
    # A backward step changes lateral position opposite to the body heading.
    shifts = [0., *(-_TAG5_RETREAT_TRAVEL_UPPER_M * sin(radians(angle)) for angle in heading_bounds)]
    limit = settings.get('lateral_clearance_m', .075) - .005
    return lateral[0] + min(shifts) >= -limit and lateral[1] + max(shifts) <= limit


def _tag5_heading_clearance(robot, nav, plan, settings):
    """Find a one-step move toward the gate center with clearance for a later turn."""
    bounds = plan.get('lateral_bounds_m')
    depth = plan.get('depth_bounds_m')
    yaw = plan.get('heading_deg')
    spread = plan.get('max_abs_heading_deg')
    if (not plan['geometry_valid'] or bounds is None or depth is None or yaw is None or spread is None
            or not settings.get('max_heading_deg', 12.) < abs(yaw) <= 35.):
        return None
    goal = next((point for point in nav.cfg['legs']['outbound'] if point['id'] == 5), {})
    live = getattr(nav.io, 'visual_motion_ready', None)
    if goal.get('allow_strafe') is not True or live is None or not live(plan['latest_stamp_ns']):
        return None
    side = 'strafe_left_once' if bounds[0] > 0 else 'strafe_right_once'
    near, far = bounds if side == 'strafe_left_once' else (-bounds[1], -bounds[0])
    if near <= 0 or (abs(yaw) >= _TAG5_CLEARANCE_YAW_DEG
                     and side != ('strafe_left_once' if yaw < 0 else 'strafe_right_once')):
        return None
    small_ready = (not getattr(robot, 'magnetic_route_only', False)
                   and getattr(robot.backend, 'small_strafe_ready', lambda: False)())
    moves = [(side, _LARGE_STRAFE_TRAVEL_UPPER_M, _TAG5_OUTSIDE_SIDE_DEPTH_M)]
    if small_ready:
        moves.insert(0, (SMALL_SIDE_ACTIONS[side], _TAG5_SMALL_SIDE_TRAVEL_UPPER_M, _TAG5_TIGHT_SIDE_DEPTH_M))
    heading_bounds = plan.get('heading_bounds_deg', (-spread, spread))
    direction = -1 if side == 'strafe_left_once' else 1
    for action, travel, depth_floor in moves:
        # Even the largest response must not move farther from the center or past the lip.
        depth_shift = min(0., *(direction * travel * sin(radians(angle)) for angle in heading_bounds))
        if (travel + _TAG5_SIDE_PROGRESS_M <= near + far
                and depth[0] + depth_shift >= depth_floor):
            return action
    return None


def _align_tag5_entry(robot, nav, profile, settings, checkpoint, alignment, *,
                      reserved_actions, approach_distance_m):
    """Use Tag 5 for normal alignment while the robot is still outside the base."""
    last_side = None
    ineffective = set()
    idle_windows = 0
    partial_recheck_stance = None
    while True:
        robot.check()
        reserve = reserved_actions + int('entry_retreat' in alignment)
        plan = _tag5_passage_plan(robot, nav, settings, checkpoint, approach_distance_m=approach_distance_m)
        if not plan['geometry_valid']:
            partial = _corridor_partial_geometry(robot, nav, checkpoint, 5)
            if partial is not None:
                plan['partial_pose'] = partial
        records = alignment['observations']
        records.append(plan)
        if len(records) > 24:
            del records[8]
        turns = alignment['visual_turns']
        if turns and 'after_heading_deg' not in turns[-1] and plan['geometry_valid']:
            turns[-1]['after_heading_deg'] = plan['heading_deg']
        if plan['ready_for_forward']:
            alignment.update(status='aligned', ready_for_forward=True)
            return plan
        clearance = None
        if 'heading_clearance' not in alignment and robot.count + reserve + 2 <= robot.cfg['max_body_macros_per_journey']:
            clearance = _tag5_heading_clearance(robot, nav, plan, settings)
        if ((clearance is None or abs(plan['heading_deg']) < _TAG5_CLEARANCE_YAW_DEG)
                and _try_tag5_visual_turn(robot, plan, turns, reserved_actions=reserve, settings=settings)):
            last_side = None
            ineffective.clear()
            idle_windows = 0
            _, checkpoint = _refresh_checkpoint_pose(robot, nav, profile, 5, reuse_settled_cache=True)
            continue
        operation = clearance or plan['operation']
        recovery_from = None
        wider = _tag5_wider_side_recovery(plan, settings, operation) if clearance is None else None
        if operation is not None or wider is not None:
            error = abs(plan.get('alignment_error_m', plan['lateral_error_m']))
            if last_side is not None and error >= last_side[1] - _TAG5_SIDE_PROGRESS_M:
                ineffective.add(last_side[0])
            else:
                ineffective.clear()
            if wider and wider not in ineffective and (operation is None or operation in ineffective
                    or abs(plan['lateral_error_m']) >= settings.get('large_move_error_m', .10)):
                recovery_from, operation = operation or 'no_small_step_available', wider
            if operation in ineffective or (operation is None and wider in ineffective):
                operation = None
                plan['reason'] = 'last Tag 5 strafe did not reduce lateral error'
            if operation is not None and robot.count + reserve + 1 > robot.cfg['max_body_macros_per_journey']:
                operation = None
                plan['reason'] = 'Tag 5 side adjustment budget exhausted'
        if operation is not None:
            if operation == clearance:
                label = '左跨' if '_left' in operation else '右跨'
                reason = f"Tag 5 航向 {plan['heading_deg']:+.1f}° · 先{label}一步让位，再用新帧纠偏。"
                _report_progress(robot, 'cmd0_tag5_entry_wait', 5, 5, reason_zh=reason)
                alignment['heading_clearance'] = {'operation': operation, 'reason_zh': reason,
                                                  'before_heading_deg': plan['heading_deg'],
                                                  'before_lateral_m': plan['lateral_error_m']}
            robot.corridor_side_action(operation)
            alignment['completed_actions'].append(operation)
            if recovery_from is not None:
                plan.update(side_recovery_from=recovery_from, side_recovery_operation=operation)
            last_side = operation, error
            idle_windows = 0
        elif ('entry_retreat' not in alignment and _tag5_retreat_clearance(plan,settings)
              and robot.count + reserve + 2 <= robot.cfg['max_body_macros_per_journey']):
            retreat = {'operation':'cmd0_carry_backward2','completed_steps':0,
                       'reason_zh':'Tag 5 门前纠偏余量不足，先退两步拉开距离，再用新位姿对门。'}
            alignment['entry_retreat'] = retreat
            _report_progress(robot,'cmd0_tag5_entry_wait',5,5,reason_zh=retreat['reason_zh'])
            robot.action(retreat['operation'],monitor_only_as=retreat['operation'])
            alignment['completed_actions'].append(retreat['operation'])
            retreat['completed_steps'] = 2
            last_side = None
            ineffective.clear()
            idle_windows = 0
        else:
            partial = plan.get('partial_pose')
            stance = robot.last_action_completed_ros_ns
            if (partial is not None and partial['consistent'] and partial['latest_fresh']
                    and 0 < partial['samples'] < nav.cfg['min_frames'] and stance != partial_recheck_stance):
                partial_recheck_stance = stance
                plan['stationary_partial_recheck'] = True
                _report_progress(robot, 'cmd0_tag5_entry_wait', 5, 5,
                                 reason_zh='Tag 5 已有少量一致新帧，正在同一站姿补齐帧数。')
                _, checkpoint = _refresh_checkpoint_pose(robot, nav, profile, 5, reuse_settled_cache=True,
                                                           quick_id=True, advisory_windows=2)
                continue
            idle_windows += 1
            if idle_windows >= _TAG5_IDLE_WINDOWS:
                reason = plan['reason']
                detail_zh = {
                    'last Tag 5 strafe did not reduce lateral error': '上次侧跨未减小横偏，未找到有通行余量的替代动作',
                    'Tag 5 side adjustment budget exhausted': '侧跨纠偏动作余量不足',
                    'Tag 5 projected path exceeds the lateral passage tolerance': '预计直行超出入口横向余量',
                    'Tag 5 heading outside the passage tolerance': '航向超出入口容差，未确认可执行的纠偏动作',
                }.get(reason)
                if detail_zh is None:
                    detail_zh = '当前位姿未确认可通行' if plan['geometry_valid'] else '未获得可信的门口位姿'
                alignment.update(status='stalled', stalled_reason=reason,
                                 stalled_reason_zh=f'Tag 5 {detail_zh}', ready_for_forward=False,
                                 consecutive_idle_windows=idle_windows)
                return None
            reason = '等待 Tag 5 的新位姿'
            if plan['geometry_valid']:
                reason = f"航向 {plan['heading_deg']:+.1f}° · 横向 {plan['lateral_error_m']*100:+.1f} cm"
            elif plan.get('partial_pose'):
                reason = 'Tag 5 有部分帧，门口位姿待复扫。'
            _report_progress(robot, 'cmd0_tag5_entry_wait', 5, 5, reason_zh=reason)
        _, checkpoint = _refresh_checkpoint_pose(robot, nav, profile, 5,
                                                   reuse_settled_cache=idle_windows == 0, quick_id=True)


def _try_tag5_visual_turn(robot, plan, turns, *, reserved_actions, settings=None):
    turn = getattr(robot, 'Corridor_TurnAtTag5', None)
    if (turn is None or not plan['geometry_valid']
            or robot.count + reserved_actions + 1 > robot.cfg['max_body_macros_per_journey']
            or abs(plan['lateral_error_m']) > .075
            or not 12. < abs(plan['heading_deg']) <= 35.):
        return False
    yaw = plan['heading_deg']
    depth = plan['depth_bounds_m'][0]
    bounds = plan.get('lateral_bounds_m')
    limit = (settings or {}).get('lateral_clearance_m', .075)
    if (depth < _TAG5_OUTSIDE_RECOVERY_DEPTH_M
            and (depth < _TAG5_OUTSIDE_SIDE_DEPTH_M or abs(yaw) >= _TAG5_CLEARANCE_YAW_DEG
                 or bounds is None or max(map(abs, bounds)) > limit - _TAG5_TIGHT_TURN_MARGIN_M)):
        return False
    if turns:
        last = turns[-1]
        before, after = last['before_heading_deg'], last.get('after_heading_deg')
        if (after is None or abs(before) - abs(after) < _TAG5_TURN_PROGRESS_DEG
                or abs(before) - abs(yaw) < _TAG5_TURN_PROGRESS_DEG):
            plan['turn_reason'] = 'last Tag 5 visual turn did not reduce heading error'
            return False
    action = 'single_micro_left' if yaw > 0 else 'single_micro_right'
    if not turn(action, plan['latest_stamp_ns']):
        return False
    turns.append({'action': action,'before_heading_deg': yaw})
    return True


def _tag5_approach_distance(action):
    if action == 'carry_forward':
        return 8 * _TAG5_STEP_UPPER_M
    return int(action.rsplit('forward',1)[1]) * _TAG5_STEP_UPPER_M


def _tag5_continuous_entry(robot, nav, settings, plan):
    """Check the whole eight-step envelope before merging the two four-step groups."""
    heading_bounds = plan.get('heading_bounds_deg')
    lateral = plan.get('lateral_bounds_m')
    projected = plan.get('projected_lateral_bounds_m')
    stamp = plan.get('latest_stamp_ns')
    live = getattr(nav.io, 'visual_motion_ready', None)
    if (not plan.get('geometry_valid') or not plan['ready_for_forward'] or plan.get('entering')
            or heading_bounds is None or lateral is None or projected is None
            or max(map(abs, heading_bounds)) > 8. or type(stamp) is not int or live is None or not live(stamp)):
        return None
    if robot.load_kind == 'double':
        pair_ops = getattr(robot.backend, 'mission_pair_ops', lambda: ())()
        if 'double_carry_forward' not in pair_ops:
            return None
    # Use an extra stride of travel and a lateral margin for the uninterrupted gait.
    distance = _tag5_approach_distance('carry_forward') + _TAG5_STEP_UPPER_M
    shifts = [0., *(distance * sin(radians(yaw)) for yaw in heading_bounds)]
    bounds = min(projected[0], lateral[0] + min(shifts)), max(projected[1], lateral[1] + max(shifts))
    limit = settings.get('lateral_clearance_m', .075) - .015
    if max(map(abs, bounds)) > limit:
        return None
    return {'action': 'carry_forward', 'latest_stamp_ns': stamp, 'steps': 8,
            'projected_lateral_bounds_m': bounds,
            'reason_zh': 'Tag 5 新位姿确认整段进场有余量，合并为连续前进 8 步。'}


def _probe_tag3_from_midpoint(robot, nav, leg, strides, action=None):
    if getattr(robot, 'boundary_visual_observation', None) is not None:
        observation = _corridor_boundary_checkpoint(robot, nav, 3, action)
        if observation is None:
            return
        observed = EntranceTrialResult.CHECKPOINT_DETECTED
    else:
        observation = {'expected_id': 3, 'observation_windows': 0, 'detected': False}
        observed, _ = EntranceTrial_ObserveCheckpoint(
            robot, nav, observation, lambda: None, quick_id=True, single_window=True, reuse_settled_cache=True)
    scan = observation.get('scan') or {}
    leg['tag3_midpoint_observation'] = {
        'status': observed.value, 'seen_tags': scan.get('observed_ids', []), 'right_strides': strides,
        'fresh_tag3_frames': scan.get('samples', 0), 'pose_used_for_next_leg': False,
        'raw_pose': _checkpoint_raw_pose(scan, 3, nav.cfg['min_frames']),
        'frame_diagnostics': observation.get('quick_scan_windows', [])}


def CorridorTrial_Run(robot, next_segment, *, entry_alignment, extra_home_steps=0):
    """Run the estimated suffix in order, recording intermediate marks that are missed."""
    entry_detected = next_segment.get('detected') is True
    result = {'status': CorridorTrialResult.ROUTE_UNAVAILABLE.value, 'reason': 'preparing corridor trial',
              'checkpoint': 2 if entry_detected else 1, 'estimated_checkpoint': 2,
              'expected_id': 5, 'detected': False, 'position_confirmed': False, 'missing_checkpoints': [],
              'route_evidence': 'estimated_trial', 'expected_load': robot.load_kind,
              'entry_pose': entry_alignment.get('route_pose'), 'legs': [],
              'release_protocol_completed': False, 'physical_release_confirmed': False}
    robot.events.append({'corridor_trial': result})

    def set_status(status, reason):
        result.update(status=status.value, reason=reason)
        return result

    robot.check()
    entry_miss = next_segment.get('checkpoint_not_seen') or {}
    entry_completed_unseen = (next_segment.get('continue_estimated') is True
                              and entry_miss.get('tag_id') == _CHECKPOINTS[0]
                              and entry_miss.get('policy') == 'continue_estimated')
    carrying_pair = robot.load_kind == 'double' and getattr(robot, 'pair_route_enabled', False)
    if ((robot.load_kind != 'single' and not carrying_pair) or next_segment.get('expected_id') != _CHECKPOINTS[0]
            or next_segment.get('forward_protocol_complete') is False
            or not (entry_detected or entry_completed_unseen)):
        return set_status(CorridorTrialResult.ROUTE_UNAVAILABLE, 'supported carry load and a completed Tag 2 leg required')
    if entry_completed_unseen:
        result['missing_checkpoints'].append(entry_miss)
    nav = robot.ensure_navigation_io()
    if nav is None or [goal['id'] for goal in nav.cfg['legs']['outbound']][:5] != [1, *_CHECKPOINTS]:
        return set_status(CorridorTrialResult.ROUTE_UNAVAILABLE, 'configured outbound Tag order differs from corridor trial')

    chain = robot.cfg.get('route_steps', {}).get('corridor')
    try:
        chain = CorridorTrial_ValidateChain(CorridorTrial_DefaultChain() if chain is None else chain,
                                            extra_home_steps=extra_home_steps)
    except ValueError as exc:
        return set_status(CorridorTrialResult.ROUTE_UNAVAILABLE, str(exc))

    profile = RoutePose_Load(robot.project)
    pose = _measure(robot, nav, profile, next_segment) if entry_detected else {}
    previous_scan = next_segment.get('scan') if entry_detected else None
    small_ready = getattr(robot.backend, 'small_strafe_ready', None)
    continuous = Cmd0Motion_ContinuousReady(robot.backend)
    right5_probe = getattr(robot.backend, 'cmd0_right5_ready', None)
    right5_ready = continuous and right5_probe is not None and right5_probe() is True
    legacy = not continuous and Cmd0Motion_LegacyReady(robot.backend)
    tag4_trial = robot.cfg.get('route_steps', {}).get('tag4_forward_trial') or {}
    tag4_departure = _tag4_departure_settings(tag4_trial)
    use_tag4_trial = tag4_trial.get('enabled') is True and continuous
    tag5_alignment = robot.cfg.get('route_steps', {}).get('tag5_entry_lateral') or {}
    use_tag5_passage = (tag5_alignment.get('enabled') is True and tag5_alignment.get('verify_passage') is True
                       and continuous and not getattr(robot.backend,'is_simulation',False))
    entry_actions = (_forward_actions(chain[_DROP][0]['steps'], continuous=True, legacy=False)
                     if use_tag5_passage else ())
    entry_distance = _tag5_approach_distance(entry_actions[0]) if entry_actions else .40
    remaining_route = [name for _, _, name in _LEGS]
    if continuous:
        remaining_route.append(_DROP)
    required_macros = {name: _nominal_macro_count(chain[name], continuous=continuous, legacy=legacy)
                       for name in remaining_route}
    after_leg = sum(required_macros.values())
    tag5_probe_steps = 0
    for previous_id, expected_id, name in _LEGS:
        after_leg -= required_macros[name]
        steps = chain[name]
        if steps:
            _revalidate_heading(robot, nav)
        departure_op = steps[0]['op'] if steps else None
        tag2_side_departure = previous_id == 2 and departure_op in _SIDE_ACTIONS
        verify_tag2 = (tag2_side_departure and next_segment.get('forward_protocol_complete') is True
                       and (robot.cfg.get('route_steps', {}).get('tag2_forward_trial') or {}).get('enabled') is True
                       and not getattr(robot.backend,'is_simulation',False))
        leg = {'from_tag': previous_id, 'expected_id': expected_id, 'detected': False,
               'observation_windows': 0, 'completed_actions': [], 'start_pose': pose,
               'last_confirmed_tag': result['checkpoint'], 'departure_tag_confirmed': result['checkpoint'] == previous_id,
               'next_motion_axis': ('stationary_scan' if departure_op is None
                                    else 'lateral' if departure_op in _SIDE_ACTIONS else 'forward'),
               'scan': None, 'side_plan': None, 'configured_steps': steps}
        if previous_id == 2:
            leg['entry_completed_actions'] = list(next_segment.get('completed_actions') or ())
            leg['entry_target_steps'] = next_segment.get('target_steps')
            leg['entry_planned_steps'] = next_segment.get('planned_steps')
            leg['entry_completed_steps'] = next_segment.get('completed_steps')
            leg['entry_forward_plan'] = next_segment.get('forward_plan')
        raw_pose = _checkpoint_raw_pose(previous_scan, previous_id, nav.cfg['min_frames'])
        if raw_pose is not None:
            leg['previous_tag_pose_raw'] = raw_pose
        result['legs'].append(leg)
        forward_actions = {step['steps']: _forward_actions(step['steps'], continuous=continuous, legacy=legacy)
                           for step in steps if step['op'] == _FORWARD_GROUP}
        for distance, actions in forward_actions.items():
            if not actions:
                return set_status(CorridorTrialResult.ROUTE_UNAVAILABLE,
                    'Tag {} read; verified {}-step forward capability required before the next leg'.format(previous_id, distance))
        if not steps:
            leg['departure_advice'] = {'status': 'stationary_scan',
                                       'reason': 'read the next Tag from the completed leg endpoint'}
        elif leg['departure_tag_confirmed'] and use_tag5_passage and expected_id == 5:
            advice = RoutePose_AdviseDeparture(pose, profile, previous_id, advisory_heading=True)
            leg['departure_advice'] = advice
            if advice['status'] == 'outside_departure_envelope':
                return set_status(CorridorTrialResult.DEPARTURE_UNSAFE,
                                  'Tag 4 起步位姿超出已标定的通行范围。')
        elif leg['departure_tag_confirmed']:
            pose, fault = _departure_pose(robot, nav, profile, previous_id, pose, leg)
            if fault:
                return set_status(*fault)
        else:
            leg['departure_advice'] = {'status': 'estimated_due_to_missed_tag',
                                       'reason': 'retain nominal motion and the existing magnetic reference'}
        if use_tag4_trial and previous_id == 4 and departure_op in _SIDE_ACTIONS:
            if tag4_departure.get('left_departure_reference'):
                leg['tag4_departure_reference'] = tag4_departure['left_departure_reference']
            pose, strafe_ready = _trim_tag4_before_left(robot, nav, profile, pose, leg, tag4_departure,
                checkpoint=result['legs'][-2],
                reserved_actions=required_macros[name] + after_leg, monitor_only=use_tag5_passage and expected_id == 5)
            tag4_trim = leg['tag4_forward_trim']
            if not strafe_ready:
                return set_status(CorridorTrialResult.DEPARTURE_UNSAFE, tag4_trim.get(
                    'retreat_reason_zh', 'Tag 4 进近复查后仍未确认左跨起点。'))
            if tag4_trim.get('depth_confirmed') or tag4_trim.get('approach_confirmed'):
                leg.update(departure_tag_confirmed=True,last_confirmed_tag=4)
                result['checkpoint'] = 4
        lateral_action = getattr(robot, 'corridor_side_action', None)
        departure_residual = None
        tag3_refresh = None
        tag2_trial_plan = _tag2_current_visual_plan(robot, nav, next_segment, continuous=continuous) if verify_tag2 else None
        if previous_id == 3 and departure_op in _SIDE_ACTIONS and leg['departure_tag_confirmed']:
            refreshed_pose, tag3_refresh = _tag3_visual_departure(robot,nav,profile,result['legs'][-2],leg,
                reserved_actions=required_macros[name]+after_leg)
            if refreshed_pose is not None:
                pose = refreshed_pose
            if (leg.get('departure_heading') or {}).get('clearance_blocked'):
                return set_status(CorridorTrialResult.DEPARTURE_UNSAFE,
                                  'Tag 3 大偏航恢复后未确认右跨余量，保留持物。')
        if tag2_trial_plan is not None:
            leg['departure_heading'] = {'status': 'prefer_verified_tag2',
                                        'visual_error_deg': tag2_trial_plan['heading_deg'], 'correction_actions': 0}
        elif (lateral_action is not None and departure_op in _SIDE_ACTIONS
                and 'departure_heading' not in leg
                and nav.cfg['post_action_heading'].get('execute')
                and not getattr(robot, 'magnetic_route_only', False)):
            robot.last_heading_report = None
            if use_tag5_passage and expected_id == 5:
                robot.monitor_heading('corridor_departure', allow_correction=False)
            else:
                robot.monitor_heading('corridor_departure')
            heading = robot.last_heading_report or {}
            first = (heading.get('correction_steps') or (heading,))[0]
            leg['departure_heading'] = {'error_deg': first.get('error_deg'),
                'residual_deg': heading.get('after_error_deg', heading.get('error_deg')),
                'correction_actions': heading.get('correction_actions', 0), 'status': heading.get('status')}
            departure_residual = CorridorTrial_HeadingSnapshot(heading, after=True, load=robot.load_kind)
            if leg['departure_heading']['correction_actions']:
                if previous_id == 2:
                    leg.pop('tag2_side_hint',None)
                if (previous_id in (2, 3) and not verify_tag2 and pose.get('position_usable') is True
                        or previous_id == 3 and steps[0].get('left_bias_stride_m') and leg['departure_tag_confirmed']):
                    pose, observation = _refresh_checkpoint_pose(robot, nav, profile, previous_id)
                    leg['departure_pose_after_turn'] = {'detected': observation['detected'],
                                                        'position_usable': pose.get('position_usable', False)}
                    if previous_id == 3:
                        tag3_refresh = observation
                else:
                    pose = {}
        if tag2_side_departure:
            checkpoint = next_segment
            heading_changed = bool(leg.get('departure_correction')
                                   or (leg.get('departure_heading') or {}).get('correction_actions'))
            if verify_tag2:
                checkpoint, ready = _tag2_recheck_before_strafe(
                    robot, nav, profile, leg, checkpoint, heading_changed=heading_changed,
                    departure_heading=departure_residual, reserved_actions=required_macros[name]+after_leg)
                if not ready:
                    if entry_completed_unseen:
                        entry_miss['held_before_departure'] = True
                    return set_status(CorridorTrialResult.DEPARTURE_UNSAFE,
                                      'Tag 2 右跨起点的前后距离仍未确认，保留持物。')
                if checkpoint:
                    pose = _measure(robot, nav, profile, checkpoint) if checkpoint is not next_segment else pose
                    if entry_completed_unseen:
                        entry_miss['recovered_before_departure'] = True
                        result['missing_checkpoints'].remove(entry_miss)
                    if checkpoint is not next_segment:
                        recovered = getattr(robot, 'route_checkpoint_observed', None)
                        if recovered is not None:
                            recovered(2)
                    result['checkpoint'] = 2
                    leg.update(departure_tag_confirmed=True,last_confirmed_tag=2)
                    latest_raw = _checkpoint_raw_pose(checkpoint.get('scan'), 2, nav.cfg['min_frames'])
                    if latest_raw is not None:
                        leg['previous_tag_pose_raw'] = latest_raw
            if checkpoint and leg['departure_tag_confirmed']:
                if not heading_changed or verify_tag2:
                    leg['tag2_side_hint'] = RoutePose_TrialLateralHint(checkpoint, nav.cfg, 2)
                pose, ready = _trim_tag2_forward(robot, nav, profile, pose, leg, checkpoint,
                    continuous=continuous, reserved_actions=required_macros[name]+after_leg,
                    require_verified=verify_tag2, trial_plan=tag2_trial_plan if checkpoint is next_segment else None)
                if not ready:
                    trim = leg.get('tag2_forward_trim') or {}
                    if trim.get('lateral_clearance_blocked'):
                        offset_cm = 100 * median(trim['lateral_bounds_m'])
                        return set_status(CorridorTrialResult.DEPARTURE_UNSAFE,
                                          f'Tag 2 实测横偏 {offset_cm:+.1f} cm，未确认可继续右跨，保留持物。')
                    return set_status(CorridorTrialResult.DEPARTURE_UNSAFE,
                                      trim.get('last_approach_reason_zh', 'Tag 2 尚未到达右跨起点，保留持物。'))
                trim = leg.get('tag2_forward_trim') or {}
                if trim.get('protocol_completed') or trim.get('heading_recovery'):
                    departure_residual = CorridorTrial_HeadingSnapshot(
                        getattr(robot,'last_heading_report',None) or {}, after=True, load=robot.load_kind)
        if (previous_id == 3 and steps and steps[0].get('left_bias_stride_m')
                and tag3_refresh is None and (leg.get('departure_correction') or {}).get('correction_executed')):
            pose, tag3_refresh = _refresh_checkpoint_pose(robot, nav, profile, 3)
        tag3_needs_new_view = False
        if previous_id == 3 and departure_op in _SIDE_ACTIONS and not leg['departure_tag_confirmed']:
            incoming = result['legs'][-2]
            severe = any(check.get('large_drift') for check in incoming.get('lateral_heading_checks', ()))
            saw_midway = (incoming.get('tag3_midpoint_observation') or {}).get('status') == 'checkpoint_detected'
            turned_at_end = (leg.get('departure_heading') or {}).get('correction_actions',0) > 0
            tag3_needs_new_view = severe or (saw_midway and turned_at_end)
        if tag3_needs_new_view:
            _report_progress(robot, 'cmd0_departure_reobserve', 3, 3, leg=leg,
                             reason_zh='右跨后 Tag 3 位置未确认，纠偏后先复扫。')
            pose, tag3_refresh = _refresh_checkpoint_pose(robot, nav, profile, 3,
                                                          quick_id=True, reuse_settled_cache=True)
            leg['tag3_recheck_after_yaw'] = {'detected':tag3_refresh['detected'],
                                             'windows':tag3_refresh.get('quick_scan_windows', [])}
            if tag3_refresh['detected']:
                notify = getattr(robot,'route_checkpoint_observed',None)
                if notify is not None:
                    notify(3)
                result['checkpoint'] = 3
                leg.update(departure_tag_confirmed=True,last_confirmed_tag=3)
                miss = incoming.get('checkpoint_not_seen')
                if miss:
                    miss['recovered_after_heading'] = True
                tag3_needs_new_view = False
        leg['start_pose'] = pose
        planned = []
        for index, step in enumerate(steps):
            op = step['op']
            if op == _FORWARD_GROUP:
                planned.append((op, forward_actions[step['steps']], None, False))
                continue
            nominal = count = step['repeat']
            small = None
            calibrated = False
            if index == 0 and leg['departure_tag_confirmed']:
                # The current checkpoint captures accumulated tracking error.
                # Prefer changing its planned full stride count over adding a nudge.
                side_plan = RoutePose_PlanSideCount(pose, profile, previous_id, op, nominal)
                if (side_plan['count'] == nominal and pose.get('position_usable') is True
                        and small_ready is not None and small_ready()):
                    side_plan = RoutePose_PlanSmallSide(pose, profile, previous_id, op, nominal)
                count = side_plan['count']
                small = side_plan.get('small_operation')
                calibrated = side_plan.get('calibrated_pose_used') is True
                if calibrated or count != nominal or small:
                    motion_ready = getattr(nav.io, 'visual_motion_ready', None)
                    if not motion_ready or not motion_ready(pose.get('latest_stamp_ns', 0)):
                        count, small, calibrated = nominal, None, False
                        side_plan.update(status='nominal', count=count, small_operation=None,
                            calibrated_pose_used=False,
                            reason='current lateral pose expired before dispatch; retain estimated count')
                leg['side_plan'] = side_plan
            restore_large_step = bool(small) and count < nominal
            if ('unverified_repeat' in step and not calibrated and count == nominal and not small):
                count = step['unverified_repeat']
                prior_reason = (leg['side_plan'] or {}).get('reason', 'expected departure Tag not confirmed')
                side_plan = {'status': 'configured_unverified_pose', 'tag_id': previous_id,
                    'operation': op, 'nominal_count': nominal, 'count': count, 'reason': prior_reason}
                skip_extra = False
                full_stride_trim = False
                if index == 0 and previous_id == 3 and op == 'strafe_right_once' and count > nominal:
                    tag3_approach = result['legs'][-2].get('lateral_heading_checks') or [{}]
                    incoming = tag3_approach[-1]
                    incoming_residual = incoming.get('residual_deg')
                    if incoming_residual is None:
                        incoming_residual = incoming.get('last_verified_error_deg', incoming.get('error_after_strafe_deg'))
                    recent = (incoming_residual, departure_residual[0] if departure_residual else None)
                    deadband = nav.cfg['post_action_heading'].get('deadband_deg')
                    hint = _tag3_lateral_hint(result['legs'][-2], nav, deadband, tag3_refresh)
                    left_count = _tag3_left_bias_count(step, hint)
                    if left_count is not None:
                        ready = getattr(nav.io, 'visual_motion_ready', None)
                        if ready is None or not ready(hint['latest_stamp_ns']):
                            _, tag3_refresh = _refresh_checkpoint_pose(robot, nav, profile, 3)
                            hint = RoutePose_TrialLateralHint(tag3_refresh, nav.cfg, 3)
                            left_count = (_tag3_left_bias_count(step, hint)
                                          if hint and ready and ready(hint['latest_stamp_ns']) else None)
                    full_stride_trim = left_count is not None
                    latest_residual = recent[1] if recent[1] is not None else recent[0]
                    skip_extra = (not full_stride_trim and deadband is not None
                                  and latest_residual is not None and abs(latest_residual) > deadband)
                    if full_stride_trim:
                        count = max(count, left_count)
                        offset_cm = -100 * median(hint['lateral_bounds_m'])
                        side_plan.update(status='tag3_left_bias_fused', count=count, trial_lateral_hint=hint,
                            reason='repeatable Tag 3 left offset increases the full right strafe plan',
                            reason_zh=f'Tag 3 偏左约 {offset_cm:.1f} cm，合入右跨计划，完成 {count} 大跨后再直行。')
                    elif skip_extra:
                        count = nominal
                        side_plan.update(status='tag3_unverified_extra_omitted', count=count,
                            reason='Tag 3 lateral pose unavailable after verified heading drift; retain nominal right strafe',
                            reason_zh='Tag 3 横向位姿未确认且进段航向偏差仍大，沿用基线右跨，省略额外侧跨。',
                            incoming_heading_residual_deg=recent[0], departure_heading_residual_deg=recent[1])
                    elif not carrying_pair and count>nominal+1:
                        if (hint is not None and hint.get('basis') != 'tag2_short_approach_projection'
                                and hint['lateral_bounds_m'][0] > _TAG3_RIGHT_OFFSET_M):
                            count -= 1
                            side_plan.update(count=count, trial_lateral_hint=hint,
                                reason='stationary lateral evidence favors one fewer full right stride')
                if (not skip_extra and not full_stride_trim and step.get('unverified_last_small')
                        and small_ready is not None and small_ready()
                        and not getattr(robot, 'magnetic_route_only', False)):
                    count -= 1
                    small = SMALL_SIDE_ACTIONS[op]
                    restore_large_step = True
                side_plan.update(large_count=count, small_operation=small)
                leg['side_plan'] = side_plan
            split_right5 = (right5_ready and index == 0 and (op, count) == ('strafe_right_once', 5)
                            and departure_residual is not None
                            and abs(departure_residual[0]) > nav.cfg['post_action_heading']['deadband_deg'])
            if split_right5:
                leg['native_right5_split'] = {'reason': 'departure_heading_residual',
                                              'residual_deg': departure_residual[0], 'groups': [3, 2]}
            monitor_every = step.get('monitor_every')
            tag2_settings = robot.cfg.get('route_steps', {}).get('tag2_forward_trial') or {}
            if previous_id == 2 and op == 'strafe_right_once' and tag2_settings.get('enabled') is True:
                hint = leg.get('tag2_side_hint') or {}
                visual_error = leg.get('tag2_visual_heading_deg', hint.get('heading_deg'))
                residual = departure_residual[0] if departure_residual else None
                initial_error = (leg.get('departure_heading') or {}).get('error_deg')
                deadband = nav.cfg['post_action_heading'].get('deadband_deg')
                lateral = hint.get('lateral_bounds_m')
                close_to_target = ('distance_m' in hint and 'target_distance_m' in tag2_settings
                                   and hint['distance_m'] < tag2_settings['target_distance_m'])
                off_center = lateral is not None and max(map(abs, lateral)) >= _TAG2_CAUTION_LATERAL_M
                if (leg.get('tag2_narrow_departure') or not leg['departure_tag_confirmed'] or (deadband is not None and
                        any(value is not None and abs(value)>deadband for value in (visual_error, residual, initial_error)))
                        or close_to_target or off_center or leg.get('tag2_close_reframe')):
                    monitor_every = min(monitor_every or 2, 2)
                    leg['tag2_cautious_strafes'] = {'max_group': monitor_every, 'visual_heading_deg': visual_error,
                                                   'departure_heading_error_deg': residual,
                                                   'arrival_heading_error_deg': initial_error,
                                                   'departure_distance_m': hint.get('distance_m'),
                                                   'lateral_bounds_m': lateral}
            if tag3_needs_new_view and op == 'strafe_right_once':
                monitor_every = min(monitor_every or 2, 2)
                leg['tag3_recheck_after_yaw']['max_strafe_group'] = monitor_every
            if previous_id == 3 and step.get('nominal_last_small') and not calibrated and count == nominal and not small:
                deadband = nav.cfg['post_action_heading'].get('deadband_deg')
                previous_leg = result['legs'][-2]
                departure_turned = bool((leg.get('departure_heading') or {}).get('correction_actions'))
                hint = _tag3_lateral_hint(previous_leg, nav, deadband, tag3_refresh,
                                          departure_turned=departure_turned)
                live = getattr(nav.io, 'visual_motion_ready', None)
                if hint and hint['tag_id'] == 3 and (not live or not live(hint['latest_stamp_ns'])):
                    hint = None
                can_small = (small_ready is not None and small_ready()
                             and not getattr(robot, 'magnetic_route_only', False))
                side_plan = _tag3_trial_side_plan(step, hint, small_available=can_small)
                count, small = side_plan['large_count'], side_plan['small_operation']
                restore_large_step = False
                leg['side_plan'] = side_plan
                incoming = (previous_leg.get('lateral_heading_checks') or [{}])[-1]
                incoming_residual = incoming.get('residual_deg')
                if incoming_residual is None:
                    incoming_residual = incoming.get('last_verified_error_deg', incoming.get('error_after_strafe_deg'))
                side_plan['incoming_heading_residual_deg'] = incoming_residual
                partial_tag3 = (previous_leg.get('checkpoint_not_seen') or {}).get('cause') == 'insufficient_frames'
                if (side_plan['status'] == 'tag3_left_bias_fused' and hint
                        and hint.get('basis') == 'tag2_short_approach_projection'):
                    monitor_every = min(monitor_every or 3, 3)
                elif hint is None and (carrying_pair or partial_tag3):
                    monitor_every = min(monitor_every or 2, 2)
                    if partial_tag3:
                        side_plan['partial_frames_use_baseline_groups'] = True
            if previous_id == 2 and index == 0 and op == 'strafe_right_once':
                count -= (leg.get('tag2_lateral_recovery') or {}).get('nominal_strides_used', 0)
            if previous_id == 3 and index == 0 and op == 'strafe_right_once':
                used = (leg.get('departure_heading') or {}).get('nominal_right_strides_used', 0)
                if used and count > 1:
                    count -= used
                    leg['side_plan'] = dict(leg['side_plan'] or {}, clearance_right_strides_used=used,
                                           remaining_large_strides=count)
            actions = CorridorTrial_GroupStrafes(op, count, continuous=continuous, right5_ready=right5_ready and not split_right5,
                                    monitor_every=monitor_every)
            if previous_id == 2 and op == 'strafe_right_once' and count > 1 and leg.get('tag2_narrow_departure'):
                # A single closed stride gives the first feedback boundary sooner.
                actions = (op, *CorridorTrial_GroupStrafes(op, count - 1, continuous=continuous, monitor_every=2))
                leg['tag2_cautious_strafes']['first_group'] = 1
            if previous_id == 4 and index == 0 and op == 'strafe_left_once' and leg.get('tag4_near_left_start'):
                actions = (op, *CorridorTrial_GroupStrafes(op, count - 1, continuous=continuous, monitor_every=2))
            planned.append((op, actions, small, restore_large_step))
        pending_actions = sum(len(actions) + bool(small) for _, actions, small, _ in planned)
        if robot.count + pending_actions > robot.cfg['max_body_macros_per_journey']:
            return set_status(CorridorTrialResult.ROUTE_UNAVAILABLE, 'insufficient body macro budget for corridor leg')

        if planned:
            set_status(CorridorTrialResult.MOVING_TO_CHECKPOINT, 'executing estimated Tag {} to Tag {} segment'.format(previous_id, expected_id))
            _report_progress(robot, 'cmd0_corridor_leg', previous_id, expected_id, leg=leg)
        previous_op = None
        omit_tag4_forward = False
        tag4_disruption = None
        tag5_handoff = None
        tag5_continuation = None
        checkpoint_action=getattr(robot,'route_checkpoint_action',None)
        for index, (op, actions, small, replaced_full) in enumerate(planned):
            guarded_forward = False
            tag4_fused_heading = False
            if previous_op in _SIDE_ACTIONS and op == _FORWARD_GROUP:
                robot.settle_since_last_completed(nav.cfg['settle_s'])
            if op in _SIDE_ACTIONS:
                omit_tag4_forward = False
            if use_tag4_trial and expected_id == 4 and op == _FORWARD_GROUP:
                if omit_tag4_forward:
                    pending_actions -= len(actions)
                    continue
                count = steps[index]['steps']
                if count in (2,4,8):
                    magnetic_fallback = None
                    group_start = leg.get('completed_forward_steps',0)
                    _revalidate_heading(robot, nav)
                    plan = _tag4_position_plan(robot, nav, profile, tag4_trial, approach_steps=count, boundary_first=True)
                    leg.setdefault('tag4_approach', []).append(plan)
                    ambiguous = ('conflicting_frames','outside_heading_or_lateral_range')
                    if (plan.get('partial_advice') in ambiguous and not plan.get('camera_contract_fault')
                            and _tag4_observed_clearance_problem(plan, nav) is None
                            and not _tag4_guarded_forward_ready(nav,plan)):
                        plan = _tag4_position_plan(robot, nav, profile, tag4_trial, approach_steps=count)
                        leg['tag4_approach'].append(plan)
                    if plan.get('camera_contract_fault'):
                        return set_status(CorridorTrialResult.CAMERA_UNAVAILABLE,
                                          'Tag 4 的扫码来源或相机配置异常。')
                    partial = plan.get('partial_pose') or {}
                    depth = partial.get('distance_bounds_m')
                    close_glimpse = (partial.get('consistent') and partial.get('latest_fresh') and depth
                                      and .02 <= depth[0] <= depth[1] <=
                                      tag4_trial['target_distance_m']+tag4_trial['deadband_m'])
                    observed_issue = _tag4_observed_clearance_problem(plan, nav)
                    if not (tag4_disruption or observed_issue) and (plan.get('partial_advice') == 'near' or close_glimpse):
                        leg['tag4_partial_approach'] = partial
                        omit_tag4_forward = True
                        pending_actions -= len(actions)
                        continue
                    if tag4_disruption is None:
                        tag4_disruption = observed_issue
                        if tag4_disruption:
                            leg['tag4_forward_disruption'] = tag4_disruption
                            _report_progress(robot, 'cmd0_corridor_scan', 3, 4, leg=leg,
                                             reason_zh=tag4_disruption['reason_zh'])
                    if tag4_disruption:
                        plan = _tag4_recover_forward_pose(robot, nav, profile, tag4_trial, plan, tag4_disruption,
                                                         leg=leg, approach_steps=count, reserved_actions=pending_actions + after_leg)
                        if plan.get('camera_contract_fault'):
                            return set_status(CorridorTrialResult.CAMERA_UNAVAILABLE,
                                              'Tag 4 的扫码来源或相机配置异常。')
                        guarded_forward = not plan['distance_verified'] and _tag4_guarded_forward_ready(nav,plan)
                        if not plan['distance_verified'] and not guarded_forward:
                            return set_status(CorridorTrialResult.DEPARTURE_UNSAFE,
                                              tag4_disruption['blocked_reason_zh'])
                        if guarded_forward:
                            tag4_disruption['short_forward_authorized'] = True
                        tag4_disruption = None
                    elif not plan['distance_verified'] and plan.get('partial_advice') in ambiguous:
                        guarded_forward = _tag4_guarded_forward_ready(nav,plan)
                        if not guarded_forward:
                            heading = CorridorTrial_HeadingSnapshot(getattr(robot,'last_heading_report',None) or {},
                                                                   after=True,load=robot.load_kind)
                            magnetic_fallback = {
                                'completed_steps':leg.get('completed_forward_steps',0),
                                'planned_steps':sum(step['steps'] for step in steps if step['op']==_FORWARD_GROUP),
                                'heading_deg':heading[0] if heading else None,
                                'next_steps':count,
                                'reason_zh':'Tag 4 位姿暂不可用，沿用已修正的磁航向和计划直行。' if heading else
                                            'Tag 4 位姿暂不可用，沿计划前进到下一动作边界再检查。'}
                            leg.setdefault('tag4_magnetic_fallback', []).append(magnetic_fallback)
                            _report_progress(robot,'cmd0_corridor_leg',3,4,leg=leg,
                                             reason_zh=magnetic_fallback['reason_zh'])
                    advance_steps = plan['steps'] if plan['distance_verified'] else count
                    range_confirmed = plan['distance_verified']
                    tag4_fused_heading = bool(range_confirmed and plan.get('heading_fused_into_forward'))
                    if tag4_fused_heading:
                        leg.setdefault('tag4_fused_forward', []).append(plan['heading_fused_into_forward'])
                        _report_progress(robot, 'cmd0_corridor_leg', 3, 4, leg=leg,
                            reason_zh=f'Tag 4 已核对整组 {advance_steps} 步的通行余量，继续直行。')
                    boundaries = leg.get('heading_correction_boundaries') or []
                    side_plan = leg['side_plan'] or {}
                    incoming_error = side_plan.get('incoming_heading_residual_deg')
                    side_heading_lost = (getattr(robot, 'magnetic_route_only', False)
                                         and incoming_error is not None
                                         and abs(incoming_error) >= _LATERAL_LARGE_DRIFT_DEG)
                    room_to_split = robot.count + pending_actions + after_leg + 1 <= robot.cfg['max_body_macros_per_journey']
                    if guarded_forward:
                        leg['tag4_partial_recovery'] = {
                            'status':'guarded_forward','heading_deg':plan['heading_deg'],
                            'distance_m':plan['distance_m'],'completed_steps':0,
                            'reason_zh':'Tag 4 位姿已确认有短步余量，先走两步再复查。'}
                        leg['tag4_guarded_forward'] = True
                    if count >= 4 and (plan.get('partial_advice') == 'far' or guarded_forward):
                        advance_steps = 2
                        if room_to_split:
                            advance_steps = _tag4_recheck_after_short_advance(
                                robot, nav, profile, tag4_trial, leg, {}, partial_far=not guarded_forward,
                                guarded_forward=guarded_forward,
                                reserved_actions=pending_actions+after_leg)
                            split = leg['tag4_approach_split']
                            range_confirmed = split['tag4_distance_verified']
                            if guarded_forward:
                                leg['tag4_partial_recovery']['completed_steps'] = 2
                            if split.get('partial_near'):
                                leg['tag4_partial_approach'] = leg['tag4_approach'][-1].get('partial_pose')
                                omit_tag4_forward = True
                    elif count >= 4 and not plan['distance_verified'] and side_heading_lost and room_to_split:
                        advance_steps = _tag4_recheck_after_short_advance(
                            robot, nav, profile, tag4_trial, leg, {}, side_heading_lost=True,
                            reserved_actions=pending_actions+after_leg)
                        range_confirmed = leg['tag4_approach_split']['tag4_distance_verified']
                    elif count == 4 and not plan['distance_verified'] and previous_op == _FORWARD_GROUP and boundaries:
                        previous_heading = boundaries[-1].get('monitor_report') or {}
                        initial = CorridorTrial_HeadingSnapshot(previous_heading, after=False, load=robot.load_kind)
                        deadband = nav.cfg['post_action_heading'].get('deadband_deg')
                        if (previous_heading.get('correction_actions', 0) > 0 and initial and deadband is not None
                                and abs(initial[0]) > deadband
                                and room_to_split):
                            if magnetic_fallback is not None:
                                magnetic_fallback['next_steps'] = 2
                            advance_steps = _tag4_recheck_after_short_advance(
                                robot, nav, profile, tag4_trial, leg, previous_heading,
                                magnetic_fallback=magnetic_fallback is not None,
                                reserved_actions=pending_actions+after_leg)
                            range_confirmed = leg['tag4_approach_split']['tag4_distance_verified']
                    if advance_steps != count:
                        pending_actions -= len(actions)
                        if count == 8 and advance_steps + leg.get('completed_forward_steps',0) - group_start < count:
                            leg['tag4_guarded_forward'] = True
                        if not advance_steps:
                            last_disruption = leg.get('tag4_forward_disruption')
                            if last_disruption and not (last_disruption.get('reacquired')
                                                        or last_disruption.get('short_forward_authorized')):
                                return set_status(CorridorTrialResult.DEPARTURE_UNSAFE,
                                                  'Tag 4 短前进后明显偏航，已重扫仍无法确认通道，保留持物。')
                            omit_tag4_forward = omit_tag4_forward or range_confirmed
                            continue
                        actions = _forward_actions(advance_steps, continuous=True, legacy=False)
                        pending_actions += len(actions)
            deferred_side_heading = None
            for action_index, action in enumerate(actions):
                _revalidate_heading(robot, nav)
                if index == 0 and action_index == 0 and leg.get('tag4_near_left_start'):
                    live = nav.io.visual_motion_ready
                    if not live(leg['tag4_near_left_start']['latest_stamp_ns']):
                        latest = _tag4_position_plan(robot, nav, profile, tag4_departure, boundary_first=True)
                        leg['tag4_near_left_recheck'] = latest
                        if not (_tag4_left_start_ready(latest, tag4_departure) or _tag4_authorize_near_left(
                                robot, nav, tag4_departure, latest, leg, reserved_actions=pending_actions + after_leg - 1)):
                            return set_status(CorridorTrialResult.DEPARTURE_UNSAFE,
                                              'Tag 4 左跨前新位姿未确认足够的前后余量。')
                if tag5_continuation is not None:
                    midpoint, checkpoint = tag5_continuation
                    tag5_continuation = None
                    if not nav.io.visual_motion_ready(midpoint['latest_stamp_ns']):
                        midpoint['decision'] = 'replace_remaining_strafes'
                        midpoint['group_rejection'] = 'Tag 5 observed pose expired before dispatch'
                        tag5_handoff = checkpoint
                        break
                is_boundary = index == len(planned)-1 and action_index == len(actions)-1 and not small
                pending_actions -= 1
                last_event = robot.events[-1] if robot.events else None
                previous_report = getattr(robot, 'last_heading_report', None)
                if lateral_action is not None and op in _SIDE_ACTIONS:
                    before = CorridorTrial_HeadingSnapshot(robot.last_heading_report or {}, after=True, load=robot.load_kind)
                    lateral_action(action)
                    leg['completed_actions'].append(action)
                    gate_approach = use_tag5_passage and expected_id == 5
                    can_read_boundary = getattr(robot, 'boundary_visual_observation', None) is not None
                    tag3_midpoint = previous_id == 2 and op == 'strafe_right_once' and not is_boundary
                    if tag3_midpoint and can_read_boundary:
                        sizes = {name: cycles for (operation, cycles), name in SIDE_ACTIONS.items() if operation == op}
                        right_strides = sum(sizes.get(name, name == op) for name in leg['completed_actions'])
                        _probe_tag3_from_midpoint(robot, nav, leg, right_strides, action)
                    midpoint = None
                    if gate_approach and not is_boundary:
                        if not can_read_boundary:
                            _, checkpoint = _refresh_checkpoint_pose(robot, nav, profile, 5, reuse_settled_cache=True)
                        else:
                            checkpoint = _corridor_boundary_checkpoint(robot, nav, 5, action)
                        if checkpoint is not None:
                            midpoint = _tag5_passage_plan(robot, nav, tag5_alignment, checkpoint,
                                                         approach_distance_m=entry_distance)
                            leg['tag5_midpoint'] = midpoint
                            leg.setdefault('tag5_side_windows', []).append(midpoint)
                    small_strafe_follows = (expected_id == 4 and op == 'strafe_right_once'
                                            and action_index == len(actions) - 1
                                            and small == 'single_small_strafe_right' and small_ready is not None
                                            and not getattr(robot, 'magnetic_route_only', False) and small_ready())
                    heading_result = CorridorTrial_CheckLateralHeading(robot, nav, action, before,
                        robot.last_heading_report or {}, leg, pending_actions + after_leg,
                        expected_id if is_boundary else None,
                        allow_turn=not gate_approach or (not is_boundary and
                                                        (midpoint is None or not midpoint['geometry_valid'])),
                        allow_clearance_translation=not gate_approach and not steps[index].get('left_bias_stride_m'),
                        small_strafe_follows=small_strafe_follows)
                    if heading_result is CorridorLateralHeadingResult.DEFERRED_TO_SMALL_STRAFE:
                        deferred_side_heading = leg['lateral_heading_checks'][-1]
                    if (heading_result is CorridorLateralHeadingResult.RECHECK_CLEARANCE
                            and _tag2_recheck_severe_lateral_drift(robot,nav,profile,leg,
                                more_right=op == 'strafe_right_once' and not is_boundary) is None):
                        return set_status(CorridorTrialResult.DEPARTURE_UNSAFE,
                                          'Tag 2 → 3 侧跨后航向突变，重扫未确认通行余量，保留持物。')
                    if (tag3_midpoint and not can_read_boundary
                            and (action == 'cmd0_strafe_right3'
                                 or leg.get('tag2_cautious_strafes') and action_index == len(actions)-2)):
                        sizes = {name: cycles for (operation, cycles), name in SIDE_ACTIONS.items() if operation == op}
                        _probe_tag3_from_midpoint(robot, nav, leg,
                            sum(sizes.get(name, 1) for name in actions[:action_index+1]), action)
                    elif not gate_approach and not is_boundary and not tag3_midpoint:
                        _corridor_boundary_checkpoint(robot, nav, expected_id, action)
                    if midpoint is not None:
                        if midpoint['geometry_valid']:
                            remaining_group = actions[action_index+1:action_index+2]
                            if not remaining_group and not small and index+1 < len(planned):
                                remaining_group = planned[index+1][1][:1]
                            group = _tag5_continuous_left_group(midpoint, remaining_group, robot.load_kind)
                            if group is not None:
                                midpoint.update(decision='keep_continuous_left_group', **group)
                                tag5_continuation = midpoint, checkpoint
                                continue
                            tag5_handoff = checkpoint
                            midpoint['decision'] = 'replace_remaining_strafes'
                            break
                        midpoint['decision'] = 'retain_estimated_group'
                    continue
                if tag4_fused_heading or (use_tag5_passage and expected_id == 5):
                    robot.action(action, monitor_only_as=action)
                elif checkpoint_action is not None and is_boundary:
                    checkpoint_action(action,expected_id)
                else:
                    robot.action(action)
                    if not is_boundary:
                        _corridor_boundary_checkpoint(robot, nav, expected_id, action)
                leg['completed_actions'].append(action)
                if guarded_forward:
                    leg['tag4_partial_recovery']['completed_steps'] += Cmd0Motion_ForwardStepCount((action,))
                if use_tag4_trial and expected_id == 4 and op == _FORWARD_GROUP:
                    leg['completed_forward_steps'] = (leg.get('completed_forward_steps',0)
                                                       + Cmd0Motion_ForwardStepCount((action,)))
                    tag4_disruption = _tag4_forward_disruption(robot, last_event, previous_report, leg)
                    if tag4_disruption and action_index + 1 < len(actions):
                        pending_actions -= len(actions) - action_index - 1
                        break
            if tag5_handoff:
                break
            if small:
                pending_actions -= 1
                _revalidate_heading(robot, nav)
                if small_ready() and not getattr(robot, 'magnetic_route_only', False):
                    if lateral_action is not None and expected_id in (4, 5):
                        before = CorridorTrial_HeadingSnapshot(robot.last_heading_report or {}, after=True, load=robot.load_kind)
                        lateral_action(small)
                        if expected_id == 4:
                            CorridorTrial_CheckLateralHeading(robot, nav, small, before,
                                robot.last_heading_report or {}, leg, pending_actions + after_leg, None,
                                allow_clearance_translation=False)
                    else:
                        robot.action(small)
                    leg['completed_actions'].append(small)
                    _corridor_boundary_checkpoint(robot, nav, expected_id, small)
                else:
                    if deferred_side_heading is not None:
                        robot.corridor_recover_heading(None)
                        report = robot.last_heading_report or {}
                        settled = CorridorTrial_HeadingSnapshot(report, after=True, load=robot.load_kind)
                        deferred_side_heading.update(
                            compensation_status='small_strafe_unavailable_heading_rechecked',
                            residual_deg=settled[0] if settled else None,
                            correction_actions=report.get('correction_actions', 0),
                            heading_status=report.get('status'))
                    # The current stage remains usable with its original full
                    # cycle count if the optional small motion becomes unavailable.
                    if replaced_full:
                        if use_tag5_passage and expected_id == 5:
                            robot.corridor_side_action(op)
                        else:
                            robot.action(op)
                        leg['completed_actions'].append(op)
                    side_plan = leg['side_plan']
                    if steps[index].get('nominal_last_small') and not replaced_full:
                        side_plan.update(count=side_plan['large_count'], small_operation=None,
                                         reason_zh='小跨不可用，本段只执行基线大跨。')
                    elif side_plan['status'] == 'configured_unverified_pose':
                        side_plan.update(large_count=side_plan['count'], small_operation=None)
                    else:
                        side_plan.update(status='nominal', count=side_plan['nominal_count'], small_operation=None,
                            reason='small strafe capability unavailable before dispatch; retain estimated count')
            if op == _FORWARD_GROUP and index+1 < len(planned) and planned[index+1][0] == _FORWARD_GROUP:
                # Each native action already runs the heading monitor on completion.
                leg.setdefault('heading_correction_boundaries', []).append({
                    'after_action': actions[-1], 'before_action': planned[index+1][1][0],
                    'monitor_report': getattr(robot, 'last_heading_report', None)})
            previous_op = op

        if tag4_disruption:
            plan = _tag4_position_plan(robot, nav, profile, tag4_trial, approach_steps=2)
            plan = _tag4_recover_forward_pose(robot, nav, profile, tag4_trial, plan, tag4_disruption,
                                             leg=leg, approach_steps=2, reserved_actions=after_leg)
            if not plan['distance_verified']:
                return set_status(CorridorTrialResult.DEPARTURE_UNSAFE, tag4_disruption['blocked_reason_zh'])

        set_status(CorridorTrialResult.WAITING_FOR_CHECKPOINT, 'stationary reacquisition of expected Tag {}'.format(expected_id))
        _report_progress(robot, 'cmd0_corridor_scan', previous_id, expected_id, leg=leg)
        boundary_arrival = tag5_handoff or _corridor_boundary_checkpoint(robot, nav, expected_id)
        if boundary_arrival:
            leg.update(boundary_arrival)
            observed = EntranceTrialResult.CHECKPOINT_DETECTED
            reason = f'fresh Tag {expected_id} confirmed at an action boundary'
            notify = getattr(robot,'route_checkpoint_observed',None)
            if notify is not None:
                notify(expected_id)
        else:
            observed, reason = EntranceTrial_ObserveCheckpoint(robot, nav, leg,
                lambda: _report_progress(robot, 'cmd0_corridor_wait', previous_id, expected_id, leg=leg),
                quick_id=expected_id==3, reuse_settled_cache=True,
                miss_policy='hold_load_for_placement' if expected_id == _CHECKPOINTS[-1] else 'continue_estimated',
                next_stationary_id=5 if expected_id == 4 and not chain[_TAG5_LEG] else None,
                advisory_windows=2 if expected_id == 3 else None)
        if expected_id == 5 and use_tag5_passage and observed is EntranceTrialResult.CHECKPOINT_NOT_SEEN:
            probe = _tag5_partial_forward_probe(robot, nav, tag5_alignment, leg, chain[_DROP][:-1])
            if probe is not None:
                observed, reason, observation = probe
                tag5_probe_steps = 2
                leg['tag5_partial_probe']['initial_miss'] = leg.pop('checkpoint_not_seen')
                observation['observation_windows'] += leg['observation_windows']
                leg.update(observation)
        if observed is EntranceTrialResult.CAMERA_UNAVAILABLE:
            return set_status(CorridorTrialResult.CAMERA_UNAVAILABLE, reason)
        result['estimated_checkpoint'] = expected_id
        if observed is EntranceTrialResult.CHECKPOINT_NOT_SEEN:
            result['missing_checkpoints'].append(leg['checkpoint_not_seen'])
            if expected_id == _CHECKPOINTS[-1]:
                return set_status(CorridorTrialResult.CHECKPOINT_NOT_SEEN, reason)
            previous_scan, pose = None, {}
            continue
        result['checkpoint'] = expected_id
        previous_scan = leg['scan']
        pose = _measure(robot, nav, profile, leg)
        leg['arrived_pose'] = pose

    result['detected'] = True
    approach = chain[_DROP][:-1]
    forward_groups = [_forward_actions(step['steps']-(tag5_probe_steps if index==0 else 0),
                                       continuous=continuous, legacy=False) for index,step in enumerate(approach)]
    if not continuous or not all(forward_groups):
        return set_status(CorridorTrialResult.CHECKPOINT_DETECTED,
                          'Tag 5 read; verified base approach and release capability unavailable; holding load')
    base_actions = tuple(action for actions in forward_groups for action in actions)
    if robot.count + len(base_actions) + 1 > robot.cfg['max_body_macros_per_journey']:
        return set_status(CorridorTrialResult.ROUTE_UNAVAILABLE, 'insufficient body macro budget for base approach and release')
    delivery = {'completed_actions': ['carry_forward2'] if tag5_probe_steps else [],
                'approach_steps': sum(step['steps'] for step in approach),
                'configured_steps': chain[_DROP]}
    if extra_home_steps:
        delivery['extra_home_steps'] = extra_home_steps
    result['delivery'] = delivery
    checkpoint = result['legs'][-1]
    if use_tag5_passage:
        advice = RoutePose_AdviseDeparture(pose, profile, 5, advisory_heading=True)
        delivery['departure_advice'] = advice
        if advice['status'] == 'outside_departure_envelope':
            return set_status(CorridorTrialResult.DEPARTURE_UNSAFE, 'Tag 5 起步位姿超出已标定的通行范围。')
        alignment = {'expected_id': 5, 'observations': [], 'completed_actions': [], 'visual_turns': [],
                     'ready_for_forward': False,
                     'entry_group_steps': Cmd0Motion_ForwardStepCount((base_actions[0],))}
        delivery['entry_lateral_alignment'] = alignment
        _revalidate_heading(robot, nav)
        set_status(CorridorTrialResult.WAITING_FOR_CHECKPOINT, '已扫到 Tag 5，正在纠正进场方向。')
        _report_progress(robot, 'cmd0_tag5_entry_wait', 5, 5)
        # Check the first forward group and the Tag crossing at the door. The
        # rest of the delivery remains an in-base approach with heading monitoring.
        aligned = _align_tag5_entry(robot, nav, profile, tag5_alignment, checkpoint, alignment,
                                    reserved_actions=len(base_actions)+1,
                                    approach_distance_m=_tag5_approach_distance(base_actions[0]))
        if aligned is None:
            reason = alignment.get('stalled_reason_zh', 'Tag 5 入口纠偏尚未完成')
            return set_status(CorridorTrialResult.DEPARTURE_UNSAFE, f'{reason}，保留方块。')
        if alignment.get('entry_retreat',{}).get('completed_steps') == 2:
            base_actions += ('carry_forward2',)
            delivery['approach_steps'] += 2
    else:
        pose, fault = _departure_pose(robot, nav, profile, 5, pose, delivery)
        if fault:
            return set_status(*fault)
        _revalidate_heading(robot, nav)
        monitor_heading = getattr(robot, 'monitor_heading', None)
        if monitor_heading is not None:
            monitor_heading(base_actions[0])
            delivery['heading_before_advance'] = robot.last_heading_report
        if (not carrying_pair and tag5_alignment.get('enabled') is True
                and delivery['departure_advice']['status'] == 'nominal_unverified'):
            _trim_tag5_entry(robot, nav, profile, tag5_alignment, checkpoint, delivery,
                             reserved_actions=len(base_actions) + 1)
    set_status(CorridorTrialResult.MOVING_TO_CHECKPOINT, 'Tag 5 已定位，开始进入大本营。')
    _report_progress(robot, 'cmd0_delivery_advance', 5, 5)
    _revalidate_heading(robot, nav)
    if use_tag5_passage and base_actions == ('carry_forward4', 'carry_forward4'):
        continuous_entry = _tag5_continuous_entry(robot, nav, tag5_alignment, aligned)
        if continuous_entry is not None:
            base_actions = (continuous_entry['action'],)
            alignment['entry_group_steps'] = continuous_entry['steps']
            delivery['continuous_entry'] = continuous_entry
            _report_progress(robot, 'cmd0_delivery_advance', 5, 5, reason_zh=continuous_entry['reason_zh'])
    for index, action in enumerate(base_actions):
        if index:
            _revalidate_heading(robot, nav)
        if use_tag5_passage:
            robot.action(action, monitor_only_as=action)
        else:
            robot.action(action)
        delivery['completed_actions'].append(action)
        if index + 1 < len(base_actions):
            delivery.setdefault('heading_correction_boundaries', []).append({
                'after_action': action, 'before_action': base_actions[index+1],
                'monitor_report': getattr(robot, 'last_heading_report', None)})
    _report_progress(robot, 'cmd0_delivery_release', 5, 5)
    robot.action(PLACE_ACTION)
    delivery['completed_actions'].append(PLACE_ACTION)
    delivery['release_operation'] = 'place_double' if carrying_pair else PLACE_ACTION
    result.update(release_protocol_completed=True, expected_load=robot.load_kind)
    return set_status(CorridorTrialResult.DELIVERY_COMPLETED,
                      '已扫到 Tag 5，进场后的双抱放物及回站姿动作已完成。' if carrying_pair
                      else 'single-object release protocol completed after Tag 5 and base approach')


def _tag5_continuous_left_group(plan, remaining, load):
    """Keep a planned group while its estimated travel stays short of the center."""
    if load != 'single' or plan['ready_for_forward'] or plan['operation'] != 'strafe_left_once':
        return None
    if len(remaining) != 1:
        return None
    strides = next((count for count in (2, 3)
                    if remaining[0] == SIDE_ACTIONS['strafe_left_once', count]), None)
    if strides is None or plan['max_abs_heading_deg'] > 30.:
        return None
    # The 20:28 single-load trial reduced x by 21 cm across four full
    # steps and one small step. Use 7.5 cm per full step as a trial upper
    # estimate, plus a separate 2.5 cm reserve before reaching the center.
    travel_upper = strides * _LARGE_STRAFE_TRAVEL_UPPER_M
    remaining_lower = plan['lateral_bounds_m'][0] - travel_upper
    depth_lower = plan['depth_bounds_m'][0] - travel_upper * sin(radians(plan['max_abs_heading_deg']))
    if remaining_lower < .025 or depth_lower < .075:
        return None
    return {'continuous_action': remaining[0], 'estimated_travel_upper_m': travel_upper,
            'estimated_remaining_lateral_lower_m': remaining_lower}
