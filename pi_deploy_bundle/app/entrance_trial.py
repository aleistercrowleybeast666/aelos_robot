"""Command 0's estimated Tag 1 to Tag 2 trial, inside its existing session."""

from math import isfinite, radians, sin

from action_chains_v5 import COMMAND0_OUTBOUND
from cmd0_motion import (COMMAND0_ENTRY_FORWARD_STEPS, Cmd0Motion_ContinuousReady,
                         Cmd0Motion_ForwardActions, Cmd0Motion_ForwardStepCount, Cmd0Motion_LegacyReady)
from checkpoint_errors import VisualCamera_GetFreshnessCheck, VisualCameraUnavailable
from checkpoint_scan import (Checkpoint_FromBoundary, Checkpoint_Observe, Checkpoint_OptionalScanTime,
                             Checkpoint_ReadWindow, CheckpointScanResult)
from nav_observation import summarize, wrap
from checkpoint_alignment import RouteTagHeadingRebaseResult
from route_pose import (RoutePose_Load, RoutePose_Measure, RoutePose_PartialCheckpointGeometry,
                        RoutePose_PlanForwardCount, RoutePose_PlanTag1ForwardTrial, RoutePose_PlanTag2ForwardTrial,
                        TAG2_APPROACH_COUNTS)


# Older tools import the checkpoint API from the entrance module.
EntranceTrial_BoundaryCheckpoint = Checkpoint_FromBoundary
EntranceTrial_ObserveCheckpoint = Checkpoint_Observe
EntranceTrialResult = CheckpointScanResult

_TAG1_FORWARD_TRIAL_COUNTS = (8, 12, 14, 15)
_TAG1_STANCE_REFERENCE_MAX_NS = 5_000_000_000
TAG1_CLEARANCE_YAW_DEG = 20.
_TAG1_CLEARANCE_SIDE_M = .10
_TAG1_CLEARANCE_STRIDE_UPPER_M = .10
_TAG1_CLEARANCE_DEPTH_M = (.11, .45)
_TAG1_CLEARANCE_EDGE_MARGIN_M = .01


def EntranceTrial_IsEntryAccepted(alignment):
    if not alignment:
        return False
    observation = alignment.get('scan') or {}
    heading = alignment.get('decision_fusion') or {}
    identified = (alignment.get('expected_id') == 1
            and alignment.get('fresh_target_detected') is True
            and (alignment.get('camera_info') or {}).get('ready') is True
            and observation.get('ids') == [1])
    if alignment.get('tag1_nominal_entry') is True:
        return identified
    return (identified and alignment.get('pose_accepted') is True
            and (alignment.get('heading_aligned') is True
                 or alignment.get('heading_residual_advisory') is True)
            and heading.get('source') == 'trusted_tag_heading'
            and heading.get('target_heading_deg') == 0
            and observation.get('pose_usable') is True)


def _report_progress(robot, phase, result):
    report = getattr(robot.backend, 'report_progress', None)
    if report is not None:
        try:
            command = getattr(robot, 'route_command', '0')
            progress={'command': command, 'phase': phase, 'direction': 'outbound',
                    'outbound_only': getattr(robot,'outbound_only',command=='1'), 'shared_outbound': command in ('1','2'),
                    'journey': getattr(robot,'deliveries',0)+1,
                    'expected_tag': result['expected_id'], 'trial_target_tag': 5,
                    'target_steps': result['target_steps'], 'planned_steps': result['planned_steps'],
                    'completed_steps': result['completed_steps'],
                    'completed_actions': list(result['completed_actions']),
                    'delivery_planned': Cmd0Motion_ContinuousReady(robot.backend), 'route_evidence': 'estimated_trial'}
            if result.get('stationary_wait_reason_zh'):
                progress['reason_zh']=result['stationary_wait_reason_zh']
            if result.get('checkpoint_not_seen'):
                progress['checkpoint_not_seen']=result['checkpoint_not_seen']
            report(progress)
        except Exception:
            pass  # Reporting must not affect execution.


def EntranceTrial_PlanForward(target_steps, *, continuous, legacy):
    operations = Cmd0Motion_ForwardActions(target_steps, continuous=continuous, legacy=legacy)
    if operations:
        return operations, target_steps
    # The base MCU only exports 8/4/2; round an unsupported odd target down.
    from task_runner import segments
    planned_steps = target_steps - target_steps % 2
    return tuple(segments(planned_steps, 'single')), planned_steps


def _tag1_recovery_geometry(robot, nav, scan):
    """Return current Tag 1 geometry when the signed position is consistent."""
    stance = robot.last_action_completed_ros_ns
    after, observed = scan.get('after_ns'), scan.get('now_ns')
    if (not all(type(value) is int for value in (stance, after, observed))
            or not 0 < stance <= after < observed
            or scan.get('camera_healthy') is not True or scan.get('contract_match') is not True):
        return None
    rows = [row for row in scan.get('observations', ())
            if type(row.get('stamp_ns')) is int and after < row['stamp_ns'] <= observed
            and observed - row['stamp_ns'] <= nav.cfg['max_age_s'] * 1e9]
    geometry = RoutePose_PartialCheckpointGeometry(rows, nav.cfg, 1)
    if (geometry is None or geometry['samples'] < nav.cfg['min_frames']
            or geometry['latest_stamp_ns'] != scan.get('latest_stamp_ns')
            or geometry['max_heading_deviation_deg'] > 3.
            or geometry['projection_bounds'][0] < .75):
        return None
    left, right = geometry['lateral_bounds_m']
    near, far = geometry['distance_bounds_m']
    if right-left > .025 or far-near > .025 or max(abs(left), abs(right)) > .24:
        return None
    return geometry


def _tag1_clearance_strafe(geometry):
    left, right = geometry['lateral_bounds_m']
    near, far = geometry['distance_bounds_m']
    direction = -1 if left > _TAG1_CLEARANCE_SIDE_M else 1 if right < -_TAG1_CLEARANCE_SIDE_M else 0
    if direction == 0:
        return None
    yaw = geometry['heading_deg']
    spread = geometry['max_heading_deviation_deg']
    shifts = tuple(direction * _TAG1_CLEARANCE_STRIDE_UPPER_M * sin(radians(angle))
                   for angle in (yaw-spread, yaw+spread))
    depth_min, depth_max = _TAG1_CLEARANCE_DEPTH_M
    inside = near + min(0., *shifts) >= depth_min and far + max(0., *shifts) <= depth_max
    # Immediately outside a boundary, a lateral clearance may also recover depth.
    # All measured yaws must move inward, and a full stride must be able to reach the band.
    from_near = (depth_min-_TAG1_CLEARANCE_EDGE_MARGIN_M <= near < depth_min
                 and near + min(shifts) >= depth_min and far + max(shifts) <= depth_max)
    from_far = (depth_max < far <= depth_max+_TAG1_CLEARANCE_EDGE_MARGIN_M
                and far + max(shifts) <= depth_max and near + min(shifts) >= depth_min)
    if not (inside or from_near or from_far):
        return None
    return 'strafe_left_once' if direction < 0 else 'strafe_right_once'


def _tag1_observe_after_recovery(robot, nav, goal, recovery):
    """Read the new stance before another action can use its pose."""
    from visual_micro import VisualMicro_ObserveWithoutMagnetic, VisualMicroStopped

    after = Checkpoint_OptionalScanTime(nav.io, robot.check)
    live = VisualCamera_GetFreshnessCheck(nav.io)
    if after is None or live is None:
        recovery['reobserve_reason'] = 'Tag 1 的相机时钟或新帧校验暂不可用。'
        return None
    try:
        alignment = VisualMicro_ObserveWithoutMagnetic(nav.cfg, nav.io, 1, goal, after_ns=after,
            deadline=min(robot.deadline, robot.clock()+nav.cfg['settle_s']+2*nav.cfg['observe_s']),
            check=robot.check, clock=robot.clock, accept_trusted_residual=True)
    except (VisualMicroStopped, OSError) as exc:
        robot.check()
        recovery['reobserve_reason'] = str(exc)
        return None
    scan = alignment['scan']
    stamp = scan.get('latest_stamp_ns')
    completed = robot.last_action_completed_ros_ns
    if (alignment.get('pose_accepted') is not True or scan.get('ids') != [1] or type(stamp) is not int
            or type(completed) is not int or stamp <= max(after, completed) or not live(stamp)):
        recovery['reobserve_reason'] = alignment['reason']
        return None
    alignment['heading_residual_advisory'] = not alignment.get('heading_aligned')
    recovery['reobservations'].append({'latest_stamp_ns': stamp, 'heading_deg': alignment['error_deg']})
    return alignment


def _tag1_recover_departure(robot, nav, goal, alignment, result, *, reserved_actions):
    """Handle trusted Tag 1 heading left over from initial acquisition."""
    turn = getattr(robot, 'Route_TurnFromTagHeading', None)
    visual = nav.cfg.get('visual_micro', {})
    if (turn is None or not alignment or alignment.get('heading_residual_advisory') is not True
            or not EntranceTrial_IsEntryAccepted(alignment)
            or visual.get('execute') is not True):
        return alignment
    error = alignment['decision_fusion'].get('error_deg')
    stamp = alignment['scan'].get('latest_stamp_ns')
    live = VisualCamera_GetFreshnessCheck(nav.io)
    deadband = visual['deadband_deg']
    completed = robot.last_action_completed_ros_ns
    if (type(error) not in (int, float) or not isfinite(error) or abs(error) <= deadband
            or type(stamp) is not int or type(completed) is not int or stamp <= completed
            or live is None or not live(stamp)):
        return alignment
    already_turned = alignment.get('correction_executed') is True
    if already_turned and abs(error) < TAG1_CLEARANCE_YAW_DEG:
        return alignment
    recovery = {'initial_heading_deg': error, 'reobservations': []}
    result['tag1_heading_recovery'] = recovery
    if abs(error) >= TAG1_CLEARANCE_YAW_DEG:
        side = getattr(robot, 'corridor_side_action', None)
        geometry = _tag1_recovery_geometry(robot, nav, alignment['scan'])
        operation = (_tag1_clearance_strafe(geometry) if geometry is not None and side is not None
                     and goal.get('allow_strafe') is True else None)
        if (operation is None or robot.count+reserved_actions+2 > robot.cfg['max_body_macros_per_journey']
                or not live(stamp)):
            recovery['reason_zh'] = 'Tag 1 偏航较大，当前未确认可执行的让位侧跨，保留视觉余差。'
            return alignment
        label = '左跨' if operation == 'strafe_left_once' else '右跨'
        _report_progress(robot, 'cmd0_entry_reacquire', dict(result, expected_id=1,
            stationary_wait_reason_zh=f'Tag 1 横偏和航向偏差较大，先{label}一步，再用新画面修正航向。'))
        side(operation)
        recovery.update(clearance_action=operation, clearance_geometry=geometry)
        before_camera = (alignment.get('camera_info') or {}).get('fingerprint')
        alignment = _tag1_observe_after_recovery(robot, nav, goal, recovery)
        if alignment is None:
            return None
        error = alignment['error_deg']
        stamp = alignment['scan']['latest_stamp_ns']
        near, far = geometry['distance_bounds_m']
        depth_min, depth_max = _TAG1_CLEARANCE_DEPTH_M
        if abs(error) >= TAG1_CLEARANCE_YAW_DEG or not (depth_min <= near <= far <= depth_max):
            updated = _tag1_recovery_geometry(robot, nav, alignment['scan'])
            if (updated is None or updated['latest_stamp_ns'] <= geometry['latest_stamp_ns']
                    or not before_camera or (alignment.get('camera_info') or {}).get('fingerprint') != before_camera):
                recovery['reason_zh'] = 'Tag 1 让位后未取得同一相机的新位姿，保留视觉余差。'
                return alignment
            near, far = updated['distance_bounds_m']
            if not depth_min <= near <= far <= depth_max:
                recovery['reason_zh'] = 'Tag 1 让位后仍在转向距离范围外，保留视觉余差。'
                return alignment
            sign = 1 if operation == 'strafe_left_once' else -1
            low, high = sorted(sign*x for x in geometry['lateral_bounds_m'])
            low_after, high_after = sorted(sign*x for x in updated['lateral_bounds_m'])
            gain = high - max(abs(low_after), abs(high_after))
            spread = max(high-low, high_after-low_after)
            recovery.update(lateral_gain_m=gain, lateral_window_spread_m=spread)
            if (abs(error) >= TAG1_CLEARANCE_YAW_DEG and gain < .01
                    and not (0 <= low_after <= high_after < low and gain >= max(.002, spread))):
                recovery['reason_zh'] = 'Tag 1 让位后仍偏航较大，新画面未能区分横偏改善和帧间波动，保留视觉余差。'
                return alignment
    if abs(error) > deadband and (not already_turned or 'clearance_action' in recovery):
        correction = turn(1, error, stamp, reserved_actions=reserved_actions)
        if correction is not None:
            recovery['visual_turn'] = correction
            alignment = _tag1_observe_after_recovery(robot, nav, goal, recovery)
            if alignment is not None:
                correction['after_error_deg'] = alignment['error_deg']
    return alignment


def _tag1_forward_plan(nav, profile, load, scan, camera_info, nominal, candidates, trial_stride):
    pose = RoutePose_Measure(dict(scan, expected_load=load), nav.cfg, profile, 1, camera_info=camera_info)
    plan = RoutePose_PlanForwardCount(pose, profile, nominal)
    if (candidates and plan.get('reason') == 'same-load Tag 1 stride and next-checkpoint clearance unverified'):
        if 'ids' not in scan:
            scan = dict(scan, **summarize(scan, 1, nav.cfg, scan.get('after_ns', 0)))
        plan = RoutePose_PlanTag1ForwardTrial({'scan': scan, 'camera_info': camera_info}, nav.cfg,
                                            nominal, candidates, trial_stride)
    return pose, plan


def _tag1_paired_heading(robot, nav, goal, alignment):
    """Carry the measured Tag 1 offset into the straight without zeroing its yaw."""
    unavailable = RouteTagHeadingRebaseResult.UNAVAILABLE
    scan = alignment['scan']
    fused = alignment['decision_fusion']
    reference = (alignment.get('magnetic') or {}).get('reference') or {}
    anchor = nav.mag.anchor or {}
    stamp, after, observed = (scan.get(k) for k in ('latest_stamp_ns','after_ns','now_ns'))
    completed = robot.last_action_completed_ros_ns
    if (not reference or not anchor or not nav.cfg['post_action_heading'].get('enabled')
            or getattr(robot.backend, 'is_simulation', False)
            or anchor.get('source') != 'reliable_tag_alignment'
            or anchor.get('tag_id') != 1 or anchor.get('load') != robot.load_kind
            or robot.load_kind not in ('single','double') or not anchor.get('capture_verified')
            or not anchor.get('usable_for_heading') or not all(type(v) is int for v in (completed,after,stamp,observed))
            or not 0 < completed <= after < stamp <= observed
            or anchor.get('visual_stamp_ns') != stamp or stamp <= getattr(nav.io,'restart_barrier',0)
            or scan.get('contract_match') is not True or scan.get('camera_healthy') is not True
            or not (alignment.get('camera_info') or {}).get('fingerprint')
            or anchor.get('visual_camera_fingerprint') != alignment['camera_info']['fingerprint']):
        return unavailable
    now_ns = Checkpoint_OptionalScanTime(nav.io, robot.check)
    if now_ns is None or not 0 <= now_ns - stamp <= _TAG1_STANCE_REFERENCE_MAX_NS:
        return unavailable
    stream = getattr(nav.io,'capture_stream',None)
    session = anchor.get('capture_session')
    calibration = anchor.get('calibration_id')
    epoch = anchor.get('load_epoch')
    scope = (getattr(robot.backend,'session',''), getattr(robot.backend,'epoch',-1))
    if (not session or not calibration or type(epoch) is not int or not scope[0]
            or type(scope[1]) is not int or scope[1] < 0
            or anchor.get('owner_observation_scope') != scope or not getattr(nav.io,'motion_capture_verified',False)
            or session != getattr(stream,'session',None) or calibration != getattr(stream,'calibration_digest',None)
            or epoch != getattr(nav.io,'magnetic_load_epoch',None)
            or any(anchor.get(k) != reference.get(k) for k in ('raw_heading_deg','offset_deg','capture_session',
                        'calibration_id','received_monotonic','load_epoch','norm'))):
        return unavailable
    yaw, goal_yaw = fused.get('error_deg'), goal.get('route_heading_deg')
    measured, offset = anchor.get('raw_heading_deg'), anchor.get('offset_deg')
    started, paired, receipt = (getattr(robot,'last_action_completed_monotonic',None),
                                anchor.get('paired_after_monotonic'),anchor.get('received_monotonic'))
    sign = nav.cfg['magnetic'].get('heading_sign')
    now = robot.clock()
    if (not all(type(v) in (int,float) and isfinite(v) for v in (yaw,goal_yaw,measured,offset,started,paired,receipt,now))
            or not started <= paired < receipt <= now or sign not in (-1,1)
            or now - receipt > _TAG1_STANCE_REFERENCE_MAX_NS / 1e9
            or abs(wrap(sign*measured + offset - goal_yaw - yaw)) > .01):
        return unavailable
    robot.check()
    departure = {'tag_id':1,'route_heading_deg':goal_yaw,'error_deg':yaw,'observed_at_ns':stamp}
    nav.mag.anchor = dict(anchor,source='relative_stationary_segment_start',route_heading_deg=goal_yaw,
                          absolute_route_heading_known=True,tag_departure=departure)
    guard = robot._heading_controller(nav)
    guard.latched = False
    robot.magnetic_route_only = False
    robot.last_heading_report = None
    robot.events.append({'segment_heading_reference':{'status':'tag_departure_reference_established',
                        'reference_source':'same_stance_tag_magnetic_pair','tag_departure':departure,
                        'reference':dict(nav.mag.anchor)}})
    return RouteTagHeadingRebaseResult.APPLIED


def _tag1_same_stance_reference(robot, nav, plan, scan, fresh, camera_info, completed_ns):
    """A settled reference can size the existing straight while the stance stays fixed."""
    latest = plan.get('latest_stamp_ns')
    after, observed = scan.get('after_ns'), scan.get('now_ns')
    current = fresh.get('camera_info') or {}
    now = Checkpoint_OptionalScanTime(nav.io, robot.check)
    return (plan.get('source') == 'tag1_forward_trial'
            and all(type(stamp) is int for stamp in (completed_ns, after, latest, observed))
            and 0 < completed_ns == robot.last_action_completed_ros_ns <= after < latest <= observed
            and now is not None and 0 <= now - latest <= _TAG1_STANCE_REFERENCE_MAX_NS
            and fresh.get('camera_healthy') is True and fresh.get('contract_match') is True
            and current.get('ready') is True and current.get('fingerprint') == camera_info['fingerprint']
            and all(row.get('id') == 1 for row in fresh.get('observations', ())))


def _tag2_boundary_pose_rows(robot, nav, boundary):
    after = boundary.get('after_ns')
    completed = robot.last_action_completed_ros_ns
    now = Checkpoint_OptionalScanTime(nav.io, robot.check)
    if (not boundary.get('camera_info_ready') or not all(type(stamp) is int for stamp in (after, completed, now))
            or not 0 < completed <= after <= now):
        return []
    return [row for row in boundary['scan'].get('observations', ())
            if type(row.get('stamp_ns')) is int and after < row['stamp_ns'] <= now
            and now-row['stamp_ns'] <= nav.cfg['max_age_s']*1e9]


def _tag2_boundary_near_hint(robot, nav, boundary, near_limit, *, rows=None):
    """A close glimpse can request a rescan from farther away, never authorize right strafe."""
    rows = _tag2_boundary_pose_rows(robot, nav, boundary) if rows is None else rows
    geometry = RoutePose_PartialCheckpointGeometry(rows, nav.cfg, 2)
    if geometry is None:
        return None
    near, far = geometry['distance_bounds_m']
    if (not .02 <= near <= far <= near_limit + .01
            or max(map(abs, geometry['lateral_bounds_m'])) > .10 or geometry['max_abs_heading_deg'] > 12.):
        return None
    return {'distance_m': far, 'latest_stamp_ns': geometry['latest_stamp_ns'],
            'stance_completed_ns': robot.last_action_completed_ros_ns, 'advisory_only': True}


def _tag2_boundary_replan(checkpoint, nav, settings, remaining):
    """Shorten the pending straight using a settled observation before another macro."""
    plan = RoutePose_PlanTag2ForwardTrial(checkpoint,nav.cfg,settings,candidate_counts=TAG2_APPROACH_COUNTS)
    if 'distance_bounds_m' not in plan:
        return None
    near, far = plan['distance_bounds_m']
    left, right = plan['lateral_bounds_m']
    if (not .02 <= near <= far <= .40 or far-near > .025 or right-left > .025
            or plan['projection_bounds'][0] < .85 or plan['max_heading_deviation_deg'] > 3.):
        return None
    if plan.get('heading_to_correct_deg') is not None or plan.get('heading_reobserve_required'):
        return 0,plan,'已提前读到 Tag 2，先修正当前航向。'
    if plan['max_abs_lateral_m'] > .12:
        return 0,plan,'已提前读到 Tag 2，横偏较大，先按当前位置核对侧跨起点。'
    if plan.get('confirmed_near') or (plan['in_departure_window'] and not plan['planned_steps']):
        return 0,plan,'已按 Tag 2 实测距离收短直行，保留当前扫码位置。'
    if 0 < plan['planned_steps'] < remaining:
        return plan['planned_steps'],plan,'已把 Tag 2 的实测距离合入剩余直行步数。'
    return remaining,plan,None


def EntranceTrial_Run(robot, *, camera_info=None, entry_alignment=None):
    """Continue from an already accepted Tag 1; scan at the next action boundary."""
    start, forward, end = COMMAND0_OUTBOUND[:3]
    if (start != {'tag': 1, 'direction': 'outbound'}
            or forward != {'op': 'carry_forward_group', 'steps': COMMAND0_ENTRY_FORWARD_STEPS}
            or end != {'tag': 2, 'direction': 'outbound'}):
        raise ValueError('Command 0 entry trial expects the Tag 1 to Tag 2 forward segment')

    expected_id = end['tag']
    route_steps = robot.cfg.get('route_steps', {})
    estimate = route_steps.get('continue_estimated_route') is True
    target_steps = route_steps.get('tag1_to_tag2_forward_steps', forward['steps'])
    continuous = Cmd0Motion_ContinuousReady(robot.backend)
    legacy = not continuous and Cmd0Motion_LegacyReady(robot.backend)
    operations, planned_steps = EntranceTrial_PlanForward(target_steps, continuous=continuous, legacy=legacy)
    result = {'from_tag': start['tag'], 'expected_id': expected_id, 'next_stage_started': False,
              'detected': False, 'position_confirmed': False, 'route_evidence': 'estimated_trial',
              'expected_load': robot.load_kind, 'completed_actions': [], 'observation_windows': 0,
              'completed_steps': 0, 'forward_protocol_complete': False,
              'nominal_target_steps': target_steps, 'target_steps': target_steps, 'planned_steps': planned_steps,
              'scan': None, 'status': CheckpointScanResult.MOTION_NOT_STARTED.value,
              'reason': 'entry trial preparing'}
    # Keep the latest snapshot once so cancellation retains diagnostics and a
    # prolonged reacquisition does not create an event for every window.
    robot.events.append({'entrance_trial': result})

    def set_status(status, reason):
        result.update(status=status.value, reason=reason)
        return result

    robot.check()
    if robot.load_kind != 'single' and not (robot.load_kind == 'double' and getattr(robot, 'pair_route_enabled', False)):
        return set_status(CheckpointScanResult.MOTION_NOT_STARTED, 'expected supported carry load required')
    if robot.count + len(operations) > robot.cfg['max_body_macros_per_journey']:
        return set_status(CheckpointScanResult.MOTION_NOT_STARTED, 'insufficient body macro budget for entry segment')
    nav = robot.ensure_navigation_io()
    if nav is None:
        return set_status(CheckpointScanResult.MOTION_NOT_STARTED, 'stationary Tag observation unavailable')
    points = nav.cfg['legs']['outbound']
    if [point['id'] for point in points[:2]] != [start['tag'], expected_id]:
        return set_status(CheckpointScanResult.MOTION_NOT_STARTED, 'configured outbound Tag order differs from entry trial')

    if estimate and not EntranceTrial_IsEntryAccepted(entry_alignment):
        result['entry_fallback'] = 'Tag 1 找码后仍未取得位姿，按预设步数走向 Tag 2。'
        entry_alignment = camera_info = None
    if camera_info is not None:
        snapshot = getattr(nav.io, 'preflight_health_snapshot', None)
        try:
            health = snapshot() if snapshot is not None else {}
        except OSError as exc:
            robot.check()
            result['camera_read_error'] = str(exc)
            health = {}
        current = health.get('camera_info', {})
        if (not health.get('camera_healthy') or not current.get('ready')
                or not camera_info.get('fingerprint') or current.get('fingerprint') != camera_info['fingerprint']):
            if not estimate:
                return set_status(CheckpointScanResult.CAMERA_UNAVAILABLE, 'chest camera stream or calibration changed before entry trial')
            result['entry_fallback'] = '胸部相机数据源已变化，舍弃旧位姿，按预设步数走向 Tag 2。'
            entry_alignment = camera_info = None

    original_alignment = entry_alignment
    entry_alignment = _tag1_recover_departure(robot, nav, points[0], entry_alignment, result,
                                             reserved_actions=len(operations))
    if entry_alignment is None and original_alignment is not None:
        if not estimate:
            return set_status(CheckpointScanResult.MOTION_NOT_STARTED,
                              'Tag 1 纠偏后未重新确认入口航向，保留持物并记录本次恢复动作。')
        result['entry_fallback'] = 'Tag 1 纠偏后暂不可见，沿预设路线进入下一段。'
    if entry_alignment is not None and entry_alignment is not original_alignment:
        camera_info = entry_alignment['camera_info']
    entry_stance_ns = robot.last_action_completed_ros_ns
    nominal_entry = bool(result.get('entry_fallback') or entry_alignment and entry_alignment.get('tag1_nominal_entry') is True)
    rebase = getattr(robot, 'Route_RebaseTagHeading', None)
    tag_rebased = False
    if rebase is not None and not nominal_entry and entry_alignment and EntranceTrial_IsEntryAccepted(entry_alignment):
        outcome = (_tag1_paired_heading(robot, nav, points[0], entry_alignment) if camera_info
                   else RouteTagHeadingRebaseResult.UNAVAILABLE)
        if outcome is RouteTagHeadingRebaseResult.APPLIED:
            result['entry_heading_source'] = 'same_stance_tag_magnetic_pair'
        else:
            stamp = entry_alignment['scan'].get('latest_stamp_ns')
            outcome = rebase(start['tag'], entry_alignment['decision_fusion'].get('error_deg'), stamp)
            live = VisualCamera_GetFreshnessCheck(nav.io)
            if (outcome is RouteTagHeadingRebaseResult.UNAVAILABLE and camera_info and estimate
                    and type(stamp) is int and live is not None and not live(stamp)
                    and callable(getattr(robot,'clock',None)) and getattr(robot,'deadline',None) is not None
                    and not getattr(robot.backend,'is_simulation',False)):
                refresh = {'reobservations':[]}
                result['entry_heading_reobserve'] = refresh
                current = _tag1_observe_after_recovery(robot, nav, points[0], refresh)
                if (current and entry_stance_ns == robot.last_action_completed_ros_ns
                        and (current.get('camera_info') or {}).get('fingerprint') == camera_info.get('fingerprint')
                        and EntranceTrial_IsEntryAccepted(current)):
                    if entry_alignment.get('correction_executed'):
                        current['correction_executed'] = True
                    current = _tag1_recover_departure(robot, nav, points[0], current, result,
                                                      reserved_actions=len(operations))
                    entry_stance_ns = robot.last_action_completed_ros_ns
                    entry_alignment = current
                    if current and current['camera_info'].get('fingerprint') == camera_info.get('fingerprint'):
                        camera_info = current['camera_info']
                        outcome = rebase(start['tag'], current['decision_fusion'].get('error_deg'),
                                         current['scan']['latest_stamp_ns'])
                        if outcome is RouteTagHeadingRebaseResult.APPLIED:
                            result['entry_heading_source'] = 'stationary_tag_reobservation'
                    else:
                        entry_alignment = camera_info = None
                        refresh['reobserve_reason'] = '入口纠偏后未重新取得同一相机的位姿，按预设路线继续。'
                elif robot.last_action_completed_ros_ns != entry_stance_ns:
                    entry_alignment = camera_info = None
                elif current:
                    entry_alignment = camera_info = None
                    refresh['reobserve_reason'] = '新观测的相机标定或入口位姿未通过核对。'
        result['tag_heading_rebase'] = outcome.value
        tag_rebased = outcome is RouteTagHeadingRebaseResult.APPLIED
    if nominal_entry:
        robot.prepare_heading_segment(fresh=False)
        result['entry_heading_source'] = 'retained_magnetic_reference'
        result['forward_plan'] = {'status': 'nominal', 'count': target_steps,
                                  'reason': result.get('entry_fallback', 'Tag 1 identified, but metric pose was not used')}
    elif not tag_rebased:
        # Establish a fresh relative zero if the Tag reading has expired or the
        # Mag stream cannot be paired with the confirmed departure stance.
        nav.mag.anchor = None
        result['entry_heading_source'] = 'relative_straight_only'
        robot.prepare_heading_segment(fresh=True)
    if (nav.cfg['post_action_heading'].get('enabled') and nav.mag.anchor is None
            and not getattr(robot,'magnetic_route_only',False) and not estimate):
        return set_status(CheckpointScanResult.MOTION_NOT_STARTED, 'fresh straight-segment heading reference unavailable')
    if entry_alignment is not None and not nominal_entry:
        profile = RoutePose_Load(robot.project)
        trial = route_steps.get('tag2_forward_trial') or {}
        trial_options = []
        if target_steps == COMMAND0_ENTRY_FORWARD_STEPS and trial.get('enabled') is True:
            for count in _TAG1_FORWARD_TRIAL_COUNTS:
                actions, exact_steps = EntranceTrial_PlanForward(count, continuous=continuous, legacy=legacy)
                if exact_steps == count and robot.count + len(actions) <= robot.cfg['max_body_macros_per_journey']:
                    trial_options.append(count)
        trial_stride = trial.get('two_step_distance_m')
        entry_scan = entry_alignment.get('scan', {})
        pose, plan = _tag1_forward_plan(nav, profile, robot.load_kind, entry_scan, camera_info,
                                       target_steps, trial_options, trial_stride)
        result['entry_pose'] = pose
        result['forward_plan'] = plan
        adjusted_steps = plan['count']
        if adjusted_steps != target_steps:
            motion_ready = VisualCamera_GetFreshnessCheck(nav.io)
            stamp = plan.get('latest_stamp_ns', pose.get('latest_stamp_ns', 0))
            same_stance_reference = False
            if motion_ready is not None and not motion_ready(stamp):
                # Heading preparation can outlive the original camera window.
                # Reacquire once when a calibrated change would otherwise be usable.
                try:
                    fresh = Checkpoint_ReadWindow(nav.io, start['tag'], None, nav.cfg['observe_s'], robot.check)
                except VisualCameraUnavailable as exc:
                    robot.check()
                    result['camera_read_error'] = str(exc)
                    fresh, new_pose = {}, {}
                    new_plan = {'status': 'nominal', 'count': target_steps, 'reason': 'Tag 1 重扫暂时不可用'}
                else:
                    new_pose, new_plan = _tag1_forward_plan(nav, profile, robot.load_kind, fresh, fresh.get('camera_info'),
                                                           target_steps, trial_options, trial_stride)
                if (camera_info and entry_alignment.get('pose_accepted') is True
                        and 'distance_m' not in new_plan and new_plan['count'] == target_steps
                        and _tag1_same_stance_reference(robot, nav, plan, entry_scan, fresh, camera_info, entry_stance_ns)):
                    same_stance_reference = True
                    plan.update(reused_same_stance_reference=True, reobserve_reason=new_plan['reason'])
                else:
                    pose, plan = new_pose, new_plan
                result.update(entry_pose=pose, forward_plan=plan)
                adjusted_steps = plan['count']
                stamp = plan.get('latest_stamp_ns', pose.get('latest_stamp_ns', 0))
            if adjusted_steps != target_steps:
                adjusted_actions, exact_steps = EntranceTrial_PlanForward(adjusted_steps, continuous=continuous, legacy=legacy)
                if not same_stance_reference and (not motion_ready or not motion_ready(stamp)):
                    plan.update(status='nominal', count=target_steps, reason='Tag 1 forward estimate expired before dispatch')
                elif exact_steps != adjusted_steps or robot.count+len(adjusted_actions) > robot.cfg['max_body_macros_per_journey']:
                    plan.update(status='nominal', count=target_steps, reason='adjusted forward segment exceeds supported motion budget')
                else:
                    operations = adjusted_actions
                    result.update(target_steps=adjusted_steps, planned_steps=exact_steps)

    move_entry = getattr(robot, 'entry_forward_action', None)
    tag2_settings = route_steps.get('tag2_forward_trial') or {}
    live_tag2_trial = tag2_settings.get('enabled') is True
    use_boundary_plan = (live_tag2_trial and continuous and move_entry is not None
                         and callable(getattr(nav.io,'boundary_visual_snapshot',None)))
    if use_boundary_plan:
        native_splits = {('cmd0_carry_forward10',):('carry_forward','carry_forward2'),
                         ('cmd0_carry_forward15',):('carry_forward','cmd0_carry_forward5','carry_forward2')}
        grouped = native_splits.get(tuple(operations))
        if grouped and robot.count+len(grouped) <= robot.cfg['max_body_macros_per_journey']:
            operations = grouped
            result['entry_forward_grouping'] = list(grouped)
    result['initial_estimated_steps'] = result['planned_steps']
    set_status(CheckpointScanResult.MOVING_TO_CHECKPOINT, 'executing estimated Tag 1 to Tag 2 segment')
    _report_progress(robot, 'cmd0_next_leg', result)
    pending = list(operations)
    while pending:
        operation = pending.pop(0)
        try:
            if move_entry is None:
                robot.action(operation)
                result['completed_actions'].append(operation)
            else:
                next_action = pending[0] if pending else None
                move_entry(operation, next_action=next_action, completed_actions=result['completed_actions'])
        finally:
            result['next_stage_started'] = bool(result['completed_actions'])
            result['completed_steps'] = Cmd0Motion_ForwardStepCount(result['completed_actions'])
        if not use_boundary_plan or not pending:
            continue
        checkpoint = Checkpoint_FromBoundary(robot,nav,expected_id,action=operation)
        if checkpoint is None:
            continue
        result['tag2_seen_earlier_boundary'] = True
        remaining = Cmd0Motion_ForwardStepCount(pending)
        decision = _tag2_boundary_replan(checkpoint,nav,tag2_settings,remaining)
        if decision is None:
            continue
        proposed, plan, reason = decision
        change = {'after_steps':result['completed_steps'], 'previous_remaining_steps':remaining,
                  'remaining_steps':remaining,'distance_m':plan['distance_m'],
                  'heading_deg':plan['heading_deg'],'lateral_bounds_m':plan['lateral_bounds_m']}
        result.setdefault('entry_boundary_observations',[]).append(change)
        if not reason or proposed == remaining:
            continue
        revised, exact_steps = EntranceTrial_PlanForward(proposed,continuous=continuous,legacy=legacy) if proposed else ((),0)
        if exact_steps != proposed or robot.count+len(revised) > robot.cfg['max_body_macros_per_journey']:
            continue
        pending = list(revised)
        change.update(remaining_steps=proposed,reason_zh=reason)
        result['planned_steps'] = result['completed_steps'] + proposed

    result['forward_protocol_complete'] = bool(operations) and result['completed_steps'] == result['planned_steps']
    if not result['forward_protocol_complete']:
        return set_status(CheckpointScanResult.MOTION_NOT_STARTED, 'Tag 1 到 Tag 2 的计划直行尚未全部收到动作回执。')

    set_status(CheckpointScanResult.WAITING_FOR_CHECKPOINT, 'stationary reacquisition of expected Tag 2')
    _report_progress(robot, 'cmd0_next_scan', result)
    last_action = result['completed_actions'][-1]
    cached = Checkpoint_FromBoundary(robot,nav,expected_id,action=last_action)
    if cached is not None:
        result.update(cached)
        notify = getattr(robot,'route_checkpoint_observed',None)
        if notify is not None:
            notify(expected_id)
        return set_status(CheckpointScanResult.CHECKPOINT_DETECTED, 'Tag 2 已在动作边界确认，直接复用当前位姿。')
    boundary = getattr(robot, 'last_boundary_visual', None) or {}
    glimpse = boundary.get('scan') or {}
    if (boundary.get('target_id') == expected_id and boundary.get('action') == last_action
            and boundary.get('boundary_valid') is True and boundary.get('contract_match') is True
            and boundary.get('camera_healthy') is True and expected_id in glimpse.get('observed_ids', ())):
        result['tag2_boundary_sighting'] = {'accepted_frames': glimpse.get('samples',0),
                                            'after_ns': boundary.get('after_ns')}
        if live_tag2_trial and 'distance_window_m' in tag2_settings and continuous:
            rows = _tag2_boundary_pose_rows(robot, nav, boundary)
            if rows:
                result['tag2_boundary_sighting'].update(partial_pose_rows=rows,
                    stance_completed_ns=robot.last_action_completed_ros_ns)
            near = _tag2_boundary_near_hint(robot, nav, boundary, tag2_settings['distance_window_m'][0], rows=rows)
            if near is not None:
                result['tag2_boundary_sighting']['near_advisory'] = near
    glimpse = result.get('tag2_boundary_sighting') or {}
    partial_boundary = glimpse.get('near_advisory') or glimpse.get('partial_pose_rows')
    observed, reason = Checkpoint_Observe(robot, nav, result,
        lambda: _report_progress(robot, 'cmd0_next_scan_wait', result), miss_policy='continue_estimated',
        advisory_windows=2 if partial_boundary or use_boundary_plan else None,
        quick_id=live_tag2_trial, reuse_settled_cache=live_tag2_trial)
    if estimate and observed is CheckpointScanResult.CAMERA_UNAVAILABLE:
        result['checkpoint_not_seen'] = {'tag_id': 2, 'policy': 'continue_estimated',
                                       'cause': 'camera_stream_unhealthy', 'detail': reason}
        observed = CheckpointScanResult.CHECKPOINT_NOT_SEEN
        reason = 'Tag 2 观测不可用，先在侧跨起点找码，再沿预设路线继续。'
    if observed is CheckpointScanResult.CHECKPOINT_NOT_SEEN:
        result['continue_estimated'] = True
        if result.get('tag2_boundary_sighting'):
            reason = '动作结束时读到 Tag 2，右跨前会复核前后距离。'
        elif result.get('tag2_seen_earlier_boundary'):
            result['checkpoint_not_seen']['seen_earlier_boundary'] = True
            reason = '直行途中已读到 Tag 2，继续移动后丢码，右跨前需确认当前位姿。'
    return set_status(observed, reason)
