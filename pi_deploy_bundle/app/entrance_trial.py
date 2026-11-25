"""Command 0's estimated Tag 1 to Tag 2 trial, inside its existing session."""

from enum import Enum

from action_chains_v5 import COMMAND0_OUTBOUND
from cmd0_motion import (COMMAND0_ENTRY_FORWARD_STEPS, Cmd0Motion_ContinuousReady,
                         Cmd0Motion_ForwardActions, Cmd0Motion_ForwardStepCount, Cmd0Motion_LegacyReady)
from nav_observation import summarize
from route_pose import (RoutePose_Load, RoutePose_Measure, RoutePose_PartialCheckpointGeometry,
                        RoutePose_PlanForwardCount, RoutePose_PlanTag1ForwardTrial, RoutePose_PlanTag2ForwardTrial,
                        TAG2_APPROACH_COUNTS)


class EntranceTrialResult(Enum):
    CHECKPOINT_DETECTED = 'checkpoint_detected'
    CHECKPOINT_NOT_SEEN = 'checkpoint_not_seen'
    MOVING_TO_CHECKPOINT = 'moving_to_checkpoint'
    WAITING_FOR_CHECKPOINT = 'waiting_for_checkpoint'
    CAMERA_UNAVAILABLE = 'camera_unavailable'
    MOTION_NOT_STARTED = 'motion_not_started'


_HEALTHY_WINDOWS_BEFORE_MISS = 4
_MAX_POLICY_WINDOWS = 2 * _HEALTHY_WINDOWS_BEFORE_MISS
_TAG1_FORWARD_TRIAL_COUNTS = (8, 12, 14, 15)
_TAG1_STANCE_REFERENCE_MAX_NS = 5_000_000_000
_PARTIAL_POSE_HISTORY_NS = 6_000_000_000
_PARTIAL_POSE_HISTORY_FRAMES = 8


def EntranceTrial_IsEntryAccepted(alignment):
    observation = alignment.get('scan', {})
    heading = alignment.get('decision_fusion', {})
    return (alignment.get('expected_id') == 1
            and alignment.get('fresh_target_detected') is True
            and alignment.get('pose_accepted') is True
            and (alignment.get('heading_aligned') is True
                 or alignment.get('heading_residual_advisory') is True)
            and heading.get('source') == 'trusted_tag_heading'
            and heading.get('target_heading_deg') == 0
            and alignment.get('camera_info', {}).get('ready') is True
            and observation.get('ids') == [1]
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


def EntranceTrial_BoundaryCheckpoint(robot, nav, expected_id, action=None, report=None):
    """Use a cached, completed-stance observation without starting another window."""
    if report is None:
        report = getattr(robot,'last_boundary_visual',None)
    if (not report or report.get('fault_diagnostic') or report.get('target_id') != expected_id
            or (action is not None and report.get('action') != action)
            or not all(report.get(key) is True for key in
                       ('boundary_valid','camera_healthy','camera_info_ready','contract_match'))):
        return None
    completed = robot.last_action_completed_ros_ns
    after, observed = report.get('after_ns'), report.get('now_ns')
    if (report.get('stance_completed_ns') != completed
            or not all(type(stamp) is int for stamp in (completed, after, observed))
            or not 0 < completed <= after < observed):
        return None
    scan = report.get('scan') or {}
    now = nav.io.ros_now_ns()
    rows = [row for row in scan.get('observations',()) if row.get('id') == expected_id
            and type(row.get('stamp_ns')) is int and after < row['stamp_ns'] <= observed
            and 0 <= now-row['stamp_ns'] <= nav.cfg['max_age_s']*1e9]
    ready = getattr(nav.io,'visual_motion_ready',None)
    if (scan.get('ids') != [expected_id] or len({row['stamp_ns'] for row in rows}) < nav.cfg['min_frames']
            or ready is None or not ready(max(row['stamp_ns'] for row in rows))):
        return None
    return {'expected_id':expected_id,'detected':True,'observation_windows':0,'reused_boundary_frames':True,
            'camera_info':report.get('camera_info') or {},
            'scan':dict(scan,observations=rows,after_ns=after,now_ns=observed,contract_match=True,camera_healthy=True,
                        camera_restart_ns=report.get('camera_restart_ns',0))}


def EntranceTrial_ObserveCheckpoint(robot, nav, result, report_wait, *, quick_id=False,
                                    miss_policy=None, next_stationary_id=None, single_window=False,
                                    advisory_windows=None, reuse_settled_cache=False):
    """Read the expected ID after acknowledged motion, with an optional ordinary-miss limit."""
    expected_id = result['expected_id']
    observe = getattr(nav.io, 'observe_checkpoint_id', nav.io.observe) if quick_id else nav.io.observe
    robot.settle_since_last_completed(nav.cfg['settle_s'])
    floor = robot.last_action_completed_ros_ns
    stance = floor
    unhealthy_windows = healthy_windows = 0
    seen_ids = set()
    live_recovery = getattr(robot, '_allows_missing_magnetic', lambda: False)()
    first_miss_limit = (_MAX_POLICY_WINDOWS if live_recovery and miss_policy == 'hold_load_for_placement'
                        else _HEALTHY_WINDOWS_BEFORE_MISS)
    window_limit = max(_MAX_POLICY_WINDOWS,first_miss_limit+_HEALTHY_WINDOWS_BEFORE_MISS)
    temporary_camera_gaps = 0
    unknown_camera_gap = False
    source_fault = False
    advisory_reobserved = False
    view_windows = 0
    partial_rows = {}
    partial_scope = None
    expected_seen = False

    def remember_partial(scan, window):
        nonlocal partial_scope
        camera = window.get('camera_info') or {}
        restart = window.get('camera_restart_ns', 0)
        scope = (robot.last_action_completed_ros_ns, restart, camera.get('fingerprint'))
        healthy = (scan.get('camera_healthy') is True and scan.get('contract_match') is True
                   and camera.get('ready') is True and bool(camera.get('fingerprint')) and scope[0] == stance)
        if scope != partial_scope or not healthy:
            partial_rows.clear()
        partial_scope = scope
        if not healthy:
            result.pop('partial_pose_rows', None)
            result.pop('partial_pose_same_stance_ns', None)
            return scan
        cutoff = max(stance, restart)
        now = window['now_ns']
        publisher = nav.cfg['pose_contract']['publisher']
        rows = [row for row in scan['observations'] if cutoff < row['stamp_ns'] <= now
                and row.get('source_publisher') == publisher]
        partial_rows.update((row['stamp_ns'], row) for row in rows)
        for stamp in sorted(partial_rows)[:-_PARTIAL_POSE_HISTORY_FRAMES]:
            del partial_rows[stamp]
        for stamp in tuple(partial_rows):
            if now - stamp > _PARTIAL_POSE_HISTORY_NS:
                del partial_rows[stamp]
        if not partial_rows:
            result.pop('partial_pose_rows', None)
            result.pop('partial_pose_same_stance_ns', None)
            return scan
        result.update(partial_pose_rows=list(partial_rows.values()), partial_pose_same_stance_ns=stance)
        if scan['ids'] or not rows or len(partial_rows) < nav.cfg['min_frames']:
            return scan
        fresh = [row for row in partial_rows.values() if now-row['stamp_ns'] <= nav.cfg['max_age_s']*1e9]
        if len(fresh) < nav.cfg['min_frames']:
            return scan
        geometry = RoutePose_PartialCheckpointGeometry(fresh, nav.cfg, expected_id)
        if geometry is None or geometry['max_heading_deviation_deg'] > 5.:
            return scan
        near, far = geometry['distance_bounds_m']
        left, right = geometry['lateral_bounds_m']
        if far-near > .035 or right-left > .025:
            return scan
        merged = summarize(dict(window, observations=fresh, after_ns=cutoff), expected_id, nav.cfg, cutoff)
        if merged['ids'] == [expected_id]:
            return dict(merged, camera_healthy=True, contract_match=True, after_ns=cutoff, now_ns=now,
                        camera_restart_ns=restart,
                        combined_stationary_windows=True)
        return scan

    def record_miss(cause):
        robot.check()
        result['checkpoint_not_seen'] = {'tag_id': expected_id, 'seen_tags': sorted(seen_ids),
            'healthy_windows': healthy_windows, 'observation_windows': result['observation_windows'],
            'policy': miss_policy, 'cause': cause, 'temporary_camera_gaps': temporary_camera_gaps}
        reason_zh = (f'Tag {expected_id} 扫码流暂未恢复，已记录并按动作链继续。' if cause=='temporary_camera_gap' else
                     f'Tag {expected_id} 已解码但有效帧不足，起步位姿待确认。' if cause=='insufficient_frames' else
                     f'未确认 Tag {expected_id}，已记录漏码，按动作链继续。' if miss_policy=='continue_estimated' else
                     f'未确认 Tag {expected_id}，已记录漏码，保留持物。')
        result['stationary_wait_reason_zh'] = reason_zh
        report_wait()
        return EntranceTrialResult.CHECKPOINT_NOT_SEEN, reason_zh

    while True:
        robot.check()
        after = nav.io.ros_now_ns()
        if type(after) is not int or after <= 0 or type(floor) is not int or after < floor:
            return EntranceTrialResult.CAMERA_UNAVAILABLE, 'invalid ROS clock at Tag {} observation'.format(expected_id)
        window = None
        if reuse_settled_cache and view_windows == 0:
            snapshot = getattr(nav.io, 'boundary_visual_snapshot', None)
            settled_at = floor + int(nav.cfg['settle_s'] * 1e9)
            if snapshot is not None and 0 < floor <= settled_at <= after:
                cached = snapshot(settled_at, robot.check)
                if (cached.get('boundary_valid') is True and cached.get('camera_healthy') is True
                        and cached.get('contract_match') is True):
                    cached_scan = summarize(cached, expected_id, nav.cfg, settled_at)
                    remember_partial(dict(cached_scan, camera_healthy=True, contract_match=True), cached)
                    if cached_scan['ids'] == [expected_id]:
                        window, after = cached, settled_at
                        result['reused_settled_frames'] = True
                    elif quick_id and expected_id in cached_scan['observed_ids']:
                        # Keep partial samples from this settled stance eligible while collecting fresh frames.
                        after = settled_at
        if window is None:
            window = observe(expected_id, after, nav.cfg['observe_s'], robot.check)
        floor = window['now_ns']
        if type(floor) is not int or floor < after:
            return EntranceTrialResult.CAMERA_UNAVAILABLE, 'ROS clock moved backwards during Tag {} observation'.format(expected_id)
        scan = summarize(window, expected_id, nav.cfg, after)
        scan.update(now_ns=floor, after_ns=max(after, window['after_ns']),
                    camera_restart_ns=window.get('camera_restart_ns',0),
                    contract_match=window.get('contract_match') is True,
                    camera_healthy=window.get('camera_healthy') is True)
        scan = remember_partial(scan, window)
        if quick_id:
            decoded = window.get('detector_expected_frames')
            if decoded is None:
                decoded = len({row['stamp_ns'] for row in window.get('observations', ())
                    if isinstance(row, dict) and row.get('id') == expected_id and type(row.get('stamp_ns')) is int
                    and scan['after_ns'] < row['stamp_ns'] <= floor
                    and floor - row['stamp_ns'] <= nav.cfg['max_age_s'] * 1e9})
            result.setdefault('quick_scan_windows', []).append({
                'decoded_expected_frames': decoded, 'accepted_expected_frames': scan['samples'],
                'decoded_since_boundary': window.get('detector_observed_expected_frames', decoded),
                'seen_tags': scan['observed_ids'], 'camera_healthy': scan['camera_healthy'],
                'contract_match': scan['contract_match'], 'receiver_rejections_total': window.get('rejected')})
        result['observation_windows'] += 1
        view_windows += 1
        window_count = view_windows
        result['scan'] = scan
        result['camera_info'] = window.get('camera_info', {})
        result['camera_window_status'] = window.get('camera_window_status', {})
        if not (scan['camera_healthy'] and scan['contract_match']):
            unhealthy_windows += 1
            healthy_windows = 0
            seen_ids.clear()
            result['stationary_wait_reason_zh'] = '胸部摄像头或扫码流暂时不可用，等待新画面。'
            camera_status = window.get('camera_window_status') or {}
            source_fault = (source_fault or camera_status.get('parameters_match') is False
                            or camera_status.get('ar_source_matches') is False)
            retryable = (live_recovery and camera_status.get('parameters_match') is True
                         and camera_status.get('ar_source_matches') is not False)
            if retryable:
                temporary_camera_gaps += 1
            else:
                unknown_camera_gap = True
            if advisory_windows is None and not retryable and unhealthy_windows >= 2:
                return EntranceTrialResult.CAMERA_UNAVAILABLE, 'two consecutive unhealthy chest camera or AR windows'
        else:
            unhealthy_windows = 0
            if scan['ids'] == [expected_id]:
                result['detected'] = True
                result.pop('stationary_wait_reason_zh', None)
                checkpoint=getattr(robot,'route_checkpoint_observed',None)
                if checkpoint is not None and not single_window:checkpoint(expected_id)
                return EntranceTrialResult.CHECKPOINT_DETECTED, 'fresh expected Tag {} detected after estimated segment'.format(expected_id)
            healthy_windows += 1
            seen_ids.update(scan['observed_ids'])
            expected_seen = expected_seen or expected_id in seen_ids or bool(
                quick_id and result['quick_scan_windows'][-1]['decoded_since_boundary'])
            if next_stationary_id in scan['observed_ids'] and miss_policy == 'continue_estimated':
                next_scan = summarize(window, next_stationary_id, nav.cfg, after)
                if next_scan['ids'] == [next_stationary_id]:
                    # The following checkpoint takes its own fresh window at this same station.
                    result['stationary_handoff_tag'] = next_stationary_id
                    return record_miss('next_tag_visible_at_same_station')
            if miss_policy and healthy_windows >= first_miss_limit:
                partial_tag = expected_seen
                if partial_tag and not advisory_reobserved:
                    advisory_reobserved = True
                    continue
                return record_miss('insufficient_frames' if partial_tag else 'tag_not_visible')
            if expected_id in scan['observed_ids']:
                result['stationary_wait_reason_zh'] = f'Tag {expected_id} 的有效新帧不足，继续原地扫码。'
            else:
                result['stationary_wait_reason_zh'] = f'当前未看到 Tag {expected_id}，继续原地扫码。'
        if single_window:
            if scan['camera_healthy'] and scan['contract_match']:
                return EntranceTrialResult.CHECKPOINT_NOT_SEEN, 'expected Tag absent from one stationary window'
            return EntranceTrialResult.CAMERA_UNAVAILABLE, 'stationary camera window unavailable'
        if advisory_windows is not None and window_count >= advisory_windows:
            partial_tag = expected_seen
            if partial_tag and not advisory_reobserved:
                advisory_reobserved = True
                continue
            if not healthy_windows and source_fault:
                return EntranceTrialResult.CAMERA_UNAVAILABLE, f'Tag {expected_id} 胸部相机或扫码来源异常。'
            return record_miss('insufficient_frames' if partial_tag else
                               'tag_not_visible' if healthy_windows else 'temporary_camera_gap')
        if miss_policy and window_count >= window_limit:
            if miss_policy=='continue_estimated' and temporary_camera_gaps and not unknown_camera_gap:
                return record_miss('temporary_camera_gap')
            return EntranceTrialResult.CAMERA_UNAVAILABLE, f'Tag {expected_id} 扫码流间歇异常，未形成足够的连续健康窗口。'
        if ((window_count==1 and (advisory_windows is not None or getattr(robot,'command0_scan_after_turn',False)))
                or window_count==2 or (quick_id and window_count>2) or window_count%5==0):
            report_wait()


def EntranceTrial_PlanForward(target_steps, *, continuous, legacy):
    operations = Cmd0Motion_ForwardActions(target_steps, continuous=continuous, legacy=legacy)
    if operations:
        return operations, target_steps
    # The base MCU only exports 8/4/2; round an unsupported odd target down.
    from task_runner import segments
    planned_steps = target_steps - target_steps % 2
    return tuple(segments(planned_steps, 'single')), planned_steps


def _tag1_forward_plan(nav, profile, load, scan, camera_info, nominal, candidates, trial_stride):
    pose = RoutePose_Measure(dict(scan, expected_load=load), nav.cfg, profile, 1, camera_info=camera_info)
    plan = RoutePose_PlanForwardCount(pose, profile, nominal)
    if (candidates and plan.get('reason') == 'same-load Tag 1 stride and next-checkpoint clearance unverified'):
        if 'ids' not in scan:
            scan = dict(scan, **summarize(scan, 1, nav.cfg, scan.get('after_ns', 0)))
        plan = RoutePose_PlanTag1ForwardTrial({'scan': scan, 'camera_info': camera_info}, nav.cfg,
                                            nominal, candidates, trial_stride)
    return pose, plan


def _tag1_same_stance_reference(robot, nav, plan, scan, fresh, camera_info, completed_ns):
    """A settled reference can size the existing straight while the stance stays fixed."""
    latest = plan.get('latest_stamp_ns')
    after, observed = scan.get('after_ns'), scan.get('now_ns')
    current = fresh.get('camera_info') or {}
    return (plan.get('source') == 'tag1_forward_trial'
            and all(type(stamp) is int for stamp in (completed_ns, after, latest, observed))
            and 0 < completed_ns == robot.last_action_completed_ros_ns <= after < latest <= observed
            and 0 <= nav.io.ros_now_ns() - latest <= _TAG1_STANCE_REFERENCE_MAX_NS
            and fresh.get('camera_healthy') is True and fresh.get('contract_match') is True
            and current.get('ready') is True and current.get('fingerprint') == camera_info['fingerprint']
            and all(row.get('id') == 1 for row in fresh.get('observations', ())))


def _tag2_boundary_pose_rows(robot, nav, boundary):
    after = boundary.get('after_ns')
    completed = robot.last_action_completed_ros_ns
    now = nav.io.ros_now_ns()
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
    target_steps = route_steps.get('tag1_to_tag2_forward_steps', forward['steps'])
    continuous = Cmd0Motion_ContinuousReady(robot.backend)
    legacy = not continuous and Cmd0Motion_LegacyReady(robot.backend)
    operations, planned_steps = EntranceTrial_PlanForward(target_steps, continuous=continuous, legacy=legacy)
    result = {'from_tag': start['tag'], 'expected_id': expected_id, 'next_stage_started': False,
              'detected': False, 'position_confirmed': False, 'route_evidence': 'estimated_trial',
              'expected_load': robot.load_kind, 'completed_actions': [], 'observation_windows': 0,
              'completed_steps': 0, 'forward_protocol_complete': False,
              'nominal_target_steps': target_steps, 'target_steps': target_steps, 'planned_steps': planned_steps,
              'scan': None, 'status': EntranceTrialResult.MOTION_NOT_STARTED.value,
              'reason': 'entry trial preparing'}
    # Keep the latest snapshot once so cancellation retains diagnostics and a
    # prolonged reacquisition does not create an event for every window.
    robot.events.append({'entrance_trial': result})

    def set_status(status, reason):
        result.update(status=status.value, reason=reason)
        return result

    robot.check()
    if robot.load_kind != 'single' and not (robot.load_kind == 'double' and getattr(robot, 'pair_route_enabled', False)):
        return set_status(EntranceTrialResult.MOTION_NOT_STARTED, 'expected supported carry load required')
    if robot.count + len(operations) > robot.cfg['max_body_macros_per_journey']:
        return set_status(EntranceTrialResult.MOTION_NOT_STARTED, 'insufficient body macro budget for entry segment')
    nav = robot.ensure_navigation_io()
    if nav is None:
        return set_status(EntranceTrialResult.MOTION_NOT_STARTED, 'stationary Tag observation unavailable')
    points = nav.cfg['legs']['outbound']
    if [point['id'] for point in points[:2]] != [start['tag'], expected_id]:
        return set_status(EntranceTrialResult.MOTION_NOT_STARTED, 'configured outbound Tag order differs from entry trial')

    if camera_info is not None:
        snapshot = getattr(nav.io, 'preflight_health_snapshot', None)
        health = snapshot() if snapshot is not None else {}
        current = health.get('camera_info', {})
        if (not health.get('camera_healthy') or not current.get('ready')
                or not camera_info.get('fingerprint') or current.get('fingerprint') != camera_info['fingerprint']):
            return set_status(EntranceTrialResult.CAMERA_UNAVAILABLE, 'chest camera stream or calibration changed before entry trial')

    entry_stance_ns = robot.last_action_completed_ros_ns
    rebase = getattr(robot, 'Route_RebaseTagHeading', None)
    tag_rebased = False
    if rebase is not None and entry_alignment and EntranceTrial_IsEntryAccepted(entry_alignment):
        from task_runner import RouteTagHeadingRebaseResult
        entry_scan = entry_alignment['scan']
        fused = entry_alignment['decision_fusion']
        outcome = rebase(start['tag'], fused.get('error_deg'), entry_scan.get('latest_stamp_ns'))
        result['tag_heading_rebase'] = outcome.value
        tag_rebased = outcome is RouteTagHeadingRebaseResult.APPLIED
    if not tag_rebased:
        # Establish a fresh relative zero if the Tag reading has expired or the
        # Mag stream cannot be paired with the confirmed departure stance.
        nav.mag.anchor = None
        robot.prepare_heading_segment(fresh=True)
    if (nav.cfg['post_action_heading'].get('enabled') and nav.mag.anchor is None
            and not getattr(robot,'magnetic_route_only',False)):
        return set_status(EntranceTrialResult.MOTION_NOT_STARTED, 'fresh straight-segment heading reference unavailable')
    if entry_alignment is not None:
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
            motion_ready = getattr(nav.io, 'visual_motion_ready', None)
            stamp = plan.get('latest_stamp_ns', pose.get('latest_stamp_ns', 0))
            same_stance_reference = False
            if motion_ready is not None and not motion_ready(stamp):
                # Heading preparation can outlive the original camera window.
                # Reacquire once when a calibrated change would otherwise be usable.
                fresh = nav.io.observe(start['tag'], nav.io.ros_now_ns(), nav.cfg['observe_s'], robot.check)
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
    set_status(EntranceTrialResult.MOVING_TO_CHECKPOINT, 'executing estimated Tag 1 to Tag 2 segment')
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
        checkpoint = EntranceTrial_BoundaryCheckpoint(robot,nav,expected_id,action=operation)
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
        return set_status(EntranceTrialResult.MOTION_NOT_STARTED, 'Tag 1 到 Tag 2 的计划直行尚未全部收到动作回执。')

    set_status(EntranceTrialResult.WAITING_FOR_CHECKPOINT, 'stationary reacquisition of expected Tag 2')
    _report_progress(robot, 'cmd0_next_scan', result)
    last_action = result['completed_actions'][-1]
    cached = EntranceTrial_BoundaryCheckpoint(robot,nav,expected_id,action=last_action)
    if cached is not None:
        result.update(cached)
        notify = getattr(robot,'route_checkpoint_observed',None)
        if notify is not None:
            notify(expected_id)
        return set_status(EntranceTrialResult.CHECKPOINT_DETECTED, 'Tag 2 已在动作边界确认，直接复用当前位姿。')
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
    observed, reason = EntranceTrial_ObserveCheckpoint(robot, nav, result,
        lambda: _report_progress(robot, 'cmd0_next_scan_wait', result), miss_policy='continue_estimated',
        advisory_windows=2 if partial_boundary or use_boundary_plan else None,
        quick_id=live_tag2_trial, reuse_settled_cache=live_tag2_trial)
    if observed is EntranceTrialResult.CHECKPOINT_NOT_SEEN:
        result['continue_estimated'] = True
        if result.get('tag2_boundary_sighting'):
            reason = '动作结束时读到 Tag 2，右跨前会复核前后距离。'
        elif result.get('tag2_seen_earlier_boundary'):
            result['checkpoint_not_seen']['seen_earlier_boundary'] = True
            reason = '直行途中已读到 Tag 2，继续移动后丢码，右跨前需确认当前位姿。'
    return set_status(observed, reason)
