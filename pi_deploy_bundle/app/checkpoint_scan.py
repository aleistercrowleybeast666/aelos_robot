"""Chest-camera checkpoint observations shared by outbound and return routes."""

from enum import Enum
from math import isfinite

from checkpoint_errors import (Checkpoint_AllScanWindowsHealthy, VisualCamera_GetFreshnessCheck,
                               Checkpoint_ClassifyStreamGap, Checkpoint_RecordMiss, CheckpointStreamResult,
                               VisualCamera_Read, VisualCameraUnavailable)
from nav_observation import Observation_DistinctTagRows, summarize, wrap
from route_pose import RoutePose_PartialCheckpointGeometry


class CheckpointScanResult(Enum):
    CHECKPOINT_DETECTED = 'checkpoint_detected'
    CHECKPOINT_NOT_SEEN = 'checkpoint_not_seen'
    MOVING_TO_CHECKPOINT = 'moving_to_checkpoint'
    WAITING_FOR_CHECKPOINT = 'waiting_for_checkpoint'
    CAMERA_UNAVAILABLE = 'camera_unavailable'
    MOTION_NOT_STARTED = 'motion_not_started'


_HEALTHY_WINDOWS_BEFORE_MISS = 4
_MAX_POLICY_WINDOWS = 2 * _HEALTHY_WINDOWS_BEFORE_MISS
_PARTIAL_POSE_HISTORY_NS = 6_000_000_000
_PARTIAL_POSE_HISTORY_FRAMES = 8


def Checkpoint_LastTravelDrift(robot, nav):
    """Compare the settled samples bracketing one completed, uncorrected translation."""
    travel = getattr(robot, '_last_search_travel', None)
    guard = getattr(robot, 'heading_guard', None)
    if not isinstance(travel,tuple) or not guard or guard.blocked or robot.magnetic_route_only:
        return None
    action, started, previous, before = travel
    after = guard.last_sample or {}
    completed = robot.last_action_completed_monotonic
    stance = robot.last_action_completed_ros_ns
    scope = (getattr(robot.backend,'session',None), getattr(robot.backend,'epoch',None))
    if (not scope[0] or scope != guard.observation_scope or not getattr(nav.io,'motion_capture_verified',False)
            or type(stance) is not int or stance <= 0
            or type(previous) not in (int,float) or type(completed) not in (int,float)
            or not previous <= started < completed or after.get('action_completed_monotonic') != completed
            or set(guard._target_observation_faults(after, robot.load_kind)) - {'sample_stale'}):
        return None
    now = robot.clock()
    for sample, lower, upper in ((before,previous,started),(after,completed,now)):
        times = sample.get('decision_capture_times') or ()
        if (sample.get('owner_observation_scope') != scope or sample.get('load') != robot.load_kind
                or sample.get('capture_verified') is not True or not sample.get('usable_for_heading')
                or not times or not all(type(t) in (int,float) and lower < t <= upper
                                       and upper-t <= _PARTIAL_POSE_HISTORY_NS/1e9 for t in times)
                or not all(type(sample.get(k)) in (int,float) and isfinite(sample[k])
                           for k in ('norm','raw_heading_deg')) or sample['norm'] <= 0):
            return None
    target = guard.reference.anchor or {}
    norm = target.get('norm')
    if (not before.get('capture_session') or not before.get('calibration_id')
            or type(before.get('load_epoch')) is not int
            or type(norm) not in (int,float) or not isfinite(norm) or norm <= 0
            or not all(type(target.get(k)) in (int,float) and isfinite(target[k])
                       for k in ('offset_deg','route_heading_deg'))
            or any(before.get(k) != after.get(k) for k in ('capture_session','load_epoch','calibration_id','norm_basis'))
            or abs(before['norm']/norm-1.) > nav.cfg['magnetic']['max_norm_fraction_change']
            or abs(after['norm']/before['norm'] - 1.) > nav.cfg['magnetic']['max_norm_fraction_change']):
        return None
    sign = nav.cfg['magnetic']['heading_sign']
    drift = wrap(sign * (after['raw_heading_deg'] - before['raw_heading_deg']))
    error = wrap(sign * after['raw_heading_deg'] + target['offset_deg'] - target['route_heading_deg'])
    deadband = nav.cfg['post_action_heading']['deadband_deg']
    # A gait can also correct a preceding error. Its direction alone does not
    # justify a search when the current course is already close or disagrees.
    if not (deadband < abs(drift) <= 40. and deadband < abs(error) <= 40. and drift*error > 0):
        return None
    return {'action':action, 'drift_deg':drift, 'remaining_heading_deg':error,
            'stance_ns':stance}


def Checkpoint_CorrectMissedTravel(robot, nav, checkpoint, *, reserved_actions):
    """Try the route heading once at a healthy blank waypoint before changing its depth."""
    miss = checkpoint.get('checkpoint_not_seen') or {}
    if (miss.get('cause') != 'tag_not_visible' or not Checkpoint_AllScanWindowsHealthy(miss)
            or miss.get('seen_tags') or checkpoint.get('partial_pose_rows')
            or checkpoint.get('stationary_handoff_tag') or checkpoint.get('missed_travel_heading')):
        return False
    drift = Checkpoint_LastTravelDrift(robot, nav)
    if drift is None or robot.count + reserved_actions + 1 > robot.cfg['max_body_macros_per_journey']:
        return False
    robot.check()
    checkpoint['missed_travel_heading'] = drift
    if robot.load_kind == 'empty':
        robot._await_heading(drift['action'], configured=True)
    else:
        boundary = 'corridor_side_boundary' if 'strafe' in drift['action'] else 'carry_forward2'
        robot.monitor_heading(boundary, reserved_route_actions=reserved_actions, one_turn_before_scan=True)
    drift['stance_changed'] = robot.last_action_completed_ros_ns != drift['stance_ns']
    return drift['stance_changed']


def Checkpoint_ReadScanTime(io):
    """Get a fresh ROS time floor without substituting the operator's wall clock."""
    try:
        stamp = io.ros_now_ns()
    except OSError as exc:
        raise VisualCameraUnavailable('胸部相机 ROS 时钟读取失败：' + str(exc)) from exc
    if type(stamp) is not int or stamp <= 0:
        raise VisualCameraUnavailable('胸部相机暂未提供有效的 ROS 时间戳。')
    return stamp


def Checkpoint_OptionalScanTime(io, check):
    if callable(getattr(io, 'ros_now_ns', None)):
        try:
            return Checkpoint_ReadScanTime(io)
        except VisualCameraUnavailable:
            check()
    return None


def Checkpoint_ReadPreflightWindow(io, tag, after_ns, timeout_s, check):
    """Check the streams without requiring the first route tag in staging."""
    read = getattr(io, 'observe_preflight', None)
    if read is not None:
        return read(after_ns, timeout_s, check)
    return io.observe(tag, after_ns, timeout_s, check)


def Checkpoint_ReadWindow(io, tag, after_ns, timeout_s, check, *, quick_id=False, classify_errors=True):
    """Read one AR window. A missing time floor starts the window at the current ROS time."""
    def read(check_owner):
        if quick_id:
            try:
                observe = io.observe_checkpoint_id
            except AttributeError:
                observe = io.observe
        else:
            observe = io.observe
        floor = after_ns
        if floor is None:
            floor = Checkpoint_ReadScanTime(io) if classify_errors else io.ros_now_ns()
        return observe(tag, floor, timeout_s, check_owner)

    if classify_errors:
        return VisualCamera_Read(read, check)
    return read(check)


def Checkpoint_FromBoundary(robot, nav, expected_id, action=None, report=None):
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
    restart = report.get('camera_restart_ns', 0)
    if type(restart) is not int:
        return None
    after = max(after, restart)
    rows = [row for row in scan.get('observations',()) if isinstance(row, dict) and row.get('id') == expected_id
            and type(row.get('stamp_ns')) is int and after < row['stamp_ns'] <= observed
            and 0 <= now-row['stamp_ns'] <= nav.cfg['max_age_s']*1e9]
    rows = list(Observation_DistinctTagRows(rows).values())
    ready = VisualCamera_GetFreshnessCheck(nav.io)
    if (scan.get('ids') != [expected_id] or len(rows) < nav.cfg['min_frames']
            or ready is None or not ready(max(row['stamp_ns'] for row in rows))):
        return None
    return {'expected_id':expected_id,'detected':True,'observation_windows':0,'reused_boundary_frames':True,
            'camera_info':report.get('camera_info') or {},
            'scan':dict(scan,observations=rows,after_ns=after,now_ns=observed,contract_match=True,camera_healthy=True,
                        camera_restart_ns=report.get('camera_restart_ns',0))}


def Checkpoint_Observe(robot, nav, result, report_wait, *, quick_id=False,
                       miss_policy=None, next_stationary_id=None, single_window=False,
                       advisory_windows=None, reuse_settled_cache=False):
    """Read the expected ID after acknowledged motion, with an optional ordinary-miss limit."""
    expected_id = result['expected_id']
    if (miss_policy is None and not single_window and advisory_windows is None
            and robot.cfg.get('route_steps', {}).get('continue_estimated_route') is True):
        miss_policy = 'continue_estimated'
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
        reason_zh = Checkpoint_RecordMiss(result, expected_id, seen_ids, healthy_windows,
                                          miss_policy, cause, temporary_camera_gaps, current_scan_windows=view_windows)
        report_wait()
        return CheckpointScanResult.CHECKPOINT_NOT_SEEN, reason_zh

    while True:
        robot.check()
        after = nav.io.ros_now_ns()
        if type(after) is not int or after <= 0 or type(floor) is not int or after < floor:
            return CheckpointScanResult.CAMERA_UNAVAILABLE, 'invalid ROS clock at Tag {} observation'.format(expected_id)
        window = None
        if reuse_settled_cache and view_windows == 0:
            snapshot = getattr(nav.io, 'boundary_visual_snapshot', None)
            settled_at = floor + int(nav.cfg['settle_s'] * 1e9)
            if snapshot is not None and 0 < floor <= settled_at <= after:
                try:
                    cached = VisualCamera_Read(lambda check: snapshot(settled_at, check), robot.check)
                except VisualCameraUnavailable as exc:
                    robot.check()
                    result['camera_cache_error'] = str(exc)
                    cached = {}
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
            try:
                window = Checkpoint_ReadWindow(nav.io, expected_id, after, nav.cfg['observe_s'],
                                                robot.check, quick_id=quick_id)
            except VisualCameraUnavailable as exc:
                robot.check()
                result.update(detected=False, scan=None, camera_info={}, camera_read_error=str(exc))
                return CheckpointScanResult.CAMERA_UNAVAILABLE, f'Tag {expected_id} 当前扫码窗口读取失败：{exc}'
        floor = window['now_ns']
        if type(floor) is not int or floor < after:
            return CheckpointScanResult.CAMERA_UNAVAILABLE, 'ROS clock moved backwards during Tag {} observation'.format(expected_id)
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
            gap = Checkpoint_ClassifyStreamGap(window.get('camera_window_status') or {}, allow_recovery=live_recovery)
            source_fault = source_fault or gap is CheckpointStreamResult.SOURCE_MISMATCH
            retryable = gap is CheckpointStreamResult.TEMPORARY_GAP
            if retryable:
                temporary_camera_gaps += 1
            else:
                unknown_camera_gap = True
            if advisory_windows is None and not retryable and unhealthy_windows >= 2:
                return CheckpointScanResult.CAMERA_UNAVAILABLE, 'two consecutive unhealthy chest camera or AR windows'
        else:
            unhealthy_windows = 0
            if scan['ids'] == [expected_id]:
                result['detected'] = True
                result.pop('stationary_wait_reason_zh', None)
                checkpoint=getattr(robot,'route_checkpoint_observed',None)
                if checkpoint is not None and not single_window:checkpoint(expected_id)
                return CheckpointScanResult.CHECKPOINT_DETECTED, 'fresh expected Tag {} detected after estimated segment'.format(expected_id)
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
                return CheckpointScanResult.CHECKPOINT_NOT_SEEN, 'expected Tag absent from one stationary window'
            return CheckpointScanResult.CAMERA_UNAVAILABLE, 'stationary camera window unavailable'
        if advisory_windows is not None and window_count >= advisory_windows:
            partial_tag = expected_seen
            if partial_tag and not advisory_reobserved:
                advisory_reobserved = True
                continue
            if not healthy_windows and source_fault:
                return CheckpointScanResult.CAMERA_UNAVAILABLE, f'Tag {expected_id} 胸部相机或扫码来源异常。'
            return record_miss('insufficient_frames' if partial_tag else
                               'tag_not_visible' if healthy_windows else 'temporary_camera_gap')
        if miss_policy and window_count >= window_limit:
            if miss_policy=='continue_estimated' and temporary_camera_gaps and not unknown_camera_gap:
                return record_miss('temporary_camera_gap')
            return CheckpointScanResult.CAMERA_UNAVAILABLE, f'Tag {expected_id} 扫码流间歇异常，未形成足够的连续健康窗口。'
        if ((window_count==1 and (advisory_windows is not None or getattr(robot,'command0_scan_after_turn',False)))
                or window_count==2 or (quick_id and window_count>2) or window_count%5==0):
            report_wait()
