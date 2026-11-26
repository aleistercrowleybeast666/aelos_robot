"""Estimated empty-load return from the base to the Tag 1 pickup area."""

from enum import Enum
import math

from checkpoint_alignment import (Checkpoint_AlignEmptyMarker as ReturnTrial_AlignEmptyMarker,
                                  Checkpoint_AlignEmptyDeparture,
                                  Checkpoint_MeasureEmptyMarker as _empty_marker_geometry,
                                  Checkpoint_EmptyMarkerCurrent as _empty_marker_current,
                                  Checkpoint_EmptyTurnAction)
from checkpoint_scan import Checkpoint_Observe, CheckpointScanResult
from nav_observation import summarize, wrap
from route_pose import (RoutePose_AdviseDeparture, RoutePose_Load, RoutePose_Measure,
                        RoutePose_PartialCheckpointGeometry, RoutePoseDepartureResult)
from checkpoint_errors import VisualCamera_Read, VisualCameraUnavailable


# Right and left are relative to the robot facing the transfer area. The
# printed Tag 6/7 orientation is opposite to the outbound set of markers.
_RETURN_TAGS = (5, 6, 7, 1)
_RETURN_REQUIRED_SCAN_WINDOWS = 8
_RETURN_TAG1_PROBE_WINDOWS = 3
_RETURN_FORWARD_PROBE_DEPTH_M = (.24, .55)
_RETURN_SINGLE_FRAME_PROBE_MIN_M = .34
_RETURN_DEPARTURE_RECHECKS = 3
_TURN_SECTIONS = ('base_turn', 'transfer_turn')
_MOVE_SECTIONS = tuple(f'tag{tag}' for tag in _RETURN_TAGS) + ('transfer_approach', 'station_side')
_EMPTY_MOVES = ('empty_forward2', 'free_forward', 'empty_strafe_left', 'empty_strafe_right')


def ReturnTrial_DefaultChain():
    turn = {'op': 'empty_right_turn', 'repeat': 6, 'target_delta_deg': 180.0}
    return {
        'base_turn': dict(turn),
        'tag5': [],
        'tag6': [{'op': 'free_forward', 'repeat': 2}],
        'tag7': [{'op': 'empty_strafe_left', 'repeat': 6}, {'op': 'free_forward', 'repeat': 2}],
        'tag1': [{'op': 'empty_strafe_right', 'repeat': 9}],
        'transfer_approach': [{'op': 'empty_forward2'}, {'op': 'free_forward', 'repeat': 2}],
        'transfer_turn': dict(turn),
        'station_side': [{'op': 'empty_strafe_left', 'repeat': 4}, {'op': 'empty_forward2', 'repeat': 2}],
    }


def _repeat(step, section):
    repeat = step.get('repeat', 1)
    if type(repeat) is not int or not 1 <= repeat <= 20:
        raise ValueError(f'返程动作链 {section} 的 repeat 必须是 1 至 20 的整数')
    return repeat


def ReturnTrial_ValidateChain(data):
    if not isinstance(data, dict) or set(data) != set(_TURN_SECTIONS + _MOVE_SECTIONS):
        raise ValueError('返程动作链必须包含 base_turn、tag5、tag6、tag7、tag1、transfer_approach、transfer_turn 和 station_side')

    chain = {}
    for section in _TURN_SECTIONS:
        step = data[section]
        if (not isinstance(step, dict) or set(step) != {'op', 'repeat', 'target_delta_deg'}
                or step['op'] not in ('empty_right_turn', 'empty_left_turn')):
            raise ValueError(f'返程动作链 {section} 需要空载转向 op、repeat 和 target_delta_deg')
        delta = step['target_delta_deg']
        sign = 1 if step['op'] == 'empty_right_turn' else -1
        if type(delta) not in (int, float) or not 0 < delta * sign <= 180:
            raise ValueError(f'返程动作链 {section} 的 target_delta_deg 须与转向同向且角度在 0 至 180 度之间')
        chain[section] = {'op': step['op'], 'repeat': _repeat(step, section), 'target_delta_deg': float(delta)}
    for section in _MOVE_SECTIONS:
        steps = data[section]
        if not isinstance(steps, list):
            raise ValueError(f'返程动作链 {section} 必须是空载移动列表')
        chain[section] = []
        for step in steps:
            if not isinstance(step, dict) or set(step) - {'op', 'repeat'} or step.get('op') not in _EMPTY_MOVES:
                raise ValueError(f'返程动作链 {section} 只允许空载前进或左右横移，可设置 repeat')
            chain[section].append({'op': step['op'], 'repeat': _repeat(step, section)})
    return chain


def ReturnTrial_MaxBodyMacros(chain=None):
    """Bound scheduled work separately from the shared navigation budget."""
    chain = ReturnTrial_DefaultChain() if chain is None else chain
    approach = _actions(chain['transfer_approach'])
    tag1_probe = len(approach) > 1 and approach[0] == 'empty_forward2'
    return (chain['base_turn']['repeat'] + int(tag1_probe)
            + sum(step.get('repeat', 1) for tag in _RETURN_TAGS for step in chain[f'tag{tag}']))


def _actions(steps):
    return tuple(step['op'] for step in steps for _ in range(step.get('repeat', 1)))


RETURN_TRIAL_MAX_BODY_MACROS = ReturnTrial_MaxBodyMacros()


class ReturnTrialResult(Enum):
    ROUTE_UNAVAILABLE = 'route_unavailable'
    CAMERA_UNAVAILABLE = 'camera_unavailable'
    CHECKPOINT_UNCONFIRMED = 'checkpoint_unconfirmed'
    RETURNING = 'returning'
    STATION_REACHED = 'station_reached'


def _report_progress(robot, phase, *, expected_tag, last_passed_tag, return_step, reason_zh=None):
    report = getattr(robot.backend, 'report_progress', None)
    if report is None:
        return
    try:
        progress={'phase': phase, 'direction': 'return', 'command': '2',
                'journey': getattr(robot,'deliveries',0),
                'return_started': True, 'expected_tag': expected_tag,
                'last_passed_tag': last_passed_tag, 'return_step': return_step,
                'expected_load': 'empty', 'route_evidence': 'estimated_trial'}
        if reason_zh:
            progress['reason_zh']=reason_zh
        report(progress)
    except Exception:
        pass  # Progress telemetry must not affect execution.


def _revalidate_heading(robot, nav):
    guard = getattr(robot, 'heading_guard', None)
    if guard is None:
        return
    reserve = (robot.cfg['request_timeout_s'] + nav.cfg['post_action_heading']['settle_s']
               + nav.cfg['magnetic']['collect_s'])
    report = robot.revalidate_route_heading(reserve)
    if report is not None:
        robot.events.append({'return_heading_reference': report})


def _return_heading_state(robot, nav):
    report = getattr(robot, 'last_heading_report', None) or {}
    residual = report.get('return_motion_residual_deg', report.get('after_error_deg', report.get('error_deg')))
    if not report:
        anchor = getattr(nav.mag, 'anchor', None) or {}
        tag = anchor.get('tag_departure') or {}
        stamp = tag.get('observed_at_ns')
        if (anchor.get('load') == 'empty' and type(stamp) is int
                and stamp > (robot.last_action_completed_ros_ns or 0)):
            residual = tag.get('error_deg')
    if type(residual) not in (int, float) or not math.isfinite(residual):
        # A simulation fixture with no sensors can still exercise nominal grouping.
        unknown = (bool(report) or residual is not None or getattr(robot, 'heading_guard', None) is not None
                   or getattr(robot.backend, 'is_simulation', False) is not True)
        return None, bool(unknown or getattr(robot,'magnetic_route_only',False))
    return residual, bool(abs(residual) > 15 or getattr(robot,'magnetic_route_only',False))


def _return_strafe_group(robot, actions, uncertain):
    action = actions[0]
    if (uncertain or action not in ('empty_strafe_left', 'empty_strafe_right')
            or getattr(robot.backend, 'empty_strafe_group_ready', lambda: False)() is not True):
        return action, 1
    run = 1
    while run < min(len(actions), 5) and actions[run] == action:
        run += 1
    group = 2 if run == 4 else min(run, 3)
    return (action + str(group) if group > 1 else action), group


def _prepare_return_group(robot, nav):
    robot.check()
    if robot.load_kind != 'empty':
        raise RuntimeError('return movement requires empty expected load')
    _revalidate_heading(robot, nav)
    previous = getattr(robot, 'last_heading_report', None) or {}
    if previous.pop('return_heading_pending', False):
        # A checkpoint gets visual priority. A successful visual rebase clears
        # this report; a miss still owes a heading check before the next motion.
        robot._await_heading(previous['action'], configured=True)
    return _return_heading_state(robot, nav)


def _move(robot, nav, actions, completed, *, scan_follows=False, station_actions=None):
    index = 0
    while index < len(actions):
        _, uncertain = _prepare_return_group(robot, nav)
        action, count = _return_strafe_group(robot, actions[index:], uncertain)
        index += count
        robot.finite(action, configured=True, checkpoint_follows=scan_follows and index == len(actions))
        completed.append(action)
        if station_actions is not None:
            station_actions.append(action)


def _turn(robot, step):
    robot.check()
    if robot.load_kind != 'empty':
        raise RuntimeError('return turn requires empty expected load')
    robot.chain_step(step, configured=True)


def ReturnTrial_TurnAtBase(robot, step):
    """Drop the loaded heading reference before the empty departure turn."""
    nav = robot.ensure_navigation_io()
    nav.mag.anchor = None
    if not getattr(robot.backend, 'is_simulation', False):
        robot.check_magnetic_before_motion(configured=True)
    robot.prepare_heading_segment(fresh=True)
    _turn(robot, step)


def _return_camera_snapshot(robot, snapshot, tag, after):
    try:
        return VisualCamera_Read(lambda check:snapshot(after,check), robot.check)
    except VisualCameraUnavailable as exc:
        robot.events.append({'return_camera_read_error': {'tag_id': tag, 'reason': str(exc)}})
        return {'camera_read_failed': True}


def _return_boundary_geometry(robot, nav, tag, *, health=None):
    """Keep an early glimpse only as an advisory in a recovery decision."""
    snapshot = getattr(nav.io, 'boundary_visual_snapshot', None)
    after = robot.last_action_completed_ros_ns
    if snapshot is None or type(after) is not int or after <= 0:
        return None
    scan = _return_camera_snapshot(robot, snapshot, tag, after)
    if health is not None:
        health['fault'] = (scan.get('camera_read_failed') or scan.get('camera_healthy') is False
                           or scan.get('contract_match') is False)
    if (scan.get('boundary_valid') is False or scan.get('camera_healthy') is not True
            or scan.get('contract_match') is not True):
        return None
    floor, now = scan.get('after_ns'), scan.get('now_ns')
    if type(floor) is not int or type(now) is not int or not after <= floor < now:
        return None
    restart = scan.get('camera_restart_ns', 0)
    if type(restart) is not int or restart > now:
        return None
    floor = max(floor, restart)
    rows = [row for row in scan.get('observations', ()) if isinstance(row, dict) and type(row.get('stamp_ns')) is int
            and floor < row['stamp_ns'] <= now and now - row['stamp_ns'] <= nav.cfg['max_age_s'] * 1e9]
    return RoutePose_PartialCheckpointGeometry(rows, nav.cfg, tag)


def _return_travel(robot, nav, leg, actions):
    """Read the settled camera buffer at action boundaries; shorten only the final approach."""
    tag = leg['expected_id']
    ranges = getattr(robot, 'mission', {}).get('checkpoint_forward_windows_m', {}).get('return', {})
    target = ranges.get(str(tag))
    snapshot = getattr(nav.io, 'boundary_visual_snapshot', None)
    goal = next(point for point in nav.cfg['legs']['return'] if point['id'] == tag)
    early_heading = False
    pending = list(actions)
    while pending:
        residual, uncertain = _prepare_return_group(robot, nav)
        if uncertain and pending[:2] == ['free_forward', 'free_forward']:
            # Two five-step macros and five two-step macros cover the same distance.
            # Shorten the feedback interval immediately after unresolved side drift.
            pending[:2] = ['empty_forward2'] * 5
            leg.setdefault('motion_adaptations', []).append({'reason':'heading_uncertain',
                'residual_deg':residual,'forward_steps':10,'group_steps':2})
        action = pending[0]
        native, consumed = _return_strafe_group(robot, pending, uncertain)
        del pending[:consumed]
        robot.finite(native, configured=True, checkpoint_follows=not pending)
        leg['completed_actions'].append(native)
        if (not pending or not snapshot or not target or goal.get('tag_heading_deg') is None
                or any(op != action for op in pending)):
            continue
        settled = robot.last_action_completed_ros_ns + int(nav.cfg['settle_s'] * 1e9)
        cached = _return_camera_snapshot(robot, snapshot, tag, settled)
        floor, now = cached.get('after_ns'), cached.get('now_ns')
        if (cached.get('boundary_valid') is not True or cached.get('camera_healthy') is not True
                or cached.get('contract_match') is not True or type(floor) is not int or type(now) is not int
                or not settled <= floor < now):
            continue
        scan = summarize(cached, tag, nav.cfg, settled)
        candidate = {'expected_id': tag, 'camera_info': cached.get('camera_info') or {},
                     'scan': dict(scan, after_ns=max(settled,cached['after_ns']), now_ns=cached['now_ns'],
                                  contract_match=True, camera_healthy=True)}
        geometry = _empty_marker_geometry(robot, nav, candidate)
        if geometry is None:
            continue
        residual = wrap(geometry['heading_deg'] - goal['tag_heading_deg'])
        angle = abs(residual)
        if not early_heading and Checkpoint_EmptyTurnAction(robot, nav, residual, geometry['max_heading_deviation_deg']):
            early_heading = True
            ReturnTrial_AlignEmptyMarker(robot, nav, candidate)
            leg['approach_heading'] = candidate['empty_heading']
            geometry = _empty_marker_geometry(robot, nav, candidate)
            if geometry is None:
                continue
            angle = abs(wrap(geometry['heading_deg'] - goal['tag_heading_deg']))
        low, high = geometry['distance_bounds_m']
        lateral = geometry['lateral_bounds_m']
        # The empty checkpoint windows are in signed marker-forward coordinates.
        # Consume a lateral remainder only when already abreast of the expected marker.
        if (len(pending) > (1 if action == 'free_forward' else 2)
                or -high < target[0] or -low > target[1] or high-low > .035
                or geometry['body_forward_bounds_m'][0] <= 0
                or max(map(abs, lateral)) > .07 or angle + geometry['max_heading_deviation_deg'] > 15.):
            continue
        leg['early_arrival'] = {'completed_actions': len(leg['completed_actions']),
                                'omitted_actions': list(pending), 'geometry': geometry}
        return candidate
    return None


def _align_return_departure(robot, nav, leg):
    ReturnTrial_AlignEmptyMarker(robot, nav, leg)
    if leg['expected_id'] in (5, 6, 7):
        Checkpoint_AlignEmptyDeparture(robot, nav, leg)
        recovery = leg.get('backward_search')
        if recovery and (leg.get('empty_departure_correction') or {}).get('actions'):
            recovery['resolution'] = 'use_corrected_departure_stance'


def _return_recovery_decision(robot, nav, leg, miss, boundary, *, forward_reserved):
    """Only the documented straight parts can borrow a step from the next leg."""
    tag = leg['expected_id']
    rows = leg.get('partial_pose_rows', ())
    same_stance = leg.get('partial_pose_same_stance_ns') == robot.last_action_completed_ros_ns
    geometry = RoutePose_PartialCheckpointGeometry(rows, nav.cfg, tag) if same_stance else None
    decision = {'reason': 'partial_pose_unusable', 'view_change_allowed': False,
                'geometry': geometry, 'forward_reserved': forward_reserved}
    if boundary is not None:
        decision['boundary_geometry'] = boundary
    if (miss['healthy_windows'] != miss['observation_windows']
            or miss['cause'] in ('temporary_camera_gap', 'camera_stream_unhealthy')):
        decision['reason'] = 'camera_windows_unhealthy'
        return decision
    previous_on_route = _RETURN_TAGS[_RETURN_TAGS.index(tag)-1] if tag != _RETURN_TAGS[0] else None
    if set(miss['seen_tags']) - {tag, previous_on_route}:
        decision['reason'] = 'unexpected_tag_seen'
        return decision

    if geometry is None:
        if boundary is not None and boundary['body_forward_bounds_m'][1] < _RETURN_FORWARD_PROBE_DEPTH_M[0]:
            decision['reason'] = 'tag_already_near'
        elif (boundary is None and not rows and miss['cause'] == 'tag_not_visible'
                and tag not in miss['seen_tags']):
            decision.update(reason='expected_tag_unseen',
                            view_change_allowed=forward_reserved and tag == 1)
        return decision

    nearest, farthest = geometry['body_forward_bounds_m']
    near, far = _RETURN_FORWARD_PROBE_DEPTH_M
    if boundary is not None and boundary['body_forward_bounds_m'][0] < near <= nearest:
        decision['reason'] = 'partial_pose_conflict'
    elif farthest < near:
        decision['reason'] = 'tag_already_near'
    elif farthest - nearest > .03 or nearest < near:
        decision['reason'] = 'partial_pose_conflict'
    elif farthest <= far:
        expected = next(point.get('tag_heading_deg') for point in nav.cfg['legs']['return'] if point['id'] == tag)
        single = geometry['samples'] == 1
        heading_limit, lateral_limit = (12., .08) if single else (20., .12)
        if (expected is not None and (not single or nearest >= _RETURN_SINGLE_FRAME_PROBE_MIN_M)
                and abs(wrap(geometry['heading_deg'] - expected)) + geometry['max_heading_deviation_deg'] <= heading_limit
                and max(map(abs, geometry['lateral_bounds_m'])) <= lateral_limit):
            decision.update(reason='partial_tag_far_enough', view_change_allowed=forward_reserved)
    return decision


def _return_retreat_basis(robot, nav, leg, *, repeat, prior_boundary=None):
    """Evaluate the expected empty-load marker at this stance, including a healthy blank view."""
    tag = leg['expected_id']
    miss = leg.get('checkpoint_not_seen') or {}
    if (miss.get('tag_id') != tag or not miss.get('healthy_windows')
            or miss['healthy_windows'] != miss.get('observation_windows')
            or miss.get('temporary_camera_gaps') or set(miss.get('seen_tags', ())) - {tag}):
        return None
    stamp = robot.last_action_completed_ros_ns
    partial = leg.get('partial_pose_rows', ())
    current = (RoutePose_PartialCheckpointGeometry(partial, nav.cfg, tag)
               if leg.get('partial_pose_same_stance_ns') == stamp else None)
    health = {}
    boundary = _return_boundary_geometry(robot, nav, tag, health=health)
    if health.get('fault'):
        return None
    candidates = [p for p in (current, boundary, prior_boundary) if p is not None]
    if candidates:
        if any(p['body_forward_bounds_m'][1] >= _RETURN_FORWARD_PROBE_DEPTH_M[0] for p in candidates):
            return None
        for pose in sorted(candidates, key=lambda p:p['latest_stamp_ns'], reverse=True):
            low, high = pose['body_forward_bounds_m']
            depth_limit = .12 if repeat else _RETURN_FORWARD_PROBE_DEPTH_M[0]
            if (stamp < pose['latest_stamp_ns'] and 0. < low <= high < depth_limit and high-low <= .035
                    and (not repeat or pose['samples'] >= 2)
                    and callable(getattr(nav.io, 'visual_motion_ready', None))
                    and _empty_marker_current(robot, nav, tag, pose['latest_stamp_ns'])):
                return {'reason':'expected_tag_near', 'geometry':pose}
        return None
    if (not repeat and not partial and miss.get('cause') == 'tag_not_visible'
            and not miss.get('seen_tags')):
        return {'reason':'healthy_view_without_tag'}
    return None


def _return_retreat_scan(robot, nav, profile, leg, previous, reserved, prior_boundary=None):
    """One two-step retreat on a miss, a second only on new near-marker evidence."""
    ready = getattr(robot.backend, 'empty_recovery_ready', None)
    if ready is None or ready() is not True or robot.load_kind != 'empty':
        return None
    tag = leg['expected_id']
    scan_result = None
    for attempt in range(2):
        robot.check()
        basis = _return_retreat_basis(robot, nav, leg, repeat=attempt > 0,
                                      prior_boundary=prior_boundary if attempt == 0 else None)
        # Leave space to undo every acknowledged retreat in addition to the remaining route.
        if (not basis or ready() is not True or
                robot.count + reserved + attempt + 2 > robot.cfg['max_body_macros_per_journey']):
            break
        recovery = leg.setdefault('backward_search', {'groups':[], 'restored_actions':[]})
        group = dict(basis, before_stance_ns=robot.last_action_completed_ros_ns)
        recovery['groups'].append(group)
        _report_progress(robot, 'cmd2_return_leg', expected_tag=tag, last_passed_tag=previous,
                         return_step='empty_backward_reobserve', reason_zh=f'返程寻找 Tag {tag}，空载后退两步再扫码。')
        _move(robot, nav, ('empty_backward2',), leg['completed_actions'], scan_follows=True)
        group['completed_ros_ns'] = robot.last_action_completed_ros_ns
        robot.nav_count += 1
        observation = {'expected_id':tag, 'direction':'return', 'detected':False, 'scan':None,
                       'observation_windows':0, 'departure_reobservations':0}
        scan_result = _observe_departure(robot, nav, profile, observation, previous, try_transfer_probe=True)
        group.update(result=scan_result[0].value, windows=observation['observation_windows'])
        for key in ('checkpoint_not_seen','camera_info','scan','partial_pose_rows','partial_pose_same_stance_ns'):
            leg.pop(key, None)
        total_windows = leg['observation_windows'] + observation['observation_windows']
        leg.update(observation)
        leg['observation_windows'] = total_windows
        if scan_result[0] is not CheckpointScanResult.CHECKPOINT_NOT_SEEN:
            break
    return scan_result


def _return_repay_search(robot, nav, leg, *, camera_ok):
    """A search is extra motion. Preserve a measured waypoint, or repay the search before the next leg."""
    recovery = leg.get('backward_search')
    if not recovery or recovery.get('resolution') == 'use_corrected_departure_stance':
        return
    tag = leg['expected_id']
    target = getattr(robot, 'mission', {}).get('checkpoint_forward_windows_m', {}).get('return', {}).get(str(tag))
    if not camera_ok:
        recovery['resolution'] = 'camera_unavailable_no_additional_search_motion'
        return
    heading = leg.get('empty_heading') or {}
    readings = heading.get('observations') or ()
    if (heading.get('status') in ('marker_lost_after_turn', 'orientation_inconsistent')
            or readings and abs(readings[-1]['error_deg']) > 15.):
        recovery['resolution'] = 'retain_reacquired_stance_after_heading_change'
        return
    for _ in recovery['groups']:
        pose = _empty_marker_geometry(robot, nav, leg)
        if pose:
            low, high = pose['body_forward_bounds_m']
            recovery['last_body_forward_bounds_m'] = (low,high)
            # Use the expected marker in front of the robot. The stored route windows
            # have opposite signs for opposite printed orientations.
            far = max(map(abs, target)) if target else _RETURN_FORWARD_PROBE_DEPTH_M[0]
            if low <= far + .035:
                recovery['resolution'] = 'use_reacquired_waypoint'
                return
        else:
            partial = _return_retreat_basis(robot, nav, leg, repeat=False)
            if partial and partial['reason'] == 'expected_tag_near':
                recovery['resolution'] = 'hold_measured_near_depth'
                return
        _move(robot, nav, ('empty_forward2',), leg['completed_actions'], scan_follows=True)
        robot.nav_count += 1
        recovery['restored_actions'].append('empty_forward2')
        if len(recovery['restored_actions']) < len(recovery['groups']):
            current = _return_boundary_geometry(robot, nav, tag)
            if current and current['body_forward_bounds_m'][0] <= _RETURN_FORWARD_PROBE_DEPTH_M[0]:
                recovery['resolution'] = 'use_new_boundary_depth'
                return
    recovery['resolution'] = 'restored_nominal_route_depth'


def _observe_departure(robot, nav, profile, leg, previous, *, try_transfer_probe=False, early=None):
    tag = leg['expected_id']
    advisory_heading = getattr(robot, '_allows_missing_magnetic', lambda: False)()
    nominal = RoutePose_AdviseDeparture({}, profile, tag, direction='return')
    advisory_tag = tag != _RETURN_TAGS[-1] and nominal['status'] == RoutePoseDepartureResult.UNVERIFIED.value

    def report_wait():
        step = 'departure_measurement' if leg['departure_reobservations'] else 'corridor'
        _report_progress(robot, 'cmd2_return_wait', expected_tag=tag, last_passed_tag=previous, return_step=step,
                         reason_zh=leg.get('stationary_wait_reason_zh'))

    for attempt in range(_RETURN_DEPARTURE_RECHECKS + 1):
        leg['detected'] = False
        windows = _RETURN_TAG1_PROBE_WINDOWS if try_transfer_probe else _RETURN_REQUIRED_SCAN_WINDOWS
        if tag == 1 and not try_transfer_probe:
            windows = max(1, windows - leg['observation_windows'])
        if attempt == 0 and early and _empty_marker_geometry(robot, nav, early):
            leg.update(detected=True, scan=early['scan'], camera_info=early['camera_info'], reused_boundary_frames=True)
            record = getattr(robot, 'route_checkpoint_observed', None)
            if record is not None:
                record(tag)
            observed, reason = CheckpointScanResult.CHECKPOINT_DETECTED, 'fresh expected marker at action boundary'
        else:
            observed, reason = Checkpoint_Observe(
                robot, nav, leg, report_wait, miss_policy='continue_estimated' if advisory_tag else None,
                advisory_windows=None if advisory_tag else windows, quick_id=try_transfer_probe,
                reuse_settled_cache=attempt == 0)
        if observed is CheckpointScanResult.CHECKPOINT_NOT_SEEN:
            if try_transfer_probe and attempt == 0:
                return observed, reason
            miss = leg.get('checkpoint_not_seen') or {}
            if not miss.get('healthy_windows') and miss.get('cause') in ('temporary_camera_gap','camera_stream_unhealthy'):
                return CheckpointScanResult.CAMERA_UNAVAILABLE, f'返程 Tag {tag} 扫码流未恢复，未进入下一段。'
            if not advisory_tag:
                return CheckpointScanResult.MOTION_NOT_STARTED, f'返程未确认 Tag {tag}，交接路线尚未定位。'
            return observed, reason
        if observed is not CheckpointScanResult.CHECKPOINT_DETECTED:
            return observed, reason

        scan = dict(leg['scan'], expected_load=robot.load_kind)
        pose = RoutePose_Measure(scan, nav.cfg, profile, tag,
                                 camera_info=leg['camera_info'], direction='return')
        advice = RoutePose_AdviseDeparture(pose, profile, tag, direction='return',
                                           advisory_heading=advisory_heading)
        leg.update(departure_pose=pose, departure_advice=advice)
        if advice['status'] in (RoutePoseDepartureResult.UNVERIFIED.value, RoutePoseDepartureResult.FUSE.value):
            return observed, reason
        if advice.get('heading_advisory_only'):
            leg['departure_resolution'] = 'verified_position_continue_with_heading_residual'
            return observed, reason

        leg.setdefault('first_departure_hold', advice)
        if attempt == _RETURN_DEPARTURE_RECHECKS:
            return CheckpointScanResult.MOTION_NOT_STARTED, f'返程 Tag {tag} 位姿多次复核仍不适合进入下一段。'
        leg['departure_reobservations'] += 1
        report_wait()


def ReturnTrial_ReachPickupStation(robot, chain, result):
    """Finish the 12-step transfer, face outbound, observe Tag 1 again, then enter the pickup bay."""
    estimate = robot.cfg.get('route_steps', {}).get('continue_estimated_route') is True
    planned_anchor = estimate and result.get('estimated_pickup_anchor_tag') == 1
    if not result.get('station_reached') or not (result.get('pickup_anchor_tag') == 1 or planned_anchor) or robot.load_kind != 'empty':
        raise ValueError('接驳取物需要本次返程确认的 Tag 1 和空载站姿')
    nav = robot.ensure_navigation_io()
    chain = ReturnTrial_ValidateChain(chain)
    approach = _actions(chain['transfer_approach'])
    borrowed = list(result['transfer_approach_actions'])
    if list(approach[:len(borrowed)]) != borrowed:
        raise ValueError('接驳前进回执和配置路线不匹配')
    last_seen = result.get('checkpoint', result.get('pickup_anchor_tag'))
    _report_progress(robot, 'cmd2_return_leg', expected_tag=1, last_passed_tag=last_seen, return_step='transfer_approach',
                     reason_zh='按接驳段剩余步数前进后空载掉头。')
    _move(robot, nav, approach[len(borrowed):], result['transfer_approach_actions'],
          station_actions=result['station_actions'])
    _turn(robot, chain['transfer_turn'])
    turn_completed_ns = robot.last_action_completed_ros_ns
    result['station_turn_completed'] = True
    leg = {'expected_id': 1, 'direction': 'outbound', 'detected': False, 'observation_windows': 0, 'ready': False}
    result['station_facing_tag1'] = leg
    def waiting():
        _report_progress(robot, 'cmd2_return_scan', expected_tag=1, last_passed_tag=last_seen, return_step='transfer_turn',
                         reason_zh='空载已掉头，正在按去程方向重新识别 Tag 1。')
    waiting()
    observed, detail = Checkpoint_Observe(robot, nav, leg, waiting, advisory_windows=3,
                                                       reuse_settled_cache=True)
    leg.update(status=observed.value, reason=detail)
    if observed is CheckpointScanResult.CHECKPOINT_DETECTED:
        ReturnTrial_AlignEmptyMarker(robot, nav, leg, direction='outbound')
    heading = leg.get('empty_heading') or {}
    if observed is CheckpointScanResult.CHECKPOINT_DETECTED and heading.get('status') not in (
            'within_empty_turn_resolution', 'retain_measured_residual', 'orientation_inconsistent'):
        retry = {'expected_id':1, 'detected':False, 'observation_windows':0}
        observed, detail = Checkpoint_Observe(robot, nav, retry, waiting, single_window=True)
        retry.update(status=observed.value, reason=detail)
        leg['post_turn_second_window'] = retry
        if observed is CheckpointScanResult.CHECKPOINT_DETECTED:
            ReturnTrial_AlignEmptyMarker(robot, nav, retry, direction='outbound')
        heading = retry.get('empty_heading') or {}
    else:
        retry = leg
    readings = heading.get('observations') or []
    if (heading.get('status') in ('retain_measured_residual', 'orientation_inconsistent')
            and readings and abs(readings[-1]['error_deg']) >= 45):
        leg['reason_zh'] = f"空载掉头后的 Tag 1 实测仍偏 {readings[-1]['error_deg']:+.1f}°，接驳侧跨方向待确认。"
        if not estimate:
            return leg
    if not (observed is CheckpointScanResult.CHECKPOINT_DETECTED and readings
            and heading.get('status') in ('within_empty_turn_resolution', 'retain_measured_residual')):
        from corridor_trial import CorridorTrial_HeadingSnapshot
        scan = retry.get('scan') or {}
        anchor = nav.mag.anchor or {}
        tag_stamp = (anchor.get('tag_departure') or {}).get('observed_at_ns')
        target = (anchor.get('planned_group_target') is True or
                  (anchor.get('tag_id') == 1 and anchor.get('absolute_route_heading_known') is True
                   and type(turn_completed_ns) is int and type(tag_stamp) is int and tag_stamp > turn_completed_ns))
        healthy_miss = (observed is not CheckpointScanResult.CAMERA_UNAVAILABLE and target
                        and not getattr(robot,'magnetic_route_only',False)
                        and scan.get('camera_healthy') is True and scan.get('contract_match') is True)
        if healthy_miss:
            robot.monitor_heading('planned_turn_complete', allow_correction=False)
        current = CorridorTrial_HeadingSnapshot(getattr(robot, 'last_heading_report', None) or {}, after=True, load='empty')
        completed = robot.last_action_completed_monotonic
        if (not healthy_miss or current is None or completed is None or current[1] < completed
                or not 0 <= robot.clock()-current[1] <= nav.cfg['magnetic']['max_age_s']
                or abs(current[0]) > nav.cfg['post_action_heading']['deadband_deg']):
            leg['reason_zh'] = '空载掉头后未重新确认 Tag 1，也未取得已对正的新磁航向。'
            if not estimate:
                return leg
            leg.update(heading_source='estimated_after_turn', reason_zh='掉头后复扫仍无可用航向，按接驳动作链接近方块。')
        else:
            leg.update(heading_source='verified_empty_turn', residual_deg=current[0])
    else:
        leg.update(heading_source='outbound_tag1_pose', residual_deg=readings[-1]['error_deg'])
    if observed is CheckpointScanResult.CHECKPOINT_DETECTED:
        last_seen = 1
    _report_progress(robot, 'cmd2_return_station', expected_tag=None, last_passed_tag=last_seen, return_step='station_side',
                     reason_zh='进入接驳位置，随后用胸部相机识别方块。')
    _move(robot, nav, _actions(chain['station_side']), result['station_side_actions'],
          station_actions=result['station_actions'], scan_follows=True)
    leg['ready'] = True
    return leg


def ReturnTrial_Run(robot):
    """Follow expected return markers, then hand off at confirmed Tag 1."""
    result = {'status': ReturnTrialResult.ROUTE_UNAVAILABLE.value, 'reason': '',
              'checkpoint': None, 'direction': 'return', 'station_reached': False,
              'position_confirmed': False, 'expected_load': robot.load_kind,
              'route_evidence': 'estimated_trial', 'legs': [], 'missing_checkpoints': [],
              'station_actions': [], 'transfer_approach_actions': [], 'station_side_actions': [],
              'base_turn_completed': False, 'station_turn_completed': False}
    robot.events.append({'return_trial': result})

    def set_status(status, reason):
        result.update(status=status.value, reason=reason)
        return result

    robot.check()
    if robot.load_kind != 'empty':
        return set_status(ReturnTrialResult.ROUTE_UNAVAILABLE, 'empty expected load required after release')
    nav = robot.ensure_navigation_io()
    if nav is None or tuple(goal['id'] for goal in nav.cfg['legs'].get('return', ())) != _RETURN_TAGS:
        return set_status(ReturnTrialResult.ROUTE_UNAVAILABLE, 'configured return checkpoint order differs from 5, 6, 7, 1')
    route_steps = robot.cfg.get('route_steps', {})
    try:
        chain = ReturnTrial_ValidateChain(route_steps.get('return_chain', ReturnTrial_DefaultChain()))
    except ValueError as exc:
        return set_status(ReturnTrialResult.ROUTE_UNAVAILABLE, str(exc))
    spare_nav = max(0, nav.cfg['max_extra_actions_per_journey'] - robot.nav_count)
    if robot.count + ReturnTrial_MaxBodyMacros(chain) + spare_nav > robot.cfg['max_body_macros_per_journey']:
        return set_status(ReturnTrialResult.ROUTE_UNAVAILABLE, 'insufficient body macro budget for empty return')
    if not getattr(robot.backend, 'is_simulation', False):
        robot.backend.check_mission_v5()
        probe = getattr(robot.backend, 'ensure_empty_recovery', None)
        if probe is not None:
            ready = probe()
            result['empty_recovery_ready'] = ready is True
        group_probe = getattr(robot.backend, 'ensure_empty_strafe_group', None)
        if group_probe is not None:
            result['empty_strafe_group_ready'] = group_probe() is True
    profile = RoutePose_Load(robot.project)

    set_status(ReturnTrialResult.RETURNING, 'moving through the estimated empty return')
    _report_progress(robot, 'cmd2_return_leg', expected_tag=5, last_passed_tag=None, return_step='base_turn')
    ReturnTrial_TurnAtBase(robot, chain['base_turn'])
    result['base_turn_completed'] = True
    estimate = robot.cfg.get('route_steps', {}).get('continue_estimated_route') is True
    route_actions = [_actions(chain[f'tag{tag}']) for tag in _RETURN_TAGS]
    route_actions.append(_actions(chain['transfer_approach']))

    for index, tag in enumerate(_RETURN_TAGS):
        previous = result['checkpoint']
        leg = {'expected_id': tag, 'direction': 'return', 'detected': False,
               'completed_actions': [], 'observation_windows': 0,
               'departure_reobservations': 0, 'scan': None}
        result['legs'].append(leg)
        _report_progress(robot, 'cmd2_return_leg', expected_tag=tag, last_passed_tag=previous, return_step='corridor')
        early = _return_travel(robot, nav, leg, route_actions[index])
        _report_progress(robot, 'cmd2_return_scan', expected_tag=tag, last_passed_tag=previous, return_step='corridor')
        # Only the straight base exit and transfer entry can borrow a group.
        # Tag 6 and Tag 7 precede lateral moves and do not have a forward probe.
        upcoming = route_actions[index+1]
        can_probe = tag in (5, 1) and len(upcoming) > 1 and upcoming[0] == 'empty_forward2'
        check_retreat = estimate and getattr(robot.backend, 'empty_recovery_ready', lambda: False)() is True
        boundary = _return_boundary_geometry(robot, nav, tag) if can_probe or check_retreat else None
        observed, reason = _observe_departure(robot, nav, profile, leg, previous,
                                               try_transfer_probe=can_probe and tag == 1, early=early)
        if (check_retreat and not leg.get('detected') and leg.get('checkpoint_not_seen')
                and observed in (CheckpointScanResult.CHECKPOINT_NOT_SEEN, CheckpointScanResult.MOTION_NOT_STARTED)):
            reserved = (sum(len(group) for group in route_actions[index+1:]) + chain['transfer_turn']['repeat']
                        + len(_actions(chain['station_side'])))
            rescanned = _return_retreat_scan(robot, nav, profile, leg, previous, reserved, boundary)
            if rescanned:
                observed, reason = rescanned
        if observed is CheckpointScanResult.CHECKPOINT_NOT_SEEN and not leg.get('backward_search'):
            miss = leg['checkpoint_not_seen']
            decision = _return_recovery_decision(robot, nav, leg, miss, boundary, forward_reserved=can_probe)
            leg['checkpoint_recovery'] = decision
            if can_probe and tag == 1:
                leg['tag1_before_transfer_probe'] = miss
                leg['tag1_transfer_probe_decision'] = decision
            if decision['view_change_allowed']:
                explanation = (f'Tag {tag} 有部分帧且仍偏远，先走下一段原定的两步，再扫码。'
                               if decision['geometry'] else 'Tag 1 暂不可见，先走中转区原定的两步，再扫码。')
                _report_progress(robot, 'cmd2_return_leg', expected_tag=tag, last_passed_tag=previous,
                                 return_step='transfer_tag1_probe' if tag == 1 else 'base_exit_tag5_probe',
                                 reason_zh=explanation)
                probe_actions = []
                if tag == 1:
                    leg['tag1_transfer_probe'] = upcoming[0]
                    probe_actions = result['transfer_approach_actions']
                _move(robot, nav, upcoming[:1], probe_actions, scan_follows=True,
                      station_actions=result['station_actions'] if tag == 1 else None)
                leg['forward_probe_action'] = upcoming[0]
                route_actions[index+1] = upcoming[1:]
            if decision['view_change_allowed'] or (can_probe and tag == 1):
                leg.pop('checkpoint_not_seen', None)
                observed, reason = _observe_departure(robot, nav, profile, leg, previous)
        if observed is CheckpointScanResult.CHECKPOINT_NOT_SEEN:
            miss = leg['checkpoint_not_seen']
            healthy = bool(miss.get('healthy_windows') and miss['healthy_windows'] == miss['observation_windows']
                           and not miss.get('temporary_camera_gaps'))
            _return_repay_search(robot, nav, leg, camera_ok=healthy)
            result['missing_checkpoints'].append(leg['checkpoint_not_seen'])
            leg['departure_resolution'] = 'continue_estimated_after_missed_tag'
            continue
        if estimate and observed in (CheckpointScanResult.MOTION_NOT_STARTED, CheckpointScanResult.CAMERA_UNAVAILABLE):
            if leg.get('detected') and observed is CheckpointScanResult.MOTION_NOT_STARTED:
                _align_return_departure(robot, nav, leg)
                result['checkpoint'] = tag
            else:
                result['missing_checkpoints'].append(leg.get('checkpoint_not_seen') or
                    {'tag_id': tag, 'cause': observed.value, 'detail': reason})
            _return_repay_search(robot, nav, leg, camera_ok=observed is not CheckpointScanResult.CAMERA_UNAVAILABLE)
            leg.update(departure_resolution='continue_estimated_after_recovery', departure_reason=reason)
            continue
        if observed is CheckpointScanResult.MOTION_NOT_STARTED:
            return set_status(ReturnTrialResult.CHECKPOINT_UNCONFIRMED, reason)
        if observed is not CheckpointScanResult.CHECKPOINT_DETECTED:
            return set_status(ReturnTrialResult.CAMERA_UNAVAILABLE, reason)
        _align_return_departure(robot, nav, leg)
        _return_repay_search(robot, nav, leg, camera_ok=True)
        result['checkpoint'] = tag

    if result['checkpoint'] == 1:
        result.update(station_reached=True, pickup_anchor_tag=1)
        reason = '已确认返程 Tag 1，准备前进到接驳区掉头位置。'
    elif estimate:
        result.update(station_reached=True, estimated_pickup_anchor_tag=1)
        reason = '返程已走完，到 Tag 1 找码后仍无可靠位姿，按预设步数前进去接驳。'
    else:
        return set_status(ReturnTrialResult.CHECKPOINT_UNCONFIRMED, '返程未确认 Tag 1，接驳区尚未定位。')
    _report_progress(robot, 'cmd2_return_station', expected_tag=None, last_passed_tag=result['checkpoint'],
                     return_step='transfer_approach', reason_zh=reason)
    return set_status(ReturnTrialResult.STATION_REACHED, reason)
