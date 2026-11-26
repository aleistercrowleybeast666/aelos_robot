"""Select and verify loaded or empty route turns using fresh checkpoint poses."""
import math
from enum import Enum

from checkpoint_errors import VisualCamera_GetFreshnessCheck, VisualCamera_IsFresh
from checkpoint_scan import Checkpoint_Observe, CheckpointScanResult
from heading_guard import MagneticDataUnavailable, MagneticLoss_IsRecoverable
from nav_observation import wrap
from route_pose import RoutePose_PartialCheckpointGeometry


_TAG3_VISUAL_MICRO_TRIGGER_DEG = 7.
# One nominal 30-degree empty turn, with 45-degree upper response and 3-degree uncertainty.
EMPTY_VISUAL_TURN_TRIGGER_DEG = 25.5


class RouteTagHeadingRebaseResult(Enum):
    APPLIED = 'applied'
    UNAVAILABLE = 'unavailable'


def Checkpoint_TurnFromTagHeading(robot, tag_id, error, observed_at_ns, *, reserved_actions=0):
    """Submit one camera-guided micro turn without relying on a magnetic reference."""
    from micro_trial import OPS
    from turn_estimator import MICRO_INITIAL_DEG
    paired_route = robot.load_kind == 'double' and getattr(robot,'pair_route_enabled',False)
    if (tag_id not in (1,2,3,4) or (robot.load_kind!='single' and not paired_route)
            or not robot._allows_missing_magnetic()
            or getattr(robot.backend,'is_simulation',False)
            or type(error) not in (int,float) or not math.isfinite(error)):
        return None
    nav=robot.ensure_navigation_io()
    cfg=nav.cfg
    visual=cfg.get('visual_micro',{})
    goal=next((point for point in cfg['legs']['outbound'] if point['id']==tag_id),{})
    if (visual.get('execute') is not True or visual.get('target_id_confirmed') is not True
            or visual.get('direction_evidence')!='measured_r3_tag' or goal.get('allow_turn') is not True
            or (tag_id == 1 and abs(error)>visual['max_error_deg'])
            or abs(error) >= 90.):
        # At a corridor marker the caller selects the recovery using the
        # fresh pose and clearance. The requested action is one micro turn.
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
        initial = robot.cfg.get('route_steps', {}).get('micro_turn_filter', {}).get('initial_deg', MICRO_INITIAL_DEG)
        selection_response = min(bound, initial)
        estimator = getattr(robot.heading_guard, 'turn_estimator', None)
        anchor = nav.mag.anchor or {}
        direction = 'left' if gains[name] < 0 else 'right'
        measured = estimator.control_estimate(direction, 'micro', robot.load_kind,
            getattr(robot.backend,'session',''), getattr(robot.backend,'epoch',-1),
            anchor.get('calibration_id',''), now_s=robot.clock()) if estimator is not None else None
        if measured is not None:
            selection_response = abs(measured['degrees'])
    if abs(abs(error)-selection_response)>abs(error)-improvement:
        return None
    timeout=robot.cfg['request_timeout_s']
    route_feedback = tag_id in (2, 3, 4) and robot.cfg.get('route_steps', {}).get('continue_estimated_route') is True
    if (robot.count+reserved_actions+1>robot.cfg['max_body_macros_per_journey']
            or (not route_feedback and robot.nav_count>=cfg['max_extra_actions_per_journey'])
            or robot.deadline-robot.clock()<=timeout+cfg['settle_s']+cfg['observe_s']):
        return None
    robot.check()
    checker=getattr(robot.backend,'check_micro',None)
    live=VisualCamera_GetFreshnessCheck(nav.io)
    if (checker is None or live is None or type(observed_at_ns) is not int
            or (tag_id in (2, 3, 4) and type(robot.last_action_completed_ros_ns) is int
                and observed_at_ns <= robot.last_action_completed_ros_ns)):
        return None
    checker()
    if not live(observed_at_ns) or not robot.micro_budget().reserve(
            'visual',name,gains[name],conservative_bound=bound,route_visual_feedback=route_feedback):
        return None
    robot._complete_body_action(name,timeout)
    robot.nav_count+=1
    robot.last_heading_report=None
    robot.events.append({'visual_micro_action':name,'tag_id':tag_id,'feedback_source':'tag_only',
                        'angle_evidence':'source_nominal','physical_success':'unobserved'})
    turn = {'action':name,'before_error_deg':error}
    if tag_id in (2, 3, 4):
        turn['selected_response_deg'] = selection_response
    return turn


def _capture_ready_for_reference(io):
    health = getattr(io, 'magnetic_capture_health', None)
    retry = getattr(io, 'retry_magnetic_preflight', None)
    return (callable(health) and MagneticLoss_IsRecoverable(health())
            and (getattr(io, 'motion_capture_verified', False) or (callable(retry) and retry())))


def Checkpoint_RebaseTagHeading(robot, tag_id, error_deg, observed_at_ns, *, direction=None):
    """Keep a fresh expected Tag's yaw error in the magnetic target for this segment."""
    unavailable = RouteTagHeadingRebaseResult.UNAVAILABLE
    bundle = getattr(robot,'navigation_bundle',None)
    empty_return = (robot.load_kind == 'empty' and getattr(robot,'route_command',None) == '2'
                    and ((direction == 'return' and tag_id in (5,6,7,1))
                         or (direction == 'outbound' and tag_id == 1)))
    loaded_route = direction is None and robot.load_kind in ('single','double') and tag_id in (1,2,3,4,5)
    if (not (empty_return or loaded_route) or not robot._allows_missing_magnetic()
            or getattr(robot.backend,'is_simulation',False)
            or not bundle or not bundle['post_action_heading'].get('enabled')
            or type(error_deg) not in (int,float) or not math.isfinite(error_deg) or abs(error_deg)>180
            or type(observed_at_ns) is not int or observed_at_ns<=0):
        return unavailable
    completed_ns = robot.last_action_completed_ros_ns
    if type(completed_ns) is int and observed_at_ns<=completed_ns:
        return unavailable
    robot.check()
    nav = robot.ensure_navigation_io()
    goal = next((point for point in nav.cfg['legs'][direction or 'outbound'] if point['id']==tag_id),None)
    live = VisualCamera_GetFreshnessCheck(nav.io)
    if goal is None or live is None or not live(observed_at_ns):
        return unavailable
    heading = {'tag_id':tag_id,'route_heading_deg':goal['route_heading_deg'],
               'error_deg':error_deg,'observed_at_ns':observed_at_ns}
    if robot.magnetic_route_only and (not _capture_ready_for_reference(nav.io) or not live(observed_at_ns)):
        robot.events.append({'segment_heading_reference':{'status':'reference_unavailable', 'tag_departure':heading}})
        return unavailable
    guard = robot._heading_controller(nav)
    try:
        report = guard.before_straight(robot.load_kind,robot.last_action_completed_monotonic,
                                       tag_departure=heading)
    except MagneticDataUnavailable as exc:
        if not robot.defer_magnetic_route_fault(exc):
            raise
        robot.events.append({'segment_heading_reference':{'status':'reference_unavailable',
                                                         'tag_departure':heading}})
        return unavailable
    robot.events.append({'segment_heading_reference':dict(report,tag_departure=heading)})
    if report['status']=='tag_departure_reference_established':
        robot.magnetic_route_only=False
        robot.last_heading_report=None
        if loaded_route and tag_id == 5 and not guard.budget.locked:
            # An empty sample window can block feedback without locking its
            # budget. The new same-stance sample supersedes that observation.
            guard.blocked = False
        return RouteTagHeadingRebaseResult.APPLIED
    return unavailable


def Checkpoint_TrackEntryHeading(robot, nav, saved_anchor=None):
    """Anchor a stationary entry or recover the exact same target after an interruption."""
    unavailable = RouteTagHeadingRebaseResult.UNAVAILABLE
    guard = getattr(robot, 'heading_guard', None)
    if (guard is None or robot.load_kind not in ('single', 'double')
            or not getattr(robot, '_allows_missing_magnetic', lambda: False)()):
        return unavailable
    robot.check()
    if not _capture_ready_for_reference(nav.io):
        return unavailable
    guard = robot._heading_controller(nav)
    prior = nav.mag.anchor
    applied = False
    try:
        if saved_anchor is None:
            # Only the relative drift AFTER this stance is observable.
            nav.mag.anchor = None
            sample = guard.before_straight(robot.load_kind, robot.last_action_completed_monotonic)
            target = nav.mag.anchor or {}
            applied = (sample.get('status') == 'relative_reference_established'
                       and target.get('capture_verified') and target.get('usable_for_heading')
                       and target.get('offset_deg') is not None)
        else:
            if saved_anchor.get('owner_observation_scope') != guard.observation_scope:
                return unavailable
            _, sample = guard.sample(robot.load_kind, robot.last_action_completed_monotonic, stationary_retry=True)
            nav.mag.anchor = dict(saved_anchor)
            applied = not guard._target_observation_faults(sample, robot.load_kind)
        if applied:
            robot.magnetic_route_only = False
            robot.last_heading_report = None
            if not guard.budget.locked:
                guard.blocked = False
            return RouteTagHeadingRebaseResult.APPLIED
    except MagneticDataUnavailable as exc:
        if not robot.defer_magnetic_route_fault(exc):
            raise
    finally:
        if not applied:
            nav.mag.anchor = None if robot.magnetic_route_only else prior
    return unavailable


def Checkpoint_TurnAtTag5(robot, name, observed_at_ns):
    """One visual-selected micro turn at the gate, followed by a fresh camera check."""
    if name not in ('single_micro_left','single_micro_right') or robot.load_kind not in ('single','double'):
        raise ValueError('Tag 5 requires a loaded micro turn')
    robot.check()
    nav = robot.ensure_navigation_io()
    timeout = robot.cfg['request_timeout_s']
    goal = next((point for point in nav.cfg['legs']['outbound'] if point['id'] == 5), {})
    passage = robot.cfg['route_steps']['tag5_entry_lateral']
    if goal.get('allow_turn') is not True or passage.get('verify_passage') is not True:
        return False
    if (robot.count >= robot.cfg['max_body_macros_per_journey']
            or robot.deadline-robot.clock() <= timeout + nav.cfg['observe_s']):
        return False
    robot.backend.check_micro()
    if not VisualCamera_IsFresh(nav.io.visual_motion_ready, observed_at_ns):
        return False
    robot.last_heading_report = None
    robot._complete_body_action(name,timeout)
    robot.monitor_heading('corridor_side_boundary',allow_correction=False)
    return True


def Checkpoint_MeasureEmptyMarker(robot, nav, leg):
    scan = leg.get('scan') or {}
    tag = leg['expected_id']
    pinned = nav.cfg.get('visual_micro', {}).get('camera_info', {})
    camera = leg.get('camera_info') or {}
    stamp = robot.last_action_completed_ros_ns
    floor, now = scan.get('after_ns'), scan.get('now_ns')
    live = VisualCamera_GetFreshnessCheck(nav.io)
    if (robot.load_kind != 'empty' or live is None or type(stamp) is not int
            or type(floor) is not int or type(now) is not int or not stamp <= floor < now
            or pinned.get('source_confirmed') is not True or not pinned.get('expected_fingerprint')
            or camera.get('ready') is not True or camera.get('fingerprint') != pinned['expected_fingerprint']
            or scan.get('ids') != [tag] or not scan.get('contract_match') or not scan.get('camera_healthy')):
        return None
    rows = [row for row in scan.get('observations', ()) if type(row.get('stamp_ns')) is int
            and floor < row['stamp_ns'] <= now and now - row['stamp_ns'] <= nav.cfg['max_age_s'] * 1e9]
    geometry = RoutePose_PartialCheckpointGeometry(rows, nav.cfg, tag)
    if (geometry and geometry['samples'] >= nav.cfg['min_frames'] and geometry['max_heading_deviation_deg'] <= 5.
            and Checkpoint_EmptyMarkerCurrent(robot, nav, tag, geometry['latest_stamp_ns'])):
        return geometry
    return None


def Checkpoint_EmptyMarkerCurrent(robot, nav, tag, stamp):
    try:
        return nav.io.visual_motion_ready(stamp)
    except OSError as exc:
        robot.check()
        robot.events.append({'return_camera_read_error': {'tag_id': tag, 'reason': str(exc)}})
        return False


class _EmptyMarkerExpired(Exception):
    pass


def Checkpoint_AlignEmptyMarker(robot, nav, leg, *, direction='return'):
    """Correct the heading of the upright, empty body using the expected floor-marker orientation."""
    tag = leg['expected_id']
    goal = next((goal for goal in nav.cfg['legs'].get(direction, ()) if goal['id'] == tag), {})
    goal_yaw = goal.get('tag_heading_deg')
    if goal_yaw is None:
        return
    heading = {'direction': direction, 'expected_yaw_deg': goal_yaw, 'observations': [], 'actions': []}
    leg['empty_heading'] = heading
    current = leg
    for attempt in range(3):
        geometry = Checkpoint_MeasureEmptyMarker(robot, nav, current)
        if geometry is None:
            heading['status'] = 'pose_unavailable'
            return
        error = wrap(geometry['heading_deg'] - goal_yaw)
        stamp = geometry['latest_stamp_ns']
        reading = {'error_deg': error, 'latest_stamp_ns': stamp, 'spread_deg': geometry['max_heading_deviation_deg']}
        heading['observations'].append(reading)
        if abs(error) >= 90.:
            heading['status'] = 'orientation_inconsistent'
            return
        rebase = getattr(robot, 'Route_RebaseTagHeading', None)
        if rebase is not None:
            reading['magnetic_reference'] = rebase(tag, error, stamp, direction=direction).value
        # The available empty turn is nominally 30 degrees, with an estimated outer
        # response of 45 degrees. Small residuals travel with the corrected magnetic target.
        if abs(error) - geometry['max_heading_deviation_deg'] <= EMPTY_VISUAL_TURN_TRIGGER_DEG:
            heading['status'] = 'within_empty_turn_resolution'
            return
        previous = heading['observations'][-2] if attempt else None
        # With several consistent frames, an improvement smaller than three degrees
        # is usable if the pre/post yaw intervals no longer overlap.
        progress = previous is None or (previous['error_deg']*error > 0 and stamp > previous['latest_stamp_ns']
            and abs(previous['error_deg']) - abs(error) > min(3., previous['spread_deg'] + reading['spread_deg']))
        if (attempt == 2 or not progress or goal.get('allow_turn') is not True
                or robot.nav_count >= nav.cfg['max_extra_actions_per_journey']
                or robot.count >= robot.cfg['max_body_macros_per_journey']):
            heading['status'] = 'retain_measured_residual'
            return
        turn = 'empty_left_turn' if error > 0 else 'empty_right_turn'

        def verify():
            robot.check()
            if not Checkpoint_EmptyMarkerCurrent(robot, nav, tag, stamp):
                raise _EmptyMarkerExpired()

        try:
            robot.finite(turn, configured=True, checkpoint_follows=True, before_dispatch=verify)
        except _EmptyMarkerExpired:
            heading['status'] = 'pose_expired_before_turn'
            return
        robot.nav_count += 1
        heading['actions'].append(turn)
        current = {'expected_id': tag, 'observation_windows': 0, 'detected': False}
        observed, _ = Checkpoint_Observe(robot, nav, current, lambda: None,
            quick_id=True, single_window=True, reuse_settled_cache=True)
        # Do not pass pre-turn positions on to route fusion after the stance changed.
        leg.update(scan=current.get('scan'), camera_info=current.get('camera_info') or {},
                   detected=observed is CheckpointScanResult.CHECKPOINT_DETECTED)
        if observed is not CheckpointScanResult.CHECKPOINT_DETECTED:
            heading['status'] = 'marker_lost_after_turn'
            return
