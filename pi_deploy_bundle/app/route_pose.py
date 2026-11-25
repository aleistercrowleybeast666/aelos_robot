"""Optional metric checkpoint guidance using a separately measured profile per direction."""

import json
from enum import Enum
from math import radians, sin, tan
from pathlib import Path
from statistics import median

from cmd0_motion import COMMAND0_ENTRY_FORWARD_STEPS, SMALL_SIDE_ACTIONS
from entry_camera import finite
from nav_observation import circular_median, ground_pose, summarize, wrap
from pixel_diagnostics import load_entrance_reference


class RoutePoseMeasureResult(Enum):
    UNAVAILABLE = 'pose_unavailable'
    MEASURED = 'measured'


class RoutePosePlanResult(Enum):
    NOMINAL = 'nominal'
    ADJUSTED = 'adjusted'


class RoutePoseDepartureResult(Enum):
    UNVERIFIED = 'nominal_unverified'
    FUSE = 'fuse_or_continue'
    FINE_ADJUSTMENT = 'fine_adjustment_needed'
    OUTSIDE_ENVELOPE = 'outside_departure_envelope'
    REOBSERVE = 'reobserve_needed'


_SCHEMA = 'r3.route-pose.v1'
_SIDE_SIGNS = {'strafe_left_once': -1, 'strafe_right_once': 1}
_TAG2_FAR_APPROACH_LIMIT_M = 0.40
TAG2_APPROACH_COUNTS = (2, 4, 5, 6, 8)
_TAG1_NOMINAL_START_M = (0.14, 0.22)
_TAG1_TRIAL_TARGET_M = 0.18
_TAG1_TRIAL_EFFECTIVE_START_M = (0.13, 0.28)
_TAG1_FAR_OBSERVATION_M = 0.45
_TAG1_FAR_EFFECTIVE_START_M = (0.13, 0.33)


def RoutePose_Load(project):
    """Load optional operator calibration. A missing file leaves the nominal route."""
    photo = load_entrance_reference(project)
    template = None
    if photo is not None:
        evidence = photo.get('evidence')
        template = {'reference_tag_id': photo['target_id'], 'load': photo['load'],
                    'tag_center_px': photo['tag_center_px'],
                    'image': evidence.get('image') if isinstance(evidence, dict) else None,
                    'camera': photo['camera'], 'diagnostic_only': True}
    try:
        profile = json.loads((Path(project) / 'config' / 'route_pose.json').read_text(encoding='utf-8'))
        if profile['schema'] != _SCHEMA or not isinstance(profile['outbound'], dict):
            raise ValueError('unsupported route pose schema')
        return dict(profile, pose_template=template, config_status='loaded')
    except (OSError, KeyError, ValueError, TypeError):
        return {'outbound': {}, 'pose_template': template, 'config_status': 'missing_or_invalid'}


def _tag_profile(profile, tag_id, direction='outbound'):
    leg = profile.get(direction, {}) if isinstance(profile, dict) else {}
    tag = leg.get(str(tag_id), {}) if isinstance(leg, dict) else {}
    return tag if isinstance(tag, dict) else {}


def _interval(values):
    return (isinstance(values, (tuple, list)) and len(values) == 2
            and all(finite(value) for value in values) and values[0] < values[1])


def _camera_fingerprint(nav_cfg, camera_info):
    try:
        pinned = nav_cfg['visual_micro']['camera_info']
        fingerprint = pinned['expected_fingerprint']
        if (pinned['source_confirmed'] is True
                and isinstance(fingerprint, str) and len(fingerprint) == 64
                and all(char in '0123456789abcdef' for char in fingerprint)
                and camera_info.get('ready') is True and camera_info.get('fingerprint') == fingerprint):
            return fingerprint
    except (KeyError, TypeError, AttributeError):
        pass
    return None


def RoutePose_Measure(scan, nav_cfg, profile, tag_id, *, camera_info, direction='outbound'):
    """Read metric x/y/yaw from a fresh, same-load and same-camera window."""
    tag = _tag_profile(profile, tag_id, direction)
    template = profile.get('pose_template') if isinstance(profile, dict) else None
    result = {'status': RoutePoseMeasureResult.UNAVAILABLE.value, 'tag_id': tag_id, 'direction': direction,
              'position_usable': False, 'departure_axis': tag.get('departure_axis'),
              'x_m': None, 'x_error_m': None, 'forward_m': None, 'forward_status': 'unavailable',
              'yaw_deg': None, 'body_right_in_tag_right': None, 'body_forward_in_tag_forward': None,
              'pose_template': dict(template, taught_for_tag=(direction == 'outbound'
                                                              and template.get('reference_tag_id') == tag_id))
              if isinstance(template, dict) else None}
    load = {'outbound': 'single', 'return': 'empty'}.get(direction)
    if (load is None or tag.get('measurement_verified') is not True or tag.get('expected_load') != load
            or not _interval(tag.get('forward_window_m'))
            or not finite(tag.get('lateral_deadband_m')) or tag['lateral_deadband_m'] <= 0
            or not finite(tag.get('lateral_uncertainty_m')) or tag['lateral_uncertainty_m'] < 0):
        return dict(result, reason='same-load metric checkpoint calibration unavailable')
    if not isinstance(scan, dict) or scan.get('expected_load') != tag['expected_load']:
        return dict(result, reason='checkpoint observation load does not match calibration')
    if not isinstance(nav_cfg, dict):
        return dict(result, reason='navigation pose contract unavailable')
    contract = nav_cfg.get('pose_contract')
    if (not isinstance(contract, dict)
            or any(contract.get(key) is not True for key in ('axes_confirmed', 'capture_time_confirmed', 'scale_confirmed'))):
        return dict(result, reason='navigation axes, capture time or metric scale unverified')
    fingerprint = _camera_fingerprint(nav_cfg, camera_info)
    if fingerprint is None or tag.get('camera_fingerprint') != fingerprint:
        return dict(result, reason='verified checkpoint CameraInfo fingerprint unavailable or changed')
    after = scan.get('after_ns')
    now = scan.get('now_ns')
    if (scan.get('contract_match') is not True or scan.get('camera_healthy') is not True
            or type(after) is not int or type(now) is not int or not 0 < after < now
            or not isinstance(scan.get('observations'), (list, tuple))):
        return dict(result, reason='fresh checkpoint window or motion barrier unavailable')
    try:
        goal = next(goal for goal in nav_cfg['legs'][direction] if goal['id'] == tag_id)
        metric = summarize(scan, tag_id, dict(nav_cfg, relative_heading_reference={}), after)
        target_x = goal['lateral_m']
        target_yaw = goal['tag_heading_deg']
        tolerance = min(nav_cfg['deadband_heading_deg'], nav_cfg['post_action_heading']['deadband_deg'])
    except (KeyError, TypeError, ValueError, AttributeError, StopIteration):
        return dict(result, reason='invalid metric checkpoint observation or goal')
    if (metric.get('ids') != [tag_id] or metric.get('position_usable') is not True
            or not all(finite(metric.get(key)) for key in (
                'x_m', 'forward_m', 'heading_deg', 'x_mad_m', 'forward_mad_m',
                'heading_mad_deg', 'body_right_in_tag_right'))
            or not all(finite(value) for value in (target_x, target_yaw, tolerance)) or tolerance <= 0):
        return dict(result, reason=metric.get('reason', 'fresh trusted metric checkpoint pose unavailable'))
    x_m, forward_m, yaw_deg = (metric[key] for key in ('x_m', 'forward_m', 'heading_deg'))
    near, far = tag['forward_window_m']
    forward_status = 'inside_window'
    if forward_m < near:
        forward_status = 'below_window'
    elif forward_m > far:
        forward_status = 'above_window'
    yaw_error = wrap(yaw_deg - target_yaw)
    return dict(result, status=RoutePoseMeasureResult.MEASURED.value, position_usable=True,
                reason='fresh metric pose in the expected tag frame', x_m=x_m, forward_m=forward_m,
                x_error_m=x_m - target_x, x_uncertainty_m=max(tag['lateral_uncertainty_m'], metric['x_mad_m']),
                forward_status=forward_status, forward_window_m=[near, far],
                forward_mad_m=metric['forward_mad_m'], heading_mad_deg=metric['heading_mad_deg'],
                yaw_deg=yaw_deg, yaw_error_deg=yaw_error,
                heading_tolerance_deg=tolerance,
                heading_within_window=abs(yaw_error) <= tolerance,
                body_forward_in_tag_forward=metric.get('body_forward_in_tag_forward'),
                body_right_in_tag_right=metric['body_right_in_tag_right'],
                latest_stamp_ns=metric['latest_stamp_ns'], goal_allows_turn=goal.get('allow_turn') is True,
                goal_allows_strafe=goal.get('allow_strafe') is True)


def _within(interval, bounds):
    return bounds[0] <= interval[0] <= interval[1] <= bounds[1]


def _departure_lateral_step(pose, tag, observed, soft_limit):
    """Identify an individually verified pre-departure step that fixes this residual."""
    side = tag.get('side_adjustment')
    small = side.get('small_step') if isinstance(side, dict) else None
    projection = pose.get('body_right_in_tag_right')
    if (not isinstance(small, dict) or small.get('verified') is not True
            or small.get('allow_departure') is not True or pose.get('goal_allows_strafe') is not True
            or not finite(projection) or not 0 < abs(projection) <= 1 + 1e-8):
        return None
    operation = side.get('operation')
    distance = small.get('body_right_displacement_m')
    improvement = side.get('minimum_improvement_m')
    if (not isinstance(operation, str) or operation not in SMALL_SIDE_ACTIONS
            or small.get('operation') != SMALL_SIDE_ACTIONS[operation]
            or not _signed_side_interval(distance, operation)
            or not finite(improvement) or improvement <= 0):
        return None
    gain, landing = _projected_effect({'projection': projection, 'residual': observed}, distance)
    if gain >= improvement and landing <= soft_limit:
        return small['operation']
    return None


def RoutePose_AdviseDeparture(pose, profile, tag_id, *, direction='outbound', advisory_heading=False):
    """Separate ordinary fusion from pose errors that need a new stationary observation."""
    tag = _tag_profile(profile, tag_id, direction)
    window = tag.get('departure_window')
    result = {'tag_id': tag_id, 'direction': direction, 'axis_status': {},
              'blocking_axes': [], 'action': None, 'micro_operation': None, 'heading_error_deg': None}
    if not isinstance(window, dict) or window.get('verified') is not True:
        return dict(result, status=RoutePoseDepartureResult.UNVERIFIED.value,
                    reason='measured departure envelope unavailable; use existing checkpoint policy')
    if (not isinstance(pose, dict) or pose.get('status') != RoutePoseMeasureResult.MEASURED.value
            or pose.get('tag_id') != tag_id or pose.get('direction', 'outbound') != direction
            or pose.get('position_usable') is not True):
        return dict(result, status=RoutePoseDepartureResult.REOBSERVE.value,
                    action='reobserve_before_translation', reason='verified departure requires a fresh trusted pose')
    forward, x, yaw = (pose.get(key) for key in ('forward_m', 'x_error_m', 'yaw_error_deg'))
    forward_floor = window.get('forward_uncertainty_m', 0)
    yaw_floor = window.get('heading_uncertainty_deg', 0)
    forward_mad, x_err, yaw_mad = (pose.get(key) for key in ('forward_mad_m', 'x_uncertainty_m', 'heading_mad_deg'))
    forward_soft, forward_hard = pose.get('forward_window_m'), window.get('forward_hard_m')
    x_soft, x_hard = window.get('lateral_fuse_m'), window.get('lateral_hard_m')
    yaw_soft, yaw_hard = pose.get('heading_tolerance_deg'), window.get('heading_hard_deg')
    deadband = tag.get('lateral_deadband_m')
    if (not all(finite(value) for value in (forward, x, yaw, forward_floor, forward_mad, x_err, yaw_floor, yaw_mad,
                                            x_soft, x_hard, yaw_soft, yaw_hard, deadband))
            or min(forward_floor, forward_mad, x_err, yaw_floor, yaw_mad) < 0
            or not (0 < deadband <= x_soft < x_hard and 0 < yaw_soft < yaw_hard <= 180)
            or not _interval(forward_soft) or not _interval(forward_hard) or not _within(forward_soft, forward_hard)):
        return dict(result, status=RoutePoseDepartureResult.REOBSERVE.value,
                    action='reobserve_before_translation', reason='invalid verified departure geometry or uncertainty')
    forward_err, yaw_err = max(forward_floor, forward_mad), max(yaw_floor, yaw_mad)
    axes = {
        'forward': ((forward - forward_err, forward + forward_err), forward_soft, forward_hard),
        'lateral': ((x - x_err, x + x_err), (-x_soft, x_soft), (-x_hard, x_hard)),
        'heading': ((yaw - yaw_err, yaw + yaw_err), (-yaw_soft, yaw_soft), (-yaw_hard, yaw_hard)),
    }
    result['axis_status'] = {axis: ('outside_hard' if not _within(observed, hard)
                                    else 'inside_soft' if _within(observed, soft) else 'outside_soft')
                             for axis, (observed, soft, hard) in axes.items()}
    result['axis_intervals'] = {axis: {'observed': observed, 'soft': soft, 'hard': hard}
                                for axis, (observed, soft, hard) in axes.items()}
    result['blocking_axes'] = [axis for axis, status in result['axis_status'].items()
                               if status != 'inside_soft' and (not advisory_heading or axis != 'heading')]
    result['heading_error_deg'] = yaw
    if any(result['axis_status'][axis] == 'outside_hard' for axis in result['blocking_axes']):
        return dict(result, status=RoutePoseDepartureResult.OUTSIDE_ENVELOPE.value,
                    action='reobserve_before_translation', reason='departure estimate exceeds measured clearance envelope')
    if advisory_heading and not result['blocking_axes'] and result['axis_status']['heading'] != 'inside_soft':
        action = 'heading_micro_after_reobserve' if (pose.get('goal_allows_turn') is True
                                                     and abs(yaw) - yaw_err > yaw_soft) else None
        return dict(result, status=RoutePoseDepartureResult.FINE_ADJUSTMENT.value,
                    action=action, heading_advisory_only=True,
                    reason='verified position inside the approach window; heading available for correction')
    if not result['blocking_axes']:
        return dict(result, status=RoutePoseDepartureResult.FUSE.value,
                    reason='pose inside measured fusion window; absorb residual in an approved planned segment')

    # An interval overlapping the soft window is not proof that a nudge helps.
    if (result['blocking_axes'] == ['heading'] and pose.get('goal_allows_turn') is True
            and abs(yaw) - yaw_err > yaw_soft):
        result['action'] = 'heading_micro_after_reobserve'
    elif result['blocking_axes'] == ['lateral'] and abs(x) - x_err > x_soft:
        result['action'] = 'lateral_micro_after_reobserve'
        if direction == 'outbound':
            result['micro_operation'] = _departure_lateral_step(pose, tag, axes['lateral'][0], x_soft)
    else:
        result['action'] = 'reobserve_before_translation'
    return dict(result, status=RoutePoseDepartureResult.FINE_ADJUSTMENT.value,
                reason='outside fusion tolerance but within measured clearance; reassess before translation')


def _side_plan_context(pose, profile, tag_id, op, nominal):
    result = {'status': RoutePosePlanResult.NOMINAL.value, 'tag_id': tag_id,
              'operation': op, 'nominal_count': nominal, 'count': nominal}
    tag = _tag_profile(profile, tag_id)
    if (tag_id not in (2, 3, 4) or tag.get('departure_axis') != 'lateral'
            or not isinstance(op, str) or op not in _SIDE_SIGNS
            or type(nominal) is not int or nominal < 1):
        return dict(result, reason='segment uses its original motion pattern'), None
    side = tag.get('side_adjustment')
    if (not isinstance(side, dict) or side.get('verified') is not True
            or side.get('operation') != op or side.get('nominal_count') != nominal):
        return dict(result, reason='same-load side displacement and clearance unverified for this segment'), None
    low_count, high_count = side.get('min_count'), side.get('max_count')
    improvement = side.get('minimum_improvement_m')
    landing_limit = side.get('max_landing_x_error_m')
    if (type(low_count) is not int or type(high_count) is not int or not 1 <= low_count <= nominal <= high_count
            or not finite(improvement) or improvement <= 0
            or not finite(landing_limit) or landing_limit <= 0):
        return dict(result, reason='measured next-checkpoint visibility, clearance or improvement bounds unavailable'), None
    if (not isinstance(pose, dict) or pose.get('status') != RoutePoseMeasureResult.MEASURED.value
            or pose.get('tag_id') != tag_id or pose.get('position_usable') is not True
            or pose.get('goal_allows_strafe') is not True):
        return dict(result, reason='current checkpoint lacks a trusted lateral pose'), None
    if pose.get('forward_status') != 'inside_window' or pose.get('heading_within_window') is not True:
        return dict(result, reason='forward or heading outside the measured corridor adjustment window'), None
    advice = RoutePose_AdviseDeparture(pose, profile, tag_id)
    if advice['status'] not in (RoutePoseDepartureResult.UNVERIFIED.value, RoutePoseDepartureResult.FUSE.value):
        return dict(result, reason=advice['reason'], departure_status=advice['status']), None
    error, uncertainty, projection = (pose.get(key) for key in (
        'x_error_m', 'x_uncertainty_m', 'body_right_in_tag_right'))
    deadband = tag.get('lateral_deadband_m')
    if (not all(finite(value) for value in (error, uncertainty, projection, deadband))
            or uncertainty < 0 or deadband <= 0 or not 0 < abs(projection) <= 1 + 1e-8):
        return dict(result, reason='invalid lateral estimate or body-axis projection'), None
    result['calibrated_pose_used'] = _signed_side_interval(side.get('body_right_displacement_m'), op)
    if abs(error) - uncertainty <= deadband:
        return dict(result, reason='lateral residual inside or overlapping the tolerance'), None
    return result, {'side': side, 'error': error, 'projection': projection,
                    'residual': (error - uncertainty, error + uncertainty)}


def _signed_side_interval(displacement, op):
    return _interval(displacement) and all(delta * _SIDE_SIGNS[op] > 0 for delta in displacement)


def _projected_effect(context, changes):
    projected = [context['projection'] * change for change in changes]
    effects = [(abs(x), max(abs(x + dx) for dx in projected)) for x in context['residual']]
    return min(before - after for before, after in effects), max(after for _, after in effects)


def RoutePose_PlanTag2ForwardTrim(pose, profile):
    """Select one measured two-step longitudinal move before the Tag 2 strafe."""
    result = {'status': RoutePosePlanResult.NOMINAL.value, 'tag_id': 2, 'operation': None}
    tag = _tag_profile(profile, 2)
    trim = tag.get('forward_trim')
    if not isinstance(trim, dict) or trim.get('verified') is not True:
        return dict(result, reason='Tag 2 two-step longitudinal reference unverified')
    target, deadband = (trim.get(key) for key in ('target_forward_m', 'deadband_m'))
    window = tag.get('forward_window_m')
    if (not _interval(window) or not all(finite(x) for x in (target, deadband))
            or deadband <= 0 or not window[0] <= target <= window[1]):
        return dict(result, reason='Tag 2 longitudinal target or clearance unverified')
    if (not isinstance(pose, dict) or pose.get('status') != RoutePoseMeasureResult.MEASURED.value
            or pose.get('tag_id') != 2 or pose.get('direction') != 'outbound'
            or pose.get('position_usable') is not True or pose.get('heading_within_window') is not True):
        return dict(result, reason='fresh same-load Tag 2 longitudinal pose unavailable')
    advice = RoutePose_AdviseDeparture(pose, profile, 2)
    if advice['status'] not in (RoutePoseDepartureResult.UNVERIFIED.value, RoutePoseDepartureResult.FUSE.value):
        return dict(result, reason=advice['reason'])
    position, uncertainty, projection = (pose.get(key) for key in
        ('forward_m', 'forward_mad_m', 'body_forward_in_tag_forward'))
    if (not all(finite(x) for x in (position, uncertainty, projection))
            or uncertainty < 0 or not 0 < abs(projection) <= 1 + 1e-8):
        return dict(result, reason='Tag 2 longitudinal axis or uncertainty unverified')
    # The deadband also prevents stepping on a sign that could be measurement noise.
    uncertainty = max(uncertainty, deadband / 2)
    error = position - target
    result.update(forward_error_m=error, forward_uncertainty_m=uncertainty)
    if abs(error) <= deadband + uncertainty or not _within(
            (position - uncertainty, position + uncertainty), window):
        return dict(result, reason='Tag 2 longitudinal error inside tolerance or outside surveyed clearance')
    forward = error * projection < 0
    distance = trim.get('forward_two_step_m' if forward else 'backward_two_step_m')
    if not _interval(distance) or distance[0] <= 0:
        return dict(result, reason='the required two-step Tag 2 displacement has not been verified')
    changes = [d if forward else -d for d in distance]
    projected = [projection * dx for dx in changes]
    landing = (position - uncertainty + min(projected), position + uncertainty + max(projected))
    gain, residual = _projected_effect(
        {'projection': projection, 'residual': (error - uncertainty, error + uncertainty)}, changes)
    if not _within(landing, window) or gain <= 0:
        return dict(result, reason='two steps would not reliably reduce the Tag 2 error within clearance')
    return dict(result, status=RoutePosePlanResult.ADJUSTED.value,
                operation='carry_forward2' if forward else 'cmd0_carry_backward2',
                reason='one separate two-step move reduces the measured Tag 2 longitudinal error',
                verified_landing_m=landing, worst_remaining_error_m=residual)


def _trial_position_rows(checkpoint, nav_cfg, tag_id):
    """Read distinct body-frame samples from the pinned chest source."""
    scan = checkpoint.get('scan') or {}
    contract = nav_cfg['pose_contract']
    if (contract.get('frame') != '/map' or contract.get('body_right') != '-y'
            or contract.get('body_forward') != '+x'
            or contract.get('capture_time_confirmed') is not True
            or _camera_fingerprint(nav_cfg, checkpoint.get('camera_info')) is None
            or scan.get('ids') != [tag_id] or scan.get('camera_healthy') is not True
            or scan.get('contract_match') is not True):
        return {}, 'fresh expected Tag and the confirmed chest camera are required'
    after, now = scan.get('after_ns'), scan.get('now_ns')
    if type(after) is not int or type(now) is not int or not 0 < after < now:
        return {}, 'checkpoint observation time is invalid'
    positions = {}
    for row in scan.get('observations', ()):
        stamp = row.get('stamp_ns')
        if (row.get('id') != tag_id or row.get('frame_id') != '/map' or row.get('headers_disagree')
                or row.get('source_publisher') != contract['publisher'] or row.get('source_topic') != nav_cfg['topic']
                or type(stamp) is not int or not after < stamp <= now
                or now - stamp > nav_cfg['max_age_s'] * 1e9):
            continue
        position = row.get('position_raw_m') or {}
        x, y = position.get('x'), position.get('y')
        if finite(x) and finite(y):
            positions[stamp] = row
    if len(positions) < nav_cfg['min_frames']:
        return {}, 'not enough distinct checkpoint position samples'
    return positions, None


def _trial_ground_depth(checkpoint, nav_cfg, tag_id):
    rows, problem = _trial_position_rows(checkpoint, nav_cfg, tag_id)
    if problem:
        return {}, problem
    try:
        poses = [ground_pose(row, nav_cfg['pose_contract']) for row in rows.values()]
    except (ValueError, KeyError, TypeError):
        return {}, f'Tag {tag_id} ground-plane estimate unavailable'
    distances = [-pose['forward_m'] for pose in poses]
    projection = [pose['body_forward_in_tag_forward'] for pose in poses]
    lateral = [pose['x_m'] for pose in poses]
    headings = [pose['heading_deg'] for pose in poses]
    heading = circular_median(headings)
    return {'distance_m': median(distances), 'distance_bounds_m': (min(distances), max(distances)),
            'heading_deg': heading, 'max_heading_deviation_deg': max(abs(wrap(yaw-heading)) for yaw in headings),
            'max_abs_yaw_deg': max(map(abs, headings)),
            'max_abs_lateral_m': max(map(abs, lateral)), 'lateral_bounds_m': (min(lateral), max(lateral)),
            'projection_bounds': (min(projection), max(projection)), 'latest_stamp_ns': max(rows)}, None


def RoutePose_PartialCheckpointGeometry(rows, nav_cfg, tag_id):
    """Describe strict same-stance samples supplied by the caller, for recovery only."""
    contract = nav_cfg.get('pose_contract', {})
    if (contract.get('capture_time_confirmed') is not True
            or contract.get('body_forward') != '+x' or contract.get('body_right') != '-y'):
        return None
    samples = {}
    for row in rows:
        stamp = row.get('stamp_ns')
        if (row.get('id') != tag_id or row.get('frame_id') != contract.get('frame') or row.get('headers_disagree')
                or not contract.get('publisher') or row.get('source_publisher') != contract['publisher']
                or row.get('source_topic') != nav_cfg.get('topic') or type(stamp) is not int or stamp <= 0):
            continue
        try:
            pose = ground_pose(row, contract)
        except (KeyError, TypeError, ValueError):
            # A malformed expected row cannot support directional evidence.
            return None
        samples[stamp] = (pose, row['position_raw_m']['x'])
    if not samples:
        return None
    poses, body_forward = zip(*samples.values())
    distance = [-pose['forward_m'] for pose in poses]
    lateral = [pose['x_m'] for pose in poses]
    headings = [pose['heading_deg'] for pose in poses]
    projection = [pose['body_forward_in_tag_forward'] for pose in poses]
    heading = circular_median(headings)
    return {'distance_bounds_m': (min(distance), max(distance)),
            'body_forward_bounds_m': (min(body_forward), max(body_forward)),
            'lateral_bounds_m': (min(lateral), max(lateral)),
            'heading_deg': heading, 'max_abs_heading_deg': max(map(abs, headings)),
            'max_heading_deviation_deg': max(abs(wrap(value-heading)) for value in headings),
            'projection_bounds': (min(projection), max(projection)),
            'latest_stamp_ns': max(samples), 'samples': len(samples)}


def RoutePose_TrialLateralHint(checkpoint, nav_cfg, tag_id):
    """Read the body lateral interval for a one-stride trial adjustment."""
    evidence = _trial_lateral_evidence(checkpoint, nav_cfg, tag_id, yaw_limit=12. if tag_id == 2 else 20.)
    if evidence is None:
        return None
    return {'tag_id': tag_id, **{key: evidence[key] for key in (
        'lateral_bounds_m', 'distance_m', 'heading_deg', 'latest_stamp_ns')}}


def RoutePose_Tag3HeadingRecovery(checkpoint, nav_cfg):
    """Keep a wider, coherent Tag 3 pose only for one clearance recovery."""
    return _trial_lateral_evidence(checkpoint, nav_cfg, 3, yaw_limit=30.)


def _trial_lateral_evidence(checkpoint, nav_cfg, tag_id, *, yaw_limit):
    evidence, problem = _trial_ground_depth(checkpoint, nav_cfg, tag_id)
    if problem:
        return None
    left, right = evidence['lateral_bounds_m']
    near, far = evidence['distance_bounds_m']
    if (right-left>.025 or far-near>.025 or not _within((near,far),(.08,.40))
            or evidence['max_abs_yaw_deg']>yaw_limit or evidence['max_heading_deviation_deg']>3.
            or evidence['projection_bounds'][0]<=0):
        return None
    return evidence


def RoutePose_PlanTag1ForwardTrial(checkpoint, nav_cfg, nominal, candidate_counts, two_step_distance_m):
    """Fold a repeatable Tag 1 depth error into the length of the existing straight."""
    result = {'status': RoutePosePlanResult.NOMINAL.value, 'tag_id': 1, 'count': nominal,
              'nominal_count': nominal, 'source': 'tag1_forward_trial'}
    if (nominal != COMMAND0_ENTRY_FORWARD_STEPS or not _interval(two_step_distance_m)
            or two_step_distance_m[0] <= 0):
        return dict(result, reason='Tag 1 trial requires the ten-step baseline and a trial stride range')
    evidence, problem = _trial_ground_depth(checkpoint, nav_cfg, 1)
    if problem:
        return dict(result, reason=problem)
    result.update(evidence)
    nearest, farthest = evidence['distance_bounds_m']
    projections = evidence['projection_bounds']
    if (evidence['max_abs_yaw_deg'] > 12.0 or farthest - nearest > .025 or projections[0] <= 0
            or not _within((nearest, farthest), (.08, _TAG1_FAR_OBSERVATION_M))):
        return dict(result, reason='Tag 1 heading, distance or measurement spread outside trial window')
    if farthest < _TAG1_NOMINAL_START_M[0]:
        direction = -1
    elif nearest > _TAG1_NOMINAL_START_M[1]:
        direction = 1
    else:
        return dict(result, reason='Tag 1 within the usual start-distance band')
    # For stable far readings, use the longest improving supported group even
    # if it cannot yet recover all the depth. Keep the same near-clearance bound.
    landing_window = (_TAG1_FAR_EFFECTIVE_START_M if nearest > .38 and evidence['max_abs_lateral_m'] <= .10
                      else _TAG1_TRIAL_EFFECTIVE_START_M)
    residual = (nearest - _TAG1_TRIAL_TARGET_M, farthest - _TAG1_TRIAL_TARGET_M)
    candidates = []
    for count in candidate_counts:
        difference = count - nominal
        if difference * direction <= 0 or abs(difference) > 5:
            continue
        shifts = [-difference * distance * projection / 2
                  for distance in two_step_distance_m for projection in projections]
        landing = (nearest + min(shifts), farthest + max(shifts))
        gain, _ = _projected_effect({'projection': 1.0, 'residual': residual}, shifts)
        if gain >= .01 and _within(landing, landing_window):
            candidates.append((gain, -abs(difference), count, landing))
    if not candidates:
        return dict(result, reason='no supported Tag 1 forward count reliably improves trial geometry')
    gain, _, count, landing = max(candidates)
    return dict(result, status=RoutePosePlanResult.ADJUSTED.value, count=count,
                reason='Tag 1 depth error absorbed into the Tag 2 forward group',
                conservative_improvement_m=gain, equivalent_nominal_start_bounds_m=landing)


def RoutePose_PlanTag2ForwardTrial(checkpoint, nav_cfg, settings, *, candidate_counts=TAG2_APPROACH_COUNTS):
    """Estimate the full forward approach from one stationary Tag 2 pose."""
    result = {'status': RoutePosePlanResult.NOMINAL.value, 'tag_id': 2, 'planned_steps': 0,
              'source': 'tag2_forward_trial'}
    evidence, problem = _trial_ground_depth(checkpoint, nav_cfg, 2)
    if problem:
        return dict(result, reason=problem)
    result.update(evidence)
    nearest, farthest = evidence['distance_bounds_m']
    projection = evidence['projection_bounds']
    near, far = settings['distance_window_m']
    target, deadband = settings['target_distance_m'], settings['deadband_m']
    stable = farthest - nearest <= .025
    reverse_bound = settings.get('approach_two_step_distance_m', settings['two_step_distance_m'])[1]
    close = stable and .02 <= nearest <= farthest < max(near, target-deadband)
    result.update(confirmed_far=stable and nearest > far,
                  confirmed_near=close
                                 and evidence['max_abs_lateral_m'] <= .10
                                 and evidence['max_abs_yaw_deg'] <= 12. and projection[0] > 0.,
                  in_departure_window=stable and _within((nearest, farthest), (near, far)))
    if result['confirmed_near']:
        return dict(result, reason='Tag 2 is inside the near side of its target band',
                    reverse_clearance_available=farthest+reverse_bound <= far)
    approach_range = (near, max(far, _TAG2_FAR_APPROACH_LIMIT_M))
    if not stable or projection[0] <= 0 or not _within((nearest, farthest), approach_range):
        return dict(result, reason='Tag 2 heading, distance or measurement spread outside trial window')
    if evidence['max_abs_yaw_deg'] > 12.0:
        if evidence['max_heading_deviation_deg'] > 3.:
            return dict(result, heading_reobserve_required=True, reason='Tag 2 frames disagree about approach heading')
        heading = evidence['heading_deg']
        if abs(heading) > 12.:
            return dict(result, heading_to_correct_deg=heading,
                        reason='Tag 2 requires heading correction before longitudinal approach')
        # A consistent window can straddle the heading threshold. Use the
        # same median for selecting a turn and admitting the short approach.
    if nearest - target <= deadband:
        return dict(result, reason='Tag 2 already within forward trial tolerance or closer')
    short, long = settings.get('approach_two_step_distance_m', settings['two_step_distance_m'])
    offset = {'projection': 1.0, 'residual': (nearest-target, farthest-target)}
    choices = []
    for steps in candidate_counts:
        movement = (steps/2 * short * projection[0], steps/2 * long * projection[1])
        landing = (nearest - movement[1], farthest - movement[0])
        gain, residual = _projected_effect(offset, [-distance for distance in movement])
        if _within(landing, (near, far)) and gain > 0:
            choices.append((residual, steps, landing))
    if not choices:
        return dict(result, reason='no Tag 2 forward count predicts a landing within the strafe window')
    _, steps, landing = min(choices)
    return dict(result, status=RoutePosePlanResult.ADJUSTED.value, planned_steps=steps,
                estimated_landing_m=landing, reason='Tag 2 forward count fitted to the estimated remaining distance')


def RoutePose_PlanTag4Longitudinal(checkpoint, nav_cfg, settings, *, approach_steps=None):
    """Use fresh Tag 4 depth to shorten an approach or trim before the left traverse."""
    plan = {'steps': approach_steps if approach_steps is not None else 0, 'distance_verified': False}
    evidence, problem = _trial_ground_depth(checkpoint, nav_cfg, 4)
    if problem:
        return dict(plan, reason=problem)
    plan.update(evidence)
    near, far = evidence['distance_bounds_m']
    projection = evidence['projection_bounds']
    window = settings['distance_window_m']
    if far - near > .025 or projection[0] <= 0:
        return dict(plan, reason='Tag 4 depth outside trial observation range')
    very_near = 0 < near <= far < window[0]
    if approach_steps is not None and very_near:
        return dict(plan, distance_verified=True, steps=0, reason='Tag 4 too near for a forward group')
    target, band = settings['target_distance_m'], settings['deadband_m']
    if approach_steps is None:
        plan['confirmed_far'] = near > target+band
    if not very_near and not _within((near, far), window):
        return dict(plan, reason='Tag 4 depth outside trial observation range')
    if approach_steps is None:
        plan['depth_confirmed'] = True
    skewed = evidence['max_abs_yaw_deg'] > 15.
    if skewed:
        left, right = evidence['lateral_bounds_m']
        if (evidence['max_heading_deviation_deg'] <= 3. and right-left <= .025
                and abs(evidence['heading_deg']) > 15.):
            plan['heading_to_correct_deg'] = evidence['heading_deg']
        if near <= target+band or evidence['max_abs_yaw_deg'] > 30. or 'heading_to_correct_deg' not in plan:
            return dict(plan, reason='Tag 4 heading needs correction before translating')
    check_after_short_move = skewed and approach_steps is not None
    if not check_after_short_move:
        plan.update(distance_verified=True, steps=0)
    if approach_steps is not None and near <= target + band:
        return dict(plan, reason='Tag 4 already near, omit further forward steps')
    if evidence['max_abs_lateral_m'] > .16:
        return dict(plan, distance_verified=False, steps=approach_steps or 0,
                    reason='Tag 4 lateral offset too large for longitudinal trial')
    if near > target+band:
        supported = (8, 4, 2) if approach_steps == 8 else (4, 2) if approach_steps == 4 else (2,)
        candidates = (2,) if check_after_short_move else supported
    elif approach_steps is None and far < target-band:
        candidates = (-2,)
    else:
        return dict(plan, reason='Tag 4 longitudinal position within trial tolerance')
    for stride in candidates:
        travel_bounds = settings['two_step_distance_m']
        if (approach_steps is None or check_after_short_move or stride == 8) and stride > 0:
            # A field run traveled 19.5 cm in four steps; reserve for ~10 cm in two.
            travel_bounds = (travel_bounds[0], max(travel_bounds[1], .10))
        if (skewed or stride == 8) and stride > 0:
            heading = evidence['heading_deg']
            spread = evidence['max_heading_deviation_deg']
            lateral_shifts = [distance * sin(radians(yaw)) for distance in (0.,stride/2*travel_bounds[1])
                              for yaw in (heading-spread, heading+spread)]
            left, right = evidence['lateral_bounds_m']
            predicted = left+min(lateral_shifts), right+max(lateral_shifts)
            plan['projected_lateral_bounds_m'] = predicted
            if not _within(predicted, (-.16,.16)):
                continue
        shifts = [-stride * travel * factor / 2
                  for travel in travel_bounds for factor in projection]
        landing = (near + min(shifts), far + max(shifts))
        gain, _ = _projected_effect(
            {'projection': 1.0, 'residual': (near-target, far-target)}, shifts)
        if (gain > 0.005 and _within(landing, window)
                and (stride < 0 or landing[0] >= target-band)):
            if check_after_short_move:
                return dict(plan, guarded_forward_steps=2, estimated_landing_m=landing,
                            reason='Tag 4 heading needs correction; only two steps fit the projected clearance')
            return dict(plan, steps=stride, estimated_landing_m=landing,
                        reason='Tag 4 depth supports a bounded longitudinal correction')
    reason = ('no two-step correction reliably improves Tag 4 departure depth' if approach_steps is None
              else 'no supported forward group reliably improves Tag 4 depth')
    return dict(plan, reason=reason)


def RoutePose_PlanTag5EntryLateral(checkpoint, nav_cfg, settings, *, small_ready):
    """Try to center the entry trajectory on Tag 5 using the chest-camera body frame."""
    result = {'status': RoutePosePlanResult.NOMINAL.value, 'operation': None}
    goal = next((goal for goal in nav_cfg['legs']['outbound'] if goal['id'] == 5), {})
    if goal.get('allow_strafe') is not True:
        return dict(result, reason='Tag 5 lateral travel unavailable')
    positions, problem = _trial_position_rows(checkpoint, nav_cfg, 5)
    if problem:
        return dict(result, reason=problem)
    forward, lateral = zip(*((row['position_raw_m']['x'], row['position_raw_m']['y'])
                             for row in positions.values()))
    near, far = settings['forward_window_m']
    spread = max(lateral) - min(lateral)
    error = median(lateral) - settings['target_map_y_m']
    result.update(map_y_error_m=error, map_y_spread_m=spread, map_x_m=median(forward),
                  samples=len(positions), latest_stamp_ns=max(positions))
    if min(forward) < near or max(forward) > far or spread > 0.025:
        return dict(result, reason='Tag 5 outside the trial observation window or lateral data unstable')
    distance = abs(error) - spread / 2
    if distance <= settings['deadband_m']:
        return dict(result, reason='Tag 5 lateral residual within trial tolerance')
    side = 'strafe_left_once' if error > 0 else 'strafe_right_once'
    operation = side
    if distance < settings['large_move_error_m']:
        if not small_ready:
            return dict(result, reason='small strafe unavailable for the measured Tag 5 offset')
        operation = SMALL_SIDE_ACTIONS[side]
    return dict(result, status=RoutePosePlanResult.ADJUSTED.value, operation=operation,
                reason='move toward the center of Tag 5 and verify the new offset')


def RoutePose_PlanTag5Passage(checkpoint, nav_cfg, settings, *, small_ready, entering=False,
                             current_group_distance_m=0.15):
    """Use fresh Tag 5 ground poses to check the next straight and plan a side trim."""
    result = {'geometry_valid': False, 'ready_for_forward': False, 'operation': None,
              'distance_m': None, 'lateral_error_m': None, 'heading_deg': None,
              'latest_stamp_ns': None}
    positions, problem = _trial_position_rows(checkpoint, nav_cfg, 5)
    if problem or len(positions) < 3:
        return dict(result, reason=problem or 'Tag 5 requires at least three distinct ground poses')
    try:
        poses = [ground_pose(row, nav_cfg['pose_contract']) for row in positions.values()]
    except (ValueError, KeyError, TypeError):
        return dict(result, reason='Tag 5 ground-plane pose unavailable')

    depth_window = (settings.get('entering_depth_window_m', (-.24, .45)) if entering
                    else settings.get('passage_depth_window_m', (.075, .45)))
    limit = settings.get('lateral_clearance_m', .075)
    max_heading = settings.get('max_heading_deg', 12.)
    target = settings.get('target_lateral_m', 0.)
    deadband = settings.get('deadband_m', .05)
    large_error = settings.get('large_move_error_m', .10)
    if (not _interval(depth_window)
            or not all(finite(value) for value in
                       (limit, max_heading, target, deadband, large_error, current_group_distance_m))
            or not (0 < deadband <= limit and deadband <= large_error and 0 < max_heading < 90)
            or current_group_distance_m < 0):
        return dict(result, reason='invalid Tag 5 passage settings')

    depths = [-pose['forward_m'] for pose in poses]
    errors = [pose['x_m'] - target for pose in poses]
    headings = [pose['heading_deg'] for pose in poses]
    heading = circular_median(headings)
    error = median(errors)
    depth_bounds = (min(depths), max(depths))
    lateral_bounds = (min(errors), max(errors))
    max_abs_heading = max(abs(value) for value in headings)
    result.update(distance_m=median(depths), lateral_error_m=error, heading_deg=heading,
                  latest_stamp_ns=max(positions), samples=len(positions), depth_bounds_m=depth_bounds,
                  lateral_bounds_m=lateral_bounds, heading_bounds_deg=(min(headings), max(headings)),
                  max_abs_heading_deg=max_abs_heading, entering=entering)
    if (depth_bounds[1]-depth_bounds[0] > .035 or lateral_bounds[1]-lateral_bounds[0] > .025
            or any(abs(wrap(yaw-heading)) > 5 for yaw in headings)):
        return dict(result, reason='Tag 5 ground poses disagree across the fresh frames')
    if not _within(depth_bounds, depth_window):
        return dict(result, reason='Tag 5 outside the passage depth window')

    result['geometry_valid'] = True
    if max_abs_heading <= max_heading and all(pose['body_forward_in_tag_forward'] > 0 for pose in poses):
        projected = list(errors)
        for lateral, depth, yaw in zip(errors, depths, headings):
            angle = radians(yaw)
            projected.append(lateral + sin(angle)*current_group_distance_m)
            projected.append(lateral + tan(angle)*max(0.,depth))
        bounds = min(projected), max(projected)
        result['projected_lateral_bounds_m'] = bounds
        if _within(bounds, (-limit,limit)):
            return dict(result, ready_for_forward=True, reason='Tag 5 next straight fits the trial passage')
        reason = 'Tag 5 projected path exceeds the lateral passage tolerance'
    else:
        reason = 'Tag 5 heading outside the passage tolerance'

    # Lateral correction uses the Tag frame even when the body yaw prevents
    # forward motion. Avoid translating when the side axes point apart.
    goal = next((goal for goal in nav_cfg['legs']['outbound'] if goal['id'] == 5), {})
    result['lateral_recovery_allowed'] = (goal.get('allow_strafe') is True
                                         and min(pose['body_right_in_tag_right'] for pose in poses) > .5)
    if result['lateral_recovery_allowed']:
        projected_bounds = result.get('projected_lateral_bounds_m')
        if projected_bounds is not None and projected_bounds[1]-projected_bounds[0] <= 2*limit:
            error = sum(projected_bounds)/2
            residual = abs(error)
            small_cutoff = min(deadband,.02)
        else:
            residual = min(errors) if error > 0 else -max(errors)
            small_cutoff = deadband
        result['alignment_error_m'] = error
        side = 'strafe_left_once' if error > 0 else 'strafe_right_once'
        if residual >= large_error:
            result['operation'] = side
        elif residual > small_cutoff and small_ready:
            result['operation'] = SMALL_SIDE_ACTIONS[side]
    return dict(result, reason=reason)


def RoutePose_PlanForwardCount(pose, profile, nominal):
    """Fold a measured Tag 1 fore/aft error into one supported two-step change."""
    result = {'status': RoutePosePlanResult.NOMINAL.value, 'tag_id': 1,
              'nominal_count': nominal, 'count': nominal}
    tag = _tag_profile(profile, 1)
    forward = tag.get('forward_adjustment')
    if (tag.get('departure_axis') != 'forward' or not isinstance(forward, dict)
            or forward.get('verified') is not True or forward.get('nominal_count') != nominal):
        return dict(result, reason='same-load Tag 1 stride and next-checkpoint clearance unverified')
    low, high = forward.get('min_count'), forward.get('max_count')
    target = forward.get('target_forward_m')
    step = forward.get('body_forward_step_m')
    gain_limit = forward.get('minimum_improvement_m')
    landing_limit = forward.get('max_landing_forward_error_m')
    uncertainty_floor = forward.get('forward_uncertainty_m', 0)
    window = tag.get('forward_window_m')
    if (any(type(value) is not int for value in (low, nominal, high)) or not 2 <= low <= nominal <= high <= 80
            or not _interval(step) or step[0] <= 0 or not _interval(window)
            or not all(finite(value) for value in (target, gain_limit, landing_limit, uncertainty_floor))
            or not window[0] <= target <= window[1]
            or min(gain_limit, landing_limit) <= 0 or uncertainty_floor < 0):
        return dict(result, reason='invalid Tag 1 stride, reference, or clearance geometry')
    if (not isinstance(pose, dict) or pose.get('status') != RoutePoseMeasureResult.MEASURED.value
            or pose.get('tag_id') != 1 or pose.get('direction', 'outbound') != 'outbound'
            or pose.get('position_usable') is not True or pose.get('heading_within_window') is not True):
        return dict(result, reason='current Tag 1 lacks a trusted forward pose within the heading window')
    advice = RoutePose_AdviseDeparture(pose, profile, 1)
    if advice['status'] not in (RoutePoseDepartureResult.UNVERIFIED.value, RoutePoseDepartureResult.FUSE.value):
        return dict(result, reason=advice['reason'], departure_status=advice['status'])
    position = pose.get('forward_m')
    mad = pose.get('forward_mad_m')
    projection = pose.get('body_forward_in_tag_forward')
    if (not all(finite(value) for value in (position, mad, projection))
            or mad < 0 or not 0 < abs(projection) <= 1 + 1e-8):
        return dict(result, reason='invalid forward estimate or body-axis projection')
    uncertainty = max(mad, uncertainty_floor)
    error = position - target
    result.update(forward_error_m=error, forward_uncertainty_m=uncertainty)
    if not _within((position-uncertainty, position+uncertainty), window) or abs(error) <= uncertainty:
        return dict(result, reason='forward error has no clear sign or is outside the measured departure window')
    context = {'projection': projection, 'residual': (error - uncertainty, error + uncertainty)}
    candidates = []
    for change in (-2, 2):
        count = nominal + change
        if count % 2 or not low <= count <= high:
            continue
        gain, landing = _projected_effect(context, [change * displacement for displacement in step])
        if gain >= gain_limit and landing <= landing_limit:
            candidates.append((gain, count, landing))
    if not candidates:
        return dict(result, reason='no two-step change reliably improves forward error within clearance')
    gain, count, landing = max(candidates)
    return dict(result, status=RoutePosePlanResult.ADJUSTED.value, count=count,
                reason='two-step change absorbs measured Tag 1 forward error',
                conservative_improvement_m=gain, worst_landing_forward_error_m=landing)


def RoutePose_PlanSideCount(pose, profile, tag_id, op, nominal):
    """Absorb lateral error in at most one cycle of the upcoming side segment."""
    result, context = _side_plan_context(pose, profile, tag_id, op, nominal)
    if context is None:
        return result
    side = context['side']
    displacement = side.get('body_right_displacement_m')
    if not _signed_side_interval(displacement, op):
        return dict(result, reason='invalid signed full-cycle displacement')
    candidates = []
    for change in (-1, 1):
        count = nominal + change
        if not side['min_count'] <= count <= side['max_count']:
            continue
        gain, landing = _projected_effect(context, [change * step for step in displacement])
        if gain >= side['minimum_improvement_m'] and landing <= side['max_landing_x_error_m']:
            candidates.append((gain, count, landing))
    if not candidates:
        return dict(result, reason='no one-cycle change robustly improves lateral residual within clearance')
    gain, count, landing = max(candidates)
    return dict(result, status=RoutePosePlanResult.ADJUSTED.value, count=count,
                reason='same-direction cycle count absorbs measured lateral residual',
                body_right_effect_m=displacement, x_error_m=context['error'],
                conservative_improvement_m=gain, worst_landing_x_error_m=landing)


def RoutePose_PlanSmallSide(pose, profile, tag_id, op, nominal):
    """Append or substitute one measured same-direction small side step."""
    result, context = _side_plan_context(pose, profile, tag_id, op, nominal)
    result['small_operation'] = None
    if context is None:
        return result
    side = context['side']
    small = side.get('small_step')
    if (not isinstance(small, dict) or small.get('verified') is not True
            or small.get('operation') != SMALL_SIDE_ACTIONS[op]):
        return dict(result, reason='same-load small-step displacement and clearance unverified')
    distance = small.get('body_right_displacement_m')
    if not _signed_side_interval(distance, op):
        return dict(result, reason='invalid signed small-step displacement')

    candidates = []
    if small.get('allow_append') is True:
        gain, landing = _projected_effect(context, distance)
        candidates.append((gain, nominal, 'append', landing))
    full = side.get('body_right_displacement_m')
    if (small.get('allow_replace') is True and nominal - 1 >= side['min_count']
            and _signed_side_interval(full, op)
            and max(map(abs, distance)) < min(map(abs, full))):
        differences = [short - long for short in distance for long in full]
        gain, landing = _projected_effect(context, differences)
        candidates.append((gain, nominal - 1, 'replace_last', landing))
    if not candidates:
        return dict(result, reason='no calibrated small-step layout permitted in this segment')
    allowed = [candidate for candidate in candidates if candidate[0] >= side['minimum_improvement_m']
               and candidate[3] <= side['max_landing_x_error_m']]
    if not allowed:
        return dict(result, reason='one small step cannot reliably improve the lateral residual inside the landing window')
    gain, count, adjustment, landing = max(allowed)
    return dict(result, status=RoutePosePlanResult.ADJUSTED.value, count=count,
                reason='one same-direction small step absorbs measured lateral residual',
                small_operation=small['operation'], adjustment=adjustment,
                small_body_right_effect_m=distance, x_error_m=context['error'],
                conservative_improvement_m=gain, worst_landing_x_error_m=landing)
