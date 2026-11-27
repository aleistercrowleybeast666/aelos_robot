"""Optional in-base placement yaw. Positive turns clockwise in the route frame."""

from math import floor, isfinite

from nav_observation import wrap


def BasePlacement_ValidateYaw(mode, value):
    if mode not in ('0', '1', '2') or type(value) is not int or not -30 <= value <= 30:
        raise ValueError('放物转角仅适用于命令 0、1、2，须为 −30 至 +30 之间的整数度数，负左正右。')
    return value


def _measured_delta(robot, nav, before, after):
    if not before or not after:
        return None
    for key in ('load', 'capture_session', 'owner_observation_scope', 'calibration_id', 'load_epoch', 'norm_basis'):
        if before.get(key) is None or before[key] != after.get(key):
            return None
    if not before['capture_session'] or not before['calibration_id'] or before['load'] != robot.load_kind:
        return None
    completed = robot.last_action_completed_monotonic
    if (not before.get('capture_verified') or not after.get('capture_verified')
            or after.get('action_completed_monotonic') != completed
            or not before.get('decision_capture_times') or not after.get('decision_capture_times')
            or not max(before['decision_capture_times']) <= completed < min(after['decision_capture_times'])):
        return None
    magnetic = nav.cfg['magnetic']
    if before['norm'] <= 0 or abs(after['norm'] - before['norm']) > before['norm'] * magnetic['max_norm_fraction_change']:
        return None
    delta = wrap(magnetic['heading_sign'] * (after['raw_heading_deg'] - before['raw_heading_deg']))
    # A single placement micro-turn cannot establish a half-turn from one magnetic jump.
    return delta if isfinite(delta) and abs(delta) <= 45. else None


def BasePlacement_TurnBeforeRelease(robot, nav, requested):
    from mission_checkpoints_v5 import Checkpoint_ReadHeading
    from turn_estimator import MICRO_INITIAL_DEG

    report = {'requested_deg': requested, 'applied_deg': 0., 'basis': 'no_turn', 'actions': []}
    direction = 'right' if requested > 0 else 'left'
    sign = 1 if requested > 0 else -1
    action = 'single_micro_' + direction
    simulated = getattr(robot.backend, 'is_simulation', False)
    if not simulated and not robot.backend.loaded_micro_ready():
        report['reason'] = 'micro_capability_unavailable'
        return report
    def read():
        return None if simulated else Checkpoint_ReadHeading(robot, nav, robot.check)

    sample = read()
    nominal = robot.cfg.get('route_steps', {}).get('micro_turn_filter', {}).get('initial_deg', MICRO_INITIAL_DEG)
    estimator = getattr(getattr(robot, 'heading_guard', None), 'turn_estimator', None)
    if estimator and sample:
        gain = estimator.control_estimate(direction, 'micro', robot.load_kind, robot.backend.session,
                                         robot.backend.epoch, sample['calibration_id'], now_s=robot.clock())
        if gain and isfinite(gain['degrees']) and abs(gain['degrees']) >= 1:
            nominal = abs(gain['degrees'])
    # Reserve the native release. A small placement request must not consume
    # the journey by alternating fine turns on either side of the target.
    for _ in range(4):
        robot.check()
        if (requested - report['applied_deg']) * sign <= nominal / 2:
            report['reason'] = 'within_one_half_step_or_target_crossed'
            break
        timeout = robot.cfg['request_timeout_s']
        if robot.count + 2 > robot.cfg['max_body_macros_per_journey'] or robot.deadline - robot.clock() <= 2 * timeout:
            report['reason'] = 'reserve_release'
            break
        robot._complete_body_action(action, timeout)
        after = read()
        actual = _measured_delta(robot, nav, sample, after)
        report['actions'].append({'action': action, 'nominal_deg': sign * nominal, 'observed_deg': actual})
        report['applied_deg'] += sign * nominal if actual is None else actual
        report['basis'] = ('measured' if all(row['observed_deg'] is not None for row in report['actions'])
                           else 'estimated_from_completed_actions')
        if sample and actual is None:
            report['reason'] = 'feedback_lost_after_completed_turn'
            break
        if actual is not None and actual * sign < 1:
            report['reason'] = 'unproductive_response'
            break
        sample = after
    else:
        report['reason'] = 'completed_bounded_adjustment'
    return report


def BasePlacement_ReturnTurn(step, applied):
    """Face the original return lane from the load's actual or estimated final bearing."""
    original = step['target_delta_deg']
    remaining = wrap(original - applied)
    if remaining == -180 and original > 0:
        remaining = 180.
    count = min(20, max(1, floor(step['repeat'] * abs(remaining / original) + .5))) if remaining else 0
    return dict(step, op='empty_right_turn' if remaining >= 0 else 'empty_left_turn',
                repeat=count, target_delta_deg=remaining)
