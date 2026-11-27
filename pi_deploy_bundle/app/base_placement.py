"""Optional in-base placement yaw. Positive turns clockwise in the route frame."""

from math import floor

from nav_observation import wrap


_COARSE_TURN_DEG = 30


def BasePlacement_ValidateYaw(mode, value):
    if mode not in ('0', '1', '2') or type(value) is not int or not -90 <= value <= 90:
        raise ValueError('放物转角仅适用于命令 0、1、2，须为 -90 至 +90 之间的整数度数，负左正右。')
    return value


def BasePlacement_TurnBeforeRelease(robot, requested):
    """Use completed full-size turns only. The next return Tag corrects the empty robot."""
    count = (abs(requested) + _COARSE_TURN_DEG // 2) // _COARSE_TURN_DEG
    report = {'requested_deg': requested, 'applied_deg': 0, 'basis': 'no_turn', 'actions': []}
    if not count:
        report['reason'] = 'nearest_coarse_angle_is_zero'
        return report
    action = 'right_turn' if requested > 0 else 'left_turn'
    delta = _COARSE_TURN_DEG if requested > 0 else -_COARSE_TURN_DEG
    timeout = robot.cfg['request_timeout_s']
    for _ in range(count):
        robot.check()
        if robot.count + 2 > robot.cfg['max_body_macros_per_journey'] or robot.deadline - robot.clock() <= 2 * timeout:
            report['reason'] = 'reserve_release'
            break
        robot._complete_body_action(action, timeout)
        report['actions'].append({'action': action, 'nominal_deg': delta})
        report['applied_deg'] += delta
        report['basis'] = 'estimated_from_completed_actions'
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
