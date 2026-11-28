"""Optional in-base placement yaw. Positive turns clockwise in the route frame."""

from math import floor, isfinite

from heading_guard import HeadingGuard, MicroTrialStopped
from micro_trial import MEDIUM_NOMINAL_DEG
from nav_observation import MagneticReference, wrap


_COARSE_TURN_DEG = 30
_APPROXIMATION_DEG = 15
_LIMIT_DEG = 90
_EDGE_MARGIN_DEG = 3
# A nominal 30-degree loaded turn can travel much further on the floor.
# These are dispatch margins for indivisible gaits, not mechanical stops.
_COARSE_ALLOWANCE_DEG = 60
_MEDIUM_ALLOWANCE_DEG = 35


def BasePlacement_ValidateYaw(mode, value):
    if mode not in ('0', '1', '2') or type(value) is not int or not -90 <= value <= 90:
        raise ValueError('放物转角仅适用于命令 0、1、2，须为 -90 至 +90 之间的整数度数，负左正右。')
    return value


def _placement_guard(robot):
    nav = robot.ensure_navigation_io()
    magnetic = nav.cfg.get('magnetic', {}) if nav else {}
    sign = magnetic.get('heading_sign')
    if (not magnetic.get('enabled') or type(sign) is not int or sign not in (-1, 1)
            or not getattr(nav.io, 'motion_capture_verified', False)):
        return None
    guard = HeadingGuard(nav.cfg, nav.io, MagneticReference(magnetic), robot.check, robot.clock)
    guard.allow_missing_magnetic = True
    guard.observation_scope = (robot.backend.session, robot.backend.epoch)
    return guard


def BasePlacement_TurnBeforeRelease(robot, requested):
    """Spread drops using measured yaw from the actual loaded stance inside the base."""
    report = {'requested_deg': requested, 'applied_deg': 0, 'basis': 'no_turn', 'actions': []}
    if abs(requested) <= _APPROXIMATION_DEG:
        report['reason'] = 'nearest_coarse_angle_is_zero'
        return report
    robot.check()
    guard = _placement_guard(robot)
    if guard is None:
        report.update(basis='magnetic_unconfirmed', reason='magnetic_reference_unavailable')
        return report
    magnetic = guard.cfg['magnetic']
    timeout = robot.cfg['request_timeout_s']
    # A post-COMPLETE window can retry once. Leave time for that window and
    # the release even if the preceding macro takes its full request timeout.
    read_budget = guard.c['settle_s'] + 2 * magnetic['collect_s']
    if robot.count + 2 > robot.cfg['max_body_macros_per_journey'] or robot.deadline - robot.clock() <= 2 * (timeout + read_budget):
        report['reason'] = 'reserve_release'
        return report
    load = robot.load_kind
    previous, issue = _placement_sample(robot, guard, load)
    if not previous:
        report.update(basis='magnetic_unconfirmed', reason=issue)
        return report
    baseline = previous
    direction = 1 if requested > 0 else -1
    side = 'right' if direction > 0 else 'left'
    report['start_raw_heading_deg'] = baseline['raw_heading_deg']
    report['basis'] = 'measured'
    coarse_upper = _COARSE_ALLOWANCE_DEG
    estimator = getattr(robot.backend, 'turn_estimator', None)
    if estimator is not None:
        forecast = estimator.control_estimate(side, 'macro', load, *guard.observation_scope,
                                              baseline['calibration_id'], now_s=robot.clock())
        if forecast:
            coarse_upper = max(coarse_upper, forecast['upper_abs_deg'])
    for _ in range(3):
        robot.check()
        remaining = direction * (requested - report['applied_deg'])
        available = _LIMIT_DEG - _EDGE_MARGIN_DEG - abs(report['applied_deg'])
        if remaining <= _APPROXIMATION_DEG:
            report['reason'] = 'within_approximate_target' if abs(remaining) <= _APPROXIMATION_DEG else 'target_crossed'
            break
        # Never replay the full-size turn, even if its first response was small.
        # A tight requested angle can be closer to the current stance than a coarse turn.
        if not report['actions'] and coarse_upper <= available and coarse_upper / 2 <= remaining:
            action, nominal, allowance = side+'_turn', _COARSE_TURN_DEG, coarse_upper
        elif (_MEDIUM_ALLOWANCE_DEG <= available
              and getattr(robot.backend, 'medium_turn_ready', lambda: False)()
              and (load == 'single' or 'double_single_medium_'+side in robot.backend.mission_pair_ops())):
            action, nominal, allowance = 'single_medium_'+side, MEDIUM_NOMINAL_DEG, _MEDIUM_ALLOWANCE_DEG
        else:
            report['reason'] = 'no_turn_within_margin'
            break
        if robot.count + 2 > robot.cfg['max_body_macros_per_journey'] or robot.deadline - robot.clock() <= 2 * timeout + read_budget:
            report['reason'] = 'reserve_release'
            break
        if (not guard.io.motion_capture_verified
                or not 0 <= robot.clock() - min(previous['decision_capture_times']) <= magnetic['max_age_s']
                or (robot.backend.session, robot.backend.epoch) != guard.observation_scope):
            report['reason'] = 'pre_turn_reference_expired'
            break
        robot._complete_body_action(action, timeout)
        step = {'action': action, 'nominal_deg': direction * nominal, 'allowance_deg': allowance}
        report['actions'].append(step)
        sample, issue = _placement_sample(robot, guard, load, baseline=baseline, previous=previous)
        if not sample:
            report.update(basis='magnetic_unconfirmed', reason=issue,
                          return_estimated_deg=report['applied_deg']+direction*nominal)
            break
        delta = magnetic['heading_sign'] * wrap(sample['raw_heading_deg'] - previous['raw_heading_deg'])
        report['applied_deg'] += delta
        step.update(observed_delta_deg=delta, accumulated_deg=report['applied_deg'])
        previous = sample
        if abs(report['applied_deg']) >= _LIMIT_DEG - _EDGE_MARGIN_DEG or not 2 < direction * delta <= allowance:
            report['reason'] = 'limit_margin_reached' if abs(report['applied_deg']) >= _LIMIT_DEG - _EDGE_MARGIN_DEG else 'unexpected_turn_response'
            break
    else:
        report['reason'] = 'bounded_turns_complete'
    return report


def _placement_sample(robot, guard, load, *, baseline=None, previous=None):
    if robot.load_kind != load or (robot.backend.session, robot.backend.epoch) != guard.observation_scope:
        return None, 'magnetic_scope_changed'
    boundary = robot.last_action_completed_monotonic
    if type(boundary) not in (int, float) or not isfinite(boundary):
        return None, 'missing_action_boundary'
    try:
        _, sample = guard.sample(load, boundary, stationary_retry=True)
    except MicroTrialStopped as exc:
        robot.check()
        return None, getattr(exc, 'guard_reason', None) or str(exc)
    if (not sample or not sample['capture_verified'] or not sample['usable_for_heading']
            or not sample.get('capture_session') or not sample.get('calibration_id')
            or type(sample.get('load_epoch')) is not int
            or sample.get('stable_samples', 0) < guard.cfg['magnetic']['min_samples'] or robot.load_kind != load
            or (robot.backend.session, robot.backend.epoch) != guard.observation_scope):
        return None, 'unverified_magnetic_window'
    if baseline:
        if any(sample.get(key) != baseline.get(key) for key in
               ('capture_session','calibration_id','load_epoch','norm_basis','owner_observation_scope')):
            return None, 'magnetic_scope_changed'
        if any(abs(sample['norm']/old['norm']-1) > guard.cfg['magnetic']['max_norm_fraction_change']
               for old in (baseline, previous)):
            return None, 'magnetic_field_changed'
    return sample, None


def BasePlacement_ReturnTurn(step, applied):
    """Face the original return lane from the load's actual or estimated final bearing."""
    original = step['target_delta_deg']
    remaining = wrap(original - applied)
    if remaining == -180 and original > 0:
        remaining = 180.
    count = min(20, max(1, floor(step['repeat'] * abs(remaining / original) + .5))) if remaining else 0
    return dict(step, op='empty_right_turn' if remaining >= 0 else 'empty_left_turn',
                repeat=count, target_delta_deg=remaining)
