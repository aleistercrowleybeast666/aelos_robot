"""Metric arrival checks and optional same-load relative-heading alignment."""
import copy
from enum import Enum
import math
import re

from entry_camera import finite
from nav_observation import summarize, wrap
from relative_heading import reference_valid

_SWEEP_ORDER=(0,1,1,0)
_MAGNETIC_WINDOW_SCOPE=('capture_session','load_epoch','owner_observation_scope','calibration_id')
_SEARCH_ENVELOPE_DEG = 40.
_SEARCH_WAYPOINT_TOLERANCE_DEG = 3.
_SEARCH_MIN_TURN_DEG = 2.


class CheckpointArrivalResult(Enum):
    ARRIVED = 'arrived'
    POSE_UNAVAILABLE = 'pose_unavailable'
    POSITION_OUTSIDE = 'position_outside'
    HEADING_OUTSIDE = 'heading_outside'


def Checkpoint_ValidateArrivals(windows, nav, directions):
    """Validate the arrival windows and the camera that gives them metric meaning."""
    contract = nav['pose_contract']
    info = nav.get('visual_micro', {}).get('camera_info', {})
    if not all(contract[k] for k in ('axes_confirmed', 'scale_confirmed', 'capture_time_confirmed')):
        raise ValueError('mission checkpoint frame/scale/capture time must be field verified')
    if not (nav['camera_health']['topic_confirmed'] and nav['visual_micro']['enabled']
            and info.get('source_confirmed') and isinstance(info.get('publisher'), str)
            and info['publisher'].startswith('/') and info.get('frame_id')
            and re.fullmatch('[0-9a-f]{64}', info.get('expected_fingerprint') or '')):
        raise ValueError('mission checkpoints require verified chest image and CameraInfo')
    if min(nav['deadband_heading_deg'], nav['post_action_heading']['deadband_deg']) < 3.0:
        raise ValueError('mission heading tolerance is below the 3-degree sensor precision')
    if not isinstance(windows, dict):
        raise ValueError('mission checkpoint forward windows must be a map')
    for direction in directions:
        leg = windows.get(direction)
        if not isinstance(leg, dict):
            raise ValueError('missing mission checkpoint forward windows for '+direction)
        goals = nav['legs'][direction]
        if set(leg) != {str(goal['id']) for goal in goals}:
            raise ValueError('missing or unexpected mission checkpoints for '+direction)
        for tag, window in leg.items():
            if (not isinstance(window, list) or len(window) != 2
                    or any(type(v) not in (int, float) or not math.isfinite(v) for v in window)
                    or not -contract['max_range_m'] <= window[0] < window[1] <= contract['max_range_m']):
                raise ValueError('unmeasured mission forward window '+direction+':'+tag)


def Checkpoint_EvaluateArrival(scan, goal, forward_window, cfg):
    if scan.get('ids') != [goal['id']] or not (scan.get('pose_usable') and scan.get('position_usable')):
        return CheckpointArrivalResult.POSE_UNAVAILABLE
    if (not forward_window[0] <= scan['forward_m'] <= forward_window[1]
            or abs(scan['x_m'] - goal['lateral_m']) > cfg['deadband_x_m']):
        return CheckpointArrivalResult.POSITION_OUTSIDE
    yaw_tolerance = min(cfg['deadband_heading_deg'], cfg['post_action_heading']['deadband_deg'])
    if abs(wrap(scan['heading_deg'] - goal['tag_heading_deg'])) > yaw_tolerance:
        return CheckpointArrivalResult.HEADING_OUTSIDE
    return CheckpointArrivalResult.ARRIVED


def Checkpoint_Locate(window, cfg, windows, direction, expected, last_passed, after_ns):
    """Describe position relative to fresh tags and the ordered leg."""
    if cfg.get('relative_heading_reference', {}).get('enabled'):
        cfg = dict(cfg, relative_heading_reference={})
    goals = cfg['legs'][direction]
    order = [goal['id'] for goal in goals]
    expected_index = order.index(expected)
    scan = summarize(window, expected, cfg, after_ns)
    positions = {}
    for index, goal in enumerate(goals):
        tag = goal['id']
        observed = scan if tag == expected else summarize(window, tag, cfg, after_ns)
        if not observed.get('pose_usable'):
            continue
        arrival = Checkpoint_EvaluateArrival(observed, goal, windows[str(tag)], cfg)
        position = {
            'tag':tag, 'offset':index-expected_index, 'arrival':arrival.value,
            'heading_error_deg':wrap(observed['heading_deg']-goal['tag_heading_deg']),
        }
        if observed.get('position_usable'):
            position['forward_m'] = observed['forward_m']
            position['x_m'] = observed['x_m']
        positions[tag] = position
    known = [p for p in positions.values() if 'forward_m' in p]
    anchor = (min(known, key=lambda p: (p['arrival'] != 'arrived', math.hypot(p['forward_m'],p['x_m'])))
              if known else None)
    result=Checkpoint_EvaluateArrival(scan,goals[expected_index],windows[str(expected)],cfg)
    location={
        'direction':direction, 'expected_tag':expected, 'last_passed_tag':last_passed,
        'seen_tags':scan['observed_ids'], 'anchor':anchor,
        'opposite_leg_ids':[tag for tag in scan['observed_ids'] if tag not in order],
        'position': 'at_expected' if result is CheckpointArrivalResult.ARRIVED else 'relative_hint',
    }
    if anchor is None:
        location['position'] = 'unlocated'
    # Require a unique arrived position. A wide-angle sighting alone cannot skip a leg.
    arrived = [tag for tag,p in positions.items() if p['arrival']=='arrived']
    next_id = order[expected_index+1] if expected_index+1<len(order) else None
    if expected_index>0 and last_passed==order[expected_index-1] and arrived==[next_id]:
        location['adjacent_at_arrival'] = next_id
    return scan,result,location


def Checkpoint_ChooseNudge(scan, goal, forward_window, cfg, last_motion):
    """One short step on the taught approach axis, using a fresh metric target."""
    alignment = wrap(scan['heading_deg'] - goal['tag_heading_deg'])
    if abs(alignment) > min(cfg['deadband_heading_deg'], cfg['post_action_heading']['deadband_deg']):
        return None
    x_error = scan['x_m'] - goal['lateral_m']
    sign = 1 if math.cos(math.radians(goal['tag_heading_deg'])) >= 0 else -1
    relative_forward = sign * scan['forward_m']
    nearest_edge = min(sign * edge for edge in forward_window)
    if ('forward' in last_motion and abs(x_error) <= cfg['deadband_x_m']
            and .01 <= nearest_edge - relative_forward <= .22):
        return 'empty_forward2' if last_motion.startswith('empty_') else 'carry_forward2'
    if ('strafe' in last_motion and goal['allow_strafe']
            and forward_window[0] <= scan['forward_m'] <= forward_window[1]
            and cfg['deadband_x_m'] < abs(x_error) <= .16):
        direction = 'left' if x_error * sign > 0 else 'right'
        if last_motion.startswith('empty_'):
            return 'empty_strafe_'+direction
        return 'strafe_'+direction+'_once'
    return None


def Checkpoint_ReadHeading(robot, nav, check):
    from heading_guard import HeadingGuard
    from nav_observation import MagneticReference

    cfg=nav.cfg
    read_cfg=dict(cfg,post_action_heading=dict(cfg['post_action_heading'],execute=False),
                  visual_micro=dict(cfg.get('visual_micro',{}),execute=False))
    guard=HeadingGuard(read_cfg,nav.io,MagneticReference(cfg['magnetic']),check,robot.clock)
    guard.observation_scope=(getattr(robot.backend,'session',''),getattr(robot.backend,'epoch',-1))
    _,sample=guard.sample(robot.load_kind,action_completed_monotonic=robot.last_action_completed_monotonic)
    return sample if sample and sample.get('capture_verified') and nav.io.motion_capture_verified else None


def _record_sweep_offset(sweep, offset):
    low, high = sweep.get('coverage_deg', (0., 0.))
    sweep.update(offset_deg=offset, coverage_deg=(min(low, offset), max(high, offset)))


def _verified_search_heading(robot, cfg, sample, scope):
    capture_session, load_epoch, owner_scope, calibration_id = scope
    now = robot.clock()
    captures = sample.get('decision_capture_times') or ()
    received = sample.get('received_monotonic')
    return (sample.get('capture_verified') is True and finite(sample.get('raw_heading_deg'))
            and finite(received) and 0 <= now - received <= cfg['magnetic']['max_age_s']
            and captures and all(finite(t) and t <= received and 0 <= now - t <= cfg['magnetic']['max_age_s']
                                 for t in captures)
            and capture_session and type(load_epoch) is int and calibration_id
            and owner_scope == (getattr(robot.backend, 'session', ''), getattr(robot.backend, 'epoch', -1)))


def _select_measured_sweep_turn(robot, heading_cfg, sweep, offset):
    from micro_trial import MEDIUM_MAX_RESPONSE_DEG, MEDIUM_NOMINAL_DEG, MEDIUM_OPS, OPS, deltas, response_limit

    offsets = sweep['target_offsets']
    if (not isinstance(offsets, (tuple, list)) or not offsets
            or not all(finite(target) and abs(target) <= _SEARCH_ENVELOPE_DEG for target in offsets)):
        sweep.update(status='invalid_waypoints', reason='search waypoints are outside the allowed yaw envelope')
        return None
    if abs(offset) > _SEARCH_ENVELOPE_DEG:
        sweep.update(status='envelope_reached', reason='measured yaw is outside the search envelope',
                     limit_reached=True)
        return None

    budget = robot.micro_budget()
    usable_medium = getattr(robot.backend, 'medium_turn_ready', None)
    signs = deltas(heading_cfg)
    ineffective = sweep.get('ineffective_ops', ())
    first = sweep.get('target_index', 0)
    for index in range(first, len(offsets)):
        target = offsets[index]
        gap = target - offset
        if abs(gap) <= _SEARCH_WAYPOINT_TOLERANCE_DEG:
            sweep['target_index'] = index + 1
            continue
        direction = -1 if gap < 0 else 1
        micro = next((op for op in OPS if finite(signs.get(op)) and signs[op] * direction > 0), None)
        candidates = []
        medium_bound = MEDIUM_MAX_RESPONSE_DEG
        if (abs(gap) >= 15. and abs(offset + direction * medium_bound) <= _SEARCH_ENVELOPE_DEG
                and usable_medium is not None and usable_medium() is True):
            candidates.append((MEDIUM_OPS[int(direction > 0)], direction * MEDIUM_NOMINAL_DEG, medium_bound))
        if micro:
            candidates.append((micro, signs[micro], response_limit(heading_cfg)))
        for operation, signed, bound in candidates:
            if (operation not in ineffective and abs(offset + direction * bound) <= _SEARCH_ENVELOPE_DEG
                    and budget.available(signed, conservative_bound=bound, op=operation)):
                sweep.update(target_index=index, target_offset_deg=target)
                return operation, signed, bound
        sweep['target_index'] = index + 1
        sweep.setdefault('unreachable_offsets_deg', []).append(target)
    if sweep.get('unreachable_offsets_deg'):
        sweep.update(status='no_safe_turn', reason='remaining search waypoints have no effective bounded turn')
    else:
        sweep.update(status='complete', reason='all search waypoints checked')
    return None


def Checkpoint_TrySweepTurn(robot, nav, sweep, *, read_heading, error=None, pair_ops=(), check_action=None,
                            check_capture=None, accept_verified_angle=False):
    """Run one budgeted turn and compare post-settle yaw with the search origin."""
    from micro_trial import deltas, response_limit

    robot.check()
    cfg=nav.cfg
    measured = robot.load_kind == 'single' and error is None and 'target_offsets' in sweep
    if sweep['blocked'] or sweep.get('limit_reached') or (not measured and sweep['turns'] >= len(_SWEEP_ORDER)):
        return False
    if (robot.nav_count >= cfg['max_extra_actions_per_journey']
            or robot.count >= robot.cfg['max_body_macros_per_journey']):
        if measured:
            sweep.update(status='motion_budget_reached', reason='no body motion remains for Tag search')
        return False
    before=read_heading()
    if before is None:
        if measured:
            sweep.update(status='heading_unavailable', reason='verified search heading unavailable')
        return False
    sign=cfg['magnetic']['heading_sign']
    scope=tuple(before.get(key) for key in _MAGNETIC_WINDOW_SCOPE)
    if measured and (sign not in (-1, 1) or not _verified_search_heading(robot, cfg, before, scope)):
        sweep.update(status='heading_unavailable', reason='search heading is unverified or stale')
        return False
    angle=sign*before['raw_heading_deg']
    if sweep['origin'] is None:
        sweep['origin']=angle
        sweep['scope']=scope
    if scope!=sweep['scope']:
        sweep['blocked']=True
        if measured:
            sweep.update(status='heading_scope_changed', reason='search heading source, load or calibration changed')
        return False
    offset=wrap(angle-sweep['origin'])
    is_micro=robot.load_kind=='single'
    h=cfg['post_action_heading']
    if measured:
        _record_sweep_offset(sweep, offset)
        selected = _select_measured_sweep_turn(robot, h, sweep, offset)
        if selected is None:
            return False
        native, signed, bound = selected
        limit = _SEARCH_ENVELOPE_DEG
    else:
        index=_SWEEP_ORDER[sweep['turns']] if error is None else int(error<0)
        names=('left_turn','right_turn')
        if robot.load_kind=='empty':
            names=('empty_left_turn','empty_right_turn')
        if is_micro:
            names=('single_micro_left','single_micro_right')
            if error is not None:
                index=next(i for i,n in enumerate(names) if deltas(h)[n]*error<0)
        native=names[index]
        if robot.load_kind=='double':
            native='double_'+native
            if native not in pair_ops:
                return False
        signed=deltas(h)[native] if is_micro else (-30.,30.)[index]
        bound=response_limit(h) if is_micro else 45.
        limit=25. if is_micro else 50.
    direction=-1 if signed<0 else 1
    if signed==0 or abs(offset+direction*bound)>limit:
        return False
    if error is not None and (signed*error>=0 or bound>=2*(abs(error)-3.)):
        return False
    if check_action is not None and check_action() is False:
        if measured:
            sweep.update(status='camera_unavailable', reason='search requires a current calibrated camera view')
        return False
    (check_capture or robot.check_magnetic_before_motion)()
    if is_micro:
        robot.backend.check_micro()
        source = 'magnetic' if measured and native.startswith('single_medium_') else 'visual'
        if not robot.micro_budget().reserve(source, native, signed, conservative_bound=bound if measured else None):
            if measured:
                sweep.update(status='motion_budget_reached', reason='no turn allowance remains for Tag search')
            return False
    robot.completed_call(native,robot.cfg['request_timeout_s'])
    sweep['turns']+=1
    robot.count+=1
    robot.nav_count+=1
    after=read_heading()
    same_measurement_scope=after is not None and tuple(after.get(k) for k in _MAGNETIC_WINDOW_SCOPE)==scope
    feedback_only = accept_verified_angle or measured
    if feedback_only and same_measurement_scope:
        captures=after.get('decision_capture_times') or ()
        completed=robot.last_action_completed_monotonic
        now=robot.clock()
        before_received=before.get('received_monotonic')
        norms=(before.get('norm'),after.get('norm'))
        same_measurement_scope=(before.get('capture_verified') is True and after.get('capture_verified') is True
            and finite(completed) and finite(before_received) and before_received<=completed
            and after.get('action_completed_monotonic')==completed and captures
            and all(finite(t) and completed<t<=now and now-t<=cfg['magnetic']['max_age_s'] for t in captures)
            and finite(after.get('raw_heading_deg'))
            and all(finite(value) and value>0 for value in norms)
            and abs(norms[1]-norms[0])/norms[0]<=cfg['magnetic']['max_norm_fraction_change'])
    actual=wrap(sign*after['raw_heading_deg']-angle) if same_measurement_scope else None
    if measured and actual is not None:
        settled_offset = wrap(angle + actual - sweep['origin'])
        _record_sweep_offset(sweep, settled_offset)
        if abs(settled_offset) > limit or abs(actual) > bound:
            sweep.update(status='envelope_reached', reason='measured turn exceeded a search bound', limit_reached=True)
        elif direction * actual < _SEARCH_MIN_TURN_DEG:
            sweep.setdefault('ineffective_ops', []).append(native)
            sweep.update(status='turn_ineffective', reason='measured response too small or reversed, do not repeat gait')
        else:
            target = sweep['target_offset_deg']
            if (abs(settled_offset - target) <= _SEARCH_WAYPOINT_TOLERANCE_DEG
                    or direction * (settled_offset - target) >= 0):
                sweep['target_index'] += 1
            sweep.update(status='turn_measured', reason='search turn measured with fresh same-load heading')
    elif actual is None:
        sweep['blocked']=True
        if measured:
            sweep.update(status='heading_unavailable', reason='same-scope post-turn heading could not be verified')
    elif direction*actual < -3. or abs(actual)>bound or abs(wrap(angle+actual-sweep['origin']))>limit:
        sweep['limit_reached' if accept_verified_angle else 'blocked']=True
    if is_micro and actual is not None and not robot.micro_budget().reconcile(
            abs(actual),feedback_only_response=feedback_only):
        sweep['blocked']=True
    event = {'checkpoint_search_turn':native,'observed_delta_deg':actual,
             'sweep_limited':sweep['blocked'] or sweep.get('limit_reached',False)}
    if measured:
        event.update(offset_deg=sweep['offset_deg'], coverage_deg=sweep['coverage_deg'],
                     target_offset_deg=sweep['target_offset_deg'], search_status=sweep['status'])
    robot.events.append(event)
    return True


def validate_profiles(profiles,nav):
    if not isinstance(profiles,dict) or len(profiles)>18:raise ValueError('checkpoint profiles map')
    for key,p in profiles.items():
        if not isinstance(p,dict) or set(p)!={'id','direction','load','reference_load','cross_load_heading_only_verified','calibration_verified','execute','reference'}:
            raise ValueError('checkpoint profile schema')
        if type(p['id']) is not int or p['direction'] not in ('outbound','return') or p['load'] not in ('empty','single'):
            raise ValueError('checkpoint profile identity')
        if key!='{}:{}:{}'.format(p['direction'],p['load'],p['id']):raise ValueError('checkpoint key/identity mismatch')
        if not any(g['id']==p['id'] for g in nav['legs'][p['direction']]):raise ValueError('profile not in configured leg')
        if any(type(p[k]) is not bool for k in ('cross_load_heading_only_verified','calibration_verified','execute')):raise ValueError('checkpoint flags')
        if p['reference_load'] not in ('empty','single'):raise ValueError('reference load')
        if p['reference_load']!=p['load'] and p['cross_load_heading_only_verified'] is not True:raise ValueError('cross-load orientation not explicitly verified')
        if p['execute'] and (p['load']!='single' or not p['calibration_verified']):raise ValueError('only verified single micro exported')
        if not reference_valid(p['reference'],p['id']):raise ValueError('same-ID confirmed relative reference required')
        if p['execute']:
            from visual_micro import validate
            validate(profile_config(nav,p))
    return profiles


def profile_config(nav,profile):
    c=copy.deepcopy(nav);c['relative_heading_reference']=copy.deepcopy(profile['reference'])
    c['visual_micro'].update(enabled=True,execute=profile['execute'],target_id_confirmed=True,confirmed_target_id=profile['id'])
    return c


class FreshCheckpointIO:
    """Retain original reader lease; don't mutate persistent subscriber configuration."""
    def __init__(self,io):self.io=io
    def __getattr__(self,name):return getattr(self.io,name)
    def observe(self,tag,after_ns,timeout,check):
        # Each visual/magnetic pair consumes a new post-settle capture window.
        return self.io.observe(tag,max(after_ns,self.io.ros_now_ns()),timeout,check)


def align_if_configured(robot,tag,direction,*,deadline=None):
    key='{}:{}:{}'.format(direction,robot.load_kind,tag)
    p=robot.mission.get('checkpoint_profiles',{}).get(key)
    if p is None or not p['calibration_verified']:return None
    from visual_micro import VisualMicro
    from heading_guard import HeadingGuard
    nav=robot.ensure_navigation_io();cfg=profile_config(nav.cfg,p)
    if robot.heading_guard is None:
        robot.heading_guard=HeadingGuard(nav.cfg,nav.io,nav.mag,robot.check,robot.clock,
                budget=robot.micro_budget(),estimator=getattr(robot.backend,'turn_estimator',None))
    robot.heading_guard.observation_scope=robot.scope()
    def one(name,verify_pair):
        robot.check()
        if robot.load_kind!='single':raise RuntimeError('single micro cannot correct empty return')
        robot.backend.check_mission_v5();robot.backend.check_micro();robot.check_magnetic_before_motion()
        reserve=robot.cfg['request_timeout_s']+cfg['settle_s']+cfg['observe_s']+cfg['magnetic']['collect_s']
        if robot.count>=robot.cfg['max_body_macros_per_journey'] or robot.nav_count>=cfg['max_extra_actions_per_journey'] or robot.deadline-robot.clock()<=reserve:return None
        verify_pair();robot.check()
        robot.completed_call(name,robot.cfg['request_timeout_s']);robot.count+=1;robot.nav_count+=1
        robot.events.append({'v5_visual_micro':name,'physical_success':'unobserved'})
        return {'after_ns':nav.io.ros_now_ns()}
    # The profile quaternion is already this leg's desired facing. Its relative
    # target is zero, even when the global route heading for return is 180 deg.
    goal=dict(next(g for g in cfg['legs'][direction] if g['id']==tag),tag_heading_deg=0.)
    result=VisualMicro(cfg,FreshCheckpointIO(nav.io),nav.mag,robot.micro_budget(),one,
            robot.check,robot.clock,robot.load_kind,motion_model=robot.heading_guard).checkpoint(tag,goal,deadline=deadline)
    robot.heading_guard.latched=False
    result.update(position_controlled=False,calibration_scope=key,reference_load=p['reference_load'],
            cross_load_position_reuse=False)
    return result
