"""Select finite command 0 macros supported by the verified live bridge."""

import json
from math import isfinite
from pathlib import Path


COMMAND0_TRANSFER_FORWARD_STEPS = 8
COMMAND0_ENTRY_FORWARD_STEPS = 10
TRANSFER_FORWARD9_ACTION = 'carry_forward9'
FORWARD_ACTIONS = {steps: f'cmd0_carry_forward{steps}' for steps in (5, 10, 15)}
_FORWARD_STEP_COUNTS = {**{op: steps for steps, op in FORWARD_ACTIONS.items()},
                        'carry_forward': 8, 'carry_forward2': 2, 'carry_forward4': 4,
                        'carry_forward7': 7, 'carry_forward9': 9}
SIDE_ACTIONS = {
    (f'strafe_{direction}_once', cycles): f'cmd0_strafe_{direction}{cycles}'
    for direction, supported in (('left', (2, 3, 4, 6)), ('right', (2, 3, 4, 5)))
    for cycles in supported
}
SMALL_SIDE_ACTIONS = {'strafe_left_once': 'single_small_strafe_left',
                      'strafe_right_once': 'single_small_strafe_right'}
LEGACY_SMALL_STRAFE_LIMIT_M = .035
INTERMEDIATE_SMALL_STRAFE_LIMIT_M = .060
TRANSFER_SMALL_STRAFE_LIMIT_M = .070
PLACEMENT_SMALL_STRAFE_LIMIT_M = .080
PLACE_ACTION = 'cmd0_place_single'
PICKUP_BACKWARD4_ACTION = 'cmd1_carry_backward4'
PICKUP_BACKWARD6_ACTION = 'cmd1_carry_backward6'
PICKUP_BACKWARD8_ACTION = 'cmd1_carry_backward8'
PICKUP_EXIT_ACTIONS = ('cmd0_carry_backward2', 'cmd0_strafe_left6')
PICKUP_MATCH_OUTBOUND_STRAFES = 'match_outbound_strafes'
_PICKUP_EXIT_AVAILABLE = frozenset((*PICKUP_EXIT_ACTIONS, *SIDE_ACTIONS.values(),
    *SMALL_SIDE_ACTIONS.values(), 'carry_forward', 'carry_forward2', 'carry_forward4',
    'strafe_left_once', 'strafe_right_once', PICKUP_MATCH_OUTBOUND_STRAFES))


class StandingStartUnavailable(RuntimeError):
    """The optional start was declined before any physical action began."""


class PickupRetreatUnavailable(RuntimeError):
    """The MCU returned an unsupported action before entering the retreat."""


def Cmd0Motion_SmallStrafeProfile(backend):
    """Only use the old, narrower sweep when this MCU session identified that gait."""
    profile = getattr(backend, 'small_strafe_profile', lambda: None)()
    if profile == 'legacy_v1':
        return profile, LEGACY_SMALL_STRAFE_LIMIT_M
    if profile == 'intermediate_v2':
        return profile, INTERMEDIATE_SMALL_STRAFE_LIMIT_M
    if profile == 'transfer_v3':
        return profile, TRANSFER_SMALL_STRAFE_LIMIT_M
    return ('placement_v4' if profile == 'placement_v4' else None), PLACEMENT_SMALL_STRAFE_LIMIT_M


def _validate_transfer_chain(chain):
    if not isinstance(chain, list) or not 3 <= len(chain) <= 24:
        raise ValueError('command0_transfer 需要按顺序列出中转区动作')
    stage = 0
    for step in chain:
        if not isinstance(step, dict) or type(step.get('op')) is not str:
            raise ValueError('中转区每项动作需要 op 名称')
        op = step['op']
        if op == 'wait' and set(step) == {'op', 'seconds'}:
            seconds = step['seconds']
            if type(seconds) in (int, float) and 0 <= seconds <= 10:
                continue
        if stage == 0:
            if op == 'free_forward' and set(step) == {'op', 'steps'} and type(step['steps']) is int and 5 <= step['steps'] <= 80 and step['steps'] % 5 == 0:
                continue
            if op == 'grab_single' and set(step) == {'op'}:
                stage = 1
                continue
        if stage == 1 and op == 'carry_forward_group' and set(step) == {'op'}:
            stage = 2
            continue
        turn_keys = {'op', 'repeat', 'target_delta_deg'}
        if stage == 2 and op in ('right_turn', 'left_turn') and turn_keys <= step.keys() <= turn_keys | {'nominal_step_deg'}:
            repeat, angle = step['repeat'], step['target_delta_deg']
            sign = 1 if op == 'right_turn' else -1
            nominal = step.get('nominal_step_deg', 30)
            if (type(repeat) is int and 1 <= repeat <= 20 and type(angle) in (int, float)
                    and 0 < sign*angle <= 180 and type(nominal) in (int, float) and 0 < nominal <= 90):
                stage = 3
                continue
        raise ValueError('中转区动作或顺序无效，需依次抓取、抱物前进并转向：'+op)
    if stage != 3:
        raise ValueError('中转区需要抓取、抱物前进和一次计划转向')
    return chain


def Cmd0Motion_LoadRouteSteps(project):
    defaults = {'transfer_forward_steps': COMMAND0_TRANSFER_FORWARD_STEPS,
                'tag1_to_tag2_forward_steps': COMMAND0_ENTRY_FORWARD_STEPS}
    path = Path(project) / 'config' / 'route_steps.json'
    try:
        steps = json.loads(path.read_text(encoding='utf-8-sig'))
    except FileNotFoundError:
        return defaults
    except (OSError, ValueError) as exc:
        raise ValueError('无法读取路线步数配置 route_steps.json') from exc
    optional = {'command0_transfer', 'pickup_exit', 'command1_pickup_exit', 'corridor', 'return_chain', 'wall_pickup_advance_steps',
                'micro_turn_filter', 'tag5_entry_lateral', 'tag2_forward_trial', 'command2_continue_return',
                'continue_estimated_route', 'command2_handoff_reference', 'wall_pickup_side_limit', 'tag4_forward_trial', 'command2_before_base_turn_extra_steps',
                'command2_empty_departure'}
    if not isinstance(steps, dict) or not defaults.keys() <= steps.keys() or steps.keys()-defaults.keys()-optional:
        raise ValueError('路线步数配置需要 transfer_forward_steps 和 tag1_to_tag2_forward_steps 两项')
    if any(type(steps[key]) is not int or not 2 <= steps[key] <= 80 for key in defaults):
        raise ValueError('路线步数须为 2 至 80 的整数')
    if steps['transfer_forward_steps'] % 2 and steps['transfer_forward_steps'] not in (9, 11):
        raise ValueError('中转区抱物前进使用偶数步数，或可分两组的 9 步、11 步')
    if 'command0_transfer' in steps:
        steps['command0_transfer'] = _validate_transfer_chain(steps['command0_transfer'])
    for field in ('pickup_exit', 'command1_pickup_exit'):
        if field not in steps:
            continue
        actions = steps[field]
        if (not isinstance(actions, list) or len(actions)>20
                or any(type(action) is not str or action not in _PICKUP_EXIT_AVAILABLE for action in actions)
                or actions.count(PICKUP_MATCH_OUTBOUND_STRAFES) > 1):
            raise ValueError(f'{field} 包含不支持的抱物动作')
    if 'continue_estimated_route' in steps and type(steps['continue_estimated_route']) is not bool:
        raise ValueError('continue_estimated_route 需填写 true 或 false')
    if 'command2_continue_return' in steps and type(steps['command2_continue_return']) is not bool:
        raise ValueError('command2_continue_return 需填写 true 或 false')
    before_base_turn = steps.get('command2_before_base_turn_extra_steps', 0)
    if 'command2_empty_departure' in steps:
        departure = steps['command2_empty_departure']
        if not isinstance(departure, dict) or not {'gate_lateral_tolerance_m', 'side_departure_depth_m'} <= departure.keys() <= {
                'gate_lateral_tolerance_m', 'side_departure_depth_m', 'tag6_side_departure_depth_m', 'handoff_strafe_min_depth_m'}:
            raise ValueError('返程出发位姿需要门口横向容差和侧跨前距离范围')
        tolerance = departure['gate_lateral_tolerance_m']
        if type(tolerance) not in (int, float) or not .025 <= tolerance <= .10:
            raise ValueError('返程出发横向容差无效，距离单位为米')
        for key in ('side_departure_depth_m', 'tag6_side_departure_depth_m'):
            band = departure.get(key, departure['side_departure_depth_m'])
            if (not isinstance(band, list) or len(band) != 2
                    or not all(type(value) in (int, float) and isfinite(value) for value in band)
                    or not .06 <= band[0] < band[1] <= .35):
                raise ValueError('返程出发位姿范围无效，距离单位为米')
        staging = departure.get('handoff_strafe_min_depth_m', .18)
        if type(staging) not in (float, int) or not isfinite(staging) or not .12 <= staging <= .30:
            raise ValueError('接驳取物前的侧跨退距无效，单位为米')
    if type(before_base_turn) is not int or before_base_turn not in (0, 2, 4):
        raise ValueError('命令 2 第一次放物前可在大本营内补进 0、2 或 4 步')
    if 'corridor' in steps:
        from corridor_trial import CorridorTrial_ValidateChain
        steps['corridor'] = CorridorTrial_ValidateChain(steps['corridor'])
    if 'return_chain' in steps:
        from return_trial import ReturnTrial_ValidateChain
        steps['return_chain'] = ReturnTrial_ValidateChain(steps['return_chain'])
    if 'command2_handoff_reference' in steps:
        ref = steps['command2_handoff_reference']
        ranges = {'lateral_m':(0,1), 'depth_m':(0,.5), 'heading_deg':(-90,90),
                  'lateral_tolerance_m':(.005,.12), 'depth_tolerance_m':(.005,.12), 'heading_tolerance_deg':(1,25)}
        if (not isinstance(ref,dict) or set(ref) != {*ranges,'source_report'}
                or not isinstance(ref['source_report'],str) or not ref['source_report']
                or any(type(ref[k]) not in (float,int) or not lo <= ref[k] <= hi for k,(lo,hi) in ranges.items())):
            raise ValueError('Tag 1 接驳示教需要有效的地面码坐标、朝向和容差')
    if 'wall_pickup_advance_steps' in steps:
        approach = steps['wall_pickup_advance_steps']
        if type(approach) is not int or approach not in (0, 2, 4):
            raise ValueError('交接带最多前进 0、2 或 4 步')
    if 'wall_pickup_side_limit' in steps:
        limit = steps['wall_pickup_side_limit']
        if type(limit) is not int or not 0 <= limit <= 4:
            raise ValueError('交接带的横移次数应为 0 至 4')
    if 'micro_turn_filter' in steps:
        filtering = steps['micro_turn_filter']
        if (not isinstance(filtering, dict) or filtering.keys() != {'initial_deg', 'alpha'}
                or type(filtering['initial_deg']) not in (int, float) or not 1 <= filtering['initial_deg'] <= 20
                or type(filtering['alpha']) not in (int, float) or not 0 <= filtering['alpha'] <= 1):
            raise ValueError('微转滤波需填写 1 至 20 度的 initial_deg 和 0 至 1 的 alpha')
    if 'tag5_entry_lateral' in steps:
        align = steps['tag5_entry_lateral']
        keys = {'enabled', 'target_map_y_m', 'deadband_m', 'large_move_error_m', 'forward_window_m'}
        if (not isinstance(align, dict) or not keys <= align.keys() or align.keys()-keys-{'verify_passage'}
                or type(align['enabled']) is not bool or type(align.get('verify_passage',False)) is not bool):
            raise ValueError('Tag 5 进场横向补偿配置无效')
        window = align['forward_window_m']
        target, deadband, large = (align[key] for key in ('target_map_y_m', 'deadband_m', 'large_move_error_m'))
        if (not isinstance(window, list) or len(window) != 2
                or not all(type(value) in (int, float) and isfinite(value)
                           for value in (target, deadband, large, *window))
                or not 0 < deadband < large or not 0 < window[0] < window[1]):
            raise ValueError('Tag 5 进场横向补偿需填写有效的米制目标、容差和前方范围')
    for key, tag in (('tag2_forward_trial', 2), ('tag4_forward_trial', 4)):
        if key not in steps:
            continue
        trim = steps[key]
        keys = {'enabled', 'target_distance_m', 'deadband_m', 'two_step_distance_m', 'distance_window_m'}
        optional = ({'approach_two_step_distance_m', 'minimum_strafe_distance_m'} if tag == 2
                    else {'left_departure_reference', 'heading_use_within_m'})
        if (not isinstance(trim, dict) or not keys <= trim.keys() <= keys | optional
                or type(trim['enabled']) is not bool):
            raise ValueError(f'Tag {tag} 前后试跑补偿配置无效')
        travel, window = trim['two_step_distance_m'], trim['distance_window_m']
        approach = trim.get('approach_two_step_distance_m', travel)
        target, deadband = trim['target_distance_m'], trim['deadband_m']
        if (not all(isinstance(bounds, list) and len(bounds)==2 for bounds in (travel, approach, window))
                or not all(type(value) in (int, float) and isfinite(value)
                           for value in (target, deadband, *travel, *approach, *window))
                or not 0 < window[0] < target < window[1] or deadband <= 0
                or not 0 < travel[0] < travel[1] or not 0 < approach[0] < approach[1]):
            raise ValueError(f'Tag {tag} 前后试跑补偿需填写有效的目标距离、容差和两步距离范围')
        if 'minimum_strafe_distance_m' in trim:
            minimum = trim['minimum_strafe_distance_m']
            if (type(minimum) not in (int, float) or not window[0] <= minimum <= target
                    or not isfinite(minimum)):
                raise ValueError('Tag 2 最小起跨距离须在观测下限和目标距离之间（含边界）')
        if 'heading_use_within_m' in trim:
            cutoff = trim['heading_use_within_m']
            if type(cutoff) not in (int, float) or not isfinite(cutoff) or not target+deadband < cutoff <= window[1]:
                raise ValueError('Tag 4 航向使用距离须大于进近目标范围，并位于观测范围内')
        if 'left_departure_reference' in trim:
            reference = trim['left_departure_reference']
            fields = {'distance_m', 'distance_deadband_m', 'lateral_m', 'heading_deg'}
            if (not isinstance(reference, dict) or reference.keys() != fields
                    or not all(type(value) in (int, float) and isfinite(value) for value in reference.values())):
                raise ValueError('Tag 4 左跨参考需填写有限数值的距离、距离容差、横偏和航向')
            distance, tolerance = reference['distance_m'], reference['distance_deadband_m']
            if (tolerance <= 0 or not window[0] <= distance - tolerance < distance + tolerance <= window[1]
                    or abs(reference['lateral_m']) > .16 or abs(reference['heading_deg']) > 15.):
                raise ValueError('Tag 4 左跨参考距离及容差需落在观测范围内，横偏不大于 0.16 米，航向不大于 15 度')
    return steps


def Cmd0Motion_ContinuousReady(backend):
    check = getattr(backend, 'cmd0_continuous_ready', None)
    return check is not None and check() is True


def Cmd0Motion_LegacyReady(backend):
    check = getattr(backend, 'cmd0_legacy_ready', None)
    return check is not None and check() is True


def Cmd0Motion_ForwardActions(steps, *, continuous, legacy=False):
    if continuous and steps in FORWARD_ACTIONS:
        return (FORWARD_ACTIONS[steps],)
    if steps == 10:
        return ('carry_forward', 'carry_forward2')
    if steps == 12:
        return ('carry_forward', 'carry_forward4')
    if steps == 15 and legacy:
        return ('carry_forward7', 'carry_forward')
    return ()


def Cmd0Motion_ForwardStepCount(actions):
    """Count nominal strides in acknowledged macros."""
    return sum(_FORWARD_STEP_COUNTS[action] for action in actions)
