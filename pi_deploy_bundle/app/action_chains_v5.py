"""命令 0 的通道和放物动作，以及命令 1/2 与 0-full 的动作链。

repeat 是原生有限宏次数。后续次数按 R3 现有动作与学长路线估算，现场试跑后修改。
FIELD_VALIDATED 标识路线证据。抓放由状态机按实际选择的抓取数量来发出。
"""

from cmd0_motion import COMMAND0_ENTRY_FORWARD_STEPS


FIELD_VALIDATED = False

# Logical step counts are grouped into supported native actions at dispatch.
# Each Tag entry gates the actions that follow it.
COMMAND0_OUTBOUND = (
    {'tag': 1, 'direction': 'outbound'},
    {'op': 'carry_forward_group', 'steps': COMMAND0_ENTRY_FORWARD_STEPS},
    {'tag': 2, 'direction': 'outbound'},
    {'op': 'strafe_right_once', 'repeat': 5},
    {'tag': 3, 'direction': 'outbound'},
    {'op': 'strafe_right_once', 'repeat': 3, 'unverified_repeat': 4, 'unverified_last_small': True},
    {'op': 'carry_forward_group', 'steps': 8},
    {'op': 'carry_forward_group', 'steps': 4},
    {'tag': 4, 'direction': 'outbound'},
    {'op': 'strafe_left_once', 'repeat': 6, 'monitor_every': 3},
    {'tag': 5, 'direction': 'outbound'},
)
COMMAND0_DELIVERY = (
    {'op': 'carry_forward_group', 'steps': 4},
    {'op': 'carry_forward_group', 'steps': 4},
    {'op': 'place_single'},
)

CHAINS = {
    'pickup_search': (),          # 有限原地搜索；观察后才可抓，不能盲抓
    'pickup_left': ({'op': 'empty_strafe_left'},),
    'pickup_right': ({'op': 'empty_strafe_right'},),
    'pickup_advance': ({'op': 'empty_forward2'},),
    # 首轮以命令 0 的 8+2、右转组为起点。视觉抓取后的起始位姿须实测复核。
    'transfer_to_entry': (
        {'op': 'carry_forward', 'allow_heading_correction': True},
        {'op': 'carry_forward2', 'allow_heading_correction': True},
        {'op': 'right_turn', 'repeat': 3, 'target_delta_deg': 90.0},
    ),
    'outbound_obstacles': (
        {'tag': 1, 'direction': 'outbound'},
        {'op': 'carry_forward'},                      # 1 → 2，沿用已跑过的 8+2 分组
        {'op': 'carry_forward2'},
        {'tag': 2, 'direction': 'outbound'},
        {'op': 'strafe_right_once', 'repeat': 3},      # 2 → 3
        {'tag': 3, 'direction': 'outbound'},
        {'op': 'strafe_right_once', 'repeat': 2},      # 绕过中间挡板端头
        {'op': 'carry_forward'},                      # 3 → 4
        {'tag': 4, 'direction': 'outbound'},
        {'op': 'strafe_left_once', 'repeat': 3},       # 对准大本营开口
        {'op': 'carry_forward2'},                     # 4 → 5
        {'tag': 5, 'direction': 'outbound'},
    ),
    'base_approach': ({'op': 'carry_forward4'}, {'op': 'carry_forward2'}),
    'base_to_return': (
        {'op': 'empty_right_turn', 'repeat': 6, 'target_delta_deg': 180.0},
        {'op': 'empty_forward2'},                     # 大本营空载掉头后到 5
    ),
    'return_obstacles': (
        {'tag': 5, 'direction': 'return'},
        {'op': 'empty_forward2', 'repeat': 2},         # 5 → 6
        {'tag': 6, 'direction': 'return'},
        {'op': 'empty_strafe_left', 'repeat': 4},      # 空载绕过同一挡板的开口
        {'op': 'empty_forward2', 'repeat': 3},         # 6 → 7
        {'tag': 7, 'direction': 'return'},
        {'op': 'empty_strafe_right', 'repeat': 5},     # 对准中转区出口
        {'op': 'empty_forward2'},                     # 7 → 1
        {'tag': 1, 'direction': 'return'},
    ),
    'return_to_transfer': (
        {'op': 'empty_forward2', 'repeat': 2},        # 先离开通道，进入起始区
        {'op': 'empty_strafe_left', 'repeat': 5},     # 朝起始区靠墙工位，实际净空待测
        {'op': 'empty_right_turn', 'repeat': 3, 'target_delta_deg': 90.0},  # 面向区内找块
    ),
    'start_to_transfer': ({'op': 'carry_forward2'},),  # 0-full 独立连接段
}


def demo_chains():
    """验证实际写出的估算次数与顺序。"""
    result = {name: tuple(dict(step) for step in steps) for name, steps in CHAINS.items()}
    result['pickup_search'] = ({'op': 'empty_left_turn'}, {'op': 'empty_right_turn'})
    return result
