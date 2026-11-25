"""主要修改各命令的区域函数；保存后，下一次显式任务读取新源码。

禁止在文件顶层执行动作。尚未实现的区域保留 @pending，预检会在身体
动作之前拒绝整项任务；实际具备动作、视觉能力并验收后再移除标记。
"""

def pending(reason):
    """标注尚未具备运行条件的区域，不能用空函数假装已经完成。"""
    def mark(fn):fn.pending_reason=reason;return fn
    return mark


def cmd0_transfer_zone(robot):
    """命令0：中转区取预备资源，走到障碍区前，扫码返回后保持抱物。

    原空前、抓1块、抱前到路口已由用户实测；新增2步、右转4次及末端
    位置还需现场确认。这里不穿障碍、不放物，也不自动执行 stand。
    """
    robot.forward(5);robot.settle_since_last_completed(.3)          # 空手接近预备资源
    robot.grab(1);robot.settle_since_last_completed(.5)             # 抓1块；负载状态是期望值，未经传感确认
    robot.carry_forward(8);robot.settle_since_last_completed(.3)    # 保留已实测到路口的一段
    robot.carry_forward(2);robot.settle_since_last_completed(.3)    # 转弯前再前进约2个单脚步
    robot.right(3);robot.settle_since_last_completed(.5)            # 完整右转宏3次，不承诺真实角度
    robot.carry_forward(4);robot.settle_since_last_completed(.3)    # 转弯后约4步，保持原段参数
    return robot.scan_ar()                  # 停稳后有限等待AR新帧，原样返回结果


@pending('中转区方块数量识别尚未实现；AR Tag ID不是方块数量')
def shared_cmd12_transfer_zone_count_blocks(robot):
    """确认空返回0，识别数量返回1/2，无法确定返回None；观察必须有界。"""
    raise NotImplementedError('中转区视觉数块')


@pending('命令1/2在中转区接近待取资源的路线尚未实测')
def shared_cmd12_transfer_zone_approach_blocks(robot,count):
    raise NotImplementedError('中转区接近资源')


@pending('命令1/2从中转区取物点到公共障碍入口的路线尚未确定')
def shared_cmd12_transfer_zone_to_obstacle_entry(robot,count):
    raise NotImplementedError('中转区取物点到障碍入口')


def shared_cmd12_transfer_zone(robot):
    """命令1/2共用中转区：视觉数块、接近、抓取、到公共去程入口。"""
    count=shared_cmd12_transfer_zone_count_blocks(robot)
    if count is None:return False
    if type(count) is not int or count not in (0,1,2):raise ValueError('block count must be 0/1/2 or None')
    if count==0:return False
    shared_cmd12_transfer_zone_approach_blocks(robot,count)
    robot.grab(count)
    shared_cmd12_transfer_zone_to_obstacle_entry(robot,count)
    return True


@pending('命令0/1/2共用的去程障碍区尚未实现；当前命令0不会调用')
def shared_cmd012_obstacle_zone_outbound(robot):
    """去程方向：中转区 → 大本营；禁止与命令2返程混用。"""
    raise NotImplementedError('公共去程障碍区')


@pending('命令0/1/2共用的大本营接近及放物尚未实现')
def shared_cmd012_base_zone(robot):
    """大本营：走到放物位置并放下；当前不是可执行动作。"""
    raise NotImplementedError('公共大本营放物')


def shared_cmd012_outbound_delivery(robot):
    """未来公共去程：障碍区 → 大本营。当前仅命令1/2骨架引用。"""
    shared_cmd012_obstacle_zone_outbound(robot)
    shared_cmd012_base_zone(robot)


@pending('命令2从大本营到返程障碍入口的路线尚未实现')
def cmd2_base_zone_return(robot):
    raise NotImplementedError('大本营进入返程障碍区')


@pending('命令2反向穿过障碍区尚未实现')
def cmd2_obstacle_zone_return(robot):
    """返程方向：大本营 → 中转区，不能复用去程顺序冒充返程。"""
    raise NotImplementedError('返程障碍区')


@pending('命令2从返程障碍出口到中转区取物点的路线尚未确定')
def cmd2_transfer_zone_return(robot):
    raise NotImplementedError('返程障碍出口到中转区取物点')


def cmd0(robot):
    """走到通道入口，返回 Tag 1 检查点供命令 0 继续执行。"""
    if not getattr(robot,'command0_scan_after_turn',False):
        return cmd0_transfer_zone(robot)
    robot.forward(5);robot.settle_since_last_completed(.3)
    robot.grab(1);robot.settle_since_last_completed(.5)
    robot.carry_forward(8)
    robot.right(3)
    robot.settle_since_last_completed(.5)
    return robot.scan_ar()


def cmd1(robot):
    """未来命令1：中转区取物 → 公共去程障碍 → 大本营放物一次。"""
    if not shared_cmd12_transfer_zone(robot):raise RuntimeError('no reliable transferred resources; stop before pickup')
    shared_cmd012_outbound_delivery(robot)


def cmd2(robot):
    """未来命令2：首次同1，之后返程再取物，循环到显式停止。"""
    while True:
        robot.check()
        robot.new_journey() # 每趟有界预算重新计数，不限制总往返次数
        if not shared_cmd12_transfer_zone(robot):
            robot.wait(robot.empty_retry_s)
            continue
        shared_cmd012_outbound_delivery(robot)
        cmd2_base_zone_return(robot)
        cmd2_obstacle_zone_return(robot)
        cmd2_transfer_zone_return(robot)
        # 下一轮复用相同的中转区取物和公共去程函数。


def required_sections(mode):
    """动作前检查所有会用到的区域；未知阶段不允许走到半途再发现。"""
    if mode=='stand':return [] # 固定站姿独立于路线，不扫描、不开始运输任务
    if mode=='stage1':return [cmd0_transfer_zone]
    if mode=='0':return [cmd0,cmd0_transfer_zone]
    transfer=[shared_cmd12_transfer_zone_count_blocks,shared_cmd12_transfer_zone_approach_blocks,shared_cmd12_transfer_zone_to_obstacle_entry]
    common=[shared_cmd012_obstacle_zone_outbound,shared_cmd012_base_zone]
    if mode=='1':return transfer+common
    if mode=='2':return transfer+common+[cmd2_base_zone_return,cmd2_obstacle_zone_return,cmd2_transfer_zone_return]
    raise ValueError('unknown command')


# 仅保留原公开入口兼容；实现与人工修改统一放在上面的区域函数中。
stage1=cmd0_transfer_zone
command0=cmd0
command1=cmd1
command2=cmd2
