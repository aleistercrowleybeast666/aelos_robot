"""面对前方的障碍区备选路线；步数需沿实际场地填写，整段预检后才发动作。"""
import json
from pathlib import Path
from task_runner import NotReady,segments

def prepare_facing_route(project,direction,load_kind):
    c=json.loads((Path(project)/'config/facing_route.json').read_text(encoding='utf-8'))
    if c.get('field_path_confirmed') is not True:raise NotReady('forward-facing corridor path/counts not field confirmed')
    if direction not in ('outbound','return'):raise ValueError('unknown corridor direction')
    if direction=='return':raise NotReady('empty-load finite strafe/return exports not installed; do not copy carried route')
    if load_kind!='single':raise NotReady('current forward-facing outbound requires single load')
    plan=c['outbound']
    for key in ('to2_forward','to4_forward','to5_forward'):segments(plan.get(key),'single')
    for key in ('to3_right','to4_right','to5_left'):
        if type(plan.get(key)) is not int or not 1<=plan[key]<=20:raise NotReady('side-cycle count not set: '+key)
    return plan

def shared_cmd012_obstacle_zone_forward_facing(robot):
    """备选去程函数：人只改facing_route中的短段，依次尝试1→2→3→4→5。"""
    p=prepare_facing_route(robot.project,'outbound',robot.load_kind)
    robot.calibrate_tag(1,'outbound')
    robot.carry_forward(p['to2_forward'])
    robot.calibrate_tag(2,'outbound')
    robot.strafe_right(p['to3_right'])
    robot.calibrate_tag(3,'outbound')
    robot.strafe_right(p['to4_right']);robot.carry_forward(p['to4_forward'])
    robot.calibrate_tag(4,'outbound')
    robot.strafe_left(p['to5_left']);robot.carry_forward(p['to5_forward'])
    robot.calibrate_tag(5,'outbound')

def cmd2_obstacle_zone_return_forward_facing(robot):
    """返程码序已支持5→6→7→1；真实空载动作/路线缺项时动作前拒绝。"""
    prepare_facing_route(robot.project,'return',robot.load_kind)


def prepare_recorded_outbound(robot):
    """旧用户去程规划的完整前置检查，当前9/7步与放物均缺项，不发半条链。"""
    missing=[]
    if robot.load_kind!='single':missing.append('recorded outbound requires single load')
    for n in (9,14,8,7,10):
        try:segments(n,'single')
        except (ValueError,NotReady) as exc:missing.append(str(n)+': '+str(exc))
    missing.append('placement macro not exported; final location not field confirmed')
    # 当前协议没有place，原计划也不是新的侧跨等价路线；必须显式补齐后再改门禁。
    raise NotReady('recorded outbound unavailable BEFORE motion: '+'; '.join(missing))


def shared_cmd012_obstacle_zone_recorded_turning_plan(robot):
    """逐行保留用户原规划，供人修改/比较；不以猜测补步，更不默认接入命令0。

    这是原来反复转向版本；转角未实测，不能凭这些次数推算等价侧跨距离。
    ID对应哪一段尚未知，故不臆造逐段校准点。
    """
    prepare_recorded_outbound(robot)
    robot.carry_forward(9)
    robot.right(3)
    robot.carry_forward(14)
    robot.left(2)
    robot.right(1)
    robot.carry_forward(8)
    robot.left(1)
    robot.carry_forward(7)
    robot.right(3)
    robot.carry_forward(10)
    # 放物属于尚未实现的大本营函数，不能用stand松手冒充。

