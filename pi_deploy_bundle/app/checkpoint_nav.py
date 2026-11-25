"""可插入动作链的有限扫码校准；普通失码返回续走，动作/通信异常继续向外抛出。"""
import collections,json,math,time
from pathlib import Path
from nav_observation import summarize,wrap,MagneticReference,axis,cross

class NavigationConfigError(ValueError):pass
def load(project):
    p=Path(project)/'config/navigation.json'
    if not p.exists():return {'enabled':False}
    c=json.loads(p.read_text(encoding='utf-8'))
    if c.get('schema')!='r3.checkpoints.v1' or type(c.get('enabled')) is not bool:raise NavigationConfigError('navigation schema')
    for flag in ('search_enabled','pose_correction_enabled'):
        if type(c.get(flag)) is not bool:raise NavigationConfigError(flag)
    if type(c['magnetic'].get('capture_freshness_verified')) is not bool:raise NavigationConfigError('magnetic capture freshness flag')
    health=c.get('camera_health',{})
    if type(health.get('topic_confirmed')) is not bool or not isinstance(health.get('topic'),str) or not health['topic'].startswith('/'):raise NavigationConfigError('camera health contract')
    if type(health.get('max_age_s')) not in (int,float) or not math.isfinite(health['max_age_s']) or not .1<=health['max_age_s']<=3:raise NavigationConfigError('camera health age')
    if type(c.get('adaptive_coarse_turn_selection',False)) is not bool:raise NavigationConfigError('adaptive coarse selection flag')
    if type(c.get('early_visual_at_action_boundaries',False)) is not bool:raise NavigationConfigError('early visual boundary flag')
    groups=c.get('planned_turn_groups',{})
    if not isinstance(groups,dict):raise NavigationConfigError('planned turn groups')
    for key,delta in groups.items():
        try:name,count=key.split(':');count=int(count)
        except (ValueError,AttributeError):raise NavigationConfigError('planned turn group key')
        if name not in ('left_turn','right_turn') or not 1<=count<=20 or type(delta) not in (int,float) or not math.isfinite(delta) or not 0<abs(delta)<=180 or delta*(1 if name=='right_turn' else -1)<=0:raise NavigationConfigError('planned turn target invalid')
    from heading_guard import validate
    validate(c)
    from visual_micro import validate as validate_visual_micro
    validate_visual_micro(c)
    for k,lo,hi in [('min_frames',3,10),('max_correction_actions',0,6),('max_extra_actions_per_journey',0,40)]:
        if type(c.get(k)) is not int or not lo<=c[k]<=hi:raise NavigationConfigError(k)
    for k,lo,hi in [('max_age_s',.1,3),('observe_s',.1,10),('settle_s',0,3),('deadband_heading_deg',1,90),('deadband_x_m',.01,.5),('checkpoint_timeout_s',1,120),('max_heading_mad_deg',1,45),('max_x_mad_m',.005,.3),('outlier_heading_floor_deg',1,30),('outlier_x_floor_m',.005,.3),('minimum_improvement',0,.9)]:
        if type(c.get(k)) not in (float,int) or not math.isfinite(c[k]) or not lo<=c[k]<=hi:raise NavigationConfigError(k)
    if not c['deadband_heading_deg']<c['max_correctable_heading_deg']<=90 or not c['deadband_x_m']<c['max_correctable_x_m']<=1:raise NavigationConfigError('coarse correction envelope')
    if c['search_pattern']!=['right_turn','left_turn','left_turn','right_turn']:raise NavigationConfigError('bounded balanced search pattern required')
    if type(c.get('window_frames')) is not int or not c['min_frames']<=c['window_frames']<=16 or type(c.get('action_directions_confirmed')) is not bool:raise NavigationConfigError('window/action settings')
    if not isinstance(c['topic'],str) or c['topic']!='/chest/ar_pose_marker':raise NavigationConfigError('single chest source required')
    for direction in ('outbound','return'):
        points=c['legs'][direction]
        if not points or len(points)>16 or any(type(x.get('id')) is not int or type(x.get('allow_turn')) is not bool or type(x.get('allow_strafe')) is not bool for x in points):raise NavigationConfigError('checkpoint list')
        if len({p['id'] for p in points})!=len(points):raise NavigationConfigError('ambiguous checkpoint order')
        for point in points:
            for k in ('tag_heading_deg','route_heading_deg','lateral_m'):
                if type(point.get(k)) not in (int,float) or not math.isfinite(point[k]):raise NavigationConfigError('checkpoint goal')
    pc=c['pose_contract']
    if cross(axis(pc['body_right']),axis(pc['body_forward']))!=axis(pc['body_up']) or pc['tag_forward'] not in ('+x','-x','+y','-y'):raise NavigationConfigError('ground axes')
    if type(pc['tag_up_sign']) is not int or pc['tag_up_sign'] not in (-1,1):raise NavigationConfigError('Tag up sign')
    if any(type(pc[k]) is not bool for k in ('axes_confirmed','scale_confirmed','capture_time_confirmed')):raise NavigationConfigError('pose contract flags')
    for k,lo,hi in [('max_ground_tilt_deg',1,80),('max_range_m',.1,10),('marker_size_cm',.1,100)]:
        if type(pc[k]) not in (int,float) or not math.isfinite(pc[k]) or not lo<=pc[k]<=hi:raise NavigationConfigError('pose contract '+k)
    m=c['magnetic']
    if type(m['enabled']) is not bool or m['heading_sign'] is not None and (type(m['heading_sign']) is not int or m['heading_sign'] not in (-1,1)) or type(m['min_samples']) is not int or not 3<=m['min_samples']<=10:raise NavigationConfigError('magnetic config')
    for k in ('max_age_s','max_mad_deg','max_norm_fraction_change','max_offset_jump_deg','collect_s'):
        if type(m[k]) not in (int,float) or not math.isfinite(m[k]) or m[k]<=0:raise NavigationConfigError('magnetic '+k)
    return c

def motion_verified(c,strafe=False):
    h=c.get('post_action_heading',{})
    if not c.get('action_directions_confirmed') or not all(h.get(k) is True for k in ('directions_confirmed','gain_confirmed','safe_stop_confirmed')):return False
    gains=h.get('turn_delta_deg',{})
    for n in ('left_turn','right_turn'):
        v=gains.get(n)
        if type(v) not in (int,float) or not math.isfinite(v) or not 0<abs(v)<=h.get('max_step_deg',0):return False
    if gains['left_turn']*gains['right_turn']>=0:return False
    return not strafe or h.get('strafe_gain_confirmed') is True

class Navigator:
    def __init__(self,cfg,io,action,check,load_kind,clock=time.monotonic,can_search=lambda n:True):
        self.cfg=cfg;self.io=io;self.action=action;self.check=check;self.load_kind=load_kind;self.clock=clock
        self.mag=MagneticReference(cfg['magnetic']);self.leg=None;self.cursor=-1;self.history=collections.deque(maxlen=32);self.can_search=can_search
    def close(self):self.io.close()
    def checkpoint(self,tag_id,direction='outbound'):
        c=self.cfg;points=c['legs'][direction]
        if direction!=self.leg:self.leg=direction;self.cursor=-1
        next_index=self.cursor+1
        if self.cursor>=0 and points[self.cursor]['id']==tag_id:index=self.cursor
        elif next_index<len(points) and points[next_index]['id']==tag_id:index=next_index
        else:raise NavigationConfigError('checkpoint call order differs from configured leg; no pose correction')
        goal=points[index];events=[];start=self.clock();deadline=start+c['checkpoint_timeout_s'];after_ns=0
        def observe():
            self.check()
            w=self.io.observe(tag_id,after_ns,min(c['observe_s'],max(.01,deadline-self.clock())),self.check)
            if not w.get('camera_healthy', False):
                raise RuntimeError('camera stream unverified/faulted; blind continuation forbidden')
            return summarize(w,tag_id,c,after_ns)
        def move(name):
            self.check()
            if not motion_verified(c,name.startswith('strafe')):return None
            if not getattr(self.io,'motion_capture_verified',False):return None
            if self.clock()>=deadline:return None
            # 仅调用已有有限宏；异常（含超时/接管）不在本层吞掉。
            done=self.action(name)
            if done is None:return None # 本趟校准预算不足：放弃校准，保留原链。
            self.io.settle(c['settle_s'],self.check)
            return done['after_ns']
        obs=observe();pattern=c['search_pattern'];used=0
        if obs['status']=='no_tag' and goal['allow_turn'] and self.can_search(len(pattern)) and c.get('search_enabled',False) and c['action_directions_confirmed'] and all(c['pose_contract'][k] for k in ('axes_confirmed','scale_confirmed','capture_time_confirmed')):
            for name in pattern:
                stamp=move(name)
                if stamp is None:break
                used+=1;after_ns=stamp;events.append({'search':name})
                obs=observe()
                if obs['pose_usable']:break
                # 仅ID可靠但坐标未确认时继续完成名义平衡搜索，不能按该位姿任意转向。
            # 找到可信位姿会交给下面校正；未找到则已走完有限名义平衡搜索或预算到期。
        corrections=0;aligned=False;last_score=None;reason=obs.get('reason',obs['status'])
        while obs['pose_usable'] and self.clock()<deadline:
            eh=wrap(obs['heading_deg']-goal['tag_heading_deg']);ex=obs['x_m']-goal['lateral_m']
            heading_ok=abs(eh)<=c['deadband_heading_deg'];x_ok=not obs['position_usable'] or abs(ex)<=c['deadband_x_m']
            score=abs(eh)/c['deadband_heading_deg']+(abs(ex)/c['deadband_x_m'] if obs['position_usable'] else 0)
            events.append({'error_heading_deg':eh,'error_x_m':ex if obs['position_usable'] else None,'samples':obs['samples']})
            if heading_ok and x_ok:aligned=True;reason='within coarse deadbands';break
            if last_score is not None and score>=last_score*(1-c['minimum_improvement']):reason='correction not improving; resume original chain';break
            if abs(eh)>c['max_correctable_heading_deg'] or obs['position_usable'] and abs(ex)>c['max_correctable_x_m']:
                reason='outside coarse correction region; keep route direction, no arbitrary turn';break
            if not c.get('pose_correction_enabled',False) or not c['action_directions_confirmed']:reason='action direction not confirmed; ID/pose report only';break
            if corrections>=c['max_correction_actions']:reason='bounded correction exhausted; resume original chain';break
            if not heading_ok:
                if not goal['allow_turn']:reason='turn forbidden at checkpoint';break
                name=next(n for n,d in c['post_action_heading']['turn_delta_deg'].items() if d*eh<0)
            else:
                if not goal['allow_strafe']:reason='strafe forbidden at checkpoint';break
                side=obs['body_right_in_tag_right']
                if abs(side)<.5:reason='side step not aligned with lateral correction axis';break
                name='strafe_left_once' if ex*side>0 else 'strafe_right_once'
            stamp=move(name)
            if stamp is None:reason='navigation motion budget unavailable';break
            before_heading=obs['heading_deg']
            corrections+=1;last_score=score;after_ns=stamp;events.append({'correct':name});obs=observe()
            if name in ('left_turn','right_turn') and obs['pose_usable']:
                measured=wrap(obs['heading_deg']-before_heading)
                expected=c['post_action_heading']['turn_delta_deg'][name]
                events.append({'observed_turn_delta_deg':measured,'calibrated_delta_deg':expected})
                if measured*expected<=0 or abs(measured)>c['post_action_heading']['max_step_deg']:
                    self.mag.anchor=None
                    raise RuntimeError('visual correction wrong direction or exceeds calibrated small-step bound; no further motion')
        if not aligned and corrections==0 and 0<used<len(pattern):
            # 仅看见ID但轴/位姿不可信时，不留在搜索偏角；完成剩余名义平衡搜索。
            for name in pattern[used:]:
                stamp=move(name)
                if stamp is None:break
                used+=1;after_ns=stamp;events.append({'search_return':name})
            reason+='; nominal search return '+('completed' if used==len(pattern) else 'incomplete')
            obs=observe()
            if obs['pose_usable'] and abs(wrap(obs['heading_deg']-goal['tag_heading_deg']))<=c['deadband_heading_deg'] and (not obs['position_usable'] or abs(obs['x_m']-goal['lateral_m'])<=c['deadband_x_m']):
                aligned=True;reason='within coarse deadbands after nominal search return'
        if not aligned and obs['status']=='no_tag' and used==0 and goal['allow_turn'] and self.can_search(len(pattern)) and c.get('search_enabled',False) and c['action_directions_confirmed'] and all(c['pose_contract'][k] for k in ('axes_confirmed','scale_confirmed','capture_time_confirmed')):
            # 纠偏后才丢码，也给一次有限搜索机会；不重试失败的native动作。
            for name in pattern:
                stamp=move(name)
                if stamp is None:break
                used+=1;after_ns=stamp;events.append({'reacquire_after_correction':name});obs=observe()
            if obs['pose_usable'] and abs(wrap(obs['heading_deg']-goal['tag_heading_deg']))<=c['deadband_heading_deg'] and (not obs['position_usable'] or abs(obs['x_m']-goal['lateral_m'])<=c['deadband_x_m']):
                aligned=True;reason='within coarse deadbands after reacquisition'
            else:reason='reacquisition bounded/exhausted; resume original chain'
        magnetic={'status':'not_updated','reason':'no reliable visual alignment'}
        if aligned:
            # 死区内仍可能残余偏角：磁参考应对应真实视觉航向，不能强行当目标0/180。
            visual_route_heading=wrap(goal['route_heading_deg']+wrap(obs['heading_deg']-goal['tag_heading_deg']))
            if hasattr(self.io,'set_magnetic_load'):self.io.set_magnetic_load(self.load_kind)
            receipt=self.clock()
            samples=self.io.magnetic_samples(receipt,c['magnetic']['collect_s'],self.check) if c['magnetic']['enabled'] else []
            magnetic=self.mag.refresh(samples,visual_route_heading,self.clock(),receipt,self.load_kind,tag_id)
            if magnetic.get('status')!='updated':self.mag.anchor=None
            magnetic['target_route_heading_deg']=goal['route_heading_deg']
            magnetic['observed_route_heading_deg']=visual_route_heading
        self.cursor=index # 只推进已尝试的路线检查点；丢码不改变原动作链，不按其他ID跳路线。
        result={'status':'aligned' if aligned else 'continue_original_chain','expected_id':tag_id,'leg':direction,
                'scan':obs,'events':events,'reason':reason,'search_actions':used,'correction_actions':corrections,
                'alignment_scope':'heading_and_lateral' if obs['position_usable'] else 'heading_only_or_unverified',
                'magnetic':magnetic,'ordinary_loss_stops_mission':False,'physical_pose_confirmed':False}
        self.history.append(result);return result
