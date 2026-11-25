"""Pick up an isolated single or the identified horizontal floor pair."""
import math

from block_camera import BlockEvidenceExpired

class PickupFault(RuntimeError):
    pass


def validate(cfg):
    for key, lo, hi in (('max_adjustments',1,20),('confirmation_windows',1,5),('max_lost_windows',0,5)):
        if type(cfg.get(key)) is not int or not lo <= cfg[key] <= hi:
            raise ValueError('pickup '+key)
    for key, lo, hi in (('target_x',0,1),('x_tolerance',.01,.3),('target_y_min',0,1),('target_y_max',0,1),('max_target_jump',.01,.5)):
        if type(cfg.get(key)) not in (float,int) or not math.isfinite(cfg[key]) or not lo <= cfg[key] <= hi:
            raise ValueError('pickup '+key)
    if cfg['target_y_min'] >= cfg['target_y_max']:
        raise ValueError('pickup y band')
    if type(cfg.get('double_pickup_enabled')) is not bool:
        raise ValueError('double pickup flag')
    return cfg


class PickupController:
    def __init__(self, robot, cfg):
        self.robot=robot; self.cfg=validate(cfg); self.search_used=False
        self.attempted=False; self.last_stamp=None; self.adjustments=0

    def reset_after_delivery(self):
        if self.robot.load_kind!='empty':raise PickupFault('reset requires empty expected load')
        self.search_used=False; self.attempted=False; self.adjustments=0

    def observe(self, *, count_only=False):
        r=self.robot; r.check()
        if r.load_kind!='empty':raise PickupFault('observation for pickup requires empty load')
        row=r.observe_blocks(count_only=True) if count_only else r.observe_blocks()
        if row.get('camera_healthy') is not True:raise PickupFault('camera fault, not confirmed empty')
        stamp=row.get('stamp_ns')
        if type(stamp) is not int or stamp<=0 or self.last_stamp is not None and stamp<=self.last_stamp:
            raise PickupFault('duplicate/reordered pickup observation')
        self.last_stamp=stamp
        if row.get('status') not in ('target','empty','unknown'):raise PickupFault('invalid vision status')
        if row['status']=='empty' and row.get('count')!=0:raise PickupFault('empty count contract')
        if row['status']=='target':
            count = row.get('count')
            if type(count) is not int or count not in (1,2,3):
                raise PickupFault('target count contract')
            grip = min(count, 2) if count_only else row.get('grip_count', 1 if count == 1 else 0)
            if not (type(grip) is int and 0 <= grip <= min(count, 2)):
                raise PickupFault('grip count does not match visible layout')
            if grip == 0:
                row = dict(row, status='unknown', reason='ambiguous ground-row pickup layout')
            elif grip == 2 and not (self.cfg['double_pickup_enabled'] and r.supports_pair()):
                row = dict(row, status='unknown', reason='horizontal pair: complete double-load native capability unavailable',
                           reason_code='double_load_capability_unavailable',
                           reason_zh=(f'已识别 {count} 块，当前会话缺少 MCU 双抱能力回执。'
                                      '请检查双抱动作包是否已加载。'),
                           required_grip_count=2)
            elif count_only:
                row = dict(row, grip_count=grip)
            else:
                t = row.get('pickup_target') or (row.get('target') if count == 1 else None) or {}
                if any(type(t.get(k)) not in (int,float) or not math.isfinite(t[k]) or not 0<=t[k]<=1 for k in ('x','y')):
                    raise PickupFault('target coordinates invalid')
                row = dict(row, target=t, grip_count=grip)
        r.events.append({'block_observation':row})
        return row

    def _grab(self,row,*,stacked_confirmed=False):
        r=self.robot
        grip=row['grip_count']
        stacked_triple=(stacked_confirmed and grip==2 and row.get('count')==3 and row.get('stacked_triple') is True)
        op,load=('grab_double','double') if grip==2 else ('grab_single','single')
        try:
            r.verify_block_observation(row)
            r.check()
            self.attempted=True
            r.finite(op,observation=row)
        except BlockEvidenceExpired:
            self.attempted=False
            return {'status':'no_pickup','reason':'evidence_expired','blind_grab':False}
        result = {'status':load+'_pickup_protocol_completed','observed_count':row.get('count'),
                'grip_count':grip,'planned_count':3 if stacked_triple else grip,
                'stacked_triple':stacked_triple,'expected_load':load,'physical_grip_confirmed':False,
                'adjustments':self.adjustments}
        if row.get('staged_default_pair') is True:
            result.update(staged_default_pair=True, reason_zh='人工摆放区未确认方块数量，已执行默认双抱。')
        return result

    def Pickup_AcquireStaged(self, *, allow_stacked=False, default_pair=False):
        """The first target is placed at gripping distance by the operator."""
        r=self.robot
        if self.attempted or r.load_kind!='empty':
            raise PickupFault('duplicate/wrong-load pickup rejected')
        expected_grip=None
        stacked_confirmed=allow_stacked
        for _ in range(self.cfg['confirmation_windows']):
            row=self.observe(count_only=True)
            if (default_pair and row.get('staged_count_unresolved') is True and row['status']=='unknown'
                    and self.cfg['double_pickup_enabled'] and r.supports_pair()):
                return self._grab(dict(row, grip_count=2, staged_default_pair=True))
            if row['status']!='target':
                result = {'status':'no_pickup','reason':row.get('reason',row['status']),
                          'observed_count':row.get('count'),'blind_grab':False}
                if 'reason_code' in row:
                    result.update(reason_code=row['reason_code'], reason_zh=row.get('reason_zh'),
                                  required_grip_count=row.get('required_grip_count'))
                elif row.get('reason') == 'concave merged block layout without a clear bottom pair':
                    result.update(reason_code='staged_block_layout_ambiguous',
                                  reason_zh='看到了方块，但尚未确认底排是单块还是并排两块，正在复核。')
                return result
            if expected_grip is not None and row['grip_count']!=expected_grip:
                return {'status':'no_pickup','reason':'staged block count changed','blind_grab':False}
            expected_grip=row['grip_count']
            stacked_confirmed=stacked_confirmed and row['count']==3 and row.get('stacked_triple') is True
        return self._grab(row,stacked_confirmed=stacked_confirmed)

    def acquire(self, *, adjust=None, allow_stacked=False):
        r=self.robot; c=self.cfg
        if self.attempted or r.load_kind!='empty':raise PickupFault('duplicate/wrong-load pickup rejected')
        aligned=0; stable=0; lost=0; previous=None; stacked_windows=0
        row=self.observe()
        if row['status']!='target' and not self.search_used and adjust is None:
            self.search_used=True
            # Search is used once per delivery episode, not reset by repeated empty polling.
            for step in r.chains['pickup_search']:
                r.chain_step(step); row=self.observe()
                if row['status']=='target':break
        while True:
            r.check()
            if row['status']!='target':
                if previous is not None and lost<c['max_lost_windows']:
                    lost+=1; stable=0; aligned=0; stacked_windows=0; row=self.observe(); continue
                return {'status':'no_pickup','reason':row.get('reason',row['status']),'blind_grab':False,
                        'scene_empty':row['status']=='empty','reason_code':row.get('reason_code'),
                        'reason_zh':row.get('reason_zh'),'observed_count':row.get('count')}
            t=row['target']
            grip=row['grip_count']
            if previous is not None:
                last_target,last_grip=previous
                if (grip!=last_grip or t.get('color')!=last_target.get('color')
                        or abs(t['x']-last_target['x'])>c['max_target_jump']
                        or abs(t['y']-last_target['y'])>c['max_target_jump']):
                    return {'status':'no_pickup','reason':'target_changed','blind_grab':False}
            previous=t,grip
            dx=t['x']-c['target_x']
            stable+=1
            if abs(dx)<=c['x_tolerance'] and c['target_y_min']<=t['y']<=c['target_y_max']:
                aligned+=1
                if allow_stacked and row['count']==3 and row.get('stacked_triple') is True:
                    stacked_windows+=1
                if aligned>=c['confirmation_windows']:
                    return self._grab(row,stacked_confirmed=stacked_windows==aligned)
                row=self.observe(); continue
            aligned=0
            stacked_windows=0
            if self.adjustments>=c['max_adjustments']:
                return {'status':'no_pickup','reason':'adjustment_budget','blind_grab':False}
            if t['y']>c['target_y_max']:
                return {'status':'no_pickup','reason':'too_close','blind_grab':False}
            name='pickup_left' if dx < -c['x_tolerance'] else 'pickup_right' if dx > c['x_tolerance'] else 'pickup_advance'
            if adjust is None:
                r.run_chain(name)
            else:
                if stable<c['confirmation_windows']:
                    row=self.observe(); continue
                if not adjust(name,row):
                    return {'status':'no_pickup','reason':'outside_handoff_adjustment_range','blind_grab':False}
            stable=0
            self.adjustments+=1
            row=self.observe()
