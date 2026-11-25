"""地面码完整坐标轴与磁航向缓存的纯计算；无ROS导入或硬件访问。"""
import math,statistics
from entry_camera import rotate,unit

def wrap(v):return (v+180.)%360.-180.
def dot(a,b):return sum(x*y for x,y in zip(a,b))
def cross(a,b):return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
def axis(name):
    if name not in ('+x','-x','+y','-y','+z','-z'):raise ValueError('invalid signed axis')
    out=[0.,0.,0.];out['xyz'.index(name[1])]=1. if name[0]=='+' else -1.;return out
def circular_median(values):
    a=math.atan2(sum(math.sin(math.radians(v)) for v in values),sum(math.cos(math.radians(v)) for v in values))
    center=math.degrees(a)
    return wrap(center+statistics.median(wrap(v-center) for v in values))
def mad(values,center,circular=False):return statistics.median(abs(wrap(v-center) if circular else v-center) for v in values)

def ground_pose(row,contract):
    """由码面内前向轴/完整旋转求航向；竖直法向仅用于地面一致性检查。"""
    br,bf,bu=[axis(contract[k]) for k in ('body_right','body_forward','body_up')]
    if any(abs(x-y)>1e-8 for x,y in zip(cross(br,bf),bu)):raise ValueError('body axes not right-handed')
    tf=axis(contract['tag_forward'])
    if contract['tag_forward'][1]=='z':raise ValueError('ground heading must use an in-plane Tag axis')
    q=unit([row['quaternion_raw'][k] for k in ('x','y','z','w')])
    forward=rotate(q,tf);up=rotate(q,[0.,0.,float(contract['tag_up_sign'])]);right=cross(forward,up)
    if dot(up,bu)<math.cos(math.radians(contract['max_ground_tilt_deg'])):raise ValueError('Tag plane not plausible ground')
    x,y=dot(bf,right),dot(bf,forward)
    if math.hypot(x,y)<.25:raise ValueError('body forward projection onto ground is degenerate')
    p=[row['position_raw_m'][k] for k in ('x','y','z')]
    if not all(math.isfinite(v) for v in p) or not .02<math.sqrt(dot(p,p))<contract['max_range_m']:raise ValueError('implausible Tag range')
    # 机器人在固定码坐标系的位置；正横向为码右方，正航向为顺时针。
    return {'x_m':-dot(p,right),'forward_m':-dot(p,forward),
            'heading_deg':wrap(math.degrees(math.atan2(x,y))),
            'body_forward_in_tag_forward':y,
            'body_right_in_tag_right':dot(br,right),
            'ground_up_dot':dot(up,bu),'stamp_ns':row['stamp_ns'],'id':row['id']}

def summarize(window,expected_id,cfg,after_ns=0):
    c=cfg['pose_contract'];now=window.get('now_ns',0);after_ns=max(after_ns,window.get('after_ns',0));seen=set();rows=[];all_ids=set();reasons=[]
    for row in window.get('observations',[]):
        try:
            stamp=row['stamp_ns'];key=(row['id'],stamp,row['frame_id'],row.get('source_topic'))
            if type(stamp) is not int or stamp<=after_ns or not 0<=now-stamp<=cfg['max_age_s']*1e9:continue
            if row.get('source_topic')!=cfg['topic']:continue
            if row.get('source_publisher') not in (None,c['publisher']):continue
            all_ids.add(row['id'])
            if row['id']!=expected_id:continue
            if row.get('headers_disagree') or row['frame_id'].lstrip('/')!=c['frame'].lstrip('/'):continue
            if key in seen:continue
            seen.add(key)
            rows.append(row)
        except (ValueError,KeyError,TypeError):continue
    result={'status':'no_tag','expected_id':expected_id,'observed_ids':sorted(all_ids),'observations':rows,
            'ids':[],'samples':len(rows),'pose_usable':False,'position_usable':False,
            'confidence_semantics':'ALVAR per-observation confidence not established; fresh-frame and geometry checks only'}
    if len(rows)<cfg['min_frames']:return result
    result.update(status='id_seen_pose_untrusted',ids=[expected_id])
    if cfg.get('relative_heading_reference',{}).get('enabled'):
        from relative_heading import summarize_relative
        if not c['capture_time_confirmed'] or not window.get('contract_match',False):
            result['reason']='relative observation source/time contract missing';return result
        return summarize_relative(rows,cfg,expected_id,result)
    if not c['axes_confirmed'] or not c['capture_time_confirmed'] or not window.get('contract_match',False):
        result['reason']='frame/axis/capture contract not confirmed; ID only, no pose-based action';return result
    samples=[]
    for row in rows:
        try:samples.append(ground_pose(row,c))
        except (ValueError,KeyError,TypeError) as exc:reasons.append(str(exc))
    if len(samples)<cfg['min_frames']:
        result.update(status='unreliable',reason='insufficient ground-compatible poses',reasons=reasons);return result
    h=circular_median([s['heading_deg'] for s in samples]);x=statistics.median(s['x_m'] for s in samples)
    hm=mad([s['heading_deg'] for s in samples],h,True);xm=mad([s['x_m'] for s in samples],x)
    kept=[s for s in samples if abs(wrap(s['heading_deg']-h))<=max(3*hm,cfg['outlier_heading_floor_deg'])
          and abs(s['x_m']-x)<=max(3*xm,cfg['outlier_x_floor_m'])]
    if len(kept)<cfg['min_frames'] or hm>cfg['max_heading_mad_deg'] or c['scale_confirmed'] and xm>cfg['max_x_mad_m']:
        result.update(status='unreliable',reason='wobble/ambiguous pose exceeds robust window');return result
    forwards=[s['forward_m'] for s in kept]
    forward=statistics.median(forwards);fm=mad(forwards,forward)
    result.update(status='usable',pose_usable=True,
                  position_usable=c['scale_confirmed'] and fm<=cfg['max_x_mad_m'],samples=len(kept),
                  heading_deg=circular_median([s['heading_deg'] for s in kept]),x_m=statistics.median(s['x_m'] for s in kept),
                  forward_m=forward,forward_mad_m=fm,latest_stamp_ns=max(s['stamp_ns'] for s in kept),
                  body_forward_in_tag_forward=statistics.median(s['body_forward_in_tag_forward'] for s in kept),
                  body_right_in_tag_right=statistics.median(s['body_right_in_tag_right'] for s in kept),
                  heading_mad_deg=hm,x_mad_m=xm,reason='fresh multi-frame ground-axis estimate')
    return result

def magnetic_field_norm(xyz, cfg, sample):
    cal=cfg.get('field_norm_calibration')
    if cal is None:return math.sqrt(sum(v*v for v in xyz))
    from magnetic_capture import calibration_id
    digest=calibration_id(cal)
    if digest!=cfg.get('capture_contract',{}).get('calibration_id') or sample.get('calibration_id')!=digest:
        raise ValueError('field magnitude calibration mismatch')
    if cal.get('yaml_version')!=1:raise ValueError('field norm requires pinned horizontal matrix')
    c=cal['matrix'];x,y=xyz[:2]
    norm=math.hypot(x*c[0][0]+y*c[0][1]+c[0][2],x*c[1][0]+y*c[1][1]+c[1][2])
    if not math.isfinite(norm) or norm<=0:raise ValueError('invalid calibrated horizontal field')
    return norm

MAGNETIC_INSUFFICIENT_SAMPLES = 'insufficient fresh magnetic receipt events'


class MagneticReference:
    """只有可靠视觉航向才刷新偏置；mag仅作航向，绝不作为位置/距离。"""
    def __init__(self,cfg):self.cfg=cfg;self.anchor=None
    def refresh(self,samples,route_heading_deg,now,after_receipt,load,tag_id):
        if not self.cfg['enabled']:return {'status':'disabled'}
        good=[];seen=set()
        for s in samples:
            try:
                if s.get('expected_load',load)!=load:continue
                t=s['received_monotonic'];h=float(s['heading_deg']);xyz=tuple(s['xyz'])
                # 实机uint16源的旧版计算可能给360；与0等价，不外推超界值。
                if not after_receipt<t<=now or now-t>self.cfg['max_age_s'] or not 0<=h<=360:continue
                h=h%360.
                if len(xyz)!=3 or not all(math.isfinite(v) and -32768<=v<=32767 for v in xyz) or not math.isfinite(sum(v*v for v in xyz)) or sum(v*v for v in xyz)<=0:continue
                # 仅去重接收事件；稳定XYZ允许重复，接收新鲜不等于传感采样新鲜。
                if t in seen:continue
                seen.add(t);good.append((h,magnetic_field_norm(xyz,self.cfg,s)))
            except (ValueError,KeyError,TypeError):continue
        if len(good)<self.cfg['min_samples']:return {'status':'skipped','reason':MAGNETIC_INSUFFICIENT_SAMPLES}
        h=circular_median([a for a,b in good])%360.;norm=statistics.median(b for a,b in good)
        if mad([a for a,b in good],h,True)>self.cfg['max_mad_deg']:return {'status':'rejected','reason':'magnetic heading unstable'}
        if max(abs(b-norm) for a,b in good)/norm>self.cfg['max_norm_fraction_change']:return {'status':'rejected','reason':'magnetic field magnitude unstable within window'}
        sign=self.cfg['heading_sign'];offset=None if sign is None else wrap(route_heading_deg-sign*h)
        old=self.anchor
        if old and old['load']==load:
            if abs(norm-old['norm'])/old['norm']>self.cfg['max_norm_fraction_change']:return {'status':'rejected','reason':'magnetic field magnitude jump'}
            if offset is not None and old['offset_deg'] is not None and abs(wrap(offset-old['offset_deg']))>self.cfg['max_offset_jump_deg']:return {'status':'rejected','reason':'magnetic/visual disagreement'}
            if offset is None and abs(abs(wrap(h-old['raw_heading_deg']))-abs(wrap(route_heading_deg-old['route_heading_deg'])))>self.cfg['max_offset_jump_deg']:return {'status':'rejected','reason':'unverified magnetic sign and unexpected heading jump'}
        self.anchor={'source':'reliable_tag_alignment' if tag_id is not None else 'sampling_probe','tag_id':tag_id,'load':load,'raw_heading_deg':h,'route_heading_deg':route_heading_deg,'offset_deg':offset,
                     'capture_verified':bool(good) and all(s.get('capture_source')=='verified_sensor_capture' and type(s.get('captured_monotonic')) in (int,float) and after_receipt < s['captured_monotonic'] <= now and now-s['captured_monotonic'] <= self.cfg['max_age_s'] for s in samples),
                     'norm':norm,'norm_basis':('calibrated_horizontal' if self.cfg.get('field_norm_calibration') else 'raw_xyz_legacy'),'raw_norm':statistics.median(math.sqrt(sum(v*v for v in s['xyz'])) for s in samples),'received_monotonic':now,'usable_for_heading':sign is not None,
                     'freshness':'ROS receipt time only; repeated XYZ allowed; sensor capture freshness unverified'}
        if self.anchor['capture_verified'] and all(s.get('capture_semantics')=='conservative_software_lower_bound' for s in samples):
            self.anchor['freshness']='DRDY-verified atomic acquisition; conservative software time lower bound, not a hardware timestamp'
            self.anchor['capture_session']=samples[-1]['session']
            self.anchor['calibration_id']=samples[-1].get('calibration_id','')
            self.anchor['load_epoch']=samples[-1].get('load_epoch')
        return dict(status='updated',reference=dict(self.anchor))
