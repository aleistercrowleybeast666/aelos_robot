"""Reference-relative heading only; no positional calibration or hardware calls."""
import math, statistics

def normalized(q):
    if len(q)!=4 or any(type(x) not in (float,int) or not math.isfinite(x) for x in q):raise ValueError('invalid quaternion')
    n=math.sqrt(sum(x*x for x in q))
    if not .5<n<1.5:raise ValueError('quaternion norm')
    return [x/n for x in q]

def multiply(a,b):
    x,y,z,w=a;X,Y,Z,W=b
    return [w*X+x*W+y*Z-z*Y,w*Y-x*Z+y*W+z*X,w*Z+x*Y-y*X+z*W,w*W-x*X-y*Y-z*Z]

def relative_angles(q,reference):
    q=normalized(q);r=normalized(reference)
    x,y,z,w=normalized(multiply(q,[-r[0],-r[1],-r[2],r[3]]))
    # Twist about empirically checked map vertical; swing retains pitch/roll.
    n=math.hypot(z,w)
    if n<.25:raise ValueError('near singular relative heading')
    heading=(math.degrees(2*math.atan2(z,w))+180)%360-180
    tilt=math.degrees(2*math.acos(min(1,max(0,n))))
    return heading,tilt

def reference_valid(r,expected_id):
    if not isinstance(r,dict) or r.get('enabled') is not True:return False
    if r.get('orientation_confirmed') is not True or r.get('tag_id')!=expected_id:return False
    if r.get('direction_evidence')!='operator_left_with_magnetic_agreement':return False
    if r.get('position_reference_confirmed') is not False:return False
    if not 0<r.get('max_relative_tilt_deg',0)<=15:return False
    normalized(r.get('quaternion_xyzw',[]));return True

def summarize_relative(rows,cfg,expected_id,result):
    r=cfg['relative_heading_reference'];out=dict(result)
    if not reference_valid(r,expected_id):out['reason']='relative reference unconfirmed';return out
    values=[];tilts=[]
    for row in rows:
        try:
            if row['frame_id'].lstrip('/')!=r['frame_id'].lstrip('/'):continue
            h,t=relative_angles([row['quaternion_raw'][k] for k in ('x','y','z','w')],r['quaternion_xyzw'])
            if t<=r['max_relative_tilt_deg']:values.append((h,row['stamp_ns']));tilts.append(t)
        except (KeyError,ValueError,TypeError):continue
    if len(values)<cfg['min_frames']:out['reason']='insufficient tilt-compatible relative poses';return out
    pivot=values[0][0];hs=[pivot+(h-pivot+180)%360-180 for h,_ in values];center=statistics.median(hs)
    dispersion=statistics.median(abs(h-center) for h in hs)
    kept=[(h,stamp) for h,stamp in values if abs((h-center+180)%360-180)<=max(3*dispersion,cfg['outlier_heading_floor_deg'])]
    if len(kept)<cfg['min_frames'] or dispersion>cfg['visual_micro']['max_heading_mad_deg']:
        out['reason']='relative heading wobble exceeds stable window';return out
    out.update(status='usable',pose_usable=True,heading_only=True,position_usable=False,heading_deg=(center+180)%360-180,
        heading_mad_deg=dispersion,relative_tilt_deg=statistics.median(tilts),samples=len(kept),latest_stamp_ns=max(t for _,t in kept),
        x_m=None,forward_m=None,reference_load=r.get('reference_load','empty'),carried_position_verified=False,
        reason='operator-approved relative orientation only; no x/y target or measured carry geometry')
    return out
