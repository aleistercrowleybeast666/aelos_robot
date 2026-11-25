"""Tag→图像像素的纯数学部分；真实来源与人工验证分开，绝不猜/map轴。

原图支持CameraInfo plumb_bob五参数；rectified支持R/P。ROI/binning非默认
明确拒绝。无内参、无同时间TF或光学坐标未验证都不能驱动机器人。
"""
import hashlib,json,math
class CameraUnavailable(ValueError):pass
def finite(v):return type(v) in (float,int) and math.isfinite(v)
def frame(v):return str(v).lstrip('/')
def unit(q):
    if len(q)!=4 or not all(finite(v) for v in q):raise CameraUnavailable('invalid quaternion')
    n=math.sqrt(sum(v*v for v in q))
    if n<1e-9 or abs(n-1)>.1:raise CameraUnavailable('invalid quaternion norm')
    return [v/n for v in q]
def multiply(a,b):
    x,y,z,w=a;X,Y,Z,W=b
    return [w*X+x*W+y*Z-z*Y,w*Y-x*Z+y*W+z*X,w*Z+x*Y-y*X+z*W,w*W-x*X-y*Y-z*Z]
def rotate(q,p):
    q=unit(q);return multiply(multiply(q,list(p)+[0]),[-q[0],-q[1],-q[2],q[3]])[:3]
def transformed_pose(row,transform):
    source=frame(row['frame_id']);target=frame(transform['target_frame'])
    if frame(transform['source_frame'])!=source or transform['requested_stamp_ns']!=row['stamp_ns']:raise CameraUnavailable('TF frame/time does not match Tag')
    q=unit(transform['rotation']);translation=transform['translation'];p=row['position_raw_m']
    if len(translation)!=3 or not all(finite(v) for v in translation):raise CameraUnavailable('invalid TF translation')
    position=[a+b for a,b in zip(rotate(q,[p[k] for k in ('x','y','z')]),translation)]
    tagq=unit([row['quaternion_raw'][k] for k in ('x','y','z','w')]);orientation=unit(multiply(q,tagq))
    return target,position,orientation
def camera_fingerprint(info,image_kind):
    keys=('frame_id','width','height','K','D','R','P','distortion_model','binning_x','binning_y','roi')
    row={k:info.get(k) for k in keys};row['image_kind']=image_kind
    return hashlib.sha256(json.dumps(row,sort_keys=True,separators=(',',':')).encode()).hexdigest()
def validate_camera(info,image):
    w=info.get('width');h=info.get('height');k=info.get('K',[])
    if type(w) is not int or type(h) is not int or min(w,h)<=0 or (w,h)!=(image.get('width'),image.get('height')):raise CameraUnavailable('Image/CameraInfo resolution mismatch')
    if frame(info.get('frame_id',''))!=frame(image.get('frame_id','')) or not image.get('frame_id'):raise CameraUnavailable('Image/CameraInfo frame mismatch')
    if len(k)!=9 or not all(finite(v) for v in k) or k[0]<=0 or k[4]<=0 or abs(k[8]-1)>1e-9 or abs(k[6])+abs(k[7])>1e-9:raise CameraUnavailable('camera uncalibrated/invalid K')
    if info.get('binning_x',0) not in (0,1) or info.get('binning_y',0) not in (0,1):raise CameraUnavailable('binning not supported')
    roi=info.get('roi',{})
    if roi.get('x_offset',0) or roi.get('y_offset',0) or roi.get('width',0) not in (0,w) or roi.get('height',0) not in (0,h):raise CameraUnavailable('ROI scaling not supported')
    return w,h
def project(point,info,image,image_kind='raw'):
    w,h=validate_camera(info,image)
    if len(point)!=3 or not all(finite(v) for v in point) or point[2]<=0:raise CameraUnavailable('Tag behind camera or invalid camera point')
    X,Y,Z=point
    if image_kind=='raw':
        d=info.get('D',[])
        if info.get('distortion_model')!='plumb_bob' or len(d)!=5 or not all(finite(v) for v in d):raise CameraUnavailable('raw camera requires plumb_bob 5-coefficient model')
        x=X/Z;y=Y/Z;k1,k2,p1,p2,k3=d;r2=x*x+y*y;rad=1+k1*r2+k2*r2*r2+k3*r2*r2*r2
        xd=x*rad+2*p1*x*y+p2*(r2+2*x*x);yd=y*rad+p1*(r2+2*y*y)+2*p2*x*y
        k=info['K'];u=k[0]*xd+k[1]*yd+k[2];v=k[3]*xd+k[4]*yd+k[5]
    elif image_kind=='rectified':
        r=info.get('R',[]);p=info.get('P',[])
        if len(r)!=9 or len(p)!=12 or not all(finite(x) for x in r+p):raise CameraUnavailable('rectified image requires R/P')
        xyz=[sum(r[i*3+j]*point[j] for j in range(3)) for i in range(3)]+[1.]
        denominator=sum(p[8+j]*xyz[j] for j in range(4))
        if denominator<=0:raise CameraUnavailable('invalid rectified projection depth')
        u=sum(p[j]*xyz[j] for j in range(4))/denominator;v=sum(p[4+j]*xyz[j] for j in range(4))/denominator
    else:raise CameraUnavailable('unknown image kind')
    if not all(finite(x) for x in (u,v)) or not 0<=u<w or not 0<=v<h:raise CameraUnavailable('projected Tag outside image; no center verdict')
    return [u,v]
def heading_from_normal(q,axis):
    if axis not in ('x','y','z'):return None
    n=rotate(q,{'x':[1.,0,0],'y':[0.,1,0],'z':[0.,0,1]}[axis])
    if math.hypot(n[0],n[2])<1e-8:raise CameraUnavailable('Tag normal heading degenerate')
    return math.degrees(math.atan2(n[0],n[2]))
def pixel_observation(row,image,info,transform,cfg):
    if row.get('headers_disagree'):raise CameraUnavailable('conflicting AR headers; cannot select pose frame safely')
    if type(row.get('stamp_ns')) is not int or row['stamp_ns']<=0:raise CameraUnavailable('Tag timestamp missing')
    skew=cfg.get('max_sync_skew_ns');tolerance=0 if skew is None else skew
    if type(tolerance) is not int or tolerance<0:raise CameraUnavailable('invalid sync tolerance')
    if abs(image['stamp_ns']-row['stamp_ns'])>tolerance:raise CameraUnavailable('image and Tag capture timestamps do not match')
    if info.get('stamp_ns',0)==0:
        if not cfg.get('static_camera_info_verified'):raise CameraUnavailable('timeless CameraInfo not yet verified')
    elif abs(info['stamp_ns']-image['stamp_ns'])>tolerance:raise CameraUnavailable('CameraInfo timestamp mismatch')
    target,point,orientation=transformed_pose(row,transform)
    if target!=frame(image['frame_id']):raise CameraUnavailable('TF did not target image optical frame')
    if cfg.get('optical_frame') is not None and frame(cfg['optical_frame'])!=target:raise CameraUnavailable('configured optical frame mismatch')
    uv=project(point,info,image,cfg['image_kind']);fingerprint=camera_fingerprint(info,cfg['image_kind'])
    verified=all(cfg.get(k) is True for k in ('optical_axes_verified','camera_model_verified','pose_capture_time_verified','projection_overlay_verified'))
    return {'id':row['id'],'stamp_ns':row['stamp_ns'],'source_seq':row.get('source_seq'),'source_pose_frame':row['frame_id'],
        'image_frame':image['frame_id'],'image_stamp_ns':image['stamp_ns'],'width':image['width'],'height':image['height'],
        'tag_center_px':uv,'center_error_x_px':uv[0]-(image['width']-1)/2,'y_center_required':False,
        'camera_position_m':point,'camera_quaternion':orientation,'heading_normal_deg':heading_from_normal(orientation,cfg.get('tag_normal_axis')),
        'camera_fingerprint':fingerprint,'projection_verified':verified,'heading_metric_verified':verified and cfg.get('tag_normal_axis_verified') is True,
        'source':'ALVAR center + exact-stamp TF + CameraInfo projection','raw_observation':row,
        'center_verdict':'unverified_projection_for_manual_check' if not verified else 'verified_projection_metrics_only'}
