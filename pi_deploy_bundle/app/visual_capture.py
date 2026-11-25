"""Actual CameraInfo evidence only. No corners, reprojection error or exposure claims."""
import math
from entry_camera import validate_camera, camera_fingerprint, CameraUnavailable
from r3_scan import header_info


def decode_info(msg, receipt):
    h=header_info(msg) or {}
    roi=getattr(msg,'roi',None)
    return dict(frame_id=h.get('frame_id',''),stamp_ns=h.get('stamp_ns',0),
        width=getattr(msg,'width',0),height=getattr(msg,'height',0),
        K=list(getattr(msg,'K',[])),D=list(getattr(msg,'D',[])),
        R=list(getattr(msg,'R',[])),P=list(getattr(msg,'P',[])),
        distortion_model=getattr(msg,'distortion_model',''),
        binning_x=getattr(msg,'binning_x',0),binning_y=getattr(msg,'binning_y',0),
        roi={k:getattr(roi,k,0) for k in ('x_offset','y_offset','width','height')},
        publisher=(getattr(msg,'_connection_header',None) or {}).get('callerid',''),received_monotonic=receipt)


def camera_status(info,image,c,now):
    out=dict(ready=False,reason='CameraInfo or image metadata missing',fingerprint=None,can_reobserve=False,
        detector_corners_available=False,reprojection_residual_available=False,exposure_time_verified=False)
    if not info or not image:return dict(out,can_reobserve=True)
    try:
        validate_camera(info,image)
        if info.get('distortion_model')!='plumb_bob' or len(info.get('D',[]))!=5 or not all(type(v) in (int,float) and math.isfinite(v) for v in info['D']):
            raise CameraUnavailable('only actual raw plumb_bob five-coefficient calibration supported')
        digest=camera_fingerprint(info,'raw');out['fingerprint']=digest
        out.update(width=info['width'],height=info['height'],frame_id=info['frame_id'],publisher=info['publisher'])
        if (not c['source_confirmed'] or info['publisher']!=c['publisher'] or image.get('publisher')!=c['publisher'] or
                info['frame_id'].lstrip('/')!=str(c['frame_id']).lstrip('/') or
                digest!=c['expected_fingerprint']):
            raise CameraUnavailable('CameraInfo source/frame/fingerprint not explicitly verified')
        image_age=now-image.get('received_monotonic',float('-inf'))
        info_age=now-info['received_monotonic']
        if image_age<0 or info_age<0 or math.isnan(image_age) or math.isnan(info_age):
            raise CameraUnavailable('CameraInfo or image receipt time not in the local clock domain')
        if image_age>c['max_age_s']:
            return dict(out,reason='actual image metadata stale',can_reobserve=True)
        if info_age>c['max_age_s']:
            return dict(out,reason='CameraInfo receipt stale',can_reobserve=True)
        if info['stamp_ns']==0:
            if not c['static_camera_info_verified']:raise CameraUnavailable('timeless CameraInfo not explicitly verified')
        elif abs(info['stamp_ns']-image['stamp_ns'])>c['max_sync_skew_s']*1e9:
            return dict(out,reason='CameraInfo and actual image timestamp mismatch',can_reobserve=True)
        out.update(ready=True,reason='actual CameraInfo matches verified source, frame and calibration')
    except (CameraUnavailable,KeyError,TypeError,ValueError) as exc:
        out['reason']=str(exc)
    return out
