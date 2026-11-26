"""Report image-y against a matching operator-taught reference."""
import json
import math
import statistics
from pathlib import Path

from entry_camera import finite, frame, rotate

REFERENCE = {'source':'user screenshot 2026-11-04, subsequently clarified as unloaded',
    'load':'empty','target_id':None,'diagnostic_only':True,'screenshot_size':[805,608],
    'active_image_bounds_xy_inclusive':[2,3,801,602],'active_image_size':[800,600],
    'near_code_center_v_px_approx':360,'manual_estimate_uncertainty_px':5,
    'near_code_vertical_extent_px_approx':[332,387], 'camera_native_resolution_verified':False,
    'bands_v_over_h':[.55,.65], 'bands_field_calibrated':False}

def classify(sample):
    out={'id':sample.get('id'), 'diagnostic_only':True, 'alignment_verdict':None,
         'load_reference':'empty_only_not_valid_for_single', 'center_x_target_required':False,
         'source':sample.get('source','unknown'), 'actual_corners_available':False,
         'reprojection_error_available':False, 'forward_backward_action_allowed':False}
    try:
        h=sample['height'];w=sample['width'];u,v=sample['tag_center_px']
        if type(h) is not int or type(w) is not int or min(w,h)<=0 or not all(type(x) in (int,float) and math.isfinite(x) for x in (u,v)) or not 0<=u<w or not 0<=v<h:
            raise ValueError('invalid frame dimensions or projected center')
        if sample.get('source')!='ALVAR center + exact-stamp TF + CameraInfo projection':
            raise ValueError('pixel source is not the verified projection pipeline')
        ratio=v/h
        out.update(status='diagnostic_projection', width=w,height=h,u_px=u,v_px=v,v_over_h=ratio,
                   band='above_proposed_band' if ratio<.55 else 'below_proposed_band' if ratio>.65 else 'inside_proposed_band',
                   proposed_band_px=[.55*h,.65*h], projection_verified=sample.get('projection_verified') is True)
    except (ValueError,KeyError,TypeError) as exc:
        out.update(status='unavailable', reason=str(exc))
    return out

def load_entrance_reference(project):
    path = Path(project) / 'config' / 'entrance_reference.json'
    try:
        reference = json.loads(path.read_text(encoding='utf-8'))
        camera = reference['camera']
        width, height = camera['size_px']
        u, v = reference['tag_center_px']
        fingerprint = camera['fingerprint']
        if (reference['schema'] == 'r3.entrance-pixel-reference.v1'
                and reference['operator_confirmed'] is True and reference['diagnostic_only'] is True
                and type(reference['target_id']) is int and reference['target_id'] >= 0
                and reference['load'] in ('single', 'double')
                and finite(reference['target_x_m']) and finite(reference['target_angle_deg'])
                and all(type(n) is int and n > 0 for n in (width, height))
                and finite(u) and finite(v) and 0 <= u < width and 0 <= v < height
                and isinstance(camera['topic'], str) and camera['topic']
                and isinstance(camera['frame'], str) and frame(camera['frame'])
                and isinstance(fingerprint, str) and len(fingerprint) == 64
                and all(c in '0123456789abcdef' for c in fingerprint)):
            return reference
    except (OSError, ValueError, KeyError, TypeError, OverflowError):
        return None
    return None


def _matches_reference(sample, reference, load_kind, image_topic):
    camera = reference['camera']
    return (load_kind == reference['load'] and image_topic == camera['topic']
            and type(sample.get('id')) is int and sample['id'] == reference['target_id']
            and frame(sample.get('image_frame', '')) == frame(camera['frame'])
            and [sample.get('width'), sample.get('height')] == camera['size_px']
            and sample.get('camera_fingerprint') == camera['fingerprint'])


def PixelDiagnostics_MeasureGround(sample, image_topic):
    """Measure range from the optical centre to a floor marker, projected onto its plane."""
    point = sample.get('camera_position_m')
    quat = sample.get('camera_quaternion')
    if (image_topic != '/usb_cam_chest/image_raw' or frame(sample.get('image_frame', '')) != 'chest_usb_cam'
            or type(sample.get('id')) is not int or not 1 <= sample['id'] <= 7
            or not isinstance(point, (list, tuple)) or len(point) != 3 or not all(map(finite, point))
            or point[2] <= 0 or not isinstance(quat, (list, tuple))):
        return {}
    try:
        normal = rotate(quat, [0., 0., 1.])
    except (TypeError, ValueError):
        return {}
    height = -sum(v*n for v,n in zip(point, normal))
    if normal[1] >= 0 or not 0. < height < 1.:
        return {}
    ground_range = math.sqrt(sum((v + height*n)**2 for v,n in zip(point, normal)))
    return {'ground_normal_optical': normal, 'camera_height_above_tag_plane_m': height,
            'camera_ground_range_m': ground_range,
            'camera_down_pitch_deg': math.degrees(math.asin(max(-1., min(1., -normal[2]))))}


def summarize_window(window, reference=None, load_kind=None, image_topic=None):
    result = {'status':'diagnostic_only','reference':dict(REFERENCE),
              'samples':[], 'diagnostic_reasons':window.get('diagnostic_reasons',[]),
              'image_header':window.get('image_header'), 'camera_info':window.get('camera_info'),
              'control_quality':'actual pixel corners and reprojection error are not supplied by this adapter',
              'head_camera_fallback':False,'body_commands':0}
    matching = []
    posture_unverified = False
    loaded = load_kind in ('single', 'double')
    for sample in window.get('pixel_samples', []):
        row = classify(sample)
        result['samples'].append(row)
        if row['status'] != 'diagnostic_projection':
            continue
        if loaded:
            # Changing hip pitch moves the same floor mark to a different image row.
            # Measure the floor plane in this camera frame without assuming a servo-to-camera gain.
            row.pop('band', None)
            row.pop('proposed_band_px', None)
            row['load_reference'] = 'carried_posture_not_taught'
            row.update(PixelDiagnostics_MeasureGround(sample, image_topic))
        if reference is None or not _matches_reference(sample, reference, load_kind, image_topic):
            continue
        taught = reference.get('ground_normal_optical')
        observed = row.get('ground_normal_optical')
        if (not isinstance(taught, (list, tuple)) or len(taught) != 3 or not all(map(finite, taught))
                or abs(sum(x*x for x in taught) - 1.) > .001 or not observed):
            posture_unverified = True
            continue
        norm = math.sqrt(sum(x*x for x in taught))
        if sum(a*b for a,b in zip(taught, observed)) / norm < math.cos(math.radians(1.)):
            posture_unverified = True
            continue
        target_y = reference['tag_center_px'][1]
        delta_y = row['v_px'] - target_y
        row.update(load_reference='operator_taught_' + load_kind, reference_v_px=target_y,
                   reference_v_over_h=target_y / row['height'], y_delta_px=delta_y,
                   y_delta_over_h=delta_y / row['height'])
        matching.append(row)
    if reference is not None:
        result['entrance_reference'] = reference
        result['entrance_reference_status'] = ('applied' if matching else
                                              'posture_unverified' if posture_unverified else 'no_matching_projection')
    if loaded and not matching:
        result['reference'] = None
    if matching:
        result.update(reference=reference, entrance_reference_sample_count=len(matching),
                      median_y_delta_px=statistics.median(row['y_delta_px'] for row in matching),
                      median_y_delta_over_h=statistics.median(row['y_delta_over_h'] for row in matching))
    return result
