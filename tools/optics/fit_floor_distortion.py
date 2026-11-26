"""Fit one multiplier of factory Brown distortion to straight checker lines in raw sensor pixels.

python fit_floor_distortion.py --camera factory.json --grids floor_grid_views.json --report fit.json
Input views contain name, an optional pose group, and a rectangular grid of [u, v] or null.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


SCALES = np.round(np.arange(0., 1.801, .025), 4)
MIN_SLOPE = .05


def project(xy, d):
    x, y = xy.T
    k1, k2, p1, p2, k3 = d
    rr = x*x + y*y
    radial = 1 + rr*(k1 + rr*(k2 + rr*k3))
    slope = 2*(k1 + rr*(2*k2 + 3*k3*rr))
    projected = np.column_stack((x*radial + 2*p1*x*y + p2*(rr + 2*x*x),
                                 y*radial + p1*(rr + 2*y*y) + 2*p2*x*y))
    cross = slope*x*y + 2*p1*x + 2*p2*y
    return projected, (radial + slope*x*x + 2*p1*y + 6*p2*x, cross,
                       radial + slope*y*y + 6*p1*y + 2*p2*x)


def principal_disk(d, sensor_radius):
    # Tangential distortion's symmetric Jacobian has operator norm <= 6*r*hypot(p1,p2).
    # Take the first positive root over all radii, guarding both radial eigenvalues.
    k1, k2, p1, p2, k3 = d
    limit = max(4., 2*sensor_radius)
    for a,b,c in ((k1,k2,k3), (3*k1,5*k2,7*k3)):
        roots = np.polynomial.polynomial.polyroots([1-MIN_SLOPE,-6*np.hypot(p1,p2),a,0,b,0,c])
        for root in roots:
            if abs(root.imag) < 1e-7 and 0 < root.real < limit:
                limit = float(root.real)
    guaranteed = limit*(1 + limit**2*(k1 + limit**2*(k2 + limit**2*k3))) - 3*np.hypot(p1,p2)*limit**2
    return limit, float(guaranteed)


def inverse(pixels, focal, center, d, limit):
    target = (pixels-center)/focal
    rays = target.copy()
    for _ in range(35):
        forward, (a,b,c) = project(rays, d)
        delta = forward-target
        det = a*c-b*b
        safe_det = np.where(det > 1e-8, det, 1.)
        dx, dy = delta.T
        step = np.column_stack(((c*dx-b*dy)/safe_det, (a*dy-b*dx)/safe_det))
        length = np.maximum(np.linalg.norm(step, axis=1), .15)
        rays -= step*(.15/length)[:,None]
        radius = np.linalg.norm(rays, axis=1)
        rays *= (limit/np.maximum(limit, radius))[:,None]
        if np.max(np.abs(step)) < 1e-10:
            break
    reprojection = np.linalg.norm((project(rays,d)[0]-target)*focal, axis=1)
    valid = (reprojection < .02) & (np.linalg.norm(rays,axis=1) < limit-1e-6)
    return rays, valid


def raw_line_errors(raw, rays, focal, center, d):
    # A straight line in ray space generally projects to a curve in the sensor.
    # Fit its orientation once, then find each point's nearest spot on that curve.
    mean = rays.mean(axis=0)
    axis = np.linalg.svd(rays-mean,full_matrices=False)[2][0]
    lengths = (rays-mean) @ axis
    for _ in range(4):
        along = mean + lengths[:,None]*axis
        predicted, (a,b,c) = project(along,d)
        offset = predicted*focal + center-raw
        tangent = np.column_stack((a*axis[0]+b*axis[1], b*axis[0]+c*axis[1]))*focal
        lengths -= np.sum(offset*tangent,axis=1)/np.sum(tangent*tangent,axis=1)
    predicted = project(mean+lengths[:,None]*axis,d)[0]*focal+center
    return np.linalg.norm(predicted-raw,axis=1)


def read_view(view, size, focal, center, factory_d, factory_limit):
    grid = view['grid']
    parts = []
    for row in grid:
        if len(row) != len(grid[0]):
            raise ValueError('Use null for missing corners in a rectangular grid')
        parts.append([[np.nan,np.nan] if xy is None else xy for xy in row])
    arr = np.array(parts, dtype=float)
    if arr.ndim != 3 or arr.shape[2] != 2:
        raise ValueError('Expected [row][column][x,y]')
    pixels = arr.reshape(-1,2)
    present = np.isfinite(pixels).all(axis=1)
    if np.isinf(pixels).any() or np.any(np.isnan(pixels).any(axis=1) != np.isnan(pixels).all(axis=1)):
        raise ValueError('Expected a finite [u,v] or null for each corner')
    if not present.any() or not np.all((pixels[present] >= 0) & (pixels[present] < size)):
        raise ValueError('All measured corners must use original sensor pixel coordinates')
    raw = pixels[present]
    _, factory_valid = inverse(raw, focal, center, factory_d, factory_limit)
    indices = np.full(present.shape,-1,dtype=int)
    indices[present] = np.arange(len(raw))
    grid_index = indices.reshape(arr.shape[:2])
    lines = []
    for family in (grid_index, grid_index.T):
        groups = []
        for line in family:
            used = line[line >= 0]
            used = used[factory_valid[used]]
            if len(used) >= 3:
                groups.append(used)
        if len(groups) < 2:
            raise ValueError(f"{view['name']}: fewer than two usable lines in one direction")
        lines.append(groups)
    used = np.unique(np.concatenate(lines[0]+lines[1]))
    return {'name':view['name'], 'group':str(view.get('group',view['name'])), 'raw':raw, 'lines':lines,
            'valid':factory_valid, 'used':used}


def score(view, focal, center, d, limit):
    raw = view['raw']
    rays, valid = inverse(raw, focal, center,d,limit)
    # The mask is fixed once using the factory model. Dropping even one such
    # point invalidates the candidate on this view, including peripheral points.
    if not np.all(valid[view['valid']]):
        return None
    families = []
    for family in view['lines']:
        values = [raw_line_errors(raw[idx],rays[idx],focal,center,d) for idx in family]
        families.append(np.concatenate(values))
    joined = np.concatenate(families)
    return {'rms_px':float(np.sqrt(np.mean([np.mean(values**2) for values in families]))),
            'p95_px':float(np.percentile(joined,95)),
            'row_rms_px':float(np.sqrt(np.mean(families[0]**2))),
            'column_rms_px':float(np.sqrt(np.mean(families[1]**2))),
            'invertible_corners':int(valid.sum())}


def best_fit(table, train):
    # Equal votes for independent poses, even if one pose contains multiple photos.
    groups = sorted(set(v['group'] for v in train))
    def cost(s):
        scores = []
        for g in groups:
            group = [table[s][v['name']] for v in train if v['group'] == g]
            if any(result is None for result in group):
                return np.inf
            scores.append(np.mean([r['rms_px']**2 for r in group]))
        return np.mean(scores)
    return min(table, key=lambda s:(cost(s),abs(s-1.)))


def camera_model(camera, data):
    size = np.array([camera['width'], camera['height']])
    if list(size) != data['size'] or camera['distortion_model'] != 'plumb_bob':
        raise ValueError('Size/model mismatch with the raw camera')
    k = np.array(camera['K'],dtype=float).reshape(3,3)
    d = np.array(camera['D'],dtype=float)
    if (d.shape != (5,) or not np.isfinite(d).all() or not np.isfinite(k).all()
            or not np.allclose(k[2],[0,0,1]) or k[0,1] != 0 or k[1,0] != 0 or min(k[0,0],k[1,1]) <= 0):
        raise ValueError('Raw K and five finite Brown coefficients required')
    focal, center = k.diagonal()[:2], k[:2,2]
    radius = np.linalg.norm(np.maximum(center,size-1-center)/focal)
    return size, focal, center, d, radius


def evaluate(camera, data):
    size, focal, center, d, radius = camera_model(camera, data)
    disks = {float(s):principal_disk(d*s,radius) for s in SCALES}
    views = [read_view(v,size,focal,center,d,disks[1.][0]) for v in data['views']]
    groups = sorted(set(v['group'] for v in views))
    if len(set(v['name'] for v in views)) != len(views) or len(groups) < 2:
        raise ValueError('Use distinct view names and two or more independent pose groups')
    table = {s:{v['name']:score(v,focal,center,d*s,disk[0]) for v in views} for s,disk in disks.items()}
    full = best_fit(table,views)
    validation = {}
    gains = []
    for group in groups:
        scale = best_fit(table,[v for v in views if v['group'] != group])
        group_gains = []
        for v in (v for v in views if v['group']==group):
            name = v['name']
            trial, base = table[scale][name], table[1.][name]
            gain = None if trial is None else base['rms_px']-trial['rms_px']
            group_gains.append(gain)
            points = v['raw'][v['used']]
            validation[name] = {'group':group, 'scale_fitted_excluding_group':scale,
                'measured_corners':len(v['raw']), 'factory_invertible_corners':int(v['valid'].sum()),
                'excluded_factory_xy_px':v['raw'][~v['valid']].tolist(),
                'corners_on_scored_lines':len(points),'scored_lines_row_col':[len(f) for f in v['lines']],
                'used_bounds_xy_px':[points.min(axis=0).tolist(),points.max(axis=0).tolist()],
                'factory':base,'zero_distortion':table[0.][name],'held_out':trial,'held_out_gain_px':gain,
                'in_sample_at_pooled_fit':table[full][name]}
        gains.append(None if None in group_gains else float(np.mean(group_gains)))
    def inverse_stats(s):
        uu,vv = np.meshgrid(np.linspace(0,size[0]-1,65),np.linspace(0,size[1]-1,49))
        pixels = np.column_stack((uu.ravel(),vv.ravel()))
        valid = inverse(pixels,focal,center,d*s,disks[s][0])[1]
        return {'valid_of_65x49':int(valid.sum()),'grid_total':len(valid),
                'principal_ray_limit_norm':disks[s][0],'guaranteed_raw_disk_norm':disks[s][1]}, valid
    factory_domain, old = inverse_stats(1.)
    candidate_domain, new = inverse_stats(full)
    lost = int(np.sum(old & ~new))
    # Explicit pixel margins prevent recommending imperceptible improvements
    # from screenshot resampling. A failing holdout remains visible in the report.
    repeatable = (None not in gains and min(gains) >= 0 and np.mean(gains) >= .15
                  and all(v['held_out']['p95_px'] <= v['factory']['p95_px'] + .15 for v in validation.values()))
    result = {'schema':'aelos.floor_plumb_scale.v1','field_use':'region observed on floor only',
              'method':'Fixed K, all five factory D multiplied by a single gain. RMS in raw sensor pixels, equal row/column and pose weights.',
              'fit_range_step':[0.,1.8,.025],'pooled_scale':full,
              'holdout_pose_groups':groups, 'views':validation,
              'mean_group_held_out_gain_px':None if None in gains else float(np.mean(gains)),
              'factory_inverse':factory_domain,'pooled_inverse':candidate_domain,'factory_grid_points_lost':lost,
              'reliable_on_observed_region':bool(repeatable),
              'candidate':None,
              'criteria':'Every held-out pose RMS must improve or tie, group-average gain >=0.15 px, each held-out p95 may rise <=0.15 px; preserve all factory-scored corners.',
              'limitations':'Grid straightness alone cannot identify focal length, principal point, metric range, absolute heading or full-field calibration. Out-of-coverage rays are unvalidated.'}
    if repeatable:
        result['candidate'] = {'K':camera['K'],'D':(d*full).tolist(),'factory_D':camera['D'],'scale':full,
            'width':int(size[0]),'height':int(size[1]),'distortion_model':'plumb_bob',
            'use':'opt-in trial within the observed floor region; actual marker pose validation required',
            'full_field_validated':False}
    return result


def frozen_test(camera, data, previous):
    """Measure new poses at the saved gain without a fit or selection on those poses."""
    if previous.get('schema') != 'aelos.floor_plumb_scale.v1':
        raise ValueError('The frozen report must come from the training/LOO mode')
    train_names = set(previous['views'])
    train_groups = set(previous['holdout_pose_groups'])
    size, focal, center, d, radius = camera_model(camera,data)
    gain = previous['pooled_scale']
    if not np.isfinite(gain) or not SCALES[0] <= gain <= SCALES[-1]:
        raise ValueError('Invalid frozen gain')
    limit, _ = principal_disk(d,radius)
    candidate_limit, _ = principal_disk(d*gain,radius)
    scores = {}
    for v in data['views']:
        name, group = v['name'], str(v.get('group',v['name']))
        if name in train_names or group in train_groups:
            continue
        if name in scores:
            raise ValueError(f'Duplicate independent view name {name}')
        view = read_view(v,size,focal,center,d,limit)
        base = score(view,focal,center,d,limit)
        trial = score(view,focal,center,d*gain,candidate_limit)
        frozen_valid = inverse(view['raw'],focal,center,d*gain,candidate_limit)[1]
        points = view['raw'][view['used']]
        scores[name] = {'group':group,'source_file':v.get('file'),'measured_corners':len(view['raw']),
            'factory_invertible_corners':int(view['valid'].sum()),
            'frozen_invertible_corners':int(frozen_valid.sum()),
            'excluded_factory_xy_px':view['raw'][~view['valid']].tolist(),
            'factory_corners_lost_at_frozen_scale_xy_px':view['raw'][view['valid'] & ~frozen_valid].tolist(),
            'corners_on_scored_lines':len(points),'scored_lines_row_col':[len(lines) for lines in view['lines']],
            'used_bounds_xy_px':[points.min(axis=0).tolist(),points.max(axis=0).tolist()],
            'factory':base,'at_frozen_scale':trial,
            'gain_px':base['rms_px']-trial['rms_px'] if trial is not None else None}
    if not scores:
        raise ValueError('No independent pose beyond the groups in the frozen report')
    return {'schema':'aelos.floor_plumb_frozen_validation.v1','fixed_scale':gain,
        'training_views':sorted(train_names),'tested_views':scores,
        'scope':'Frozen prediction on independent poses. No fitting, model selection or threshold tuning on these images.'}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for key in ('camera','grids','report'):
        p.add_argument('--'+key,required=True,type=Path)
    p.add_argument('--frozen-report',type=Path,help='Evaluate new pose groups at the exact gain in an earlier report')
    args=p.parse_args()
    camera = json.loads(args.camera.read_text(encoding='utf-8-sig'))
    grids = json.loads(args.grids.read_text(encoding='utf-8-sig'))
    hashes = {path.name:hashlib.sha256(path.read_bytes()).hexdigest() for path in (args.camera,args.grids)}
    if args.frozen_report:
        if args.report.resolve() == args.frozen_report.resolve():
            p.error('Write the independent test to a new report')
        previous = json.loads(args.frozen_report.read_text(encoding='utf-8-sig'))
        if previous['inputs_sha256'].get(args.camera.name) != hashes[args.camera.name]:
            p.error('Camera file differs from the frozen training report')
        hashes[args.frozen_report.name] = hashlib.sha256(args.frozen_report.read_bytes()).hexdigest()
        result = frozen_test(camera,grids,previous)
    else:
        result = evaluate(camera,grids)
    result['inputs_sha256'] = hashes
    args.report.parent.mkdir(parents=True,exist_ok=True)
    args.report.write_text(json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf-8')
    print(args.report, result.get('fixed_scale',result.get('pooled_scale')))


if __name__=='__main__':
    main()
