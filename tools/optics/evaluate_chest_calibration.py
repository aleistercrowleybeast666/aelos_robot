"""Fit and independently compare chest camera lens models from raw chessboard photos."""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


MODELS = ('plumb_bob', 'equidistant')
REGIONS = ('center', 'left', 'right', 'top', 'bottom', 'top_left', 'top_right',
           'bottom_left', 'bottom_right')


def _find_grid(image, pattern, channel, vertical_scale=1., search_rect=None):
    """Locate a complete inner grid; return points in the original image pixel coordinates."""
    x, y, w, h = search_rect or (0, 0, image.shape[1], image.shape[0])
    if min(x,y) < 0 or min(w,h) < 1 or x+w > image.shape[1] or y+h > image.shape[0]:
        raise ValueError('棋盘搜索范围越过原图边界')
    crop = image[y:y+h, x:x+w]
    if channel == 'yellow':
        # A shadow can make the white tile darker than yellow. Lab b encodes the colour difference.
        b = cv2.cvtColor(crop, cv2.COLOR_BGR2LAB)[:, :, 2]
        bounds = [(10, 90), (5, 95), (15, 85), (0, 100)]
    else:
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        bounds = [None]
    target_height = round(h*vertical_scale)
    flags = cv2.CALIB_CB_NORMALIZE_IMAGE
    if channel == 'yellow':
        flags |= cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    for limits in bounds:
        if limits is not None:
            low, high = np.percentile(b, limits)
            gray = np.uint8(np.clip((b.astype(float)-low)*255/max(1., high-low), 0, 255))
        sample = cv2.resize(gray, (w, target_height), interpolation=cv2.INTER_LINEAR) if target_height != h else gray
        ok, points = cv2.findChessboardCornersSB(sample, pattern, flags)
        if ok:
            grid = points.reshape(pattern[1], pattern[0], 2).astype(np.float64)
            grid[...,0] += x
            grid[...,1] = (grid[...,1]+.5)*(h/target_height)-.5+y
            if _grid_is_contiguous(grid):
                return grid.reshape(-1, 2)
    return None


def _grid_is_contiguous(grid):
    # Adjacent checker intersections preserve their local order under perspective.
    # Reject skipped tiles and folded/background false grids from repetitive colour patterns.
    dc, dr = np.diff(grid, axis=1), np.diff(grid, axis=0)
    col_lengths, row_lengths = np.linalg.norm(dc, axis=2), np.linalg.norm(dr, axis=2)
    def consistent(a, axis):
        lhs, rhs = np.take(a, range(a.shape[axis]-1), axis), np.take(a, range(1,a.shape[axis]), axis)
        return np.all(np.minimum(lhs,rhs) >= .56*np.maximum(lhs,rhs))
    orientation = dc[:-1,:,0]*dr[:,:-1,1]-dc[:-1,:,1]*dr[:,:-1,0]
    return (min(col_lengths.min(), row_lengths.min()) >= 3.5 and consistent(col_lengths,1)
            and consistent(row_lengths,0) and np.all(orientation*orientation.flat[0] > 0))


def _read_views(folder, pattern, expected_size=None, *, channel='gray', vertical_scale=1.):
    views, rejected = [], []
    for path in sorted(folder.rglob('*')):
        if not path.is_file() or path.suffix.lower() not in ('.png', '.jpg', '.jpeg', '.bmp', '.tif', '.tiff'):
            continue
        name = str(path.relative_to(folder))
        data = np.fromfile(path, dtype=np.uint8)
        image = cv2.imdecode(data, cv2.IMREAD_COLOR) if data.size else None
        if image is None:
            rejected.append({'file': name, 'reason': 'unreadable'})
            continue
        size = (image.shape[1], image.shape[0])
        if expected_size is not None and size != expected_size:
            raise ValueError(f'{path} 尺寸为 {size}，与同组原图 {expected_size} 不符')
        points = _find_grid(image, pattern, channel, vertical_scale)
        if points is None:
            rejected.append({'file': name, 'reason': 'complete_inner_grid_not_detected'})
            continue
        expected_size = size
        views.append((name, points.reshape(-1, 2).astype(np.float64)))
    return views, rejected, expected_size


def _keep_distinct(views, previous, pattern, threshold):
    kept, rejected = [], []
    cols, rows = pattern
    for name, points in views:
        grid = points.reshape(rows, cols, 2)
        # Chessboard indexing can reverse between photographs of the same stance.
        grids = [grid, grid[::-1], grid[:, ::-1], grid[::-1, ::-1]]
        if cols == rows:
            grids.extend([g.transpose(1, 0, 2) for g in grids[:]])
        match = None
        for other_name, other in previous + kept:
            rms = min(float(np.sqrt(np.mean(np.sum((g.reshape(-1, 2)-other)**2, axis=1)))) for g in grids)
            if rms <= threshold:
                match = {'file': name, 'reason': 'near_duplicate_pose', 'matches': other_name, 'rms_px': rms}
                break
        if match:
            rejected.append(match)
        else:
            kept.append((name, points))
    return kept, rejected


def _fit(model, grid, train, size):
    xyz = grid.reshape(-1, 1, 3)
    pixels = [points.reshape(-1, 1, 2) for _, points in train]
    if model == 'plumb_bob':
        rms, k, d, _, _ = cv2.calibrateCamera(
            [xyz.astype(np.float32)] * len(train), [p.astype(np.float32) for p in pixels], size, None, None)
    else:
        w, h = size
        guess = np.array([[w/2, 0., (w-1)/2], [0., w/2, (h-1)/2], [0., 0., 1.]])
        flags = (cv2.fisheye.CALIB_USE_INTRINSIC_GUESS | cv2.fisheye.CALIB_FIX_SKEW
                 | cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_CHECK_COND)
        rms, k, d, _, _ = cv2.fisheye.calibrate([xyz] * len(train), pixels, size, guess, np.zeros((4, 1)),
            flags=flags, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 100, 1e-9))
    if not np.isfinite(rms) or not np.isfinite(k).all() or not np.isfinite(d).all() or min(k[0, 0], k[1, 1]) <= 0:
        raise ValueError('内参估计无效')
    return float(rms), k, d


def _rays(model, pixels, k, d):
    p = np.ascontiguousarray(pixels, dtype=np.float64).reshape(-1, 1, 2)
    if model == 'equidistant':
        out = cv2.fisheye.undistortPoints(p, k, d)
    else:
        criteria = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 60, 1e-10)
        if hasattr(cv2, 'undistortPointsIter'):
            out = cv2.undistortPointsIter(p, k, d, None, None, criteria)
        else:
            out = cv2.undistortPoints(p, k, d, criteria=criteria)
    return out.reshape(-1, 2)


def _project(model, xyz, rvec, tvec, k, d):
    xyz = np.ascontiguousarray(xyz, dtype=np.float64).reshape(-1, 1, 3)
    project = cv2.fisheye.projectPoints if model == 'equidistant' else cv2.projectPoints
    return project(xyz, rvec, tvec, k, d)[0].reshape(-1, 2)


def _regions(pixels, size):
    w, h = size
    result = []
    for x, y in pixels:
        horizontal = 'left' if x < .2*w else ('right' if x > .8*w else '')
        vertical = 'top' if y < .2*h else ('bottom' if y > .8*h else '')
        result.append('_'.join(filter(None, (vertical, horizontal))) or 'center')
    return np.array(result)


def _validation_errors(model, grid, split, views, k, d):
    measurements, failed = [], []
    for name, points in views:
        try:
            rays = _rays(model, points[split], k, d)
            ok, rot, pos = cv2.solvePnP(grid[split], rays, np.eye(3), None, flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok or not np.isfinite(pos).all() or np.min((grid @ cv2.Rodrigues(rot)[0].T)[:, 2]+pos[2, 0]) <= 0:
                raise ValueError('无法解出板面位姿')
            tested = points[~split]
            predicted = _project(model, grid[~split], rot, pos, k, d)
            errors = np.linalg.norm(predicted-tested, axis=1)
            if not np.isfinite(errors).all():
                raise ValueError('投影无效')
            measurements.append((name, tested, errors))
        except (cv2.error, ValueError) as exc:
            failed.append({'file': name, 'reason': str(exc)})
    return measurements, failed


def _region_scores(train, measurements, size):
    train_regions = [_regions(points, size) for _, points in train]
    test_regions = [(_regions(points, size), errors) for _, points, errors in measurements]
    result = {}
    for region in REGIONS:
        sampled = [errors[tags == region] for tags, errors in test_regions if np.any(tags == region)]
        values = np.concatenate(sampled) if sampled else np.empty(0)
        result[region] = {'train_views': sum(bool(np.any(tags == region)) for tags in train_regions),
                          'holdout_views': len(sampled), 'holdout_points': len(values),
                          'rms_px': float(np.sqrt(np.mean(values**2))) if len(values) else None,
                          'p95_px': float(np.percentile(values, 95)) if len(values) else None}
    return result


def _coverage(points, size):
    if not points:
        return {'hull_fraction': 0., 'nearest_corner_fraction': {}}
    xy = np.vstack(points) / (np.asarray(size)-1.)
    hull = cv2.convexHull(xy.astype(np.float32))
    corners = {'top_left': (0,0), 'top_right': (1,0), 'bottom_left': (0,1), 'bottom_right': (1,1)}
    return {'hull_fraction': cv2.contourArea(hull),
            'nearest_corner_fraction': {key: float(np.min(np.max(np.abs(xy-corner), axis=1)))
                                        for key, corner in corners.items()}}


def _check_inverse(model, k, d, size):
    # Inverting a lens polynomial past its monotone branch can appear to succeed at the image corners.
    x, y = np.meshgrid(np.linspace(0, size[0]-1, 33), np.linspace(0, size[1]-1, 25))
    pixels = np.c_[x.ravel(), y.ravel()]
    rays = _rays(model, pixels, k, d)
    xyz = np.c_[rays, np.ones(len(rays))]
    zero = np.zeros(3)
    predicted = _project(model, xyz, zero, zero, k, d)
    residual = np.linalg.norm(predicted-pixels, axis=1)
    # Check local orientation of the inverse lattice, including both triangles of each cell.
    lattice = rays.reshape(*x.shape, 2)
    a, b, c, e = lattice[:-1, :-1], lattice[:-1, 1:], lattice[1:, :-1], lattice[1:, 1:]
    def positive(u, v):
        return u[..., 0]*v[..., 1]-u[..., 1]*v[..., 0] > 0
    oriented = bool(np.all(positive(b-a, c-a)) and np.all(positive(e-c, e-b)))
    worst = float(np.max(residual)) if np.isfinite(residual).all() else None
    return {'grid': [33, 25], 'max_roundtrip_px': worst, 'orientation_preserved': oriented,
            'passed': worst is not None and worst <= .25 and oriented}


def _evaluate(model, train, holdout, grid, pattern, size, max_p95):
    rms, k, d = _fit(model, grid, train, size)
    split = (np.indices((pattern[1], pattern[0])).sum(axis=0).ravel() % 2 == 0)
    measurements, failed = _validation_errors(model, grid, split, holdout, k, d)
    scores = _region_scores(train, measurements, size)
    coverage = {'train': _coverage([points for _,points in train], size),
                'holdout': _coverage([points for _,points,_ in measurements], size)}
    inverse = _check_inverse(model, k, d, size)
    missing = [region for region, v in scores.items() if v['train_views'] < 2 or
               v['holdout_views'] < 2 or v['holdout_points'] < 8]
    corner_support = all(len(c['nearest_corner_fraction']) == 4 and
                         max(c['nearest_corner_fraction'].values()) <= .10 for c in coverage.values())
    enough = len(train) >= 12 and len(holdout) >= 6 and not missing and corner_support
    quality = not failed and all(v['p95_px'] is not None and v['p95_px'] <= max_p95 for v in scores.values())
    status = ('insufficient_coverage' if not enough else
              ('candidate_passed_holdout' if quality and inverse['passed'] else 'validation_failed'))
    validation = {'status': status, 'training_rms_px': rms, 'regions': scores, 'undercovered': missing,
                  'coverage': coverage, 'failed_holdout_views': failed, 'inverse': inverse}
    p = np.c_[k, np.zeros(3)]
    return {'K': k.ravel().tolist(), 'D': d.ravel().tolist(), 'R': np.eye(3).ravel().tolist(),
            'P': p.ravel().tolist(), 'validation': validation}


def _save(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--train', type=Path, required=True, help='训练集原始照片目录')
    parser.add_argument('--holdout', type=Path, required=True, help='独立取景的验证集目录')
    parser.add_argument('--pattern', required=True, help='棋盘内角点列数x行数，例如 9x6')
    geometry = parser.add_mutually_exclusive_group(required=True)
    geometry.add_argument('--square-mm', type=float, help='实测正方形单格边长，单位毫米')
    geometry.add_argument('--cell-mm', type=float, nargs=2, metavar=('COLUMN','ROW'), help='沿棋盘列、行方向的格宽、格高，单位毫米')
    parser.add_argument('--channel', choices=('gray','yellow'), default='gray', help='黄白格使用 yellow')
    parser.add_argument('--vertical-scale', type=float, default=1., help='低视角时仅沿图像竖直方向放大检测，输出返回原坐标')
    parser.add_argument('--out', type=Path, required=True, help='模型和验证报告 JSON 的输出目录')
    parser.add_argument('--frame-id', default='chest_usb_cam')
    parser.add_argument('--max-p95-px', type=float, default=1.5, help='每一分区验证误差的 P95 目标，单位像素')
    args = parser.parse_args()
    try:
        cols, rows = (int(n) for n in args.pattern.lower().split('x'))
    except ValueError:
        parser.error('--pattern 格式应为内角点列数x行数')
    cell = args.cell_mm or [args.square_mm, args.square_mm]
    if (min(cols, rows) < 3 or cols*rows < 12 or not 1 <= args.vertical_scale <= 5 or
            not all(np.isfinite(v) and v > 0 for v in (*cell, args.max_p95_px))):
        parser.error('棋盘每方向至少三个内角点，总数至少十二；格长和误差目标须为正数，检测放大倍率为 1 至 5')
    train_dir, test_dir = args.train.resolve(), args.holdout.resolve()
    if not train_dir.is_dir() or not test_dir.is_dir() or (train_dir == test_dir or
            train_dir in test_dir.parents or test_dir in train_dir.parents):
        parser.error('请指定两个独立且互不包含的照片目录')
    pattern = cols, rows
    train, bad_train, size = _read_views(train_dir, pattern, channel=args.channel, vertical_scale=args.vertical_scale)
    if size is None:
        parser.error('训练照片未检出指定的完整棋盘')
    holdout, bad_test, _ = _read_views(test_dir, pattern, size, channel=args.channel, vertical_scale=args.vertical_scale)
    # One per cent of the short sensor edge is about 4.8 pixels on the R3.
    cutoff = .01*min(size)
    train, dupe = _keep_distinct(train, [], pattern, cutoff)
    holdout, dupe_test = _keep_distinct(holdout, [('train/'+n, p) for n,p in train], pattern, cutoff)
    if len(train) < 4:
        parser.error('独立训练视图少于四张，无法稳定拟合两个模型')
    xy = np.indices((rows, cols))[::-1].transpose(1, 2, 0).reshape(-1, 2)
    grid = np.c_[xy*cell, np.zeros(len(xy))]
    args.out.mkdir(parents=True, exist_ok=True)
    report = {'pattern_inner_corners': list(pattern), 'square_mm': args.square_mm, 'cell_mm': cell,
              'detection_channel': args.channel, 'vertical_scale': args.vertical_scale, 'frame_id': args.frame_id,
              'width': size[0], 'height': size[1], 'training': [n for n,_ in train],
              'holdout': [n for n,_ in holdout], 'rejected_train': bad_train+dupe, 'rejected_holdout': bad_test+dupe_test,
              'criteria': {'min_train_views': 12, 'min_holdout_views': 6, 'per_region_min_views_per_set': 2,
                           'per_region_min_holdout_points': 8, 'edge_fraction': .2, 'max_p95_px': args.max_p95_px,
                           'max_nearest_corner_fraction': .10, 'near_duplicate_rms_px': cutoff,
                           'max_roundtrip_px': .25},
              'models': {}}
    for model in MODELS:
        try:
            result = _evaluate(model, train, holdout, grid, pattern, size, args.max_p95_px)
        except (cv2.error, ValueError, FloatingPointError) as exc:
            result = {'validation': {'status': 'fit_failed', 'reason': str(exc)}}
        result.update(width=size[0], height=size[1], distortion_model=model,
                      frame_id=args.frame_id, square_mm=args.square_mm, cell_mm=cell)
        _save(args.out/(model+'.json'), result)
        report['models'][model] = result['validation']
        print(model, result['validation']['status'])
    _save(args.out/'validation.json', report)
    return int(all(v['status'] == 'fit_failed' for v in report['models'].values()))


if __name__ == '__main__':
    raise SystemExit(main())
