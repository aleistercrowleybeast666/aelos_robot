"""Measure how straight a flat tile grid appears before and after an existing lens correction."""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from evaluate_chest_calibration import MODELS, _find_grid, _project, _rays, _save


def _target(camera):
    ci = camera.get('ci', camera)
    model = ci['distortion_model']
    size = (ci.get('width', ci.get('w')), ci.get('height', ci.get('h')))
    if model not in MODELS or not all(type(v) is int and v > 0 for v in size):
        raise ValueError('相机模型或分辨率无效')
    return model, size, np.asarray(ci['K'], float).reshape(3, 3), np.asarray(ci['D'], float)


def _per_line(grid, valid):
    def measure(lines, allowed):
        result = []
        for i, (line, mask) in enumerate(zip(lines, allowed)):
            points = line[mask]
            entry = {'index': i+1, 'points': len(points), 'rms_px': None}
            if len(points) >= 3:
                centered = points-points.mean(axis=0)
                _, _, vh = np.linalg.svd(centered, full_matrices=False)
                residual = centered @ vh[-1]
                entry['rms_px'] = float(np.sqrt(np.mean(residual**2)))
            result.append(entry)
        known = [row for row in result if row['rms_px'] is not None]
        weight = sum(row['points'] for row in known)
        return {'lines': result, 'rms_px': (sum(row['rms_px']**2*row['points'] for row in known)/weight)**.5 if weight else None}
    return {'rows': measure(grid, valid), 'columns': measure(grid.transpose(1, 0, 2), valid.T)}


def _compare(grid, calibration):
    model, size, k, d = calibration
    pts = grid.reshape(-1, 2)
    rays = _rays(model, pts, k, d)
    zero = np.zeros(3)
    xy = np.c_[rays, np.ones(len(rays))]
    returned = _project(model, xy, zero, zero, k, d)
    diff = np.linalg.norm(returned-pts, axis=1)
    # Check the local forward mapping as well; a good round trip alone can return another inverse branch.
    eps = 1e-5
    dx = (_project(model, xy+[eps, 0, 0], zero, zero, k, d)-returned)/eps
    dy = (_project(model, xy+[0, eps, 0], zero, zero, k, d)-returned)/eps
    valid = ((diff <= .25) & (dx[:, 0]*dy[:, 1]-dx[:, 1]*dy[:, 0] > 0)).reshape(grid.shape[:2])
    undistorted = (xy @ k.T)[:,:2].reshape(grid.shape)
    return {'raw': _per_line(grid, valid), 'rectified_same_k': _per_line(undistorted, valid),
            'valid': valid.tolist(), 'used_points': int(np.count_nonzero(valid)),
            'roundtrip_error_px': [float(v) if np.isfinite(v) else None for v in diff]}


def _read_view(spec, base, pattern, camera):
    pattern = tuple(spec.get('pattern', pattern))
    if len(pattern) != 2 or min(pattern) < 3 or pattern[0]*pattern[1] < 12:
        raise ValueError('请为可见连续子网格指定列数和行数，每方向至少三个内角点、总数至少十二')
    path = base/spec['image']
    data = np.fromfile(path, np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_COLOR) if data.size else None
    if image is None:
        raise ValueError('无法读取输入图像')
    _, size, _, _ = camera
    rect = spec.get('preview_rect') or [0, 0, image.shape[1], image.shape[0]]
    x, y, w, h = rect
    if min(x,y) < 0 or min(w,h) <= 0 or x+w > image.shape[1] or y+h > image.shape[0]:
        raise ValueError('预览图有效范围越过原图边界')
    if spec.get('preview_rect') is None and tuple(rect[2:]) != size:
        raise ValueError('非原始分辨率，请用 preview_rect 指定完整相机画面在截图中的位置')
    scale = spec.get('vertical_scale', 1.)
    if not 1 <= scale <= 5:
        raise ValueError('检测放大倍率须为 1 至 5')
    roi = spec.get('search_roi') or rect
    if min(roi[2:]) < 1 or roi[0] < x or roi[1] < y or roi[0]+roi[2] > x+w or roi[1]+roi[3] > y+h:
        raise ValueError('棋盘搜索框必须落在完整相机画面内')
    points = _find_grid(image, pattern, spec.get('channel', 'yellow'), scale, roi)
    if points is None:
        raise ValueError('该视角未检出完整且连续的内角点网格')
    pixels = points.reshape(pattern[1], pattern[0], 2)
    if pixels[0,:,1].mean() > pixels[-1,:,1].mean():
        pixels = pixels[::-1]
    if pixels[:,0,0].mean() > pixels[:,-1,0].mean():
        pixels = pixels[:,::-1]
    grid = (pixels-[x,y]+.5)*(np.asarray(size)/[w,h])-.5
    return {'name': spec.get('name', path.stem), 'file': path.name, 'preview_rect': rect,
            'search_roi': roi, 'pattern': pattern, 'vertical_scale': scale, 'source_pixels': pixels.tolist(), 'grid': grid.tolist(),
            'straightness': _compare(grid, camera)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sources = parser.add_mutually_exclusive_group(required=True)
    sources.add_argument('--image', type=Path, help='单张黄白色平面棋盘照片')
    sources.add_argument('--views', type=Path, help='多视图 JSON，各项可指定 image/name/search_roi/preview_rect/vertical_scale/pattern')
    sources.add_argument('--points', type=Path, help='复用已记录的角点坐标，计算同组点的校正前后直线误差')
    parser.add_argument('--camera', type=Path, required=True, help='ROS 相机标定 JSON 或含 ci 的实机采样 JSON')
    parser.add_argument('--pattern', default='12x3', help='内角点列数x行数')
    parser.add_argument('--search-roi', type=int, nargs=4, metavar=('X','Y','W','H'), help='单图棋盘搜索范围')
    parser.add_argument('--preview-rect', type=int, nargs=4, metavar=('X','Y','W','H'), help='截图中完整相机画面所占区域')
    parser.add_argument('--vertical-scale', type=float, default=1., help='低视角可尝试 2 或 3')
    parser.add_argument('--out', type=Path, required=True, help='角点与逐条直线误差 JSON 文件')
    args = parser.parse_args()
    try:
        cols, rows = map(int, args.pattern.lower().split('x'))
        if min(cols, rows) < 3 or cols*rows < 12:
            raise ValueError('棋盘每方向至少三个内角点，总数至少十二')
        camera = _target(json.loads(args.camera.read_bytes()))
    except (ValueError, KeyError) as exc:
        parser.error(str(exc))
    if args.points:
        result = json.loads(args.points.read_bytes())
        if tuple(result['size']) != camera[1]:
            parser.error('角点保存分辨率与所选标定不一致')
        for view in result['views']:
            grid = np.asarray(view['grid'], float)
            if grid.ndim != 3 or grid.shape[-1] != 2 or not np.isfinite(grid).all():
                parser.error('角点文件中存在不完整的坐标')
            view['straightness'] = _compare(grid, camera)
        args.out.parent.mkdir(parents=True, exist_ok=True)
        _save(args.out, result)
        return 0
    if args.image:
        base = args.image.parent
        specs = [{'image': args.image.name, 'search_roi': args.search_roi, 'preview_rect': args.preview_rect,
                  'vertical_scale': args.vertical_scale}]
    else:
        base = args.views.parent
        specs = json.loads(args.views.read_bytes())['views']
    result = {'size': list(camera[1]), 'pattern_inner_corners': [cols, rows], 'views': [], 'excluded': []}
    for spec in specs:
        try:
            result['views'].append(_read_view(spec, base, (cols,rows), camera))
        except (ValueError, OSError, cv2.error) as exc:
            result['excluded'].append({'file': spec['image'], 'reason': str(exc)})
    args.out.parent.mkdir(parents=True, exist_ok=True)
    _save(args.out, result)
    print(f"有效视图 {len(result['views'])}，未采用 {len(result['excluded'])}，报告 {args.out}")
    return int(not result['views'])


if __name__ == '__main__':
    raise SystemExit(main())
