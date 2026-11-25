"""Green/orange blocks and a floor-row pickup target in camera coordinates.
Legacy source: Silver robot refactor/perception/box_detector.py. HSV contour
approach only; old pixel thresholds and elongated-contour count are NOT reused.
"""
import math
from itertools import combinations


def default_config():
    return {'calibration_verified': False, 'empty_scene_verified': False, 'floor_pair_silhouettes': False,
            'topic_confirmed': False, 'topic': '/usb_cam_chest/image_raw',
            'frame_id': '', 'resolution': [640, 480], 'roi': [0., 0., 1., 1.],
            'hsv_ranges': {}, 'min_area': .002, 'max_area': .35,
            'max_aspect': 1.8, 'min_fill': .55, 'edge_margin': .005, 'isolation_gap': .08,
            'confirmation_frames': 3, 'stability': .04,
            'max_age_s': .6, 'timeout_s': 3., 'settle_s': .3,
            'min_brightness': 15., 'max_brightness': 245.}


def validate_config(cfg, live=False, *, staged_trial=False):
    if not isinstance(cfg, dict): raise ValueError('block vision config required')
    for key in ('calibration_verified', 'empty_scene_verified', 'topic_confirmed'):
        if type(cfg.get(key)) is not bool: raise ValueError(key)
    if type(cfg.get('floor_pair_silhouettes', False)) is not bool:
        raise ValueError('floor_pair_silhouettes')
    for key in ('min_area', 'max_area', 'max_aspect', 'min_fill', 'edge_margin',
                'isolation_gap', 'stability', 'max_age_s', 'timeout_s', 'settle_s',
                'min_brightness', 'max_brightness'):
        v = cfg.get(key)
        if type(v) not in (int, float) or not math.isfinite(v) or v < 0:
            raise ValueError(key)
    if not 0 < cfg['min_area'] < cfg['max_area'] < 1: raise ValueError('area range')
    if not 1 <= cfg['max_aspect'] <= 5 or not 0 < cfg['min_fill'] <= 1: raise ValueError('shape limits')
    if not 0 < cfg['isolation_gap'] < .5: raise ValueError('isolation_gap')
    if not 0 <= cfg['edge_margin'] < .1 or not 0 < cfg['stability'] < .5: raise ValueError('spatial limits')
    if not 0 < cfg['max_age_s'] <= 5 or not 0 < cfg['timeout_s'] <= 20 or cfg['settle_s'] > 5: raise ValueError('time limits')
    if not 0 <= cfg['min_brightness'] < cfg['max_brightness'] <= 255: raise ValueError('brightness range')
    if type(cfg.get('confirmation_frames')) is not int or not 2 <= cfg['confirmation_frames'] <= 10: raise ValueError('confirmation_frames')
    r = cfg.get('resolution', [])
    if len(r) != 2 or any(type(v) is not int or not 16 <= v <= 4096 for v in r): raise ValueError('resolution')
    roi = cfg.get('roi', [])
    if len(roi) != 4 or any(type(v) not in (int, float) or not math.isfinite(v) for v in roi) or not (0 <= roi[0] < roi[2] <= 1 and 0 <= roi[1] < roi[3] <= 1): raise ValueError('roi')
    ranges = cfg.get('hsv_ranges')
    if not isinstance(ranges, dict): raise ValueError('hsv_ranges')
    for color, limits in ranges.items():
        if color not in ('green', 'orange') or not isinstance(limits, dict): raise ValueError('block color')
        lo, hi = limits.get('lower', []), limits.get('upper', [])
        if len(lo) != 3 or len(hi) != 3 or any(type(a) is not int or type(b) is not int or not 0 <= a <= b <= bound for a,b,bound in zip(lo,hi,(179,255,255))): raise ValueError('HSV limits')
    if live:
        if (not cfg['topic_confirmed'] or not cfg.get('frame_id') or set(ranges) != {'green','orange'}
                or not isinstance(cfg.get('topic'),str) or not cfg['topic'].startswith('/')):
            raise ValueError('block vision requires chest source and green/orange HSV thresholds')
        if not staged_trial and not all(cfg[k] for k in ('calibration_verified','empty_scene_verified')):
            raise ValueError('block vision requires verified scene/camera/HSV calibration before motion')
    return cfg


def unknown(reason):
    return {'status': 'unknown', 'count': None, 'target': None, 'candidates': [], 'reason': reason}


def _floor_pair(candidates):
    """A horizontal pair has adjacent sides and aligned bottom edges."""
    pairs = []
    for left, right in combinations(candidates, 2):
        lx, ly, lw, lh = left['bbox']
        rx, ry, rw, rh = right['bbox']
        width = min(lw, rw)
        height = min(lh, rh)
        if max(lw, rw) > 1.6 * width or max(lh, rh) > 1.6 * height:
            continue
        if not -.05 * width <= rx - (lx + lw) <= .65 * width:
            continue
        if abs((ly + lh) - (ry + rh)) > .35 * height:
            continue
        if len(candidates) == 3:
            upper = next(c for c in candidates if c is not left and c is not right)
            _, uy, _, uh = upper['bbox']
            if not (lx <= upper['x'] <= rx + rw and
                    upper['y'] < min(left['y'], right['y']) - .3 * height and
                    abs(uy + uh - (ly + ry) / 2) <= .55 * height):
                continue
        pairs.append((left, right))
    return pairs[0] if len(pairs) == 1 else None


def _split_floor_silhouette(mask, contour, bounds):
    """Decompose a clear wide floor row, optionally with a narrow block above."""
    import cv2
    import numpy as np

    x,y,w,h=bounds
    if min(w,h)<8 or w/h<.7 or w/h>2.45:
        return None
    shape=np.zeros_like(mask[y:y+h,x:x+w])
    cv2.drawContours(shape,[contour],-1,1,cv2.FILLED,offset=(-x,-y))
    # Intersect the outline with the color mask so occlusions do not appear solid.
    shape *= mask[y:y+h,x:x+w] > 0
    widths=np.sum(shape,axis=1)
    broad=widths>=.8*w
    if 1.65<=w/h<=2.45 and np.mean(broad)>=.88:
        row_start=0
        upper=None
    else:
        broad_rows=np.flatnonzero(broad)
        if broad_rows.size==0:
            return None
        row_start=int(broad_rows[0])
        if not .29*h<=row_start<=.65*h:
            return None
        if not 1.65<=w/(h-row_start)<=2.45 or np.mean(broad[row_start:])<.85:
            return None
        # The upper block must also have a solid, almost rectangular top silhouette.
        columns=np.flatnonzero(np.mean(shape[:row_start],axis=0)>=.75)
        if columns.size==0:
            return None
        left,right=int(columns[0]),int(columns[-1])+1
        if not .38*w<=right-left<=.64*w or np.mean(shape[:row_start,left:right])<.82:
            return None
        upper=(x+left,y,right-left,row_start)
    half=w//2
    boxes=[(x,y+row_start,half,h-row_start),(x+half,y+row_start,w-half,h-row_start)]
    if upper is not None:
        boxes.append(upper)
    return boxes


def _staged_silhouette_count(hsv, cfg, roi):
    """Count staged blocks by their combined outline, independent of color placement."""
    import cv2
    import numpy as np

    x0, y0, x1, y1 = roi
    height, width = hsv.shape[:2]
    crop = hsv[y0:y1, x0:x1]
    mask = np.zeros(crop.shape[:2], np.uint8)
    for color, limits in cfg['hsv_ranges'].items():
        upper = list(limits['upper'])
        if color == 'orange':
            # The yellow wall bleeds into the ordinary orange mask under the chest lighting.
            upper[0] = min(upper[0], 21)
            upper[0] = max(upper[0], limits['lower'][0])
        mask |= cv2.inRange(crop, np.array(limits['lower'], np.uint8), np.array(upper, np.uint8))
    # Close narrow seams between touching colors and discard thin shadows/noise.
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
    min_area = cfg['min_area'] * width * height
    contours = [c for c in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
                if cv2.contourArea(c) >= min_area]
    if not 1 <= len(contours) <= 3:
        return None
    outline = np.vstack(contours)
    x, y, w, h = bounds = cv2.boundingRect(outline)
    margin = cfg['edge_margin']
    area = sum(cv2.contourArea(contour) for contour in contours)
    if (min(w, h) < 12 or area > cfg['max_area'] * width * height
            or x <= margin*width or y <= margin*height
            or x+w >= crop.shape[1]-margin*width or y+h >= crop.shape[0]-margin*height):
        return None
    hull_area = cv2.contourArea(cv2.convexHull(outline))
    if not hull_area:
        return None
    solidity = area / hull_area
    fill = area / (w*h)
    boxes = _split_floor_silhouette(mask, contours[0], bounds) if len(contours) == 1 else None
    if .60 <= w/h <= 1.65 and fill >= cfg['min_fill'] and .65 <= solidity <= .955:
        shape = np.zeros((h, w), np.uint8)
        cv2.drawContours(shape, contours, -1, 1, cv2.FILLED, offset=(-x, -y))
        shape &= mask[y:y+h, x:x+w] > 0
        # Look for a persistent upper block and a wider support row, even when
        # the two base colors merge with the upper face or separate in shadow.
        upper = shape[int(.15*h):int(.40*h)]
        lower = shape[int(.53*h):int(.97*h)]
        upper_cols = np.flatnonzero(np.mean(upper, axis=0) >= .55)
        lower_cols = np.flatnonzero(np.mean(lower, axis=0) >= .35)
        if upper_cols.size and lower_cols.size:
            upper_width = upper_cols[-1] - upper_cols[0] + 1
            lower_width = lower_cols[-1] - lower_cols[0] + 1
            over_base = (min(lower_cols[-1], upper_cols[-1])
                         - max(lower_cols[0], upper_cols[0]) + 1)
            lower_rows = np.count_nonzero(lower, axis=1)
            stack = (.25*w <= upper_width <= .78*w and upper_cols.size >= .75*upper_width
                     and lower_width >= max(.82*w, 1.22*upper_width)
                     and over_base >= .70*upper_width
                     and np.count_nonzero(lower_rows >= max(.62*w, 1.10*upper_width)) >= .13*h)
            if stack:
                return {'status': 'target', 'count': 3, 'target': None, 'candidates': [],
                        'stacked_triple': True, 'reason': 'staged block group silhouette',
                        'silhouette_bbox_px': [x+x0, y+y0, w, h]}
    if boxes:
        count = len(boxes)
    elif len(contours) == 1 and 1.55 <= w/h <= 2.45 and solidity >= .90 and fill >= .76:
        count = 2
    elif len(contours) == 1 and solidity >= .945 and fill >= cfg['min_fill'] and .65 <= w/h <= 1.36:
        count = 1
    elif len(contours) == 1 and .72 <= solidity < .945 and fill >= cfg['min_fill'] and .70 <= w/h <= 1.60:
        shape = np.zeros((h, w), np.uint8)
        cv2.drawContours(shape, contours, -1, 1, cv2.FILLED, offset=(-x, -y))
        widths = np.count_nonzero(shape & (mask[y:y+h, x:x+w] > 0), axis=1)
        top = np.median(widths[int(.18*h):int(.39*h)])
        bottom = widths[int(.56*h):int(.84*h)]
        if (top < .36*w or np.median(bottom) < 1.30*top
                or np.mean(bottom >= .72*w) < .85):
            return None
        count = 3
    else:
        return None
    return {'status': 'target', 'count': count, 'target': None, 'candidates': [],
            'stacked_triple': count == 3, 'reason': 'staged block group silhouette',
            'silhouette_bbox_px': [x+x0, y+y0, w, h]}


class BlockDetector:
    def __init__(self, cfg, *, staged_trial=False, trial_alignment=False):
        self.cfg = validate_config(cfg)
        self.staged_trial = staged_trial
        self.trial_alignment = trial_alignment

    def classify(self, candidates, width, height, brightness=128., *, count_only=False):
        c = self.cfg
        if [width,height] != c['resolution']: return unknown('resolution changed; recalibration required')
        if not (c['calibration_verified'] or self.staged_trial and (count_only or self.trial_alignment)):
            return unknown('visual calibration unverified')
        if not c['min_brightness'] <= brightness <= c['max_brightness']: return unknown('dark/overexposed or occluded scene')
        valid = []
        for item in candidates:
            x,y,w,h = item['bbox']; area = item['area']
            if area < c['min_area']: continue
            if (area > c['max_area'] or min(w,h) <= 0 or max(w*width,h*height)/min(w*width,h*height) > c['max_aspect'] or area/(w*h) < c['min_fill']):
                return unknown('merged/occluded/non-block shape; cannot infer count')
            a,b,d,e = c['roi']; m = c['edge_margin']
            if x <= a+m or y <= b+m or x+w >= d-m or y+h >= e-m:
                return unknown('candidate clipped at ROI edge')
            valid.append(dict(item, x=x+w/2., y=y+h/2.))
        if len(valid) > 3: return unknown('more than three candidates/clutter')
        pair = _floor_pair(sorted(valid, key=lambda block: block['x']))
        stacked_triple = len(valid) == 3 and pair is not None
        for i,a in enumerate(valid):
            for b in valid[i+1:]:
                ax,ay,aw,ah=a['bbox']; bx,by,bw,bh=b['bbox']
                overlap_x = min(ax+aw,bx+bw) - max(ax,bx)
                overlap_y = min(ay+ah,by+bh) - max(ay,by)
                if overlap_x>1e-9 and overlap_y>1e-9 and (pair is None or a in pair and b in pair):
                    return unknown('overlapping color regions; count ambiguous')
        if not valid and not c['empty_scene_verified']: return unknown('empty scene visibility not validated')
        if count_only:
            return {'status':'target' if valid else 'empty', 'count':len(valid),
                    'target':None, 'candidates':valid, 'stacked_triple':stacked_triple,
                    'reason':'visible block count'}
        for a in valid:
            ax,ay,aw,ah=a['bbox']
            a['isolated_single']=all(max(b['bbox'][0]-(ax+aw), ax-(b['bbox'][0]+b['bbox'][2])) >= c['isolation_gap'] for b in valid if b is not a)
        valid.sort(key=lambda a: (-a['y'], -a['area'], a['x']))
        grip_count = 0
        pickup_target = None
        if pair:
            left, right = pair
            grip_count = 2
            pickup_target = {
                'x': (left['x'] + right['x']) / 2,
                'y': (left['y'] + right['y']) / 2,
                'color': (left['color'], right['color']),
            }
        elif len(valid) < 3:
            pickup_target = next((block for block in valid if block['isolated_single']), None)
            grip_count = 1 if pickup_target is not None else 0
        return {'status':'target' if valid else 'empty', 'count':len(valid),
                'target':valid[0] if valid else None, 'candidates':valid,
                'grip_count':grip_count, 'pickup_target':pickup_target, 'stacked_triple':stacked_triple,
                'reason':'visible separated color contours; physical grip unobserved'}

    def detect(self, bgr, *, count_only=False):
        import cv2
        import numpy as np
        if not isinstance(bgr, np.ndarray) or bgr.ndim != 3 or bgr.shape[2] != 3 or bgr.dtype != np.uint8:
            return unknown('invalid BGR image')
        h,w = bgr.shape[:2]
        if [w,h] != self.cfg['resolution']: return unknown('resolution changed; recalibration required')
        if (not (self.cfg['calibration_verified'] or self.staged_trial and (count_only or self.trial_alignment))
                or set(self.cfg['hsv_ranges']) != {'green','orange'}):
            return unknown('HSV calibration unverified')
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        a,b,d,e = self.cfg['roi']; x0,y0,x1,y1=int(a*w),int(b*h),int(d*w),int(e*h)
        brightness = float(np.mean(hsv[y0:y1,x0:x1,2]))
        staged_shape = (count_only and self.staged_trial and self.cfg.get('floor_pair_silhouettes')
                        and self.cfg['min_brightness'] <= brightness <= self.cfg['max_brightness'])
        if staged_shape:
            group = _staged_silhouette_count(hsv, self.cfg, (x0, y0, x1, y1))
            if group is not None:
                return group
        candidates=[]
        for color,limits in self.cfg['hsv_ranges'].items():
            mask=cv2.inRange(hsv,np.array(limits['lower'],np.uint8),np.array(limits['upper'],np.uint8))
            mask[:y0,:]=0;mask[y1:,:]=0;mask[:,:x0]=0;mask[:,x1:]=0
            contours=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
            for contour in contours:
                x,y,bw,bh=cv2.boundingRect(contour)
                area=float(cv2.contourArea(contour))
                if self.cfg.get('floor_pair_silhouettes') and area/(w*h)>=self.cfg['min_area']:
                    boxes=_split_floor_silhouette(mask,contour,(x,y,bw,bh))
                    if boxes:
                        for bx,by,bw_part,bh_part in boxes:
                            candidates.append({'bbox':[bx/w,by/h,bw_part/w,bh_part/h],'color':color,
                                'area':float(np.count_nonzero(mask[by:by+bh_part,bx:bx+bw_part]))/(w*h),
                                'source':'floor_silhouette'})
                        continue
                    hull_area=cv2.contourArea(cv2.convexHull(contour))
                    if hull_area and area/hull_area<.9:
                        return unknown('concave merged block layout without a clear bottom pair')
                candidates.append({'bbox':[x/float(w),y/float(h),bw/float(w),bh/float(h)],
                                   'area':area/(w*h),'color':color})
        return self.classify(candidates,w,h,brightness,count_only=count_only)


class ConfirmedWindow:
    def __init__(self,cfg,*,count_only=False):
        self.cfg=validate_config(cfg);self.rows=[];self.last_stamp=0
        self.count_only=count_only

    def add(self,result,frame):
        stamp=frame['stamp_ns']
        if stamp <= self.last_stamp: return unknown('duplicate/out-of-order frame')
        self.last_stamp=stamp
        if result['status'] not in ('target','empty'):
            self.rows=[];return result
        if self.rows:
            prev,pf=self.rows[-1]
            same=all(frame.get(k)==pf.get(k) for k in ('scope','load','width','height'))
            if self.count_only:
                same=same and min(2,prev['count'])==min(2,result['count'])
            else:
                same=same and prev['count']==result['count'] and prev.get('grip_count')==result.get('grip_count')
            if same and result['count'] and not self.count_only:
                before=sorted(prev['candidates'],key=lambda t:(t['color'],t['x']))
                after=sorted(result['candidates'],key=lambda t:(t['color'],t['x']))
                same=all(a['color']==b['color'] and abs(a['x']-b['x'])<=self.cfg['stability'] and abs(a['y']-b['y'])<=self.cfg['stability'] for a,b in zip(before,after))
            if not same:self.rows=[]
        self.rows.append((result,frame));self.rows=self.rows[-self.cfg['confirmation_frames']:]
        if len(self.rows)<self.cfg['confirmation_frames']: return unknown('insufficient stable independent frames')
        out=dict(result);out.update({k:frame[k] for k in ('stamp_ns','received_monotonic','scope','load','width','height')})
        out['confirmation_frames']=len(self.rows)
        # In count-only mode, two and three share a grab action. Only a
        # consistent stack can add the ungripped upper block to the estimate.
        out['stacked_triple']=all(row.get('stacked_triple') is True for row,_ in self.rows)
        return out
