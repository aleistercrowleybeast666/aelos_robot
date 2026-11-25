"""Report a persistent magnetic anomaly and any accompanying scene change."""

import math
import statistics
from enum import Enum
from typing import NamedTuple


class FallDiagnosticResult(Enum):
    NO_EVIDENCE = 'insufficient_evidence'
    EXPECTED_TURN = 'intentional_turn'
    NO_PERSISTENT_CHANGE = 'no_persistent_magnetic_change'
    MAGNETIC_ANOMALY = 'persistent_magnetic_anomaly'
    MAGNETIC_AND_SCENE_ANOMALY = 'persistent_magnetic_anomaly_with_scene_change'


_HISTORY_S = 4.0
_FRESH_S = 0.9
# Diagnostic thresholds only. They carry no posture or deployment calibration.
_MAX_CAPTURE_INTERVAL_S = 0.75
_MIN_JUMP_DEG = 45.0
_MAX_STABLE_SPREAD_DEG = 14.0
_MIN_VERTICAL_FRACTION_CHANGE = 0.20
_MIN_IMAGE_LUMA_CHANGE = 0.23
_MAX_IMAGE_LUMA_SPREAD = 0.12


class _MagneticSample(NamedTuple):
    sequence: int
    lower: float
    ready: float
    direction: tuple
    norm: float
    session: str
    calibration: str


def FallDiag_ImageLuma(message):
    """Return normalized mean sampled luma, or None for unsupported raw data."""
    encoding = getattr(message, 'encoding', '')
    if encoding not in ('mono8', 'rgb8', 'bgr8', 'rgba8', 'bgra8'):
        return None
    channels = {'mono8': 1, 'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}[encoding]
    width, height, step = (getattr(message, k, 0) for k in ('width', 'height', 'step'))
    if (type(width) is not int or type(height) is not int or type(step) is not int
            or min(width, height) < 8 or step < width * channels):
        return None
    try:
        data = memoryview(message.data).cast('B')
    except (AttributeError, TypeError, ValueError):
        return None
    if len(data) < height * step:
        return None
    r, b = (2, 0) if encoding.startswith('bgr') else (0, 2)
    luma = []
    for row in range(6):
        y = (2 * row + 1) * height // 12
        for col in range(8):
            x = (2 * col + 1) * width // 16
            offset = y * step + x * channels
            value = data[offset] if channels == 1 else (
                0.299 * data[offset + r] + 0.587 * data[offset + 1] + 0.114 * data[offset + b])
            luma.append(value)
    return statistics.mean(luma) / 255


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _field_rows(rows, now):
    valid = []
    for row in rows:
        if not isinstance(row, dict) or row.get('capture_source') != 'verified_sensor_capture':
            continue
        lower, ready = row.get('capture_not_before_monotonic'), row.get('ready_observed_monotonic')
        seq, xyz = row.get('seq'), row.get('xyz')
        if (not _finite(lower) or not _finite(ready) or not 0 <= lower <= ready <= now
                or not 0 <= now - lower <= _HISTORY_S or type(seq) is not int or seq < 1
                or not isinstance(xyz, (list, tuple)) or len(xyz) != 3 or not all(
                    _finite(value) and -32768 < value < 32767 for value in xyz)):
            continue
        session, calibration = row.get('session'), row.get('calibration_id')
        if not isinstance(session, str) or not session or not isinstance(calibration, str) or not calibration:
            continue
        norm = math.hypot(*xyz)
        if norm > 0:
            valid.append(_MagneticSample(seq, lower, ready, tuple(value / norm for value in xyz),
                                         norm, session, calibration))
    return valid


def _angle(a, b):
    dot = sum(left * right for left, right in zip(a, b))
    return math.degrees(math.acos(max(-1.0, min(1.0, dot))))


def _stable(group):
    return all(_angle(group[0].direction, row.direction) <= _MAX_STABLE_SPREAD_DEG for row in group[1:])


def _scene_change(images, now, lower, upper):
    usable = []
    for row in images:
        if not isinstance(row, dict):
            continue
        received, luma, stamp = row.get('received_monotonic'), row.get('luma'), row.get('stamp_ns')
        if (not _finite(received) or not 0 <= now - received <= _HISTORY_S
                or not _finite(luma) or not 0 <= luma <= 1 or type(stamp) is not int or stamp <= 0):
            continue
        if not usable or stamp > usable[-1][2]:
            usable.append((received, luma, stamp))
    before = [luma for receipt, luma, _ in usable if receipt <= lower][-3:]
    after = [luma for receipt, luma, _ in usable if receipt >= upper][-3:]
    info = {'status': 'insufficient_fresh_scene_evidence', 'before_count': len(before),
            'after_count': len(after), 'exposure_verified': False}
    if min(len(before), len(after)) < 2 or now - usable[-1][0] > _FRESH_S:
        return info
    first, last = statistics.median(before), statistics.median(after)
    changed = (abs(first - last) >= _MIN_IMAGE_LUMA_CHANGE and
               max(max(group) - min(group) for group in (before, after)) <= _MAX_IMAGE_LUMA_SPREAD)
    return dict(info, status='sustained_brightness_change' if changed else 'no_sustained_brightness_change',
                before_luma=round(first, 4), after_luma=round(last, 4))


def FallDiag_Assess(action, rows, images, now):
    """Identify a short magnetic jump and check whether its new field persists."""
    report = {'status': FallDiagnosticResult.NO_EVIDENCE.value, 'action': action,
              'diagnostic_only': True, 'posture_verified': False, 'front_back': None,
              'recovery_dispatched': False}
    if not isinstance(action, str) or not action or not _finite(now):
        return report
    if 'turn' in action or action in ('single_micro_left', 'single_micro_right'):
        return dict(report, status=FallDiagnosticResult.EXPECTED_TURN.value)
    captures = _field_rows(rows, now)
    report['magnetic_samples'] = len(captures)
    if len(captures) < 5 or now - captures[-1].ready > _FRESH_S or any(
            current.sequence <= previous.sequence or current.lower <= previous.lower
            or current.session != previous.session or current.calibration != previous.calibration
            for previous, current in zip(captures, captures[1:])):
        return report
    for idx in reversed(range(2, len(captures) - 2)):
        pre, post = captures[idx - 2:idx], captures[idx:]
        before, after = pre[-1], post[0]
        interval = after.ready - before.lower
        if (after.sequence != before.sequence + 1 or not 0 < interval <= _MAX_CAPTURE_INTERVAL_S
                or not _stable(pre) or not _stable(post)):
            continue
        angle = _angle(before.direction, after.direction)
        vertical_change = abs(before.direction[2] - after.direction[2])
        norm_change = abs(after.norm - before.norm) / before.norm
        if angle < _MIN_JUMP_DEG or vertical_change < _MIN_VERTICAL_FRACTION_CHANGE:
            continue
        # A scene change can accompany a fall, an obstruction or auto-exposure.
        # Neither it nor a rotated/disturbed magnetic field determines posture.
        camera = _scene_change(images, now, before.lower, after.ready)
        status = (FallDiagnosticResult.MAGNETIC_AND_SCENE_ANOMALY
                  if camera['status'] == 'sustained_brightness_change' else FallDiagnosticResult.MAGNETIC_ANOMALY)
        return dict(report, status=status.value, camera=camera,
                    magnetic={'max_observation_interval_s': round(interval, 3),
                              'raw_vector_jump_deg': round(angle, 2),
                              'vertical_field_fraction_change': round(vertical_change, 4),
                              'raw_field_norm_fraction_change': round(norm_change, 4),
                              'before_samples': len(pre), 'persisting_samples': len(post)})
    return dict(report, status=FallDiagnosticResult.NO_PERSISTENT_CHANGE.value)
