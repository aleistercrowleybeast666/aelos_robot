"""Camera failures and control-connection checks shared by Tag checkpoints."""

from enum import Enum
from functools import partial


class CheckpointStreamResult(Enum):
    SOURCE_MISMATCH = 'source_mismatch'
    TEMPORARY_GAP = 'temporary_gap'
    UNKNOWN_GAP = 'unknown_gap'


def Checkpoint_ClassifyStreamGap(stream, *, allow_recovery):
    """Classify an unhealthy AR window using the camera parameter and publisher evidence."""
    if stream.get('parameters_match') is False or stream.get('ar_source_matches') is False:
        return CheckpointStreamResult.SOURCE_MISMATCH
    if allow_recovery and stream.get('parameters_match') is True:
        return CheckpointStreamResult.TEMPORARY_GAP
    return CheckpointStreamResult.UNKNOWN_GAP


def Checkpoint_RecordMiss(result, expected_id, seen_ids, healthy_windows, policy, cause, temporary_gaps):
    """Store the evidence and operator message for a completed, unsuccessful scan."""
    result['checkpoint_not_seen'] = {
        'tag_id': expected_id, 'seen_tags': sorted(seen_ids),
        'healthy_windows': healthy_windows, 'observation_windows': result['observation_windows'],
        'policy': policy, 'cause': cause, 'temporary_camera_gaps': temporary_gaps,
    }
    if cause == 'temporary_camera_gap':
        reason = f'Tag {expected_id} 扫码流暂未恢复，已记录并按动作链继续。'
    elif cause == 'insufficient_frames':
        reason = f'Tag {expected_id} 已解码但有效帧不足，起步位姿待确认。'
    elif policy == 'continue_estimated':
        reason = f'未确认 Tag {expected_id}，已记录漏码，按动作链继续。'
    else:
        reason = f'未确认 Tag {expected_id}，已记录漏码，保留持物。'
    result['stationary_wait_reason_zh'] = reason
    return reason


class VisualMicroStopped(RuntimeError):
    def __init__(self, reason, *, report=None):
        super().__init__(reason)
        self.report = report
        if report and report.get('reason_zh'):
            self.error_details = {
                'error_code': 'passive_tag_observation_failed',
                'reason_zh': report['reason_zh'],
            }


class VisualCameraUnavailable(VisualMicroStopped):
    """A camera or AR read failed during an observation."""


def VisualCamera_IsFresh(read, stamp_ns):
    """A failed camera freshness read cannot authorize motion from that pose."""
    try:
        return read(stamp_ns)
    except OSError:
        return False


def VisualCamera_GetFreshnessCheck(io, default=None):
    read = getattr(io, 'visual_motion_ready', None)
    return default if read is None else partial(VisualCamera_IsFresh, read)


def VisualCamera_Read(read, check):
    """Read a camera window while preserving cancellation and control failures."""
    def check_owner():
        try:
            check()
        except OSError as exc:
            raise RuntimeError('读取图像时控制连接校验失败。') from exc

    try:
        return read(check_owner)
    except OSError as exc:
        raise VisualCameraUnavailable('图像或扫码话题读取失败：' + str(exc)) from exc


def VisualCamera_DiagnoseWindow(window, camera, expected_fingerprint):
    """Return (reason, Chinese explanation, can_reobserve), or None when ready."""
    stream = window.get('camera_window_status', {})
    waiting_for_ar = (stream.get('parameters_match') is True
                      and 'ar_source_matches' in stream and stream['ar_source_matches'] is None)
    if (stream.get('parameters_match') is False or stream.get('ar_source_matches') is False
            or (not window.get('contract_match') and not waiting_for_ar)):
        return ('camera/AR source or parameters changed during passive Tag checkpoint',
                '扫码话题来源或参数校验失败。', False)

    sources = (window.get('camera_info', {}), camera)
    for item in sources:
        fingerprint = item.get('fingerprint')
        if ((fingerprint is not None or item.get('ready') is True)
                and fingerprint != expected_fingerprint):
            return ('actual CameraInfo calibration/source/frame changed',
                    '胸部相机标定或来源校验失败。', False)
        if item.get('ready') is not True and not item.get('can_reobserve'):
            return ('actual CameraInfo calibration/source/frame/time unavailable or changed',
                    '胸部相机标定、来源或时间校验失败。', False)

    if not window.get('camera_healthy') or not window.get('contract_match'):
        return ('camera/AR stream fault during passive Tag checkpoint',
                '胸部相机或扫码话题暂未收到符合要求的新帧。', True)
    if any(item.get('ready') is not True for item in sources):
        return ('CameraInfo/actual image timing unavailable during passive Tag checkpoint',
                '胸部相机图像与标定帧暂未同步。', True)
    return None
