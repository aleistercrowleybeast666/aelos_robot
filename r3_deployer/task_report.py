"""Archive terminal robot receipts locally and summarize the last diagnostics."""

from datetime import datetime
import json
import math
import os
from pathlib import Path
import re
import tempfile
from uuid import uuid4

from .control_api import RobotControlUncertainError
from .mission_progress import MissionPickup_DoubleLoadWaitLabel, MissionProgress
from .settings import settings_path


_SECRET_FIELDS = {"password", "passwd", "passphrase", "credential", "secret", "dpapi"}
_HIDDEN = "[密码已隐藏]"
_LEGACY_BOUNDARY_PREFIX = "micro trial stopped at completed macro boundary: "
_LEGACY_BOUNDARY_SUFFIX = "; no remaining route, no reverse retry, no release; native immediate stop unavailable"
_VISUAL_BOUNDARY_PREFIX = "visual micro stopped at completed macro boundary: "
_VISUAL_BOUNDARY_SUFFIX = "; no next route macro, no reverse retry or release; no native immediate stop"
_HEADING_LIMIT = re.compile(r"heading error exceeds configured trial envelope: error=(\S+) limit=(\S+)")
_ACTION_TIMEOUT = re.compile(
    r"operation (\S+) (?:near timeout|returned near timeout \((\S+)s / (\S+)s\); completion uncertain)"
    r"; stop remaining macros"
)
_ACTION_FAILURE = re.compile(r"operation (\S+) failed; reason=([^;]*); last_result=.+")
_CORRECTION_FAILURE = re.compile(r"correction verification failed: ([a-z_,]+); no retry(?:; .*|$)")
_SCAN_LOG_LABELS = ("AR扫码", "入口视觉航向", "入口示教校正", "到位扫码微调")
_MESSAGES_ZH = {
    "previous automatic task may still hold a block; explicitly restore empty stand before starting again":
        "上次任务可能仍在持物，恢复空载站姿后再启动。",
    "empty reset requires idle; no active task cancelled": "当前仍有任务，等待空闲再恢复站姿。",
    "horizontal pair: complete double-load native capability unavailable": "已识别多块，双抱动作包尚未就绪",
    "staged block count changed": "连续观测的方块数量不一致，正在重新识别",
    "two-object release protocol completed after Tag 5 and base approach": "已收到进场后的双抱放物动作回执",
    "single-object release protocol completed after Tag 5 and base approach": "已收到进场后的单块放物动作回执",
    "block camera has no fresh post-settle stream": "动作结束后未收到新的方块相机帧",
    "block camera stopped delivering fresh images during observation": "识别过程中方块相机缺少新帧",
    "camera fault, not confirmed empty": "方块图像不可用",
    "duplicate/reordered pickup observation": "方块图像时间戳重复或乱序",
    "evidence_expired": "取物前图像已经过期",
    "Tag 2 frames disagree about approach heading": "Tag 2 多帧航向偏差不一致",
    "Tag 2 requires heading correction before longitudinal approach": "Tag 2 需要先纠正航向再前进",
    "Tag 2 heading, distance or measurement spread outside trial window": "Tag 2 位姿超出当前进近范围或测量不稳定",
    "two steps would not reliably improve the Tag 2 trial distance": "预计前进两步无法可靠减小 Tag 2 距离误差",
    "Tag 2 longitudinal pose expired before dispatch": "发动作前的 Tag 2 位姿已经过期",
    "Tag 2 trim capability or reserved motion budget unavailable": "Tag 2 补偿动作或剩余动作余量不足",
    "Tag 2 turn did not reduce measured heading error": "Tag 2 转向后航向误差没有减小",
    "Tag 2 distance did not measurably decrease after the forward group": "前进两步后未测到距离缩短",
    "no Tag 2 forward count predicts a landing within the strafe window": "现有连步动作的预计落点均超出右跨范围",
    "Tag 2 forward count fitted to the estimated remaining distance": "已按剩余距离估算前进步数",
    "Tag 5 requires at least three distinct ground poses": "地面位姿不足三帧",
    "Tag 5 ground-plane pose unavailable": "暂无可用的地面位姿",
    "Tag 5 ground poses disagree across the fresh frames": "新帧之间的地面位姿不一致",
    "Tag 5 outside the passage depth window": "距码超出入口深度范围",
    "Tag 5 heading outside the passage tolerance": "航向超出入口容差",
    "Tag 5 projected path exceeds the lateral passage tolerance": "预计直行横偏超出入口余量",
    "Tag 5 next straight fits the trial passage": "预计下一段直行满足试跑通行范围",
    "Tag 5 passage observation expired": "入口观测已过期",
    "Tag 5 moved out of view after a checked straight with stable heading": "已核验直行，航向稳定，Tag 5 已移出视野",
    "last Tag 5 strafe did not reduce lateral error": "上次侧跨未减小横偏",
    "last Tag 5 visual turn did not reduce heading error": "上次视觉小转的改善不足",
    "Tag 5 side adjustment budget exhausted": "本段侧跨纠偏次数已用完",
    "full side step deferred after entering the Tag 5 passage": "已进入通道，暂停整步侧跨",
    "bounded carry recovery unresolved": "持物纠偏未能恢复航向",
    "no safe corridor fallback configured": "未配置备用通行路线",
    "insufficient fresh magnetic receipt events": "未收到足够的新鲜磁力计样本",
    "magnetic owner, capture session or load epoch changed during collection": "磁力计采样过程中控制会话、采样会话或载物状态发生变化",
    "magnetic expected load changed": "磁力计采样对应的载物状态发生变化",
    "magnetic decision window expired or capture invalid immediately before native submission": "动作发出前，磁力计数据已过期或失效",
    "capture contract is unverified, stale, changed or unhealthy": "传感器采样会话无法验证、已过期或异常",
    "capture unhealthy before next straight segment": "下一段直行前，传感器采样异常",
    "corridor heading reference unavailable before motion": "动作开始前未能建立通行航向参考",
    "invalid true action-completion boundary": "无法确认动作的完成时刻",
    "shared journey budget exhausted": "本段路线的纠偏次数已用完",
    "no safe predicted visual correction": "当前没有预计能减小误差的视觉纠偏动作",
    "Verified Tag heading within arrival deadband. No available turn safely improves it.": "Tag 航向已在到位容差内，现有转向动作无法进一步减小误差。",
    "verified residual is smaller than any available same-direction turn can improve": "剩余角度小于同方向动作可有效纠正的范围",
    "visual heading outside bounded correction envelope": "视觉航向误差超出微调范围",
    "target pose expired before paired sample completed": "配对采样完成前，Tag 位姿数据已过期",
    "target lost; no search/retry motion": "目标 Tag 已丢失，未执行搜索动作",
    "bounded ordinary Tag loss; return original route": "短时丢失 Tag，按原路线继续",
    "target pose/geometry/CameraInfo untrusted; no correction": "Tag 位姿、图形或相机参数未通过检查，本次跳过纠偏",
    "same-target visual/magnetic observations disagree; no new correction, continue original route": "同一目标的视觉和磁力计读数不一致，本次跳过纠偏并继续原路线",
    "explicit target ID missing/mismatched; never select first observed ID": "未指定目标 Tag，或扫到的 Tag 与预期不符",
    "MCU handshake missing/stale": "树莓派未收到 MCU 的新握手",
    "warm owner/MCU is not ready and idle": "机器人控制服务或 MCU 尚未就绪，或正在执行动作",
    "owner busy or old cancelled native action not drained": "控制服务正忙，或上次取消的动作尚未结束",
    "owner completion/cancel state did not settle": "未能确认动作完成或取消",
    "local owner disconnected; no automatic resume": "树莓派控制服务已断开，任务未自动恢复",
    "local task cancelled; no automatic resume": "自动任务已取消，后续动作未恢复",
    "manual takeover/cancel; no automatic resume": "已手动接管或取消任务，后续动作未恢复",
    "automatic mode cancelled": "自动模式已取消",
    "journey macro budget exceeded": "动作次数超出本趟任务上限",
    "chain time budget exhausted": "本趟任务已超过时限",
    "AR observation cancelled": "Tag 扫描已取消",
    "fresh nonce/sequence START+COMPLETE evidence missing": "未收到与本次动作匹配的开始和完成回执",
    "SSH transport is inactive.": "SSH 传输连接已断开。",
    "Authentication failed.": "SSH 身份验证失败。",
    "Permission denied": "权限不足",
    "Permission denied (publickey,password).": "SSH 公钥和密码验证均未通过。",
    "receipt lost": "未收到任务回执",
    "cancelled_before_dispatch": "已在发出动作前取消",
    "preflight_rejected": "启动检查未通过",
    "start_unconfirmed": "启动结果待确认",
    "startup_error": "任务启动失败",
    "not_started_or_stopped": "任务未启动或已停止",
    "stopped_no_further_auto_macros": "自动任务已停止发送后续动作",
    "stop_requested": "已收到停止请求",
    "recognized": "已识别",
    "timeout": "超时",
    "partial": "部分完成",
    "blocked": "未执行",
    "no_tag": "未识别到 Tag",
    "heading_accepted_near_target": "航向已在到位容差内",
    "planned_turn_complete": "计划转向已完成",
    "wrong_direction": "方向相反",
    "visual and magnetic heading changes disagree": "视觉与磁力计测得的转角不一致",
    "unreliable_or_oversized_prediction": "转角估计不稳或超过单次上限",
    "prediction selection rejected: no_safe_candidate": "本次没有可用的微转动作",
    "stable entry heading; use planned short approach or stationary Tag 2 scan": "航向复测稳定，按计划完成入口短步或原地扫 Tag 2",
    "transfer forward complete; use the measured heading for the planned turn": "中转区前进完成，按实测航向进入计划转向",
    "insufficient_worst_case_improvement": "无法保证转向后误差减小",
    "single_micro_left": "小左转",
    "single_micro_right": "小右转",
    "single_medium_left": "中左转",
    "single_medium_right": "中右转",
    "corridor_departure": "侧跨前航向",
    "corridor_side_boundary": "侧跨后航向",
    "stationary small turn moved slightly in the wrong direction; resampled before another correction": "小转轻微反向，已复测航向",
}
_CORRECTION_FAILURES_ZH = {
    "wrong_direction_or_zero_change": "实测转向与指令相反或未转动",
    "observed_step_oversized": "实测单次转角过大",
    "observed_angle_budget_exhausted": "累计转角超过本段上限",
}
_MESSAGE_PREFIXES_ZH = {
    "Error reading SSH protocol banner": "未收到 SSH 握手标识",
    "timed out": "操作超时",
    "Permission denied": "权限不足",
    "owner preparation failed: ": "机器人控制服务准备失败 · ",
}


def _redact(value, password):
    if isinstance(value, str):
        return value.replace(password, _HIDDEN) if password else value
    if isinstance(value, dict):
        return {
            _redact(str(key), password): _HIDDEN if str(key).lower() in _SECRET_FIELDS else _redact(item, password)
            for key, item in value.items()
        }
    if isinstance(value, (list, tuple)):
        return [_redact(item, password) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    return value


def _report_path(value):
    if not isinstance(value, (str, Path)) or not str(value).strip():
        raise ValueError("请选择日志文件夹。")
    directory = Path(value).expanduser()
    if not directory.is_absolute() or any(ord(ch) < 32 for ch in str(directory)):
        raise ValueError("日志文件夹须为绝对路径。")
    return directory


def TaskReport_Directory():
    """Return the selected folder, including the original location for older installations."""
    local = settings_path().parent
    try:
        data = (local / "report_preferences.json").read_text(encoding="utf-8")
    except FileNotFoundError:
        return local / "task_reports"
    try:
        return _report_path(json.loads(data)["directory"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("无法读取日志文件夹设置，请重新选择文件夹。") from exc


def _write_json(destination, data):
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile("w", dir=destination.parent, encoding="utf-8",
                                         suffix=".tmp", delete=False) as file:
            temporary = Path(file.name)
            json.dump(data, file, ensure_ascii=False, indent=2, allow_nan=False)
            file.write("\n")
        os.replace(temporary, destination)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def TaskReport_SetDirectory(directory):
    """Check that future receipts can be written before committing the selection."""
    directory = _report_path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=directory, prefix=".r3-write-") as probe:
        probe.write(b"r3")
        probe.flush()
    _write_json(settings_path().with_name("report_preferences.json"), {"directory": str(directory)})
    return directory


def TaskReport_Save(outcome, *, command, target, after_update, password=None):
    """Persist the result on Windows before displaying the terminal task state."""
    if isinstance(outcome, RobotControlUncertainError):
        result = outcome.as_result()
    elif isinstance(outcome, Exception):
        result = {"ok": False, "status": "startup_error",
                  "exception_type": type(outcome).__name__, "reason": str(outcome)}
    else:
        result = outcome
    now = datetime.now().astimezone()
    report = {
        "schema": "r3.console.task-report.v1",
        "received_at": now.isoformat(timespec="milliseconds"),
        "command": command,
        "target": target,
        "after_update": after_update,
        "result": result,
    }
    summary = _failure_summary(result)
    if summary:
        report["summary_zh"] = summary
    report = _redact(report, password)
    filename = f"{now:%Y%m%d_%H%M%S_%f}_cmd{command}_{uuid4().hex[:8]}.json"
    default = settings_path().parent / "task_reports"
    directory = None
    try:
        directory = TaskReport_Directory()
    except (OSError, ValueError):
        report["archive_warning_zh"] = "无法读取日志文件夹设置，本次报告已保存到默认目录。"
    if directory is not None:
        destination = directory / filename
        try:
            _write_json(destination, report)
            return destination
        except OSError:
            if directory == default:
                raise
            report["archive_warning_zh"] = "所选日志文件夹暂时无法写入，本次报告已保存到默认目录。"
    destination = default / filename
    _write_json(destination, report)
    return destination


def _as_dict(value):
    return value if isinstance(value, dict) else {}


def _receipt_events(receipt):
    result = _as_dict(receipt)
    events = result.get("events") or _as_dict(result.get("observed_status")).get("events")
    return events if isinstance(events, list) else []


def _reason_details(message):
    if (translated := _MESSAGES_ZH.get(message)) is not None:
        return translated
    if (limit := _HEADING_LIMIT.fullmatch(message)) is not None:
        return f"航向偏差 {limit[1]}° 超出试跑上限 {limit[2]}°"
    if (timeout := _ACTION_TIMEOUT.fullmatch(message)) is not None:
        elapsed = f"（耗时 {timeout[2]} 秒，上限 {timeout[3]} 秒）" if timeout[2] else ""
        return f"动作 {timeout[1]} 接近超时{elapsed}，完成状态未确认，未执行后续动作"
    if (failure := _ACTION_FAILURE.fullmatch(message)) is not None:
        detail = _MESSAGES_ZH.get(failure[2], failure[2])
        return f"动作 {failure[1]} 执行失败 · {detail}" if detail and detail != "None" else f"动作 {failure[1]} 执行失败"
    if (failure := _CORRECTION_FAILURE.fullmatch(message)) is not None:
        checks = "、".join(_CORRECTION_FAILURES_ZH.get(check, check) for check in failure[1].split(","))
        return f"转向复测未通过 · {checks}"
    if message.startswith("fresh magnetic sample unavailable: "):
        sample = message.removeprefix("fresh magnetic sample unavailable: ")
        if "insufficient fresh magnetic receipt events" in sample:
            return _MESSAGES_ZH["insufficient fresh magnetic receipt events"]
        return f"未收到可用的新鲜磁力计样本 · {sample}"
    fallback = "; no safe corridor fallback configured"
    if message.endswith(fallback):
        reason = _reason_details(message.removesuffix(fallback))
        return f"{reason}，{_MESSAGES_ZH['no safe corridor fallback configured']}"
    for prefix, translated in _MESSAGE_PREFIXES_ZH.items():
        if message.startswith(prefix):
            return translated + message[len(prefix):]
    return message


def TaskReport_Message(value, *, receipt=None):
    """Translate known operator-facing text while preserving unfamiliar details."""
    message = str(value)
    label, _, scan_log = message.partition("：")
    if label in _SCAN_LOG_LABELS and scan_log:
        status, _, reason = scan_log.partition("；")
        fields = (label, _reason_details(status), _reason_details(reason))
        return " · ".join(field for field in fields if field)
    if message.startswith(_VISUAL_BOUNDARY_PREFIX):
        details = message.removeprefix(_VISUAL_BOUNDARY_PREFIX).removesuffix(_VISUAL_BOUNDARY_SUFFIX)
        return f"自动任务中断 · 视觉纠偏 · {_reason_details(details)}"
    if not message.startswith(_LEGACY_BOUNDARY_PREFIX):
        return _reason_details(message)
    details = message.removeprefix(_LEGACY_BOUNDARY_PREFIX).removesuffix(_LEGACY_BOUNDARY_SUFFIX)
    stages = [
        _as_dict(_as_dict(event).get("stage_timing")).get("stage")
        for event in _receipt_events(receipt)
    ]
    if "sensor_preflight" in stages and "protocol_action" not in stages:
        return f"启动检查未通过 · {_reason_details(details)}。未发出搬运动作。"
    return f"自动任务中断 · {_reason_details(details)}。未执行后续动作。"


def TaskReport_Reason(receipt):
    result = _as_dict(receipt)
    reason = result.get("reason_zh") or result.get("reason") or result.get("message")
    return TaskReport_Message(reason, receipt=result) if reason else None


def _failure_summary(outcome):
    result = _as_dict(outcome)
    if result.get("ok") is not False:
        return None
    candidate = _as_dict(result.get("start"))
    if candidate.get("ok") is False:
        result = candidate
    reason = TaskReport_Reason(result) or TaskReport_Message(result.get("status") or "任务未完成")
    return {"reason": reason, "diagnostics": TaskReport_Diagnostics(result)}


def _angle_label(name, value):
    if type(value) is int or (type(value) is float and math.isfinite(value)):
        return f"{name} {value:+.2f}°"
    return None


def _distance_label(value):
    if type(value) in (int, float) and math.isfinite(value):
        return f"距码约 {value * 100:.1f} cm"
    return None


def _record_line(prefix, report):
    measurement = _as_dict(report.get("correction_diagnostic"))
    observation = _as_dict(report.get("turn_observation"))
    limit = measurement.get("max_step_deg")
    oversized = "observed_step_oversized" in (measurement.get("failures") or ())
    parts = (
        prefix,
        TaskReport_Message(report["action"]) if report.get("action") else None,
        TaskReport_Message(report["correction"]) if report.get("correction") else None,
        _angle_label("纠偏前", report.get("error_deg", measurement.get("before_error_deg"))),
        _angle_label("剩余", report.get("after_error_deg", measurement.get("after_error_deg"))),
        _angle_label("实测转角", measurement.get("observed_change_deg", observation.get("delta_deg"))),
        f"单次上限 {limit:g}°" if oversized and type(limit) in (int, float) and math.isfinite(limit) else None,
        TaskReport_Reason(report),
    )
    return " · ".join(str(part) for part in parts if part)


def _tag5_pose_details(pose):
    details = []
    distance = pose.get("distance_m")
    if type(distance) in (int, float) and math.isfinite(distance):
        details.append(f"距码约 {distance * 100:.1f} cm")
    angle = _angle_label("航向", pose.get("heading_deg"))
    if angle:
        details.append(angle)
    lateral = pose.get("lateral_error_m")
    if type(lateral) in (int, float) and math.isfinite(lateral):
        details.append(f"横偏 {lateral * 100:+.1f} cm")
    bounds = pose.get("projected_lateral_bounds_m")
    if (isinstance(bounds, (list, tuple)) and len(bounds) == 2
            and all(type(value) in (int, float) and math.isfinite(value) for value in bounds)):
        details.append(f"预计横偏 {bounds[0] * 100:+.1f} 至 {bounds[1] * 100:+.1f} cm")
    return details


def _tag5_entry_notes(alignment):
    observations = [_as_dict(row) for row in alignment.get("observations") or ()]
    actions = {
        "strafe_left_once": "左大跨", "strafe_right_once": "右大跨",
        "single_small_strafe_left": "左小跨", "single_small_strafe_right": "右小跨",
        "single_micro_left": "小左转", "single_micro_right": "小右转",
        "single_medium_left": "中左转", "single_medium_right": "中右转",
    }
    moves = [actions.get(name.removeprefix("double_"), name)
             for name in alignment.get("completed_actions") or () if isinstance(name, str)]
    lines = []
    checks = [plan for plan in observations if type(plan.get("ready_for_forward")) is bool]
    if len(checks) > 4:
        lines.append(f"Tag 5 入口纠偏 · 共 {len(checks)} 窗，显示首次和最近三次")
        checks = [checks[0], *checks[-3:]]
    for plan in checks:
        details = _tag5_pose_details(plan)
        if plan.get("continued_after_expected_tag_occlusion") is True:
            details.append("沿已核验路线继续")
        else:
            details.append("允许下一段直行" if plan["ready_for_forward"] else "暂缓直行")
        reason = TaskReport_Reason(plan)
        if reason:
            details.append(reason)
        turn_reason = plan.get("turn_reason")
        if turn_reason:
            details.append(TaskReport_Message(turn_reason))
        lines.append(" · ".join(("Tag 5 入口纠偏", *details)))
    if not lines:
        offsets = [row.get("map_y_error_m") for row in observations]
        offsets = [value for value in offsets if type(value) in (int, float) and math.isfinite(value)]
        if offsets:
            readings = " → ".join(f"{value * 100:+.1f} cm" for value in offsets)
            return [f"Tag 5 进场横向估计 · {readings}" + (" · " + "、".join(moves) if moves else "")]
        if not moves:
            reason = TaskReport_Reason(alignment) or "暂无可用位姿"
            lines.append(f"Tag 5 入口纠偏 · {reason}")
    if moves:
        lines.append("Tag 5 入口纠偏 · 已完成 " + "、".join(moves))
    for plan in observations:
        recovery = plan.get("side_recovery_operation")
        if isinstance(recovery, str) and recovery in actions:
            lines.append(f"Tag 5 入口纠偏 · 小跨改善不足，复核余量后改用{actions[recovery]}")
    for raw in alignment.get("visual_turns") or ():
        turn = _as_dict(raw)
        name = turn.get("action")
        if not isinstance(name, str):
            continue
        before = _angle_label("转前", turn.get("before_heading_deg"))
        after = _angle_label("转后", turn.get("after_heading_deg"))
        lines.append(" · ".join(filter(None, ("Tag 5 视觉转向", actions.get(name, name),
                                             before, after or "等待新帧复核"))))
    return lines


def _entry_search_notes(events):
    sweep = next((_as_dict(event["terminal_tag_sweep"]) for event in reversed(events)
                  if "terminal_tag_sweep" in event), {})
    stationary = next((_as_dict(event["terminal_tag_reacquisition"]) for event in reversed(events)
                       if "terminal_tag_reacquisition" in event), {})
    advance = next((_as_dict(event["terminal_tag_forward_reacquisition"]) for event in reversed(events)
                     if "terminal_tag_forward_reacquisition" in event), {})
    observation = _as_dict(stationary.get("last_observation"))
    first_retry = next((_as_dict(event["terminal_stationary_reobserve"]) for event in reversed(events)
                        if "terminal_stationary_reobserve" in event), {})
    if not observation:
        rows = advance.get("observations") or sweep.get("observations") or ()
        if rows:
            observation = _as_dict(rows[-1])
        else:
            observation = first_retry
    lines = []
    if advance.get("completed_actions"):
        steps = 2 * len(advance["completed_actions"])
        lines.append(f"Tag 1 找码 · 已补进 {steps} 步，每两步重扫")
    extent = sweep.get("coverage_deg")
    if (isinstance(extent, (tuple, list)) and len(extent) == 2
            and all(type(angle) in (int, float) and math.isfinite(angle) for angle in extent)):
        lines.append(f"Tag 1 搜码实测 · 相对起始朝向 {extent[0]:+.0f}° 至 {extent[1]:+.0f}°")
    if sweep and first_retry.get("decoded_expected_frames") == 0 and observation.get("accepted_expected_frames", 0):
        lines.append("Tag 1 首窗未读到 · 转动搜码后已确认")
    decoded, accepted = observation.get("decoded_expected_frames"), observation.get("accepted_expected_frames")
    if type(decoded) is int and type(accepted) is int:
        lines.append(f"Tag 1 最近一窗 · 检测器报出 {decoded} 帧 · 程序接受 {accepted} 帧")
    return lines


def _tag_heading_reference_notes(events):
    latest = {}
    labels = {
        "tag_departure_reference_established": "已用视觉更新磁航向参考",
        "reference_unavailable": "磁航向参考待更新",
    }
    for event in events:
        reference_report = _as_dict(event.get("segment_heading_reference"))
        label = labels.get(reference_report.get("status"))
        if label is None:
            continue
        reference = _as_dict(reference_report.get("reference"))
        tag = _as_dict(reference_report.get("tag_departure") or reference.get("tag_departure"))
        tag_id = tag.get("tag_id")
        if type(tag_id) is int:
            latest[tag_id] = (label, _angle_label("视觉航向余差", tag.get("error_deg")))
    return [" · ".join(part for part in (f"Tag {tag_id}", *latest[tag_id]) if part)
            for tag_id in sorted(latest)]


def _pickup_turn_note(check):
    if check.get("heading_check_pending"):
        return "本次航向检查未完成"
    actions = check.get("heading_actions")
    if isinstance(actions, list) and actions:
        names = [TaskReport_Message(name.removeprefix("double_")) for name in actions if isinstance(name, str)]
        if names:
            return "已执行" + "、".join(names)
    count = check.get("correction_actions")
    if type(count) is int:
        if count > 0:
            return f"已执行 {count} 次转向"
        if count == 0:
            return "未发转向"
    return None


def _route_notes(result):
    events = [_as_dict(event) for event in _receipt_events(result)]
    corridor = _as_dict(result.get("corridor_trial"))
    entrance = _as_dict(result.get("entrance_trial") or result.get("next_segment"))
    returned = _as_dict(result.get("return_trial"))
    pickup_exit = _as_dict(result.get("pickup_exit"))
    for event in reversed(events):
        corridor = corridor or _as_dict(event.get("corridor_trial"))
        entrance = entrance or _as_dict(event.get("entrance_trial"))
        returned = returned or _as_dict(event.get("return_trial"))
        pickup_exit = pickup_exit or _as_dict(event.get("pickup_exit"))
        if corridor and entrance and returned and pickup_exit:
            break
    missing = list(corridor.get("missing_checkpoints") or ())
    entrance_miss = _as_dict(entrance.get("checkpoint_not_seen"))
    if entrance_miss and not any(_as_dict(row).get("tag_id") == entrance_miss.get("tag_id") for row in missing):
        missing.insert(0, entrance_miss)
    lines = _entry_search_notes(events)
    host = _as_dict(result.get("client_startup_timing"))
    fields = (("SSH 连接", "ssh_connect_s"), ("启动前状态核对", "device_check_s"),
              ("发令至树莓派接收", "dispatch_to_accept_s"))
    startup = [f"{label} {value:.2f} 秒" for label, key in fields
               if type(value := host.get(key)) in (float, int) and math.isfinite(value) and value >= 0]
    if startup:
        lines.append("上位机启动耗时 · " + " · ".join(startup))
    for item in events:
        turn = _as_dict(item.get("base_placement_turn"))
        if turn:
            basis = {"measured": "实测", "estimated_from_completed_actions": "按动作估计",
                     "no_turn": "未转身"}.get(turn.get("basis"), "回执")
            target = _angle_label("目标", turn.get("requested_deg"))
            applied = _angle_label(basis, turn.get("applied_deg"))
            lines.append(" · ".join(part for part in ("大本营放物转角", target, applied) if part))
    entry_fallback = entrance.get("entry_fallback")
    if entry_fallback:
        lines.append(str(entry_fallback))
    retreat = _as_dict(pickup_exit.get("retreat"))
    steps = retreat.get("planned_steps")
    if type(steps) is int and retreat.get("completed_actions") and retreat.get("heading_status") != "pending":
        motion = f"连续后退 {steps} 步" if retreat.get("continuous") else f"分组后退 {steps} 步"
        groups = retreat.get("groups")
        if isinstance(groups,list) and len(groups)>1 and all(type(group) is int for group in groups):
            motion += "（" + "＋".join(str(group) for group in groups) + "）"
        before = _angle_label("纠偏前",retreat.get("heading_before_correction_deg"))
        after = _angle_label("余差",retreat.get("residual_deg"))
        details = (motion, before, _pickup_turn_note(retreat), after or "未取得可用航向")
        lines.append(" · ".join(part for part in details if part))
    if retreat.get("native_six_unavailable") is True:
        lines.append("MCU 未注册连续后退六步 · 已改用两步动作分组后退")
    for group, item in enumerate(pickup_exit.get("lateral_heading_checks") or (), start=1):
        check = _as_dict(item)
        if not check:
            continue
        measured = _angle_label("跨后偏航", check.get("error_after_strafe_deg"))
        residual = _angle_label("余差", check.get("residual_deg")) if not check.get("heading_check_pending") else None
        updated = check.get("residual_deg") != check.get("error_after_strafe_deg")
        direction = "右跨" if "right" in str(check.get("action", "")) else "左跨"
        parts = (f"中转区{direction}第 {group} 组", measured, _pickup_turn_note(check),
                 residual if updated or check.get("correction_actions") else None)
        if not residual:
            parts += ("本组结束时航向未确认",)
        lines.append(" · ".join(part for part in parts if part))
    entry_check = next((_as_dict(event["pickup_exit_entry"]) for event in reversed(events)
                        if "pickup_exit_entry" in event), {})
    if entry_check.get("confirmed_for_forward") is True:
        heading = _angle_label("视觉航向余差",entry_check.get("heading_error_deg"))
        lines.append(" · ".join(part for part in ("Tag 1 已通过进场确认", heading) if part))
    elif entry_check and not entry_fallback:
        lines.append("左跨后正在确认 Tag 1" if entry_check.get("status") == "scanning"
                     else "左跨完成后未确认 Tag 1，继续持物")
    deliveries = result.get("deliveries")
    blocks = result.get("expected_blocks_delivered")
    if type(deliveries) is int and deliveries > 0 and type(blocks) is int:
        lines.append(f"已完成 {deliveries} 趟放物动作 · 预计合计 {blocks} 块")
    return_turns = {"empty_micro_left": "空载小左转", "empty_micro_right": "空载小右转",
                    "empty_left_turn": "空载左转", "empty_right_turn": "空载右转"}
    for raw_leg in returned.get("legs") or ():
        leg = _as_dict(raw_leg)
        tag = leg.get("expected_id")
        heading = _as_dict(leg.get("empty_heading"))
        if type(tag) is not int:
            continue
        retreat_groups = _as_dict(leg.get("backward_search")).get("groups") or ()
        completed_retreats = sum(type(_as_dict(group).get("completed_ros_ns")) is int for group in retreat_groups)
        if completed_retreats:
            lines.append(f"返程 Tag {tag} 找码 · 已空载后退 {2 * completed_retreats} 步")
        readings = heading.get("observations") or ()
        if readings:
            turns = [return_turns[op] for op in heading.get("actions") or () if op in return_turns]
            pieces = [f"返程 Tag {tag} 已识别", _angle_label("视觉航向", _as_dict(readings[0]).get("error_deg"))]
            if turns:
                pieces.append("、".join(turns))
                if len(readings) > len(turns):
                    pieces.append(_angle_label("转后余差", _as_dict(readings[-1]).get("error_deg")))
            else:
                pieces.append(heading.get("reason_zh") or "本窗未执行转向")
            lines.append(" · ".join(piece for piece in pieces if piece))
        correction = _as_dict(leg.get("empty_departure_correction"))
        positions = correction.get("observations") or ()
        if positions:
            key = "body_lateral_m" if tag == 5 else "body_depth_m"
            measurements = []
            for row in positions:
                interval = _as_dict(row).get(key)
                if (isinstance(interval, (tuple, list)) and len(interval) == 2 and
                        all(type(v) in (int, float) and math.isfinite(v) for v in interval)):
                    measurements.append(f"{sum(interval) * 50:+.1f}" if tag == 5 else f"{sum(interval) * 50:.1f}")
            parts = [f"返程 Tag {tag} 出发位置"]
            if measurements:
                label = "机身横偏（右为正）" if tag == 5 else "距码"
                parts.append(label + " " + " → ".join(measurements) + " cm")
            translations = {"empty_strafe_left":"左跨", "empty_strafe_right":"右跨",
                            "empty_forward2":"前进两步", "empty_backward2":"后退两步"}
            actions = [translations[op] for op in correction.get("actions", ()) if op in translations]
            if actions:
                parts.append("已执行" + "、".join(actions))
            if correction.get("status") == "within_departure_band":
                parts.append("已在出发容差内")
            elif actions and correction.get("status") in ("marker_lost_after_translation", "no_current_pose"):
                parts.append("移动后未确认新位姿")
            lines.append(" · ".join(parts))
    for missing_tag in returned.get("missing_checkpoints") or ():
        tag = _as_dict(missing_tag).get("tag_id")
        if type(tag) is int:
            lines.append(f"返程漏扫 Tag {tag} · 已按估算路线继续")
    if returned.get("checkpoint") == 1:
        lines.append("返程已确认 Tag 1")
    elif returned.get("estimated_pickup_anchor_tag") == 1:
        lines.append("返程 Tag 1 未确认 · 已按预设路线前往接驳点")
    station = _as_dict(returned.get("handoff_station"))
    if station.get("status") == "taught_position":
        lines.append("返程接驳站 · Tag 1 与示教站位相符，已结束进近")
    elif station.get("status") == "nominal_endpoint":
        lines.append("返程接驳站 · 已走完接驳动作，Tag 1 示教站位待确认")
    nominal, planned = entrance.get("nominal_target_steps"), entrance.get("planned_steps")
    initial = entrance.get("initial_estimated_steps",planned)
    if type(nominal) is int and type(initial) is int and nominal != initial:
        lines.append(f"Tag 1 前后补偿 · 已合入直行计划 {nominal} → {initial} 步")
    for sighting in entrance.get("entry_boundary_observations") or ():
        sighting = _as_dict(sighting)
        at, before, remaining = (sighting.get(key) for key in
                                 ("after_steps","previous_remaining_steps","remaining_steps"))
        if not all(type(value) is int for value in (at,before,remaining)) or before == remaining:
            continue
        details = [f"Tag 2 提前读码 · 第 {at} 步 · 剩余直行 {before} → {remaining} 步"]
        frames = sighting.get("advisory_frames")
        if type(frames) is int and frames > 0 and sighting.get("confirmed_for_strafe") is False:
            details.append(f"{frames} 帧提示已靠近，原地补帧后再判断侧跨")
        distance = sighting.get("distance_m")
        if type(distance) in (int,float) and math.isfinite(distance):
            details.append(f"距码约 {distance*100:.1f} cm")
        angle = _angle_label("航向",sighting.get("heading_deg"))
        if angle:
            details.append(angle)
        lines.append(" · ".join(details))
    if entrance.get("reused_boundary_frames") is True:
        lines.append("Tag 2 已复用当前动作结束后的有效帧")
    completed = entrance.get("completed_steps")
    if type(planned) is int and type(completed) is int and entrance.get("next_stage_started") is True:
        lines.append(f"Tag 1 → 2 · MCU 动作回执 {completed}/{planned} 个名义直行步")
    lines.extend(_tag_heading_reference_notes(events))
    pickup = _as_dict(result.get("pickup"))
    pickup_events = [event["pickup"] for event in reversed(events) if isinstance(event.get("pickup"), dict)]
    latest_pickup = pickup or (pickup_events[0] if pickup_events else {})
    if not pickup and latest_pickup.get("status") == "no_pickup":
        for event in pickup_events:
            if event.get("grip_count") in (1, 2):
                break
            if MissionPickup_DoubleLoadWaitLabel(event):
                latest_pickup = event
                break
    if wait := MissionPickup_DoubleLoadWaitLabel(latest_pickup):
        lines.append(wait)
    elif latest_pickup.get("reason_code") == "staged_block_layout_ambiguous":
        lines.append(TaskReport_Reason(latest_pickup))
    if not pickup:
        pickup = next((_as_dict(event.get("pickup")) for event in reversed(events)
                       if _as_dict(event.get("pickup")).get("grip_count") in (1, 2)), {})
    if pickup.get("staged_default_pair") is True or pickup.get("default_pair_station") == "tag1_handoff":
        station = "返程接驳区" if pickup.get("default_pair_station") == "tag1_handoff" else "人工摆放区"
        reason = pickup.get("vision_reason")
        if reason in ("empty","unknown"):
            reason = "识别窗内没有看到方块" if reason == "empty" else "图像不足以确认方块数量"
        detail = f" · {TaskReport_Message(reason)}" if reason else ""
        lines.append(f"{station}未确认方块数量{detail} · 已执行默认双抱，实物件数待确认")
    elif pickup.get("grip_count") in (1, 2) and type(pickup.get("observed_count")) is int:
        grip = "单抓" if pickup["grip_count"] == 1 else "双抱底排两块"
        if pickup.get("stacked_triple") is True and pickup.get("planned_count") == 3:
            grip = "双抱底排、预计带走三块"
        lines.append(f"胸部相机识别到 {pickup['observed_count']} 块 · 已执行{grip}")
    visual = _as_dict(result.get("entry_alignment"))
    if not visual:
        for event in reversed(events):
            visual = _as_dict(_as_dict(event.get("visual_checkpoint_failure")).get("report"))
            if not visual:
                visual = _as_dict(event.get("AR_checkpoint"))
            if visual:
                break
    for step in visual.get("correction_steps") or (visual,):
        disagreement = _as_dict(_as_dict(step).get("heading_disagreement"))
        tag = disagreement.get("tag_id")
        if type(tag) is int:
            source = {
                "trusted_tag_heading": "以 Tag 航向为准",
                "route_advisory": "原地复核未确认，保留余差按原路线接续",
            }.get(disagreement.get("selected_source"))
            parts = (f"Tag {tag} 转角对照",
                     _angle_label("视觉", disagreement.get("visual_delta_deg")),
                     _angle_label("磁力计", disagreement.get("magnetic_delta_deg")),
                     _angle_label("相差", disagreement.get("delta_disagreement_deg")), source)
            lines.append(" · ".join(part for part in parts if part))
    for entry in missing:
        entry = _as_dict(entry)
        if entry.get("recovered_before_departure") or entry.get("held_before_departure"):
            continue
        tag = entry.get("tag_id")
        policy = entry.get("policy")
        if type(tag) is int and policy in ("continue_estimated", "hold_load_for_placement"):
            nominal = policy == "continue_estimated" or tag == 5 and corridor.get("release_protocol_completed") is True
            followup = "已记录，按预定路线继续" if nominal else "保持持物，放物区未确认"
            cause = entry.get("cause")
            if cause in ("temporary_camera_gap","camera_stream_unhealthy"):
                summary = f"Tag {tag} 扫码流暂未恢复"
            elif cause == "insufficient_frames":
                summary = f"Tag {tag} 已解码，有效帧不足"
            else:
                summary = f"未扫到 Tag {tag}"
            lines.append(f"{summary} · {followup}")
    for fallback in corridor.get("route_fallbacks") or ():
        tag = _as_dict(fallback).get("tag_id")
        if type(tag) is int:
            detail = TaskReport_Message(_as_dict(fallback).get("detail") or "未取得稳定的视觉位姿")
            line = f"Tag {tag} · {detail} · 已转入该段剩余的预设动作"
            if line not in lines:
                lines.append(line)
    for leg in corridor.get("legs") or ():
        leg = _as_dict(leg)
        start, end = leg.get("from_tag"), leg.get("expected_id")
        if type(start) is not int or type(end) is not int:
            continue
        recheck = _as_dict(leg.get("tag2_departure_recheck"))
        far_reframe = _as_dict(leg.get("tag2_far_reframe"))
        reframe = far_reframe or _as_dict(leg.get("tag2_close_reframe"))
        if reframe.get("completed_steps") == 2:
            distance = reframe.get("before_distance_m")
            depth = f"距码约 {distance*100:.1f} cm · " if type(distance) in (int,float) and math.isfinite(distance) else ""
            outcome = "已取得新帧" if reframe.get("status") == "recovered" else "复扫仍待确认"
            kind, move = ("偏远帧少","前探") if far_reframe else ("距离偏近","后退")
            lines.append(f"Tag 2 {kind} · {depth}已{move}两步 · {outcome}")
        if recheck.get("status") == "recovered":
            if not reframe:
                lines.append("右跨前已重扫 Tag 2 并取得新帧")
        elif recheck.get("status") == "distance_unconfirmed":
            sighting = _as_dict(recheck.get("boundary_sighting"))
            seen = sighting.get("accepted_frames")
            hint = f"动作结束时读到 {seen} 帧 · " if type(seen) is int else ""
            lines.append(f"Tag 2 · {hint}右跨起点的前后距离仍未确认")
        trim = _as_dict(leg.get("tag2_forward_trim"))
        for turn in trim.get("heading_recovery") or ():
            turn = _as_dict(turn)
            before = _angle_label("纠偏前", turn.get("before_error_deg"))
            after = _angle_label("复测余差", turn.get("after_error_deg"))
            if before:
                lines.append(" · ".join(part for part in ("Tag 2 视觉纠偏", before, after) if part))
        batches = trim.get("approach_batches")
        planned = trim.get("planned_steps")
        approach_distance = trim.get("approach_distance_m")
        if type(planned) is int and type(approach_distance) in (int,float) and math.isfinite(approach_distance):
            if planned:
                detail = f"Tag 2 估算进近 · MCU 回执 {trim.get('completed_steps',0)}/{planned} 步"
                bounds = trim.get("estimated_landing_m")
                if (isinstance(bounds,(list,tuple)) and len(bounds)==2
                        and all(type(value) in (int,float) and math.isfinite(value) for value in bounds)):
                    detail += f" · 预计落点距码 {bounds[0]*100:.1f}–{bounds[1]*100:.1f} cm"
            else:
                detail = f"Tag 2 距码约 {approach_distance*100:.1f} cm"
            if trim.get("ready_to_strafe") is False:
                reason = TaskReport_Reason({"reason": trim.get("last_approach_reason")})
                detail += f" · {reason}" if reason else " · 右跨起点待确认"
            lines.append(detail)
        elif isinstance(batches,list):
            detail = f"Tag {start} 偏远"
            if batches:
                detail += f" · 已分组前进 {2*len(batches)} 步"
            elif trim.get("protocol_completed") is not True:
                detail += " · 尚未发出补进动作"
            final = trim.get("final_distance_m")
            if type(final) in (int,float) and math.isfinite(final):
                detail += f" · 复测距码约 {final*100:.1f} cm"
            if trim.get("approach_confirmed") is not True:
                reason = TaskReport_Reason({"reason": trim.get("last_approach_reason")})
                detail += f" · {reason}" if reason else " · 侧跨起点待确认"
            lines.append(detail)
        elif trim.get("protocol_completed") is True:
            move = {"carry_forward2": "前进", "cmd0_carry_backward2": "后退"}.get(trim.get("operation"))
            if move:
                detail = f"Tag {start} 前后补偿 · {move}两步后再侧跨"
                before, after = trim.get("distance_m"), trim.get("after_distance_m")
                if type(before) in (int, float) and math.isfinite(before):
                    detail += f" · 补偿前约 {before * 100:.1f} cm"
                    if type(after) in (int, float) and math.isfinite(after):
                        detail += f" · 补偿后约 {after * 100:.1f} cm"
                lines.append(detail)
        elif trim.get("reason") == "Tag 2 two-step longitudinal reference unverified":
            lines.append("Tag 2 前后补偿待标定 · 已按原动作侧跨")
        tag4_approach = [_as_dict(record) for record in leg.get("tag4_approach") or ()]
        tag4_recovery = _as_dict(leg.get("tag4_partial_recovery"))
        disruption = _as_dict(leg.get("tag4_forward_disruption"))
        for plan in tag4_approach:
            distance, steps = _distance_label(plan.get("distance_m")), plan.get("steps")
            if plan.get("distance_verified") is True and distance and type(steps) is int:
                lines.append(f"Tag 4 进近 · {distance} · 本组计划 {steps} 步")
        latest = tag4_approach[-1] if tag4_approach else {}
        partial = _as_dict(latest.get("partial_pose"))
        samples = partial.get("samples")
        if not tag4_recovery and not disruption and type(samples) is int and samples == 1:
            state = "复扫时已过期" if partial.get("latest_fresh") is False else "未确认航向"
            lines.append(f"Tag 4 进近 · 仅有 1 帧位姿，{state}")
        elif (not tag4_recovery and not disruption
                and latest.get("partial_advice") == "outside_heading_or_lateral_range"
                and partial.get("consistent") is True and partial.get("latest_fresh") is True):
            frames = f"{samples} 帧位姿一致" if type(samples) is int and samples >= 2 else None
            parts = ("Tag 4 进近", frames, _distance_label(latest.get("distance_m")),
                     _angle_label("视觉航向", partial.get("heading_deg")), "位姿超出本组前进容差")
            lines.append(" · ".join(part for part in parts if part))
        fallback = leg.get("tag4_magnetic_fallback")
        records = fallback if isinstance(fallback, list) else [fallback]
        for record in records:
            record = _as_dict(record)
            if not record:
                continue
            completed, planned = record.get("completed_steps"), record.get("planned_steps")
            progress = (f"直行已完成 {completed}/{planned} 步"
                        if type(completed) is int and type(planned) is int and 0 <= completed <= planned else None)
            steps = record.get("next_steps")
            parts = ("Tag 4 磁航向续行", _angle_label("磁航向余差", record.get("heading_deg")),
                     progress, f"下组计划 {steps} 步" if type(steps) is int and steps > 0 else None,
                     TaskReport_Reason(record))
            lines.append(" · ".join(part for part in parts if part))
        if tag4_recovery:
            completed = tag4_recovery.get("completed_steps")
            parts = ("Tag 4 进近恢复", _distance_label(tag4_recovery.get("distance_m")),
                     _angle_label("视觉航向", tag4_recovery.get("heading_deg")),
                     f"已前进 {completed} 步" if type(completed) is int and completed > 0 else None,
                     TaskReport_Reason(tag4_recovery))
            lines.append(" · ".join(part for part in parts if part))
        tag4_trim = _as_dict(leg.get("tag4_forward_trim"))
        tag4_batches = tag4_trim.get("approach_batches")
        if isinstance(tag4_batches,list):
            state = "已确认左跨起点" if tag4_trim.get("approach_confirmed") is True else "左跨起点待确认"
            detail = f"Tag 4 左跨前复核 · 已分组前进 {2*len(tag4_batches)} 步"
            final = tag4_trim.get("after_distance_m")
            if type(final) in (int,float) and math.isfinite(final):
                detail += f" · 复测距码约 {final*100:.1f} cm"
            lines.append(f"{detail} · {state}")
        split = _as_dict(leg.get("tag4_approach_split"))
        if split.get("reason_zh"):
            lines.append(split["reason_zh"])
            if split.get("recheck_reason_zh"):
                lines.append(split["recheck_reason_zh"])
        if leg.get("tag4_partial_approach"):
            lines.append("Tag 4 部分帧显示已经靠近 · 省去余下前进并复扫")
        tag4_trim = _as_dict(leg.get("tag4_forward_trim"))
        action = tag4_trim.get("completed_action")
        detail = {"carry_forward2": "前进两步", "carry_forward4": "前进四步",
                  "cmd0_carry_backward2": "后退两步"}.get(action)
        if detail:
            message = f"Tag 4 左跨前 · 已{detail}并重扫"
            before, after = tag4_trim.get("distance_m"), tag4_trim.get("after_distance_m")
            if type(before) in (int,float) and math.isfinite(before):
                message += f" · 补偿前约 {before*100:.1f} cm"
            if type(after) in (int,float) and math.isfinite(after):
                message += f" · 补偿后约 {after*100:.1f} cm"
            lines.append(message)
        midpoint = _as_dict(leg.get("tag3_midpoint_observation"))
        if midpoint.get("status") == "checkpoint_detected" and leg.get("detected") is False:
            strides = midpoint.get("right_strides", 3)
            lines.append(f"右跨 {strides} 步处读到 Tag 3 · 完成右跨后未再次确认")
        tag3_recheck = _as_dict(leg.get("tag3_recheck_after_yaw"))
        if tag3_recheck:
            outcome = ("纠偏后已重新读到" if tag3_recheck.get("detected") else
                       "纠偏后仍未读到 · 下一段改为短组右跨")
            lines.append(f"Tag 3 · {outcome}")
        midpoint = _as_dict(leg.get("tag5_midpoint"))
        probe = _as_dict(leg.get("tag5_partial_probe"))
        if probe.get("completed_steps") == 2:
            outcome = "已重新确认" if probe.get("status") == "checkpoint_detected" else "入口仍待确认"
            lines.append(f"Tag 5 偏远帧少 · 已走进场计划中的两步 · {outcome}")
        if midpoint.get("decision") == "replace_remaining_strafes":
            pose = _as_dict(midpoint.get("plan")) or midpoint
            lines.append(" · ".join(("左跨中途扫到 Tag 5，转入入口纠偏", *_tag5_pose_details(pose))))
        elif midpoint.get("decision") == "keep_continuous_left_group":
            group = {"cmd0_strafe_left2": 2, "cmd0_strafe_left3": 3}.get(midpoint.get("continuous_action"))
            if group:
                lines.append(" · ".join((f"左跨中途扫到 Tag 5，继续一组左跨 {group} 步",
                                         *_tag5_pose_details(midpoint))))
        quick = leg.get("quick_scan_windows") or ()
        if end == 3 and quick:
            decoded = sum(_as_dict(window).get("decoded_expected_frames", 0) or 0 for window in quick)
            accepted = sum(_as_dict(window).get("accepted_expected_frames", 0) or 0 for window in quick)
            seen = sum(_as_dict(window).get("decoded_since_boundary", _as_dict(window).get("decoded_expected_frames",0)) or 0
                       for window in quick)
            expired = f" · 其中 {seen-decoded} 帧超出时效" if seen > decoded else ""
            lines.append(f"Tag 3 终点短扫 · 检测器报出 {seen} 帧{expired} · 程序接受 {accepted} 帧")
        side = _as_dict(leg.get("side_plan"))
        if side.get("status") == "tag2_entry_lateral_fused":
            lines.append(TaskReport_Reason(side))
        handoff = _as_dict(leg.get("tag3_straight_heading"))
        if handoff:
            turn = _as_dict(handoff.get("turn")).get("action")
            previous = handoff.get("recent_turn")
            observed = _angle_label("实测", handoff.get("recent_turn_delta_deg"))
            repeated = (f"上次{TaskReport_Message(previous)}{observed}，本次略过"
                        if not turn and isinstance(previous,str) and observed else None)
            parts = ("Tag 3 侧跨转直行", _angle_label("转前视觉", handoff.get("before_error_deg")), repeated,
                     TaskReport_Message(turn) if isinstance(turn, str) else None,
                     _angle_label("复测余差", handoff.get("residual_deg"))
                     if handoff.get("post_turn_pose_confirmed") is True else None,
                     "先前进两步再看码" if handoff.get("short_first_forward") is True else None)
            if any(parts[1:]):
                lines.append(" · ".join(part for part in parts if part))
        short_groups = _as_dict(leg.get("tag2_cautious_strafes"))
        if short_groups:
            size = short_groups.get("max_group")
            observed = _angle_label("出发视觉航向", short_groups.get("visual_heading_deg"))
            groups = short_groups.get("groups")
            grouping = ("先右跨 2 步，航向复查后连续右跨 3 步" if groups == [2, 3]
                        else f"每 {size} 跨复查航向")
            lines.append(" · ".join(filter(None, (f"Tag 2 → 3 {grouping}", observed))))
        hint = _as_dict(side.get("trial_lateral_hint"))
        actual, nominal = side.get("large_count"), side.get("nominal_count")
        if (start == 3 and end == 4 and hint.get("tag_id") in (2,3)
                and side.get("status") == "configured_unverified_pose"
                and type(actual) is int and type(nominal) is int and actual < nominal):
            lines.append(f"Tag 3 → 4 · 根据 Tag {hint['tag_id']} 的可信横向读数减少一步大右跨")
        if side.get("status") in ("tag3_left_bias_fused", "tag3_right_bias_fused",
                                   "tag3_unverified_extra_omitted", "tag3_midpoint_full_right_group"):
            lines.append(TaskReport_Reason(side))
        elif side.get("status") in ("configured_unverified_pose", "field_baseline") and type(side.get("count")) is int:
            direction = {"strafe_left_once": "左", "strafe_right_once": "右"}.get(side.get("operation"))
            if direction:
                if side.get("small_operation") and type(side.get("large_count")) is int:
                    lines.append(f"Tag {start} → {end} · 按试跑路线{direction}大跨 {side['large_count']} 步，再小跨 1 步")
                elif side.get("status") == "field_baseline" and side.get("large_count") == side["count"]:
                    lines.append(f"Tag {start} → {end} · 按基线路线{direction}大跨 {side['count']} 步")
                else:
                    lines.append(f"Tag {start} → {end} · 位姿未确认，按试跑距离{direction}跨 {side['count']} 步")
        if disruption:
            recovery = "已重新确认通道" if disruption.get("reacquired") else "通道仍待确认"
            turn = _as_dict(disruption.get("visual_turn"))
            action = turn.get("action")
            parts = (TaskReport_Reason(disruption), _distance_label(disruption.get("evidence_distance_m")),
                     _angle_label("纠偏前", disruption.get("heading_error_deg")),
                     f"已执行{TaskReport_Message(action)}" if isinstance(action, str) and action else None,
                     recovery)
            lines.append(" · ".join(part for part in parts if part))
        departure = _as_dict(leg.get("departure_heading"))
        if departure.get("correction_actions"):
            before = _angle_label("纠偏前", departure.get("error_deg"))
            residual = _angle_label("剩余", departure.get("residual_deg"))
            if before and residual:
                lines.append(f"Tag {start} 出发前航向 · {before} · {residual}")
        for check in leg.get("lateral_heading_checks") or ():
            check = _as_dict(check)
            if start == 4 and end == 5:
                parts = (_angle_label("本组偏航", check.get("drift_deg")),
                         _angle_label("剩余", check.get("residual_deg")))
                if any(parts):
                    lines.append(" · ".join(("Tag 4 → 5 左跨", *(part for part in parts if part))))
            if check.get("compensation_status") != "applied":
                continue
            direction = {"carry_forward2": "前进", "cmd0_carry_backward2": "后退"}.get(check.get("compensation_action"))
            step = direction + ("四步" if check.get("compensation_steps") == 4 else "两步") if direction else None
            drift = _angle_label("侧跨偏航", check.get("drift_deg"))
            if step and drift:
                if check.get("compensation_order") == "before_heading_correction":
                    motion = f"先{step}腾出空间，再复测航向"
                else:
                    motion = f"纠偏完成后{step}"
                if check.get("compensation_requested_steps") == 4 and check.get("compensation_steps") == 2:
                    motion += " · 复测未满足第二组让位条件"
                lines.append(f"Tag {start} → {end} · {drift} · {motion}")
    lateral = _as_dict(_as_dict(corridor.get("delivery")).get("entry_lateral_alignment"))
    if lateral:
        lines.extend(_tag5_entry_notes(lateral))
    return tuple(lines)


def TaskReport_Diagnostics(result):
    """Return route outcomes and, on failure, the latest diagnostic details."""
    result = _as_dict(result)
    route_notes = _route_notes(result)
    if result.get("ok") is not False:
        return route_notes
    link_detail = ()
    if result.get("status") == "start_unconfirmed":
        details = []
        if result.get("accepted") is True:
            details.append("已收到树莓派接收回执，任务结果待确认。")
        observed = _as_dict(result.get("last_observed_status"))
        phase = _as_dict(observed.get("mission_progress")).get("phase")
        if observed.get("task_active") is True and isinstance(phase, str):
            progress = MissionProgress()
            progress.observe(observed)
            details.append(f"失联前最近一次状态 · {progress.text}")
        link_detail = tuple(details)
    if result.get("reason") == "MCU handshake missing/stale":
        link = _as_dict(result.get("diagnostic"))
        age = link.get("mcu_last_contact_age_s")
        if type(age) in (int, float) and math.isfinite(age) and age >= 0:
            source = {"request": "请求", "complete": "完成回执"}.get(str(link.get("mcu_last_contact_source")))
            contact = f"（{source}）" if source else ""
            link_detail = (f"MCU 通信 · 距上次收到消息{contact} {age:.2f} 秒",)
    scan = _as_dict(result.get("scan_result"))
    checkpoint = _as_dict(scan.get("navigation_checkpoint"))
    scan_reason = TaskReport_Reason(checkpoint) or TaskReport_Reason(scan)
    scan_detail = route_notes + link_detail + ((f"入口扫码 · {scan_reason}",) if scan_reason else ())
    events = _receipt_events(result)
    for event in reversed(events):
        fall = _as_dict(_as_dict(event).get("fall_diagnostic"))
        status = fall.get("status")
        if status == "persistent_magnetic_anomaly_with_scene_change":
            scan_detail += ("磁场持续异常，画面也有变化 · 姿态待确认",)
            break
        if status == "persistent_magnetic_anomaly":
            scan_detail += ("磁场持续异常 · 姿态待确认",)
            break
    last_action = None
    for event in reversed(events):
        item = _as_dict(event)
        timing = _as_dict(item.get("stage_timing"))
        if last_action is None and timing.get("stage") == "protocol_action":
            last_action = timing
        visual = _as_dict(item.get("visual_checkpoint_failure"))
        detail = _as_dict(item.get("post_action_heading"))
        if visual:
            detail = _as_dict(visual.get("report")) or visual
        if detail:
            lines = [_record_line("最近视觉诊断" if visual else "最近航向诊断", detail)]
            if detail.get("action") == "planned_turn_complete":
                coarse = next((
                    prediction for row in reversed(events)
                    if (prediction := _as_dict(_as_dict(row).get("coarse_turn_prediction")))
                    and prediction.get("applied_to_route") is True
                    and prediction.get("use_original_macro") is False
                ), {})
                upper = _as_dict(coarse.get("estimate")).get("upper_abs_deg")
                allowed = coarse.get("allowed_upper_abs_deg")
                if (type(upper) in (int, float) and type(allowed) in (int, float)
                        and math.isfinite(upper) and math.isfinite(allowed) and upper > allowed):
                    lines.append(f"已切换到小幅转向 · 大转预测上界 {upper:g}° · 保留航向余量后允许 {allowed:g}°")
            steps = detail.get("correction_steps")
            if isinstance(steps, list):
                for number, step in enumerate(steps, start=1):
                    if isinstance(step, dict):
                        lines.append(_record_line(f"纠偏第 {number} 步", step))
            prediction = _as_dict(detail.get("motion_prediction"))
            rejected = _as_dict(prediction.get("rejected"))
            if rejected:
                lines.append("转角候选被排除 · " + "、".join(
                    f"{name} {TaskReport_Message(reason)}" for name, reason in rejected.items()
                ))
            budget = _as_dict(detail.get("budget"))
            if budget.get("used_actions") is not None and budget.get("max_actions") is not None:
                lines.append(f"纠偏次数 · 已用 {budget['used_actions']} / {budget['max_actions']}")
            return scan_detail + tuple(lines)
    if last_action and last_action.get("action"):
        complete = "收到完成回执" if last_action.get("protocol_completed") is True else "未确认完成"
        return scan_detail + (f"最近动作 · {last_action['action']} · {complete}",)
    return scan_detail
