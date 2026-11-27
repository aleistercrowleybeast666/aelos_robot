"""PySide6 control console for AELOS R3."""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
from queue import Empty, Queue
import subprocess
import sys
from threading import Event, Lock, Thread
import time

from PySide6.QtCore import QTimer, QUrl
from PySide6.QtGui import QDesktopServices, QImage
from PySide6.QtWidgets import QApplication, QFileDialog, QMainWindow, QMessageBox

from . import settings, usb_mode
from .chest_camera import ChestCamera
from .control_api import (EMPTY_RESET_REQUIRED_MESSAGE, RobotApi, RobotControlUncertainError,
                          RobotStart_EmptyResetStatus)
from .deploy import SshLinkError, operate
from .mission_progress import (Command0_LastPassedTag, Command2_LastReturnTag, MissionDelivery_Label,
                               MissionDelivery_Tag5HoldLabel, MissionProgress)
from .qt_view import ControlView
from .report_dispatch import ReportDispatcher
from .report_window import ReportWindow
from .task_report import (TaskReport_Diagnostics, TaskReport_Directory, TaskReport_Message,
                          TaskReport_Reason, TaskReport_Save, TaskReport_SetDirectory)


_STATUS_INTERVAL_MS = 3000
_CAMERA_STALE_S = 2.0
_CAMERA_OFFLINE = "机器人离线，预览已关闭"
_CONNECT_RETRY_DELAY_S = 1.0
_MCU_RECONNECT_TIMEOUT_S = 60.0
_TASK_NAMES = (
    "单程搬运原始物资",
    "单程搬运转运物资",
    "连续搬运转运物资",
)
_RESET_EMPTY = "stand"
_RESET_EMPTY_LABEL = "恢复空载站姿"
_COMMAND0_RESULT_TEXT = {
    "stage2_checkpoint_reached": "已扫到 Tag 2，持物试跑段完成",
    "stage2_trial_unconfirmed": "Tag 2 试跑待确认",
    "stage3_checkpoint_reached": "已扫到 Tag 5，持物通道试跑完成",
    "stage3_trial_unconfirmed": "通道试跑待确认",
}
_ACTION_NAMES = {
    "check": "检查部署连接",
    "deploy": "部署树莓派程序",
    "rollback": "回滚树莓派程序",
}


@dataclass(frozen=True)
class _Request:
    values: dict
    password: str | None

    @property
    def target(self):
        return f"{self.values['host']}:{self.values['port']}"

    @property
    def deploy_target(self):
        return self.target.lower(), self.values["username"], self.values["remote_path"]

    @property
    def deployment_origin(self):
        if self.values["deployment_source"] == "remote":
            return f"远程部署包\n{self.values['remote_url']}"
        return f"本地部署包\n{self.values['local_path']}"


@dataclass
class _PendingMcuReadback:
    target: tuple
    receipt: Path
    drive: str
    reset_confirmed: bool = False


@dataclass
class _Motion:
    request: _Request
    command: int | str
    after_update: bool
    cancelled: Event
    last_observed_status: dict | None = None


@dataclass(frozen=True)
class _MotionCompletion:
    outcome: dict | Exception
    diagnostics: tuple[str, ...] = ()
    report_path: str | None = None
    report_error: str | None = None


@dataclass(frozen=True)
class _Probe:
    request: _Request
    action: str
    quiet: bool
    uncertain: _Motion | None
    stop_wait: _Request | None
    motion: _Motion | None = None


def _robot_api(request, log=None):
    return RobotApi(request.values, request.password, log=log)


def _chest_frames(request, stop):
    return ChestCamera(request.values, request.password).frames(stop)


def _safe_text(value, password):
    text = TaskReport_Message(value)
    return text.replace(password, "[密码已隐藏]") if password else text


def _motion_result(task, outcome):
    if isinstance(outcome, RobotControlUncertainError):
        result = outcome.as_result()
    elif isinstance(outcome, dict):
        start = outcome.get("start") if task.after_update else None
        result = start if isinstance(start, dict) else outcome
    else:
        return None
    observed = task.last_observed_status
    if observed is not None and result.get("status") == "start_unconfirmed":
        return dict(result, last_observed_status=observed)
    return result


def _robot_idle(status):
    owner = status.get("owner") or {}
    return (status.get("task_active") is False and owner.get("in_flight") is False
            and owner.get("mode") == "manual")


def _wait_for_mcu_owner(request, cancelled, log):
    """Wait for the rebooted control service before starting the write transaction."""
    api = _robot_api(request)
    deadline = time.monotonic() + _MCU_RECONNECT_TIMEOUT_S
    log("正在等待机器人重新上线和 MCU 握手…")
    while not cancelled.is_set():
        try:
            status = api.status()
        except SshLinkError:
            pass
        else:
            owner = status.get("owner") or {}
            if status.get("ok") is True and owner.get("robot_id") == 3:
                if status.get("task_active") or owner.get("mode") == "auto" or owner.get("in_flight"):
                    raise RuntimeError("机器人正在运行，MCU 核验回执已保留。")
                if _robot_idle(status) and status.get("prepared") is True and owner.get("mcu_ready") is True:
                    return True
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise RuntimeError("等待机器人上线超时。MCU 核验回执已保留，联网后重新点击部署。")
        cancelled.wait(min(1.0, remaining))
    return False


class _ControlWindow(QMainWindow):
    def __init__(self, values):
        super().__init__()
        self.setWindowTitle("AELOS R3 Control")
        self.resize(1170, 740)
        self.view = ControlView(values)
        self.setCentralWidget(self.view)
        self._events = Queue()
        self._session = None
        self._link = "disconnected"
        self._link_confirmed = False
        self._last_robot = {}
        self._motion = None
        self._uncertain = None
        self._stop_request = None
        self._stop_wait = None
        self._stop_retry = None
        self._probe = None
        self._maintenance = None
        self._mcu_receipt = None
        self._deployment_cancel = None
        self._progress = MissionProgress()
        self._camera = None
        self._camera_on_connect = False
        self._frame_lock = Lock()
        self._frame = None
        self._last_frame_at = None
        self._latest_report = None
        self._report_dialog = None
        self.report_dispatch = ReportDispatcher(self)
        self.report_dispatch.message.connect(self._log)
        self.report_dispatch.ReportDispatch_Retry()

        view = self.view
        view.connection_button.clicked.connect(self._toggle_connection)
        view.camera_button.clicked.connect(self._toggle_camera)
        view.stop_button.clicked.connect(self._request_stop)
        view.reset_empty_button.clicked.connect(lambda: self._start_motion(_RESET_EMPTY))
        view.scan_button.clicked.connect(self._request_scan)
        view.save_button.clicked.connect(self._save_settings)
        view.deployment_source.currentIndexChanged.connect(self._update_buttons)
        view.route_edit_button.clicked.connect(self._edit_action_chain)
        view.clear_log_button.clicked.connect(self._clear_log)
        view.open_reports_button.clicked.connect(self._open_report_directory)
        view.change_reports_button.clicked.connect(self._change_report_directory)
        view.share_reports_button.clicked.connect(self._show_report_sharing)
        view.update_start_button.clicked.connect(
            lambda: self._start_motion(view.update_task.currentData(), after_update=True)
        )
        for command, button in view.task_buttons.items():
            button.clicked.connect(lambda checked=False, command=command: self._start_motion(command))
        for action, button in view.pi_buttons.items():
            button.clicked.connect(lambda checked=False, action=action: self._start_maintenance(action))
        for field in ("host", "port"):
            view.fields[field].textChanged.connect(self._update_buttons)

        self._poll_timer = QTimer(self)
        self._poll_timer.timeout.connect(self._monitor)
        self._poll_timer.start(_STATUS_INTERVAL_MS)
        self._event_timer = QTimer(self)
        self._event_timer.timeout.connect(self._drain)
        self._event_timer.start(45)
        self._set_link("disconnected", "未连接")
        self._render_progress()
        self._status("就绪")
        self._update_buttons()

    def _current_target(self):
        task = self._motion or self._uncertain
        return task.request if task is not None else self._stop_request or self._stop_wait or self._session

    def _locally_busy(self):
        return self._awaiting_robot() or self._maintenance is not None

    def _awaiting_robot(self):
        return any((self._motion, self._uncertain, self._stop_request, self._stop_wait))

    def _reported_busy(self):
        if self._current_target() is None:
            return False
        owner = self._last_robot.get("owner") or {}
        return (self._last_robot.get("task_active") is True or owner.get("in_flight") is True
                or owner.get("mode") == "auto")

    def _can_request_start(self):
        # A click starts the authoritative Pi preflight. Cached readiness can
        # explain a delay, but must not lock out a fresh check.
        return (self._link == "connected" and self._session is not None
                and not self._locally_busy() and not self._reported_busy()
                and not self._needs_empty_reset())

    def _needs_empty_reset(self):
        return (self._last_robot.get("owner") or {}).get("restart_requires_empty_reset") is True

    def _supports_empty_reset(self):
        return type((self._last_robot.get("owner") or {}).get("restart_requires_empty_reset")) is bool

    def _can_reset_empty(self):
        owner = self._last_robot.get("owner") or {}
        return (self._link == "connected" and self._session is not None
                and self._supports_empty_reset() and not self._locally_busy() and _robot_idle(self._last_robot)
                and owner.get("mcu_ready") is True)

    def _task_hint_text(self, busy, robot_busy):
        if busy:
            return "等待当前操作结束或确认机器人空闲。"
        if self._link != "connected":
            return "连接机器人后可启动任务。"
        if robot_busy:
            operation = (self._last_robot.get("owner") or {}).get("current_operation") or {}
            if operation.get("cancelled") is True:
                return "已取消后续动作，等待 MCU 确认当前动作已结束。"
            return "机器人正在执行任务。"
        if self._needs_empty_reset():
            if (self._last_robot.get("owner") or {}).get("mcu_ready") is not True:
                return "上次任务可能仍在持物。等待 MCU 握手恢复后，再固定或移走方块并恢复空载站姿。"
            return EMPTY_RESET_REQUIRED_MESSAGE
        if self._last_robot.get("prepared") is not True:
            return "控制服务尚未就绪，可点击任务重新检查。"
        if (self._last_robot.get("owner") or {}).get("mcu_ready") is not True:
            return "MCU 握手未就绪，可点击任务重新检查。"
        return ""

    def _update_buttons(self, *_):
        view = self.view
        current = self._current_target()
        if current is not None:
            view.target_label.setText(current.target)
        else:
            view.target_label.setText(f"{view.fields['host'].text()}:{view.fields['port'].text()}")
        busy = self._locally_busy()
        robot_busy = self._reported_busy()
        can_request_start = self._can_request_start()
        for button in view.task_buttons.values():
            button.setEnabled(can_request_start)
        view.drop_yaw.setEnabled(not busy and not robot_busy)
        needs_empty_reset = self._needs_empty_reset()
        view.reset_empty_button.setEnabled(self._can_reset_empty())
        task_hint = self._task_hint_text(busy, robot_busy)
        view.task_hint.setText(task_hint)
        view.task_hint.setVisible(bool(task_hint))
        connection_action = "重新连接" if self._link == "offline" else (
            "断开连接" if self._session else "连接机器人"
        )
        view.connection_button.setText("连接中…" if self._link == "connecting" else connection_action)
        can_reconnect = self._link == "offline" and self._maintenance is None
        view.connection_button.setEnabled(self._link != "connecting" and (not busy or can_reconnect))
        view.camera_button.setEnabled(self._link != "offline" and (self._camera is None or not self._camera.is_set()))
        view.stop_button.setEnabled(self._stop_request is None)
        view.scan_button.setEnabled(not busy and not robot_busy and self._link == "connected" and self._probe is None)
        network_locked = self._session is not None or busy
        use_remote = view.deployment_source.currentData() == "remote"
        source_field_enabled = {"remote_url": use_remote, "local_path": not use_remote}
        for key, field in view.fields.items():
            locked = network_locked if key in ("host", "port", "username") else busy
            field.setEnabled(not locked and source_field_enabled.get(key, True))
        view.deployment_source.setEnabled(not busy)
        view.local_browse_button.setEnabled(view.fields["local_path"].isEnabled())
        view.password.setEnabled(not network_locked)
        view.save_button.setEnabled(not busy)
        view.route_edit_button.setEnabled(not busy)
        view.pi_buttons["check"].setEnabled(not busy)
        for action in ("deploy", "rollback"):
            view.pi_buttons[action].setEnabled(not busy and not robot_busy)
        view.update_start_button.setEnabled(not busy and not robot_busy and not needs_empty_reset)
        view.update_task.setEnabled(view.update_start_button.isEnabled())

    def _set_link(self, state, label):
        self._link = state
        self.view.set_connection_state(state, label)

    def _status(self, text):
        self.view.status_label.setText(text)

    def _log(self, text):
        self.view.log.appendPlainText(f"{time.strftime('%H:%M:%S')}  {str(text).rstrip()}")

    def _clear_log(self):
        self._drain()
        self.view.log.clear()

    def _open_report_directory(self):
        try:
            directory = TaskReport_Directory()
            directory.mkdir(parents=True, exist_ok=True)
            if not QDesktopServices.openUrl(QUrl.fromLocalFile(str(directory))):
                raise OSError("资源管理器未能打开该目录")
        except (ValueError, OSError) as exc:
            self._log(f"打开日志文件夹失败 · {exc}")

    def _change_report_directory(self):
        try:
            current = TaskReport_Directory()
        except (ValueError, OSError):
            current = Path.home()
        path = QFileDialog.getExistingDirectory(self, "选择今后保存任务报告的文件夹", str(current))
        if path:
            self.view.change_reports_button.setEnabled(False)
            self._start_worker("report_directory", None, "r3-report-directory",
                               lambda: TaskReport_SetDirectory(path))

    def _report_directory_finished(self, context, outcome):
        self.view.change_reports_button.setEnabled(True)
        if isinstance(outcome, Exception):
            self._log(f"更改日志目录失败 · {outcome}")
        else:
            self._log(f"新任务报告将保存到 {outcome}。已有报告可在原目录查看。")

    def _show_report_sharing(self):
        if self._report_dialog is None:
            self._report_dialog = ReportWindow(self, self._start_worker)
            self._report_dialog.auto_changed.connect(self._report_auto_changed)
        self._report_dialog.ReportWindow_Show(self._latest_report)

    def _report_auto_changed(self, enabled):
        if enabled:
            if self._latest_report:
                self.report_dispatch.ReportDispatch_Enqueue(self._latest_report)
            self.report_dispatch.ReportDispatch_Retry()
        self._log("任务报告自动推送已启用。" if enabled else "任务报告自动推送已关闭。")

    def _report_share_finished(self, dialog, outcome):
        self._log(dialog.ReportWindow_Finish(outcome))

    def _edit_action_chain(self):
        path = settings.ROBOT_WORKSPACE / "offline_validation" / "config" / "route_steps.json"
        if not path.is_file():
            self._status("未找到动作链配置文件")
            self._log(f"{self.view.status_label.text()} · {path}")
            return
        editor = Path(os.environ.get("SystemRoot", r"C:\Windows")) / "System32" / "notepad.exe"
        try:
            subprocess.Popen([str(editor), str(path)])
        except OSError as exc:
            self._status("无法打开动作链配置")
            self._log(f"{self.view.status_label.text()} · {exc}")

    def _settings_values(self):
        values = {key: field.text().strip() for key, field in self.view.fields.items()}
        values["deployment_source"] = self.view.deployment_source.currentData()
        try:
            values["port"] = int(values["port"])
        except ValueError:
            raise ValueError("请输入 1 到 65535 之间的端口。") from None
        return settings.validate(values)

    def _request(self):
        values = self._settings_values()
        password = self.view.password.text() or settings.password_for(values)
        return _Request(values, password)

    def _try_request(self):
        try:
            return self._request()
        except (ValueError, OSError) as exc:
            QMessageBox.warning(self, "请检查连接配置", _safe_text(exc, self.view.password.text()))
            return None

    def _save_settings(self):
        password = self.view.password.text() or None
        try:
            settings.save(self._settings_values(), password)
        except (OSError, ValueError) as exc:
            self._status("保存配置失败")
            self._log(_safe_text(exc, password))
            return
        if not self._session:
            self.view.password.clear()
        self._status("配置已保存")
        self._log("配置已保存。")

    def _toggle_connection(self):
        reconnect = self._link == "offline"
        if self._link == "connecting" or (self._locally_busy() and not reconnect) or self._maintenance:
            return
        if self._session and not reconnect:
            self._end_connection()
            return
        request = (self._current_target() if reconnect else None) or self._try_request()
        if request is None:
            return
        self._session = request
        self._link_confirmed = False
        self._last_robot = {}
        self._camera_on_connect = self._camera is None or self._camera.is_set()
        self._set_link("connecting", "连接中")
        self.view.robot_status.setText("正在核对 R3 控制服务")
        self._status(f"正在连接 {request.target}")
        self._log(f"连接 {request.target}。")
        self._start_probe(request, "status", quiet=False)
        self._update_buttons()

    def _link_dropped(self):
        if self._link == "offline":
            return
        pending = self._awaiting_robot()
        self._probe = None
        self._last_robot = {}
        self._link_confirmed = False
        self._camera_on_connect = False
        if not pending:
            self._session = None
        self._set_link("offline", "机器人离线")
        self._stop_camera(_CAMERA_OFFLINE)
        self.view.robot_status.setText("任务结果待确认" if pending else "开机并恢复网络后可重新连接")
        note = "机器人离线 · 任务结果待确认" if pending else "机器人离线 · 状态轮询已暂停"
        self._status(note)
        self._log(note + "。恢复供电和网络后点击「重新连接」。")
        self._update_buttons()

    def _end_connection(self):
        self._session = None
        self._probe = None
        self._link_confirmed = False
        self._last_robot = {}
        self._camera_on_connect = False
        self._set_link("disconnected", "未连接")
        self._progress.reset()
        self._render_progress()
        self._stop_camera()
        self.view.robot_status.setText("等待连接")
        self._status("已断开连接")
        self._log("已断开控制台连接。")
        self._update_buttons()

    def _confirm(self, title, prompt):
        return QMessageBox.question(self, title, prompt, QMessageBox.StandardButton.Yes |
                                    QMessageBox.StandardButton.No,
                                    QMessageBox.StandardButton.No) == QMessageBox.StandardButton.Yes

    def _post_log(self, request=None):
        return lambda message: self._events.put(("log", None, _safe_text(message, request.password if request else None)))

    def _save_receipt(self, request, command, outcome, *, after_update=False):
        path = TaskReport_Save(outcome, command=command, target=request.target,
                               after_update=after_update, password=request.password)
        self.report_dispatch.ReportDispatch_Enqueue(path)
        return path

    def _start_worker(self, kind, context, name, operation, *, receipt=None):
        def run():
            try:
                outcome = operation()
            except Exception as exc:
                outcome = exc
            self._events.put((kind, context, outcome))
            if receipt is not None:
                request, command = receipt
                archived = outcome
                if isinstance(outcome, RobotControlUncertainError):
                    archived = dict(outcome.as_result(), status=f"{command}_unconfirmed")
                elif isinstance(outcome, Exception):
                    archived = {"ok": False, "status": f"{command}_failed",
                                "exception_type": type(outcome).__name__, "reason": str(outcome)}
                try:
                    path = self._save_receipt(request, command, archived)
                except Exception as exc:
                    self._post_log(request)(f"本机操作报告保存失败 · {exc}")
                else:
                    self._events.put(("ssh_report", None, str(path)))

        # Finish saving explicit receipts if the last window closes just after the reply.
        worker = Thread(target=run, name=name, daemon=receipt is None)
        try:
            worker.start()
        except RuntimeError as exc:
            self._events.put((kind, context, exc))
            return None
        return worker

    def _start_probe(self, request, action, *, quiet=True):
        if self._probe is not None:
            return
        context = _Probe(request, action, quiet, self._uncertain, self._stop_wait, self._motion)
        self._probe = context
        retry_connect = not quiet and action == "status" and self._link == "connecting"

        def read():
            api = _robot_api(request, None if quiet else self._post_log(request))
            if action != "status":
                return api.scan()
            try:
                return api.status()
            except SshLinkError:
                if not retry_connect:
                    raise
                self._post_log(request)("首次 SSH 未响应，正在重试一次连接。")
                time.sleep(_CONNECT_RETRY_DELAY_S)
                return api.status()

        self._start_worker("probe", context, "r3-status", read,
                           receipt=(request, "scan") if action == "scan" else None)
        self._update_buttons()

    def _request_scan(self):
        if self._session and self._link == "connected" and not self._locally_busy() and not self._reported_busy():
            self._start_probe(self._session, "scan", quiet=False)

    def _monitor(self):
        if self._link == "offline":
            return
        request = self._current_target()
        if request is not None and (self._maintenance is None or self._stop_wait):
            self._start_probe(request, "status")

    def _probe_finished(self, context, outcome):
        if context is not self._probe:
            return
        self._probe = None
        request = context.request
        if request is not self._current_target():
            if self._link == "connecting" and self._session is not None:
                self._start_probe(self._session, "status", quiet=False)
            self._update_buttons()
            return
        if isinstance(outcome, SshLinkError):
            self._link_dropped()
            return
        if context.action == "scan":
            if isinstance(outcome, Exception):
                self._log(_safe_text(outcome, request.password))
            elif outcome.get("ids"):
                self._log("识别到 Tag " + "、".join(map(str, outcome["ids"])))
            else:
                self._log(_safe_text(TaskReport_Reason(outcome) or "当前画面未识别到 Tag。", request.password))
            self._update_buttons()
            return

        owner = outcome.get("owner") if isinstance(outcome, dict) else None
        valid = (isinstance(owner, dict) and owner.get("robot_id") == 3 and outcome.get("ok") is True
                 and outcome.get("protocol") == "r3.warm.v1" and outcome.get("event") == "status")
        if valid:
            self._read_robot_status(context, outcome)
        else:
            if request is self._session:
                was_error = self._link == "error"
                pending_reconnect = self._awaiting_robot() and not self._link_confirmed
                self._set_link("offline" if pending_reconnect else "error",
                               "连接中断" if self._link_confirmed else "连接失败")
                if not self._link_confirmed and not pending_reconnect:
                    self._session = None
                    self._camera_on_connect = False
                self.view.robot_status.setText("任务结果待确认" if pending_reconnect else "R3 控制服务状态不可用")
                if not self._locally_busy():
                    self._status(self.view.connection_label.text())
                if not was_error or not context.quiet:
                    detail = TaskReport_Reason(outcome) if isinstance(outcome, dict) else outcome
                    detail = detail or "未收到 R3 状态回执。"
                    self._log(_safe_text(detail, request.password))
        self._update_buttons()

    def _read_robot_status(self, context, status):
        owner = status["owner"]
        if context.request is self._session:
            previous_link = self._link
            self._link_confirmed = True
            self._set_link("connected", "已连接")
            if previous_link != "connected":
                self._log(f"R3 控制服务在线，目标 {context.request.target}。")
                if not self._locally_busy():
                    self._status("已连接")
            if self._camera_on_connect and self._camera is None:
                self._camera_on_connect = False
                self._start_camera(context.request)
        self._last_robot = status
        motion = context.motion
        progress = status.get("mission_progress")
        if (motion is not None and motion is self._motion and status.get("task_active") is True
                and isinstance(progress, dict) and str(progress.get("command")) == str(motion.command)):
            motion.last_observed_status = {
                "task_active": True, "mission_progress": dict(progress),
                "owner": {key: owner.get(key) for key in
                          ("session", "epoch", "mode", "in_flight", "operation_seq")},
            }
        mode = {"manual": "手动待机", "auto": "自动模式"}.get(owner.get("mode"), "模式未知")
        task = "任务运行中" if status.get("task_active") else "空闲" if _robot_idle(status) else "状态待确认"
        if owner.get("in_flight") is True:
            task = "动作执行中"
        mcu = {True: "MCU 就绪", False: "MCU 未就绪"}.get(owner.get("mcu_ready"), "MCU 状态未知")
        service = "控制服务就绪" if status.get("prepared") else "控制服务未就绪"
        self.view.robot_status.setText(f"{mode}  ·  {task}\n{mcu}  ·  {service}")
        self._progress.observe(status)
        self._render_progress()
        late_task = self._stop_retry
        started = (owner.get("mode") == "auto" or owner.get("in_flight") is True
                   or status.get("task_active") is True and isinstance(status.get("mission_progress"), dict))
        if (late_task is not None and late_task is (self._motion or self._uncertain)
                and context.stop_wait is late_task.request and self._stop_request is None and started):
            self._log("第一次停止时机器人仍空闲，检测到并发启动后再次发送停止。")
            self._send_stop(late_task.request)
        if _robot_idle(status) and self._motion is None:
            if context.uncertain is not None and context.uncertain is self._uncertain:
                self._uncertain = None
                self._status("机器人已确认空闲")
            if context.stop_wait is not None and context.stop_wait is self._stop_wait:
                self._stop_wait = None
                self._stop_retry = None
                self._status("机器人已确认空闲")

    def _start_motion(self, command, *, after_update=False):
        if self._locally_busy() or self._reported_busy():
            return
        empty_reset = command == _RESET_EMPTY
        if empty_reset:
            request = self._session
            if (after_update or not self._can_reset_empty()
                    or not self._confirm(
                        _RESET_EMPTY_LABEL, "复位会放低双臂。请确认机器人站稳、周围留有空间，"
                        "并已托住或移走方块。复位完成后再把机器人摆回起点。\n\n确认复位？"
                    )):
                return
        elif command not in range(3) or self._needs_empty_reset():
            return
        elif after_update:
            request = self._try_request()
            if request is None or not self._confirm(
                "部署后启动", f"将部署到 {request.target} 的 {request.values['remote_path']}。\n"
                f"\n{request.deployment_origin}\n\n"
                f"确认部署成功后自动启动「{_TASK_NAMES[command]}」，放物转角 {self.view.drop_yaw.value():+d}°。\n继续？"
            ):
                return
            self._prepare_deployment(request, command=command)
            return
        elif self._can_request_start():
            request = self._session
        else:
            return
        self._queue_motion(request, command)

    def _queue_motion(self, request, command, *, after_update=False, mcu_readback=None, cancelled=None):
        empty_reset = command == _RESET_EMPTY
        task = _Motion(request, command, after_update, cancelled or Event())
        angle = 0 if empty_reset else self.view.drop_yaw.value()
        self._motion = task
        self._progress.reset(None if empty_reset else command)
        self._render_progress()
        if empty_reset:
            self._status(f"正在{_RESET_EMPTY_LABEL}…")
        else:
            operation_name = "正在部署，完成后检查并启动" if after_update else "正在检查并启动"
            suffix = f" · 放物转角 {angle:+d}°" if angle else ""
            self._status(f"{operation_name}「{_TASK_NAMES[command]}」{suffix}")
        self._log(self.view.status_label.text())
        self._update_buttons()

        def run():
            try:
                log = self._post_log(request)
                api = _robot_api(request, log)
                options = {"drop_yaw_deg": angle} if angle else {}
                if empty_reset:
                    outcome = api.reset_empty(cancel_event=task.cancelled)
                elif mcu_readback is not None:
                    _wait_for_mcu_owner(request, task.cancelled, log)
                    outcome = api.update_then_start(command, cancel_event=task.cancelled,
                                                    mcu_readback=mcu_readback, **options)
                else:
                    action = api.update_then_start if after_update else api.start
                    outcome = action(command, cancel_event=task.cancelled, **options)
            except Exception as exc:
                outcome = exc
            result = _motion_result(task, outcome)
            diagnostics = TaskReport_Diagnostics(result)
            archived = outcome
            if result and result.get("status") == "start_unconfirmed":
                archived = (dict(outcome, start=result)
                            if after_update and isinstance(outcome, dict) and isinstance(outcome.get("start"), dict)
                            else result)
            try:
                path = self._save_receipt(request, command, archived, after_update=after_update)
            except Exception as exc:
                return _MotionCompletion(outcome, diagnostics, report_error=_safe_text(exc, request.password))
            return _MotionCompletion(outcome, diagnostics, report_path=str(path))

        self._start_worker("motion", task, "r3-motion", run)

    def _motion_finished(self, task, outcome):
        completion = outcome if isinstance(outcome, _MotionCompletion) else _MotionCompletion(outcome)
        outcome = completion.outcome
        if task.after_update and isinstance(outcome, dict):
            deployment = outcome.get("deployment")
            if isinstance(deployment, dict) and deployment.get("mcu_readback_registered") is True:
                self._mcu_receipt = None
        self._motion = None
        description = _RESET_EMPTY_LABEL if task.command == _RESET_EMPTY else _TASK_NAMES[task.command]
        result = _motion_result(task, outcome)
        reset_status = RobotStart_EmptyResetStatus(result)
        if reset_status is not None and task.request == self._session:
            # A poll issued before this rejection can still be in flight.
            self._probe = None
            self._last_robot = reset_status
        if isinstance(outcome, Exception):
            if isinstance(outcome, RobotControlUncertainError):
                self._uncertain = task
            self._status(f"{description} · " + ("结果待确认" if self._uncertain else "启动失败"))
            if isinstance(outcome, SshLinkError) or isinstance(outcome.__cause__, SshLinkError):
                self._link_dropped()
            else:
                self._log(_safe_text(outcome, task.request.password))
        else:
            status = (result or {}).get("status")
            outbound_only = (result or {}).get("outbound_only") is True
            shared_route = task.command == 0 or outbound_only
            tag5_hold = MissionDelivery_Tag5HoldLabel(result)
            if task.command == _RESET_EMPTY and status == "stand_request_completed" and outcome.get("ok"):
                text = "已收到复位回执，摆回起点后可重新启动任务"
            elif reset_status is not None:
                text = ("等待 MCU 握手" if reset_status["owner"].get("mcu_ready") is False
                        else "需恢复空载站姿")
            elif tag5_hold:
                text = tag5_hold
            elif shared_route and status == "task_complete" and outcome.get("ok"):
                text = MissionDelivery_Label(task.command, outbound_only)
            elif shared_route and isinstance(status, str) and status in _COMMAND0_RESULT_TEXT:
                text = _COMMAND0_RESULT_TEXT[status]
                if status == "stage3_trial_unconfirmed":
                    corridor = result.get("corridor_trial")
                    if isinstance(corridor, dict) and corridor.get("status") == "departure_unsafe":
                        text = TaskReport_Reason(corridor) or text
                    else:
                        tag = Command0_LastPassedTag(result)
                        if tag is not None:
                            text = f"已扫到 Tag {tag}，{text}"
            elif task.command == 2 and status == "return_trial_unconfirmed":
                text = "返程试跑待确认"
                tag = Command2_LastReturnTag(result)
                if tag is not None:
                    text = f"已扫到 Tag {tag}，{text}"
            elif outcome.get("ok"):
                text = "本版试跑结束" if task.command == 0 else "任务完成"
            elif shared_route and status == "stage1_partial_alignment":
                text = "本版试跑动作已结束，入口位姿待确认"
            elif status == "cancelled_before_dispatch":
                text = "已在发出前取消"
            elif status == "preflight_rejected":
                text = "启动检查未通过"
            elif status == "start_unconfirmed":
                self._uncertain = task
                text = "启动结果待确认"
            else:
                text = "任务未完成"
            self._status(f"{description} · {text}")
            reason = (EMPTY_RESET_REQUIRED_MESSAGE if reset_status is not None and task.command != _RESET_EMPTY
                      else TaskReport_Reason(result))
            if reason:
                self._log(_safe_text(reason, task.request.password))
        for line in completion.diagnostics:
            self._log(_safe_text(line, task.request.password))
        if completion.report_path:
            self._latest_report = completion.report_path
            self._log(f"本机任务报告已保存 · {completion.report_path}")
        elif completion.report_error:
            self._log(f"本机任务报告保存失败 · {completion.report_error}")
        if task.command == _RESET_EMPTY:
            self._progress.reset()
        else:
            self._progress.finish(result)
        self._render_progress()
        self._log(self.view.status_label.text())
        self._update_buttons()
        self._monitor()

    def _request_stop(self):
        if self._stop_request is not None:
            return
        request = self._current_target() or self._try_request()
        if request is None:
            return
        task = self._motion or self._uncertain
        if task is not None:
            task.cancelled.set()
        if self._deployment_cancel is not None:
            self._deployment_cancel.set()
        self._send_stop(request, task)

    def _send_stop(self, request, retry_candidate=None):
        self._stop_retry = None
        self._stop_request = request
        self._status(f"正在向 {request.target} 发送停止请求")
        self._log(self.view.status_label.text())
        self._update_buttons()
        self._start_worker("stop", (request, retry_candidate), "r3-stop",
                           lambda: _robot_api(request, self._post_log(request)).stop(), receipt=(request, "stop"))

    def _stop_finished(self, context, outcome):
        request, retry_candidate = context
        self._stop_request = None
        if isinstance(outcome, Exception):
            if isinstance(outcome, RobotControlUncertainError):
                self._stop_wait = request
            self._status("停止请求未确认")
            if isinstance(outcome, SshLinkError) or isinstance(outcome.__cause__, SshLinkError):
                self._link_dropped()
            else:
                self._log(_safe_text(outcome, request.password))
        elif outcome.get("ok"):
            self._stop_wait = request
            if outcome.get("status") == "already_idle":
                self._stop_retry = retry_candidate
            self._status("停止请求已送达，等待机器人空闲")
        else:
            self._status("停止请求未确认")
            self._log(_safe_text(TaskReport_Reason(outcome) or outcome.get("status"), request.password))
        self._log(self.view.status_label.text())
        self._update_buttons()
        self._monitor()

    def _render_progress(self):
        value = self._progress.value
        self.view.progress_bar.setRange(0, 0 if value is None else 100)
        if value is not None:
            self.view.progress_bar.setValue(value)
        self.view.progress_label.setText(self._progress.text)

    def _prepare_deployment(self, request, *, command=None):
        receipts = settings.settings_path().parent / "usb_receipts"
        cancelled = Event()
        self._deployment_cancel = cancelled
        self._maintenance = "deploy"
        self._status("正在检测 R3 U 盘…")
        self._update_buttons()
        log = self._post_log(request)
        self._start_worker("deployment_usb", (request, command, cancelled), "r3-mcu-readback",
                           lambda: usb_mode.McuReadback_VerifyConnected(settings.DEFAULT_MCU_OVERLAY, receipts, log)
                           if request.values["remote_path"] == settings.DEFAULT_PI_PATH else None)

    def _deployment_usb_finished(self, context, outcome):
        request, command, cancelled = context
        if isinstance(outcome, Exception):
            self._maintenance_finished(("deploy", request), outcome)
            return
        if outcome is not None:
            self._mcu_receipt = _PendingMcuReadback(request.deploy_target,
                                                    Path(outcome["receipt_path"]), outcome["drive"])
            self._log(f"MCU 文件核验通过 · {self._mcu_receipt.receipt}")
        pending = self._mcu_receipt
        if pending is not None and (pending.target != request.deploy_target or not pending.receipt.is_file()):
            pending = None
        if pending is not None and not pending.reset_confirmed and not cancelled.is_set():
            self._status("MCU 核验完成，等待复位")
            confirmed = self._confirm("复位 MCU 后继续",
                f"已核验 R3 U 盘 {pending.drive}。请弹出 U 盘、退出 U 盘模式并按机器人复位键。\n"
                "完成后点「是」，程序会等待机器人上线，再部署并自动登记 MCU。")
            if not confirmed or Path(pending.drive).exists():
                self._maintenance = None
                self._deployment_cancel = None
                self._status("MCU 回执已保留")
                self._log("退出 U 盘模式并复位后，可再次点击部署继续。")
                self._update_buttons()
                return
            pending.reset_confirmed = True
        if cancelled.is_set():
            self._maintenance_finished(("deploy", request), {"status": "cancelled"})
            return
        receipt = pending.receipt if pending is not None else None
        if command is not None:
            self._maintenance = None
            self._deployment_cancel = None
            self._queue_motion(request, command, after_update=True, mcu_readback=receipt, cancelled=cancelled)
            return
        self._status("部署树莓派程序中…")
        log = self._post_log(request)

        def deploy():
            if receipt is None:
                return operate("deploy", request.values, request.password, log)
            if not _wait_for_mcu_owner(request, cancelled, log):
                return {"status": "cancelled"}
            return operate("deploy", request.values, request.password, log, mcu_readback=receipt)

        self._start_worker("maintenance", ("deploy", request), "r3-deploy", deploy, receipt=(request, "deploy"))

    def _start_maintenance(self, action):
        if self._locally_busy() or (action != "check" and self._reported_busy()):
            return
        request = self._try_request()
        if request is None:
            return
        values = request.values
        label = _ACTION_NAMES[action]
        if action != "check":
            destination = f"{values['username']}@{request.target}\n{values['remote_path']}"
            prompt = f"操作目标\n{destination}"
            if action == "deploy":
                prompt += f"\n\n{request.deployment_origin}\n\n写入前会保留备份。"
                if values["remote_path"] == settings.DEFAULT_PI_PATH:
                    prompt += "\n检测到 R3 U 盘时会核验已覆盖的 MCU 文件，并在复位后登记。"
            if not self._confirm(label, prompt + "\n\n继续？"):
                return
        if action == "deploy":
            self._prepare_deployment(request)
            return

        self._maintenance = action
        self._status(f"{label}中…")
        self._log(self.view.status_label.text())
        self._update_buttons()
        report = self._post_log(request)

        self._start_worker("maintenance", (action, request), "r3-deploy",
                           lambda: operate(action, values, request.password, report), receipt=(request, action))

    def _maintenance_finished(self, context, outcome):
        action, request = context
        self._maintenance = None
        self._deployment_cancel = None
        label = _ACTION_NAMES[action]
        if isinstance(outcome, Exception):
            self._status(label + "失败")
            self._log(_safe_text(outcome, request.password))
        elif outcome.get("status") == "cancelled":
            self._status("部署已取消")
        else:
            self._status(label + "完成")
            if outcome.get("mcu_readback_registered") is True:
                self._mcu_receipt = None
                self._log("MCU 读回已登记。下一次任务会探测本次 MCU 的动作能力。")
        self._log(self.view.status_label.text())
        self._update_buttons()
        self._monitor()

    def _toggle_camera(self):
        self._camera_on_connect = False
        if self._camera is not None:
            self._stop_camera()
            return
        request = self._current_target() or self._try_request()
        if request is not None:
            self._start_camera(request)

    def _start_camera(self, request):
        if self._camera is not None:
            return
        stop = Event()
        self._camera = stop
        self.view.camera_button.setText("关闭预览")
        self._clear_camera(f"正在连接 {request.target}")

        def receive():
            for frame in _chest_frames(request, stop):
                if stop.is_set():
                    break
                decoded = QImage.fromData(frame.jpeg)
                if decoded.isNull():
                    raise ValueError("胸部相机图像解码失败。")
                with self._frame_lock:
                    self._frame = stop, decoded, request.target, time.monotonic()

        self._start_worker("camera", (stop, request), "r3-chest-camera", receive)

    def _stop_camera(self, label=None):
        if self._camera is not None:
            self._camera.set()
            self.view.camera_button.setEnabled(False)
            label = label or "正在关闭预览"
        if label is not None:
            self._clear_camera(label)

    def _clear_camera(self, label):
        self._last_frame_at = None
        self.view.camera_view.set_frame(None)
        self.view.camera_view.setText(label)
        self.view.camera_status.setText(label)

    def _camera_finished(self, context, outcome):
        event, request = context
        if event is not self._camera:
            return
        self._camera = None
        self.view.camera_button.setText("打开预览")
        if self._link == "offline":
            self._clear_camera(_CAMERA_OFFLINE)
        elif isinstance(outcome, SshLinkError) and not event.is_set():
            self._clear_camera("胸部预览连接中断")
            if self._session is request and self._link == "connected":
                self._start_probe(request, "status")
        elif isinstance(outcome, Exception) and not event.is_set():
            self._clear_camera("胸部相机不可用")
            self._log(_safe_text(outcome, request.password))
        else:
            self._clear_camera("预览已关闭")
        self._update_buttons()

    def _poll_camera(self):
        with self._frame_lock:
            frame, self._frame = self._frame, None
        if frame is not None:
            event, picture, host, received = frame
            if event is self._camera and not event.is_set():
                self._last_frame_at = received
                self.view.camera_view.set_frame(picture)
                self.view.camera_status.setText(f"{host} · {picture.width()} × {picture.height()}")
        if self._last_frame_at is not None and time.monotonic() - self._last_frame_at > _CAMERA_STALE_S:
            self._clear_camera("画面中断，等待新帧")

    def _drain(self):
        handlers = {"probe": self._probe_finished, "motion": self._motion_finished,
                    "stop": self._stop_finished, "maintenance": self._maintenance_finished,
                    "deployment_usb": self._deployment_usb_finished,
                    "camera": self._camera_finished, "report_share": self._report_share_finished,
                    "report_directory": self._report_directory_finished}
        for _ in range(100):
            try:
                kind, context, outcome = self._events.get_nowait()
            except Empty:
                break
            if kind == "log":
                self._log(outcome)
            elif kind == "ssh_report":
                self._latest_report = outcome
                self._log(f"本机操作报告已保存 · {outcome}")
            else:
                handlers[kind](context, outcome)
        self._poll_camera()

    def closeEvent(self, event):
        if self._report_dialog and self._report_dialog.busy:
            QMessageBox.information(self, "正在分享任务报告", "报告传输结束后即可关闭控制台。")
            event.ignore()
            return
        if (self._motion and self._link != "offline") or self._stop_request or self._maintenance:
            QMessageBox.information(self, "操作进行中", "请先停止自动任务，或等待部署操作结束。")
            event.ignore()
            return
        if (self._motion or self._uncertain or self._stop_wait) and not self._confirm(
            "关闭控制台", "机器人是否已空闲仍待确认。关闭窗口？"
        ):
            event.ignore()
            return
        self._stop_camera()
        self.report_dispatch.ReportDispatch_Close()
        self._poll_timer.stop()
        self._event_timer.stop()
        event.accept()


def run_control_window(values=None):
    """Open the Windows control console."""
    initial = settings.load() if values is None else settings.validate(values)
    application = QApplication.instance() or QApplication(sys.argv[:1])
    application.setApplicationName("AELOS R3 Control")
    window = _ControlWindow(initial)
    window.show()
    application.exec()
