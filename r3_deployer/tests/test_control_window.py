import json
import os
from pathlib import Path
from tempfile import TemporaryDirectory
from threading import Event
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QBuffer, QIODevice, QPoint, QRect, QSettings
from PySide6.QtGui import QColor, QImage, QPalette
from PySide6.QtWidgets import QApplication, QScrollArea, QStyle, QStyleOptionFrame

from r3_deployer import control_window, qt_view, settings, task_report
from r3_deployer.qt_theme import ConsoleTheme


def _status():
    return {
        "ok": True, "event": "status", "protocol": "r3.warm.v1",
        "prepared": True, "task_active": False,
        "owner": {"robot_id": 3, "mode": "manual", "in_flight": False, "mcu_ready": True},
    }


class ControlWindowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.application = QApplication.instance() or QApplication([])

    def setUp(self):
        temporary = TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        report_path = Path(temporary.name) / "settings.json"
        self.report_directory = report_path.parent / "task_reports"
        reports = patch.object(task_report, "settings_path", return_value=report_path)
        reports.start()
        self.addCleanup(reports.stop)
        camera = patch.object(control_window, "_chest_frames", return_value=())
        camera.start()
        self.addCleanup(camera.stop)
        self.enterContext(patch.object(control_window, "ReportDispatcher"))
        self.window = control_window._ControlWindow(settings.DEFAULTS)
        self.receipt_workers = []
        start_worker = self.window._start_worker

        def track_worker(*args, **kwargs):
            worker = start_worker(*args, **kwargs)
            if kwargs.get("receipt") and worker is not None:
                self.receipt_workers.append(worker)
            return worker

        self.enterContext(patch.object(self.window, "_start_worker", side_effect=track_worker))
        self.view = self.window.view
        self.window._poll_timer.setInterval(100)
        self.window.show()
        self.application.processEvents()

    def tearDown(self):
        self.window._poll_timer.stop()
        self.window._event_timer.stop()
        self.window._stop_camera()
        # A terminal SSH reply can reach Qt while its worker still owns a report file.
        for worker in self.receipt_workers:
            worker.join(timeout=4)
            self.assertFalse(worker.is_alive())
        self.window.hide()
        self.window.deleteLater()
        self.application.processEvents()

    def _until(self, condition):
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            self.application.processEvents()
            if condition():
                return
            time.sleep(.01)
        self.fail("Qt worker did not reach the expected state")

    def test_tabs_show_requested_actions_and_log_clear(self):
        self.assertEqual([self.view.tabs.tabText(i) for i in range(self.view.tabs.count())],
                         ["控制", "SSH配置与程序部署"])
        self.assertEqual([self.view.task_buttons[i].text() for i in range(3)],
                         ["单程搬运原始物资", "单程搬运转运物资", "连续搬运转运物资"])
        self.assertEqual(self.view.connection_label.text(), "未连接")
        self.assertFalse(self.view.task_buttons[0].isEnabled())
        self.assertTrue(self.view.reset_empty_button.isVisible())
        self.assertFalse(self.view.reset_empty_button.isEnabled())
        self.window._log("test log")
        self.view.clear_log_button.click()
        self.assertEqual(self.view.log.toPlainText(), "")

    def test_angle_editor_shows_the_entire_signed_value_with_the_caret_at_either_end(self):
        combo = self.view.drop_yaw
        editor = combo.lineEdit()
        for theme in ConsoleTheme:
            self.view.theme = theme
            self.view._apply_theme()
            combo.setFocus()
            for angle in ("-90", "+90"):
                with self.subTest(theme=theme, angle=angle):
                    combo.setCurrentText(angle)
                    self.application.processEvents()
                    option = QStyleOptionFrame()
                    editor.initStyleOption(option)
                    contents = editor.style().subElementRect(QStyle.SE_LineEditContents, option, editor)
                    width = contents.marginsRemoved(editor.textMargins()).width()
                    text_width = editor.fontMetrics().horizontalAdvance(angle)
                    # QLineEdit reserves two pixels on either side of its text.
                    self.assertGreaterEqual(width, text_width + 4)
                    editor.setCursorPosition(0)
                    left = editor.cursorRect().x()
                    editor.setCursorPosition(len(angle))
                    self.assertAlmostEqual(editor.cursorRect().x() - left, text_width, delta=1)

    def test_task_controls_fit_and_remain_reachable_at_scaled_window_sizes(self):
        # Logical client sizes from normal windows and maximized 1080p at 125% / 150%.
        for width, height in ((1170, 740), (1536, 788), (1280, 656)):
            with self.subTest(width=width, height=height):
                self.window.resize(width, height)
                self.application.processEvents()
                self.assertLessEqual(self.window.width(), width)
                self.assertLessEqual(self.window.height(), height)
                view = self.view
                task_controls = (view.drop_yaw, *view.task_buttons.values(), view.task_hint,
                                 view.reset_empty_button, view.progress_bar, view.progress_label)
                rectangles = [QRect(widget.mapTo(view, QPoint()), widget.size()) for widget in task_controls]
                for upper, lower in zip(rectangles, rectangles[1:]):
                    self.assertLess(upper.bottom(), lower.top(), "Task controls must not overlap")
                for widget in (view.scan_button, view.robot_status, *task_controls):
                    self.assertTrue(widget.isVisible())
                    ancestor = widget.parentWidget()
                    while ancestor is not None:
                        if isinstance(ancestor, QScrollArea):
                            ancestor.ensureWidgetVisible(widget, 0, 0)
                            self.application.processEvents()
                            viewport = ancestor.viewport()
                            visible = QRect(widget.mapTo(viewport, QPoint()), widget.size())
                            self.assertTrue(viewport.rect().contains(visible),
                                            f"{widget!r} occupies {visible} within {viewport.rect()}")
                        ancestor = ancestor.parentWidget()
                    self.assertTrue(view.rect().contains(QRect(widget.mapTo(view, QPoint()), widget.size())))

    def test_open_action_chain_edits_workspace_source(self):
        with TemporaryDirectory() as directory:
            workspace = Path(directory)
            route = workspace / "offline_validation" / "config" / "route_steps.json"
            route.parent.mkdir(parents=True)
            route.write_text("{}", encoding="utf-8")
            self.view.tabs.setCurrentIndex(1)
            self.assertEqual(self.view.route_edit_button.text(), "编辑动作链")
            with patch.object(settings, "ROBOT_WORKSPACE", workspace):
                with patch.object(control_window.subprocess, "Popen") as open_editor:
                    self.view.route_edit_button.click()
            open_editor.assert_called_once()
            editor, path = open_editor.call_args.args[0]
            self.assertEqual(Path(editor).name, "notepad.exe")
            self.assertEqual(Path(path), route)

    def test_dark_theme_keeps_typography_and_is_restored(self):
        with TemporaryDirectory() as directory:
            path = str(Path(directory) / "appearance.ini")
            appearance = QSettings(path, QSettings.Format.IniFormat)
            appearance.setValue("appearance/theme", "light")
            with patch.object(qt_view, "QSettings", return_value=appearance):
                view = qt_view.ControlView(settings.DEFAULTS)
                self.addCleanup(view.deleteLater)
                self.assertIs(view.theme, ConsoleTheme.LIGHT)
                view.theme_button.click()
                self.assertIs(view.theme, ConsoleTheme.DARK)
                self.assertEqual(view.theme_button.text(), "切换到亮色")
                self.assertLess(view.palette().color(QPalette.ColorRole.Window).lightness(), 50)
                self.assertEqual(view.log.font().pointSize(), 11)
                self.assertEqual(view.camera_view.font().pointSize(), 11)
            reloaded = QSettings(path, QSettings.Format.IniFormat)
            with patch.object(qt_view, "QSettings", return_value=reloaded):
                restored = qt_view.ControlView(settings.DEFAULTS)
            self.addCleanup(restored.deleteLater)
            self.assertIs(restored.theme, ConsoleTheme.DARK)
            restored.theme_button.click()
            self.assertEqual(reloaded.value("appearance/theme"), "light")

    def test_failed_connection_keeps_tasks_disabled(self):
        self.view.password.setText("test-credential")
        with patch.object(control_window, "_robot_api", side_effect=RuntimeError("SSH 登录被拒绝")):
            self.view.connection_button.click()
            self._until(lambda: self.view.connection_label.text() == "连接失败")
        self.assertTrue(self.view.connection_button.isEnabled())
        self.assertFalse(self.view.task_buttons[0].isEnabled())
        self.assertIn("SSH 登录被拒绝", self.view.log.toPlainText())

    def test_cold_boot_waits_for_ssh_then_owner_then_enables_routes(self):
        replies = iter((control_window.SshLinkError("WinError 10054"),
                        control_window.SshLinkError("WinError 10054"),
                        {"ok":False,"status":"owner_socket_starting"},
                        {"ok":False,"status":"not_started_or_stopped","reason":"[Errno 111] Connection refused"},
                        dict(_status(), boot_timing={"owner_endpoint_boottime_s":36.2,"ssh_active_monotonic_s":14.7})))

        def status():
            reply = next(replies, _status())
            if isinstance(reply, Exception):
                raise reply
            return reply

        api = SimpleNamespace(status=status)
        self.view.password.setText("test-credential")
        with patch.object(control_window, "_robot_api", return_value=api):
            with patch.object(control_window, "_CONNECT_RETRY_DELAY_S", 0):
                self.view.connection_button.click()
                self._until(lambda: self.view.connection_label.text() == "已连接")
        log = self.view.log.toPlainText()
        self.assertEqual(log.count("正在等待 SSH 上线"), 1)
        self.assertEqual(log.count("正在等待 R3 控制服务上线"), 1)
        self.assertIn("14.7 秒",log)
        self.assertIn("36.2 秒",log)
        self.assertNotIn("机器人离线", log)
        self.assertTrue(self.view.task_buttons[0].isEnabled())

    def test_cold_boot_wait_can_be_cancelled_and_late_reply_is_ignored(self):
        queried, reply = Event(), Event()
        def status():
            queried.set()
            reply.wait(2)
            return _status()
        self.view.password.setText("test-credential")
        with patch.object(control_window, "_robot_api", return_value=SimpleNamespace(status=status)):
            self.view.connection_button.click()
            self._until(queried.is_set)
            self.view.connection_button.click()
            reply.set()
            self._until(lambda: not self.window._events.empty())
            self.window._drain()
        self.assertFalse(self.view.task_buttons[0].isEnabled())
        self.assertIsNone(self.window._session)
        self.assertEqual(self.window._link, "disconnected")

    def test_cold_boot_deadline_and_nontransient_errors(self):
        for result, label in ((control_window.SshLinkError("unreachable"), "机器人离线"),
                              ({"status":"owner_socket_starting"}, "连接失败"),
                              ({"status":"not_started_or_stopped","reason":"Robot3 CPU+machine identity mismatch"}, "连接失败")):
            with self.subTest(result=result):
                def status():
                    if isinstance(result, Exception):
                        raise result
                    return result
                self.view.password.setText("test-credential")
                with patch.object(control_window, "_BOOT_CONNECT_WINDOW_S", 0), patch.object(
                        control_window, "_robot_api", return_value=SimpleNamespace(status=status)):
                    self.view.connection_button.click()
                    self._until(lambda: self.view.connection_label.text() == label)
                    self.assertFalse(self.view.task_buttons[0].isEnabled())

    def test_read_only_boot_probe_can_yield_to_route_but_not_deployment(self):
        state = dict(_status(), boot_probe_in_flight=True)
        state["owner"]["in_flight"] = True
        self.view.password.setText("test-credential")
        with patch.object(control_window, "_robot_api", return_value=SimpleNamespace(status=lambda:state)):
            self.view.connection_button.click()
            self._until(lambda: self.view.connection_label.text() == "已连接")
            self.assertTrue(self.view.task_buttons[1].isEnabled())
            self.assertFalse(self.view.pi_buttons["deploy"].isEnabled())
            state["boot_probe_in_flight"] = False
            self._until(lambda: not self.view.task_buttons[1].isEnabled())

    def test_connected_mcu_not_ready_allows_fresh_preflight_and_explains_rejection(self):
        def status():
            value = _status()
            value["owner"]["mcu_ready"] = False
            return value

        commands = []

        def start(command, *, cancel_event):
            commands.append(command)
            return {"ok": False, "status": "preflight_rejected", "reason": "MCU 握手未就绪，任务未发送。"}

        self.view.password.setText("test-credential")
        with patch.object(control_window, "_robot_api", return_value=SimpleNamespace(status=status, start=start)):
            self.view.connection_button.click()
            self._until(lambda: self.view.connection_label.text() == "已连接")
            self.assertTrue(all(button.isEnabled() for button in self.view.task_buttons.values()))
            self.assertIn("MCU 握手未就绪", self.view.task_hint.text())
            self.view.task_buttons[0].click()
            self._until(lambda: self.view.status_label.text().endswith("启动检查未通过"))
            self.assertIn("MCU 握手未就绪，任务未发送", self.view.log.toPlainText())
        self.assertEqual(commands, [0])

    def test_power_off_releases_idle_session_and_closes_preview(self):
        off = Event()
        status_calls = []

        def read_status():
            status_calls.append(True)
            if off.is_set():
                raise control_window.SshLinkError("Error reading SSH protocol banner [WinError 10054]")
            return _status()

        def frames(_request, stop):
            stop.wait(3)
            return ()

        self.view.password.setText("test-credential")
        with patch.object(control_window, "_robot_api", return_value=SimpleNamespace(status=read_status)):
            with patch.object(control_window, "_chest_frames", side_effect=frames):
                self.view.connection_button.click()
                self._until(lambda: self.window._camera is not None)
                camera_stop = self.window._camera
                off.set()
                self._until(lambda: self.view.connection_label.text() == "机器人离线")
                self.assertTrue(camera_stop.is_set())
                self._until(lambda: self.window._camera is None)
                self.assertEqual(self.view.camera_status.text(), "机器人离线，预览已关闭")
                self.assertEqual(self.view.connection_button.text(), "重新连接")
                self.assertTrue(self.view.fields["host"].isEnabled())
                self.assertFalse(self.view.task_buttons[0].isEnabled())
                self.assertEqual(self.window._last_robot, {})
                count = len(status_calls)
                self.window._monitor()
                self.window._monitor()
                self.assertIsNone(self.window._probe)
                self.assertEqual(len(status_calls), count)
                log = self.view.log.toPlainText()
                self.assertNotIn("WinError", log)
                self.assertEqual(log.count("状态轮询已暂停"), 1)

                off.clear()
                self.view.connection_button.click()
                self._until(lambda: self.view.task_buttons[0].isEnabled())
                self.assertEqual(self.view.connection_label.text(), "已连接")

    def test_gui_sends_selected_release_angle(self):
        captured = []
        def run(command, *, cancel_event, drop_yaw_deg):
            captured.append((command, drop_yaw_deg))
            return {"ok": True, "accepted": True, "event": "done", "status": "task_complete"}
        self.view.password.setText("test-credential")
        api = SimpleNamespace(status=_status, start=run)
        with patch.object(control_window, "_robot_api", return_value=api):
            self.view.connection_button.click()
            self._until(lambda: self.view.task_buttons[2].isEnabled())
            choices = self.view.drop_yaw
            self.assertEqual([choices.itemText(i) for i in range(choices.count())], ["-90", "-45", "0", "+45", "+90"])
            self.assertEqual(choices.currentText(), "0")
            choices.setCurrentIndex(4)
            self.assertTrue(self.view.task_buttons[2].isEnabled())
            for bad in ("", "-", "+", "91", "1.5"):
                choices.setEditText(bad)
                self.assertFalse(self.view.task_buttons[2].isEnabled())
                self.assertFalse(self.view.update_start_button.isEnabled())
            choices.setEditText("-15")
            self.view.task_buttons[2].click()
            self._until(lambda: self.window._motion is None)
            self.assertEqual(captured, [(2, -15)])

    def test_power_off_during_task_keeps_uncertainty_until_manual_reconnect(self):
        off = Event()
        started = Event()
        release = Event()
        self.addCleanup(release.set)
        starts = []

        def read_status():
            if off.is_set():
                raise control_window.SshLinkError("Connection reset")
            if started.is_set() and not release.is_set():
                return dict(_status(), task_active=True,
                            mission_progress={"phase": "cmd0_turn", "command": "0", "direction": "outbound"})
            return _status()

        def run(command, *, cancel_event):
            starts.append(command)
            started.set()
            release.wait(3)
            raise control_window.RobotControlUncertainError("Lost task reply",accepted=True)

        self.view.password.setText("test-credential")
        api = SimpleNamespace(status=read_status, start=run)
        with patch.object(control_window, "_robot_api", return_value=api):
            self.view.connection_button.click()
            self._until(lambda: self.view.task_buttons[0].isEnabled())
            self.view.task_buttons[0].click()
            self._until(started.is_set)
            self._until(lambda: self.window._motion.last_observed_status is not None)
            off.set()
            self._until(lambda: self.view.robot_status.text() == "任务结果待确认")
            self.assertTrue(self.view.connection_button.isEnabled())
            self.assertFalse(self.view.fields["host"].isEnabled())
            self.assertFalse(self.view.task_buttons[0].isEnabled())
            self.assertIsNone(self.window._probe)
            release.set()
            self._until(lambda: self.window._uncertain is not None)
            self.assertFalse(self.view.task_buttons[0].isEnabled())
            path, = self.report_directory.glob("*.json")
            archived = json.loads(path.read_text(encoding="utf-8"))
            self.assertTrue(archived["result"]["accepted"])
            self.assertEqual(archived["result"]["last_observed_status"]["mission_progress"]["phase"], "cmd0_turn")
            self.assertIn("失联前最近一次状态", self.view.log.toPlainText())
            self.assertIn("结果待确认", self.view.progress_label.text())

            off.clear()
            self.view.connection_button.click()
            self._until(lambda: self.view.task_buttons[0].isEnabled())
            self.assertIsNone(self.window._uncertain)
        self.assertEqual(starts, [0])

    def test_failed_motion_logs_and_archives_pi_diagnostics(self):
        password = "fixture-password"
        motion_result = {
            "ok": False, "event": "done", "status": "stopped_no_further_auto_macros",
            "reason": "bounded carry recovery unresolved",
            "events": [{"post_action_heading": {
                "action": "carry_forward", "reason": "shared journey budget exhausted",
                "after_error_deg": 7.25,
                "correction_steps": [{"reason": "first correction", "error_deg": 11.0, "after_error_deg": 7.25}],
                "password": password,
            }}],
        }
        api = SimpleNamespace(status=_status, start=lambda command, cancel_event: motion_result)
        self.view.password.setText(password)
        with patch.object(control_window, "_robot_api", return_value=api):
            self.view.connection_button.click()
            self._until(lambda: self.view.task_buttons[0].isEnabled())
            self.view.task_buttons[0].click()
            self._until(lambda: self.view.status_label.text().endswith("任务未完成"))

        report_path, = self.report_directory.glob("*.json")
        saved = report_path.read_text(encoding="utf-8")
        report = json.loads(saved)
        self.assertEqual(report["result"]["events"][0]["post_action_heading"]["after_error_deg"], 7.25)
        self.assertNotIn(password, saved)
        log = self.view.log.toPlainText()
        self.assertEqual(report["result"]["reason"], "bounded carry recovery unresolved")
        self.assertEqual(report["summary_zh"]["reason"], "持物纠偏未能恢复航向")
        for text in ("持物纠偏未能恢复航向", "本段路线的纠偏次数已用完",
                     "carry_forward", "+7.25°", "纠偏第 1 步", str(report_path)):
            self.assertIn(text, log)

    def test_command_two_return_receipt_displays_last_confirmed_marker(self):
        result = {"ok": False, "event": "done", "status": "return_trial_unconfirmed",
                  "command": "2", "return_trial": {"checkpoint": 7}, "reason": "return scan unavailable"}
        api = SimpleNamespace(status=_status, start=lambda command, cancel_event: result)
        self.view.password.setText("test-credential")
        with patch.object(control_window, "_robot_api", return_value=api):
            self.view.connection_button.click()
            self._until(lambda: self.view.task_buttons[2].isEnabled())
            self.view.task_buttons[2].click()
            self._until(lambda: "已扫到 Tag 7，返程试跑待确认" in self.view.status_label.text())
        self.assertIn("返程 · 已扫到 Tag 7", self.view.progress_label.text())
        self.assertIn("return scan unavailable", self.view.log.toPlainText())

    def test_stop_keeps_the_in_progress_start_target(self):
        entered = Event()
        release = Event()
        self.addCleanup(release.set)
        stops = []
        password = "test-credential"

        class Api:
            def __init__(self, request):
                self.request = request

            def start(self, command, *, cancel_event):
                entered.set()
                release.wait(3)
                return {"ok": False, "status": "cancelled_before_dispatch"} if cancel_event.is_set() else {"ok": True}

            def stop(self):
                stops.append((self.request.target, self.request.password))
                release.set()
                return {"ok": True, "status": "stop_requested"}

            def status(self):
                return _status()

        with patch.object(control_window, "_robot_api", side_effect=lambda request, log: Api(request)):
            with patch.object(settings, "password_for", return_value=password):
                self.view.connection_button.click()
            self._until(lambda: self.view.task_buttons[0].isEnabled())
            self.assertEqual(self.view.connection_label.text(), "已连接")
            self.view.task_buttons[0].click()
            self._until(entered.is_set)
            self.assertFalse(self.view.task_buttons[0].isEnabled())
            self.view.fields["host"].setText("192.0.2.35")
            self.assertEqual(self.view.target_label.text(), "192.168.1.100:22")
            self.view.stop_button.click()
            self._until(lambda: self.view.status_label.text() == "机器人已确认空闲")

        self.assertEqual(stops, [("192.168.1.100:22", password)])
        self.assertIsNone(self.window._stop_request)
        self.assertIsNone(self.window._stop_wait)
        self.assertIsNone(self.window._motion)

    def test_stop_rechecks_an_already_idle_receipt_that_races_start(self):
        entered = Event()
        started_late = Event()
        stopped = Event()
        self.addCleanup(stopped.set)
        stops = []

        class Api:
            def __init__(self, request):
                self.request = request

            def status(self):
                reply = _status()
                if started_late.is_set() and not stopped.is_set():
                    reply["task_active"] = True
                    reply["owner"]["mode"] = "auto"
                return reply

            def start(self, command, *, cancel_event):
                entered.set()
                stopped.wait(3)
                return {"ok": False, "status": "not_started_or_stopped"}

            def stop(self):
                stops.append(self.request.target)
                if len(stops) == 1:
                    started_late.set()
                    return {"ok": True, "status": "already_idle"}
                stopped.set()
                return {"ok": True, "status": "stop_requested"}

        self.view.password.setText("test-credential")
        with patch.object(control_window, "_robot_api", side_effect=lambda request, log: Api(request)):
            self.view.connection_button.click()
            self._until(lambda: self.view.task_buttons[2].isEnabled())
            self.view.task_buttons[2].click()
            self._until(entered.is_set)
            self.view.stop_button.click()
            self._until(lambda: stopped.is_set() and self.window._stop_wait is None
                        and self.window._stop_request is None and self.window._motion is None)

        self.assertEqual(stops, ["192.168.1.100:22", "192.168.1.100:22"])

    def test_stopped_loaded_task_waits_for_drain_and_explicit_empty_reset(self):
        running = Event()
        stopped = Event()
        drained = Event()
        reset = Event()
        self.addCleanup(stopped.set)
        starts = []

        def start(command, *, cancel_event):
            starts.append(command)
            if len(starts) == 1:
                running.set()
                stopped.wait(3)
                return {"ok": False, "status": "stopped_no_further_auto_macros"}
            return {"ok": True, "status": "task_complete"}

        def stop():
            stopped.set()
            return {"ok": True, "status": "stop_requested"}

        def status():
            reply = _status()
            if stopped.is_set() and not reset.is_set():
                reply["owner"].update(in_flight=not drained.is_set(), restart_requires_empty_reset=True)
            return reply

        def reset_empty(*, cancel_event):
            reset.set()
            return {"ok": True, "status": "stand_request_completed"}

        api = SimpleNamespace(start=start, stop=stop, status=status, reset_empty=reset_empty)
        self.view.password.setText("test-credential")
        with patch.object(control_window, "_robot_api", return_value=api):
            self.view.connection_button.click()
            self._until(lambda: self.view.task_buttons[0].isEnabled())
            self.view.task_buttons[0].click()
            self._until(running.is_set)
            self.view.stop_button.click()
            self._until(lambda: self.window._stop_wait is not None and self.window._needs_empty_reset())
            self.assertFalse(self.view.reset_empty_button.isEnabled())
            self.assertFalse(self.view.task_buttons[0].isEnabled())
            drained.set()
            self._until(self.view.reset_empty_button.isEnabled)
            with patch.object(self.window, "_confirm", return_value=False):
                self.view.reset_empty_button.click()
            self.assertFalse(reset.is_set())
            self.assertFalse(self.view.task_buttons[0].isEnabled())
            with patch.object(self.window, "_confirm", return_value=True):
                self.view.reset_empty_button.click()
            self._until(lambda: self.view.task_buttons[0].isEnabled())
            self.assertEqual(starts, [0])
            self.view.task_buttons[0].click()
            self._until(lambda: starts == [0, 0] and self.window._motion is None)

    def test_loaded_rejection_shows_reset_before_future_poll_and_discards_older_poll(self):
        entered = Event()
        reject = Event()
        stale_entered = Event()
        stale_released = Event()
        refresh = Event()
        for event in (reject, stale_released, refresh):
            self.addCleanup(event.set)
        polls = []
        reset = []

        def status():
            polls.append(None)
            if len(polls) == 2:
                stale_entered.set()
                stale_released.wait(3)
            elif len(polls) > 2:
                refresh.wait(3)
            return _status()

        def start(command, *, cancel_event):
            entered.set()
            reject.wait(3)
            owner = dict(_status()["owner"], restart_requires_empty_reset=True)
            return {"ok": False, "status": "not_started_or_stopped", "accepted": False,
                    "reason": "owner rejected start", "observed_status": {"event": "error", "diagnostic": owner}}

        def reset_empty(*, cancel_event):
            reset.append(cancel_event)

        api = SimpleNamespace(status=status, start=start, reset_empty=reset_empty)
        self.window._poll_timer.stop()
        self.view.password.setText("test-credential")
        with patch.object(control_window, "_robot_api", return_value=api):
            self.view.connection_button.click()
            self._until(lambda: self.view.task_buttons[1].isEnabled())
            self.assertTrue(self.view.reset_empty_button.isVisible())
            self.assertFalse(self.view.reset_empty_button.isEnabled())
            self.view.task_buttons[1].click()
            self._until(entered.is_set)
            self.window._monitor()
            self._until(stale_entered.is_set)
            reject.set()
            self._until(self.view.reset_empty_button.isEnabled)
            self.assertTrue(self.view.reset_empty_button.isVisible())
            self.assertIn("需恢复空载站姿", self.view.status_label.text())
            self.assertIn("固定或移走方块", self.view.log.toPlainText())
            self.assertFalse(self.view.task_buttons[1].isEnabled())
            stale_released.set()
            self._until(lambda: len(polls) == 3)
            with patch.object(self.window, "_confirm", return_value=False):
                self.view.reset_empty_button.click()
            self.assertFalse(reset)
            self.assertTrue(self.view.reset_empty_button.isEnabled())
            refresh.set()
            self._until(lambda: self.view.task_buttons[1].isEnabled())

    def test_loaded_reset_waits_for_mcu_handshake(self):
        ready = Event()
        calls = []

        def status():
            reply = _status()
            reply["owner"].update(restart_requires_empty_reset=True, mcu_ready=ready.is_set())
            return reply

        def reset_empty(*, cancel_event):
            calls.append(cancel_event)

        self.view.password.setText("test-credential")
        with patch.object(control_window, "_robot_api", return_value=SimpleNamespace(
                status=status, reset_empty=reset_empty)):
            self.view.connection_button.click()
            self._until(lambda: self.window._link == "connected" and self.window._needs_empty_reset())
            self.assertTrue(self.view.reset_empty_button.isVisible())
            self.assertFalse(self.view.reset_empty_button.isEnabled())
            self.assertIn("等待 MCU 握手恢复", self.view.task_hint.text())
            self.view.reset_empty_button.click()
            self.assertFalse(calls)
            ready.set()
            self._until(self.view.reset_empty_button.isEnabled)
            self.assertFalse(calls)

    def test_combined_start_without_receipt_is_held_for_status(self):
        release_status = Event()
        self.addCleanup(release_status.set)

        class Api:
            def update_then_start(self, command, *, cancel_event):
                return {
                    "ok": False,
                    "deployment": {"ok": True},
                    "start": {"ok": False, "status": "start_unconfirmed", "accepted": None},
                }

            def status(self):
                release_status.wait(3)
                return _status()

        self.view.password.setText("test-credential")
        with patch.object(control_window, "_robot_api", return_value=Api()):
            with patch.object(self.window, "_confirm", return_value=True):
                self.view.update_start_button.click()
            self._until(lambda: self.window._uncertain is not None)
            self.assertFalse(self.view.update_start_button.isEnabled())
            release_status.set()
            self._until(lambda: self.window._uncertain is None)
        self.assertTrue(self.view.update_start_button.isEnabled())

    def test_camera_preview_draws_and_closes(self):
        jpeg = QBuffer()
        jpeg.open(QIODevice.OpenModeFlag.WriteOnly)
        picture = QImage(320, 180, QImage.Format.Format_RGB32)
        picture.fill(QColor("blue"))
        self.assertTrue(picture.save(jpeg, "JPEG"))

        def frames(_request, stop):
            yield SimpleNamespace(jpeg=bytes(jpeg.data()))
            stop.wait(3)

        self.view.password.setText("test-credential")
        with patch.object(control_window, "_chest_frames", side_effect=frames):
            self.view.camera_button.click()
            self._until(lambda: self.window._last_frame_at is not None)
            self.assertIn("320 × 180", self.view.camera_status.text())
            self.view.camera_button.click()
            self._until(lambda: self.window._camera is None)

        self.assertEqual(self.view.camera_status.text(), "预览已关闭")
        self.assertIsNone(self.window._last_frame_at)

    def test_connect_auto_opens_camera_once_and_honors_manual_switch(self):
        def frames(_request, stop):
            stop.wait(3)
            return ()

        self.view.password.setText("test-credential")
        api = SimpleNamespace(status=_status)
        with patch.object(control_window, "_robot_api", return_value=api):
            with patch.object(control_window, "_chest_frames", side_effect=frames) as camera:
                self.view.connection_button.click()
                self._until(lambda: self.window._camera is not None and camera.call_count == 1)
                self.assertEqual(self.view.connection_label.text(), "已连接")
                self.view.camera_button.click()
                self._until(lambda: self.window._camera is None)

                # A fresh read-only status result must not override a manual close.
                self._until(lambda: self.window._probe is None)
                self.window._monitor()
                self._until(lambda: self.window._probe is None)
                self.assertEqual(camera.call_count, 1)
                self.assertEqual(self.view.camera_status.text(), "预览已关闭")

                self.view.camera_button.click()
                self._until(lambda: self.window._camera is not None and camera.call_count == 2)
                self.view.camera_button.click()
                self._until(lambda: self.window._camera is None)

    def test_ssh_page_deploys_with_current_paths(self):
        self.view.password.setText("test-credential")
        self.view.tabs.setCurrentIndex(1)
        with patch.object(control_window, "operate", return_value={"status": "deployed"}) as deploy:
            with patch.object(self.window, "_confirm", return_value=True):
                self.view.pi_buttons["deploy"].click()
            self._until(lambda: self.view.status_label.text() == "部署树莓派程序完成")
        self.assertEqual(deploy.call_args.args[:3], ("deploy", settings.DEFAULTS, "test-credential"))

    def test_deployment_source_switch_keeps_both_addresses_and_saves_selection(self):
        url = "https://example.test/releases/pi_deploy_bundle"
        self.view.tabs.setCurrentIndex(1)
        source = self.view.deployment_source
        remote = self.view.fields["remote_url"]
        local = self.view.fields["local_path"]

        self.assertEqual(source.currentData(), "remote")
        self.assertTrue(remote.isEnabled())
        self.assertFalse(local.isEnabled())
        self.assertFalse(self.view.local_browse_button.isEnabled())

        remote.setText(url)
        source.setCurrentIndex(source.findData("local"))
        local.setText(r"D:\R3Deploy\bundle")
        self.assertFalse(remote.isEnabled())
        self.assertTrue(local.isEnabled())
        self.assertTrue(self.view.local_browse_button.isEnabled())

        with patch.object(settings, "save") as save:
            self.view.save_button.click()
        saved = save.call_args.args[0]
        self.assertEqual(saved["deployment_source"], "local")
        self.assertEqual(saved["remote_url"], url)
        self.assertEqual(saved["local_path"], r"D:\R3Deploy\bundle")

        restored = qt_view.ControlView(saved)
        self.addCleanup(restored.deleteLater)
        self.assertEqual(restored.deployment_source.currentData(), "local")
        self.assertEqual(restored.fields["remote_url"].text(), url)
        source.setCurrentIndex(source.findData("remote"))
        self.assertEqual(remote.text(), url)
        self.assertFalse(local.isEnabled())

    def test_both_deployment_actions_show_the_selected_source(self):
        url = "https://example.test/releases/pi_deploy_bundle"
        local_bundle = r"D:\R3Deploy\bundle"
        self.view.password.setText("test-credential")
        self.view.tabs.setCurrentIndex(1)
        self.view.fields["remote_url"].setText(url)
        self.view.fields["local_path"].setText(local_bundle)
        source_field = self.view.deployment_source

        for source, address, button in (
                ("remote", url, self.view.pi_buttons["deploy"]),
                ("local", local_bundle, self.view.update_start_button)):
            with self.subTest(source=source):
                source_field.setCurrentIndex(source_field.findData(source))
                with patch.object(self.window, "_confirm", return_value=True) as confirm, \
                        patch.object(self.window, "_prepare_deployment") as prepare:
                    button.click()
                self.assertIn(address, confirm.call_args.args[1])
                self.assertEqual(prepare.call_args.args[0].values["deployment_source"], source)

    def test_deploy_registers_the_detected_usb_on_the_same_click(self):
        self.view.password.setText("test-credential")
        self.view.tabs.setCurrentIndex(1)
        self.application.processEvents()
        controls = [self.view.save_button, self.view.route_edit_button, *self.view.pi_buttons.values(),
                    self.view.update_task, self.view.update_start_button]
        self.assertLessEqual(max(w.geometry().center().y() for w in controls)
                             - min(w.geometry().center().y() for w in controls), 2)
        with TemporaryDirectory() as directory:
            receipt = Path(directory) / "readback.json"
            receipt.write_text("{}", encoding="utf-8")
            verified = {"receipt_path": str(receipt), "drive": str(Path(directory) / "ejected")}
            with patch.object(control_window.usb_mode, "McuReadback_VerifyConnected", return_value=verified), \
                    patch.object(control_window, "_robot_api", return_value=SimpleNamespace(status=_status)), \
                    patch.object(control_window, "operate", return_value={"status": "deployed",
                                                                         "mcu_readback_registered": True}) as deploy, \
                    patch.object(self.window, "_confirm", return_value=True) as confirm:
                self.view.pi_buttons["deploy"].click()
                self._until(lambda: self.view.status_label.text() == "部署树莓派程序完成")
            self.assertEqual(confirm.call_count, 2)
            self.assertEqual(deploy.call_args.kwargs["mcu_readback"], receipt)
            self.assertIn("MCU 读回已登记", self.view.log.toPlainText())
            self.assertIsNone(self.window._mcu_receipt)


if __name__ == "__main__":
    unittest.main()
