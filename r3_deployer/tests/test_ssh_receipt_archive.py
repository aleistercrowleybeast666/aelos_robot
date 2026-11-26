"""Local integration tests for terminal receipts from explicit SSH operations."""

import json
import os
from pathlib import Path
from tempfile import TemporaryDirectory
from threading import Event
import time
import unittest
from unittest.mock import Mock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QSettings
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

from r3_deployer import control_window, qt_view, settings, task_report


class SshReceiptArchiveTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.application = QApplication.instance() or QApplication([])

    def setUp(self):
        temp = TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        config = self.root / "settings.json"
        self.enterContext(patch.object(settings, "settings_path", return_value=config))
        self.enterContext(patch.object(task_report, "settings_path", return_value=config))
        preferences = QSettings(str(self.root / "qt.ini"), QSettings.Format.IniFormat)
        self.enterContext(patch.object(qt_view, "QSettings", return_value=preferences))
        self.dispatch = self.enterContext(patch.object(control_window, "ReportDispatcher")).return_value
        self.api = Mock()
        self.enterContext(patch.object(control_window, "_robot_api", return_value=self.api))
        self.enterContext(patch.object(control_window, "_chest_frames", return_value=()))
        self.enterContext(patch.object(control_window.usb_mode, "McuReadback_VerifyConnected", return_value=None))
        self.operate = self.enterContext(patch.object(control_window, "operate"))
        self.window = control_window._ControlWindow(settings.DEFAULTS)
        self.addCleanup(self._close_window)
        self.window._poll_timer.stop()
        self.enterContext(patch.object(self.window, "_monitor"))
        self.enterContext(patch.object(self.window, "_confirm", return_value=True))
        self.request = control_window._Request(dict(settings.DEFAULTS), "fake-password")
        self.window.view.password.setText(self.request.password)
        self.window._session = self.request
        self.window._link = "connected"
        self.api.status.return_value = {
            "ok": True, "event": "status", "protocol": "r3.warm.v1", "prepared": True,
            "task_active": False, "owner": {"robot_id": 3, "mode": "manual", "in_flight": False},
        }

    def _close_window(self):
        self.window._event_timer.stop()
        self.window.hide()
        self.window.deleteLater()
        self.application.processEvents()

    def _until(self, condition):
        end = time.monotonic() + 4
        while time.monotonic() < end:
            self.window._drain()
            if condition():
                return
            QTest.qWait(10)
        self.fail("The console did not receive the simulated SSH result.")

    def _reports(self):
        return [json.loads(path.read_text(encoding="utf-8")) for path in
                sorted((self.root / "task_reports").glob("*.json"))]

    def test_manual_scan_and_stop_are_archived_but_polling_and_camera_are_excluded(self):
        self.window._start_probe(self.request, "status")
        self._until(lambda: self.window._probe is None)
        self.window._start_camera(self.request)
        self._until(lambda: self.window._camera is None)
        self.assertEqual(self._reports(), [])
        self.api.scan.return_value = {"ok": True, "ids": [3], "observation": "test"}
        self.window._request_scan()
        self._until(lambda: self.window._probe is None)
        self.assertIn("识别到 Tag 3", self.window.view.log.toPlainText())
        self._until(lambda: self.window._latest_report and "_cmdscan_" in self.window._latest_report)
        self.api.stop.return_value = {"ok": True, "status": "already_idle"}
        self.window._request_stop()
        self._until(lambda: self.window._stop_request is None)
        self.assertIs(self.window._stop_wait, self.request)
        self._until(lambda: "_cmdstop_" in self.window._latest_report)
        reports = self._reports()
        self.assertEqual([item["command"] for item in reports], ["scan", "stop"])
        self.assertEqual([item["result"] for item in reports], [self.api.scan.return_value, self.api.stop.return_value])
        self.assertEqual(self.dispatch.ReportDispatch_Enqueue.call_count, 2)
        self.assertEqual(str(self.dispatch.ReportDispatch_Enqueue.call_args.args[0]), self.window._latest_report)

    def test_failed_scan_and_uncertain_stop_keep_original_callbacks_and_sanitize_reports(self):
        self.api.scan.side_effect = control_window.SshLinkError("link down " + self.request.password)
        self.window._request_scan()
        self._until(lambda: self.window._probe is None)
        self.assertEqual(self.window._link, "offline")
        self._until(lambda: self.dispatch.ReportDispatch_Enqueue.call_count == 1)
        self.api.stop.side_effect = control_window.RobotControlUncertainError("unknown stop", accepted=None)
        self.window._request_stop()
        self._until(lambda: self.window._stop_request is None)
        self.assertIsNotNone(self.window._stop_wait)
        self._until(lambda: self.dispatch.ReportDispatch_Enqueue.call_count == 2)
        items = self._reports()
        self.assertEqual([item["result"]["status"] for item in items], ["scan_failed", "stop_unconfirmed"])
        self.assertNotIn(self.request.password, str(items))
        self.assertNotIn(self.request.password, self.window.view.log.toPlainText())
        self.assertEqual(self.dispatch.ReportDispatch_Enqueue.call_count, 2)

    def test_deploy_check_rollback_and_combined_deployment_each_have_one_terminal_report(self):
        self.operate.side_effect = lambda action, *args: {"status": action + "_done", "ok": True}
        for count, action in enumerate(("deploy", "check", "rollback"), start=1):
            self.window._start_maintenance(action)
            self._until(lambda: self.window._maintenance is None)
            self._until(lambda: self.dispatch.ReportDispatch_Enqueue.call_count == count)
        self.api.update_then_start.return_value = {"ok": True, "deployment": {"status": "deployed"},
                                                    "start": {"ok": True, "status": "task_complete"}}
        self.window._start_motion(0, after_update=True)
        self._until(lambda: self.window._maintenance is None and self.window._motion is None)
        items = self._reports()
        self.assertEqual([item["command"] for item in items], ["deploy", "check", "rollback", 0])
        self.assertEqual([item["after_update"] for item in items], [False, False, False, True])
        self.assertEqual(items[-1]["result"], self.api.update_then_start.return_value)
        self.assertEqual(self.dispatch.ReportDispatch_Enqueue.call_count, 4)

    def test_slow_unavailable_archive_does_not_hold_up_stop_receipt(self):
        writing = Event()
        release = Event()
        self.api.stop.return_value = {"ok": True, "status": "already_idle"}

        def offline_folder(*args, **kwargs):
            writing.set()
            release.wait(4)
            raise OSError("disk full")

        with patch.object(control_window, "TaskReport_Save", side_effect=offline_folder):
            self.window._request_stop()
            try:
                self._until(writing.is_set)
                self._until(lambda: self.window._stop_request is None)
                self.assertIs(self.window._stop_wait, self.request)
                self.assertIn("停止请求已送达", self.window.view.status_label.text())
            finally:
                release.set()
            self._until(lambda: "本机操作报告保存失败" in self.window.view.log.toPlainText())
        self.assertEqual(self._reports(), [])
        self.dispatch.ReportDispatch_Enqueue.assert_not_called()
