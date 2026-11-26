"""Exercise the report-sharing dialog with temporary settings and offline transports."""

import json
import os
from pathlib import Path
from tempfile import TemporaryDirectory
from threading import Event
import time
import unittest
from unittest.mock import patch
from zipfile import ZipFile

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QSettings, Qt
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

from r3_deployer import control_window, qt_view, report_delivery, report_dispatch, report_window, settings, task_report
from r3_deployer.report_upload import DEFAULT_REPORT_REPOSITORY, ReportUploadSummary


class ReportWindowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        temporary = TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        config = self.root / "settings.json"
        self.enterContext(patch.object(settings, "settings_path", return_value=config))
        self.enterContext(patch.object(task_report, "settings_path", return_value=config))
        self.enterContext(patch.dict(os.environ, {"AELOS_GITHUB_TOKEN": ""}))
        if os.name != "nt":
            self.enterContext(patch.object(settings, "_dpapi", side_effect=lambda data, _: data[::-1]))
        preferences = QSettings(str(self.root / "preferences.ini"), QSettings.Format.IniFormat)
        self.enterContext(patch.object(qt_view, "QSettings", return_value=preferences))
        self.enterContext(patch.object(report_window, "QSettings", return_value=preferences))
        receipt = ReportUploadSummary(1, 0, "https://github.com/example/test/tree/robot-logs")
        self.automatic = self.enterContext(patch.object(report_dispatch, "ReportUpload_Push", return_value=receipt))
        self.manual = self.enterContext(patch.object(report_window, "ReportUpload_Push", return_value=receipt))
        self.window = control_window._ControlWindow(settings.DEFAULTS)
        self.addCleanup(self._close_window)
        self.window.show()
        self.latest = task_report.TaskReport_Save({"ok": True}, command=1, target="test-robot",
                                                 after_update=False)
        self.window._latest_report = self.latest
        self.window.view.share_reports_button.click()
        self.dialog = self.window._report_dialog

    def _close_window(self):
        self.window.report_dispatch.ReportDispatch_Close()
        self.window.report_dispatch._thread.join(timeout=3)
        self.window._poll_timer.stop()
        self.window._event_timer.stop()
        self.window.hide()
        self.window.deleteLater()
        self.app.processEvents()

    def _until(self, condition):
        end = time.monotonic() + 4
        while time.monotonic() < end:
            if condition():
                return
            QTest.qWait(10)
        self.fail("The report-sharing UI did not complete its offline operation.")

    def _enable(self, keypress=False):
        self.dialog.token.setText("test-token")
        if keypress:
            self.dialog.token.setFocus()
            QTest.keyClick(self.dialog.token, Qt.Key.Key_Return)
        else:
            self.dialog.enable.click()
        self._until(lambda: self.automatic.call_count == 1 and report_delivery.ReportDelivery_Status().pending_count == 0)
        self.assertEqual(self.automatic.call_args.args, ([self.latest], DEFAULT_REPORT_REPOSITORY, "test-token"))
        self.assertEqual(self.dialog.token.text(), "")
        self.assertTrue(self.dialog.disable.isEnabled())

    def test_default_recent_report_only_sent_after_return_and_disable_forgets_token(self):
        self.assertEqual(self.dialog.method.currentData(), "github")
        self.assertEqual(self.dialog.repository.text(), DEFAULT_REPORT_REPOSITORY)
        self.assertIn(self.latest.name, self.dialog.selected.text())
        self.manual.assert_not_called()
        self.automatic.assert_not_called()
        self._enable(keypress=True)
        self.assertEqual(report_delivery.ReportDelivery_Token(DEFAULT_REPORT_REPOSITORY), "test-token")
        self.dialog.disable.click()
        self.assertFalse(report_delivery.ReportDelivery_Status().enabled)
        self.assertIsNone(report_delivery.ReportDelivery_Token(DEFAULT_REPORT_REPOSITORY))
        self.assertFalse(self.dialog.disable.isEnabled())
        self.dialog.send.click()
        self.manual.assert_not_called()
        self.assertIn("令牌", self.dialog.feedback.text())

    def test_pasted_pat_enables_saves_and_sends_without_confirm_click(self):
        token = "github_pat_" + "x" * 48
        self.dialog.token.insert(token)
        self._until(lambda: self.automatic.called)
        self.assertEqual(self.dialog.token.text(), "")
        self.assertEqual(report_delivery.ReportDelivery_Token(DEFAULT_REPORT_REPOSITORY), token)
        self.assertEqual(self.automatic.call_args.args[0], [self.latest])

    def test_enable_button_and_historical_selection_require_a_separate_token_for_another_repository(self):
        self._enable()
        history = task_report.TaskReport_Save({"ok": False}, command=0, target="test-robot", after_update=True)
        with patch.object(report_window.QFileDialog, "getOpenFileNames", return_value=([str(history)], "")):
            self.dialog.choose.click()
        self.assertIn(history.name, self.dialog.selected.text())
        self.dialog.repository.setText("test-team/historical")
        self.dialog.send.click()
        self.assertIn("令牌", self.dialog.feedback.text())
        self.assertFalse(self.dialog.busy)
        self.manual.assert_not_called()
        self.dialog.token.setText("second-test-token")
        self.dialog.send.click()
        self._until(lambda: not self.dialog.busy)
        self.manual.assert_called_once_with((history,), "test-team/historical", "second-test-token")
        self.assertEqual(report_delivery.ReportDelivery_Status().repository, DEFAULT_REPORT_REPOSITORY)
        self.assertIsNone(report_delivery.ReportDelivery_Token("test-team/historical"))

    def test_zip_and_shared_folder_export_use_selected_reports_without_credentials(self):
        self.dialog.method.setCurrentIndex(self.dialog.method.findData("zip"))
        destination = self.root / "比赛"
        with patch.object(report_window.QFileDialog, "getSaveFileName", return_value=(str(destination), "")):
            self.dialog.send.click()
        self._until(lambda: not self.dialog.busy)
        self.assertIn("已导出 1 份", self.dialog.feedback.text())
        with ZipFile(destination.with_suffix(".zip")) as archive:
            member, = archive.namelist()
            shared = archive.read(member)
        self.assertNotIn("target", json.loads(shared))
        directory = self.root / "分享"
        self.dialog.method.setCurrentIndex(self.dialog.method.findData("folder"))
        with patch.object(report_window.QFileDialog, "getExistingDirectory", return_value=str(directory)):
            self.dialog.send.click()
        self._until(lambda: not self.dialog.busy)
        self.assertIn("新增 1 份", self.dialog.feedback.text())
        self.assertEqual((directory / member).read_bytes(), shared)
        self.manual.assert_not_called()
        self.automatic.assert_not_called()

    def test_double_click_only_starts_one_background_upload(self):
        started, finish = Event(), Event()

        def delayed(*_):
            started.set()
            finish.wait(4)
            return ReportUploadSummary(1, 0, "")

        self.manual.side_effect = delayed
        self.dialog.token.setText("manual-test-token")
        self.dialog.send.click()
        try:
            self._until(started.is_set)
            self.assertTrue(self.dialog.busy)
            self.assertFalse(self.dialog.choose.isEnabled())
            self.assertFalse(self.dialog.send.isEnabled())
            self.dialog.send.click()
            self.assertEqual(self.manual.call_count, 1)
        finally:
            finish.set()
            self._until(lambda: not self.dialog.busy)
        self.assertTrue(self.dialog.send.isEnabled())
        self.assertFalse(report_delivery.ReportDelivery_Status().enabled)
