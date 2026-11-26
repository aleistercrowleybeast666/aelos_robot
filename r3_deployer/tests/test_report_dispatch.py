from pathlib import Path
from tempfile import TemporaryDirectory
from threading import Event, current_thread, main_thread
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from r3_deployer import report_delivery as delivery, report_dispatch as dispatch


class ReportDispatchTests(unittest.TestCase):
    def setUp(self):
        temporary = TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.folder = Path(temporary.name)
        for patcher in (patch.object(delivery.settings, "settings_path", return_value=self.folder / "settings.json"),
                        patch.object(delivery.settings, "_dpapi", side_effect=lambda value, encrypt: value)):
            patcher.start()
            self.addCleanup(patcher.stop)
        delivery.ReportDelivery_Configure("team/r3", "synthetic-token")
        self.worker = dispatch.ReportDispatcher()
        self.addCleanup(self.close_worker)

    def close_worker(self):
        self.worker.ReportDispatch_Close()
        self.worker._thread.join(3)
        self.assertFalse(self.worker._thread.is_alive())

    def enqueue_dormant(self, *names):
        for name in names:
            delivery.ReportDelivery_Enqueue(self.folder / (name + ".json"))

    def wait_empty(self):
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            if delivery.ReportDelivery_Status().pending_count == 0:
                return
            time.sleep(.015)
        self.fail("outbox did not drain")

    def test_missing_and_malformed_reports_do_not_block_other_reports(self):
        self.enqueue_dormant("removed", "malformed", "good")
        seen = []

        def send(paths, *args, **kwargs):
            seen.append(paths[0].stem)
            if seen[-1] != "good":
                raise dispatch.ReportUploadInvalidReport("报告已删除或无法解析")
            return SimpleNamespace(uploaded=1)

        with patch.object(dispatch, "ReportUpload_Push", side_effect=send):
            self.worker.ReportDispatch_Retry()
            self.wait_empty()
        self.assertEqual(seen, ["removed", "malformed", "good"])

    def test_unavailable_source_is_retained_without_blocking_later_batches(self):
        names = ["drive_offline"] + [f"ready{i}" for i in range(13)]
        self.enqueue_dormant(*names)
        seen = []

        def send(paths, *args, **kwargs):
            seen.append(paths[0].stem)
            if seen[-1] == "drive_offline":
                raise dispatch.ReportUploadSourceUnavailable("共享目录暂不可读")
            return SimpleNamespace(uploaded=1)

        with patch.object(dispatch, "ReportUpload_Push", side_effect=send):
            result = self.worker._drain()
        self.assertIs(result, dispatch._DispatchResult.RETRY)
        self.assertCountEqual(seen, names)
        self.assertEqual([p.stem for p in delivery.ReportDelivery_Pending().paths], ["drive_offline"])

    def test_retries_offline_in_background_and_new_reports_wake_single_sender(self):
        self.enqueue_dormant("first")
        sending, release = Event(), Event()
        seen = []

        def send(paths, *args, **kwargs):
            self.assertIsNot(current_thread(), main_thread())
            seen.append(paths[0].stem)
            if len(seen) == 1:
                raise dispatch.ReportUploadError("网络暂不可用")
            if len(seen) == 2:
                sending.set()
                self.assertTrue(release.wait(3))
            return SimpleNamespace(uploaded=1)

        with patch.object(dispatch, "_FIRST_RETRY_SECONDS", .02), patch.object(
                dispatch, "ReportUpload_Push", side_effect=send):
            self.worker.ReportDispatch_Retry()
            self.assertTrue(sending.wait(3))
            self.worker.ReportDispatch_Enqueue(self.folder / "later.json")
            for _ in range(5):
                self.worker.ReportDispatch_Retry()
            self.assertEqual(seen, ["first", "first"])
            release.set()
            self.wait_empty()
        self.assertEqual(seen, ["first", "first", "later"])

    def test_close_during_request_preserves_pending_and_starts_no_more_requests(self):
        self.enqueue_dormant("first", "second")
        sending, release = Event(), Event()

        def send(paths, repo, token, *, still_current):
            sending.set()
            self.assertTrue(release.wait(3))
            self.assertFalse(still_current())
            raise dispatch.ReportUploadCancelled("已关闭")

        with patch.object(dispatch, "ReportUpload_Push", side_effect=send) as push:
            self.worker.ReportDispatch_Retry()
            self.assertTrue(sending.wait(3))
            self.worker.ReportDispatch_Close()
            release.set()
            self.worker._thread.join(3)
            self.assertEqual(push.call_count, 1)
        self.assertEqual(delivery.ReportDelivery_Status().pending_count, 2)

    def test_revoked_authorization_is_checked_before_next_file(self):
        self.enqueue_dormant("first", "second")

        def send(paths, repo, token, *, still_current):
            self.assertTrue(still_current())
            delivery.ReportDelivery_Disable()
            self.assertFalse(still_current())
            raise dispatch.ReportUploadCancelled("已撤销")

        with patch.object(dispatch, "ReportUpload_Push", side_effect=send) as push:
            self.assertIs(self.worker._drain(), dispatch._DispatchResult.IDLE)
        self.assertEqual(push.call_count, 1)
