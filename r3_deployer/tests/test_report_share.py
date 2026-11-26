from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch
from zipfile import ZipFile

from r3_deployer import report_share
from r3_deployer.report_payload import ReportPayload_Load


class ReportShareTests(unittest.TestCase):
    def setUp(self):
        temporary = TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        # Two machines can have the same local report filename.
        self.reports = []
        for index in range(2):
            machine = self.root / str(index)
            machine.mkdir()
            path = machine / "20261126_123456_001000_cmd0_abc123ef.json"
            path.write_text(json.dumps({
                "schema": "r3.console.task-report.v1",
                "received_at": "2026-11-26T12:34:56.001+08:00",
                "command": 0,
                "target": "192.0.2.1:22",
                "after_update": False,
                "result": {"status": "completed", "heading_deg": index, "password": "test-password"},
            }), encoding="utf-8")
            self.reports.append(path)
        self.zip_path = self.root / "队内日志.zip"
        self.share = self.root / "队内共享"

    def test_zip_and_folder_contain_same_selected_sanitized_reports(self):
        (self.reports[0].parent / "settings.json").write_text('{"password":"private"}')
        selection = [*self.reports, self.reports[0]]
        zip_receipt = report_share.ReportShare_ExportZip(selection, self.zip_path)
        folder_receipt = report_share.ReportShare_CopyToFolder(selection, self.share)
        self.assertEqual(zip_receipt.report_count, 2)
        self.assertEqual(folder_receipt.written_count, 2)
        payloads = [ReportPayload_Load(path) for path in self.reports]
        with ZipFile(self.zip_path) as archive:
            self.assertCountEqual(archive.namelist(), [report.relative_path for report in payloads])
            for report in payloads:
                self.assertEqual(archive.read(report.relative_path), report.data)
                self.assertEqual((self.share / report.relative_path).read_bytes(), report.data)
                self.assertNotIn(b"test-password", report.data)
        self.assertIn("test-password", self.reports[0].read_text())

    def test_reversed_selection_is_idempotent_and_existing_files_keep_their_timestamps(self):
        for publish, destination in ((report_share.ReportShare_ExportZip, self.zip_path),
                                     (report_share.ReportShare_CopyToFolder, self.share)):
            first = publish(self.reports, destination)
            files = [self.zip_path] if destination == self.zip_path else list(self.share.rglob("*.json"))
            timestamps = [file.stat().st_mtime_ns for file in files]
            again = publish(reversed(self.reports), destination)
            self.assertEqual(first.result, report_share.ReportShareResult.WRITTEN)
            self.assertEqual(again.result, report_share.ReportShareResult.ALREADY_SHARED)
            self.assertEqual(again.existing_count, 2)
            self.assertEqual(timestamps, [file.stat().st_mtime_ns for file in files])

    def test_existing_different_bytes_are_never_replaced(self):
        report = ReportPayload_Load(self.reports[0])
        shared_file = self.share / report.relative_path
        shared_file.parent.mkdir(parents=True)
        for publish, destination, existing in (
                (report_share.ReportShare_ExportZip, self.zip_path, self.zip_path),
                (report_share.ReportShare_CopyToFolder, self.share, shared_file)):
            existing.write_bytes(b"teammate")
            with self.assertRaises(report_share.ReportShareError):
                publish(self.reports[:1], destination)
            self.assertEqual(existing.read_bytes(), b"teammate")
            self.assertEqual(list(destination.parent.rglob(".r3-*.tmp")), [])

    def test_failed_batch_keeps_originals_and_can_resume(self):
        before = [path.read_bytes() for path in self.reports]
        original_publish = report_share._publish
        def fail_second(temporary, destination):
            if any(self.share.rglob("*.json")):
                raise OSError("模拟共享盘断开")
            return original_publish(temporary, destination)
        with patch.object(report_share, "_publish", side_effect=fail_second):
            with self.assertRaisesRegex(report_share.ReportShareError, "已写入 1 份"):
                report_share.ReportShare_CopyToFolder(self.reports, self.share)
        self.assertEqual(before, [path.read_bytes() for path in self.reports])
        self.assertEqual(list(self.share.rglob("*.tmp")), [])
        resumed = report_share.ReportShare_CopyToFolder(self.reports, self.share)
        self.assertEqual((resumed.written_count, resumed.existing_count), (1, 1))

    def test_unrelated_json_never_leaves_partially_selected_batch(self):
        settings = self.root / "settings.json"
        settings.write_text('{"password":"private"}')
        for publish, destination in ((report_share.ReportShare_ExportZip, self.zip_path),
                                     (report_share.ReportShare_CopyToFolder, self.share)):
            with self.assertRaises(report_share.ReportShareError):
                publish([self.reports[0], settings], destination)
            self.assertFalse(destination.exists())

    def test_simultaneous_senders_commit_each_report_once(self):
        for publish, destination in ((report_share.ReportShare_ExportZip, self.zip_path),
                                     (report_share.ReportShare_CopyToFolder, self.share)):
            with ThreadPoolExecutor(max_workers=5) as pool:
                receipts = list(pool.map(lambda _: publish(self.reports, destination), range(5)))
            self.assertEqual(sum(receipt.written_count for receipt in receipts), 2)
            self.assertEqual(sum(receipt.existing_count for receipt in receipts), 8)
            self.assertEqual(list(self.root.rglob(".r3-*.tmp")), [])
