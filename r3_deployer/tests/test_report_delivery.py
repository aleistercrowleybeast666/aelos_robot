from concurrent.futures import ThreadPoolExecutor
from contextlib import closing
import os
from pathlib import Path
import sqlite3
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

from r3_deployer import report_delivery as delivery


class ReportDeliveryTests(unittest.TestCase):
    def setUp(self):
        temporary = TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        settings_path = patch.object(delivery.settings, "settings_path", return_value=self.root / "settings.json")
        settings_path.start()
        self.addCleanup(settings_path.stop)
        if os.name != "nt":
            # Other platforms can exercise the queue; production requires Windows DPAPI.
            fake_protection = patch.object(delivery.settings, "_dpapi", side_effect=lambda data, _: data[::-1])
            fake_protection.start()
            self.addCleanup(fake_protection.stop)
        self.repository = "test-team/Robot"
        self.token = "dummy-token-for-offline-test"
        self.database = self.root / "report_delivery.sqlite3"

    def configure(self):
        return delivery.ReportDelivery_Configure(self.repository, self.token)

    def test_explicit_opt_in_and_secret_binding(self):
        path = self.root / "report.json"
        path.write_text("content-never-persisted-in-queue")
        self.assertFalse(delivery.ReportDelivery_Status().enabled)
        self.assertFalse(delivery.ReportDelivery_Enqueue(path))
        self.assertIsNone(delivery.ReportDelivery_Token(self.repository))
        self.assertIsNone(delivery.ReportDelivery_Pending())
        self.assertEqual(self.configure().pending_count, 0)
        self.assertTrue(delivery.ReportDelivery_Enqueue(path))
        batch = delivery.ReportDelivery_Pending()
        self.assertEqual(batch.paths, (path,))
        self.assertEqual(batch.token, self.token)
        self.assertNotIn(self.token, repr(batch))
        self.assertEqual(delivery.ReportDelivery_Token("https://github.com/test-team/robot.git"), self.token)
        self.assertIsNone(delivery.ReportDelivery_Token("test-team/different"))
        self.assertNotIn(self.token.encode(), self.database.read_bytes())
        self.assertNotIn(path.read_bytes(), self.database.read_bytes())
        self.assertTrue(delivery.ReportDelivery_IsCurrent(batch))

    def test_switch_repository_never_moves_queue_and_prevents_stale_ack(self):
        self.configure()
        path = self.root / "report.json"
        delivery.ReportDelivery_Enqueue(path)
        old = delivery.ReportDelivery_Pending()
        self.assertEqual(delivery.ReportDelivery_Configure("test-team/Other", "other-dummy").pending_count, 0)
        self.assertIsNone(delivery.ReportDelivery_Pending())
        delivery.ReportDelivery_Enqueue(path)
        other = delivery.ReportDelivery_Pending()
        self.assertEqual(other.repository, "test-team/Other")
        self.assertEqual(other.token, "other-dummy")
        self.assertFalse(delivery.ReportDelivery_IsCurrent(old))
        self.configure()
        delivery.ReportDelivery_Acknowledge(old, old.paths)
        delivery.ReportDelivery_Acknowledge(other, other.paths)
        self.assertEqual(delivery.ReportDelivery_Status().pending_count, 1)
        fresh = delivery.ReportDelivery_Pending()
        delivery.ReportDelivery_Acknowledge(fresh, fresh.paths)
        self.assertIsNone(delivery.ReportDelivery_Pending())

    def test_disable_and_reenable_same_repository_is_new_consent(self):
        self.configure()
        path = self.root / "record.json"
        delivery.ReportDelivery_Enqueue(path)
        old = delivery.ReportDelivery_Pending()
        self.assertFalse(delivery.ReportDelivery_Disable().enabled)
        self.assertIsNone(delivery.ReportDelivery_Token(self.repository))
        self.assertIsNone(delivery.ReportDelivery_Pending())
        self.configure()
        self.assertIsNone(delivery.ReportDelivery_Pending())
        delivery.ReportDelivery_Enqueue(path)
        delivery.ReportDelivery_Acknowledge(old, old.paths)
        self.assertEqual(delivery.ReportDelivery_Status().pending_count, 1)

    def test_failed_or_deleted_report_does_not_starve_others(self):
        self.configure()
        paths = [self.root / f"{index}.json" for index in range(4)]
        for path in paths:
            delivery.ReportDelivery_Enqueue(path)
        first = delivery.ReportDelivery_Pending(limit=2)
        self.assertEqual(first.paths, tuple(paths[:2]))
        delivery.ReportDelivery_Acknowledge(first, [paths[1]])
        next_batch = delivery.ReportDelivery_Pending(limit=2)
        self.assertEqual(next_batch.paths, tuple(paths[2:]))
        with self.assertRaises(delivery.ReportDeliveryError):
            delivery.ReportDelivery_Acknowledge(next_batch, [paths[0]])
        delivery.ReportDelivery_Acknowledge(next_batch, next_batch.paths)
        self.assertEqual(delivery.ReportDelivery_Pending().paths, (paths[0],))

    def test_repeated_and_concurrent_enqueue_is_idempotent(self):
        self.configure()
        path = self.root / "report.json"
        with ThreadPoolExecutor(5) as pool:
            self.assertTrue(all(pool.map(delivery.ReportDelivery_Enqueue, [path] * 10)))
        self.assertEqual(delivery.ReportDelivery_Status().pending_count, 1)

    def test_failed_credential_replacement_keeps_previous_settings(self):
        self.configure()
        with patch.object(delivery.settings, "_dpapi", side_effect=OSError("test")):
            with self.assertRaises(delivery.ReportDeliveryError):
                delivery.ReportDelivery_Configure("elsewhere/other", "test")
        self.assertEqual(delivery.ReportDelivery_Token(self.repository), self.token)
        with closing(sqlite3.connect(self.database)) as db, db:
            db.execute("UPDATE authorization SET repository = 'elsewhere/other'")
        with self.assertRaises(delivery.ReportDeliveryError):
            delivery.ReportDelivery_Token("elsewhere/other")
