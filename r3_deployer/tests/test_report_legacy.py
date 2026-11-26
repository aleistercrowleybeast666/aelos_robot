import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from r3_deployer.report_payload import ReportPayloadError, ReportPayload_Load


class LegacyReportTests(unittest.TestCase):
    def setUp(self):
        temp = TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.folder = Path(temp.name)
        self.scan_name = "20261020_091500_012345_r3_entrance_reference.json"
        self.scan = dict(status="timeout", type="AR_TAG_ALVAR", scan_only=True,
                         observations=[], ids=[], observed_ids=[])
        self.pi_name = "pi_20261020_092000_0f0fabcd.json"
        self.pi = dict(event="done", status="stopped_no_further_auto_macros", elapsed_s=3.50,
                       reason="synthetic", native_immediate_stop=False,
                       diagnostic={"robot_id": 3, "schema": "r3.simple.v1", "micro_trial": {}},
                       events=[{"stage_timing": {"stage": "micro_capability"}}])

    def read(self, name, payload):
        file = self.folder / name
        file.write_text(json.dumps(payload), encoding="utf-8")
        return json.loads(ReportPayload_Load(file).data)

    def test_recognizes_both_legacy_shapes_and_redacts_contents(self):
        scan = self.read(self.scan_name, dict(self.scan, ssh_password="fake"))
        self.assertEqual(scan["received_at"], "2026-10-20T09:15:00.012345")
        self.assertEqual(scan["command"], "scan")
        self.assertNotIn("ssh_password", scan["result"])
        trial = self.read(self.pi_name, self.pi)
        self.assertEqual(trial["received_at"], "2026-10-20T09:20:00")
        self.assertEqual(trial["command"], "micro_trial")
        self.assertEqual(trial["result"]["events"], self.pi["events"])

    def test_requires_both_matching_filename_and_matching_schema(self):
        for name, content in [(self.pi_name, self.scan), (self.scan_name, self.pi),
                              ("settings.json", self.pi), (self.pi_name, dict(self.pi, diagnostic={})),
                              (self.scan_name, dict(self.scan, schema="alien")),
                              (self.pi_name, dict(self.pi, events=[])),
                              (self.scan_name, dict(self.scan, observed_ids=["1"])),
                              (self.scan_name.replace("1020", "1320"), self.scan)]:
            with self.subTest(name=name), self.assertRaises(ReportPayloadError):
                self.read(name, content)
