import base64
import hashlib
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

from r3_deployer import report_upload as upload
from r3_deployer.report_payload import ReportPayload_Load


class MemoryGitHub:
    def __init__(self):
        self.refs = {"main": "c0"}
        self.trees = {"t0": {"README.md": "original"}}
        self.commits = {"c0": {"parents": [], "tree": {"sha": "t0"}}}
        self.blobs = {}
        self.calls = []
        self.default = "main"
        self.before_publish = None

    def request(self, method, path="", payload=None):
        self.calls.append((method, path, payload))
        if (method, path) == ("GET", ""):
            return {"default_branch": self.default}
        if method == "GET" and path.startswith("/git/ref/heads/"):
            branch = path.split("/")[-1]
            if branch not in self.refs:
                raise upload._HttpFailure(404)
            return {"object": {"sha": self.refs[branch]}}
        if method == "GET" and path.startswith("/contents/"):
            name, head = path[len("/contents/"):].split("?ref=")
            tree = self.trees[self.commits[head]["tree"]["sha"]]
            if name not in tree:
                raise upload._HttpFailure(404)
            return {"type": "file", "sha": tree[name]}
        if (method, path) == ("POST", "/git/blobs"):
            raw = base64.b64decode(payload["content"])
            sha = hashlib.sha1(f"blob {len(raw)}\0".encode() + raw).hexdigest()
            self.blobs[sha] = raw
            return {"sha": sha}
        if method == "GET" and path.startswith("/git/commits/"):
            return self.commits[path.split("/")[-1]]
        if (method, path) == ("POST", "/git/trees"):
            sha = "t" + str(len(self.trees))
            tree = self.trees.get(payload.get("base_tree"), {}).copy()
            tree.update({entry["path"]: entry["sha"] for entry in payload["tree"]})
            self.trees[sha] = tree
            return {"sha": sha}
        if (method, path) == ("POST", "/git/commits"):
            sha = "c" + str(len(self.commits))
            self.commits[sha] = dict(payload, tree={"sha": payload["tree"]})
            return {"sha": sha}
        if method in ("POST", "PATCH") and path.startswith("/git/refs"):
            if self.before_publish:
                callback, self.before_publish = self.before_publish, None
                callback()
            if method == "POST":
                assert payload["ref"] == "refs/heads/robot-logs"
                if "robot-logs" in self.refs:
                    raise upload._HttpFailure(422)
            else:
                assert path == "/git/refs/heads/robot-logs" and payload["force"] is False
                if self.refs["robot-logs"] not in self.commits[payload["sha"]]["parents"]:
                    raise upload._HttpFailure(422)
            self.refs["robot-logs"] = payload["sha"]
            return {}
        raise AssertionError((method, path))

    def files(self):
        return self.trees[self.commits[self.refs["robot-logs"]]["tree"]["sha"]]


class ReportUploadTests(unittest.TestCase):
    def setUp(self):
        tmp = TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.folder = Path(tmp.name)
        self.github = MemoryGitHub()
        mock = patch.object(upload, "_GitHub", return_value=self.github)
        self.client = mock.start()
        self.addCleanup(mock.stop)

    def report(self, name, result):
        path = self.folder / name
        path.write_text(json.dumps({"schema": "r3.console.task-report.v1", "command": 1,
                        "received_at": "2026-11-26T12:01:02.123+08:00",
                        "target": "private-network:22", "result": result}), encoding="utf-8")
        return path

    def push(self, *files):
        return upload.ReportUpload_Push(files, "https://github.com/team/r3.git", "test-token")

    def test_first_batch_creates_one_independent_commit_and_deduplicates_retries(self):
        first = self.report("one.json", {"status": "completed"})
        second = self.report("two.json", {"status": "cancelled"})
        result = self.push(first, second, first)
        self.assertEqual((result.uploaded, result.skipped), (2, 1))
        self.assertEqual(result.branch_url, "https://github.com/team/r3/tree/robot-logs")
        self.assertEqual(len(self.github.files()), 2)
        self.assertTrue(all(p.startswith("reports/2026/11/26/20261126_120102_cmd1_")
                            for p in self.github.files()))
        head = self.github.refs["robot-logs"]
        self.assertEqual(self.github.commits[head]["parents"], [])
        self.assertEqual(self.github.refs["main"], "c0")
        first.rename(self.folder / "renamed.json")
        result = self.push(self.folder / "renamed.json", second)
        self.assertEqual((result.uploaded, result.skipped), (0, 2))
        self.assertEqual(self.github.refs["robot-logs"], head)

    def test_redacts_credentials_and_keeps_source(self):
        path = self.report("one.json", {"steps": 12, "nested": [{"sshPassword": "secret1",
                    "github_token": "secret2", "echo": "Authorization: Bearer secret3",
                    "private-key": "secret4", "line": "https://user:secret5@example.org/path",
                    "debug": "-----BEGIN OPENSSH PRIVATE KEY-----secret6",
                    "details": "github_pat_secret7"}]})
        self.push(path)
        archived = next(iter(self.github.blobs.values())).decode()
        for word in ["secret", "private-network"]:
            self.assertNotIn(word, archived)
        self.assertIn('"steps": 12', archived)
        self.assertIn("secret4", path.read_text(encoding="utf-8"))

    def test_collision_with_unexpected_contents_never_overwrites(self):
        file = self.report("one.json", {"ok": True})
        self.push(file)
        self.github.files()[ReportPayload_Load(file).relative_path] = "other"
        previous = self.github.refs.copy()
        with self.assertRaisesRegex(upload.ReportUploadError, "不同"):
            self.push(file)
        self.assertEqual(self.github.refs, previous)

    def test_race_on_first_creation_and_later_update_preserves_both_writers(self):
        for i in range(2):
            first = self.report("one.json", {"i": i})
            second = self.report("two.json", {"other": i})
            self.github.before_publish = lambda: self.push(second)
            self.assertEqual(self.push(first).uploaded, 1)
            self.assertEqual(len(self.github.files()), (i + 1) * 2)
        self.assertEqual(self.github.refs["main"], "c0")

    def test_concurrent_identical_report_is_skipped(self):
        first = self.report("one.json", {"same": True})
        self.github.before_publish = lambda: self.push(first)
        self.assertEqual(self.push(first).skipped, 1)
        self.assertEqual(len(self.github.files()), 1)

    def test_rejects_nonreport_before_any_network_and_unsafe_urls(self):
        first = self.report("one.json", {"ok": False})
        bad = self.folder / "settings.json"
        bad.write_text('{"password":"excluded"}', encoding="utf-8")
        with self.assertRaises(upload.ReportUploadError):
            self.push(first, bad)
        self.client.assert_not_called()
        for bad_repo in ("https://github.com.evil/a/b", "https://ghp_secret@github.com/a/b",
                         "https://github.com/a/b?ref=main", "team/.."):
            with self.assertRaises(upload.ReportUploadError):
                upload.ReportUpload_NormalizeRepository(bad_repo)

    def test_empty_and_default_log_branch_repositories_remain_untouched(self):
        first = self.report("one.json", {"ok": True})
        self.github.refs.clear()
        with self.assertRaisesRegex(upload.ReportUploadError, "初始化"):
            self.push(first)
        self.github.default = "robot-logs"
        with self.assertRaisesRegex(upload.ReportUploadError, "默认分支"):
            self.push(first)
        self.assertFalse(self.github.blobs)

    def test_denied_permissions_and_rate_limits_are_actionable(self):
        first = self.report("one.json", {"ok": False})
        for failure, message in [(upload._HttpFailure(401), "过期"), (upload._HttpFailure(403), "读写权限"),
                                 (upload._HttpFailure(403, True), "额度")]:
            with patch.object(self.github, "request", side_effect=failure):
                with self.assertRaisesRegex(upload.ReportUploadError, message) as ctx:
                    self.push(first)
                self.assertNotIn("test-token", str(ctx.exception))
