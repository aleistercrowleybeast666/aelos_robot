"""Append selected task reports to an independent GitHub branch in one commit."""

import base64
from dataclasses import dataclass
from enum import Enum
import json
import re
from urllib.error import HTTPError, URLError
from urllib.parse import quote
from urllib.request import build_opener, HTTPRedirectHandler, Request

from .report_payload import ReportPayload_Load, ReportPayloadError, ReportPayloadUnavailable


DEFAULT_REPORT_REPOSITORY = "aleistercrowleybeast666/aelos_robot"
REPORT_BRANCH = "robot-logs"


class ReportUploadError(RuntimeError):
    """A user-facing failure, without authentication or raw response details."""

class ReportUploadInvalidReport(ReportUploadError):
    """The source is missing or malformed and cannot be uploaded."""


class ReportUploadSourceUnavailable(ReportUploadError):
    """Wait for the file system before retrying this particular report."""


class ReportUploadCancelled(ReportUploadError):
    """Local authorization changed during this upload."""



class ReportUploadResult(Enum):
    UPLOADED = "uploaded"
    ALREADY_PRESENT = "already_present"


@dataclass(frozen=True)
class ReportUploadSummary:
    uploaded: int
    skipped: int
    branch_url: str

    @property
    def status(self):
        return ReportUploadResult.UPLOADED if self.uploaded else ReportUploadResult.ALREADY_PRESENT


class _HttpFailure(Exception):
    def __init__(self, status, limited=False):
        self.status = status
        self.limited = limited


class _NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


def ReportUpload_NormalizeRepository(value):
    value = str(value or "").strip().rstrip("/")
    if value.lower().startswith("https://github.com/"):
        value = value[len("https://github.com/"):]
    if value.endswith(".git"):
        value = value[:-4]
    match = re.fullmatch(r"([a-zA-Z0-9][a-zA-Z0-9-]{0,38})/([a-zA-Z0-9_.-]{1,100})", value)
    if not match or match[2] in (".", ".."):
        raise ReportUploadError("日志仓库请填写 GitHub 的所有者/仓库名，或对应的 HTTPS 仓库网址。")
    return value


class _GitHub:
    def __init__(self, repository, token, still_current=None):
        self.base = "https://api.github.com/repos/" + repository
        self.opener = build_opener(_NoRedirect())
        self.still_current = still_current
        self.headers = {"Authorization": "Bearer " + token, "Accept": "application/vnd.github+json",
                        "Content-Type": "application/json", "User-Agent": "AelosR3Control"}

    def request(self, method, path="", payload=None):
        if self.still_current is not None and not self.still_current():
            raise ReportUploadCancelled("本次自动发送已取消。")
        body = None if payload is None else json.dumps(payload).encode("utf-8")
        req = Request(self.base + path, data=body, headers=self.headers, method=method)
        try:
            with self.opener.open(req, timeout=30) as response:
                return json.load(response)
        except HTTPError as exc:
            limited = exc.headers.get("X-RateLimit-Remaining") == "0" or "Retry-After" in exc.headers
            exc.close()
            raise _HttpFailure(exc.code, limited) from None
        except (URLError, OSError, ValueError):
            raise ReportUploadError("连接 GitHub 或读取响应失败，请检查网络后重新推送。") from None


def _get_head(api):
    try:
        return api.request("GET", "/git/ref/heads/" + REPORT_BRANCH)["object"]["sha"]
    except _HttpFailure as exc:
        if exc.status != 404:
            raise
        return None


def _pending_reports(api, reports, head):
    if head is None:
        return list(reports.values())
    pending = []
    for path, report in reports.items():
        try:
            existing = api.request("GET", f"/contents/{path}?ref={head}")
        except _HttpFailure as exc:
            if exc.status != 404:
                raise
            pending.append(report)
            continue
        if existing.get("type") != "file" or existing.get("sha") != report.blob_sha:
            raise ReportUploadError("日志分支中有同路径但内容不同的文件，已保留远程文件。")
    return pending


def _push(api, reports, selected_count, repository):
    branch_url = f"https://github.com/{repository}/tree/{REPORT_BRANCH}"
    default = api.request("GET")["default_branch"]
    if not default or default.casefold() == REPORT_BRANCH:
        raise ReportUploadError("日志仓库需要一个独立于 robot-logs 的默认分支。")
    head = _get_head(api)
    if head is None:
        try:
            api.request("GET", "/git/ref/heads/" + quote(default, safe="/"))
        except _HttpFailure as exc:
            if exc.status in (404, 409):
                raise ReportUploadError("仓库尚未初始化，请先在 GitHub 创建默认分支。") from None
            raise

    uploaded_blobs = set()
    for _ in range(4):
        pending = _pending_reports(api, reports, head)
        if not pending:
            return ReportUploadSummary(0, selected_count, branch_url)
        tree = {"tree": []}
        for report in pending:
            if report.blob_sha not in uploaded_blobs:
                blob = api.request("POST", "/git/blobs", {
                    "content": base64.b64encode(report.data).decode("ascii"), "encoding": "base64"})
                if blob["sha"] != report.blob_sha:
                    raise ReportUploadError("GitHub 返回的文件校验值不一致，日志分支未更新。")
                uploaded_blobs.add(report.blob_sha)
            tree["tree"].append({"path": report.relative_path, "mode": "100644",
                                 "type": "blob", "sha": report.blob_sha})
        if head:
            tree["base_tree"] = api.request("GET", "/git/commits/" + head)["tree"]["sha"]
        tree_sha = api.request("POST", "/git/trees", tree)["sha"]
        commit = api.request("POST", "/git/commits", {
            "message": f"归档 {len(pending)} 份机器人任务报告", "tree": tree_sha,
            "parents": [head] if head else []})["sha"]
        try:
            if head:
                api.request("PATCH", "/git/refs/heads/" + REPORT_BRANCH, {"sha": commit, "force": False})
            else:
                api.request("POST", "/git/refs", {"ref": "refs/heads/" + REPORT_BRANCH, "sha": commit})
            return ReportUploadSummary(len(pending), selected_count - len(pending), branch_url)
        except _HttpFailure as exc:
            if exc.status not in (409, 422):
                raise
            latest = _get_head(api)
            if latest == head:
                raise
            head = latest
    raise ReportUploadError("其他队员正在同时推送，请稍后重试。已发布的报告会自动跳过。")


def ReportUpload_Push(paths, repository, token, *, still_current=None):
    """Upload only explicitly selected local task reports, with immutable names."""
    repository = ReportUpload_NormalizeRepository(repository)
    token = (token or "").strip()
    if not token or not re.fullmatch(r"[!-~]+", token):
        raise ReportUploadError("请提供用于写入日志仓库的有效 GitHub 令牌。")
    reports = {}
    selected_count = 0
    try:
        for path in paths:
            report = ReportPayload_Load(path)
            reports[report.relative_path] = report
            selected_count += 1
    except ReportPayloadUnavailable as exc:
        raise ReportUploadSourceUnavailable(str(exc)) from None
    except ReportPayloadError as exc:
        raise ReportUploadInvalidReport(str(exc)) from None
    if not reports:
        raise ReportUploadError("请至少选择一份任务报告。")
    try:
        return _push(_GitHub(repository, token, still_current), reports, selected_count, repository)
    except _HttpFailure as exc:
        if exc.limited or exc.status == 429:
            message = "GitHub 请求额度已用尽，请稍后重试。"
        else:
            message = {
                401: "GitHub 凭据无效或已过期。",
                403: "GitHub 拒绝写入，请确认对该仓库拥有 Contents 读写权限，并检查分支规则。",
                404: "找不到仓库或凭据没有访问该仓库的权限。",
                409: "GitHub 返回仓库冲突，请确认仓库已初始化后重试。",
                422: "GitHub 未接受日志提交，请检查仓库权限和分支规则。",
            }.get(exc.status, "GitHub 请求失败，请稍后重试。")
        raise ReportUploadError(f"{message}（HTTP {exc.status}）") from None
    except (KeyError, TypeError, AttributeError):
        raise ReportUploadError("GitHub 返回了无法识别的仓库或提交信息。") from None
