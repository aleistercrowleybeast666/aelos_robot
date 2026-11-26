"""Build a deterministic, shareable copy of a local R3 console task report."""

from dataclasses import dataclass
from datetime import datetime
import hashlib
import json
from pathlib import Path
import re


class ReportPayloadError(ValueError):
    """The selected file cannot be shared as an R3 task report."""


class ReportPayloadUnavailable(ReportPayloadError):
    """The report may become readable again after a local or shared drive recovers."""


@dataclass(frozen=True)
class ReportPayload:
    relative_path: str
    data: bytes
    blob_sha: str


_SCHEMA = "r3.console.task-report.v1"
_MAX_BYTES = 16 * 1024 * 1024
_SECRET_FIELD = re.compile(
    r"password|passwd|passphrase|credential|secret|privatekey|apikey|authorization|"
    r"cookie|dpapi|token$|^auth$|^pwd$|sshkey"
)
_SECRET_TEXT = re.compile(
    r"-----BEGIN [^\r\n]*PRIVATE KEY-----|github_pat_[A-Za-z0-9_]+|"
    r"\bgh[pousr]_[A-Za-z0-9_]+|\b(?:bearer|basic)\s+[a-z0-9+/=._-]+|"
    r"://[^/\s@]+:[^/\s@]*@|"
    r"(?:password|passwd|passphrase|secret|token|api[_-]?key|"
    r"authorization|cookie)[\x22']?\s*[:=]|sshpass\s+-p\b",
    re.IGNORECASE,
)
_HIDDEN = "[凭据已隐藏]"
_LEGACY_SCAN = re.compile(r"([0-9]{8}_[0-9]{6}_[0-9]{6})_r3_entrance_reference\.json")
_LEGACY_PI = re.compile(r"pi_([0-9]{8}_[0-9]{6})_[0-9a-f]{8}\.json")


def _clean(value):
    if isinstance(value, dict):
        clean = {}
        for key, entry in value.items():
            compact_key = re.sub(r"[^a-z0-9]", "", key.casefold())
            if not _SECRET_FIELD.search(compact_key) and not _SECRET_TEXT.search(key):
                clean[key] = _clean(entry)
        return clean
    if isinstance(value, list):
        return [_clean(entry) for entry in value]
    if isinstance(value, str) and _SECRET_TEXT.search(value):
        return _HIDDEN
    return value


def _legacy_record(record, filename):
    if not isinstance(record, dict) or "schema" in record:
        raise ReportPayloadError("文件不是 R3 控制台任务报告。")
    if scan := _LEGACY_SCAN.fullmatch(filename):
        if (record.get("type") != "AR_TAG_ALVAR" or record.get("scan_only") is not True
                or not isinstance(record.get("status"), str)
                or not all(isinstance(record.get(key), list) for key in ("observations", "ids", "observed_ids"))
                or not all(type(tag) is int for key in ("ids", "observed_ids") for tag in record[key])
                or not all(isinstance(obs, dict) for obs in record["observations"])):
            raise ReportPayloadError("旧式入口扫码报告的内容与格式不符。")
        stamp = datetime.strptime(scan[1], "%Y%m%d_%H%M%S_%f")
        command = "scan"
    elif trial := _LEGACY_PI.fullmatch(filename):
        diag = record.get("diagnostic")
        events = record.get("events")
        if (record.get("event") != "done" or not isinstance(record.get("status"), str)
                or not isinstance(record.get("reason"), str) or type(record.get("native_immediate_stop")) is not bool
                or type(record.get("elapsed_s")) not in (int, float)
                or not isinstance(diag, dict) or diag.get("schema") != "r3.simple.v1" or diag.get("robot_id") != 3
                or not isinstance(diag.get("micro_trial"), dict) or not isinstance(events, list)
                or not all(isinstance(event, dict) for event in events)
                or not any(isinstance(event.get("stage_timing"), dict)
                           and event["stage_timing"].get("stage") == "micro_capability" for event in events)):
            raise ReportPayloadError("旧式树莓派试跑报告的内容与格式不符。")
        stamp = datetime.strptime(trial[1], "%Y%m%d_%H%M%S")
        command = "micro_trial"
    else:
        raise ReportPayloadError("文件不是 R3 控制台任务报告。")
    # The old filenames contain local wall time without a timezone. Keep that
    # precision and timezone ambiguity identical on every teammate's computer.
    return {"schema": _SCHEMA, "received_at": stamp.isoformat(), "command": command, "result": record}


def ReportPayload_Load(path):
    source = Path(path)
    if source.suffix.lower() != ".json":
        raise ReportPayloadError("请选择本机保存的任务报告 JSON 文件。")
    try:
        with source.open("rb") as stream:
            raw = stream.read(_MAX_BYTES + 1)
        if len(raw) > _MAX_BYTES:
            raise ReportPayloadError("单份任务报告超过 16 MiB。")
        record = json.loads(raw.decode("utf-8-sig"))
        if not isinstance(record, dict) or record.get("schema") != _SCHEMA:
            record = _legacy_record(record, source.name)
        if not isinstance(record.get("result"), dict):
            raise ReportPayloadError("任务报告缺少结果。")
        stamp = datetime.fromisoformat(record["received_at"].replace("Z", "+00:00"))
        command = record["command"]
        if type(command) not in (int, str) or not re.fullmatch(r"[a-zA-Z0-9_-]{1,32}", str(command)):
            raise ReportPayloadError("任务报告的命令标识无效。")
        kept = ("schema", "received_at", "command", "after_update", "result", "summary_zh")
        shared = _clean({key: record[key] for key in kept if key in record})
        data = (json.dumps(shared, ensure_ascii=False, sort_keys=True, indent=2,
                           allow_nan=False) + "\n").encode("utf-8")
    except ReportPayloadError:
        raise
    except FileNotFoundError as exc:
        try:
            directory_available = source.parent.is_dir()
        except OSError:
            directory_available = False
        if getattr(exc, "winerror", None) in (None, 2, 3) and directory_available:
            raise ReportPayloadError(f"任务报告已被移动或删除：{source.name}") from None
        raise ReportPayloadUnavailable("任务报告所在目录暂时无法访问。") from None
    except OSError:
        raise ReportPayloadUnavailable(f"暂时无法读取任务报告 {source.name}。") from None
    except (ValueError, TypeError, KeyError, AttributeError, RecursionError):
        raise ReportPayloadError(f"无法读取或解析任务报告 {source.name}。") from None
    digest = hashlib.sha256(data).hexdigest()
    name = f"{stamp:%Y%m%d_%H%M%S}_cmd{command}_{digest}.json"
    git_object = f"blob {len(data)}\0".encode("ascii") + data
    return ReportPayload(f"reports/{stamp:%Y/%m/%d}/{name}", data, hashlib.sha1(git_object).hexdigest())
