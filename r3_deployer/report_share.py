"""Publish explicitly selected console reports to a ZIP or shared folder."""

from dataclasses import dataclass
from enum import Enum
import os
from pathlib import Path
import tempfile
from zipfile import ZIP_DEFLATED, ZipFile, ZipInfo

from .report_payload import ReportPayloadError, ReportPayload_Load


class ReportShareResult(Enum):
    WRITTEN = "written"
    ALREADY_SHARED = "already_shared"


@dataclass(frozen=True)
class ReportShareReceipt:
    destination: Path
    report_count: int
    written_count: int
    existing_count: int

    @property
    def result(self):
        return ReportShareResult.WRITTEN if self.written_count else ReportShareResult.ALREADY_SHARED


class ReportShareError(ValueError):
    pass


def _selected_reports(paths):
    reports = {}
    try:
        for path in paths:
            report = ReportPayload_Load(path)
            previous = reports.setdefault(report.relative_path, report)
            if previous.data != report.data:
                raise ReportShareError(f"两个报告生成了相同的文件名：{report.relative_path}")
    except (ReportPayloadError, OSError) as exc:
        raise ReportShareError(str(exc)) from exc
    if not reports:
        raise ReportShareError("请至少选择一份任务报告。")
    return [reports[name] for name in sorted(reports)]


def _files_match(first, second):
    if first.stat().st_size != second.stat().st_size:
        return False
    with first.open("rb") as left, second.open("rb") as right:
        while True:
            block = left.read(256 * 1024)
            if right.read(256 * 1024) != block:
                return False
            if not block:
                return True


def _publish(temporary, destination):
    """Install completed bytes in the same directory without replacing a teammate's file."""
    try:
        if os.name == "nt":
            # Windows rename fails if the name already exists, including on SMB shares.
            os.rename(temporary, destination)
        else:
            os.link(temporary, destination)
        return True
    except FileExistsError:
        if _files_match(temporary, destination):
            return False
        raise ReportShareError(f"目标已存在且内容不同，请另选名称或目录：{destination}") from None


def _write_atomic(destination, writer):
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=destination.parent, prefix=".r3-", suffix=".tmp",
                                         delete=False) as stream:
            temporary = Path(stream.name)
            writer(stream)
            stream.flush()
            os.fsync(stream.fileno())
        return _publish(temporary, destination)
    finally:
        if temporary is not None:
            try:
                temporary.unlink(missing_ok=True)
            except OSError:
                # If the share went offline even cleanup may be unavailable.
                pass


def ReportShare_ExportZip(paths, destination):
    """Write a reproducible archive. An identical retry succeeds without replacement."""
    reports = _selected_reports(paths)
    destination = Path(destination).expanduser()
    if destination.suffix.lower() != ".zip":
        raise ReportShareError("请选择扩展名为 .zip 的导出文件。")

    def write_archive(stream):
        with ZipFile(stream, "w") as archive:
            for report in reports:
                # A fixed ZIP timestamp also makes retrying after midnight idempotent.
                entry = ZipInfo(report.relative_path)
                entry.create_system = 3
                entry.external_attr = 0o600 << 16
                archive.writestr(entry, report.data, compress_type=ZIP_DEFLATED)

    try:
        written = _write_atomic(destination, write_archive)
    except OSError as exc:
        raise ReportShareError(f"无法导出 ZIP：{exc}") from exc
    count = len(reports)
    return ReportShareReceipt(destination, count, count if written else 0, 0 if written else count)


def ReportShare_CopyToFolder(paths, directory):
    """Copy shareable JSON files, resuming incomplete batches on the next send."""
    reports = _selected_reports(paths)
    if not directory:
        raise ReportShareError("请选择共享目录。")
    directory = Path(directory).expanduser()
    written = existing = 0
    try:
        root = directory.resolve()
        for report in reports:
            destination = directory / report.relative_path
            if not destination.resolve().is_relative_to(root):
                raise ReportShareError(f"报告目标超出选定的共享目录：{destination}")
            try:
                with destination.open("rb") as stream:
                    matches = stream.read(len(report.data) + 1) == report.data
            except FileNotFoundError:
                if _write_atomic(destination, lambda stream: stream.write(report.data)):
                    written += 1
                else:
                    existing += 1
            else:
                if not matches:
                    raise ReportShareError(f"目标已存在且内容不同，请检查共享目录：{destination}")
                existing += 1
    except (OSError, ReportShareError) as exc:
        raise ReportShareError(
            f"共享目录写入未完成（已写入 {written} 份，已有 {existing} 份）。可重试同一批报告。{exc}"
        ) from exc
    return ReportShareReceipt(directory, len(reports), written, existing)
