"""Per-Windows-user opt-in, protected GitHub credential and durable report outbox."""

from contextlib import closing, contextmanager
from dataclasses import dataclass, field
from enum import IntEnum
import json
import os
from pathlib import Path
import re
import sqlite3
from uuid import uuid4

from . import settings
from .report_upload import ReportUploadError, ReportUpload_NormalizeRepository


class ReportDeliveryError(ValueError):
    pass


class ReportDeliveryEnqueueResult(IntEnum):
    DISABLED = 0
    QUEUED = 1


@dataclass(frozen=True)
class ReportDeliveryState:
    enabled: bool
    repository: str
    pending_count: int


@dataclass(frozen=True)
class ReportDeliveryBatch:
    repository: str
    paths: tuple
    token: str = field(repr=False, compare=False)
    generation: str = field(repr=False)


@contextmanager
def _transaction():
    path = settings.settings_path().with_name("report_delivery.sqlite3")
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        with closing(sqlite3.connect(path)) as db, db:
            db.execute("PRAGMA secure_delete = ON")
            db.execute("BEGIN IMMEDIATE")
            db.execute("""CREATE TABLE IF NOT EXISTS authorization
                (id INTEGER PRIMARY KEY CHECK(id=1), repository TEXT NOT NULL,
                 protected BLOB NOT NULL, generation TEXT NOT NULL)""")
            db.execute("""CREATE TABLE IF NOT EXISTS outbox
                (id INTEGER PRIMARY KEY, repository TEXT NOT NULL COLLATE NOCASE,
                 path_key TEXT NOT NULL, path TEXT NOT NULL, turn INTEGER NOT NULL DEFAULT 0,
                 UNIQUE(repository, path_key))""")
            yield db
    except (sqlite3.Error, OSError) as exc:
        raise ReportDeliveryError(f"无法读写自动发送设置和待传队列：{exc}") from exc


def _state(db):
    row = db.execute("SELECT repository FROM authorization WHERE id = 1").fetchone()
    if row is None:
        return ReportDeliveryState(False, "", 0)
    count, = db.execute("SELECT count(*) FROM outbox WHERE repository = ?", row).fetchone()
    return ReportDeliveryState(True, row[0], count)


def ReportDelivery_Status():
    """Read consent and pending count without decrypting the credential."""
    with _transaction() as db:
        return _state(db)


def _repository(value):
    try:
        return ReportUpload_NormalizeRepository(value)
    except ReportUploadError as exc:
        raise ReportDeliveryError(str(exc)) from exc


def _bound_token(repository, protected):
    try:
        bound_repository, token = json.loads(settings._dpapi(protected, False).decode("utf-8"))
        if bound_repository != repository.casefold() or not isinstance(token, str) or not token:
            raise ValueError("binding mismatch")
        return token
    except (OSError, ValueError, TypeError) as exc:
        raise ReportDeliveryError("无法读取该仓库绑定的令牌，请重新设置自动发送。") from exc


def ReportDelivery_Token(repository):
    """Return the saved credential only for an explicitly enabled matching repository."""
    repository = _repository(repository)
    with _transaction() as db:
        row = db.execute("""SELECT protected FROM authorization WHERE id = 1
                            AND repository = ? COLLATE NOCASE""", (repository,)).fetchone()
        return _bound_token(repository, row[0]) if row else None


def ReportDelivery_Configure(repository, token):
    """Enable only for this repository. Require the intended token on every change."""
    repository = _repository(repository)
    if not isinstance(token, str) or not re.fullmatch(r"[!-~]+", token.strip()):
        raise ReportDeliveryError("请输入有效的 GitHub 令牌。")
    binding = json.dumps([repository.casefold(), token.strip()]).encode("utf-8")
    try:
        protected = settings._dpapi(binding, True)
    except OSError as exc:
        raise ReportDeliveryError("Windows 无法加密 GitHub 令牌，自动发送设置未改变。") from exc
    with _transaction() as db:
        db.execute("INSERT OR REPLACE INTO authorization VALUES (1, ?, ?, ?)",
                   (repository, protected, uuid4().hex))
        return _state(db)


def ReportDelivery_Disable():
    """Revoke local consent, forget the protected token and discard pending sends."""
    with _transaction() as db:
        db.execute("DELETE FROM authorization")
        db.execute("DELETE FROM outbox")
        return _state(db)


def ReportDelivery_Enqueue(path):
    """Register a newly saved report for the currently authorized destination."""
    with _transaction() as db:
        row = db.execute("SELECT repository FROM authorization WHERE id = 1").fetchone()
        if row is None:
            return ReportDeliveryEnqueueResult.DISABLED
        # Store the saved path even when a removable/network report folder is offline.
        report = Path(os.path.abspath(path))
        if report.suffix.lower() != ".json":
            raise ReportDeliveryError("只能加入任务报告 JSON 路径。")
        db.execute("""INSERT OR IGNORE INTO outbox (repository, path_key, path)
                      VALUES (?, ?, ?)""", (row[0], os.path.normcase(str(report)), str(report)))
        return ReportDeliveryEnqueueResult.QUEUED


def ReportDelivery_Pending(limit=10):
    """Return a fair batch from the active destination, keeping paths until acknowledged."""
    if type(limit) is not int or limit <= 0:
        raise ReportDeliveryError("待传批次大小必须为正整数。")
    with _transaction() as db:
        authorization = db.execute(
            "SELECT repository, protected, generation FROM authorization WHERE id = 1").fetchone()
        if authorization is None:
            return None
        repository, protected, generation = authorization
        selected = db.execute("""SELECT id, path FROM outbox WHERE repository = ?
                                 ORDER BY turn, id LIMIT ?""", (repository, limit)).fetchall()
        if not selected:
            return None
        token = _bound_token(repository, protected)
        turn, = db.execute("SELECT max(turn) + 1 FROM outbox WHERE repository = ?", (repository,)).fetchone()
        db.executemany("UPDATE outbox SET turn = ? WHERE id = ?", ((turn, row[0]) for row in selected))
        return ReportDeliveryBatch(repository, tuple(Path(row[1]) for row in selected), token, generation)


def _batch_current(db, batch):
    row = db.execute("SELECT generation FROM authorization WHERE id=1 AND repository = ?",
                     (batch.repository,)).fetchone()
    return row == (batch.generation,)


def ReportDelivery_IsCurrent(batch):
    with _transaction() as db:
        return _batch_current(db, batch)


def ReportDelivery_Acknowledge(batch, paths):
    """Remove the successful subset of a batch if its authorization is still current."""
    sent = {Path(path) for path in paths}
    if not sent.issubset(batch.paths):
        raise ReportDeliveryError("确认列表中有未在本批次发送的报告。")
    with _transaction() as db:
        if _batch_current(db, batch):
            db.executemany("DELETE FROM outbox WHERE repository = ? AND path_key = ?",
                           ((batch.repository, os.path.normcase(str(path))) for path in sent))
