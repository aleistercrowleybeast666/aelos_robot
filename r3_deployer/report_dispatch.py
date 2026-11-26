"""Deliver the durable task-report outbox without occupying the robot or Qt thread."""

from enum import Enum
from threading import Event, Thread

from PySide6.QtCore import QObject, Signal

from . import report_delivery as delivery
from .report_upload import (ReportUpload_Push, ReportUploadCancelled, ReportUploadError,
                            ReportUploadInvalidReport, ReportUploadSourceUnavailable)


_FIRST_RETRY_SECONDS = 30
_MAX_RETRY_SECONDS = 300


class _DispatchResult(Enum):
    IDLE = "idle"
    RETRY = "retry"


class ReportDispatcher(QObject):
    message = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._closed = Event()
        self._wake = Event()
        self._last_problem = None
        # destroyed can run after the C++ object has died; retain only the events.
        closed, wake = self._closed, self._wake
        self.destroyed.connect(lambda *_: (closed.set(), wake.set()))
        self._thread = Thread(target=self._run, daemon=True, name="r3-report-delivery")
        self._thread.start()

    def ReportDispatch_Enqueue(self, path):
        """Register at report-save time on the motion worker, even if Qt is closing."""
        try:
            if delivery.ReportDelivery_Enqueue(path):
                self._wake.set()
        except delivery.ReportDeliveryError as exc:
            self._message(f"任务报告已保存在本机，加入自动发送队列失败。{exc}")

    def ReportDispatch_Retry(self):
        """Wake the worker after startup or a change to delivery settings."""
        self._wake.set()

    def ReportDispatch_Close(self):
        self._closed.set()
        self._wake.set()

    def _message(self, text):
        if not self._closed.is_set():
            try:
                self.message.emit(text)
            except RuntimeError:
                # The Qt owner may have closed while a network call was in flight.
                pass

    def _problem(self, text):
        if text != self._last_problem:
            self._message(text)
            self._last_problem = text

    def _drain(self):
        deferred = set()
        while not self._closed.is_set():
            batch = delivery.ReportDelivery_Pending()
            if not batch:
                return _DispatchResult.IDLE
            paths = [path for path in batch.paths if path not in deferred]
            if not paths:
                return _DispatchResult.RETRY

            def still_current():
                return not self._closed.is_set() and delivery.ReportDelivery_IsCurrent(batch)

            for path in paths:
                if not still_current():
                    return _DispatchResult.IDLE
                try:
                    result = ReportUpload_Push([path], batch.repository, batch.token, still_current=still_current)
                except ReportUploadCancelled:
                    return _DispatchResult.IDLE
                except ReportUploadSourceUnavailable as exc:
                    self._problem(f"报告暂不可读，已保留待传并继续处理其他报告。{exc}")
                    deferred.add(path)
                    continue
                except ReportUploadInvalidReport as exc:
                    delivery.ReportDelivery_Acknowledge(batch, [path])
                    self._message(f"已跳过无法发送的报告 {path.name}。{exc}")
                except ReportUploadError as exc:
                    self._problem(f"GitHub 自动发送暂未完成，后台将重试。{exc}")
                    return _DispatchResult.RETRY
                else:
                    delivery.ReportDelivery_Acknowledge(batch, [path])
                    self._last_problem = None
                    state = "已发送" if result.uploaded else "远程已归档"
                    self._message(f"GitHub 自动发送 · {state} · {path.name}")
        return _DispatchResult.IDLE

    def _run(self):
        delay = None
        while not self._closed.is_set():
            self._wake.wait(delay)
            self._wake.clear()
            if self._closed.is_set():
                return
            try:
                result = self._drain()
            except delivery.ReportDeliveryError as exc:
                self._problem(f"自动发送队列暂不可用，后台将重试。{exc}")
                result = _DispatchResult.RETRY
            except Exception:
                self._problem("自动发送遇到意外错误，未确认的报告保留在本机队列，后台将重试。")
                result = _DispatchResult.RETRY
            if result is _DispatchResult.RETRY:
                delay = min(delay * 2, _MAX_RETRY_SECONDS) if delay else _FIRST_RETRY_SECONDS
            else:
                delay = None
