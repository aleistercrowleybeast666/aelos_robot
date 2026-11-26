"""Non-modal task report sharing controls for the Windows console."""

from datetime import datetime
import os
from pathlib import Path
import re

from PySide6.QtCore import QSettings, Qt, Signal
from PySide6.QtWidgets import (QComboBox, QDialog, QFileDialog, QFormLayout, QHBoxLayout,
                               QLabel, QLineEdit, QPushButton, QVBoxLayout, QWidget)

from . import report_delivery as delivery
from .qt_theme import Theme_Apply
from .report_share import ReportShare_CopyToFolder, ReportShare_ExportZip
from .report_upload import (DEFAULT_REPORT_REPOSITORY, REPORT_BRANCH,
                            ReportUpload_NormalizeRepository, ReportUpload_Push)
from .task_report import TaskReport_Directory


class ReportWindow(QDialog):
    auto_changed = Signal(bool)

    def __init__(self, parent, start_worker):
        super().__init__(parent)
        self.setWindowTitle("分享任务报告")
        self.setObjectName("controlView")
        self.setMinimumWidth(600)
        self._start_worker = start_worker
        self.preferences = QSettings("AELOS", "R3Control")
        self.paths = ()
        self.busy = False
        self.auto_enabled = False
        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 18, 20, 16)
        layout.setSpacing(12)
        self.method = QComboBox()
        for label, mode in (("GitHub", "github"), ("共享文件夹", "folder"), ("导出 ZIP", "zip")):
            self.method.addItem(label, mode)
        form = QFormLayout()
        form.addRow("分享方式", self.method)
        layout.addLayout(form)

        self.github = QWidget()
        git_form = QFormLayout(self.github)
        git_form.setContentsMargins(0, 4, 0, 4)
        self.repository = QLineEdit(self.preferences.value("reports/repository", DEFAULT_REPORT_REPOSITORY))
        git_form.addRow("目标仓库", self.repository)
        self.token = QLineEdit()
        self.token.setEchoMode(QLineEdit.EchoMode.Password)
        self.token.setPlaceholderText("粘贴令牌即在本机保存并自动推送")
        self._typed_length = 0
        token_row = QHBoxLayout()
        token_row.addWidget(self.token, 1)
        token_link = QLabel('<a href="https://github.com/settings/personal-access-tokens/new">创建令牌</a>')
        token_link.setOpenExternalLinks(True)
        token_row.addWidget(token_link)
        git_form.addRow("访问令牌", token_row)
        hint = QLabel(f"日志存入 {REPORT_BRANCH} 分支。默认仓库公开可读，推送需要有写权限的令牌。")
        hint.setWordWrap(True)
        git_form.addRow(hint)
        auto_row = QHBoxLayout()
        self.auto_state = QLabel()
        auto_row.addWidget(self.auto_state, 1)
        self.enable = QPushButton("启用自动推送")
        self.enable.setProperty("role", "primary")
        self.disable = QPushButton("关闭自动推送")
        auto_row.addWidget(self.enable)
        auto_row.addWidget(self.disable)
        git_form.addRow(auto_row)
        layout.addWidget(self.github)

        self.selected = QLabel()
        self.selected.setTextFormat(Qt.TextFormat.PlainText)
        self.selected.setWordWrap(True)
        selection = QHBoxLayout()
        selection.addWidget(self.selected, 1)
        self.choose = QPushButton("选择报告…")
        selection.addWidget(self.choose)
        layout.addLayout(selection)
        self.feedback = QLabel()
        self.feedback.setTextFormat(Qt.TextFormat.PlainText)
        self.feedback.setWordWrap(True)
        self.feedback.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addWidget(self.feedback)
        actions = QHBoxLayout()
        actions.addStretch(1)
        self.send = QPushButton()
        close = QPushButton("关闭窗口")
        actions.addWidget(self.send)
        actions.addWidget(close)
        layout.addLayout(actions)
        for button in self.findChildren(QPushButton):
            button.setAutoDefault(False)
        self.choose.clicked.connect(self._choose_reports)
        self.method.currentIndexChanged.connect(self._update_view)
        self.enable.clicked.connect(self._enable_auto)
        self.token.returnPressed.connect(self._enable_auto)
        self.token.textEdited.connect(self._token_edited)
        self.token.textChanged.connect(self._token_changed)
        self.disable.clicked.connect(self._disable_auto)
        self.send.clicked.connect(self._send)
        close.clicked.connect(self.close)
        parent.report_dispatch.message.connect(self._delivery_update)
        self._show_config()
        self._update_view()

    def ReportWindow_Show(self, latest):
        Theme_Apply(self, self.parentWidget().view.theme)
        if not self.isVisible():
            self._show_config()
        if not self.busy and latest and not self.isVisible():
            self.paths = (Path(latest),)
        self._update_view()
        self.show()
        self.raise_()
        self.activateWindow()

    def _update_view(self):
        mode = self.method.currentData()
        self.github.setVisible(mode == "github")
        self.method.setEnabled(not self.busy)
        self.choose.setEnabled(not self.busy)
        self.github.setEnabled(not self.busy)
        self.disable.setEnabled(self.auto_enabled)
        if self.paths:
            self.selected.setText(f"已选 {len(self.paths)} 份报告\n{self.paths[0].name}" +
                                  (" 等" if len(self.paths) > 1 else ""))
        else:
            self.selected.setText("可选择已保存的任务报告")
        self.send.setText({"github": "推送到 GitHub", "folder": "发送到共享文件夹…",
                           "zip": "导出 ZIP…"}[mode])
        self.send.setEnabled(bool(self.paths) and not self.busy)

    def _delivery_update(self, message):
        if self.isVisible():
            self._show_config(load_repository=False)
            if not self.busy and self.method.currentData() == "github":
                self.feedback.setText(message)

    def _show_config(self, *, load_repository=True):
        try:
            config = delivery.ReportDelivery_Status()
            self.auto_enabled = config.enabled
            if config.enabled:
                if load_repository:
                    self.repository.setText(config.repository)
                self.auto_state.setText(f"已启用 · 待发送 {config.pending_count} 份")
            else:
                self.auto_state.setText("自动推送已关闭")
        except Exception as exc:
            self.auto_state.setText("无法读取推送设置")
            self.feedback.setText(str(exc))

    def _token_changed(self, text):
        if not text:
            self._typed_length = 0

    def _token_edited(self, text):
        # Pasting is one user edit. A typed token is submitted with Enter.
        inserted = len(text) - self._typed_length
        self._typed_length = len(text)
        if inserted >= 20 and re.fullmatch(r"(?:github_pat_|gh[pousr]_)[A-Za-z0-9_]{20,}", text.strip()):
            self._enable_auto()

    def _enable_auto(self):
        if self.busy:
            return
        try:
            repository = ReportUpload_NormalizeRepository(self.repository.text())
            token = self.token.text().strip() or os.environ.get("AELOS_GITHUB_TOKEN", "").strip()
            if not token:
                config = delivery.ReportDelivery_Status()
                if config.enabled and config.repository.casefold() == repository.casefold():
                    self.feedback.setText("此仓库已启用自动推送。")
                    return
                raise ValueError("请先填入 GitHub 访问令牌。")
            delivery.ReportDelivery_Configure(repository, token)
        except Exception as exc:
            self.feedback.setText(str(exc))
            return
        self.token.clear()
        self.preferences.setValue("reports/repository", repository)
        self.feedback.setText("本机已保存令牌。当前会话最近收到的报告和后续新报告将自动发送。")
        self.auto_changed.emit(True)
        self._show_config()
        self._update_view()

    def _disable_auto(self):
        try:
            delivery.ReportDelivery_Disable()
        except Exception as exc:
            self.feedback.setText(str(exc))
            return
        self.token.clear()
        self.feedback.setText("已关闭自动推送，已清除保存的令牌和待发队列。")
        self.auto_changed.emit(False)
        self._show_config()
        self._update_view()

    def _choose_reports(self):
        try:
            directory = self.paths[0].parent if self.paths else TaskReport_Directory()
        except (ValueError, OSError):
            directory = Path.home()
        names, _ = QFileDialog.getOpenFileNames(self, "选择任务报告（可多选）", str(directory),
                                                "任务报告 (*.json)")
        if names:
            self.paths = tuple(map(Path, names))
            self.feedback.clear()
            self._update_view()

    def _send(self):
        if self.busy or not self.paths:
            return
        mode = self.method.currentData()
        paths = self.paths
        if mode == "github":
            try:
                repository = ReportUpload_NormalizeRepository(self.repository.text())
                token = self.token.text().strip() or os.environ.get("AELOS_GITHUB_TOKEN", "").strip()
                if not token:
                    token = delivery.ReportDelivery_Token(repository)
                if not token:
                    raise ValueError("请先填入有仓库写入权限的 GitHub 令牌。")
            except Exception as exc:
                self.feedback.setText(str(exc))
                return
            self.preferences.setValue("reports/repository", repository)
            self.token.clear()

            def operation():
                result = ReportUpload_Push(paths, repository, token)
                return f"GitHub 推送完成 · 新增 {result.uploaded} 份，已有 {result.skipped} 份 · {result.branch_url}"
        elif mode == "folder":
            directory = QFileDialog.getExistingDirectory(self, "选择接收报告的共享文件夹",
                                                          self.preferences.value("reports/shareFolder", ""))
            if not directory:
                return
            self.preferences.setValue("reports/shareFolder", directory)

            def operation():
                receipt = ReportShare_CopyToFolder(paths, directory)
                return f"共享文件夹接收完成 · 新增 {receipt.written_count} 份，已有 {receipt.existing_count} 份 · {directory}"
        else:
            name = datetime.now().strftime("AELOS-任务报告-%Y%m%d-%H%M%S.zip")
            filename, _ = QFileDialog.getSaveFileName(self, "导出任务报告", str(Path.home() / name),
                                                      "ZIP 压缩包 (*.zip)")
            if not filename:
                return
            if not filename.lower().endswith(".zip"):
                filename += ".zip"

            def operation():
                receipt = ReportShare_ExportZip(paths, filename)
                return f"已导出 {receipt.report_count} 份任务报告 · {receipt.destination}"
        self.busy = True
        self.feedback.setText("正在后台处理所选报告…")
        self._update_view()
        self._start_worker("report_share", self, "r3-report-share", operation)

    def ReportWindow_Finish(self, outcome):
        self.busy = False
        message = f"分享报告失败 · {outcome}" if isinstance(outcome, Exception) else str(outcome)
        self._show_config()
        self.feedback.setText(message)
        self._update_view()
        return message
