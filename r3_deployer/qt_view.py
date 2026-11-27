"""PySide6 views for the R3 desktop control console."""

from __future__ import annotations

from pathlib import Path

from PySide6.QtCore import QPoint, QRect, QSettings, Qt
from PySide6.QtGui import QColor, QImage, QPainter, QPalette, QPen
from PySide6.QtWidgets import (
    QComboBox,
    QFileDialog,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QStyle,
    QStyleOptionComboBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from .qt_theme import ConsoleTheme, Theme_Apply, Theme_System
from .control_api import DROP_YAW_LIMIT_DEG


_TASKS = {
    0: "单程搬运原始物资",
    1: "单程搬运转运物资",
    2: "连续搬运转运物资",
}


def _Label_Create(text="", *, role="", wrap=False):
    label = QLabel(text)
    label.setTextFormat(Qt.TextFormat.PlainText)
    label.setWordWrap(wrap)
    if role == "muted":
        label.setProperty("muted", True)
    elif role:
        label.setObjectName(role)
    return label


def _Button_Create(text, role=""):
    button = QPushButton(text)
    button.setCursor(Qt.CursorShape.PointingHandCursor)
    if role:
        button.setProperty("role", role)
    return button


def _Panel_Create(title="", margin=14):
    panel = QFrame()
    panel.setObjectName("card")
    layout = QVBoxLayout(panel)
    layout.setContentsMargins(margin, margin, margin, margin)
    layout.setSpacing(8)
    if title:
        layout.addWidget(_Label_Create(title, role="cardHeading"))
    return panel, layout


def _Form_Create(parent_layout):
    form = QGridLayout()
    form.setColumnStretch(1, 1)
    form.setHorizontalSpacing(9)
    form.setVerticalSpacing(8)
    parent_layout.addLayout(form)
    return form


class _ConsoleComboBox(QComboBox):
    def paintEvent(self, event):
        super().paintEvent(event)
        option = QStyleOptionComboBox()
        self.initStyleOption(option)
        arrow = self.style().subControlRect(
            QStyle.ComplexControl.CC_ComboBox, option, QStyle.SubControl.SC_ComboBoxArrow, self
        ).center()
        color = option.palette.color(QPalette.ColorRole.ButtonText)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setPen(QPen(color, 1.6))
        lower = arrow + QPoint(0, 3)
        painter.drawLine(arrow + QPoint(-4, -1), lower)
        painter.drawLine(lower, arrow + QPoint(4, -1))


class CameraView(QWidget):
    """Present the latest camera frame with its original aspect ratio."""

    def __init__(self):
        super().__init__()
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setMinimumSize(290, 200)
        self._image = None
        self._placeholder = "胸部相机未开启"

    def set_frame(self, frame: QImage | None):
        self._image = frame
        self.update()

    def setText(self, text):
        self._placeholder = str(text)
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor("#0C1421"))
        painter.drawRoundedRect(self.rect(), 10, 10)
        if self._image is None or self._image.isNull():
            painter.setFont(self.font())
            painter.setPen(QColor("#D4DEE9"))
            area = self.rect().adjusted(20, 20, -20, -20)
            painter.drawText(area, Qt.AlignmentFlag.AlignCenter | Qt.TextFlag.TextWordWrap, self._placeholder)
            return
        bounds = self.rect().adjusted(6, 6, -6, -6)
        size = self._image.size().scaled(bounds.size(), Qt.AspectRatioMode.KeepAspectRatio)
        origin = QPoint(bounds.center().x() - size.width() // 2, bounds.center().y() - size.height() // 2)
        painter.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)
        painter.drawImage(QRect(origin, size), self._image)


class ControlView(QWidget):
    """Contain controls and display widgets without invoking robot operations."""

    def __init__(self, values):
        super().__init__()
        self.setObjectName("controlView")
        self._appearance = QSettings("AELOS", "R3Control")
        saved_theme = self._appearance.value("appearance/theme", "", type=str)
        self.theme = ConsoleTheme(saved_theme) if saved_theme in ("light", "dark") else Theme_System()
        self.setAutoFillBackground(True)
        self.fields = {name: QLineEdit(str(values.get(name, ""))) for name in
                       ("host", "port", "username", "local_path", "remote_path", "remote_url")}
        self.deployment_source = _ConsoleComboBox()
        self.deployment_source.addItem("远程下载", "remote")
        self.deployment_source.addItem("本地目录", "local")
        self.deployment_source.setCurrentIndex(max(0, self.deployment_source.findData(values.get("deployment_source"))))
        self.fields["remote_url"].setPlaceholderText("https://…/pi_deploy_bundle")
        self.password = QLineEdit()
        self.password.setEchoMode(QLineEdit.EchoMode.Password)
        self.password.setPlaceholderText("留空沿用已保存的密码")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 14, 20, 11)
        layout.setSpacing(8)
        layout.addLayout(self._build_header(values))

        self.tabs = QTabWidget()
        self.tabs.setDocumentMode(True)
        self.tabs.addTab(self._build_control(), "控制")
        self.tabs.addTab(self._build_pi(), "SSH配置与程序部署")
        layout.addWidget(self.tabs, 1)
        layout.addWidget(self._build_log())
        self.status_label = _Label_Create("就绪", role="muted")
        self.status_label.setMinimumHeight(19)
        layout.addWidget(self.status_label)
        self._apply_theme()

    def _build_header(self, values):
        header = QHBoxLayout()
        header.setSpacing(10)
        header.addWidget(_Label_Create("AELOS R3", role="headerTitle"))
        self.theme_button = _Button_Create("")
        self.theme_button.clicked.connect(self._switch_theme)
        header.addWidget(self.theme_button)
        header.addStretch(1)
        self.target_label = _Label_Create(f"{values.get('host', '')}:{values.get('port', '')}", role="muted")
        self.connection_label = _Label_Create("未连接")
        self.connection_label.setObjectName("connectionBadge")
        self.set_connection_state("disconnected", "未连接")
        self.connection_button = _Button_Create("连接机器人", "primary")
        self.connection_button.setMinimumWidth(112)
        self.stop_button = _Button_Create("停止任务", "danger")
        self.stop_button.setMinimumWidth(95)
        for widget in (self.target_label, self.connection_label, self.connection_button, self.stop_button):
            header.addWidget(widget)
        return header

    def _apply_theme(self):
        Theme_Apply(self, self.theme)
        self.theme_button.setText("切换到亮色" if self.theme is ConsoleTheme.DARK else "切换到暗色")

    def _switch_theme(self):
        self.theme = ConsoleTheme.LIGHT if self.theme is ConsoleTheme.DARK else ConsoleTheme.DARK
        self._apply_theme()
        self._appearance.setValue("appearance/theme", self.theme.value)
        self._appearance.sync()

    def set_connection_state(self, state, text):
        self.connection_label.setText(text)
        if self.connection_label.property("state") != state:
            self.connection_label.setProperty("state", state)
            style = self.connection_label.style()
            style.unpolish(self.connection_label)
            style.polish(self.connection_label)

    def _build_control(self):
        page = QWidget()
        layout = QHBoxLayout(page)
        layout.setContentsMargins(0, 8, 0, 0)
        layout.setSpacing(12)

        camera, camera_layout = _Panel_Create()
        camera_header = QHBoxLayout()
        camera_header.addWidget(_Label_Create("胸部相机", role="cardHeading"))
        camera_header.addStretch(1)
        self.camera_button = _Button_Create("打开预览")
        camera_header.addWidget(self.camera_button)
        camera_layout.addLayout(camera_header)
        self.camera_view = CameraView()
        camera_layout.addWidget(self.camera_view, 1)
        self.camera_status = _Label_Create("预览已关闭", role="muted")
        camera_layout.addWidget(self.camera_status)
        layout.addWidget(camera, 1)

        right = QVBoxLayout()
        right.setContentsMargins(0, 0, 0, 0)
        right.setSpacing(10)
        state, state_layout = _Panel_Create("机器人状态")
        self.robot_status = _Label_Create("连接后读取运行状态", wrap=True)
        self.robot_status.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        self.robot_status.setMinimumHeight(42)
        state_layout.addWidget(self.robot_status)
        self.scan_button = _Button_Create("读取 Tag")
        state_layout.addWidget(self.scan_button, 0, Qt.AlignmentFlag.AlignLeft)
        right.addWidget(state)

        tasks, task_layout = _Panel_Create("自动任务")
        angle_row = QHBoxLayout()
        angle_label = _Label_Create("放物转角（负左，正右）")
        self.drop_yaw = QSpinBox()
        self.drop_yaw.setRange(-DROP_YAW_LIMIT_DEG, DROP_YAW_LIMIT_DEG)
        self.drop_yaw.setSuffix("°")
        self.drop_yaw.setAccessibleName(angle_label.text())
        angle_label.setBuddy(self.drop_yaw)
        angle_row.addWidget(angle_label)
        angle_row.addStretch(1)
        angle_row.addWidget(self.drop_yaw)
        task_layout.addLayout(angle_row)
        task_layout.addWidget(_Label_Create("连续搬运第二趟向相反方向放物", role="muted"))
        self.task_buttons = {}
        for number, title in _TASKS.items():
            button = _Button_Create(title, "task")
            task_layout.addWidget(button)
            self.task_buttons[number] = button
        self.task_hint = _Label_Create(role="muted", wrap=True)
        task_layout.addWidget(self.task_hint)
        self.reset_empty_button = _Button_Create("恢复空载站姿")
        self.reset_empty_button.setEnabled(False)
        task_layout.addWidget(self.reset_empty_button)
        rule = QFrame()
        rule.setObjectName("rule")
        rule.setFixedHeight(1)
        task_layout.addWidget(rule)
        task_layout.addWidget(_Label_Create("单程进度", role="subHeading"))
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_bar.setTextVisible(False)
        self.progress_bar.setFixedHeight(10)
        self.progress_bar.setAccessibleName("单程进度")
        task_layout.addWidget(self.progress_bar)
        self.progress_label = _Label_Create("等待任务", role="muted", wrap=True)
        task_layout.addWidget(self.progress_label)
        right.addWidget(tasks)
        right.addStretch(1)
        column = QWidget()
        column.setMinimumWidth(281)
        column.setMaximumWidth(345)
        column.setLayout(right)
        layout.addWidget(column)
        return page

    @staticmethod
    def _scroll_page():
        body = QWidget()
        body.setObjectName("scrollBody")
        layout = QVBoxLayout(body)
        layout.setContentsMargins(0, 8, 0, 1)
        layout.setSpacing(10)
        scroll = QScrollArea()
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setWidgetResizable(True)
        scroll.setWidget(body)
        return scroll, layout

    def _build_pi(self):
        page, layout = self._scroll_page()
        connection, items = _Panel_Create("SSH 连接")
        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(8)
        grid.setColumnStretch(1, 2)
        grid.setColumnStretch(3, 1)
        grid.setColumnStretch(5, 0)
        for index, (key, label) in enumerate((("host", "地址"), ("username", "用户名"), ("port", "端口"))):
            self._add_field(grid, 0, self.fields[key], label, column=index * 2, span=1)
        self.fields["port"].setMaximumWidth(95)
        self._add_field(grid, 1, self.password, "密码", span=4)
        self.reveal_password = _Button_Create("显示密码")
        self.reveal_password.setCheckable(True)
        self.reveal_password.toggled.connect(self._show_password)
        grid.addWidget(self.reveal_password, 1, 5)
        items.addLayout(grid)
        layout.addWidget(connection)

        paths, path_items = _Panel_Create("程序来源与目标")
        form = _Form_Create(path_items)
        self._add_field(form, 0, self.deployment_source, "部署来源")
        self._add_field(form, 1, self.fields["remote_url"], "远程部署包网址")
        self.local_browse_button = self._add_field(form, 2, self.fields["local_path"], "电脑部署包", browse=True)
        self._add_field(form, 3, self.fields["remote_path"], "树莓派目录")
        layout.addWidget(paths)

        operations, operation_items = _Panel_Create("部署与恢复")
        buttons = QHBoxLayout()
        buttons.setSpacing(7)
        self.save_button = _Button_Create("保存配置")
        buttons.addWidget(self.save_button)
        self.route_edit_button = _Button_Create("编辑动作链")
        self.route_edit_button.setToolTip("编辑工作区的 route_steps.json。使用本地部署时应用这些修改。")
        buttons.addWidget(self.route_edit_button)
        self.pi_buttons = {
            "check": _Button_Create("检查 SSH"),
            "deploy": _Button_Create("部署程序", "primary"),
            "rollback": _Button_Create("恢复上一版"),
        }
        for button in self.pi_buttons.values():
            buttons.addWidget(button)
        buttons.addStretch(1)
        self.update_task = _ConsoleComboBox()
        self.update_task.setAccessibleName("部署后启动的任务")
        for number, title in _TASKS.items():
            self.update_task.addItem(title, number)
        self.update_start_button = _Button_Create("部署后启动")
        buttons.addWidget(self.update_task)
        buttons.addWidget(self.update_start_button)
        operation_items.addLayout(buttons)
        layout.addWidget(operations)
        layout.addStretch(1)
        return page

    def _build_log(self):
        panel, layout = _Panel_Create(margin=10)
        log_header = QHBoxLayout()
        log_header.addWidget(_Label_Create("运行日志", role="cardHeading"))
        log_header.addStretch(1)
        self.open_reports_button = _Button_Create("打开日志文件夹")
        self.change_reports_button = _Button_Create("更改日志目录")
        self.share_reports_button = _Button_Create("分享任务报告…")
        self.clear_log_button = _Button_Create("清空显示")
        for button in (self.open_reports_button, self.change_reports_button,
                       self.share_reports_button, self.clear_log_button):
            log_header.addWidget(button)
        layout.addLayout(log_header)
        self.log = QPlainTextEdit()
        self.log.setObjectName("events")
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(1500)
        self.log.setMinimumHeight(76)
        self.log.setMaximumHeight(110)
        self.log.setAccessibleName("运行日志")
        layout.addWidget(self.log)
        return panel

    def _add_field(self, form, row, edit, text, *, column=0, span=1, browse=False):
        label = _Label_Create(text)
        label.setBuddy(edit)
        edit.setAccessibleName(text)
        form.addWidget(label, row, column)
        form.addWidget(edit, row, column + 1, 1, span)
        if browse:
            button = _Button_Create("浏览…")
            button.setAccessibleName(f"选择{text}")
            button.clicked.connect(lambda: self._choose_directory(edit, text))
            form.addWidget(button, row, column + span + 1)
            return button

    def _choose_directory(self, edit, title):
        if not edit.isEnabled() or edit.isReadOnly():
            return
        path = Path(edit.text()).expanduser()
        initial = str(path if path.is_dir() else path.parent)
        selected = QFileDialog.getExistingDirectory(self, f"选择{title}", initial)
        if selected:
            edit.setText(selected)

    def _show_password(self, show):
        self.password.setEchoMode(QLineEdit.EchoMode.Normal if show else QLineEdit.EchoMode.Password)
        self.reveal_password.setText("隐藏密码" if show else "显示密码")
