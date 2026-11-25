"""Shared typography and accessible color themes for the Qt console."""

from enum import Enum
from string import Template

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QFont, QPalette
from PySide6.QtWidgets import QApplication


class ConsoleTheme(Enum):
    LIGHT = "light"
    DARK = "dark"


_COLORS = {
    ConsoleTheme.LIGHT: {
        "canvas": "#F3F5F7", "surface": "#FFFFFF", "field": "#F5F7FA",
        "text": "#111827", "muted": "#37465A", "border": "#CBD4E0",
        "field_border": "#A2B1C4", "focus": "#2458A6",
        "hover": "#EAF0F8", "pressed": "#DCE6F4", "scroll": "#8394A9",
        "accent": "#2458A6", "accent_text": "#FFFFFF", "accent_hover": "#194887", "accent_pressed": "#153C73",
        "link": "#174C95", "track": "#D9E2EE",
        "disabled": "#E6EBF2", "disabled_text": "#475569",
        "danger": "#A4192E", "danger_bg": "#FDECEF", "danger_hover": "#FAD9DF", "danger_border": "#D78390",
        "wait": "#643B06", "wait_bg": "#FCEBCC",
        "ready": "#07563D", "ready_bg": "#DCF1E8",
    },
    ConsoleTheme.DARK: {
        "canvas": "#10151D", "surface": "#1A232F", "field": "#222E3E",
        "text": "#F3F6FC", "muted": "#C0CDDB", "border": "#39485B",
        "field_border": "#718198", "focus": "#A9C8FF",
        "hover": "#28374A", "pressed": "#31435A", "scroll": "#74869D",
        "accent": "#A9C8FF", "accent_text": "#10233E", "accent_hover": "#C1D8FF", "accent_pressed": "#92B6F4",
        "link": "#BAD4FF", "track": "#344459",
        "disabled": "#293647", "disabled_text": "#A8B5C5",
        "danger": "#FFB6C1", "danger_bg": "#3B2531", "danger_hover": "#502D3B", "danger_border": "#9E6574",
        "wait": "#FFD891", "wait_bg": "#422F19",
        "ready": "#AAEBD5", "ready_bg": "#173E34",
    },
}


_STYLE = Template("""
QWidget#controlView {
    background: ${canvas};
}
QWidget#controlView QLabel,
QWidget#controlView QTabBar,
QWidget#controlView QLineEdit,
QWidget#controlView QComboBox,
QWidget#controlView QPushButton {
    color: ${text};
}
QWidget#controlView QLabel#headerTitle {
    font-size: 20pt;
    font-weight: 700;
}
QWidget#controlView QLabel#cardHeading {
    font-size: 12pt;
    font-weight: 700;
}
QWidget#controlView QLabel#subHeading {
    font-weight: 600;
}
QWidget#controlView QLabel[muted="true"] {
    color: ${muted};
}
QWidget#controlView QLabel#connectionBadge {
    padding: 6px 12px;
    border-radius: 11px;
    background: ${disabled};
    color: ${disabled_text};
    font-weight: 600;
}
QWidget#controlView QLabel#connectionBadge[state="connecting"] {
    background: ${wait_bg};
    color: ${wait};
}
QWidget#controlView QLabel#connectionBadge[state="connected"] {
    background: ${ready_bg};
    color: ${ready};
}
QWidget#controlView QLabel#connectionBadge[state="error"],
QWidget#controlView QLabel#connectionBadge[state="offline"] {
    background: ${danger_bg};
    color: ${danger};
}
QWidget#controlView QFrame#card {
    background: ${surface};
    border: 1px solid ${border};
    border-radius: 12px;
}
QWidget#controlView QFrame#rule {
    background: ${border};
}
QWidget#controlView QTabWidget::pane, QWidget#controlView QTabBar::base {
    border: none;
}
QWidget#controlView QTabBar::tab {
    background: transparent;
    color: ${muted};
    border-bottom: 3px solid transparent;
    font-weight: 600;
    padding: 8px 15px;
}
QWidget#controlView QTabBar::tab:selected {
    color: ${link};
    border-bottom-color: ${accent};
}
QWidget#controlView QTabBar::tab:hover:!selected {
    color: ${text};
    background: ${hover};
}
QWidget#controlView QPushButton {
    min-height: 23px;
    padding: 6px 12px;
    border: 1px solid ${border};
    border-radius: 7px;
    background: ${surface};
    font-weight: 600;
}
QWidget#controlView QPushButton:hover {
    background: ${hover};
    border-color: ${field_border};
}
QWidget#controlView QPushButton:pressed {
    background: ${pressed};
}
QWidget#controlView QPushButton[role="primary"] {
    background: ${accent};
    border-color: ${accent};
    color: ${accent_text};
}
QWidget#controlView QPushButton[role="primary"]:hover {
    background: ${accent_hover};
    border-color: ${accent_hover};
}
QWidget#controlView QPushButton[role="primary"]:pressed {
    background: ${accent_pressed};
    border-color: ${accent_pressed};
}
QWidget#controlView QPushButton[role="danger"] {
    background: ${danger_bg};
    border-color: ${danger_border};
    color: ${danger};
    font-weight: 700;
}
QWidget#controlView QPushButton[role="danger"]:hover {
    background: ${danger_hover};
}
QWidget#controlView QPushButton[role="task"] {
    min-height: 25px;
    text-align: left;
    background: ${field};
    border-color: ${field_border};
}
QWidget#controlView QPushButton[role="task"]:hover {
    background: ${hover};
}
QWidget#controlView QPushButton[role="task"]:pressed {
    background: ${pressed};
}
QWidget#controlView QLineEdit, QWidget#controlView QComboBox {
    background: ${field};
    border: 1px solid ${field_border};
    border-radius: 6px;
    min-height: 23px;
    padding: 5px 8px;
    selection-background-color: ${accent};
    selection-color: ${accent_text};
}
QWidget#controlView QComboBox {
    padding-right: 25px;
}
QWidget#controlView QComboBox::drop-down {
    width: 24px;
    border: none;
}
QWidget#controlView QComboBox::down-arrow {
    width: 0;
    height: 0;
}
QWidget#controlView QComboBox QAbstractItemView {
    background: ${surface};
    color: ${text};
    border: 1px solid ${field_border};
    outline: 0;
    selection-background-color: ${accent};
    selection-color: ${accent_text};
}
QWidget#controlView QComboBox QAbstractItemView::item {
    padding: 4px 8px;
}
QWidget#controlView QPushButton:focus,
QWidget#controlView QPushButton[role]:focus,
QWidget#controlView QLineEdit:focus,
QWidget#controlView QComboBox:focus {
    border: 2px solid ${focus};
}
QWidget#controlView QPushButton[role="primary"]:focus {
    border-color: ${accent_text};
}
QWidget#controlView QPushButton:disabled,
QWidget#controlView QPushButton[role="primary"]:disabled,
QWidget#controlView QPushButton[role="danger"]:disabled,
QWidget#controlView QPushButton[role="task"]:disabled,
QWidget#controlView QLineEdit:disabled,
QWidget#controlView QComboBox:disabled {
    background: ${disabled};
    border-color: ${border};
    color: ${disabled_text};
}
QWidget#controlView QProgressBar {
    border: none;
    border-radius: 5px;
    background: ${track};
    color: ${text};
    text-align: center;
}
QWidget#controlView QProgressBar::chunk {
    background: ${accent};
    border-radius: 5px;
}
QWidget#controlView QPlainTextEdit#events {
    border: 1px solid ${border};
    border-radius: 7px;
    padding: 7px;
    background: ${field};
    color: ${text};
    selection-background-color: ${accent};
    selection-color: ${accent_text};
}
QWidget#controlView QScrollArea, QWidget#controlView QWidget#scrollBody {
    background: ${canvas};
    border: none;
}
QWidget#controlView QScrollBar:vertical {
    width: 12px;
    background: ${field};
}
QWidget#controlView QScrollBar::handle:vertical {
    background: ${scroll};
    border-radius: 4px;
    min-height: 25px;
    margin: 2px;
}
QWidget#controlView QScrollBar::add-line:vertical,
QWidget#controlView QScrollBar::sub-line:vertical {
    height: 0;
}
""")


def Theme_System():
    """Return the color scheme Qt obtained from Windows."""
    hint = QApplication.styleHints().colorScheme()
    return ConsoleTheme.DARK if hint == Qt.ColorScheme.Dark else ConsoleTheme.LIGHT


def Theme_Apply(view, theme):
    """Update widget colors and the palette inherited by Qt popups and dialogs."""
    app = QApplication.instance()
    if app.style().objectName() != "fusion":
        app.setStyle("Fusion")
    font = QFont()
    font.setFamilies(["Microsoft YaHei UI", "Microsoft YaHei", "Noto Sans CJK SC", "Segoe UI"])
    font.setPointSize(11)
    if app.font() != font:
        app.setFont(font)

    colors = _COLORS[theme]
    roles = QPalette.ColorRole
    palette = QPalette()
    for role, key in (
        (roles.Window, "canvas"), (roles.WindowText, "text"), (roles.Base, "surface"),
        (roles.AlternateBase, "field"), (roles.Text, "text"), (roles.Button, "surface"),
        (roles.ButtonText, "text"), (roles.ToolTipBase, "surface"), (roles.ToolTipText, "text"),
        (roles.PlaceholderText, "muted"), (roles.Highlight, "accent"),
        (roles.HighlightedText, "accent_text"), (roles.Link, "link"),
        (roles.Light, "hover"), (roles.Midlight, "border"), (roles.Mid, "field_border"),
        (roles.Dark, "scroll"), (roles.Shadow, "canvas"),
    ):
        palette.setColor(role, QColor(colors[key]))
    for role in (roles.Text, roles.ButtonText, roles.WindowText):
        palette.setColor(QPalette.ColorGroup.Disabled, role, QColor(colors["disabled_text"]))
    app.setPalette(palette)
    view.setPalette(palette)
    view.setStyleSheet(_STYLE.substitute(colors))
