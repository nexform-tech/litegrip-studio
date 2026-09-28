"""The console's colours, and the stylesheet built from them.

Named here rather than spelled at each call site because several of them carry
meaning across widgets: the slider's thin line and the target readout are the
same cyan, the actual-position triangle and the "达到" state are the same green.
A colour that means "the jaws really are here" must not be a different green in
the plot than it is on the slider.

The palette is dark by default, which is what a bench under a machine tends to
be, and it is applied to the whole application rather than per widget so a Qt
dialog the console did not write (a file chooser, a message box) does not come
back bright white in the middle of it.
"""

from __future__ import annotations

from typing import Any

# ── surface ─────────────────────────────────────────────────────────────────
BACKGROUND = "#1b1e24"
PANEL = "#23272f"
PANEL_ALT = "#2a2f39"
BORDER = "#333945"
GROOVE = "#2f3540"

# ── text ────────────────────────────────────────────────────────────────────
TEXT = "#dfe4ec"
TEXT_MUTED = "#8b93a3"
TEXT_DISABLED = "#5c6472"

# ── meaning ─────────────────────────────────────────────────────────────────
#: The commanded target: the slider's thin line, the target readout, the plot's
#: setpoint trace.
TARGET = "#35c6d8"
#: Where the jaws measurably are.  Used for the slider's triangle, the actual
#: readout, and the plot's position trace — one green, three places.
ACTUAL = "#4ad07a"
GRASPED = "#8f7ae0"
WARN = "#e0a33a"
ERROR = "#e05252"
OK = "#4ad07a"
DISABLED = "#5c6472"

# ── geometry ────────────────────────────────────────────────────────────────
RADIUS = 4
PAD = 8

#: Monospaced, so a number that changes at 50 Hz does not make the layout
#: twitch: proportional digits would shift every neighbouring label as the
#: value goes from 9.9 to 10.0.
MONO_FAMILY = "DejaVu Sans Mono, Consolas, monospace"


def stylesheet() -> str:
    """The application-wide stylesheet."""
    return f"""
    QWidget {{
        background: {BACKGROUND};
        color: {TEXT};
        font-size: 13px;
    }}
    QGroupBox {{
        background: {PANEL};
        border: 1px solid {BORDER};
        border-radius: {RADIUS}px;
        margin-top: 14px;
        padding: {PAD}px;
        font-weight: bold;
    }}
    QGroupBox::title {{
        subcontrol-origin: margin;
        left: 10px;
        padding: 0 4px;
        color: {TEXT_MUTED};
    }}
    QPushButton {{
        background: {PANEL_ALT};
        border: 1px solid {BORDER};
        border-radius: {RADIUS}px;
        padding: 6px 14px;
        min-height: 20px;
    }}
    QPushButton:hover:enabled {{ background: #333947; }}
    QPushButton:pressed:enabled {{ background: #3b4250; }}
    QPushButton:disabled {{ color: {TEXT_DISABLED}; border-color: #2b303a; }}
    QPushButton[accent="true"] {{
        background: {TARGET};
        color: #10131a;
        border: none;
        font-weight: bold;
    }}
    QPushButton[accent="true"]:hover:enabled {{ background: #4ad3e4; }}
    QPushButton[danger="true"] {{
        background: {ERROR};
        color: #ffffff;
        border: none;
        font-weight: bold;
    }}
    QPushButton[danger="true"]:hover:enabled {{ background: #ea6565; }}
    QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {{
        background: {PANEL_ALT};
        border: 1px solid {BORDER};
        border-radius: {RADIUS}px;
        padding: 4px 6px;
        selection-background-color: {TARGET};
    }}
    QLineEdit:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled, QComboBox:disabled {{
        color: {TEXT_DISABLED};
    }}
    QTabWidget::pane {{ border: 1px solid {BORDER}; border-radius: {RADIUS}px; }}
    QTabBar::tab {{
        background: {PANEL};
        border: 1px solid {BORDER};
        border-bottom: none;
        border-top-left-radius: {RADIUS}px;
        border-top-right-radius: {RADIUS}px;
        padding: 7px 16px;
        margin-right: 2px;
        color: {TEXT_MUTED};
    }}
    QTabBar::tab:selected {{ background: {PANEL_ALT}; color: {TEXT}; }}
    QTabBar::tab:disabled {{ color: {TEXT_DISABLED}; }}
    QTableWidget, QTableView {{
        background: {PANEL};
        gridline-color: {BORDER};
        border: 1px solid {BORDER};
        border-radius: {RADIUS}px;
    }}
    QHeaderView::section {{
        background: {PANEL_ALT};
        border: none;
        border-right: 1px solid {BORDER};
        border-bottom: 1px solid {BORDER};
        padding: 5px 8px;
        color: {TEXT_MUTED};
    }}
    QPlainTextEdit, QTextEdit {{
        background: #15181e;
        border: 1px solid {BORDER};
        border-radius: {RADIUS}px;
        font-family: {MONO_FAMILY};
        font-size: 12px;
    }}
    QScrollBar:vertical {{ background: {BACKGROUND}; width: 10px; margin: 0; }}
    QScrollBar::handle:vertical {{ background: {BORDER}; border-radius: 5px; min-height: 24px; }}
    QScrollBar::add-line, QScrollBar::sub-line {{ height: 0; width: 0; }}
    QStatusBar {{ background: {PANEL}; color: {TEXT_MUTED}; }}
    QToolTip {{
        background: {PANEL_ALT};
        color: {TEXT};
        border: 1px solid {BORDER};
        padding: 4px;
    }}
    """


def apply(app: Any) -> None:
    """Apply the stylesheet.  Takes the ``QApplication``, not a widget."""
    app.setStyleSheet(stylesheet())


def level_colour(level: str) -> str:
    """The colour for an alert or log level.

    Unknown levels are muted rather than an error colour: something arriving
    that this module has not been taught about is not the operator's emergency.
    """
    return {
        "info": TEXT_MUTED,
        "debug": TEXT_DISABLED,
        "warn": WARN,
        "warning": WARN,
        "error": ERROR,
        "fatal": ERROR,
    }.get(level, TEXT_MUTED)
