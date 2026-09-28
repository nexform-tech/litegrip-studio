"""The strip along the top: who we are talking to, and whether it is listening.

Two things live here that are easy to get wrong, and both are about *refusing*
rather than doing:

* every button is enabled only when the command it sends could actually be
  accepted, so the console never offers 断开 while it is disconnected or 使能
  while the motor is already enabled;
* the link state is derived from the telemetry the worker publishes rather than
  from whether a connect call returned, because a CAN link that has gone quiet
  since then is the case this display exists for.

The derivation is a pure function so it can be tested without Qt — the
interesting part is the thresholds, not the widget.
"""

from __future__ import annotations

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QGridLayout,
    QGroupBox,
    QLabel,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from .. import constants
from ..core import commands as cmd
from ..core.worker import (
    CONN_CONNECTED,
    CONN_CONNECTING,
    CONN_DISCONNECTED,
    CONN_ERROR,
)
from ..telemetry import EMPTY_FRAME, TelemetryFrame
from . import theme
from .widgets import StatusDot

#: Past this much silence the display is a warning rather than a fact worth
#: stating calmly; the worker refuses motion at LINK_STALE_MS.
LINK_WARN_MS = constants.LINK_STALE_MS / 2.0

CONN_LABELS = {
    CONN_DISCONNECTED: ("未连接", theme.TEXT_MUTED),
    CONN_CONNECTING: ("连接中…", theme.WARN),
    CONN_CONNECTED: ("已连接", theme.OK),
    CONN_ERROR: ("连接失败", theme.ERROR),
}


def link_state(frame: TelemetryFrame) -> tuple[str, str]:
    """The link dot's state and, for the second line, only what is knowable.

    Silence is only evidence when the motor is being addressed.  A console that
    has not enabled anything receives nothing, and calling that "链路断开" would
    be a permanent false alarm on a perfectly good setup.
    """
    if not frame.enabled:
        return "idle", "未使能"
    if frame.stale_ms >= constants.LINK_STALE_MS:
        return "error", f"链路超时 {frame.stale_ms:.0f} ms"
    if frame.stale_ms >= LINK_WARN_MS:
        return "warn", f"响应变慢 {frame.stale_ms:.0f} ms"
    return "ok", f"{frame.rx_hz:.0f} Hz"


#: The link dot while the console's own connection is what is known.  The same
#: words as ``CONN_LABELS``, rendered for a dot rather than for a line of text.
CONN_DOTS = {
    CONN_DISCONNECTED: ("idle", "未连接"),
    CONN_CONNECTING: ("warn", "连接中…"),
    CONN_ERROR: ("error", "连接失败"),
}


def connection_dot(state: str, detail: str = "") -> tuple[str, str]:
    """The link dot when there is no frame to speak of, because there is no link.

    ``link_state`` reads a frame, so it can only ever describe a link that has
    delivered one.  Once the connection drops there is no such frame, and the
    last one received would go on holding the dot green over a cable that is
    out — so the connection state, which needs no frame to be true, takes over.
    """
    dot, text = CONN_DOTS.get(state, ("idle", state))
    if state == CONN_ERROR and detail:
        # Only the failure carries a reason worth the room; "未连接（已断开）"
        # says the same thing twice.
        text = f"{text}：{detail}"
    return dot, text


def fault_state(frame: TelemetryFrame) -> tuple[str, str]:
    """The fault dot's state and text."""
    if frame.error_code in constants.OK_ERROR_CODES:
        return "ok", "无故障"
    text = constants.describe_error(frame.error_code)
    hint = constants.FAULT_HINTS.get(frame.error_code)
    return "error", f"{text}（{hint}）" if hint else text


class ConnectBar(QWidget):
    """Connection, power and fault controls, plus the two status dots."""

    def __init__(self, submit, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._submit = submit
        self._frame = EMPTY_FRAME
        self._conn = "DISCONNECTED"

        self._target = QLabel("未连接")
        self._target.setTextFormat(Qt.RichText)
        self._target.setWordWrap(True)
        self._target.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)

        self._conn_label = QLabel()
        self._conn_label.setTextFormat(Qt.RichText)

        self._link_dot = StatusDot("idle", "未使能")
        self._fault_dot = StatusDot("ok", "无故障")

        self._connect = QPushButton("连接")
        self._disconnect = QPushButton("断开")
        self._enable = QPushButton("使能")
        self._disable = QPushButton("失能")
        self._clear = QPushButton("清除故障")
        self._reset = QPushButton("急停复位")

        self._connect.setProperty("accent", True)
        self._enable.setProperty("accent", True)
        self._disable.setProperty("danger", True)

        self._connect.clicked.connect(lambda: self._submit(cmd.Connect()))
        self._disconnect.clicked.connect(lambda: self._submit(cmd.Disconnect()))
        self._enable.clicked.connect(lambda: self._submit(cmd.Enable()))
        self._disable.clicked.connect(lambda: self._submit(cmd.Disable()))
        self._clear.clicked.connect(lambda: self._submit(cmd.ClearFault()))
        self._reset.clicked.connect(lambda: self._submit(cmd.ResetEStop()))

        self._build()
        self._refresh()

    def _build(self) -> None:
        buttons = QGridLayout()
        buttons.setSpacing(6)
        buttons.addWidget(self._connect, 0, 0)
        buttons.addWidget(self._disconnect, 0, 1)
        buttons.addWidget(self._enable, 1, 0)
        buttons.addWidget(self._disable, 1, 1)
        buttons.addWidget(self._clear, 0, 2)
        buttons.addWidget(self._reset, 1, 2)

        status = QVBoxLayout()
        status.setSpacing(4)
        status.addWidget(self._target)
        status.addWidget(self._conn_label)
        status.addWidget(self._link_dot)
        status.addWidget(self._fault_dot)
        status.addStretch(1)

        box = QGroupBox("连接与电源")
        inner = QGridLayout(box)
        inner.addLayout(status, 0, 0)
        inner.addLayout(buttons, 0, 1)
        inner.setColumnStretch(0, 1)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(box)

    # ── slots ───────────────────────────────────────────────────────────────
    def set_backend_description(self, text: str) -> None:
        self._target.setText(f'<span style="color:{theme.TEXT}">{text}</span>')

    def set_conn_state(self, state: str, detail: str = "") -> None:
        """``state`` is one of the worker's ``CONN_*`` values."""
        self._conn = state
        label, colour = CONN_LABELS.get(state, (state, theme.TEXT_MUTED))
        text = f'<span style="color:{colour}">{label}</span>'
        if detail:
            text += f' <span style="color:{theme.TEXT_MUTED}">{detail}</span>'
        self._conn_label.setText(text)
        self._refresh()

    def update_frame(self, frame: TelemetryFrame) -> None:
        self._frame = frame
        state, text = link_state(frame)
        self._link_dot.set(state, text)
        state, text = fault_state(frame)
        self._fault_dot.set(state, text)
        self._refresh()

    # ── enablement ──────────────────────────────────────────────────────────
    def _refresh(self) -> None:
        """Offer only what could be accepted, and never a second click of what
        has already happened."""
        frame = self._frame
        connected = self._conn == CONN_CONNECTED
        busy = self._conn == CONN_CONNECTING
        estopped = frame.motion_state == "ESTOP"

        self._connect.setEnabled(not connected and not busy)
        self._connect.setText("连接中…" if busy else "连接")
        self._disconnect.setEnabled(connected)
        self._enable.setEnabled(connected and not frame.enabled and not estopped)
        self._disable.setEnabled(connected and frame.enabled)
        self._clear.setEnabled(
            connected and frame.error_code not in constants.OK_ERROR_CODES
        )
        self._reset.setEnabled(estopped)
