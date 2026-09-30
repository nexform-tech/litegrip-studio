"""The status page: what the hardware is doing, in numbers that can be read out.

Everything on it comes from the published frame, so nothing here queries the
backend — a status page that called into the SDK would be a second thread
touching a bus that is not thread-safe, and it would also be the page that hangs
when the link is the thing that is broken.

The values are laid out as labels rather than as a table.  A table would want
rebuilding whenever a row changed, and at 50 Hz that is a repaint of the whole
widget every time a digit moves; a label whose text is set is a repaint of the
label.

The two dots are rendered by the same pure functions the connection bar uses, so
the link is never described as healthy at the top of the window and as dead at
the bottom of it.
"""

from __future__ import annotations

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QVBoxLayout,
    QWidget,
)

from .. import constants
from ..core.worker import CONN_CONNECTED, GateState
from ..telemetry import TelemetryFrame
from . import theme
from .connect_bar import connection_dot, fault_state, link_state
from .widgets import UNKNOWN, Banner, StatusDot

#: The page's own labels and how to render a frame into them.  Kept as data so
#: the list is auditable against :class:`TelemetryFrame` in one place, and so a
#: field added to the frame has an obvious home.
_READING_ROWS = (
    ("位置", lambda f: UNKNOWN if f.position_mm is None else f"{f.position_mm:.2f} mm"),
    ("速度", lambda f: f"{f.velocity_mm_s:+.1f} mm/s"),
    ("夹持力", lambda f: f"{f.force_n:+.2f} N"),
    ("力矩", lambda f: f"{f.torque_nm:+.3f} Nm"),
    ("速度参考", lambda f: f"{f.vel_ref_mm_s:.1f} mm/s"),
    ("误差", lambda f: "—" if f.err_mm is None else f"{f.err_mm:+.2f} mm"),
    ("MOS 温度", lambda f: f"{f.temperature_mos} °C"),
    ("线圈温度", lambda f: f"{f.temperature_coil} °C"),
)

_HEALTH_ROWS = (
    ("运动状态", lambda f: f.motion_state),
    ("已抓取", lambda f: "是" if f.grasped else "否"),
    ("状态帧", lambda f: f"{f.rx_frames}"),
    ("帧速率", lambda f: f"{f.rx_hz:.0f} Hz"),
    ("链路静默", lambda f: f"{f.stale_ms:.0f} ms"),
    ("tick 耗时", lambda f: f"{f.cycle_ms:.2f} ms"),
    ("tick 超时", lambda f: f"{f.overruns}"),
    ("错误码", lambda f: f"0x{f.error_code:X}"),
)

#: The rows whose text is a temperature, and the thresholds that colour them.
_HOT_ROWS = {"MOS 温度": constants.TEMP_MOS_WARN_C, "线圈温度": constants.TEMP_COIL_WARN_C}


class StatusPage(QWidget):
    """Read-only, and deliberately without a single control on it.

    清除故障 lives on the connection bar rather than here.  A fault is a
    precondition for everything else the console does, so the control that
    clears it belongs next to the one that enables the motor, not on a page the
    operator has to go looking for.
    """

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._values: dict[str, QLabel] = {}
        self._frame: TelemetryFrame | None = None
        self._gate: GateState | None = None
        self._gate_reason = ""

        # Only the fault drives this one.  Alerts from the worker go to the
        # window's banner: a note painted here would be erased by the next
        # published frame, twenty milliseconds later.
        self._fault_banner = Banner()
        self._gate_banner = Banner()
        self._link_dot = StatusDot("idle", "未连接")
        self._fault_dot = StatusDot("idle", "未知")

        readings = self._panel("实时读数", _READING_ROWS)
        health = self._panel("链路与回路", _HEALTH_ROWS)

        columns = QHBoxLayout()
        columns.setSpacing(theme.GAP)
        columns.addWidget(readings, 1)
        columns.addWidget(health, 1)

        state = QGroupBox("连接与故障")
        state_layout = QHBoxLayout(state)
        state_layout.setSpacing(8)
        state_layout.addWidget(self._link_dot)
        state_layout.addWidget(self._fault_dot)
        state_layout.addStretch(1)

        layout = QVBoxLayout(self)
        layout.setSpacing(theme.GAP)
        layout.setContentsMargins(*theme.PAGE_MARGINS)
        layout.addWidget(self._gate_banner)
        layout.addWidget(self._fault_banner)
        layout.addLayout(columns)
        layout.addWidget(state)
        layout.addStretch(1)
        theme.subscribe(self.restyle)

    def _panel(self, title: str, rows) -> QGroupBox:
        box = QGroupBox(title)
        form = QFormLayout(box)
        form.setLabelAlignment(Qt.AlignLeft)
        form.setFormAlignment(Qt.AlignLeft | Qt.AlignTop)
        form.setHorizontalSpacing(16)
        form.setVerticalSpacing(7)
        for label, _render in rows:
            key = QLabel(label)
            key.setProperty("role", "field")
            value = QLabel("—")
            value.setTextFormat(Qt.RichText)
            self._values[label] = value
            form.addRow(key, value)
        return box

    def restyle(self) -> None:
        """Repaint everything this page holds in the current palette.

        The values are written as an inline stylesheet because the colour is a
        *decision made per frame* — a temperature past its threshold is red —
        and that decision is re-made here against the new palette.  The banners
        and the chips repaint themselves, so they are not re-driven here.
        """
        if self._frame is not None:
            self.update_frame(self._frame)
        if self._gate is not None:
            self.set_gate(self._gate, self._gate_reason)

    # ── slots ───────────────────────────────────────────────────────────────
    def update_frame(self, frame: TelemetryFrame) -> None:
        self._frame = frame
        for label, render in _READING_ROWS + _HEALTH_ROWS:
            text = render(frame)
            self._values[label].setText(text)
            colour = self._colour(label, frame)
            self._values[label].setStyleSheet(
                f"font-family: {theme.MONO_FAMILY};"
                f" font-size: {theme.FONT_PX}px;"
                f" font-weight: 600;"
                f" color: {colour};"
            )

        state, text = link_state(frame)
        self._link_dot.set(state, text if frame.enabled else self._not_addressed(text))
        self._refresh_fault(frame)

    def set_gate(self, state: GateState, reason: str) -> None:
        self._gate = state
        self._gate_reason = reason
        if state is GateState.READY:
            self._gate_banner.set(None)
            return
        self._gate_banner.set(
            "error" if state is GateState.BLOCKED else "warn",
            f"运动闸门：{'已阻断' if state is GateState.BLOCKED else '出厂标定待确认'}",
            reason,
        )

    def set_conn_state(self, state: str, detail: str = "") -> None:
        """Take the dots away from the frames when the link goes down.

        A frame is evidence, and evidence outlives its subject: the last one
        received goes on saying "250 Hz" and "无故障", both in green, over a
        cable that has been pulled — in the same window as a bar that says
        未连接.  Connecting is nothing to do here, because the next frame
        restores both dots under telemetry's authority a few milliseconds later;
        only losing the link needs saying, because nothing else will say it.

        The fault *banner* is left standing.  A dot claims the present, so a
        stale one claims health that is not there; the banner reports a fault
        that did happen, and under-reporting that is the harmless direction.
        """
        if state == CONN_CONNECTED:
            return
        self._link_dot.set(*connection_dot(state, detail))
        self._fault_dot.set("idle", "未连接（无故障信息）")

    # ── rendering ───────────────────────────────────────────────────────────
    def _not_addressed(self, text: str) -> str:
        """The link dot's second line while the motor is not being addressed.

        ``link_state`` reports "未使能" for this case, which is right on a bar
        whose job is connection control; on a status page the interesting part
        is that silence here proves nothing about the bus.
        """
        return f"{text}（无数据）"

    def _colour(self, label: str, frame: TelemetryFrame) -> str:
        if label in _HOT_ROWS:
            threshold = _HOT_ROWS[label]
            value = (
                frame.temperature_mos
                if label == "MOS 温度"
                else frame.temperature_coil
            )
            if value >= threshold:
                return theme.ERROR
            if value >= threshold - 10:
                return theme.WARN
        if label == "错误码" and frame.error_code not in constants.OK_ERROR_CODES:
            return theme.ERROR
        if label == "链路静默" and frame.stale_ms >= constants.LINK_STALE_MS:
            return theme.ERROR
        if label == "tick 超时" and frame.overruns:
            return theme.WARN
        return theme.TEXT

    def _refresh_fault(self, frame: TelemetryFrame) -> None:
        """Never call a disabled motor fault-free.

        A disabled DM4310 reports ``error_code`` 0, which the fault table reads
        as "no fault" — but the console is not receiving status frames at all in
        that state, so the truthful reading is that there is nothing to report
        rather than that the hardware is well.
        """
        if not frame.enabled:
            self._fault_dot.set("idle", "未使能（无故障信息）")
            self._fault_banner.set(None)
            return
        state, text = fault_state(frame)
        self._fault_dot.set(state, text)
        if state == "ok":
            self._fault_banner.set(None)
            return
        hint = constants.FAULT_HINTS.get(frame.error_code, "")
        self._fault_banner.set("error", text, hint)

    @property
    def value(self) -> dict[str, str]:
        """The rendered labels, for tests."""
        return {label: widget.text() for label, widget in self._values.items()}

    @property
    def fault_text(self) -> str:
        return self._fault_dot.label

    @property
    def link_text(self) -> str:
        return self._link_dot.label

    @property
    def fault_alert(self) -> tuple[str | None, str, str]:
        """The fault banner's severity, headline and detail, for tests."""
        return (self._fault_banner.severity, self._fault_banner.headline,
                self._fault_banner.detail)

    @property
    def gate_alert(self) -> tuple[str | None, str, str]:
        return self._gate_banner.severity, self._gate_banner.headline, self._gate_banner.detail
