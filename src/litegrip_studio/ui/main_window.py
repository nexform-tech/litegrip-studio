"""The window: the four pages, the E-stop, and the log.

Three things live here rather than on a page, because each of them has to be
reachable whatever the operator is looking at.

*The E-stop*, on the Esc key as well as the button, and on an application-wide
shortcut so that a focused spinbox or a modal dialog cannot swallow it.  It goes
through ``worker.estop()`` and not through the command queue: an emergency stop
that waits its turn behind fifty coalesced drag commands is not an emergency
stop.

The application-wide scope has a cost that is worth naming: Esc stops the
gripper even while a file dialog is up, where the operator may have meant it to
close the dialog.  The failure goes the safe way — a spurious stop de-energises a
motor and may drop what it is holding, while a swallowed one leaves the machine
moving — and the button is on screen at all times, so the key is a convenience
rather than the only way to reach it.

*The alert banner*, because an alert is by definition something the operator has
to see without knowing which page to open.

*The log*, and the log is worth a word.  Every command the console sends is
logged by the worker at debug level with its source, and the file gets
everything; the dock shows the same stream with debug hidden behind a checkbox
and state transitions interleaved, so that what is on screen after an unexpected
movement is enough to reconstruct what was asked for and when.
"""

from __future__ import annotations

import logging

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import (
    QCheckBox,
    QDockWidget,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QShortcut,
    QStatusBar,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from .. import constants
from ..core import commands as cmd
from ..core.worker import GateState
from ..settings import Settings
from ..telemetry import TelemetryFrame
from . import theme
from .calibration_page import CalibrationPage
from .connect_bar import ConnectBar
from .control_page import ControlPage
from .plots_page import PlotsPage
from .status_page import StatusPage
from .widgets import UNKNOWN, Banner

#: Lines kept in the dock.  The file has everything; this is what a person can
#: scroll through after something surprising happened.
LOG_LINES = 2000

TAB_NAMES = ("控制", "状态", "曲线", "标定")


class MainWindow(QMainWindow):
    """Tabs, the E-stop, the log dock and the wiring between worker and pages."""

    def __init__(self, worker, settings: Settings | None = None,
                 parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.worker = worker
        self._settings = settings
        self._estopped = False
        self._lines: list[tuple[str, str]] = []

        self.tabs = QTabWidget()
        self.control_page = ControlPage(worker.submit, settings)
        self.status_page = StatusPage()
        self.plots_page = PlotsPage()
        self.calibration_page = CalibrationPage(
            worker.submit, worker.set_allow_factory, settings
        )
        self.connect_bar = ConnectBar(worker.submit)

        self.alert_banner = Banner()
        self.estop_banner = Banner()
        self.estop_button = QPushButton("急停 (Esc)")
        self._log = QPlainTextEdit()
        self._log_dock = QDockWidget("日志")
        self._show_debug = QCheckBox("显示调试信息")
        self._status_position = QLabel("—")
        self._status_gate = QLabel("—")

        self._build()
        self._wire()
        self._restore()

    # ── construction ────────────────────────────────────────────────────────
    def _build(self) -> None:
        self.estop_button.setProperty("danger", True)
        self.estop_button.setMinimumHeight(52)
        self.estop_button.setMinimumWidth(150)
        self.estop_button.setToolTip("立即零力矩并失能，闩锁；复位前拒绝一切运动")

        top = QHBoxLayout()
        top.setSpacing(10)
        top.addWidget(self.connect_bar, 1)
        top.addWidget(self.estop_button, 0)

        for page, name in zip(
            (self.control_page, self.status_page, self.plots_page, self.calibration_page),
            TAB_NAMES,
        ):
            self.tabs.addTab(page, name)

        central = QWidget()
        layout = QVBoxLayout(central)
        layout.setSpacing(8)
        layout.addLayout(top)
        layout.addWidget(self.estop_banner)
        layout.addWidget(self.alert_banner)
        layout.addWidget(self.tabs, 1)
        self.setCentralWidget(central)

        self._log.setReadOnly(True)
        self._log.setMaximumBlockCount(LOG_LINES)
        self._log.setStyleSheet(f"font-family: {theme.MONO_FAMILY}; font-size: 11px;")
        dock_body = QWidget()
        dock_layout = QVBoxLayout(dock_body)
        dock_layout.setContentsMargins(4, 4, 4, 4)
        dock_layout.addWidget(self._show_debug)
        dock_layout.addWidget(self._log, 1)
        self._log_dock.setWidget(dock_body)
        self._log_dock.setFeatures(
            QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable
        )
        self.addDockWidget(Qt.BottomDockWidgetArea, self._log_dock)

        bar = QStatusBar()
        bar.addPermanentWidget(QLabel("位置"))
        bar.addPermanentWidget(self._status_position)
        bar.addPermanentWidget(QLabel("闸门"))
        bar.addPermanentWidget(self._status_gate)
        self.setStatusBar(bar)
        self.resize(1000, 720)
        self.setWindowTitle("LiteGrip 夹爪控制台")

    def _wire(self) -> None:
        worker = self.worker
        self.estop_button.clicked.connect(self._on_estop)
        # Application-wide, so the shortcut still fires while a spinbox has
        # focus or a modal dialog is up: the one key that must never be eaten.
        self._estop_shortcut = QShortcut(QKeySequence(Qt.Key_Escape), self)
        self._estop_shortcut.setContext(Qt.ApplicationShortcut)
        self._estop_shortcut.activated.connect(self._on_estop)

        self._show_debug.toggled.connect(self._replay_log)

        worker.telemetry.connect(self._on_telemetry)
        worker.motion_state.connect(self._on_motion_state)
        worker.conn_state.connect(self._on_conn_state)
        worker.fault.connect(self._on_fault)
        worker.gate_state.connect(self._on_gate_state)
        worker.calib_info.connect(self._on_calib_info)
        worker.calib_progress.connect(self._on_calib_progress)
        worker.log.connect(self._on_log)
        worker.alert.connect(self._on_alert)
        worker.busy.connect(self._on_busy)

        self._heartbeat = QTimer(self)
        self._heartbeat.setInterval(constants.HEARTBEAT_INTERVAL_MS)
        self._heartbeat.timeout.connect(
            lambda: self.worker.submit(cmd.Heartbeat())
        )
        self._heartbeat.start()

    def _restore(self) -> None:
        if self._settings is None:
            return
        state = self._settings.window_state
        if state:
            self.restoreGeometry(state)
        index = self._settings.active_tab
        if 0 <= index < self.tabs.count():
            self.tabs.setCurrentIndex(index)
        self._show_debug.setChecked(self._settings.show_debug_log)

    def start(self) -> None:
        """Show what the backend is, before a single frame has arrived."""
        description = getattr(self.worker.backend, "describe", None)
        if callable(description):
            self.connect_bar.set_backend_description(description())

    # ── slots ───────────────────────────────────────────────────────────────
    def _on_telemetry(self, frame: TelemetryFrame) -> None:
        self.control_page.update_frame(frame)
        self.status_page.update_frame(frame)
        self.plots_page.update_frame(frame)
        self.calibration_page.update_frame(frame)
        self.connect_bar.update_frame(frame)
        self._status_position.setText(
            UNKNOWN if frame.position_mm is None else f"{frame.position_mm:7.2f} mm"
        )

    def _on_motion_state(self, state: str) -> None:
        # The worker does not log these itself, and after an unexpected movement
        # "when did it leave SERVO" is the first question asked of the log.
        self._append("state", f"运动状态：{state}")
        # ESTOP is the one state that is also a thing the operator has to act
        # on, and it stays set until they do: the banner is the reset button's
        # only address, so it has to appear and disappear with the latch.
        if state == "ESTOP":
            self.set_estopped(True)
        elif self._estopped:
            self.set_estopped(False)

    def _on_conn_state(self, state: str, detail: str) -> None:
        """Three readers, and each wants it for a different reason.

        The bar owns the 连接/使能 buttons.  The status page has to stop trusting
        the last frame the moment there will not be a next one.  And the
        calibration page gates its probe buttons on being connected — which is
        the only reason a console can be connected, enabled, and still offer no
        way to calibrate.
        """
        self.connect_bar.set_conn_state(state, detail)
        self.status_page.set_conn_state(state, detail)
        self.calibration_page.set_conn_state(state, detail)

    def _on_fault(self, code: int, message: str, hint: str) -> None:
        self._alert("error", f"{message}（{hint}）" if hint else message)

    def _on_gate_state(self, state: str, reason: str) -> None:
        gate = GateState(state)
        self.control_page.set_gate(gate, reason)
        self.status_page.set_gate(gate, reason)
        self.calibration_page.set_gate(gate, reason)
        colour = {
            GateState.READY: theme.OK,
            GateState.FACTORY: theme.WARN,
            GateState.BLOCKED: theme.ERROR,
        }[gate]
        self._status_gate.setText(
            f'<span style="color:{colour}">{gate.value}</span>'
        )

    def _on_calib_info(self, info) -> None:
        self.control_page.set_calibration(info)
        self.plots_page.set_limits(info.limits if info is not None else None)
        self.calibration_page.set_calibration(info)

    def _on_calib_progress(self, phase: str, progress: float, note: str) -> None:
        self.calibration_page.set_progress(phase, progress, note)
        if phase == "RECORDING":
            # The slider is disabled during a probe — there is no valid travel
            # to span yet — so the page that has the only live reading is the
            # one the operator needs to be looking at.
            self.tabs.setCurrentWidget(self.calibration_page)

    def _on_log(self, level: str, text: str) -> None:
        self._append(level, text)

    def _on_alert(self, level: str, text: str) -> None:
        self._alert(level, text)

    def _on_busy(self, busy: bool, what: str) -> None:
        self.calibration_page.set_busy(busy, what)
        if busy:
            self.statusBar().showMessage(what)

    def _on_estop(self) -> None:
        self.worker.estop("手动急停（界面按钮）")

    def _on_reset_estop(self) -> None:
        self.worker.submit(cmd.ResetEStop())

    def set_estopped(self, estopped: bool) -> None:
        """Called when the worker reports the latch, and by the tests."""
        self._estopped = estopped
        if not estopped:
            self.estop_banner.set(None)
            return
        self.estop_banner.set(
            "error", "急停已闩锁",
            "电机已失能，复位前拒绝一切运动。确认危险已排除后再复位。",
        )
        if not self.estop_banner.findChild(QPushButton):
            button = QPushButton("急停复位")
            button.clicked.connect(self._on_reset_estop)
            self.estop_banner.add_action(button)

    # ── the log ─────────────────────────────────────────────────────────────
    def _append(self, level: str, text: str) -> None:
        self._lines.append((level, text))
        if not self._show_debug.isChecked() and level == "debug":
            return
        self._write(level, text)

    def _write(self, level: str, text: str) -> None:
        self._log.appendHtml(
            f'<span style="color:{theme.level_colour(level)}">{text}</span>'
        )

    def _replay_log(self) -> None:
        self._log.clear()
        for level, text in self._lines:
            if level == "debug" and not self._show_debug.isChecked():
                continue
            self._write(level, text)

    def _alert(self, level: str, text: str) -> None:
        severity = "error" if level in ("error", "fatal") else "warn"
        self.alert_banner.set(severity, text)

    # ── closing ─────────────────────────────────────────────────────────────
    def closeEvent(self, event) -> None:  # noqa: N802 - Qt's name
        self.control_page.persist()
        self.calibration_page.persist()
        if self._settings is not None:
            self._settings.window_state = bytes(self.saveGeometry())
            self._settings.active_tab = self.tabs.currentIndex()
            self._settings.show_debug_log = self._show_debug.isChecked()
        self._heartbeat.stop()

        if not self.worker.shutdown():
            # The thread did not stop inside the timeout.  The motor may still be
            # energised, and the process teardown will end it — but the operator
            # has to be told, because "the window closed" is not the same as
            # "the motor is off".
            message = "控制线程未在超时内退出；电机可能仍处于使能状态，请检查供电"
            self._append("fatal", message)
            logging.getLogger(__name__).error(message)
        event.accept()

    @property
    def lines(self) -> list[tuple[str, str]]:
        """Everything the log received, including what the filter is hiding."""
        return list(self._lines)
