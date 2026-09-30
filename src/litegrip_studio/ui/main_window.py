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
to see without knowing which page to open.  It shows the alert *in force* and no
history: the worker retracts it when the condition it named stops holding
(``alert_cleared``), and the ones it does not retract are the reports of a single
event — a save that failed, a probe that ended — which the next alert replaces.
The log is the record either way.

*The log*, and the log is worth a word.  Every command the console sends is
logged by the worker at debug level with its source, and the file gets
everything; the dock shows the same stream with debug hidden behind a checkbox
and state transitions interleaved, so that what is on screen after an unexpected
movement is enough to reconstruct what was asked for and when.

*The theme switch* is here for the same reason the E-stop is: it changes how
everything looks, so it belongs where it is reachable from every page, and it
belongs to the window rather than to a page that would then have to be revisited
to change it back.  The window is also the one object that outlives every page,
which makes it the natural place to hold the subscription that tells them the
palette moved.
"""

from __future__ import annotations

import logging

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QDockWidget,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QShortcut,
    QSizePolicy,
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
from .calibration_page import CalibrationPage, manual_active
from .connect_bar import ConnectBar
from .control_page import ControlPage
from .plots_page import PlotsPage
from .status_page import StatusPage
from .widgets import UNKNOWN, Banner, ThemeSwitch

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
        self._gate: GateState | None = None
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
        self.theme_switch = ThemeSwitch()
        self._log = QPlainTextEdit()
        self._log_dock = QDockWidget("日志")
        self._show_debug = QCheckBox("显示调试信息")
        self._status_position = QLabel("—")
        self._status_gate = QLabel("—")
        self._status_message = QLabel("")

        self._build()
        self._wire()
        self._restore()
        self.restyle()
        theme.subscribe(self.restyle)

    # ── construction ────────────────────────────────────────────────────────
    def _build(self) -> None:
        self.estop_button.setProperty("danger", True)
        # The type size comes from the stylesheet, under this role; the size
        # policy is here because a QPushButton is vertically Fixed by default
        # and would ignore the card's stretch, leaving the button its natural
        # height in a mostly empty panel.
        self.estop_button.setProperty("role", "estop")
        self.estop_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.estop_button.setMinimumHeight(52)
        self.estop_button.setMinimumWidth(150)
        self.estop_button.setToolTip("立即零力矩并失能，闩锁；复位前拒绝一切运动")

        # The E-stop gets a card of its own, filling the height of the row.
        #
        # It used to float beside the connection card as a lone button, and it
        # read as an afterthought: two controls centred against a card four
        # times their height, with dead space above and below.  A card makes the
        # right-hand end of the header deliberate, and a tall target is the
        # right shape for the one control that has to be hit without looking.
        safety = QGroupBox("安全")
        safety_layout = QVBoxLayout(safety)
        safety_layout.setSpacing(theme.GAP)
        safety_layout.addWidget(self.estop_button, 1)

        top = QHBoxLayout()
        top.setSpacing(theme.GAP)
        top.addWidget(self.connect_bar, 1)
        top.addWidget(safety, 0)

        for page, name in zip(
            (self.control_page, self.status_page, self.plots_page, self.calibration_page),
            TAB_NAMES,
        ):
            self.tabs.addTab(page, name)
        # Document mode drops the pane's bevel, which is the difference between
        # a tab strip and a segmented control; the base line is the groove a
        # classic tab bar is seated in, and a segmented control has no groove.
        self.tabs.setDocumentMode(True)
        self.tabs.tabBar().setDrawBase(False)

        central = QWidget()
        central.setObjectName("Shell")
        layout = QVBoxLayout(central)
        # The gutter belongs to the *window*, not to the central widget: a
        # QDockWidget is a child of the window rather than of the central
        # widget, so a gutter set here inset every card and the log panel did
        # not follow — the log's frame started 10px left of everything else.
        # One gutter, in one place, is what keeps the header, the pages and the
        # dock on the same two vertical lines.
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(theme.GAP)
        layout.addLayout(top)
        layout.addWidget(self.estop_banner)
        layout.addWidget(self.alert_banner)
        layout.addWidget(self.tabs, 1)
        self.setCentralWidget(central)

        self._log.setReadOnly(True)
        self._log.setMaximumBlockCount(LOG_LINES)
        # The header and the body are two widgets, and the card they belong to
        # is the pair of them: QFrame rather than QWidget because a plain
        # QWidget does not paint a stylesheet background unless it is told to,
        # and these two carry the panel the log sits on.
        dock_body = QFrame()
        dock_body.setObjectName("LogBody")
        dock_body.setFrameShape(QFrame.NoFrame)
        dock_layout = QVBoxLayout(dock_body)
        dock_layout.setContentsMargins(theme.PAD + 2, theme.PAD, theme.PAD + 2, theme.PAD + 2)
        dock_layout.addWidget(self._log, 1)
        self._log_dock.setWidget(dock_body)
        self._log_dock.setTitleBarWidget(self._build_log_title())
        self._log_dock.setFeatures(
            QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable
        )
        self.addDockWidget(Qt.BottomDockWidgetArea, self._log_dock)
        self.setContentsMargins(theme.GAP, theme.GAP, theme.GAP, theme.GAP)
        # The dock opens shallow.  Its default height comes from the text
        # area's size hint, which is a good deal taller than a log usually has
        # anything to say — and every pixel of it comes off the calibration
        # page, which is the one page that is taller than the window.
        self.resizeDocks([self._log_dock], [150], Qt.Vertical)

        # A status bar is the only thing QMainWindow puts *below* the dock area,
        # so the row lives in one — but as a single expanding permanent widget,
        # not through addWidget.  Everything the status bar offers for content
        # (addWidget, showMessage) is pinned to its left end, and the left end is
        # exactly where this console does not want the message.
        bar = QStatusBar()
        bar.setSizeGripEnabled(False)
        bar.addPermanentWidget(self._build_state_line(), 1)
        self.setStatusBar(bar)
        self.resize(1120, 780)
        self.setWindowTitle("LiteGrip 夹爪控制台")

    def _build_state_line(self) -> QWidget:
        """The row along the bottom: the switch, the readouts, then the message.

        Inside a status bar, but not *of* one: the message is a label of this
        row's own rather than ``QStatusBar.showMessage``, because everything the
        status bar lays out for content is pinned to its left end, and the left
        end is exactly where this console does not want the message.  What the
        operator set and what the machine is doing come first; "正在使能…" goes
        after them.
        """
        def field(caption: str) -> QLabel:
            label = QLabel(caption)
            label.setProperty("role", "field")
            return label

        self._status_message.setProperty("role", "field")

        line = QWidget()
        line.setObjectName("StateLine")
        row = QHBoxLayout(line)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(theme.GAP)
        # The switch carries no caption.  Its two names are the label — that is
        # the whole reason it is a pair of pills and not one button — and a
        # "主题" in front of 深色/浅色 only says a third time what the two of
        # them already say.
        row.addWidget(self.theme_switch)
        for caption, widget in (
            ("位置", self._status_position),
            ("闸门", self._status_gate),
        ):
            row.addWidget(field(caption))
            row.addWidget(widget)
        row.addStretch(1)
        row.addWidget(self._status_message)
        self.theme_switch.chosen.connect(self._on_choose_theme)
        return line

    def _build_log_title(self) -> QWidget:
        """The log dock's header: its name, and the filter that belongs to it.

        A custom title bar rather than the default one, because the default is
        drawn by the style and cannot hold a widget.  That is why the debug
        filter used to sit on a row of its own *inside* the body, where it
        started 10px to the left of the title above it — the title bar carries
        its own padding and the body did not.  A control about the log belongs
        on the log's header; putting it there removes both the orphan row and
        the misalignment that made it look like an overflow.
        """
        bar = QFrame()
        bar.setObjectName("LogTitle")
        bar.setFrameShape(QFrame.NoFrame)
        layout = QHBoxLayout(bar)
        # The same inset as the body below it, so the header's label starts in
        # the column the log panel starts in.  The vertical room is what makes
        # it read as a header rather than as a caption.
        layout.setContentsMargins(theme.PAD + 2, theme.PAD, theme.PAD + 2, theme.PAD)
        layout.setSpacing(theme.GAP)
        caption = QLabel("日志")
        caption.setProperty("role", "title")
        layout.addWidget(caption)
        layout.addStretch(1)
        layout.addWidget(self._show_debug)
        return bar

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
        worker.alert_cleared.connect(self._on_alert_cleared)
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

    # ── the theme ───────────────────────────────────────────────────────────
    def _on_choose_theme(self, name: str) -> None:
        """Switch the palette — on the application as well as in the tokens.

        ``app=`` is what pushes the new stylesheet and palette out to every
        widget; without it the tokens move and only the widgets that repaint
        themselves follow.  Nothing else happens here: this window is a
        subscriber, so :meth:`restyle` puts the switch's own highlight, the
        log's colours and the status bar's gate word right on its own.
        """
        theme.set_theme(name, app=QApplication.instance())
        if self._settings is not None:
            self._settings.theme = name

    def restyle(self) -> None:
        """Re-render what a stylesheet cannot reach.

        The log is written as markup with a colour per line and the status bar's
        gate label with the colour of the gate, so both are rebuilt here from
        what they already hold rather than from the worker.
        """
        self.theme_switch.set_current(theme.current_theme())
        self._log.setStyleSheet(
            f"font-family: {theme.MONO_FAMILY}; font-size: {theme.FONT_SMALL_PX}px;"
        )
        self._replay_log()
        if self._gate is not None:
            self._render_gate(self._gate)

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
        """A fault code, or — with code 0 — a notice that is not a fault.

        The worker sends both down this signal because both are about the drive,
        but only one of them is a thing to redden the window over.  A code of 0
        is the notice sentinel (``ERROR_DISABLED``), and the two notices that use
        it are the E-stop engaging and being released: the latch has its own
        banner with the reset button in it, set and cleared by the motion state,
        so painting these red as well would leave the E-stop's text up after the
        operator had dealt with it.  They go to the log, which is where the
        timeline of an E-stop is read anyway.
        """
        text = f"{message}（{hint}）" if hint else message
        if code == constants.ERROR_DISABLED:
            self._append("warn", text)
            return
        self._alert("error", text)

    def _on_gate_state(self, state: str, reason: str) -> None:
        gate = GateState(state)
        self.control_page.set_gate(gate, reason)
        self.status_page.set_gate(gate, reason)
        self.calibration_page.set_gate(gate, reason)
        self._gate = gate
        self._render_gate(gate)

    def _render_gate(self, gate: GateState) -> None:
        """The status bar's gate word, in the colour of the gate."""
        colour = {
            GateState.READY: theme.OK,
            GateState.FACTORY: theme.WARN,
            GateState.BLOCKED: theme.ERROR,
        }[gate]
        self._status_gate.setText(
            f'<span style="color:{colour};font-family:{theme.MONO_FAMILY};'
            f'font-weight:600">{gate.value}</span>'
        )

    def _on_calib_info(self, info) -> None:
        self.control_page.set_calibration(info)
        self.plots_page.set_limits(info.limits if info is not None else None)
        self.calibration_page.set_calibration(info)

    def _on_calib_progress(self, phase: str, progress: float, note: str) -> None:
        self.calibration_page.set_progress(phase, progress, note)
        if manual_active(phase):
            # A manual probe is the one the operator drives themselves, and the
            # buttons that say "the jaws are at the open extreme" are on this
            # page and nowhere else: a window left on another tab is an operator
            # holding a limp gripper with no way to record where it is.  A
            # guided probe steers itself and asks for nothing mid-step, so it
            # does not take the tab the operator chose.
            self.tabs.setCurrentWidget(self.calibration_page)

    def _on_log(self, level: str, text: str) -> None:
        self._append(level, text)

    def _on_alert(self, level: str, text: str) -> None:
        self._alert(level, text)

    def _on_alert_cleared(self) -> None:
        """The worker has decided the line it put up is no longer true."""
        self.alert_banner.set(None)

    def _on_busy(self, busy: bool, what: str) -> None:
        self.calibration_page.set_busy(busy, what)
        # Set *and* cleared.  The status bar this replaced only ever set it —
        # showMessage with no timeout stays up until something else clears it —
        # so the first busy spell of a session held the line for the rest of it.
        self._status_message.setText(what if busy else "")

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
        """Put one line on the banner — except a success, which is not one.

        ``info`` is what the worker uses for something that went *right* (a
        calibration that was written out), and a banner is for what the operator
        has to act on: a strip saying the save worked is a thing to dismiss
        rather than to read, and it never went away by itself.  It goes to the
        dock instead, where the record is, so the banner carries ``warn`` and up.
        """
        if level == "info":
            self._append("info", text)
            return
        severity = "error" if level in ("error", "fatal") else "warn"
        self.alert_banner.set(severity, text)

    # ── closing ─────────────────────────────────────────────────────────────
    def closeEvent(self, event) -> None:  # noqa: N802 - Qt's name
        self.control_page.persist()
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
