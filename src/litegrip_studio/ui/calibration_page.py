"""The calibration page: where the numbers that make millimetres mean something come from.

The page answers the question the console cannot answer by itself — *is the
calibration loaded right now the one for this gripper?*  The SDK resolves two
files silently, preferring the user's and falling back to a factory file shipped
inside the package, and both paths return ``True``.  On a real gripper that
fallback means every millimetre and every newton on screen is wrong with nothing
on screen to say so, so the provenance is read here from the files themselves and
displayed with the two angles, the coefficient and the travel it implies.

Two probes are offered and a third is not.

*Automatic* steps the jaws into each hard stop and records where they stopped.
It is a rewrite of the SDK's ``calibrate_guided``, which waits on ``stdin`` —
which a GUI cannot answer, and which in a worker thread never receives anything
at all — and whose motion loop has no abort hook.  The confirmation it waits for
is a button here, and the button says which limit it is confirming.  It is the
one an operator runs first on a gripper nobody has calibrated.

*Manual* holds the axis limp and records the two extremes the operator moves
the jaws to, one labelled point each: open first, then closed, which is 0 mm.
The SDK's version sweeps the axis for a fixed duration with the motor and takes
the extremes it passed through — an extreme a hand swept past is not a limit
anyone measured, and with the two points unnamed the result cannot say which end
is 0 mm.  Here the operator says which point they are recording, so the direction
comes out of the labels rather than being declared beforehand or assumed.

The SDK's own ``calibrate()`` — one sweep, start to finish, with no way in — is
deliberately absent.  It moves the jaws for about twenty-four seconds and cannot
be interrupted, and a stop button that does nothing for half a minute is worse
than not offering it.
"""

from __future__ import annotations

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QCheckBox,
    QFileDialog,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QProgressBar,
    QPushButton,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from .. import calibration, constants
from ..calibration import CalibrationInfo
from ..core import commands as cmd
from ..core.calibration_fsm import GuidedPhase, TwoPointPhase
from ..core.worker import GateState
from ..settings import Settings
from ..telemetry import TelemetryFrame
from . import theme
from .widgets import UNKNOWN, Banner

#: Phases that mean no probe of that kind is running, so a new one may be
#: started.
GUIDED_IDLE_PHASES = frozenset(
    phase.value for phase in (GuidedPhase.IDLE, GuidedPhase.DONE,
                              GuidedPhase.FAILED, GuidedPhase.CANCELLED)
)
MANUAL_IDLE_PHASES = frozenset(
    phase.value for phase in (TwoPointPhase.IDLE, TwoPointPhase.DONE,
                              TwoPointPhase.FAILED, TwoPointPhase.CANCELLED)
)

#: The phases each probe owns while it runs.  Both report through one progress
#: signal and their idle phases are spelled the same, so "some probe is running"
#: is never the question a widget is asking — "is *this* probe running" is, and
#: only the phase *name* can answer it.  Testing against the idle set instead
#: would call every one of the other probe's phases active.
GUIDED_ACTIVE_PHASES = frozenset(
    phase.value for phase in GuidedPhase
) - GUIDED_IDLE_PHASES
MANUAL_ACTIVE_PHASES = frozenset(
    phase.value for phase in TwoPointPhase
) - MANUAL_IDLE_PHASES

#: What the confirmation button says while each limit is being probed.  The SDK
#: waits for the Enter key, which tells the operator nothing about what they are
#: agreeing to.
CONFIRM_LABELS = {
    GuidedPhase.OPEN_PROBE.value: "已到张开极限，确认",
    GuidedPhase.CLOSE_PROBE.value: "已到闭合极限，确认",
}

#: What the confirmation button says between probes, when there is no limit to
#: name.
CONFIRM_DEFAULT = "已到极限，确认"

PHASE_LABELS = {
    GuidedPhase.IDLE.value: "未开始",
    GuidedPhase.OPEN_PROBE.value: "正在顶向张开极限",
    GuidedPhase.OPEN_BACKOFF.value: "正在退回",
    GuidedPhase.CLOSE_PROBE.value: "正在顶向闭合极限",
    GuidedPhase.DONE.value: "完成",
    GuidedPhase.FAILED.value: "失败",
    GuidedPhase.CANCELLED.value: "已取消",
    TwoPointPhase.RECORD_OPEN.value: "等待记录张开极限",
    TwoPointPhase.RECORD_CLOSE.value: "等待记录闭合极限",
}

#: The severity to paint each provenance with.  The factory case is a warning
#: rather than an error because it is surmountable — see the worker's gate.
PROVENANCE_SEVERITY = {
    calibration.PROVENANCE_USER: "info",
    calibration.PROVENANCE_FACTORY: "warn",
    calibration.PROVENANCE_MEMORY: "warn",
    calibration.PROVENANCE_INVALID: "error",
    calibration.PROVENANCE_MISSING: "error",
}


def guided_active(phase: str) -> bool:
    """Whether this phase belongs to a guided probe that is still running."""
    return phase in GUIDED_ACTIVE_PHASES


def manual_active(phase: str) -> bool:
    """Whether this phase belongs to a manual probe that is still running."""
    return phase in MANUAL_ACTIVE_PHASES


class CalibrationPage(QWidget):
    """Provenance, file handling, and the two probes."""

    def __init__(self, submit, allow_factory=None, settings: Settings | None = None,
                 parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._submit = submit
        self._allow_factory = allow_factory
        self._settings = settings
        self._info: CalibrationInfo | None = None
        self._phase = GuidedPhase.IDLE.value
        self._connected = False
        self._busy = False
        self._gate: GateState | None = None
        self._gate_reason = ""
        self._frame: TelemetryFrame | None = None
        #: The progress note as plain text, kept because ``_note`` holds it as
        #: markup and a repaint cannot read the plain words back out of it.
        self._note_text = ""

        self._banner = Banner()
        self._file_label = QLabel("—")
        self._file_label.setTextFormat(Qt.RichText)
        self._file_label.setWordWrap(True)
        self._table: QFormLayout | None = None
        self._summary: QFormLayout | None = None
        self._table_box = QGroupBox("当前标定")
        self._table_target = QVBoxLayout(self._table_box)
        self._summary_holder = QWidget()
        self._summary_target = QVBoxLayout(self._summary_holder)
        self._summary_target.setContentsMargins(0, 0, 0, 0)

        # The rad values, gains and CAN ids are real and are kept — but behind a
        # checkbox, because they are what an operator reads while something is
        # wrong, not what anyone reads before pressing 闭合.
        self._detail_toggle = QCheckBox("显示技术详情")
        self._detail = QWidget()
        self._detail.setVisible(False)
        self._detail_target = QVBoxLayout(self._detail)
        self._detail_target.setContentsMargins(0, 0, 0, 0)
        self._detail_toggle.toggled.connect(self._detail.setVisible)

        self._gate_label = QLabel()
        self._gate_label.setTextFormat(Qt.RichText)
        self._gate_label.setWordWrap(True)

        self._allow = QCheckBox("我了解风险，允许使用出厂标定值")
        self._allow.setToolTip(
            "出厂标定随软件分发，未必属于这台夹爪。勾选后所有读数都可能是错的"
        )
        self._allow.toggled.connect(self._on_allow_toggled)

        self._reload = QPushButton("重新读取")
        self._load = QPushButton("载入文件…")
        self._save = QPushButton("重新保存标定…")

        self._progress = QProgressBar()
        self._phase_label = QLabel()
        self._phase_label.setTextFormat(Qt.RichText)
        self._note = QLabel()
        self._note.setTextFormat(Qt.RichText)
        self._note.setWordWrap(True)
        self._position = QLabel("—")
        self._position.setProperty("role", "readout")

        self._guided_start = QPushButton("开始自动标定")
        self._guided_confirm = QPushButton(CONFIRM_LABELS[GuidedPhase.OPEN_PROBE.value])
        self._guided_cancel = QPushButton("取消标定")
        self._manual_start = QPushButton("开始手动标定")
        self._manual_open = QPushButton("记录张开极限")
        self._manual_close = QPushButton("记录闭合极限")
        self._manual_cancel = QPushButton("取消标定")

        self._build()
        self._wire()
        self._load_settings()
        self._refresh()
        theme.subscribe(self.restyle)

    # ── construction ────────────────────────────────────────────────────────
    def _build(self) -> None:
        """One scroll area, two rows, and the reasons for both.

        This page is the tallest thing in the window by a wide margin, and Qt
        will not let a window shrink below its tallest page's minimum size.  A
        page whose content sets the floor for the whole console is how a
        calibration tab ends up making the main window taller than the screen it
        is on — which takes the title bar off the screen and leaves the operator
        holding a window they cannot move.  The scroll area breaks that link:
        the content keeps whatever height it needs, and all the window has to
        fit is the frame around it.

        The rows then keep the content close to the size of the window it is
        given, so that the scrollbar stays the exception rather than the normal
        state.  The split is by *what the operator is doing*: the top row is the
        two probes — the actions, one per kind of calibration — and underneath
        them is what they are there to answer, the calibration in force and the
        file it came from.  Actions above their evidence, and the two of them
        side by side so that neither is the "other" one by position.  It used to
        be one stack of six group boxes, which is most of why it stood 890
        pixels tall before anyone had done anything.
        """
        self._guided_start.setProperty("accent", True)
        self._guided_confirm.setProperty("accent", True)
        self._save.setProperty("accent", True)

        probes = QHBoxLayout()
        probes.setSpacing(theme.GAP)
        probes.addWidget(self._build_guided_box(), 1)
        probes.addWidget(self._build_manual_box(), 1)

        current = QHBoxLayout()
        current.setSpacing(theme.GAP)
        current.addWidget(self._build_table_box(), 1)
        current.addWidget(self._build_file_box(), 1)

        content = QWidget()
        inner = QVBoxLayout(content)
        inner.setContentsMargins(0, theme.GAP, 0, theme.GAP + 4)
        inner.setSpacing(theme.GAP)
        inner.addWidget(self._banner)
        inner.addLayout(probes)
        inner.addLayout(current)
        # Anything the window has over stays at the bottom, so the boxes keep
        # their own heights rather than being stretched to fill it.
        inner.addStretch(1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        # Frameless: the tab already draws a frame around this, and two nested
        # ones read as a widget that failed to load rather than as a page.
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setWidget(content)

        rule = QFrame()
        rule.setFrameShape(QFrame.HLine)
        rule.setFrameShadow(QFrame.Sunken)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addWidget(scroll, 1)
        layout.addWidget(rule)
        layout.addWidget(self._build_progress_strip())
        # Paint the idle state once, so the strip opens saying "未开始" with the
        # live reading rather than as a blank label above an empty bar.
        self._render_progress()

    def _build_table_box(self) -> QGroupBox:
        """What the calibration in force actually is."""
        self._table_target.addWidget(self._summary_holder)
        self._table_target.addWidget(self._detail_toggle)
        self._table_target.addWidget(self._detail)
        return self._table_box

    def _build_file_box(self) -> QGroupBox:
        """Where that calibration came from, and how to change it."""
        files = QGridLayout()
        files.setSpacing(8)
        files.addWidget(self._reload, 0, 0)
        files.addWidget(self._load, 0, 1)
        files.addWidget(self._save, 0, 2)

        box = QGroupBox("标定文件")
        layout = QVBoxLayout(box)
        layout.addWidget(self._file_label)
        layout.addLayout(files)
        layout.addWidget(self._allow)
        layout.addWidget(self._gate_label)
        layout.addStretch(1)
        return box

    def _build_guided_box(self) -> QGroupBox:
        """The probe that finds both hard stops by itself."""
        buttons = QHBoxLayout()
        buttons.setSpacing(8)
        buttons.addWidget(self._guided_start)
        buttons.addWidget(self._guided_confirm)
        buttons.addWidget(self._guided_cancel)

        text = QLabel(
            "分别顶向张开与闭合两个硬限位，记录停住的位置。"
            "每一步都会重新锚定在实测位置，因此顶住时的力矩有上限；"
            "夹爪停住不动时会自动判定为已到极限，也可以手动确认。"
        )
        text.setWordWrap(True)

        box = QGroupBox("自动标定（需要电机已使能）")
        layout = QVBoxLayout(box)
        layout.addWidget(text)
        layout.addLayout(buttons)
        return box

    def _build_manual_box(self) -> QGroupBox:
        """The probe where the operator moves the jaws by hand.

        Its buttons are in two rows rather than one.  Start has a row to itself,
        and the two record buttons sit together under it with the cancel — the
        four of them on a single row made this the widest thing on the page, and
        this box sits in a row beside the other probe, so its half of the width
        is what the window's minimum width has to fit.

        The two record buttons are separate and each is live only in its own
        step, rather than one button meaning "record whatever is due".  Which
        end is being recorded is what decides the gripper's direction: the point
        recorded as closed is 0 mm, and a flow where the label can be got wrong
        is one that can save an inverted calibration that looks perfect.
        """
        start_row = QHBoxLayout()
        start_row.setSpacing(8)
        start_row.addWidget(self._manual_start)
        start_row.addWidget(self._manual_cancel)
        start_row.addStretch(1)

        record_row = QHBoxLayout()
        record_row.setSpacing(8)
        record_row.addWidget(self._manual_open)
        record_row.addWidget(self._manual_close)
        record_row.addStretch(1)

        self._manual_open.setToolTip(
            "把两片手指推到张得最大的位置，停住，再按这里。"
            "按下之后取下一控制周期的角度读数作为张开极限。"
        )
        self._guided_start.setToolTip(
            "需要电机已使能，探针记录的是电机上报的角度。"
            "未使能时按钮仍可点击，控制台会说明原因。"
        )
        self._manual_start.setToolTip(
            "需要电机已使能：标定一开始就进入零重力，用手掰动之前电机得先上电。"
            "未使能时按钮仍可点击，控制台会说明原因。"
        )
        self._manual_close.setToolTip(
            "把两片手指合到最小的位置，停住，再按这里。"
            "这一端记为 0 mm，张开那一端记为设定的行程。"
        )

        text = QLabel(
            "标定期间电机零力矩，用手把夹爪分别摆到两个极限，每到一个按一次对应的"
            "「记录」：先记张开极限，再记闭合极限（闭合记为 0 mm）。"
            "两个极限都记到之后标定自动完成并写入文件，不需要再按保存；"
            "写入失败时可以用「重新保存标定…」重试。"
        )
        text.setWordWrap(True)

        box = QGroupBox("手动两点标定（零重力，可用手掰动）")
        layout = QVBoxLayout(box)
        layout.addWidget(text)
        layout.addLayout(start_row)
        layout.addLayout(record_row)
        return box

    def _build_progress_strip(self) -> QWidget:
        """What a running probe is doing, pinned below the scrolling content.

        This used to be the sixth group box in the stack, which put the progress
        bar and the live position at the *bottom* of the page.  The moment this
        page is watched rather than read is the moment a probe is running, and
        that is exactly the moment the operator would have had to scroll to it.
        Outside the scroll area it cannot be scrolled away; outside the two
        columns it does not have to compete with the summaries for room, and it
        costs one line instead of a box with a title and its own margins.
        """
        strip = QWidget()
        layout = QVBoxLayout(strip)
        layout.setContentsMargins(0, 8, 0, 8)
        layout.setSpacing(6)

        head = QHBoxLayout()
        head.setSpacing(12)
        head.addWidget(self._phase_label, 1)
        head.addWidget(self._position, 0)

        layout.addLayout(head)
        layout.addWidget(self._progress)
        layout.addWidget(self._note)
        return strip

    def _wire(self) -> None:
        self._guided_start.clicked.connect(self._on_guided_start)
        self._guided_confirm.clicked.connect(
            lambda: self._submit(cmd.ConfirmProbeLimit())
        )
        self._guided_cancel.clicked.connect(self._on_cancel)
        self._manual_start.clicked.connect(self._on_manual_start)
        self._manual_open.clicked.connect(
            lambda: self._submit(cmd.RecordOpenLimit())
        )
        self._manual_close.clicked.connect(
            lambda: self._submit(cmd.RecordCloseLimit())
        )
        self._manual_cancel.clicked.connect(self._on_cancel)
        self._reload.clicked.connect(lambda: self._submit(cmd.LoadCalibration()))
        self._load.clicked.connect(self._on_load_clicked)
        self._save.clicked.connect(lambda: self._submit(cmd.SaveCalibration()))

    def _load_settings(self) -> None:
        allowed = (
            False if self._settings is None
            else self._settings.allow_factory_calibration
        )
        self._allow.setChecked(allowed)

    # ── slots from the worker ───────────────────────────────────────────────
    def set_calibration(self, info: CalibrationInfo | None) -> None:
        self._info = info
        self._rebuild_table(info)
        self._render_banner(info)
        self._refresh()

    def set_gate(self, state: GateState, reason: str) -> None:
        self._gate = state
        self._gate_reason = reason
        self._render_gate()
        self._refresh()

    def _render_gate(self) -> None:
        """Paint the gate line: ready in the measured green, the two closed
        states in the colour of how bad they are."""
        if self._gate is None:
            return
        if self._gate is GateState.READY:
            self._gate_label.setText(
                f'<span style="color:{theme.OK};font-weight:600">运动闸门：就绪</span>'
            )
            return
        colour = theme.ERROR if self._gate is GateState.BLOCKED else theme.WARN
        self._gate_label.setText(
            f'<span style="color:{colour};font-weight:600">运动闸门：'
            f'{"已阻断" if self._gate is GateState.BLOCKED else "待确认"}'
            f"</span><br><span style=\"color:{theme.TEXT_MUTED}\">{self._gate_reason}</span>"
        )

    def restyle(self) -> None:
        """Repaint this page's coloured markup in the current palette.

        Four labels carry a colour inside their text — the gate line, the phase,
        the provenance line and the live reading — and the provenance banner
        repaints itself.  The two probe summaries and the detail table are plain
        text with a role, so the stylesheet reaches them on its own.
        """
        self._render_gate()
        if self._info is not None:
            self._render_banner(self._info)
        if self._frame is not None:
            self.update_frame(self._frame)
        self._render_progress()
        self._refresh()

    def set_conn_state(self, state: str, detail: str = "") -> None:
        self._connected = state == "connected"
        self._refresh()

    def set_busy(self, busy: bool, text: str = "") -> None:
        self._busy = busy
        self._refresh()

    def update_frame(self, frame: TelemetryFrame) -> None:
        """The live reading, in millimetres *and* in radians.

        The radians are normally the number nobody needs, and they are here
        because the millimetres cannot be used to check the calibration they came
        from: the mm reading is computed with the very numbers under suspicion,
        while the angle is what the encoder reports.
        """
        self._frame = frame
        measured = UNKNOWN if frame.position_mm is None else f"{frame.position_mm:>7.2f} mm"
        angle = (
            UNKNOWN
            if frame.position_rad is None
            else f"{frame.position_rad:>8.4f} rad"
        )
        self._position.setText(f"实测位置 {measured} / {angle}")
        # ``frame.enabled`` is deliberately not kept: nothing on this page is
        # gated on the motor any more (see :meth:`_refresh`), and a flag that
        # only ever fed the widget states would be the copy that drifts the
        # moment an enable request fails.
        self._refresh()

    def set_progress(self, phase: str, progress: float, note: str = "") -> None:
        """The worker's ``calib_progress`` signal."""
        self._phase = phase
        self._note_text = note
        self._progress.setValue(int(max(0.0, min(1.0, progress)) * 100))
        self._render_progress()
        self._refresh()

    def _render_progress(self) -> None:
        """Draw the probe strip for whatever phase is in force.

        Split out from :meth:`set_progress` so that a theme change can repaint
        the strip without also re-setting a progress the worker owns.
        """
        label = PHASE_LABELS.get(self._phase, self._phase)
        colour = {
            GuidedPhase.DONE.value: theme.OK,
            TwoPointPhase.DONE.value: theme.OK,
            GuidedPhase.FAILED.value: theme.ERROR,
            TwoPointPhase.FAILED.value: theme.ERROR,
        }.get(self._phase, theme.TEXT)
        self._phase_label.setText(
            f'<span style="color:{colour};font-weight:600">{label}</span>'
        )
        # An empty bar is a claim that something is nought per cent done, and
        # before any probe has run there is nothing for that claim to be about.
        # It sat across the page as an unlabelled grey slab that a reviewer had
        # to ask about; with no probe running, the strip carries the live
        # reading and nothing else.
        self._progress.setVisible(self._phase != GuidedPhase.IDLE.value)
        self._note.setText(self._note_text)
        # An empty note is a blank line reserved at the bottom of a strip whose
        # whole purpose is to stay small, so it is only given room when there is
        # something in it to read.
        self._note.setVisible(bool(self._note_text))

    # ── rendering ───────────────────────────────────────────────────────────
    def _rebuild_table(self, info: CalibrationInfo | None) -> None:
        unknown = [("来源", "—")]
        self._summary = self._fill_rows(
            self._summary_target, self._summary,
            info.summary() if info is not None else unknown,
        )
        self._table = self._fill_rows(
            self._detail_target, self._table,
            info.as_table() if info is not None else unknown,
            mono=True,
        )

    def _fill_rows(self, target: QVBoxLayout, old: QFormLayout | None,
                   rows: list[tuple[str, str]], mono: bool = False) -> QFormLayout:
        """Put ``rows`` into ``target``, dropping whatever ``old`` held.

        The old rows are unparented rather than left to deleteLater(), so the
        previous calibration is gone from the widget tree the moment the new one
        arrives.  A table that still held the factory numbers underneath the
        user's would be the exact confusion this page exists to prevent — and
        the detail rows are searched for exactly that in the tests.
        """
        if old is not None:
            while old.count():
                item = old.takeAt(0)
                widget = item.widget()
                if widget is not None:
                    widget.setParent(None)
            target.removeItem(old)

        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignLeft)
        form.setHorizontalSpacing(16)
        form.setVerticalSpacing(6)
        for key, value in rows:
            value_label = QLabel(value)
            value_label.setTextFormat(Qt.PlainText)
            value_label.setWordWrap(True)
            if mono:
                value_label.setProperty("role", "mono")
            form.addRow(QLabel(key), value_label)
        target.addLayout(form)
        return form

    def _render_banner(self, info: CalibrationInfo | None) -> None:
        if info is None:
            self._banner.set(
                "error", "没有标定信息",
                "后端尚未报告标定状态；在它报到之前运动会被拒绝",
            )
            self._file_label.setText("—")
            return
        severity = PROVENANCE_SEVERITY.get(info.provenance, "warn")
        detail = "；".join(info.problems or info.warnings)
        self._banner.set(severity, info.headline(), detail)
        self._file_label.setText(
            f'<span style="color:{theme.TEXT_MUTED}">当前文件</span> '
            f'<span style="font-family:{theme.MONO_FAMILY}">'
            f"{calibration.friendly_path(info.path) if info.path else '（内存中的结果）'}"
            f"</span>"
        )

    def _refresh(self) -> None:
        """Offer only what could be accepted.

        The probes are enabled on the *connection*, not on the gate: a console
        whose gate is blocked is exactly the console that needs a calibration,
        and refusing to probe until one exists would be a deadlock.

        Not on the motor either, for the same reason in a smaller way.  A
        disabled button is a click Qt throws away without a word, so keying the
        start buttons on ``self._enabled`` meant the first press of 开始手动标定
        on a console that was not enabled yet did nothing at all — and a probe
        that starts by putting the axis in zero gravity is asked for by an
        operator who has usually just powered the bench up.  The worker answers
        the command instead: ``_start_probe`` refuses it with the reason, and
        that reason is what the operator needed.  ``_busy`` stays, because a
        control that is live while the SDK is inside a blocking call collects
        clicks that arrive out of order.
        """
        info = self._info
        ready = self._connected and not self._busy
        guided_running = guided_active(self._phase)
        manual_running = manual_active(self._phase)
        any_running = guided_running or manual_running

        self._guided_start.setEnabled(ready and not any_running)
        self._guided_cancel.setEnabled(guided_running)
        self._guided_confirm.setEnabled(guided_running and self._phase in CONFIRM_LABELS)
        self._guided_confirm.setText(
            CONFIRM_LABELS.get(self._phase, CONFIRM_DEFAULT)
        )
        self._manual_start.setEnabled(ready and not any_running)
        self._manual_cancel.setEnabled(manual_running)
        # One record button live at a time, and only in its own step: the label
        # on the button is the operator's whole answer to "which end is 0 mm",
        # so a button that could be pressed out of turn would let the answer be
        # given to the wrong question.
        self._manual_open.setEnabled(self._phase == TwoPointPhase.RECORD_OPEN.value)
        self._manual_close.setEnabled(self._phase == TwoPointPhase.RECORD_CLOSE.value)

        self._reload.setEnabled(not any_running)
        self._load.setEnabled(not any_running)
        # Still only a calibration whose numbers are self-consistent, and still
        # never the factory one.  The probe writes its own result out now, so
        # this button is normally a retry for a write that failed on something
        # outside the numbers (a read-only directory, a full disk, the SDK
        # refusing) — and a usable in-memory result is exactly what a retry
        # needs.  Two things are not for retrying.  Numbers the console has just
        # called unusable come back with no limits, so writing them would save a
        # file that only looks like a calibration.  The factory numbers are the
        # one case the backend's own guard cannot catch: they can arrive with no
        # path at all (the SDK's bundled fallback), and then the target is the
        # *user* file, which is how they would be laundered into this gripper's
        # own calibration on the next launch.
        self._save.setEnabled(
            not any_running
            and info is not None
            and info.usable
            and info.provenance in (
                calibration.PROVENANCE_MEMORY, calibration.PROVENANCE_USER
            )
        )
        # Shown only where it means something, but never unset behind the
        # operator's back: the acknowledgement is a standing preference, and
        # silently clearing it would make them tick it again on the next launch.
        self._allow.setVisible(bool(info is not None and info.is_factory))

    # ── actions ─────────────────────────────────────────────────────────────
    def _on_cancel(self) -> None:
        self._submit(cmd.CancelCalibration())

    def _on_guided_start(self) -> None:
        self._submit(cmd.StartGuidedCalibration())

    def _on_manual_start(self) -> None:
        self._submit(cmd.StartManualCalibration())

    def _on_allow_toggled(self, allowed: bool) -> None:
        if self._settings is not None:
            self._settings.allow_factory_calibration = allowed
        if self._allow_factory is not None:
            self._allow_factory(allowed)

    def _on_load_clicked(self) -> None:
        path, _filter = QFileDialog.getOpenFileName(
            self, "选择标定文件", "", "标定文件 (*.json);;所有文件 (*)"
        )
        if path:
            self._load_file(path)

    def _load_file(self, path: str) -> None:
        self._submit(cmd.LoadCalibration(path=path))

    @property
    def phase(self) -> str:
        return self._phase

    @property
    def allows_factory(self) -> bool:
        return self._allow.isChecked()
