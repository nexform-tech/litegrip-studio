"""The page the console exists for: move the gripper, and watch it move.

Every control here turns an intent into one command and nothing else.  What the
command then does is the worker's business, which is why this page can be driven
in a test by handing it a recorder instead of a thread.

Two pieces of bookkeeping live here because they are about *interaction* rather
than control, and both are the kind of thing that is invisible until it is wrong:

*The echo guard.*  The worker publishes at 50 Hz and accepts commands between
any two of those publishes, so the frame that arrives immediately after a drag
often describes the state *before* it — no command, nothing moving.  Acting on
that frame would pull the handle back out of the operator's hand, which is
precisely the thing the slider is careful not to do.  So the page ignores a
frame that says "no command" for a few frames after issuing one, and hands the
authority to the worker as soon as it sees its command echoed back.

*The owner.*  The slider is told who is driving: while the command it is
carrying out is the slider's, the handle stays at the target; while it is a
button's, the handle follows the jaws.  Ownership is released once the axis has
arrived and stopped, so a gripper nudged by hand updates the bar again.
"""

from __future__ import annotations

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QCheckBox,
    QDoubleSpinBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from .. import constants
from ..core import commands as cmd
from ..core.worker import GateState
from ..settings import Settings
from ..telemetry import TelemetryFrame
from . import theme
from .slider import OWNER_SLIDER, StrokeSlider
from .widgets import PositionReadout

#: How many published frames to distrust after issuing a command.  Ten frames is
#: 200 ms at 50 Hz: comfortably longer than one publish interval, and short
#: enough that a refused command does not leave the slider stuck for long.
ECHO_FRAMES = 10


class ControlPage(QWidget):
    """The slider, the action buttons, and the speed and force controls."""

    def __init__(self, submit, settings: Settings | None = None,
                 parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._submit = submit
        self._settings = settings
        self._echo = ECHO_FRAMES
        self._gate = GateState.BLOCKED
        self._frame: TelemetryFrame | None = None
        self._note = ""

        self.slider = StrokeSlider()
        self.readout = PositionReadout()
        self._state_label = QLabel()
        self._state_label.setTextFormat(Qt.RichText)

        self._open = QPushButton("全部张开")
        self._close = QPushButton("全部闭合")
        self._grasp = QPushButton()
        self._back_off = QPushButton("放开")
        self._stop = QPushButton("停止（保持位置）")
        self._release = QPushButton("零重力（可手掰）")

        self._speed = QSlider(Qt.Horizontal)
        self._speed_value = QLabel()
        self._speed_value.setMinimumWidth(76)
        self._force = QDoubleSpinBox()
        self._live = QCheckBox("拖动时实时跟随")

        self._build()
        self._wire()
        self._load_settings()
        # Fail closed, and say why.  ``_gate`` starts BLOCKED, and until the
        # worker's first gate_state arrives that intent has to reach the widgets
        # too — a slider that accepts a drag before the console knows whether
        # there is a calibration is a slider that can move an uncalibrated motor.
        self.set_gate(GateState.BLOCKED, "尚未收到标定信息")
        theme.subscribe(self.restyle)

    # ── construction ────────────────────────────────────────────────────────
    def _build(self) -> None:
        self._stop.setToolTip("停在当前角度并保持，电机保持使能")
        self._release.setToolTip("零刚度零力矩，可以用手掰动，电机保持使能")
        self._grasp.setProperty("accent", True)
        self._back_off.setProperty("accent", True)
        self._back_off.setToolTip(
            f"从夹爪现在的位置再张开 {constants.RELEASE_OPEN_MM:.1f} mm，用来松开夹住的物体。"
            "不是回到夹取前的目标位置，也不是量程顶端"
        )
        # The two end-to-end moves sit either side of the bar, on the ends they
        # command, so "闭合 lives at the left and 张开 at the right" is read off
        # the layout rather than learned.  Their tooltips carry the millimetres,
        # because "全部张开" is only meaningful against a travel — and the travel
        # is the calibration's, which arrives after this runs.
        for button in (self._open, self._close):
            button.setMinimumWidth(96)
            button.setMinimumHeight(32)
        # Every button on the page is the same height, so the three columns under
        # the bar read as one row of controls rather than as three stacks that
        # happen to sit side by side.
        for button in (self._stop, self._release, self._grasp, self._back_off):
            button.setMinimumHeight(32)

        actions = QVBoxLayout()
        actions.setSpacing(8)
        actions.addWidget(self._stop)
        actions.addWidget(self._release)

        action_box = QGroupBox("动作")
        action_layout = QVBoxLayout(action_box)
        action_layout.setSpacing(8)
        action_layout.addLayout(actions)
        action_layout.addWidget(self._state_label)
        action_layout.addStretch(1)

        self._speed.setMinimum(int(constants.SPEED_MIN_MM_S))
        self._speed.setMaximum(int(constants.SPEED_MAX_MM_S))
        self._speed.setSingleStep(1)
        self._speed.setToolTip(
            f"移动速度 {constants.SPEED_MIN_MM_S:.0f}–{constants.SPEED_MAX_MM_S:.0f} mm/s"
        )
        speed_ends = QHBoxLayout()
        speed_ends.setContentsMargins(0, 0, 0, 0)
        low = QLabel(f"{constants.SPEED_MIN_MM_S:.0f}")
        low.setProperty("role", "caption")
        high = QLabel(f"{constants.SPEED_MAX_MM_S:.0f} mm/s")
        high.setProperty("role", "caption")
        speed_ends.addWidget(low, 0)
        speed_ends.addStretch(1)
        speed_ends.addWidget(high, 0)

        speed_box = QGroupBox("速度")
        speed_layout = QVBoxLayout(speed_box)
        speed_layout.setSpacing(8)
        speed_layout.addWidget(self._speed)
        speed_layout.addLayout(speed_ends)
        speed_layout.addWidget(self._speed_value)
        speed_layout.addStretch(1)

        self._force.setRange(0.0, constants.FORCE_MAX_N)
        self._force.setSingleStep(0.5)
        self._force.setDecimals(1)
        self._force.setSuffix(" N")
        self._force.setToolTip(
            f"夹持力上限 {constants.FORCE_MAX_N:.0f} N（机械额定）；"
            f"超过 {constants.FORCE_SOFT_WARN_N:.0f} N 会持续告警"
        )
        force_box = QGroupBox("夹持力")
        force_layout = QVBoxLayout(force_box)
        force_layout.setSpacing(8)
        force_layout.addWidget(self._force)
        force_layout.addWidget(self._grasp)
        # 放开 goes directly under 夹取 because the two are one gesture: the
        # grasp is the only thing that can leave the jaws holding something, and
        # this is the only button that undoes it.  Apart, in the action column,
        # they would read as two unrelated moves — and the grasp's own button is
        # drawn here in the first place, under the force it carries.
        force_layout.addWidget(self._back_off)
        force_layout.addStretch(1)

        lower = QHBoxLayout()
        lower.setSpacing(theme.GAP)
        lower.addWidget(action_box, 2)
        lower.addWidget(speed_box, 2)
        lower.addWidget(force_box, 1)

        self._live.setToolTip(
            "拖动过程中就下发目标，而不是松手才下发。"
            "每条命令都会被限速，所以拖出多远都不会瞬移；"
            "命令队列会把拖动产生的一串目标合并成最后一条。"
        )

        position = QHBoxLayout()
        position.setSpacing(14)
        position.addWidget(self._close)
        position.addWidget(self.slider, 1)
        position.addWidget(self.readout, 0)
        position.addWidget(self._open)
        position_box = QGroupBox("位置（拖动即控制，松手执行）")
        position_layout = QVBoxLayout(position_box)
        position_layout.setSpacing(8)
        position_layout.addLayout(position)
        position_layout.addWidget(self._live)

        layout = QVBoxLayout(self)
        layout.setSpacing(theme.GAP)
        layout.setContentsMargins(*theme.PAGE_MARGINS)
        layout.addWidget(position_box)
        layout.addLayout(lower)
        layout.addStretch(1)

    def _wire(self) -> None:
        self.slider.dragCommitted.connect(self._on_drag_committed)
        self.slider.dragMoved.connect(self._on_drag_moved)
        self._open.clicked.connect(lambda: self._issue_button(cmd.Open()))
        self._close.clicked.connect(lambda: self._issue_button(cmd.Close()))
        self._grasp.clicked.connect(
            lambda: self._issue_button(cmd.Grasp(force_n=self._force.value()))
        )
        self._back_off.clicked.connect(lambda: self._issue_button(cmd.BackOff()))
        self._stop.clicked.connect(self._on_stop)
        self._release.clicked.connect(self._on_release)
        self._speed.valueChanged.connect(self._on_speed_changed)
        self._speed.sliderReleased.connect(self._persist)
        self._force.valueChanged.connect(self._on_force_changed)
        self._force.editingFinished.connect(self._persist)
        self._live.toggled.connect(self._on_live_follow_toggled)

    def _load_settings(self) -> None:
        if self._settings is None:
            self._speed.setValue(int(constants.SPEED_DEFAULT_MM_S))
            self._force.setValue(constants.FORCE_DEFAULT_N)
        else:
            self._speed.setValue(int(self._settings.speed_mm_s))
            self._force.setValue(self._settings.force_n)
            self._live.setChecked(self._settings.live_follow)
        self.slider.set_live_follow(self._live.isChecked())
        self._on_speed_changed(self._speed.value())
        self._on_force_changed(self._force.value())

    # ── slots from the worker ───────────────────────────────────────────────
    def set_calibration(self, info) -> None:
        """Tell the page which travel the bar spans.

        ``info`` is the worker's :class:`~litegrip_studio.calibration.CalibrationInfo`
        or None.  The bar takes its range from the *limits*, not from the gate:
        a console with a blocked gate still knows how wide the gripper is, and
        showing the travel while refusing to move is more useful than showing
        an empty bar.

        The two end-to-end buttons take their tooltips from the same place, so
        the numbers a command goes to are readable before it is pressed.
        """
        limits = info.limits if info is not None else None
        self.slider.set_limits(limits)
        self._describe_endpoints(limits)

    def _describe_endpoints(self, limits) -> None:
        """Say in millimetres where the two one-click moves go.

        Without a travel to quote there is nothing to name, and the honest
        tooltip says that rather than quoting a number the console is not
        working to — the same reason the slider spans nothing until it is told
        what the travel is.
        """
        if limits is None:
            self._open.setToolTip("以设定速度张开到量程顶端（尚未读到标定）")
            self._close.setToolTip("以设定速度闭合到行程 0 点（尚未读到标定）")
            return
        self._open.setToolTip(
            f"以设定速度张开到量程顶端 {limits.max_stroke_mm:.1f} mm"
        )
        self._close.setToolTip("以设定速度闭合到行程 0.0 mm（标定的闭合位置）")

    def set_gate(self, state: GateState, reason: str) -> None:
        """Close the position control when the gate is not open.

        The slider is disabled rather than hidden: the reason it is unavailable
        is on the tooltip and on the calibration page, and a control that
        vanished would leave the operator wondering whether it was ever there.
        """
        self._gate = state
        ready = state is GateState.READY
        self.slider.set_blocked("" if ready else reason)
        for button in (self._open, self._close, self._grasp, self._back_off):
            button.setEnabled(ready)
        # 停止 and 零重力 stay available through the gate.  They are the two
        # controls whose whole purpose is to stop rather than to go anywhere,
        # and refusing them on an uncalibrated gripper would be refusing to let
        # go — which is the wrong way to fail on a machine holding something.
        # 放开 is not one of them: it is a millimetre command, so it needs the
        # travel the gate is holding back, and the two above are what an
        # operator reaches for when there is no travel to command.

    def update_frame(self, frame: TelemetryFrame) -> None:
        self._frame = frame
        self._note = ""
        self._echo += 1
        # Ownership is settled *before* the measurement is handed to the slider,
        # so a frame that releases the handle moves it in the same frame rather
        # than one publish later.
        self._track(frame)
        self.slider.set_actual(frame.position_mm)
        self.readout.update_position(
            frame.position_mm, frame.cmd_mm, grasping=frame.grasped
        )
        self._render_state()

    def _render_state(self) -> None:
        """Say what the axis is doing, in the colour that matches the state."""
        if self._note:
            self._state_label.setText(
                f'<span style="color:{theme.WARN}">{self._note}</span>'
            )
            return
        frame = self._frame
        if frame is None:
            self._state_label.clear()
            return
        self._state_label.setText(
            f'<span style="color:{theme.TEXT_MUTED}">状态 </span>'
            f'<span style="color:{self._state_colour(frame)};font-weight:600">'
            f"{frame.motion_state}</span>"
        )

    def restyle(self) -> None:
        """Repaint this page's own markup in the current palette.

        The widgets are styled by the application stylesheet; what has to be
        re-rendered here is the handful of strings that carry a colour inside
        them, plus the readout, which holds the last frame's numbers.
        """
        self.readout.restyle()
        self._render_state()
        self._render_speed(self._speed.value())
        self._render_force(self._force.value())

    def _track(self, frame: TelemetryFrame) -> None:
        """Decide who owns the handle on this frame — see the module docstring."""
        if frame.cmd_mm is not None:
            self._echo = ECHO_FRAMES
            self.slider.set_target(frame.cmd_mm)
        elif self._echo >= ECHO_FRAMES:
            self.slider.set_target(None)

        if self._echo < ECHO_FRAMES:
            return
        # An absent error figure means "unknown", not "no error".  Reading a
        # missing err_mm as zero would count every frame that did not carry one
        # as an arrival, and hand the handle back to the measurement mid-move —
        # which is the failure the owner exists to prevent.
        holding = (
            frame.cmd_mm is not None
            and frame.err_mm is not None
            and abs(frame.err_mm) <= constants.TOL_MM
        )
        if not frame.moving and (frame.cmd_mm is None or holding):
            self.slider.set_owner(None)

    def _state_colour(self, frame: TelemetryFrame) -> str:
        if frame.motion_state == "ESTOP":
            return theme.ERROR
        if not frame.enabled:
            return theme.TEXT_MUTED
        return theme.ACTUAL if frame.moving else theme.TEXT

    def set_status_note(self, text: str) -> None:
        """A line for something the page should say — a UV cap, typically."""
        self._note = text
        self._render_state()

    # ── issuing commands ────────────────────────────────────────────────────
    def _on_drag_committed(self, mm: float) -> None:
        self._issue(cmd.MoveToMm(target_mm=mm, source="slider"), target=mm,
                    owner=OWNER_SLIDER)

    def _on_drag_moved(self, mm: float) -> None:
        """Live following: every update is a command, and the queue coalesces
        them into the last one, so what reaches the motor is still a single
        rate-limited move."""
        self._issue(cmd.MoveToMm(target_mm=mm, source="slider"), target=mm,
                    owner=OWNER_SLIDER)

    def _issue_button(self, command) -> None:
        self._issue(command, target=None, owner=command.source)

    def _on_stop(self) -> None:
        self._issue(cmd.Stop(), target=None, owner=None)

    def _on_release(self) -> None:
        self._issue(cmd.Release(), target=None, owner=None)

    def _issue(self, command, *, target: float | None, owner: str | None) -> None:
        self._submit(command)
        self._echo = 0
        self.slider.set_target(target)
        self.slider.set_owner(owner)

    def _on_live_follow_toggled(self, enabled: bool) -> None:
        """Written immediately rather than on close: it is one deliberate click,
        not a drag, and it changes what a drag means — so it is worth surviving
        a crash."""
        self.slider.set_live_follow(enabled)
        if self._settings is not None:
            self._settings.live_follow = enabled

    # ── speed and force ─────────────────────────────────────────────────────
    def _on_speed_changed(self, value: int) -> None:
        self._submit(cmd.SetSpeed(speed_mm_s=float(value)))
        self._render_speed(value)

    def _render_speed(self, value: int) -> None:
        """The speed in force, named.

        The card is titled 速度, but so is the row of range ends above this —
        "5" and "150 mm/s" — and an unlabelled "50 mm/s" sitting under them is
        the third millimetre-per-second figure in the same box with nothing to
        say which of the three the machine is actually running at.  The
        position panel already names its readings the same way (实际位置,
        目标, 误差); this is that convention applied where it was missing.
        """
        self._speed_value.setText(
            f'<span style="color:{theme.TEXT_MUTED}">当前速度：</span>'
            f'<span style="color:{theme.TEXT};font-family:{theme.MONO_FAMILY};'
            f'font-size:{theme.FONT_PX + 1}px;font-weight:600">'
            # No width padding: the caption ahead of it fixes where the number
            # starts, and the padded form now reads as a gap after the colon.
            f"{value} mm/s</span>"
        )

    def _on_force_changed(self, value: float) -> None:
        self._submit(cmd.SetForce(force_n=float(value)))
        self._render_force(value)

    def _render_force(self, value: float) -> None:
        """Colour the force field by how close it is to the mechanism's rating.

        Separated from :meth:`_on_force_changed` because the two have different
        callers: a value the operator set is a command, and a repaint after a
        theme change is not.  Sending a ``SetForce`` from the repaint would mean
        switching the console to light mode re-commanded the gripper.
        """
        self._grasp.setText(f"夹取 {value:.1f} N")
        if value >= constants.FORCE_MAX_N:
            self._force.setStyleSheet(f"color: {theme.ERROR}; font-weight: 600;")
        elif value > constants.FORCE_SOFT_WARN_N:
            self._force.setStyleSheet(f"color: {theme.WARN}; font-weight: 600;")
        else:
            self._force.setStyleSheet("")

    @property
    def speed_mm_s(self) -> float:
        return float(self._speed.value())

    @property
    def force_n(self) -> float:
        return float(self._force.value())

    @property
    def live_follow(self) -> bool:
        return self._live.isChecked()

    def _persist(self) -> None:
        """Write the speed once the operator has stopped dragging.

        Not on every ``valueChanged``: a drag is a hundred values, and a hundred
        fsyncs of the preferences file is a lot of disk for one number.
        """
        if self._settings is None:
            return
        self._settings.speed_mm_s = self.speed_mm_s
        self._settings.force_n = self.force_n

    def persist(self) -> None:
        """Called by the window on close; the buttons persist as they are used."""
        self._persist()
