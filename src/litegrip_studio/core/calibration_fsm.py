"""The two calibration probes, rewritten as pure state machines.

Both replace an SDK method that cannot be used from a GUI, and in both cases
the reason is the same shape: the SDK's version owns its own loop, and there is
no way into it.

``calibrate_guided`` (gripper.py:520)
    Waits for Enter (``sys.stdin.isatty()`` and ``select``, gripper.py:582), so
    with no terminal it silently degrades to stall-detection only — in a GUI
    there is no terminal at all.  Each step calls ``control_mit_stream`` for a
    fixed 0.3 s, so nothing can interrupt it, and its output goes to ``print``.

``calibrate_manual`` (gripper.py:354)
    Documents "press Ctrl+C to stop early", which can never happen here: Python
    delivers signals to the main thread only, so the worker thread would wait
    out the full duration whatever the operator did.

Pure by construction — no backend, no Qt, no clock.  :meth:`tick` is handed the
measured angle and the elapsed time and returns the frame to send, so the whole
probe is a unit test that runs in microseconds and can be driven through a stall
without a gripper.

Two things both probes are careful about:

**Nothing is ever sent with the position gain at zero and the reference left
where it was.**  A motor holds the last MIT frame it was given, so "send
nothing" during a probe means "keep pressing with the last reference" — which at
a hard stop is exactly the command that must not persist.  Every terminal path
returns a zero-gain frame, and the reference is anchored to the measurement
rather than integrated, so a stalled axis cannot wind up.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

from .. import constants
from ..units import derive_scale, mm_to_rad_per_s, torque_from_force

#: A reading moving further than this in one tick is not a reading.
PROBE_MAX_JUMP_RAD_PER_TICK = constants.PROBE_MAX_JUMP_RAD_PER_TICK

#: The same bound for a probe driven by a hand rather than by the motor.
#: ``RAD_TO_MM_MIN`` is the *most* radians per millimetre any plausible gripper
#: has, so this is the most permissive reading of "1 m/s" — the guard should
#: never be the thing that rejects a real hand move.
#:
#: ``abs`` because this is a magnitude and the conversion is signed; the
#: direction it is asked for is therefore arbitrary, and a negative bound here
#: would reject every sample in silence.
MANUAL_MAX_JUMP_RAD_PER_TICK = abs(
    mm_to_rad_per_s(
        constants.MANUAL_MAX_HAND_SPEED_MM_S, constants.RAD_TO_MM_MIN, direction=-1.0
    )
) * constants.CTRL_DT


@dataclass(frozen=True)
class ProbeOut:
    """What a calibration FSM wants sent this tick.

    ``q_rad is None`` means send nothing at all, which is only correct in
    :data:`NO_FRAME`'s case: an idle probe that has never taken the axis.
    """

    q_rad: float | None
    kp: float = 0.0
    kd: float = 0.0
    tau_nm: float = 0.0
    note: str = ""


NO_FRAME = ProbeOut(None)
#: Zero stiffness: the axis is limp and can be moved by hand.
FREE_FRAME = ProbeOut(0.0, 0.0, 0.0, 0.0)


@dataclass(frozen=True)
class CalibResult:
    """A completed probe, in the terms the calibration file uses.

    ``zero_rad`` is the closed position (0 mm) and ``open_rad`` the open one
    (full stroke).  Which of the two is numerically the larger depends on the
    mounting and is not this class's business — see
    :attr:`~litegrip_studio.units.Limits.direction` — so the travel below is a
    magnitude and the ordering is preserved rather than normalised.  Angles are
    rounded to 6 decimal places and the conversion factor to 2, the same shape
    the SDK's own files have, so a file this console writes can be read by
    anything that reads theirs.  Only the shape: the factor here is derived from
    the travel rather than the nominal stroke the SDK would divide by, so the two
    are meant to differ — see :func:`summarise`.
    """

    zero_rad: float
    open_rad: float
    rad_to_mm: float
    max_stroke_mm: float
    notes: tuple[str, ...] = ()

    @property
    def travel_rad(self) -> float:
        return abs(self.zero_rad - self.open_rad)

    @property
    def stroke_mm(self) -> float:
        """The span the recorded extremes imply: the travel plus the inset.

        Wider than :attr:`max_stroke_mm` by construction, and reported rather
        than commanded — the probe reaches the open limit by pressing into it,
        and the commanded range stops a millimetre short of where it stopped.
        """
        return self.travel_rad * self.rad_to_mm

    def as_raw(self) -> dict[str, float]:
        """The calibration fields, in the file's own key names."""
        return {
            "zero_position_rad": self.zero_rad,
            "max_position_rad": self.open_rad,
            "travel_range_rad": self.travel_rad,
            "rad_to_mm": self.rad_to_mm,
        }


def summarise(
    zero_rad: float,
    open_rad: float,
    max_stroke_mm: float,
    notes: tuple[str, ...],
    *,
    degenerate_message: str | None = None,
) -> CalibResult | str:
    """Round a probe's readings into a result, or return why it is unusable.

    Both ``travel_range_rad`` and ``rad_to_mm`` are derived from the *rounded*
    angles, not the raw ones, so the file is internally consistent: the SDK's own
    calibration files satisfy ``travel = zero - open`` exactly, and one that did
    not would make the validation in :mod:`litegrip_studio.calibration` depend
    on which field it happened to read.

    ``rad_to_mm`` comes from :func:`~litegrip_studio.units.derive_scale`, not
    from ``max_stroke_mm / travel``: the probe records the two angles and the
    operator's measured travel settles the millimetres, so the recorded span is
    one millimetre wider than the travel and the commanded range stops just
    inside the open extreme it pressed against.  The SDK computes the other
    number, and its own three calibrations do not even agree with each other on
    which nominal to divide by.

    A degenerate range is reported by the probe that saw it, because the two
    probes fail differently: a guided probe found the same place twice and needs
    the angles, while a manual probe found nothing and needs the procedure
    repeated — the SDK's own wording for that, which is what the operator will
    find in the SDK's documentation.
    """
    zero = round(zero_rad, 6)
    opened = round(open_rad, 6)
    travel = abs(zero - opened)
    if travel <= 0:
        if degenerate_message is not None:
            return degenerate_message
        return (
            f"行程异常: 两个极限落在同一个位置 ({zero:.6f} rad)。"
            "请确认两个极限都真正走到了位置"
        )
    return CalibResult(
        zero_rad=zero,
        open_rad=opened,
        rad_to_mm=round(derive_scale(travel, max_stroke_mm), 2),
        max_stroke_mm=max_stroke_mm,
        notes=notes,
    )


# ═══════════════════════════════════════════════════════════════════════════
# Guided probe
# ═══════════════════════════════════════════════════════════════════════════
class GuidedPhase(str, Enum):
    IDLE = "IDLE"
    OPEN_PROBE = "OPEN_PROBE"
    OPEN_BACKOFF = "OPEN_BACKOFF"
    CLOSE_PROBE = "CLOSE_PROBE"
    DONE = "DONE"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"


#: Phases after which the probe holds no authority over the axis.  They still
#: return a zero-gain frame, because the motor is executing the last frame it was
#: given and during a probe that frame is a press against a hard stop.
TERMINAL_PHASES = frozenset(
    {GuidedPhase.DONE, GuidedPhase.FAILED, GuidedPhase.CANCELLED}
)


class GuidedCalibFSM:
    """Steps the jaws into each hard stop and records where they stopped.

    The axis is driven one bounded step at a time toward the open limit, then
    backed off, then toward the closed limit.  A limit is taken when the operator
    confirms it — the GUI's replacement for the SDK's Enter key — or
    automatically when the jaws have stopped moving for
    :data:`~litegrip_studio.constants.GUIDED_STALL_CYCLES` steps.

    The reference never integrates.  Each step re-anchors it to
    ``measured + one step``, so the MIT law's position error can never exceed one
    step and the press at a hard stop is bounded by ``kp × step`` — which is why
    the step is derived from the mechanical rating rather than taken from the SDK
    (:data:`~litegrip_studio.constants.GUIDED_MAX_FORCE_N`).  The SDK's own loop
    advances its target from the previous target, so a jammed axis accumulates
    0.08 rad of error per iteration for up to 40 iterations.

    Which way the jaws open has to be given, not assumed — see
    ``reversed_mount``: this is the one probe that produces a calibration out of
    nothing, so it has no file to read the mounting from, and a probe that steps
    the wrong way records the closed stop as the open one.  The result would
    still validate, still be saved, and drive the gripper inverted.
    """

    def __init__(
        self,
        max_stroke_mm: float = constants.DEFAULT_TRAVEL_MM,
        *,
        reversed_mount: bool = False,
        kp: float = constants.GUIDED_KP,
        kd: float = constants.KD_DEFAULT,
        step_rad: float = constants.GUIDED_STEP_RAD,
        stall_delta: float = constants.GUIDED_STALL_DELTA,
        stall_cycles: int = constants.GUIDED_STALL_CYCLES,
        max_iter: int = constants.GUIDED_MAX_ITER,
        step_interval_s: float = constants.GUIDED_STEP_INTERVAL_S,
        backoff_rad: float = constants.GUIDED_BACKOFF_RAD,
        max_force_n: float = constants.GUIDED_MAX_FORCE_N,
        max_jump_rad: float = constants.PROBE_MAX_JUMP_RAD_PER_TICK,
    ) -> None:
        self.max_stroke_mm = float(max_stroke_mm)
        # The mounting, in the terms the rest of the console names it
        # (``Limits.reversed_mount``), and the one sign every step below is taken
        # in.  The conversion happens here and nowhere else: further down, a
        # second opinion about which way is open is how a probe ends up driving
        # the jaws into the stop it is not looking for.
        self.reversed_mount = bool(reversed_mount)
        self.direction = 1.0 if self.reversed_mount else -1.0
        self.kp = float(kp)
        self.kd = float(kd)
        # The one derivation that matters: at a stall the torque is kp × step, so
        # the step is what decides how hard the probe can press.
        self.step_rad = min(float(step_rad), torque_from_force(max_force_n) / self.kp)
        self.stall_delta = float(stall_delta)
        self.stall_cycles = int(stall_cycles)
        self.max_iter = int(max_iter)
        self.step_interval_s = float(step_interval_s)
        self.backoff_rad = float(backoff_rad)
        # Counted in steps, not measured against a target: see _backoff_tick.
        self.backoff_steps = max(1, int(math.ceil(self.backoff_rad / self.step_rad)))
        self.max_jump_rad = float(max_jump_rad)

        self.phase = GuidedPhase.IDLE
        self.open_rad: float | None = None
        self.close_rad: float | None = None
        self.result: CalibResult | None = None
        self.note = ""
        self.iterations = 0
        self.stalls = 0

        #: Where the probe began, for the GUI to show alongside the limits.
        self.entry_rad: float | None = None
        self._measured = 0.0
        self._ref_rad = 0.0
        self._step_mark_rad = 0.0
        self._since_step = 0.0
        self._confirmed = False
        self._prev_rad: float | None = None
        self._notes: list[str] = []

    # ── commands ────────────────────────────────────────────────────────────
    def start(self, measured_rad: float) -> bool:
        """Begin the open probe from where the jaws are.  False if it cannot start."""
        if not math.isfinite(measured_rad):
            self.phase = GuidedPhase.FAILED
            self.note = f"位置读数无效 ({measured_rad!r})，无法开始探测"
            return False
        self.phase = GuidedPhase.OPEN_PROBE
        self.result = None
        self.open_rad = None
        self.close_rad = None
        self.entry_rad = float(measured_rad)
        self._measured = float(measured_rad)
        # Straight in with one step rather than idling at the start position for
        # a whole interval: waiting would burn the first window with a command
        # that cannot move anything, and the first window is what the stall
        # detector would then read as "not moving".
        self._ref_rad = float(measured_rad) + self._sign * self.step_rad
        self._step_mark_rad = float(measured_rad)
        self.iterations = 0
        self.stalls = 0
        self._since_step = 0.0
        self._confirmed = False
        self._prev_rad = None
        self._notes = []
        # The direction is said out loud while the first stop is being
        # approached, because that is the moment the operator can see whether the
        # jaws are opening or closing and cancel a probe that has it backwards.
        self.note = f"正在探测张开极限{self._mounting}"
        return True

    def confirm(self) -> None:
        """The operator says the jaws are at the limit.

        Consumed by the next :meth:`tick`, which is where the measurement is
        available — the button knows the operator pressed it, not where the jaws
        were when they did.
        """
        if self.phase in (GuidedPhase.OPEN_PROBE, GuidedPhase.CLOSE_PROBE):
            self._confirmed = True

    def cancel(self) -> None:
        self.phase = GuidedPhase.CANCELLED
        self.note = "已取消标定"

    def fail(self, reason: str) -> None:
        """Stop the probe from outside, with a reason the operator can act on.

        The state machine steers by the angle it reads, so it cannot tell "the
        jaws are against a stop" from "no frame is reaching the motor" or "the
        feedback has gone silent" — in both of those the reading simply stops
        changing, which is what a stall looks like.  Only the caller knows, and
        only the caller can say so.  Ignored once terminal: whatever ended the
        probe first is the better explanation.
        """
        if not self.is_terminal:
            self._fail(reason)

    @property
    def notes(self) -> tuple[str, ...]:
        """How each limit was decided, in order — worth logging on failure too.

        On a *successful* probe these are the record of the calibration that was
        written.  On a failed one they are the only account of where the probe
        got to, which is exactly when someone needs to read them.
        """
        return tuple(self._notes)

    # ── the tick ────────────────────────────────────────────────────────────
    def tick(self, measured_rad: float, dt: float) -> ProbeOut:
        if self.phase is GuidedPhase.IDLE:
            return NO_FRAME
        if self.phase in TERMINAL_PHASES:
            # Zero gains at wherever the jaws are: the motor would otherwise go
            # on executing the last frame, which during a probe is a press.
            return ProbeOut(self._measured, 0.0, 0.0, 0.0, self.note)

        if not math.isfinite(measured_rad):
            self._fail(f"位置读数无效 ({measured_rad!r})，已停止探测")
            return ProbeOut(self._measured, 0.0, 0.0, 0.0, self.note)

        # A reading that could not physically have happened is refused before it
        # is adopted: the probe steers by this number, and the file it produces
        # is derived from it.
        if (
            self._prev_rad is not None
            and abs(measured_rad - self._prev_rad) > self.max_jump_rad
        ):
            self._fail(
                f"位置读数从 {self._prev_rad:.4f} rad 跳到 {measured_rad:.4f} rad，"
                f"超过 {self.max_jump_rad:.2f} rad/tick —— 读数不可信，已停止探测"
            )
            return ProbeOut(self._measured, 0.0, 0.0, 0.0, self.note)

        self._prev_rad = float(measured_rad)
        self._measured = float(measured_rad)
        self._since_step += dt

        if self.phase is GuidedPhase.OPEN_BACKOFF:
            return self._backoff_tick()

        if self._confirmed:
            self._confirmed = False
            self._finish_limit(self._measured, "操作员确认")
            return self._phase_frame()

        if self._since_step >= self.step_interval_s:
            self._advance_step()
            if self.phase in TERMINAL_PHASES:
                return self._phase_frame()

        return ProbeOut(self._ref_rad, self.kp, self.kd, 0.0, self.note)

    # ── state, for the worker and the GUI ───────────────────────────────────
    @property
    def is_terminal(self) -> bool:
        return self.phase in TERMINAL_PHASES

    @property
    def is_active(self) -> bool:
        """Started and still running — the worker holds the axis, not the FSM."""
        return self.phase is not GuidedPhase.IDLE and not self.is_terminal

    @property
    def target_rad(self) -> float | None:
        """Where the probe is steering, or ``None`` before it has started."""
        if self.phase is GuidedPhase.IDLE:
            return None
        if self.is_terminal:
            return self._measured
        return self._ref_rad

    # ── internals ───────────────────────────────────────────────────────────
    @property
    def _sign(self) -> float:
        """+1 toward closed, -1 toward open — on this mounting.

        The open probe steps the way the jaws open and the close probe the other
        way, so the two are each other's negation whichever way round the
        encoder runs; :attr:`direction` is the one fact they are derived from.
        """
        return self.direction if self.phase is GuidedPhase.OPEN_PROBE else -self.direction

    @property
    def _mounting(self) -> str:
        """The mounting direction, in words, for a status line."""
        return "（反向装配：张开时角度变大）" if self.reversed_mount else ""

    @property
    def progress(self) -> float:
        """A rough completion fraction, for a progress bar."""
        if self.phase is GuidedPhase.OPEN_PROBE:
            return 0.45 * min(self.iterations / max(self.max_iter, 1), 1.0)
        if self.phase is GuidedPhase.OPEN_BACKOFF:
            return 0.5
        if self.phase is GuidedPhase.CLOSE_PROBE:
            return 0.5 + 0.45 * min(self.iterations / max(self.max_iter, 1), 1.0)
        return 1.0 if self.phase is GuidedPhase.DONE else 0.0

    def _advance_step(self) -> None:
        """One probe step: measure what the last one achieved, then re-anchor."""
        moved = abs(self._measured - self._step_mark_rad)
        self._step_mark_rad = self._measured
        self._since_step = 0.0
        self.iterations += 1

        if moved < self.stall_delta:
            self.stalls += 1
        else:
            self.stalls = 0

        if self.stalls >= self.stall_cycles:
            self._finish_limit(
                self._measured, f"{self.stalls} 步未移动，判定已到极限"
            )
            return
        if self.iterations >= self.max_iter:
            self._finish_limit(
                self._measured, f"达到最大步数 {self.max_iter}，按当前位置记录"
            )
            return

        # Anchored to the measurement, never to the previous reference: a jammed
        # axis then holds the error at one step instead of accumulating it.
        self._ref_rad = self._measured + self._sign * self.step_rad

    def _backoff_tick(self) -> ProbeOut:
        """Move away from the open stop, one bounded step at a time.

        The SDK backs off by commanding ``open_rad - 0.15`` (gripper.py:610),
        which is *further into* the open stop — the sign is wrong on its own
        mounting and it presses at kp=80 for half a second.  Backing off means
        moving toward closed, which is :attr:`_sign` during this phase.

        The length of the back-off is a step *count*
        (:attr:`backoff_steps`), not a distance to be measured.  Everywhere else
        in this class the reference is anchored to the measurement so that the
        position error can never exceed one step — and that anchoring is exactly
        what makes a distance-based back-off unable to finish: on an axis that
        will not move, every step re-anchors to the same measurement and commands
        the same place, so the reference never reaches its target and the probe
        waits there for ever.  Counting steps ends it either way, and the
        anchoring is kept, so the press stays bounded at ``kp × step`` even if
        the back-off is the thing that is stuck.
        """
        if self.iterations >= self.backoff_steps:
            self._enter_close_probe()
            return ProbeOut(self._ref_rad, self.kp, self.kd, 0.0, self.note)

        # ``iterations == 0`` steps at once, so the axis leaves the stop on the
        # tick the limit is recorded rather than an interval later.
        if self.iterations == 0 or self._since_step >= self.step_interval_s:
            self._since_step = 0.0
            self.iterations += 1
            self._ref_rad = self._measured + self._sign * min(
                self.step_rad, self.backoff_rad
            )
        return ProbeOut(self._ref_rad, self.kp, self.kd, 0.0, self.note)

    def _finish_limit(self, measured_rad: float, how: str) -> None:
        if self.phase is GuidedPhase.OPEN_PROBE:
            self.open_rad = float(measured_rad)
            self._notes.append(f"张开极限 {self.open_rad:.6f} rad（{how}）")
            # Away from the stop now, not later: every tick spent holding at the
            # open limit is one more tick pressing at kp into a hard stop.
            self.phase = GuidedPhase.OPEN_BACKOFF
            self._ref_rad = self._measured
            self._since_step = 0.0
            self.iterations = 0
            self.stalls = 0
            self.note = f"已记录张开极限（{how}），正在回退"
            return

        if self.phase is GuidedPhase.CLOSE_PROBE:
            self.close_rad = float(measured_rad)
            self._notes.append(f"闭合极限 {self.close_rad:.6f} rad（{how}）")
            self._complete()

    def _enter_close_probe(self) -> None:
        """Leave the back-off and begin probing toward the closed stop.

        Steps in immediately, for the same reason :meth:`start` does: the first
        window is what the stall detector reads, and a window whose command
        cannot move anything reads as a stall.
        """
        self.phase = GuidedPhase.CLOSE_PROBE
        self._ref_rad = self._measured + self._sign * self.step_rad
        self._step_mark_rad = self._measured
        self._since_step = 0.0
        self.iterations = 0
        self.stalls = 0
        self.note = "正在探测闭合极限"

    def _complete(self) -> None:
        assert self.open_rad is not None and self.close_rad is not None
        outcome = summarise(
            self.close_rad, self.open_rad, self.max_stroke_mm, tuple(self._notes)
        )
        if isinstance(outcome, str):
            self._fail(outcome)
            return
        self.result = outcome
        self.phase = GuidedPhase.DONE
        self.note = (
            f"标定完成：行程 {outcome.travel_rad:.6f} rad，可命令 "
            f"{outcome.max_stroke_mm:.1f} mm（记录极限跨度 {outcome.stroke_mm:.1f} mm），"
            f"系数 {outcome.rad_to_mm:.2f} mm/rad"
        )

    def _fail(self, reason: str) -> None:
        self.phase = GuidedPhase.FAILED
        self.note = reason

    def _phase_frame(self) -> ProbeOut:
        if self.phase in TERMINAL_PHASES:
            return ProbeOut(self._measured, 0.0, 0.0, 0.0, self.note)
        return ProbeOut(self._ref_rad, self.kp, self.kd, 0.0, self.note)


# ═══════════════════════════════════════════════════════════════════════════
# Manual (zero-gravity) probe
# ═══════════════════════════════════════════════════════════════════════════
class ManualPhase(str, Enum):
    IDLE = "IDLE"
    RECORDING = "RECORDING"
    SETTLE = "SETTLE"
    RECOVER = "RECOVER"
    DONE = "DONE"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"


MANUAL_TERMINAL = frozenset(
    {ManualPhase.DONE, ManualPhase.FAILED, ManualPhase.CANCELLED}
)

#: The SDK's own failure text for a manual probe that captured no range
#: (gripper.py:462), keeping the console and the SDK saying the same thing about
#: the same procedure, with the console's extra guidance appended.
_NO_RANGE_MESSAGE = (
    "标定失败：未能捕获有效的位置范围。请确认已使能且有反馈，"
    "并在标定期间把夹爪推拉到两个极限"
)


class ManualCalibFSM:
    """Runs the axis limp and records the extremes the operator pushes it to.

    Zero torque throughout the recording, so the jaws can be driven by hand
    through the whole travel.  The operator stops when they are done — by
    pressing a button, which is what the SDK's Ctrl+C can never be from a worker
    thread — or the duration simply runs out.

    Sampling happens every tick rather than at the SDK's 100 Hz
    (gripper.py:376), which is strictly better for a min/max: more samples can
    only ever include more of the travel.
    """

    def __init__(
        self,
        max_stroke_mm: float = constants.DEFAULT_TRAVEL_MM,
        *,
        duration_s: float | None = None,
        settle_s: float = constants.MANUAL_SETTLE_S,
        recover_s: float = constants.MANUAL_RECOVER_S,
        pos_guard_rad: float = constants.MANUAL_POS_GUARD_RAD,
        hold_kp: float = constants.KP_MOVE,
        hold_kd: float = constants.KD_DEFAULT,
        max_jump_rad: float = MANUAL_MAX_JUMP_RAD_PER_TICK,
    ) -> None:
        self.max_stroke_mm = float(max_stroke_mm)
        self.duration_s = float(
            constants.MANUAL_DURATION_DEFAULT_S if duration_s is None else duration_s
        )
        self.settle_s = float(settle_s)
        self.recover_s = float(recover_s)
        self.pos_guard_rad = float(pos_guard_rad)
        self.hold_kp = float(hold_kp)
        self.hold_kd = float(hold_kd)
        self.max_jump_rad = float(max_jump_rad)

        self.phase = ManualPhase.IDLE
        self.open_rad: float | None = None
        self.close_rad: float | None = None
        self.result: CalibResult | None = None
        self.note = ""
        self.samples = 0
        self.rejected = 0

        self._elapsed = 0.0
        self._measured = 0.0
        self._have_measurement = False
        self._prev_rad: float | None = None

    # ── commands ────────────────────────────────────────────────────────────
    def start(self, measured_rad: float) -> bool:
        if not math.isfinite(measured_rad):
            self.phase = ManualPhase.FAILED
            self.note = f"位置读数无效 ({measured_rad!r})，无法开始标定"
            return False
        self.phase = ManualPhase.RECORDING
        self.result = None
        self._measured = float(measured_rad)
        self._have_measurement = True
        self._elapsed = 0.0
        self.open_rad = None
        self.close_rad = None
        self.samples = 0
        self.rejected = 0
        self._record(self._measured)
        self.note = "零重力：请用手把夹爪推到底再拉到底"
        return True

    def stop(self) -> None:
        """The operator is done: keep what was recorded and settle."""
        if self.phase is ManualPhase.RECORDING:
            self.phase = ManualPhase.SETTLE
            self._elapsed = 0.0
            self.note = "已停止记录，正在沉降"

    def cancel(self) -> None:
        self.phase = ManualPhase.CANCELLED
        self.note = "已取消标定"

    def fail(self, reason: str) -> None:
        """Stop the probe from outside — see :meth:`GuidedCalibFSM.fail`.

        A hand-moving operator sees the jaws move whether or not this console
        does, so the extremes recorded by a probe that has stopped hearing the
        motor are the extremes of nothing.
        """
        if self.phase not in MANUAL_TERMINAL:
            self._fail(reason)

    # ── the tick ────────────────────────────────────────────────────────────
    def tick(self, measured_rad: float, dt: float) -> ProbeOut:
        if self.phase is ManualPhase.IDLE:
            return NO_FRAME
        if self.phase is ManualPhase.CANCELLED:
            # Cancelled from zero gravity, so back to zero gravity: the operator
            # asked for the probe to stop, not for the jaws to stiffen.
            return FREE_FRAME
        if self.phase in MANUAL_TERMINAL:
            return self._hold_frame()

        if not math.isfinite(measured_rad):
            self._fail(f"位置读数无效 ({measured_rad!r})，已停止标定")
            return self._hold_frame()

        # A jump is dropped rather than fatal here: this probe only records, so
        # one bad sample costs nothing and the operator can carry on.  It matters
        # because a dropped frame leaves the SDK's cached position at zero, which
        # the ``|pos| < 50`` guard below does *not* catch — zero is well inside
        # it — and zero is exactly where a misread would put the open limit.
        jumped = (
            self._prev_rad is not None
            and abs(measured_rad - self._prev_rad) > self.max_jump_rad
        )
        self._prev_rad = float(measured_rad)
        self._measured = float(measured_rad)
        self._have_measurement = True
        self._elapsed += dt

        if self.phase is ManualPhase.RECORDING:
            if jumped:
                self.rejected += 1
            else:
                self._record(self._measured)
            if self._elapsed >= self.duration_s:
                self.phase = ManualPhase.SETTLE
                self._elapsed = 0.0
                self.note = "记录时间到，正在沉降"
            # Zero stiffness: the axis is limp and follows the operator's hand.
            return FREE_FRAME

        if self.phase is ManualPhase.SETTLE:
            # The SDK keeps sampling through its settle phase (gripper.py:448),
            # and so does this: the jaws often drift the last millimetre as the
            # hand releases them.
            if jumped:
                self.rejected += 1
            else:
                self._record(self._measured)
            if self._elapsed >= self.settle_s:
                self.phase = ManualPhase.RECOVER
                self._elapsed = 0.0
                self.note = "正在恢复保持"
            return FREE_FRAME

        # RECOVER: hand the axis back under position control, which is what the
        # SDK's exit_zero_gravity does (gripper.py:342) at the same gains.  The
        # alternative is leaving the motor executing a zero-gain frame while the
        # operator lets go of a gripper that has just been run to both stops.
        #
        # No sampling here: the recording is over, and folding in where the
        # recovery hold happens to settle would widen the travel the operator
        # measured by hand with a number the servo produced.
        if self._elapsed >= self.recover_s:
            self._complete()
        return self._hold_frame()

    # ── state, for the worker and the GUI ───────────────────────────────────
    @property
    def is_terminal(self) -> bool:
        return self.phase in MANUAL_TERMINAL

    @property
    def is_active(self) -> bool:
        return self.phase is not ManualPhase.IDLE and not self.is_terminal

    @property
    def progress(self) -> float:
        if self.phase is ManualPhase.RECORDING:
            return 0.8 * min(self._elapsed / max(self.duration_s, 1e-9), 1.0)
        if self.phase is ManualPhase.SETTLE:
            return 0.8 + 0.1 * min(self._elapsed / max(self.settle_s, 1e-9), 1.0)
        if self.phase is ManualPhase.RECOVER:
            return 0.9 + 0.1 * min(self._elapsed / max(self.recover_s, 1e-9), 1.0)
        return 1.0 if self.phase is ManualPhase.DONE else 0.0

    @property
    def remaining_s(self) -> float:
        if self.phase is not ManualPhase.RECORDING:
            return 0.0
        return max(self.duration_s - self._elapsed, 0.0)

    # ── internals ───────────────────────────────────────────────────────────
    def _hold_frame(self) -> ProbeOut:
        """Hold where the jaws are, at the SDK's own exit gains.

        This is a deliberate divergence from the guided probe, which ends at zero
        gains: a hold at the end of a *manual* probe holds the position the
        operator left the jaws at, while a hold at the end of a *guided* probe
        would be a press against the hard stop the probe just drove into.

        A probe that never saw a valid reading has no position to hold — holding
        at the ``0.0`` it was constructed with would command a move to zero.  It
        gets zero gains instead, which is both safe and the honest description of
        a probe that has measured nothing.
        """
        if not self._have_measurement:
            return FREE_FRAME
        return ProbeOut(self._measured, self.hold_kp, self.hold_kd, 0.0, self.note)

    def _record(self, rad: float) -> None:
        """Fold one sample into the extremes, ignoring implausible readings.

        The guard is the SDK's (gripper.py:427) and it is not a formality: a
        lost frame leaves the cached position at zero, and adopting that as a
        travel limit would produce a calibration with the closed stop in the
        middle of the stroke.
        """
        if abs(rad) >= self.pos_guard_rad:
            self.rejected += 1
            return
        self.samples += 1
        # ``open`` is the numerically smaller angle, ``close`` the larger.
        if self.open_rad is None or rad < self.open_rad:
            self.open_rad = rad
        if self.close_rad is None or rad > self.close_rad:
            self.close_rad = rad

    def _complete(self) -> None:
        if self.open_rad is None or self.close_rad is None or self.samples == 0:
            self._fail(_NO_RANGE_MESSAGE)
            return
        outcome = summarise(
            self.close_rad,
            self.open_rad,
            self.max_stroke_mm,
            (f"手工记录 {self.samples} 个样本（忽略 {self.rejected} 个越界读数）",),
            degenerate_message=_NO_RANGE_MESSAGE,
        )
        if isinstance(outcome, str):
            self._fail(outcome)
            return
        self.result = outcome
        self.phase = ManualPhase.DONE
        self.note = (
            f"标定完成：行程 {outcome.travel_rad:.6f} rad，可命令 "
            f"{outcome.max_stroke_mm:.1f} mm（记录极限跨度 {outcome.stroke_mm:.1f} mm），"
            f"系数 {outcome.rad_to_mm:.2f} mm/rad"
        )

    def _fail(self, reason: str) -> None:
        self.phase = ManualPhase.FAILED
        self.note = reason
