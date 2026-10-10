"""The worker: the command queue, the tick, the gate, and the stopping.

The loop is driven a tick at a time with an injected clock, so a 200 Hz control
loop and a three-second watchdog cost microseconds and produce the same result
on every machine.  The thread is exercised for real in the two tests that are
*about* the thread — the teardown ordering and the abort path — because those are
the ones where being wrong leaves a motor energised.

No hardware and no SDK: the backends here are either the simulator or a recorder
that logs what it was asked to do.
"""

from __future__ import annotations

import json
import logging
import threading
import time
from typing import Callable

import pytest

from litegrip_studio import calibration, constants
from litegrip_studio.backend import GripperBackend
from litegrip_studio.backend.plant import PlantConfig
from litegrip_studio.backend.sim import SimBackend
from litegrip_studio.calibration import CalibrationInfo
from litegrip_studio.core import commands as cmd
from litegrip_studio.core.calibration_fsm import (
    MANUAL_MAX_JUMP_RAD_PER_TICK,
    GuidedPhase,
)
from litegrip_studio.core.commands import AnyCommand
from litegrip_studio.core.motion import MotionState
from litegrip_studio.can_link import (
    LINK_CONFIGURED,
    LINK_DENIED,
    LINK_FAILED,
    LINK_FD,
    LINK_MISSING,
    LINK_OK,
    LinkOutcome,
)
from litegrip_studio.core.worker import (
    CommandQueue,
    GateState,
    GripperWorker,
    WorkerLoop,
    evaluate_gate,
)
from litegrip_studio.telemetry import Telemetry, TelemetryFrame
from litegrip_studio.units import Limits, frame_mismatch

from conftest import FakeClock

# The example user calibration: closed is numerically the LARGER angle.
CLOSED_RAD = 1.775959
OPEN_RAD = -0.064279
LIMITS = Limits(CLOSED_RAD, OPEN_RAD, 65.21, 120.0)

#: Closed is at 0 mm, so this is a bit past the middle of the travel.
MID_RAD = LIMITS.to_rad(60.0)

USER_CAL = CalibrationInfo(
    provenance=calibration.PROVENANCE_USER, limits=LIMITS, path="/tmp/cal.json"
)
BROKEN = CalibrationInfo(
    provenance=calibration.PROVENANCE_INVALID,
    limits=None,
    problems=("缺少必需字段: rad_to_mm",),
)


def _wait_for(predicate, timeout_s: float = 2.0, interval_s: float = 0.002) -> bool:
    """Poll ``predicate`` until it is true or the deadline passes.

    Only used by the tests that run the loop on a real thread, where the main
    thread has to wait for a state the loop reaches on its own — and where
    touching the backend to ask would be the very cross-thread access the worker
    exists to prevent.  The predicates handed to it are plain reads of the
    loop's own attributes.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval_s)
    return bool(predicate())


# ═══════════════════════════════════════════════════════════════════════════
# Doubles
# ═══════════════════════════════════════════════════════════════════════════
class Recorder:
    """Stands in for the Qt signals, recording instead of emitting them.

    Anything with the emit attributes works — that is the whole reason the loop
    takes its emitter as an argument — so this doubles as documentation of the
    signal contract.
    """

    def __init__(self) -> None:
        self.calls: list[tuple[str, tuple]] = []

    def __getattr__(self, name: str):
        def emit(*args) -> None:
            self.calls.append((name, args))

        return type("Signal", (), {"emit": staticmethod(emit)})()

    # ── queries ─────────────────────────────────────────────────────────────
    def of(self, name: str) -> list[tuple]:
        return [args for kind, args in self.calls if kind == name]

    def first(self, name: str) -> tuple:
        found = self.of(name)
        assert found, f"no {name} signal; got {[k for k, _ in self.calls]}"
        return found[0]

    def has(self, name: str) -> bool:
        return bool(self.of(name))

    def alerts(self) -> list[str]:
        return [text for _level, text in self.of("alert")]

    def logs(self) -> list[str]:
        """The log lines, level discarded — for asserting that something the
        operator needs was said at all, not how loudly."""
        return [text for _level, text in self.of("log")]


class RecordingBackend(GripperBackend):
    """A backend that records every call and can be told to fail.

    Not a mock of the SDK — a mock of *our* interface, which is the thing the
    worker is written against.  The physics tests use the simulator instead; this
    exists for the properties that are about ordering and refusal, where a
    simulated gripper only gets in the way.
    """

    def __init__(
        self,
        *,
        limits: Limits = LIMITS,
        info: CalibrationInfo | None = None,
        fail: tuple[str, ...] = (),
        fresh: bool = True,
    ) -> None:
        super().__init__()
        self.calls: list[tuple[str, tuple]] = []
        self._limits = limits
        self.info = info if info is not None else CalibrationInfo(
            provenance=calibration.PROVENANCE_USER, limits=limits, path="/tmp/cal.json"
        )
        self.fail = set(fail)
        self.fresh = fresh
        self.connected = False
        self.enabled = False
        self.q_rad = MID_RAD
        #: The angle the drive last reported while it was being addressed.  A
        #: disabled DM motor stops answering, so the SDK goes on serving this
        #: number while the mechanism itself sits wherever it was left — which is
        #: why :meth:`drag` moves ``q_rad`` without touching it.
        self._served_rad: float | None = None
        self.error_code = constants.ERROR_DISABLED
        self.rx_frames = 0

    def _record(self, name: str, *args) -> None:
        if name in self.fail:
            raise RuntimeError(f"{name} 故意失败")
        self.calls.append((name, args))

    def names(self) -> list[str]:
        return [name for name, _ in self.calls]

    # ── lifecycle ───────────────────────────────────────────────────────────
    def connect(self) -> None:
        self._claim()
        self._record("connect")
        self.connected = True

    def disconnect(self) -> None:
        self._claim()
        self._record("disconnect")
        self.connected = False

    def enable(self) -> None:
        self._claim()
        self._record("enable")
        self.enabled = True
        self.error_code = constants.ERROR_ENABLED

    def disable(self) -> None:
        self._claim()
        self._record("disable")
        self.enabled = False
        self.error_code = constants.ERROR_DISABLED

    def clear_fault(self) -> None:
        self._claim()
        self._record("clear_fault")
        self.error_code = constants.ERROR_ENABLED if self.enabled else constants.ERROR_DISABLED

    # ── control-rate primitives ─────────────────────────────────────────────
    def stream_frame(self, q_rad, kp, kd, dq_rad_s=0.0, tau_nm=0.0, *, ungated=False):
        self._claim()
        self._record("stream_frame", q_rad, kp, kd, dq_rad_s, tau_nm, ungated)
        return self.enabled

    def poll(self) -> bool:
        self._claim()
        self._record("poll")
        return self.fresh

    def drag(self, q_rad: float) -> None:
        """Push the jaws by hand while the drive is dark.

        The real backend reads the SDK's cache, and a disabled drive stops
        updating it: the angle served stays where it was while the mechanism
        moves.  A fake whose ``read`` followed ``q_rad`` straight away would hand
        the worker the hand's work as though the encoder had seen it, and the
        tests below would pass with the bug they exist to catch still in place.
        """
        self.q_rad = q_rad

    def read(self) -> Telemetry:
        self._claim()
        if self.enabled or self._served_rad is None:
            self._served_rad = self.q_rad
        q_rad = self._served_rad
        return Telemetry(
            position_rad=q_rad,
            velocity_rad_s=0.0,
            torque_nm=0.0,
            temperature_mos=30,
            temperature_coil=30,
            error_code=self.error_code,
            position_mm=self._limits.to_mm(q_rad),
            force_n=0.0,
            t=0.0,
        )

    def zero_torque(self) -> None:
        self._claim()
        self._record("zero_torque")

    # ── calibration ─────────────────────────────────────────────────────────
    def load_calibration(self, path: str | None = None) -> bool:
        self._claim()
        self._record("load_calibration", path)
        return self.info.usable

    def save_calibration(self, path: str | None = None) -> str:
        self._claim()
        self._record("save_calibration", path)
        written = path or self.info.path or "/tmp/cal.json"
        # The write is followed by a re-read in the real backend, and the re-read
        # is the whole point of it: it is what turns an in-memory probe result
        # into a saved one, promoting the provenance and opening the gate
        # (real.py::RealBackend.save_calibration).  A fake that only recorded the
        # call would leave an applied calibration looking unsaved — which is not
        # a harmless simplification: it is the difference between the gate
        # opening on 应用标定 and staying shut after it.
        #
        # The re-validation below is what resolve() does to a file it reads back,
        # minus the file: same raw evidence, same checks, and the "not saved yet"
        # warning no longer applies.
        if self.info is not None and self.info.provenance == calibration.PROVENANCE_MEMORY:
            raw = dict(self.info.raw)
            stroke = self.info.max_stroke_mm
            limits = calibration.limits_from_raw(raw, stroke)
            hard, soft = calibration.validate_limits(
                limits, stroke, calibration.file_scale(raw)
            )
            self.info = CalibrationInfo(
                provenance=(
                    calibration.PROVENANCE_USER if not hard else calibration.PROVENANCE_INVALID
                ),
                limits=limits if not hard else None,
                path=written,
                raw=raw,
                problems=tuple(hard),
                warnings=tuple(soft),
                max_stroke_mm=stroke,
            )
            if self.info.limits is not None:
                self._limits = self.info.limits
        return written

    def limits(self) -> Limits:
        return self._limits

    def describe(self) -> str:
        return "记录型后端"

    def calibration_info(self) -> CalibrationInfo | None:
        return self.info

    def set_calibration_memory(self, zero_rad, open_rad, rad_to_mm, max_stroke_mm=None):
        self._claim()
        self._record("set_calibration_memory", zero_rad, open_rad, rad_to_mm, max_stroke_mm)
        self.info = calibration.in_memory(
            zero_rad, open_rad, rad_to_mm, max_stroke_mm or self._limits.max_stroke_mm
        )
        if self.info.limits is not None:
            self._limits = self.info.limits
        return self.info

    def set_travel_mm(self, max_stroke_mm: float) -> None:
        self._claim()
        self._record("set_travel_mm", max_stroke_mm)
        self._limits = self._limits.with_max_stroke(max_stroke_mm)


class RecordingSim(SimBackend):
    """The simulator, plus the record of what it was told.

    The physics tests want the plant; the few tests that are about which *frames*
    the loop sends through a real probe want both, and the plant alone cannot
    answer them — a mechanism with no spring in it sits still whether it is held
    or abandoned.
    """

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.frames: list[tuple] = []
        self.zeroes = 0

    def stream_frame(self, q_rad, kp, kd, dq_rad_s=0.0, tau_nm=0.0, *, ungated=False):
        self.frames.append((q_rad, kp, kd, dq_rad_s, tau_nm, ungated))
        return super().stream_frame(q_rad, kp, kd, dq_rad_s, tau_nm, ungated=ungated)

    def zero_torque(self) -> None:
        self.zeroes += 1
        super().zero_torque()


class FakeLink:
    """Stands in for the CAN bring-up the worker runs before it connects.

    A double for the same reason as the backend: the loop is written against one
    method and one attribute — ``ensure()`` and ``channel`` — so this is also
    the record of that contract.  ``backend`` is set by the tests that care about
    the interface being raised *before* the bus is opened.
    """

    channel = "can0"

    def __init__(
        self,
        outcome: LinkOutcome | None = None,
        *,
        raises: BaseException | None = None,
        on_ensure: Callable[[], None] | None = None,
    ) -> None:
        self.calls = 0
        self.outcome = outcome if outcome is not None else LinkOutcome(
            LINK_OK, "can0 已就绪（已 up，经典 CAN，比特率 1000000），未改动"
        )
        self.raises = raises
        self.on_ensure = on_ensure
        self.backend = None
        self.connected_when_asked: bool | None = None

    def ensure(self) -> LinkOutcome:
        self.calls += 1
        if self.backend is not None:
            self.connected_when_asked = self.backend.connected
        if self.on_ensure is not None:
            self.on_ensure()
        if self.raises is not None:
            raise self.raises
        return self.outcome


class Bench:
    """A loop, its backend and its recorder, ticked by hand."""

    def __init__(self, backend=None, clock=None, **kwargs) -> None:
        # A backend that integrates its own physics needs the same clock the
        # loop is ticked by, so a bench against the simulator is handed one
        # rather than making its own — two clocks would make the plant's dt
        # whatever the wall did between two lines of a test.
        self.clock = clock if clock is not None else FakeClock()
        self.backend = backend if backend is not None else RecordingBackend()
        self.signals = Recorder()
        self.loop = WorkerLoop(
            self.backend,
            self.signals,
            clock=self.clock,
            sleep=lambda seconds: None,
            watchdog_s=kwargs.pop("watchdog_s", None),
            **kwargs,
        )

    def tick(self, count: int = 1, dt: float = constants.CTRL_DT):
        for _ in range(count):
            self.loop.tick_once(dt)
            self.clock.advance(dt)
        return self.loop.last_frame

    def frame(self, count: int = 1) -> TelemetryFrame:
        """Tick, then keep ticking until a telemetry frame is published.

        Telemetry is published at 50 Hz from a 200 Hz loop, so a single tick
        after a command usually has *not* published yet — and the frame the GUI
        would be showing is the previous one.  Tests that assert on what the
        operator sees have to wait for the publish, exactly as the GUI does.
        """
        self.tick(count)
        before = len(self.signals.of("telemetry"))
        for _ in range(8):
            if len(self.signals.of("telemetry")) > before:
                break
            self.tick()
        return self.loop.last_frame

    def send(self, command: AnyCommand, count: int = 1) -> TelemetryFrame:
        self.loop.submit(command)
        return self.frame(count)

    def bring_up(self) -> None:
        """Connected, enabled, gate open — the state most tests start from."""
        self.send(cmd.Connect())
        self.send(cmd.Enable())

    def swap_calibration(self, info: CalibrationInfo | None) -> None:
        """Change the calibration the way the console discovers one.

        Through :class:`LoadCalibration` and the backend rather than by poking
        the loop's own field: the refresh path is part of what is being tested,
        and a test that reached past it would pass with the refresh broken.
        """
        self.backend.info = info
        self.send(cmd.LoadCalibration())

    def drive_to(self, mm: float) -> None:
        """Walk the reported position to ``mm``, a little per tick.

        A teleport is not a hand-move: the manual probe carries a jump guard, and
        a test that jumped from one end of the travel to the other in one tick
        would be testing the guard rather than the probe.  The step is half the
        guard, so the walk is comfortably inside what the probe accepts.
        """
        current = self.backend.q_rad
        target = self.backend.limits().to_rad(mm)
        delta = target - current
        step = MANUAL_MAX_JUMP_RAD_PER_TICK / 2.0
        ticks = max(2, int(abs(delta) / step) + 1)
        for _ in range(ticks):
            current += delta / ticks
            self.backend.q_rad = current
            self.tick()

    def run_until_probe_finishes(self, timeout_s: float = 10.0) -> None:
        for _ in range(int(timeout_s / constants.CTRL_DT)):
            if self.loop.probe is None:
                return
            self.tick()
        raise AssertionError("探测未在超时内结束")


# ═══════════════════════════════════════════════════════════════════════════
# The gate
# ═══════════════════════════════════════════════════════════════════════════
class TestEvaluateGate:
    def _info(self, provenance: str, limits: Limits | None = LIMITS, problems=()) -> CalibrationInfo:
        return CalibrationInfo(
            provenance=provenance,
            limits=limits,
            path="/tmp/cal.json",
            problems=tuple(problems),
        )

    def test_a_user_calibration_opens_the_gate(self) -> None:
        state, why = evaluate_gate(self._info(calibration.PROVENANCE_USER))
        assert state is GateState.READY
        assert why

    def test_a_missing_calibration_blocks(self) -> None:
        state, why = evaluate_gate(self._info(calibration.PROVENANCE_MISSING, limits=None))
        assert state is GateState.BLOCKED
        assert why

    def test_a_calibration_with_no_travel_blocks_and_says_why(self) -> None:
        """The one shape a pair of recorded angles can have that is unusable:
        both the same, so there is no travel to derive a scale from.

        Deliberately not the other ordering of two different angles — that is a
        good calibration for a unit whose angle grows as the jaws open, and it
        opens the gate like any other.  The problems are taken from the
        validator rather than written out here, so this test cannot go on
        passing against a message the console no longer produces.
        """
        problems, _warnings = calibration.validate_limits(Limits(1.0, 1.0, 65.21, 120.0))

        state, why = evaluate_gate(
            self._info(calibration.PROVENANCE_INVALID, limits=None, problems=problems)
        )

        assert state is GateState.BLOCKED
        assert GateState.READY != state
        assert "行程" in why

    def test_the_factory_file_is_ready_with_nothing_to_acknowledge(self) -> None:
        """The console's default calibration has to be usable as it stands.

        There used to be a third gate state here and a risk checkbox to leave it
        through.  The factory file is what the console ships and falls back to on
        a fresh machine, so a state that always asks the same question has no
        answer that is ever different — whether these numbers describe *this*
        gripper is put to the encoder instead (see :func:`frame_mismatch`).
        """
        info = self._info(calibration.PROVENANCE_FACTORY)

        assert evaluate_gate(info)[0] is GateState.READY

    def test_an_unsaved_probe_result_is_gated(self) -> None:
        """Its numbers are fine and the UI should show them; it is not *saved*,
        so nothing about it survives a restart and motion cannot be planned on it."""
        info = calibration.in_memory(CLOSED_RAD, OPEN_RAD, 65.21, 120.0)
        assert info.usable
        state, why = evaluate_gate(info)
        assert state is GateState.BLOCKED
        assert "保存" in why

    def test_nothing_lifts_a_block(self) -> None:
        """A blocked provenance stays blocked.  These numbers do not describe
        this gripper and there is no risk to accept that would make them."""
        for provenance in (
            calibration.PROVENANCE_MISSING,
            calibration.PROVENANCE_INVALID,
            calibration.PROVENANCE_MEMORY,
        ):
            info = self._info(provenance, limits=None)
            assert evaluate_gate(info)[0] is GateState.BLOCKED

    def test_an_unknown_backend_blocks(self) -> None:
        """A backend that does not track its calibration is not a backend to
        drive: an unknown calibration is an unusable one."""
        assert evaluate_gate(None)[0] is GateState.BLOCKED


class TestFrameMismatch:
    """The one gate check that reads the hardware instead of the file.

    Every other check asks whether the numbers are self-consistent, and they can
    all pass while the file describes a completely different gripper.  ``LIMITS``
    here is that file: closed at +1.775959 rad, which is what the console had
    been loading while the unit's own travel was [-1.436446, -0.109674].
    """

    def test_a_calibration_from_another_frame_blocks_with_the_numbers(self) -> None:
        why = frame_mismatch(LIMITS, -1.0)

        assert why
        assert "-1.00" in why, "the reading the judgement was made from"
        assert f"{LIMITS.rad_low:.4f}" in why and f"{LIMITS.rad_high:.4f}" in why
        assert "重新标定" in why, "the operator needs a way out, not just a refusal"

    def test_an_axis_parked_on_a_hard_stop_is_not_a_mismatch(self) -> None:
        """Why this check needs a tolerance at all: a calibration that stops
        short of the boundaries — the safer kind, and the one this console
        recommends — leaves the gripper legitimately outside its own commanded
        range whenever it rests on a stop."""
        red_lines = Limits(-0.170982, -1.403653, 65.0229, 80.16)

        assert frame_mismatch(red_lines, -0.109674) == ""  # closed hard stop
        assert frame_mismatch(red_lines, -1.436446) == ""  # open hard stop

    def test_the_tolerance_is_where_it_says_it_is(self) -> None:
        """Pinned from both sides, because the number is a compromise: it must
        clear how a file was written and still catch a different frame."""
        slack = constants.CALIB_MISMATCH_RAD

        assert frame_mismatch(LIMITS, LIMITS.rad_low - slack) == ""
        assert frame_mismatch(LIMITS, LIMITS.rad_high + slack) == ""
        assert frame_mismatch(LIMITS, LIMITS.rad_low - slack - 1e-6)
        assert frame_mismatch(LIMITS, LIMITS.rad_high + slack + 1e-6)

    def test_a_travel_larger_than_the_offset_is_still_a_mismatch(self) -> None:
        """The tolerance is in radians, not a fraction of the travel: a file
        cannot buy room by claiming a longer one."""
        generous = Limits(1.775959, -0.064279, 65.21, 300.0)

        assert frame_mismatch(generous, -1.0)

    def test_a_reading_that_is_not_a_number_condemns_nothing(self) -> None:
        """NaN is what a backend reports when it has no answer; it is not
        evidence about the calibration."""
        assert frame_mismatch(LIMITS, float("nan")) == ""

    def test_no_reading_leaves_the_gate_exactly_as_it_was(self) -> None:
        info = CalibrationInfo(
            provenance=calibration.PROVENANCE_USER, limits=LIMITS, path="/tmp/cal.json"
        )

        assert evaluate_gate(info)[0] is GateState.READY
        assert evaluate_gate(info, measured_rad=MID_RAD)[0] is GateState.READY
        assert evaluate_gate(info, measured_rad=-1.0)[0] is GateState.BLOCKED

    def test_a_factory_file_from_another_frame_is_refused(self) -> None:
        """Which is the whole of what is left of the factory check.  A file from
        the wrong gripper is not a risk to accept — it is a fact about the wrong
        gripper, and the encoder is what says so."""
        info = CalibrationInfo(
            provenance=calibration.PROVENANCE_FACTORY, limits=LIMITS, path="/tmp/f.json"
        )

        assert evaluate_gate(info, measured_rad=-1.0)[0] is GateState.BLOCKED


class TestGateInTheLoop:
    def test_a_factory_calibration_drives_the_axis_with_no_acknowledgement(
        self,
    ) -> None:
        bench = Bench(RecordingBackend(info=CalibrationInfo(
            provenance=calibration.PROVENANCE_FACTORY, limits=LIMITS, path="/tmp/f.json"
        )))
        bench.send(cmd.Connect())

        assert bench.loop.gate is GateState.READY

        bench.send(cmd.Enable())
        assert bench.loop.enabled

        # Straight to SERVO, with nothing ticked in between.  This is the case
        # the console starts in on a machine that has never been calibrated.
        bench.send(cmd.MoveToMm(30.0), count=10)
        assert bench.loop.motion.state is MotionState.SERVO

    def test_a_move_is_refused_while_the_gate_is_shut(self) -> None:
        bench = Bench()
        bench.bring_up()
        bench.send(cmd.MoveToMm(30.0), count=10)
        assert bench.loop.motion.state is MotionState.SERVO

        bench.swap_calibration(BROKEN)
        bench.tick()
        assert bench.loop.gate is GateState.BLOCKED
        assert bench.loop.motion.state is MotionState.BLOCKED

        bench.send(cmd.MoveToMm(30.0))
        assert bench.loop.motion.state is not MotionState.SERVO
        assert "被拒绝" in bench.signals.alerts()[-1]

    def test_closing_the_gate_withdraws_the_position_command(self) -> None:
        """A refused frame is not enough: with the limits in doubt, the motor
        must not go on executing the last command derived from them."""
        bench = Bench()
        bench.bring_up()
        bench.send(cmd.MoveToMm(30.0), count=20)
        assert bench.loop.motion.state is MotionState.SERVO
        bench.backend.calls.clear()

        bench.swap_calibration(BROKEN)
        assert "zero_torque" in bench.backend.names()

    def test_opening_the_gate_takes_up_the_hold(self) -> None:
        """An enabled DM motor with no frames sent to it is not a safe resting
        state, so the axis holds its pose the moment it is allowed to."""
        bench = Bench()
        bench.bring_up()
        bench.swap_calibration(BROKEN)
        bench.send(cmd.Stop())
        assert bench.loop.motion.state is MotionState.IDLE

        bench.swap_calibration(USER_CAL)
        assert bench.loop.gate is GateState.READY
        assert bench.loop.motion.state is MotionState.HOLD

    def test_a_file_from_another_unit_sends_no_frame_at_all(self) -> None:
        """The reported failure, reproduced end to end.

        A file putting 0 mm at +1.775959 rad, on a unit whose own range is
        [-1.436446, -0.109674]: readings came out 123-209 mm, each one clamped to
        the closed end, and what came out of that clamp was 2.95 mm *past* the
        closed hard stop.  Enabling drove the gripper into it.
        """
        backend = RecordingBackend(limits=LIMITS, fresh=False)
        backend.q_rad = -1.0  # where this unit actually is
        bench = Bench(backend)

        bench.send(cmd.Connect())
        assert bench.loop.gate is GateState.READY, "nothing has answered yet"
        backend.calls.clear()  # from here on, only the enable and what it causes

        backend.fresh = True  # the motor answers the moment it is enabled
        bench.send(cmd.Enable())
        bench.tick(5)

        assert bench.loop.gate is GateState.BLOCKED
        assert "stream_frame" not in backend.names(), "the frame never goes out"
        assert "zero_torque" in backend.names(), "withdrawn, not merely refused"
        assert "不属于这台夹爪" in bench.signals.of("gate_state")[-1][1]

    def test_loading_a_file_for_this_unit_opens_the_gate_again(self) -> None:
        """A judgement about the file in force, not a latch — and the way out is
        the correct file, so it has to be reachable from the blocked state."""
        this_unit = Limits(-0.170982, -1.403653, 65.0229, 80.16)
        backend = RecordingBackend(limits=LIMITS, fresh=False)
        backend.q_rad = -1.0
        bench = Bench(backend)
        bench.send(cmd.Connect())
        backend.fresh = True
        bench.send(cmd.Enable())
        bench.tick(5)
        assert bench.loop.gate is GateState.BLOCKED

        backend.info = CalibrationInfo(
            provenance=calibration.PROVENANCE_USER, limits=this_unit, path="/tmp/ok.json"
        )
        # The double converts mm with its own copy of the limits, so it has to be
        # told as well — otherwise the reading it hands back would still be the
        # one computed from the file now in doubt.
        backend._limits = this_unit
        bench.send(cmd.LoadCalibration())
        bench.tick(5)

        assert bench.loop.gate is GateState.READY
        sent = [
            q for name, args in backend.calls if name == "stream_frame" for q in args[:1]
        ]
        assert sent, "the axis takes up its hold again"
        assert all(this_unit.rad_low <= q <= this_unit.rad_high for q in sent)
        assert abs(sent[-1] - backend.q_rad) < constants.CALIB_MISMATCH_RAD


# ═══════════════════════════════════════════════════════════════════════════
# Energising a console whose calibration is unusable
# ═══════════════════════════════════════════════════════════════════════════
class TestEnablingWithNoUsableCalibration:
    """使能 must not be the thing that gets refused.

    It used to be: a calibration that fails validation shuts the gate, and a
    shut gate refused 使能 outright — which locks the operator out of the one
    flow that can replace the file.  The manual probe records its limits off a
    motor that answers, and enabling is what makes it answer.

    What the gate withholds is *millimetres*, so the axis comes up on a hold of
    the angle the encoder reports.  That needs no calibration at all: it is the
    identity under every file there could be, and it is the only hold that is
    safe under a file whose numbers are in doubt — a millimetre hold would clamp
    the measured angle into a travel nobody has vetted and drive there.
    """

    def blocked(self) -> Bench:
        bench = Bench(RecordingBackend(info=BROKEN, limits=LIMITS))
        bench.send(cmd.Connect())
        assert bench.loop.gate is GateState.BLOCKED
        return bench

    def held_frames(self, bench: Bench) -> list[tuple]:
        bench.backend.calls.clear()
        bench.tick(5)
        return [c[1] for c in bench.backend.calls if c[0] == "stream_frame"]

    def test_a_blocked_gate_still_lets_the_axis_be_energised(self) -> None:
        bench = self.blocked()

        bench.send(cmd.Enable())

        assert bench.loop.enabled
        assert bench.loop.motion.state is MotionState.HOLD_RAD

    def test_the_operator_is_told_why_it_cannot_move_in_millimetres(self) -> None:
        """Left unsaid, a gripper that holds but will not move reads as broken,
        and the operator goes looking for a hardware fault instead of a probe."""
        bench = self.blocked()

        bench.send(cmd.Enable())

        alert = bench.signals.alerts()[-1]
        assert "标定不可用" in alert
        assert "重新标定" in alert, "the way out, not just the refusal"

    def test_the_hold_is_the_angle_the_encoder_reports(self) -> None:
        bench = self.blocked()

        bench.send(cmd.Enable())

        frames = self.held_frames(bench)
        assert len(frames) == 5, "an enabled axis is talked to every tick"
        assert all(f[0] == pytest.approx(bench.backend.q_rad) for f in frames)
        assert all(f[1] == constants.KP_MOVE for f in frames)
        assert all(f[4] == 0.0 for f in frames), "no feed-forward from a bad file"

    def test_the_hold_claims_no_millimetre_target(self) -> None:
        """Published as no command, so the UI never shows a target derived from
        the limits under suspicion — and so nothing in the frame goes through
        the conversion that a wrong file would corrupt."""
        bench = self.blocked()

        bench.send(cmd.Enable())

        assert self.held_frames(bench), "held, not left silent"
        assert bench.frame().cmd_mm is None

    def test_a_multi_tick_hold_never_clamps_the_measured_angle(self) -> None:
        """The failure a millimetre hold would produce here: the reading is
        outside ``LIMITS``, so a conversion would pull it to the nearest end of
        the travel and command the jaws *there*."""
        backend = RecordingBackend(info=BROKEN, limits=LIMITS)
        backend.q_rad = -1.0  # outside LIMITS in both directions
        bench = Bench(backend)
        bench.send(cmd.Connect())
        assert not (LIMITS.rad_low <= backend.q_rad <= LIMITS.rad_high)

        bench.send(cmd.Enable())

        frames = self.held_frames(bench)
        assert len(frames) == 5
        assert all(f[0] == pytest.approx(-1.0) for f in frames)

    def test_a_move_is_still_refused_while_the_gate_is_shut(self) -> None:
        """Energising is allowed; steering by a number nobody has vetted is
        not."""
        bench = self.blocked()
        bench.send(cmd.Enable())

        bench.send(cmd.MoveToMm(30.0), count=4)

        assert bench.loop.motion.state is MotionState.HOLD_RAD
        assert "被拒绝" in bench.signals.alerts()[-1]

    def test_loading_a_usable_file_returns_the_hold_to_millimetres(self) -> None:
        """The radian hold is a fallback, not a mode.  Once a travel can be
        trusted the console names the pose in millimetres again — the same
        pose, but with a target the UI can show and a move that starts from
        where the operator thinks it does."""
        backend = RecordingBackend(limits=LIMITS, info=BROKEN)
        bench = Bench(backend)
        bench.send(cmd.Connect())
        bench.send(cmd.Enable())
        bench.tick(3)
        assert bench.loop.motion.state is MotionState.HOLD_RAD

        backend.info = USER_CAL
        bench.send(cmd.LoadCalibration())
        bench.tick(3)

        assert bench.loop.gate is GateState.READY
        assert bench.loop.motion.state is MotionState.HOLD
        assert bench.frame().cmd_mm == pytest.approx(
            LIMITS.to_mm(backend.q_rad), abs=1e-6
        )

    def test_a_dark_axis_does_not_condemn_the_file_with_a_leftover_angle(self) -> None:
        """The gate is a judgement about the hardware, so it needs the hardware
        to be answering.  A disabled drive goes on serving the angle it last
        saw while the jaws sit anywhere at all; judging the file against that
        leftover is a verdict about nothing — and it would tell the operator
        their calibration is wrong when the gripper is simply switched off."""
        backend = RecordingBackend(limits=LIMITS, info=USER_CAL)
        backend.q_rad = -1.0  # a pose LIMITS knows nothing about
        bench = Bench(backend)
        bench.send(cmd.Connect())
        bench.send(cmd.Enable())
        bench.tick(3)
        assert bench.loop.gate is GateState.BLOCKED, "while it is answering"

        bench.send(cmd.Disable())
        bench.tick(3)

        assert backend.read().position_rad == pytest.approx(-1.0), "still served"
        assert bench.loop.gate is GateState.READY, "but not by anyone answering"


# ═══════════════════════════════════════════════════════════════════════════
# The command queue
# ═══════════════════════════════════════════════════════════════════════════
class TestCommandQueue:
    def test_it_drains_in_order(self) -> None:
        queue = CommandQueue()
        queue.put(cmd.Connect())
        queue.put(cmd.MoveToMm(10.0))
        assert queue.drain() == [cmd.Connect(), cmd.MoveToMm(10.0)]
        assert queue.drain() == []

    def test_an_overflow_drops_the_oldest_repeatable_command(self) -> None:
        """A dropped slider tick costs nothing: the operator's finger has moved
        on, and the next tick carries the position it moved to."""
        queue = CommandQueue(maxlen=3)
        queue.put(cmd.Connect())
        queue.put(cmd.MoveToMm(1.0))
        queue.put(cmd.MoveToMm(2.0))
        assert queue.put(cmd.MoveToMm(3.0)) == 1
        assert queue.drain() == [cmd.Connect(), cmd.MoveToMm(2.0), cmd.MoveToMm(3.0)]

    def test_an_overflow_never_drops_a_lifecycle_command(self) -> None:
        """A connect or a fault clear has an effect a later one of the same kind
        does not supersede — and it is usually what unblocks the backlog."""
        queue = CommandQueue(maxlen=2)
        queue.put(cmd.Connect())
        queue.put(cmd.ClearFault())
        assert queue.put(cmd.MoveToMm(1.0)) == 1
        assert queue.drain() == [cmd.Connect(), cmd.ClearFault()]

    def test_the_cap_is_a_cap(self) -> None:
        cap = 8
        queue = CommandQueue(maxlen=cap)
        for i in range(200):
            queue.put(cmd.MoveToMm(float(i)))
        assert len(queue) <= cap

    def test_waiting_wakes_on_a_command(self) -> None:
        queue = CommandQueue()
        assert not queue.wait_for_work(0.0)
        queue.put(cmd.Connect())
        assert queue.wait_for_work(0.0)

    def test_waiting_wakes_on_close(self) -> None:
        queue = CommandQueue()
        queue.close()
        assert queue.wait_for_work(5.0)


class TestCommandBursts:
    def test_fifty_drag_events_start_one_move_to_the_last_position(self) -> None:
        """Fifty drag events between two ticks describe one position: where the
        operator's finger ended up."""
        bench = Bench()
        bench.bring_up()
        bench.backend.calls.clear()

        for mm in range(50):
            bench.loop.submit(cmd.MoveToMm(float(mm), source="slider"))
        bench.tick()

        assert bench.loop.motion.last_command_mm == 49.0
        assert bench.loop.motion.state is MotionState.SERVO
        sent = [c for c in bench.backend.calls if c[0] == "stream_frame"]
        assert len(sent) == 1, "the burst should have produced exactly one frame"

    def test_a_burst_does_not_lose_a_stop_queued_behind_it(self) -> None:
        bench = Bench()
        bench.bring_up()
        for mm in range(50):
            bench.loop.submit(cmd.MoveToMm(float(mm), source="slider"))
        bench.loop.submit(cmd.Stop())
        bench.tick()
        assert bench.loop.motion.state is MotionState.HOLD


# ═══════════════════════════════════════════════════════════════════════════
# The tick
# ═══════════════════════════════════════════════════════════════════════════
class TestTick:
    def test_telemetry_is_published_at_the_publish_rate(self) -> None:
        bench = Bench()
        bench.bring_up()
        bench.signals.calls.clear()
        bench.tick(constants.CTRL_HZ)  # one second of control ticks
        published = bench.signals.of("telemetry")
        assert len(published) == pytest.approx(constants.TELEMETRY_HZ, abs=2)

    def test_the_published_position_follows_the_backend(self) -> None:
        """This is what makes the slider track the jaws while they move: the
        number the widget draws is the one the backend just reported."""
        bench = Bench()
        bench.bring_up()
        assert bench.frame().position_mm == pytest.approx(60.0, abs=1e-6)

        bench.backend.q_rad = LIMITS.to_rad(12.5)
        assert bench.frame().position_mm == pytest.approx(12.5, abs=1e-6)

    def test_the_motion_label_names_the_probe_while_one_is_running(self) -> None:
        """A status line reading IDLE while the jaws are being driven into a
        hard stop would be worse than useless."""
        bench = Bench()
        bench.bring_up()
        bench.send(cmd.StartGuidedCalibration())
        assert bench.frame().motion_state == GuidedPhase.OPEN_PROBE.value

    def test_it_starts_disconnected(self) -> None:
        bench = Bench()
        frame = bench.tick()
        assert not bench.loop.connected
        assert not frame.enabled
        assert frame.motion_state == MotionState.IDLE.value

    def test_connecting_loads_a_calibration_explicitly(self) -> None:
        """Nothing may leave the choice to the SDK: its fallback to the factory
        file is silent, and a silent fallback is how mm readings go wrong."""
        bench = Bench()
        bench.send(cmd.Connect())
        assert ("load_calibration", (None,)) in bench.backend.calls
        assert bench.loop.gate is GateState.READY

    def test_an_unusable_calibration_is_reported_loudly(self) -> None:
        bench = Bench(RecordingBackend(info=CalibrationInfo(
            provenance=calibration.PROVENANCE_INVALID,
            limits=None,
            problems=("缺少必需字段: rad_to_mm",),
        )))
        bench.send(cmd.Connect())
        assert bench.loop.gate is GateState.BLOCKED
        assert "缺少必需字段" in bench.signals.alerts()[-1]

    def test_enabling_holds_where_the_jaws_are(self) -> None:
        bench = Bench()
        bench.bring_up()
        assert bench.loop.motion.state is MotionState.HOLD
        assert bench.frame().cmd_mm == pytest.approx(60.0, abs=1e-6)

    def test_a_failing_enable_reports_the_fault_it_actually_is(self) -> None:
        """“欠压” and “使能失败” call for completely different actions, so the
        code has to survive the trip out."""
        clock = FakeClock()
        backend = SimBackend(clock=clock, uv=True)
        signals = Recorder()
        loop = WorkerLoop(backend, signals, clock=clock, sleep=lambda s: None, watchdog_s=None)
        loop.submit(cmd.Connect())
        loop.tick_once(constants.CTRL_DT)

        loop.submit(cmd.Enable())
        loop.tick_once(constants.CTRL_DT)

        code, message, hint = signals.first("fault")
        assert code == constants.ERROR_UV
        assert message == constants.describe_error(constants.ERROR_UV)
        assert hint == constants.FAULT_HINTS[constants.ERROR_UV]
        assert not loop.enabled

    def test_link_death_refuses_motion(self) -> None:
        bench = Bench(RecordingBackend(fresh=False))
        bench.bring_up()
        bench.send(cmd.MoveToMm(30.0))
        bench.clock.advance(constants.LINK_STALE_MS / 1000.0 + 0.01)
        bench.send(cmd.MoveToMm(30.0))
        assert "链路已断" in bench.signals.alerts()[-1]

    def test_a_quiet_link_while_disabled_is_not_a_fault(self) -> None:
        """Both backends' poll() returns False whenever the motor is not enabled,
        so a console that read silence as death would declare the link dead the
        moment it disabled the motor and refuse to enable it again."""
        bench = Bench(RecordingBackend(fresh=False))
        bench.send(cmd.Connect())
        assert bench.tick().stale_ms == 0.0
        bench.send(cmd.Enable())
        assert bench.loop.enabled
        assert bench.tick().stale_ms < constants.LINK_STALE_MS

    def test_a_motor_fault_stops_the_axis_and_latches_the_gate(self) -> None:
        bench = Bench()
        bench.bring_up()
        bench.send(cmd.MoveToMm(30.0), count=10)
        bench.backend.calls.clear()

        bench.backend.error_code = constants.ERROR_OC
        bench.tick()

        assert bench.loop.motion.state is MotionState.FAULT
        assert "zero_torque" in bench.backend.names()
        code, _message, hint = bench.signals.first("fault")
        assert code == constants.ERROR_OC
        assert hint == constants.FAULT_HINTS[constants.ERROR_OC]

    def test_a_command_that_raises_is_reported_and_not_fatal(self) -> None:
        bench = Bench(RecordingBackend(fail=("load_calibration",)))
        bench.send(cmd.Connect())
        assert "命令失败" in bench.signals.alerts()[-1]

    def test_the_watchdog_stops_the_session_when_the_gui_goes_away(self) -> None:
        """The console's only protection against a hung GUI, and it fails in the
        safe direction: no heartbeat means no operator."""
        bench = Bench(watchdog_s=1.0)
        bench.bring_up()
        bench.tick()
        assert bench.loop.enabled

        bench.clock.advance(1.5)
        bench.tick()
        assert not bench.loop.enabled
        assert bench.loop.stopping
        assert bench.signals.of("alert")[-1][0] == "fatal"
        assert "心跳" in bench.signals.alerts()[-1]

    def test_a_slow_command_is_not_mistaken_for_a_dead_gui(self) -> None:
        """The loop spends the whole of a blocking SDK call inside it — the SDK's
        own ``enable()`` retries for up to ten seconds — while the GUI keeps
        queueing heartbeats that have no chance of being read until it returns.

        The GUI is demonstrably alive throughout, which is why the heartbeat is
        stamped where it arrives and not where it is applied.  Get that wrong and
        the console answers a slow enable by cutting the motor.
        """
        bench = Bench(watchdog_s=1.0)
        backend = bench.backend
        enable = backend.enable

        def slow_enable() -> None:
            bench.clock.advance(5.0)
            for _ in range(10):  # the GUI's 500 ms timer, still firing
                bench.loop.submit(cmd.Heartbeat())
            enable()

        backend.enable = slow_enable  # type: ignore[method-assign]

        bench.send(cmd.Connect())
        bench.send(cmd.Enable())

        assert bench.loop.enabled
        assert not bench.loop.stopping

    def test_a_heartbeat_keeps_it_alive(self) -> None:
        bench = Bench(watchdog_s=1.0)
        bench.bring_up()
        for _ in range(20):
            bench.clock.advance(0.25)
            bench.send(cmd.Heartbeat())
        assert bench.loop.enabled


# ═══════════════════════════════════════════════════════════════════════════
# 使能 before the axis has answered
# ═══════════════════════════════════════════════════════════════════════════
class TestEnablingBeforeTheAxisHasAnswered:
    """按使能 used to drive the jaws to the closed stop.

    ``get_state().position_rad`` is 0.0 until a status frame arrives, and
    ``to_mm(0.0)`` is a place *inside* the travel, so the console converted an
    angle the motor had never sent into a plausible-looking position and then
    held it — on a real gripper, 0 mm after the clamp.  The same number is what
    every later hold would have been frozen at, so the property is worth
    pinning in the loop rather than in the widget that drew it.
    """

    def silent(self) -> Bench:
        """A bench whose axis answers nothing — no frame ever arrives."""
        bench = Bench(RecordingBackend(fresh=False))
        bench.bring_up()
        return bench

    def sent_gains(self, bench: Bench) -> list[tuple[float, float, float]]:
        return [
            (kp, kd, tau)
            for _q, kp, kd, _dq, tau, _probe in [
                c[1] for c in bench.backend.calls if c[0] == "stream_frame"
            ]
        ]

    def test_it_does_not_hold_a_position_it_cannot_measure(self) -> None:
        bench = self.silent()
        assert bench.loop.enabled
        assert bench.loop.motion.state is MotionState.RELEASE

    def test_no_frame_it_sends_carries_a_stiffness(self) -> None:
        """Zero gain is the only command that is not derived from a position:
        with kp and kd both zero a stray q is inert, which is what makes this
        state safe to sit in while the link wakes up."""
        bench = self.silent()
        gains = self.sent_gains(bench)
        assert gains, "使能后必须继续发帧，电机需要它才保持使能"
        assert all(kp == 0.0 and kd == 0.0 and tau == 0.0 for kp, kd, tau in gains)

    def test_the_operator_is_told_why_the_jaws_are_limp(self) -> None:
        bench = self.silent()
        assert any("尚未读到位置" in text for text in bench.signals.logs())

    def test_the_first_frame_turns_that_into_a_hold_where_the_jaws_are(self) -> None:
        """The other half: the axis answers, and the console holds *that* —
        not the target it would have derived a moment earlier."""
        bench = self.silent()
        bench.backend.fresh = True
        bench.backend.q_rad = LIMITS.to_rad(37.5)
        bench.frame(2)
        assert bench.loop.motion.state is MotionState.HOLD
        assert bench.loop.last_frame.position_mm == pytest.approx(37.5, abs=1e-6)
        assert bench.loop.last_frame.cmd_mm == pytest.approx(37.5, abs=1e-6)
        assert any("已读到位置 37.5 mm" in text for text in bench.signals.logs())

    def test_motion_is_refused_until_then(self) -> None:
        bench = self.silent()
        bench.send(cmd.MoveToMm(30.0))
        assert "尚未读到位置" in bench.signals.alerts()[-1]
        assert not [c for c in bench.backend.calls if c[0] == "stream_frame" and c[1][1]]

    def test_a_position_nobody_has_measured_is_not_published(self) -> None:
        """``None`` and not ``0.0``: zero is the closed stop, and a GUI drawing
        it would be drawing a place the jaws have never been."""
        bench = self.silent()
        assert bench.loop.last_frame.position_mm is None
        assert bench.loop.last_frame.as_dict()["position_mm"] is None

    def test_the_first_frame_ever_published_claims_no_position(self) -> None:
        """The frame the GUI is built on, before the loop has even connected —
        the same rule, at the moment there is nothing to measure at all."""
        assert Bench().loop.last_frame.position_mm is None

    def test_the_awaiting_state_does_not_outlive_the_enable_it_belonged_to(self) -> None:
        """Disabling and enabling again re-measures from scratch: a stale flag
        would hold the *new* session to a position the old one read."""
        bench = self.silent()
        assert bench.loop._awaiting_position
        bench.send(cmd.Disable())
        assert not bench.loop._awaiting_position

    def test_a_stop_before_a_measurement_does_not_invent_one(self) -> None:
        bench = self.silent()
        bench.send(cmd.Stop())
        assert bench.loop.motion.state is MotionState.RELEASE
        assert all(kp == 0.0 for kp, _kd, _tau in self.sent_gains(bench))

    def test_leaving_zero_gravity_before_a_measurement_does_not_invent_one(self) -> None:
        bench = self.silent()
        bench.send(cmd.SetZeroGravity(True))
        assert bench.loop.motion.state is MotionState.ZERO_G
        bench.send(cmd.SetZeroGravity(False))
        assert bench.loop.motion.state is MotionState.RELEASE


# ═══════════════════════════════════════════════════════════════════════════
# 使能 an axis a hand has moved while the drive was dark
# ═══════════════════════════════════════════════════════════════════════════
class TestEnablingAfterTheJawsWerePushedByHand:
    """使能 holds where the jaws *are*, not what the drive last said.

    A disabled drive stops answering, so the SDK goes on serving the angle from
    before the disable while a hand can push the jaws anywhere.  The worker's own
    frame count outlived the disable, so the next 使能 took the hold branch with
    that pre-disable pose, froze it, and commanded it with full position gain —
    driving the gripper back to where it had been pushed away from.  The display
    is drawn from the same reading, so the number the operator watched while they
    pushed was the number the hold then aimed at.
    """

    PUSHED_RAD = LIMITS.to_rad(52.5)

    def frames_since(self, bench: Bench, mark: int) -> list[tuple]:
        """Every frame the backend was sent after ``mark`` calls were made.

        The mark matters: the axis was *holding* before it was disabled, so a
        scan of the whole call log would find the pre-disable hold and read the
        bug into frames that predate it.
        """
        return [
            call[1] for call in bench.backend.calls[mark:] if call[0] == "stream_frame"
        ]

    def pushed(self) -> Bench:
        """Enabled and measured, then disabled and shoved a long way by hand."""
        bench = Bench()
        bench.bring_up()
        bench.tick(2)
        assert bench.loop.motion.state is MotionState.HOLD
        assert bench.loop.last_frame.position_mm == pytest.approx(60.0, abs=1e-6)
        bench.send(cmd.Disable())
        bench.backend.drag(self.PUSHED_RAD)
        bench.tick(2)
        return bench

    def test_a_dark_axis_claims_no_position(self) -> None:
        """``None``, so the display reads 「—」.  The drive's last angle describes
        where the jaws *were*, and drawing it as the current position is what
        made a stale pose look like a live one."""
        bench = self.pushed()
        assert bench.loop.last_frame.position_mm is None

    def test_the_next_enable_holds_where_the_hand_left_the_jaws(self) -> None:
        bench = self.pushed()
        bench.send(cmd.Enable())
        bench.tick(2)
        assert bench.loop.motion.state is MotionState.HOLD
        assert bench.loop.last_frame.cmd_mm == pytest.approx(52.5, abs=1e-6)

    def test_no_frame_ever_aims_at_the_pose_from_before_the_disable(self) -> None:
        """The regression itself, stated as the thing that hurt: not one frame
        may carry the old pose with a stiffness on it.  A hold freezes its target
        when it is entered, so a single such frame is enough to drive there."""
        bench = self.pushed()
        mark = len(bench.backend.calls)
        bench.send(cmd.Enable())
        bench.tick(4)
        aimed = [(q_rad, kp) for (q_rad, kp, *_rest) in self.frames_since(bench, mark) if kp]
        assert aimed, "使能后必须真的发出一个带增益的驻留帧"
        assert all(
            q_rad == pytest.approx(self.PUSHED_RAD) for q_rad, _kp in aimed
        ), "驻留帧指向了失能前的位姿"

    def test_a_re_enable_whose_link_stays_dark_holds_nothing(self) -> None:
        """The other way the same mistake lands: with no frame since this
        energisation there is no pose to hold, so the axis is left free.  The
        frames counted in the session before the disable are not evidence about
        this one, and holding on their strength would aim at that old pose."""
        bench = self.pushed()
        bench.backend.fresh = False
        mark = len(bench.backend.calls)
        bench.send(cmd.Enable())
        assert bench.loop.motion.state is MotionState.RELEASE
        assert all(kp == 0.0 for _q, kp, *_rest in self.frames_since(bench, mark))

    def test_the_console_says_where_it_is_holding(self) -> None:
        bench = self.pushed()
        bench.send(cmd.Enable())
        bench.tick(2)
        assert any("52.5 mm" in text for text in bench.signals.logs())


# ═══════════════════════════════════════════════════════════════════════════
# The CAN interface, prepared before the bus is opened
# ═══════════════════════════════════════════════════════════════════════════
class TestTheInterfaceIsPreparedBeforeConnecting:
    """连接 does this by itself now, and the property that makes that safe is
    that it is never the reason a connection fails."""

    def bench(self, link: FakeLink | None = None) -> Bench:
        bench = Bench(can_link=link if link is not None else FakeLink())
        bench.loop._can_link.backend = bench.backend
        return bench

    def test_the_interface_is_raised_before_the_sdk_opens_the_bus(self) -> None:
        """Order is the whole point: the SDK cannot open an interface that is
        down, so a bring-up that ran afterwards would be useless."""
        bench = self.bench()
        bench.send(cmd.Connect())
        assert bench.loop._can_link.calls == 1
        assert bench.loop._can_link.connected_when_asked is False
        assert "connect" in bench.backend.names()

    def test_a_connection_that_is_refused_asks_again_next_time(self) -> None:
        """It is per connect and not one-shot: an adapter unplugged at the first
        press is normally plugged back in for the second."""
        bench = self.bench()
        bench.send(cmd.Connect())
        bench.send(cmd.Disconnect())
        bench.send(cmd.Connect())
        assert bench.loop._can_link.calls == 2

    def test_the_operator_is_told_an_authorization_may_be_coming(self) -> None:
        """A password dialog appearing out of nowhere is alarming; a status line
        naming the interface first is not."""
        bench = self.bench()
        bench.send(cmd.Connect())
        busy = [text for _flag, text in bench.signals.of("busy")]
        assert any("can0" in text and "授权" in text for text in busy), busy

    @pytest.mark.parametrize(
        "problem",
        [
            LINK_MISSING,
            LINK_DENIED,
            LINK_FAILED,
        ],
    )
    def test_a_problem_with_the_interface_does_not_stop_the_connection(
        self, problem: str
    ) -> None:
        """The interface may legitimately be up already — managed by hand, by a
        unit file, or a virtual bus — and only the open attempt knows whether
        the link works.  A refused dialog must not take the console down with
        it, and it must not go unmentioned either."""
        link = FakeLink(LinkOutcome(problem, "can0 有点问题：这是给操作员看的说明"))
        bench = self.bench(link)

        bench.send(cmd.Connect())

        assert bench.loop.connected
        assert "connect" in bench.backend.names()
        assert "这是给操作员看的说明" in bench.signals.alerts()[-1]

    @pytest.mark.parametrize("state", [LINK_OK, LINK_FD])
    def test_nothing_that_might_still_work_interrupts_anybody(self, state: str) -> None:
        """The common case — the operator raised the interface themselves — and
        an FD bus, which may connect perfectly well.  Either read as a problem
        would turn the banner into something to ignore."""
        link = FakeLink(LinkOutcome(state, f"can0 的状态是「{state}」，未改动"))
        bench = self.bench(link)

        bench.send(cmd.Connect())

        assert bench.signals.alerts() == []
        logged = [text for level, text in bench.signals.of("log") if level == "info"]
        assert any("未改动" in text for text in logged), logged

    def test_the_bring_up_is_logged_at_the_level_it_deserves(self) -> None:
        """A privileged change that was just made is worth a line in the file
        even though it worked."""
        link = FakeLink(LinkOutcome(LINK_CONFIGURED, "can0 原为「未 up」，已配置"))
        bench = self.bench(link)

        bench.send(cmd.Connect())

        levels = {level for level, text in bench.signals.of("log") if "已配置" in text}
        assert levels == {"info"}, "working is not a warning"

    def test_an_ensure_that_raises_does_not_reach_the_connection(self) -> None:
        """Whatever this helper does, a failure of *it* is not a failure of the
        link.  The bug it is guarding against is a traceback out of a subprocess
        helper being read as "cannot connect"."""
        link = FakeLink(raises=RuntimeError("pkexec 不见了"))
        bench = self.bench(link)

        bench.send(cmd.Connect())

        assert bench.loop.connected
        assert any("仍会尝试连接" in text for text in bench.signals.alerts())

    def test_the_simulator_asks_for_nothing(self) -> None:
        """No interface exists behind it, and a password dialog for a bus that
        is not there would be theatre."""
        bench = Bench()
        bench.send(cmd.Connect())
        busy = [text for _flag, text in bench.signals.of("busy")]
        assert not any("授权" in text for text in busy), busy
        assert bench.loop.connected

    def test_a_password_dialog_may_outlast_the_watchdog(self) -> None:
        """The loop is blocked while the operator reads the dialog, and from
        inside that looks exactly like a dead GUI.

        What saves it is that the GUI is in fact alive and still queueing its
        heartbeats; they are drained on the next tick, before the watchdog is
        serviced again.  The ordering is invisible, so it is pinned here: the
        cost of getting it wrong is that the console kills the session the
        moment the operator authorizes it.
        """
        clock = FakeClock()
        signals = Recorder()
        backend = RecordingBackend()
        link = FakeLink()
        loop = WorkerLoop(
            backend,
            signals,
            clock=clock,
            sleep=lambda seconds: None,
            can_link=link,
        )

        def while_the_dialog_is_open() -> None:
            for _ in range(60):  # 30 s of reading it, at the GUI's 500 ms rate
                clock.advance(constants.HEARTBEAT_INTERVAL_MS / 1000.0)
                loop.submit(cmd.Heartbeat())

        link.on_ensure = while_the_dialog_is_open
        loop.submit(cmd.Connect())
        loop.tick_once(constants.CTRL_DT)   # the connect, and the dialog
        clock.advance(constants.CTRL_DT)
        loop.tick_once(constants.CTRL_DT)   # the heartbeats, then the watchdog

        assert loop.connected
        assert not loop.stopping
        assert not any("心跳" in text for text in signals.alerts())


# ═══════════════════════════════════════════════════════════════════════════
# Stopping
# ═══════════════════════════════════════════════════════════════════════════
class TestStopping:
    def test_stop_holds_with_no_torque(self) -> None:
        bench = Bench()
        bench.bring_up()
        bench.send(cmd.MoveToMm(30.0), count=10)
        bench.send(cmd.Stop())
        assert bench.loop.motion.state is MotionState.HOLD
        _q, _kp, _kd, _dq, tau, _u = [
            c[1] for c in bench.backend.calls if c[0] == "stream_frame"
        ][-1]
        assert tau == 0.0

    def test_release_goes_limp_and_stays_enabled(self) -> None:
        bench = Bench()
        bench.bring_up()
        frame = bench.send(cmd.Release())
        assert bench.loop.motion.state is MotionState.RELEASE
        assert bench.loop.enabled
        assert frame.cmd_mm is not None  # a frame is still being sent
        _q, kp, kd, _dq, tau, _u = [
            c[1] for c in bench.backend.calls if c[0] == "stream_frame"
        ][-1]
        assert (kp, kd, tau) == (0.0, 0.0, 0.0)

    def test_release_is_allowed_while_the_gate_is_shut(self) -> None:
        """Making the jaws limp is a safety action, not something to gate."""
        bench = Bench()
        bench.bring_up()
        bench.loop._info = CalibrationInfo(
            provenance=calibration.PROVENANCE_INVALID, limits=None, problems=("坏了",)
        )
        bench.send(cmd.Release())
        assert bench.loop.motion.state is MotionState.RELEASE

    def test_the_estop_zero_torques_and_disables(self) -> None:
        bench = Bench()
        bench.bring_up()
        bench.backend.calls.clear()
        bench.loop.estop("测试")
        bench.tick(2)
        commands = [name for name in bench.backend.names() if name != "poll"]
        assert commands[:2] == ["zero_torque", "disable"]
        assert not bench.loop.enabled
        assert bench.frame().motion_state == "ESTOP"

    def test_the_estop_latches_against_every_motion_command(self) -> None:
        """A latch, not a stop: the operator's finger may already be on the way
        to another button, and the answer to that must not be to move."""
        bench = Bench()
        bench.bring_up()
        bench.loop.estop("测试")
        bench.tick()
        bench.backend.calls.clear()

        for command in (cmd.MoveToMm(10.0), cmd.Open(), cmd.Close(), cmd.Grasp()):
            bench.send(command)
        assert not [c for c in bench.backend.calls if c[0] == "stream_frame"]
        assert all("急停" in text for text in bench.signals.alerts()[-4:])

    def test_the_estop_latches_against_enabling_too(self) -> None:
        """The bar blocks the button, but the worker has to block the command.

        ``_service_commands`` runs before the tick's E-stop check, so an 使能
        that was already in the queue when the latch went down would be
        serviced first — re-energising the axis the E-stop had just put down,
        while every frame after it was refused for being latched."""
        bench = Bench()
        bench.bring_up()
        bench.loop.estop("测试")
        bench.tick()
        assert not bench.loop.enabled
        bench.backend.calls.clear()

        bench.send(cmd.Enable())

        assert not bench.loop.enabled
        assert "enable" not in bench.backend.names()
        assert "急停" in bench.signals.alerts()[-1]

    def test_resetting_the_estop_does_not_re_enable(self) -> None:
        """Releasing the latch and energising the motor are two different
        decisions, and the operator has only made the first one."""
        bench = Bench()
        bench.bring_up()
        bench.loop.estop("测试")
        bench.tick()
        bench.send(cmd.ResetEStop())
        assert not bench.loop.estopped
        assert not bench.loop.enabled

    def test_the_estop_cannot_be_reset_onto_a_shut_gate(self) -> None:
        """Otherwise releasing the latch would undo the reason it latched."""
        bench = Bench()
        bench.bring_up()
        bench.loop.estop("测试")
        bench.tick()
        bench.loop._info = CalibrationInfo(
            provenance=calibration.PROVENANCE_INVALID, limits=None, problems=("坏了",)
        )
        bench.send(cmd.ResetEStop())
        assert bench.loop.estopped
        assert "无法复位急停" in bench.signals.alerts()[-1]


# ═══════════════════════════════════════════════════════════════════════════
# 放开: opening further than where the jaws are
# ═══════════════════════════════════════════════════════════════════════════
class TestOpeningFurtherToLetGo:
    """The release after a grasp, and the two poses it can be measured from.

    A grasp is :meth:`MotionFSM.grasp` — a move to 0 mm under a force cap — so
    its target is the *closed* end.  "The target plus ten millimetres" is
    therefore 10 mm, a command back into the object being held, and the jaws
    stopped somewhere the target never described.  Every test below is one
    consequence of measuring from the reading instead — except while a grip is
    being held, where the reading has moved on from the object: a held force has
    no position gain in it, so the jaws are driven into the object as the torque
    climbs to its setpoint, and ten millimetres from *there* is ten millimetres
    minus that draw-in of clearance around the object.
    """

    #: A compliant object: 20 N pushes the jaws ~9 mm into it, so a release
    #: measured from where they have got to spends most of its ten millimetres
    #: on the draw-in.  The default plant bends 0.2 mm.
    SOFT = 8.0
    SETPOINT_N = 20.0
    OBJECT_MM = 40.0

    def _held_at(self, mm: float) -> Bench:
        """A bench whose jaws are pinched at ``mm`` with the gate open."""
        bench = Bench()
        bench.backend.q_rad = LIMITS.to_rad(mm)
        bench.bring_up()
        return bench

    @staticmethod
    def _until(bench: Bench, state: MotionState, ticks: int = 4000) -> None:
        for _ in range(ticks):
            if bench.loop.motion.state is state:
                return
            bench.tick()
        raise AssertionError(f"未到达 {state.value}；当前 {bench.loop.motion.state.value}")

    def _grasp_and_settle(self, bench: Bench) -> float:
        """Grasp the injected object and hand back the pose the grip was made at.

        The jaws are opened first: the simulated unit starts near the closed
        stop, and an object injected there is a spring already compressed
        against it, which pushes the jaws open instead of being closed on.
        """
        bench.send(cmd.Open())
        self._until(bench, MotionState.HOLD)
        bench.backend.inject(obj_mm=self.OBJECT_MM)
        bench.send(cmd.Grasp(force_n=self.SETPOINT_N))
        self._until(bench, MotionState.HOLD_FORCE)
        bench.tick(300)  # the feed-forward ramps at FORCE_RAMP_N_S
        made_at = bench.loop.motion.grip_mm
        assert made_at is not None
        return made_at

    def _sent_q(self, bench: Bench) -> list[float]:
        return [
            q for name, args in bench.backend.calls if name == "stream_frame"
            for q in args[:1]
        ]

    def _commanded(self, bench: Bench) -> bool:
        """Whether any frame carried a position gain.

        A refused command still leaves the loop publishing — an enabled DM
        motor that is sent nothing is not a safe resting state — so "nothing was
        commanded" is the stiffness, not the silence.
        """
        return any(
            kp != 0.0
            for _q, kp, _kd, _dq, _tau, _probe in [
                args for name, args in bench.backend.calls if name == "stream_frame"
            ]
        )

    def test_it_opens_from_where_the_jaws_are(self) -> None:
        bench = Bench()
        bench.bring_up()
        bench.send(cmd.BackOff(), count=4)
        assert bench.loop.motion.state is MotionState.SERVO
        assert bench.loop.motion.last_command_mm == pytest.approx(
            (60.0 + constants.RELEASE_OPEN_MM), abs=1e-6
        )

    def test_it_measures_from_the_object_not_from_the_grasp_target(self) -> None:
        """The failure mode this command exists to avoid.

        The grasp's target is 0 mm and the object holds the jaws at 28 mm.  A
        release computed from the target would command 10 mm — into the object,
        through the gap it is not in — instead of 38 mm, which is clear of it.
        """
        bench = self._held_at(28.0)
        bench.send(cmd.Grasp(force_n=18.0), count=4)
        assert bench.loop.motion.last_command_mm == pytest.approx(0.0, abs=1e-6)

        bench.send(cmd.BackOff(), count=4)
        assert bench.loop.motion.last_command_mm == pytest.approx(38.0, abs=1e-6)

    def test_a_grip_that_drew_itself_in_is_measured_from_where_it_was_made(self) -> None:
        """The grip is made at 28 mm and the jaws are later at 19 mm.

        Nothing holds the pose once force mode starts — the frame carries no
        position gain — so the object's own stiffness decides where they come to
        rest, and a release from *there* would command 29 mm: ten millimetres of
        button, nine of them spent on the draw-in, and the jaws still on the
        object.
        """
        bench = self._held_at(28.0)
        bench.send(cmd.Grasp(force_n=18.0), count=30)
        assert bench.loop.motion.state is MotionState.HOLD_FORCE

        bench.backend.q_rad = LIMITS.to_rad(19.0)  # driven in by the setpoint
        bench.tick(4)
        assert bench.backend.read().position_mm == pytest.approx(19.0, abs=1e-6)

        bench.send(cmd.BackOff(), count=4)

        assert bench.loop.motion.last_command_mm == pytest.approx(38.0, abs=1e-6)
        assert any("已内收 9.0 mm" in line for line in bench.signals.logs()), (
            "how far it drew in is the number nobody can otherwise see"
        )

    def test_a_grip_pushed_out_is_measured_from_where_the_jaws_are(self) -> None:
        """The other sign of the same subtraction.

        The jaws can also be pushed *out* of the grip — by hand, or by an object
        that springs back under the load.  There the reading is the further end
        and the one to open from; the pose the grip was made at is behind it.
        """
        bench = self._held_at(28.0)
        bench.send(cmd.Grasp(force_n=18.0), count=30)

        bench.backend.q_rad = LIMITS.to_rad(31.0)
        bench.tick(4)

        bench.send(cmd.BackOff(), count=4)

        assert bench.loop.motion.last_command_mm == pytest.approx(41.0, abs=1e-6)

    def test_one_press_frees_a_grip_that_drew_itself_in(self) -> None:
        """The reported failure, end to end against the plant.

        A 20 N grasp of a compliant object draws the jaws ~9 mm in, and the
        release measured from where they are ends up *on* the object rather than
        clear of it: the operator presses again — 夹取 20 N, 放开, 放开, the
        shape of every cycle in the bench log of 2026-10-09.  Measured from the
        pose the grip was made at, one press is the ten millimetres of clearance
        the button has always claimed to be.
        """
        clock = FakeClock()
        bench = Bench(
            RecordingSim(config=PlantConfig(contact_k=self.SOFT), clock=clock), clock=clock
        )
        bench.bring_up()
        made_at = self._grasp_and_settle(bench)

        drawn_in = made_at - bench.backend.read().position_mm
        assert drawn_in > 5.0, f"the draw-in should be visible here: {drawn_in:.2f} mm"

        bench.send(cmd.BackOff())
        self._until(bench, MotionState.HOLD)

        released = bench.backend.read()
        assert released.position_mm >= self.OBJECT_MM + 5.0, "clear of the object"
        # Not zero: with the jaws off the object what the drive reports is its
        # own torque, which the plant's friction puts at 1–2 N.  What matters is
        # that it is nowhere near the 20 N the grip was holding with.
        assert abs(released.force_n) < 2.5, f"still squeezing at {released.force_n:.2f} N"

        # The arithmetic that got it there, against the physics: ten millimetres
        # from the pose the grip was made at, not from the draw-in.
        assert released.position_mm == pytest.approx(
            made_at + constants.RELEASE_OPEN_MM, abs=constants.TOL_MM
        )

    def test_a_release_at_the_top_of_the_travel_is_clamped_not_refused(self) -> None:
        """Nothing left to open: the move is short, and it is still a move.

        Refusing would leave the operator holding 停止 and 零重力 as the only
        answers at the one moment they are trying to put something down.
        """
        bench = self._held_at(LIMITS.max_stroke_mm)
        bench.backend.calls.clear()
        bench.send(cmd.BackOff(), count=4)

        assert bench.loop.motion.state is MotionState.SERVO
        assert bench.loop.motion.last_command_mm == LIMITS.max_stroke_mm
        sent = self._sent_q(bench)
        assert sent, "仍然要发帧，电机靠它保持使能"
        assert all(LIMITS.rad_low <= q <= LIMITS.rad_high for q in sent)

    def test_the_command_carries_the_distance(self) -> None:
        """The worker uses the delta it was handed, not the default: the button
        and the log have to describe the same move as the one that happens."""
        bench = self._held_at(20.0)
        bench.send(cmd.BackOff(delta_mm=3.0), count=4)
        assert bench.loop.motion.last_command_mm == pytest.approx(23.0, abs=1e-6)

    def test_a_release_is_refused_while_the_gate_is_shut(self) -> None:
        """It is a millimetre command, so it needs the travel like any other."""
        bench = Bench()
        bench.bring_up()
        bench.swap_calibration(BROKEN)
        assert bench.loop.gate is GateState.BLOCKED
        bench.backend.calls.clear()

        bench.send(cmd.BackOff(), count=2)

        assert bench.loop.motion.state is not MotionState.SERVO
        assert "被拒绝" in bench.signals.alerts()[-1]
        assert "放开" in bench.signals.alerts()[-1]
        assert not self._commanded(bench)

    def test_a_release_without_a_reading_is_refused(self) -> None:
        """No measurement, no arithmetic: the release says so rather than
        commanding a target derived from an angle nobody has read."""
        bench = Bench(RecordingBackend(fresh=False))
        bench.bring_up()
        bench.backend.calls.clear()

        bench.send(cmd.BackOff(), count=2)

        assert "尚未读到位置" in bench.signals.alerts()[-1]
        assert not self._commanded(bench)


class TestTheHandOverTheOperatorReads:
    """What the log says when a grip is made, and why the pose is not enough.

    A line carrying only a millimetre reading cannot be told from the case the
    operator hit: every grip in the bench log of 2026-10-09 was made 1.4–2.2 mm
    into the close and the release then opened all of that again, and nothing in
    the console said whether the jaws had met an object or were a mechanism that
    had not broken away from rest being read as one.  The channel and the
    distance travelled are those two readings, and they survive only here.
    """

    OBJECT_MM = 40.0
    SETPOINT_N = 20.0

    def _bench(self) -> Bench:
        clock = FakeClock()
        bench = Bench(RecordingSim(clock=clock), clock=clock)
        bench.bring_up()
        return bench

    def _grasp(self, bench: Bench, speed_mm_s: float, obj_mm: float | None) -> None:
        bench.loop.motion.set_speed(speed_mm_s)
        bench.send(cmd.Open())
        for _ in range(4000):
            if bench.loop.motion.state is MotionState.HOLD:
                break
            bench.tick()
        else:
            raise AssertionError(f"未张开到位；当前 {bench.loop.motion.state.value}")
        if obj_mm is not None:
            bench.backend.inject(obj_mm=obj_mm)
        bench.send(cmd.Grasp(force_n=self.SETPOINT_N))
        for _ in range(4000):
            if bench.loop.motion.state is MotionState.HOLD_FORCE:
                # The hand-over is logged by the publish that notices the
                # transition, and publishing runs at TELEMETRY_HZ against the
                # loop's own rate — so the state can turn over a few ticks
                # before anything is written.
                bench.tick(4)
                return
            bench.tick()
        raise AssertionError(f"未进入力保持；当前 {bench.loop.motion.state.value}")

    def _hand_over(self, bench: Bench) -> str:
        lines = [text for text in bench.signals.logs() if text.startswith("夹取：")]
        assert lines, f"没有交棒日志：{bench.signals.logs()}"
        return lines[-1]

    def test_a_grasp_names_the_object_it_met_and_how_far_it_travelled(self) -> None:
        bench = self._bench()

        self._grasp(bench, speed_mm_s=25.0, obj_mm=self.OBJECT_MM)

        grip = bench.loop.motion.grip_mm
        assert grip == pytest.approx(self.OBJECT_MM, abs=1.0)
        line = self._hand_over(bench)
        assert f"在 {grip:.1f} mm 交棒" in line
        # The jaws started at the open stop, so the travel is the whole close.
        assert "已驶出" in line, line
        assert "接触：丢失位移" in line, line

    def test_a_close_that_met_nothing_is_not_logged_as_a_contact(self) -> None:
        """The wrong answer here is worse than no line at all: it sends the
        operator looking for an obstruction that is not there."""
        bench = self._bench()

        self._grasp(bench, speed_mm_s=50.0, obj_mm=None)

        line = self._hand_over(bench)
        assert "接触：" not in line, line
        assert "到位：" in line, line


# ═══════════════════════════════════════════════════════════════════════════
# Calibration through the worker
# ═══════════════════════════════════════════════════════════════════════════
class TestCalibrationCommands:
    def test_a_probe_needs_an_enabled_motor(self) -> None:
        """Both start buttons, because this refusal is now the whole answer to
        a click: the page no longer greys them out on a motor that is off, so
        the operator gets no other signal that anything was missing."""
        bench = Bench()
        bench.send(cmd.Connect())
        bench.tick()
        for command in (cmd.StartGuidedCalibration(), cmd.StartManualCalibration()):
            bench.send(command)
            assert bench.loop.probe is None
            assert "使能" in bench.signals.alerts()[-1]

    def test_the_probe_drives_the_axis_with_an_ungated_frame(self) -> None:
        """The one motion in the application allowed before a calibration
        exists — it is what produces one."""
        bench = Bench()
        bench.bring_up()
        bench.send(cmd.StartGuidedCalibration())
        bench.backend.calls.clear()
        bench.tick()
        frames = [c[1] for c in bench.backend.calls if c[0] == "stream_frame"]
        assert frames and frames[-1][-1] is True

    def test_it_is_refused_when_the_estop_is_latched(self) -> None:
        bench = Bench()
        bench.bring_up()
        bench.loop.estop("测试")
        bench.tick()
        bench.send(cmd.StartGuidedCalibration())
        assert "急停" in bench.signals.alerts()[-1]

    def test_stop_cancels_an_active_probe(self) -> None:
        """A probe that goes on pressing a hard stop after the operator has
        pressed stop is not a behaviour anyone would defend."""
        bench = Bench()
        bench.bring_up()
        bench.send(cmd.StartGuidedCalibration())
        bench.tick()
        assert bench.loop.probe is not None

        bench.send(cmd.Stop())
        assert bench.loop.probe is None
        assert any("中断了正在进行的标定" in text for _l, text in bench.signals.of("log"))
        # The samples are discarded, and the operator has to be told: a cancel
        # that only reached the log would look like a calibration that worked.
        assert "标定未完成" in bench.signals.alerts()[-1]

    def test_release_also_cancels_an_active_probe(self) -> None:
        bench = Bench()
        bench.bring_up()
        bench.send(cmd.StartGuidedCalibration())
        bench.tick()
        bench.send(cmd.Release())
        assert bench.loop.probe is None

    def test_a_second_probe_is_refused_while_one_runs(self) -> None:
        bench = Bench()
        bench.bring_up()
        bench.send(cmd.StartGuidedCalibration())
        bench.tick()
        first = bench.loop.probe
        bench.send(cmd.StartGuidedCalibration())
        assert bench.loop.probe is first

    def test_the_probe_opens_the_way_this_console_does(self) -> None:
        """A guided probe runs in one direction and is told nothing about it.

        There is no file to read a direction from — the probe is running because
        there is none — so it goes toward the stop the units this console drives
        have at the smaller angle, and the travel it measures is what the file
        ends up describing.
        """
        bench = Bench()
        bench.bring_up()
        bench.send(cmd.StartGuidedCalibration())
        bench.tick()

        assert bench.loop.probe is not None
        assert bench.loop.probe.direction == -1.0
        assert not hasattr(bench.loop.probe, "reversed_mount")

    def test_confirm_reaches_the_guided_probe(self) -> None:
        bench = Bench()
        bench.bring_up()
        bench.send(cmd.StartGuidedCalibration())
        bench.tick()
        bench.send(cmd.ConfirmProbeLimit())
        assert bench.loop.probe.open_rad is not None

    def test_the_manual_probe_runs_limp(self) -> None:
        """Zero torque is the whole mechanism: the operator drives the jaws."""
        bench = Bench()
        bench.bring_up()
        bench.send(cmd.StartManualCalibration())
        bench.tick()
        assert bench.loop.probe is not None
        _q, kp, kd, _dq, tau, _u = [
            c[1] for c in bench.backend.calls if c[0] == "stream_frame"
        ][-1]
        assert (kp, kd, tau) == (0.0, 0.0, 0.0)

    def test_starting_a_manual_probe_enters_zero_gravity(self) -> None:
        """The wizard asks the operator to push the jaws, so the axis has to be
        free *and say so*: a 记录 button that works while the page looks like it
        is holding a position is a wizard nobody trusts."""
        bench = Bench()
        bench.bring_up()
        bench.send(cmd.StartManualCalibration())
        bench.tick()

        assert bench.loop.motion.state is MotionState.ZERO_G
        assert bench.loop.motion.source == "开始手动标定"

    def test_starting_a_guided_probe_does_not_enter_zero_gravity(self) -> None:
        """A guided probe drives the jaws into the stops itself.  Freeing the
        axis would take away the only thing doing the driving."""
        bench = Bench()
        bench.bring_up()
        bench.send(cmd.StartGuidedCalibration())
        bench.tick()

        assert bench.loop.motion.state is not MotionState.ZERO_G

    def test_finishing_a_manual_probe_leaves_zero_gravity_and_holds_the_angle(
        self,
    ) -> None:
        """The operator is holding the jaws for the whole probe, so the console
        has to take them back at the end — and with the write broken, the only
        thing it can name is the angle the encoder reports."""
        bench = Bench(RecordingBackend(fail=("save_calibration",)))
        bench.bring_up()
        bench.send(cmd.StartManualCalibration())
        bench.tick()
        assert bench.loop.motion.state is MotionState.ZERO_G

        bench.drive_to(120.0)
        bench.send(cmd.RecordOpenLimit())
        bench.drive_to(0.0)
        bench.send(cmd.RecordCloseLimit())

        assert bench.loop.motion.state is MotionState.HOLD_RAD
        logged = [text for _level, text in bench.signals.of("log")]
        assert any("已退出零重力" in text for text in logged)

    def test_a_cancelled_manual_probe_also_leaves_zero_gravity(self) -> None:
        """Cancelling is the way out an operator reaches for when something is
        wrong, which makes it the worst possible time to leave the axis free."""
        bench = Bench()
        bench.bring_up()
        bench.send(cmd.StartManualCalibration())
        bench.tick()
        assert bench.loop.motion.state is MotionState.ZERO_G

        bench.send(cmd.CancelCalibration())
        bench.run_until_probe_finishes()

        assert bench.loop.probe is None
        assert bench.loop.motion.state is not MotionState.ZERO_G
        bench.backend.calls.clear()
        bench.tick(3)
        assert len([c for c in bench.backend.calls if c[0] == "stream_frame"]) == 3

    def test_the_two_labelled_points_finish_the_probe(self) -> None:
        """The reported flow end to end, at the worker's own level: the
        operator works the jaws to the open extreme, records it, works them back
        to the closed one, records that — and is asked whether to keep the
        result.

        The question is the one thing the second press does not answer.  The
        result is measured either way; whether it should govern the machine is
        decided while it is still in memory and the operator is looking at it,
        and nothing is written until they say yes."""
        bench = Bench()
        bench.bring_up()
        bench.send(cmd.StartManualCalibration())
        bench.tick()
        assert bench.loop.probe is not None

        bench.drive_to(120.0)
        bench.send(cmd.RecordOpenLimit())
        assert bench.loop.probe is not None, "one point is not a calibration"

        bench.drive_to(0.0)
        bench.send(cmd.RecordCloseLimit())

        assert bench.loop.probe is None, "the second point should finish it"
        assert "set_calibration_memory" in bench.backend.names()
        assert "save_calibration" not in bench.backend.names(), "not without an answer"
        assert bench.loop.info.provenance == calibration.PROVENANCE_MEMORY
        assert bench.loop.gate is GateState.BLOCKED
        assert bench.signals.has("calib_ready_to_apply")

        bench.send(cmd.ApplyCalibration())

        assert "save_calibration" in bench.backend.names()
        assert ("save_calibration", (None,)) in bench.backend.calls
        assert bench.loop.info.provenance == calibration.PROVENANCE_USER
        # Open, and held in millimetres: the file now carries the result, so the
        # numbers are the ones the console will run on and a command derived
        # from them is exactly what the axis should be given.
        assert bench.loop.gate is GateState.READY
        assert bench.loop.motion.state is MotionState.HOLD
        assert "标定已保存到" in bench.signals.alerts()[-1]

    def test_answering_no_leaves_the_result_in_memory_and_the_gate_shut(self) -> None:
        """暂不 is not a cancel: the measurement stands, the page goes on showing
        it, and what it does not get is authority.  The axis is left holding the
        pose the probe ended on, which is the same treatment an unwritten result
        has always had."""
        bench = Bench()
        bench.bring_up()
        bench.send(cmd.StartManualCalibration())
        bench.tick()
        bench.drive_to(120.0)
        bench.send(cmd.RecordOpenLimit())
        bench.drive_to(0.0)
        bench.send(cmd.RecordCloseLimit())

        bench.send(cmd.DiscardCalibration())

        assert "save_calibration" not in bench.backend.names()
        assert bench.loop.info.provenance == calibration.PROVENANCE_MEMORY
        assert bench.loop.gate is GateState.BLOCKED
        assert bench.loop.motion.state is MotionState.HOLD_RAD
        assert "未应用" in bench.signals.logs()[-1]

    def test_finishing_a_manual_probe_writes_the_probed_angles(self) -> None:
        """What lands on disk is what the probe just measured.

        The bug this pins down is a re-parse of the default user file at the
        moment the probe finished, which quietly replaced the angles the
        operator had just worked the jaws through with the ones already on disk
        — and the save then wrote that older file back, so pressing 保存 looked
        like it had done nothing at all.  Two things say it cannot come back:
        the limits in force after the probe are the probed angles, and the write
        is handed no path, so nothing in the loop is in a position to reload a
        file over the result on the way to the disk.
        """
        bench = Bench()
        bench.bring_up()
        bench.send(cmd.StartManualCalibration())
        bench.tick()
        bench.drive_to(120.0)
        bench.send(cmd.RecordOpenLimit())
        bench.drive_to(0.0)
        bench.send(cmd.RecordCloseLimit())

        recorded = [
            c[1] for c in bench.backend.calls if c[0] == "set_calibration_memory"
        ][-1]
        close_rad, open_rad = recorded[0], recorded[1]
        limits = bench.loop.info.limits
        assert limits is not None
        assert limits.closed_rad == pytest.approx(close_rad)
        assert limits.open_rad == pytest.approx(open_rad)

        bench.send(cmd.ApplyCalibration())

        assert ("save_calibration", (None,)) in bench.backend.calls
        assert bench.loop.info.provenance == calibration.PROVENANCE_USER

    def test_a_probe_result_that_fails_validation_is_never_offered(self) -> None:
        """A result the console has just called unusable is not put to the
        operator at all.

        The probe's own check only catches a degenerate travel, so a pair of
        readings that produces an absurd scale gets this far — and the file the
        result would replace is a working calibration.  Asking would be worse
        than useless: the honest answer is unknown to the operator, and 应用标定
        would be sitting there inviting them to press it.
        """
        bench = Bench()
        bench.bring_up()
        bench.send(cmd.StartManualCalibration())
        bench.tick()
        bench.drive_to(60.0)
        bench.send(cmd.RecordOpenLimit())
        bench.drive_to(60.2)
        bench.send(cmd.RecordCloseLimit())

        assert bench.loop.probe is None, "the two points were still taken"
        assert "set_calibration_memory" in bench.backend.names()
        assert not bench.signals.has("calib_ready_to_apply")
        assert "save_calibration" not in bench.backend.names()
        assert bench.loop.info.provenance == calibration.PROVENANCE_MEMORY
        assert bench.loop.gate is GateState.BLOCKED
        alert = bench.signals.alerts()[-1]
        assert "未通过校验" in alert
        assert "合理范围" in alert

    def test_applying_a_result_that_cannot_be_written_asks_again(self) -> None:
        """A failed write is the one case 应用标定 exists as a retry for, so the
        prompt has to come back rather than the console reporting a dead end:
        the result is still in memory and still usable, and the operator has no
        other way to reach it."""
        bench = Bench(RecordingBackend(fail=("save_calibration",)))
        bench.bring_up()
        bench.send(cmd.StartManualCalibration())
        bench.tick()
        bench.drive_to(120.0)
        bench.send(cmd.RecordOpenLimit())
        bench.drive_to(0.0)
        bench.send(cmd.RecordCloseLimit())
        assert len(bench.signals.of("calib_ready_to_apply")) == 1

        bench.send(cmd.ApplyCalibration())

        assert "保存标定失败" in bench.signals.alerts()[-1]
        assert len(bench.signals.of("calib_ready_to_apply")) == 2
        assert bench.loop.gate is GateState.BLOCKED

    def test_applying_a_calibration_says_where_it_landed(self) -> None:
        """The GUI remembers this path, and it is the whole mechanism by which an
        applied calibration survives a restart — a file nobody named is not the
        file in effect."""
        bench = Bench()
        bench.bring_up()
        bench.send(cmd.StartManualCalibration())
        bench.tick()
        bench.drive_to(120.0)
        bench.send(cmd.RecordOpenLimit())
        bench.drive_to(0.0)
        bench.send(cmd.RecordCloseLimit())

        bench.send(cmd.ApplyCalibration())

        (path,) = bench.signals.first("calibration_applied")
        assert path == "/tmp/cal.json"
        assert bench.loop.info.path == path

    def test_a_record_without_a_manual_probe_is_refused(self) -> None:
        bench = Bench()
        bench.bring_up()
        bench.send(cmd.RecordOpenLimit())
        assert "没有正在进行的手动标定" in bench.signals.alerts()[-1]

    def test_a_record_out_of_turn_is_refused_rather_than_relabelled(self) -> None:
        """The press carries the label, and the label is what decides which end
        is 0 mm — so a press that arrives during the other step must not be
        taken as the point that happens to be due.  A calibration built from the
        two angles swapped passes every check the console makes and drives the
        gripper backwards."""
        bench = Bench()
        bench.bring_up()
        bench.send(cmd.StartManualCalibration())
        bench.tick()

        bench.drive_to(120.0)
        bench.send(cmd.RecordCloseLimit())

        probe = bench.loop.probe
        assert probe is not None
        assert probe.close_rad is None and probe.open_rad is None
        assert "已忽略这次按键" in bench.signals.of("log")[-1][1]

    def test_a_probe_that_captured_no_range_is_reported_not_adopted(self) -> None:
        """Both buttons pressed in the same place: a "calibration" whose every
        millimetre is one angle.  It must be reported, not adopted."""
        bench = Bench()
        bench.bring_up()
        bench.send(cmd.StartManualCalibration())
        bench.tick()

        bench.drive_to(60.0)
        bench.send(cmd.RecordOpenLimit())
        bench.send(cmd.RecordCloseLimit())
        bench.run_until_probe_finishes()

        assert "set_calibration_memory" not in bench.backend.names()
        assert bench.loop.info.provenance == calibration.PROVENANCE_USER
        assert "标定未完成" in bench.signals.alerts()[-1]

    def test_saving_an_adopted_probe_opens_the_gate(self) -> None:
        bench = Bench()
        bench.bring_up()
        bench.swap_calibration(calibration.in_memory(CLOSED_RAD, OPEN_RAD, 65.21, 120.0))
        assert bench.loop.gate is GateState.BLOCKED

        # Saving is what promotes it: the file is the only thing that survives.
        bench.swap_calibration(USER_CAL)
        assert bench.loop.gate is GateState.READY
        assert bench.loop.motion.state is MotionState.HOLD


class TestRefreshingTheDisplay:
    """刷新显示 re-reads the file in force.  It never picks a different one.

    The button next to it, 载入文件…, is how the operator changes which file is in
    effect.  If refreshing silently resolved the default instead, a file that had
    been renamed would come back as the factory numbers: the page would look
    refreshed, and the console would be running on another gripper's calibration.
    That is the whole failure this class pins down.
    """

    def test_it_re_reads_the_file_named_by_the_calibration_in_force(self, tmp_path) -> None:
        target = tmp_path / "bench.json"
        target.write_text("{}", encoding="utf-8")
        info = calibration.CalibrationInfo(
            provenance=calibration.PROVENANCE_USER, limits=LIMITS, path=str(target)
        )
        bench = Bench(RecordingBackend(info=info))
        bench.bring_up()
        bench.backend.calls.clear()

        bench.send(cmd.RefreshCalibration())

        assert ("load_calibration", (str(target),)) in bench.backend.calls
        assert bench.loop.info.provenance == calibration.PROVENANCE_USER

    def test_a_file_that_has_gone_is_reported_rather_than_replaced(self, tmp_path) -> None:
        """The report matters more than the repaint: the numbers on screen are
        still the ones that were read, and pretending otherwise is how the
        console ends up measuring with a calibration nobody chose."""
        gone = tmp_path / "gone.json"
        bench = Bench(RecordingBackend(info=calibration.CalibrationInfo(
            provenance=calibration.PROVENANCE_USER, limits=LIMITS, path=str(gone)
        )))
        bench.bring_up()
        bench.backend.calls.clear()

        bench.send(cmd.RefreshCalibration())

        assert "load_calibration" not in bench.backend.names()
        assert str(gone) in bench.signals.alerts()[-1]
        assert bench.loop.info.provenance == calibration.PROVENANCE_USER

    def test_a_probe_result_in_memory_is_republished_and_nothing_is_read(self) -> None:
        bench = Bench()
        bench.bring_up()
        bench.swap_calibration(calibration.in_memory(CLOSED_RAD, OPEN_RAD, 65.21, 120.0))
        bench.backend.calls.clear()
        before = len(bench.signals.of("calib_info"))

        bench.send(cmd.RefreshCalibration())

        assert "load_calibration" not in bench.backend.names()
        assert len(bench.signals.of("calib_info")) > before
        assert bench.loop.info.provenance == calibration.PROVENANCE_MEMORY


class TestTheAxisIsHandedBackAfterAProbe:
    """An enabled motor must keep being told what to do, and a probe is where
    it is easiest to stop telling it.

    The last thing a probe does is press a hard stop at the rated force, so the
    axe's own release is a real event — and if the loop drops to IDLE at that
    moment it sends no frame at all, which leaves the drive acting on whatever
    it was last given and the fingers free to go wherever the mechanism's springs
    push them.  On the real gripper that is a pop open and a fault light, right
    as the operator is looking at the wizard that just said 标定完成.

    Every finished probe is now that run.  A probe writes nothing on its own —
    the write is the answer to 应用标定 — so at the moment the wizard says 标定完成
    the console is still gated by construction, and the hand-back has to be
    honest about it.  The runs that *do* end up saved are held in millimetres
    instead — see
    ``TestCalibrationCommands::test_the_two_labelled_points_finish_the_probe``.
    """

    def _finished_manual_probe(self, fail: tuple[str, ...] = ()) -> Bench:
        """A manual probe run to the end, with the write optionally broken.

        The write is not part of the probe any more, so what is left here is the
        state the operator is in the moment the probe stops: a result in memory,
        a shut gate, and an axis that must not be dropped.
        """
        bench = Bench(RecordingBackend(fail=fail))
        bench.bring_up()
        bench.send(cmd.StartManualCalibration())
        bench.tick()
        bench.drive_to(120.0)
        bench.send(cmd.RecordOpenLimit())
        bench.drive_to(0.0)
        bench.send(cmd.RecordCloseLimit())
        assert bench.loop.probe is None
        return bench

    def test_it_holds_the_pose_the_probe_ended_on(self) -> None:
        bench = self._finished_manual_probe()
        assert bench.loop.motion.state is MotionState.HOLD_RAD

        # Commanded, not abandoned: one frame per tick, all of them at the angle
        # the probe ended on, at the move gain and with no feed-forward.
        ended = bench.backend.q_rad
        bench.backend.calls.clear()
        bench.tick(20)
        frames = [c[1] for c in bench.backend.calls if c[0] == "stream_frame"]
        assert len(frames) == 20
        assert all(f[0] == pytest.approx(ended) for f in frames)
        assert all(f[1] == constants.KP_MOVE for f in frames)
        assert all(f[4] == 0.0 for f in frames)

    def test_it_does_not_zero_torque_the_axis_on_the_way_out(self) -> None:
        """The withdrawal this replaced: zero torque once, then silence."""
        bench = self._finished_manual_probe()
        bench.backend.calls.clear()
        bench.tick(20)

        assert "zero_torque" not in bench.backend.names()
        assert bench.loop.motion.state is MotionState.HOLD_RAD

    def test_the_hold_is_not_a_millimetre_command(self) -> None:
        """Nothing about it is derived from the limits that are still in doubt,
        which is what makes it safe to hold through a shut gate.  Published as
        no command at all, so the UI does not claim a target the operator never
        set."""
        bench = self._finished_manual_probe()
        assert bench.frame().cmd_mm is None

    def test_a_write_that_fails_says_the_result_is_still_unsaved(self) -> None:
        """The retry has to be worth pressing, which means the operator has to
        be told the file did not take the result — otherwise the console looks
        like it saved and the gate staying shut looks like a fault."""
        bench = self._finished_manual_probe(fail=("save_calibration",))

        bench.send(cmd.ApplyCalibration())

        assert "保存标定失败" in bench.signals.alerts()[-1]
        assert bench.loop.gate is GateState.BLOCKED
        assert bench.loop.info.provenance == calibration.PROVENANCE_MEMORY

    def test_a_probe_that_never_saw_a_reading_hands_back_a_free_axis(self) -> None:
        """With no reading there is no pose to hold to, and guessing one is
        worse than it sounds: the SDK's cached angle is 0.0 rad until a status
        frame arrives, and 0.0 rad is somewhere inside any real travel — a
        plausible-looking number the jaws have never been at.  Zero stiffness
        asks nothing of a position nobody has measured, and it still keeps the
        frames going, which is what the axis needs to stay under control."""
        bench = Bench(RecordingBackend(fresh=False))
        bench.bring_up()
        bench.swap_calibration(BROKEN)
        bench.send(cmd.StartGuidedCalibration())
        bench.run_until_probe_finishes()

        assert bench.loop.motion.state is MotionState.RELEASE
        bench.backend.calls.clear()
        bench.tick(3)
        frames = [c[1] for c in bench.backend.calls if c[0] == "stream_frame"]
        assert len(frames) == 3, "an enabled axis is still talked to"
        assert all(f[1] == 0.0 and f[2] == 0.0 and f[4] == 0.0 for f in frames)

    def test_a_guided_probe_against_the_plant_also_ends_held(self) -> None:
        """The reported flow, end to end: a real guided probe against the
        simulated mechanism, which presses both stops and stops on one — and,
        once the operator accepts the result, holds the axis on the file's own
        numbers rather than on the angle alone.

        The middle step is the one that changed: between the probe ending and
        the answer to 应用标定 the console is gated, and the axis is held at the
        angle the probe stopped on.  Nothing here asserts a save happens on its
        own, because nothing does.
        """
        clock = FakeClock()
        bench = Bench(RecordingSim(clock=clock), clock=clock)
        bench.send(cmd.Connect())
        bench.send(cmd.Enable())
        bench.send(cmd.StartGuidedCalibration())
        bench.run_until_probe_finishes(timeout_s=40.0)

        assert bench.loop.info.provenance == calibration.PROVENANCE_MEMORY
        assert bench.loop.gate is GateState.BLOCKED
        assert bench.loop.motion.state is MotionState.HOLD_RAD

        bench.send(cmd.ApplyCalibration())

        assert bench.loop.info.provenance == calibration.PROVENANCE_USER
        assert bench.loop.gate is GateState.READY
        assert bench.loop.motion.state is MotionState.HOLD

        backend = bench.backend
        backend.frames.clear()
        backend.zeroes = 0
        bench.tick(20)
        assert len(backend.frames) == 20
        # One angle, held: the closed limit the probe just recorded, which is
        # where the guided probe ends and what the file now says 0 mm is.
        ended = backend.frames[0][0]
        assert all(f[0] == ended for f in backend.frames)
        assert ended == pytest.approx(bench.loop.info.limits.rad_high, abs=1e-4)
        assert all(f[1] == constants.KP_MOVE for f in backend.frames)
        assert all(f[4] == 0.0 for f in backend.frames)
        assert backend.zeroes == 0


class TestAProbeThatIsMovingNothing:
    """A probe records a limit when the angle it reads stops changing.

    That is the same signal for "the jaws are against a stop", for "no frame is
    reaching the motor", and for "the feedback has gone silent" — and when it is
    one of the last two, the probe records *both* limits wherever the jaws
    happen to be sitting and reports a degenerate travel.  A real run did
    exactly that and the operator was told "行程异常: 闭合 -1.370650 rad 未大于
    张开 -1.370650 rad" by a probe that had not moved the axis at all; nothing in
    that message distinguished it from a gripper whose two stops coincide.

    Only the caller can tell them apart, because only the caller knows what the
    bus did.  These tests pin all three to being told apart.
    """

    def test_refused_frames_stop_the_probe_instead_of_inventing_a_limit(self) -> None:
        bench = Bench()
        bench.bring_up()
        # The motor drops out from under the loop: the worker still believes it
        # is enabled, and every frame is refused at the backend from here on.
        bench.backend.enabled = False

        bench.send(cmd.StartGuidedCalibration())
        bench.run_until_probe_finishes()

        assert bench.loop.probe is None
        assert "set_calibration_memory" not in bench.backend.names()
        alert = bench.signals.alerts()[-1]
        assert "标定未完成" in alert
        assert "位置帧" in alert, alert
        assert "行程异常" not in alert

    def test_a_link_that_goes_quiet_stops_the_probe(self) -> None:
        """The case a refused-frame check cannot see: the frames leave, the
        answers stop coming, and the probe steers by a reading that is frozen.
        """
        bench = Bench()
        bench.bring_up()
        bench.send(cmd.StartGuidedCalibration())

        # Let the open phase finish: with the axis never moving, the stall
        # detector records the open limit one stall window in.
        deadline = int(3.0 / constants.CTRL_DT)
        for _ in range(deadline):
            probe = bench.loop.probe
            if probe is None or probe.open_rad is not None:
                break
            bench.tick()
        probe = bench.loop.probe
        assert probe is not None and probe.open_rad is not None, "张开极限没有被记录"

        bench.backend.fresh = False
        bench.run_until_probe_finishes()

        assert "set_calibration_memory" not in bench.backend.names()
        alert = bench.signals.alerts()[-1]
        assert "标定未完成" in alert and "状态帧" in alert, alert
        assert "行程异常" not in alert

    def test_a_failed_probe_still_logs_which_limit_it_decided_and_why(self) -> None:
        """On a failed probe these lines are the only account of where it got
        to.  They used to be logged on success only, which is exactly backwards:
        the operator reading them is the one whose calibration did not finish.
        """
        bench = Bench()
        bench.bring_up()
        bench.send(cmd.StartGuidedCalibration())
        for _ in range(int(3.0 / constants.CTRL_DT)):
            probe = bench.loop.probe
            if probe is None or probe.open_rad is not None:
                break
            bench.tick()
        assert bench.loop.probe is not None

        bench.backend.fresh = False
        bench.run_until_probe_finishes()

        lines = [text for _l, text in bench.signals.of("log")]
        assert any("张开极限" in text for text in lines), lines
        assert any("步未移动" in text or "最大步数" in text for text in lines), lines


# ═══════════════════════════════════════════════════════════════════════════
# The thread
# ═══════════════════════════════════════════════════════════════════════════
class TestTheThread:
    def test_the_backend_is_claimed_by_the_thread_that_drives_it(self) -> None:
        """The SDK is not thread-safe, and this turns the race into a failure
        that reproduces every time instead of one that interleaves frames."""
        bench = Bench()
        bench.tick()
        assert bench.backend.owner_tid == threading.get_ident()

        errors: list[BaseException] = []

        def intrude() -> None:
            try:
                bench.backend.read()
            except BaseException as exc:  # noqa: BLE001
                errors.append(exc)

        thread = threading.Thread(target=intrude)
        thread.start()
        thread.join(2.0)
        assert len(errors) == 1
        assert isinstance(errors[0], RuntimeError)
        assert "two threads" in str(errors[0])

    def test_shutdown_returns_and_the_teardown_order_is_safe(self) -> None:
        """The order is the whole point: a disabled DM motor coasts, and the
        frames still in flight must not be a position command when that
        happens."""
        backend = RecordingBackend()
        worker = GripperWorker(backend, watchdog_s=None)
        thread = threading.Thread(target=worker.run)
        thread.start()

        worker.submit(cmd.Connect())
        worker.submit(cmd.Enable())
        deadline = time.monotonic() + 2.0
        while not worker.enabled and time.monotonic() < deadline:
            time.sleep(0.005)
        assert worker.enabled

        backend.calls.clear()
        started = time.monotonic()
        assert worker.shutdown(timeout_ms=2000)
        assert time.monotonic() - started < 2.0

        thread.join(2.0)
        assert not thread.is_alive()

        names = backend.names()
        assert "zero_torque" in names and "disable" in names and "disconnect" in names
        assert names.index("zero_torque") < names.index("disable") < names.index("disconnect")
        assert not worker.connected

    def test_the_teardown_runs_once_however_often_it_is_called(self) -> None:
        backend = RecordingBackend()
        worker = GripperWorker(backend, watchdog_s=None)
        worker.loop._connected = True
        worker.loop._enabled = True
        worker.teardown()
        worker.teardown()
        assert backend.names().count("disconnect") == 1

    def test_an_exception_in_a_tick_does_not_abort_the_process(self) -> None:
        """An exception escaping QThread.run aborts the process, which is
        exactly what the teardown exists to prevent: the motor would be left
        enabled with the last frame it was given."""
        backend = RecordingBackend()
        signals = Recorder()
        loop = WorkerLoop(backend, signals, watchdog_s=None)

        # The setup runs through the queue rather than through tick_once: the
        # backend binds itself to the thread that first touches it (that is what
        # keeps the SDK single-threaded), so a setup tick here would poison it
        # for the thread about to run the loop.
        loop.submit(cmd.Connect())
        loop.submit(cmd.Enable())
        thread = threading.Thread(target=loop.run)
        thread.start()
        assert _wait_for(lambda: loop.enabled, timeout_s=2.0), "the motor never enabled"

        # Armed only now, and only for the frames the loop sends after this
        # point: enabling itself succeeded, so what fails is a hold frame.
        backend.fail.add("stream_frame")
        backend.calls.clear()
        thread.join(5.0)

        assert not thread.is_alive(), "the loop should have given up and exited"
        levels = [level for level, _text in signals.of("log")]
        assert "fatal" in levels
        assert any("tick 出错" in text for text in (t for _l, t in signals.of("log")))
        assert not loop.enabled
        # The teardown ran, and in the order that leaves the motor harmless: a
        # disabled DM motor coasts, so the last frame in flight had to be zero
        # torque before the disable went out.
        names = backend.names()
        assert names[-1] == "disconnect"
        assert names.index("zero_torque") < names.index("disable") < names.index("disconnect")

    def test_the_loop_ends_promptly_when_asked(self) -> None:
        backend = RecordingBackend()
        loop = WorkerLoop(backend, Recorder(), clock=time.monotonic, watchdog_s=None)
        thread = threading.Thread(target=loop.run)
        thread.start()
        loop.shutdown()
        thread.join(1.0)
        assert not thread.is_alive()

    def test_the_worker_exposes_everything_the_gui_needs_to_paint(self) -> None:
        """The first paint happens before the loop has ticked, so these have to
        answer without one."""
        worker = GripperWorker(RecordingBackend(), watchdog_s=None)
        assert worker.gate is None
        assert worker.info is None
        assert not worker.connected
        assert not worker.enabled
        assert not worker.estopped
        assert worker.loop.last_frame.motion_state == MotionState.IDLE.value


class TestAReadingThatHasGoneStale:
    """The loop marks the *reading*, not the link.

    ``LINK_STALE_MS`` is where the axis is treated as gone; a link that misses a
    few frames is not that.  But the position in hand is a cache — the SDK's
    ``get_state`` returns whatever the last poll brought — and a cache accumulates
    lost motion at the reference speed while the jaws are travelling perfectly
    well.  The state machine cannot tell: the frozen frame reports no velocity
    either, so even the veto that separates "lagging" from "held" reads it as a
    standstill.  So the loop, which knows when the last frame arrived, tells it.
    """

    def test_a_few_missed_frames_are_not_an_obstruction(self) -> None:
        bench = Bench()
        bench.bring_up()
        bench.send(cmd.SetSpeed(speed_mm_s=20.0))
        bench.send(cmd.MoveToMm(30.0), count=2)
        assert bench.loop.motion.state is MotionState.SERVO

        # The frames stop; what the loop is reading does not change.  A hundred
        # milliseconds of that is well short of a dead link, and long enough for
        # a frozen position to have "lost" a millimetre at this speed.
        bench.backend.fresh = False
        bench.tick(30)

        assert "堵转" not in bench.loop.motion.note
        assert bench.loop.motion.state is MotionState.SERVO


# ═══════════════════════════════════════════════════════════════════════════
# Alerts: what the operator is told, and what survives the window
# ═══════════════════════════════════════════════════════════════════════════
class TestAlertsReachTheLogFile:
    """An alert is a widget message; the log file is the record.

    The alerts worth having are the ones explaining a refusal, and until these
    existed the only account of one was on screen — so it went away with the
    window, and an operator reporting 「标定不可用」 had nothing to attach to
    the report.  The messages were reaching the file only by accident, through
    whichever of them happened to also be written as a log line at a level the
    file keeps.
    """

    def test_an_error_alert_is_written_as_an_error(self, caplog) -> None:
        bench = Bench()

        with caplog.at_level(logging.ERROR, logger="litegrip_studio.core.worker"):
            bench.loop._alert("error", "标定不可用：推导出的rad超出合理范围")

        records = [r for r in caplog.records if "超出合理范围" in r.getMessage()]
        assert [r.levelname for r in records] == ["ERROR"]

    def test_a_warning_alert_is_written_as_a_warning(self, caplog) -> None:
        bench = Bench()

        with caplog.at_level(logging.WARNING, logger="litegrip_studio.core.worker"):
            bench.loop._alert("warn", "结果未自动保存")

        records = [r for r in caplog.records if "未自动保存" in r.getMessage()]
        assert [r.levelname for r in records] == ["WARNING"]

    def test_an_info_alert_is_not_demoted_to_debug(self, caplog) -> None:
        """The one level where this deliberately departs from ``_log``: an
        alert the operator was shown is a record, not running commentary."""
        bench = Bench()

        with caplog.at_level(logging.INFO, logger="litegrip_studio.core.worker"):
            bench.loop._alert("info", "标定已保存到 /tmp/cal.json")

        records = [r for r in caplog.records if "标定已保存到" in r.getMessage()]
        assert [r.levelname for r in records] == ["INFO"]

    def test_an_unusable_calibration_says_so_in_the_log(self, caplog, tmp_path) -> None:
        """The reported symptom, end to end: the console refuses to move, and
        the reason has to be findable afterwards.

        The file is the shape of the one that did it — a recorded span too short
        for the travel it claims — and it goes through ``resolve``, so the text
        asserted here is the text ``validate_limits`` produces rather than one
        written into the test.
        """
        broken = tmp_path / "tiny.json"
        broken.write_text(
            json.dumps(
                {
                    "zero_position_rad": 1.0,
                    "max_position_rad": 0.996,
                    "rad_to_mm": 21500.0,
                }
            ),
            encoding="utf-8",
        )
        bench = Bench()
        bench.backend.info = calibration.resolve(str(broken))

        with caplog.at_level(logging.ERROR, logger="litegrip_studio.core.worker"):
            bench.send(cmd.LoadCalibration())

        assert bench.loop.gate is GateState.BLOCKED
        assert any("超出合理范围" in r.getMessage() for r in caplog.records)


class TestTheAlertGoesWhenTheReasonDoes:
    """The banner was a one-way channel: an alert went up and stayed until the
    next one replaced it, so a console that had recovered from a fault, an
    E-stop or a refused move still read as broken.

    It is the one thing on screen that speaks without being asked, so the loop
    watches the condition behind the alert it put up and retracts it.  Nothing is
    emitted to say a recovery happened — that is the whole problem — so the edge
    is looked for on the tick, by the object that knows the state.
    """

    def test_a_fault_that_clears_takes_its_banner_with_it(self) -> None:
        bench = Bench()
        bench.bring_up()
        bench.backend.error_code = constants.ERROR_OC
        bench.tick()
        assert bench.signals.of("fault"), "the fault was reported at all"
        assert not bench.signals.has("alert_cleared"), (
            "the tick that reports the fault is not the tick that retires it"
        )

        bench.backend.error_code = constants.ERROR_ENABLED
        bench.tick()

        assert bench.signals.has("alert_cleared")
        # Once, on the edge: the ticks after it have nothing left to retract.
        count = len(bench.signals.of("alert_cleared"))
        bench.tick(5)
        assert len(bench.signals.of("alert_cleared")) == count

    def test_clearing_a_fault_from_the_console_is_a_recovery_too(self) -> None:
        """The button the operator actually presses.  It re-reads the drive rather
        than waiting for the code to change by itself, so the retraction has to
        come out of that path as well."""
        bench = Bench()
        bench.bring_up()
        bench.backend.error_code = constants.ERROR_OC
        bench.tick()

        bench.send(cmd.ClearFault())

        assert bench.signals.has("alert_cleared")

    def test_a_fault_cleared_while_the_axis_is_still_disabled_goes_anyway(self) -> None:
        """Which is why the edge is "the frame stopped reporting an error" and not
        "motion is allowed again".  A fault can be cleared on a gripper nobody has
        enabled yet, and that red line has to go even though the axis is still
        sitting there unenergised and refusing to move."""
        bench = Bench()
        bench.send(cmd.Connect())
        bench.backend.error_code = constants.ERROR_UV
        bench.tick()
        assert bench.signals.of("fault")

        bench.backend.error_code = constants.ERROR_DISABLED
        bench.tick()

        assert "电机未使能" in bench.loop._refusal(), "the refusal never went away"
        assert bench.signals.has("alert_cleared")

    def test_the_estop_alert_goes_when_the_latch_does(self) -> None:
        bench = Bench()
        bench.bring_up()
        bench.loop.estop("测试")
        bench.tick()
        assert "急停" in bench.signals.alerts()[-1]
        assert not bench.signals.has("alert_cleared")

        bench.send(cmd.ResetEStop())

        assert bench.signals.has("alert_cleared")

    def test_a_refused_move_stops_being_shown_once_the_gate_opens(self) -> None:
        bench = Bench()
        bench.bring_up()
        bench.swap_calibration(BROKEN)
        bench.send(cmd.MoveToMm(30.0))
        assert "被拒绝" in bench.signals.alerts()[-1]
        assert not bench.signals.has("alert_cleared")

        bench.swap_calibration(USER_CAL)

        assert bench.signals.has("alert_cleared")

    def test_a_different_refusal_still_retires_the_one_on_screen(self) -> None:
        """The banner holds the refusal it was raised with, and one refusal is as
        good a reason to take it down as none: the line is no longer the answer to
        "why can't it move", and the next press says the real one out loud."""
        bench = Bench()
        bench.send(cmd.Connect())
        bench.send(cmd.SetZeroGravity(True))
        assert "未使能" in bench.signals.alerts()[-1]

        bench.send(cmd.Enable())
        bench.swap_calibration(BROKEN)

        assert "电机未使能" not in bench.loop._refusal()
        assert bench.signals.has("alert_cleared")

    def test_the_speed_cap_warning_survives_a_later_fault_clearing(self) -> None:
        """It names a supply the operator has to go and look at, and the loop never
        un-degrades itself: a fault clearing afterwards is a recovery for the alert
        it belongs to, and must not take this one down with it."""
        bench = Bench()
        bench.bring_up()
        for _ in range(constants.UV_FAULT_MAX):
            bench.backend.error_code = constants.ERROR_UV
            bench.tick()
            bench.backend.error_code = constants.ERROR_ENABLED
            bench.tick()
        clears = len(bench.signals.of("alert_cleared"))

        bench.backend.error_code = constants.ERROR_UV
        bench.tick()  # the one too many: the cap goes on, and it is what is shown
        assert "速度已限制" in bench.signals.alerts()[-1]
        assert bench.loop.motion.params.speed_mm_s == constants.UV_DEGRADED_SPEED_MM_S

        bench.backend.error_code = constants.ERROR_ENABLED
        bench.tick()  # the fault clears: the edge that retires a plain fault banner

        assert len(bench.signals.of("alert_cleared")) == clears

    def test_an_alert_about_an_event_is_left_where_it_is(self) -> None:
        """Not every alert names a condition.  A save that failed is a thing that
        happened, and no amount of the console recovering afterwards makes it
        untrue — so it stays until the next alert replaces it, which is what every
        alert did before any of this."""
        bench = Bench(RecordingBackend(fail=("save_calibration",)))
        bench.send(cmd.Connect())
        bench.send(cmd.SaveCalibration())
        assert "保存标定失败" in bench.signals.alerts()[-1]

        bench.send(cmd.Enable())

        assert bench.loop.enabled, "the refusal really did go away"
        assert not bench.signals.has("alert_cleared")

    def test_a_console_with_nothing_to_say_retracts_nothing(self) -> None:
        bench = Bench()
        bench.bring_up()
        bench.send(cmd.MoveToMm(30.0), count=20)

        assert not bench.signals.alerts()
        assert not bench.signals.has("alert_cleared")
