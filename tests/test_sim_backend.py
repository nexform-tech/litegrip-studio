"""The simulator's own semantics: stopping, disabling, and time.

:mod:`tests.test_plant` pins the physics and :mod:`tests.test_motion_fsm` pins
the trajectories.  What is left, and what this file covers, is the contract the
*backend* adds on top of the plant — the part the console talks to — because the
simulator is only useful if the states it can be put into mean the same thing
they mean on the bench.

The clock is injected, so a simulated second costs nothing and the same run
produces the same numbers on every machine.
"""

from __future__ import annotations

import json

import pytest

from litegrip_studio import calibration, constants
from litegrip_studio.backend.sim import SimBackend
from litegrip_studio.calibration import PROVENANCE_INVALID, PROVENANCE_USER
from litegrip_studio.core.worker import GateState, evaluate_gate
from litegrip_studio.units import derive_scale

from conftest import FakeClock


class Bench:
    """A connected, enabled simulator driven a tick at a time by hand."""

    def __init__(self, **kwargs) -> None:
        self.clock = FakeClock()
        self.sim = SimBackend(clock=self.clock, **kwargs)
        self.sim.connect()
        self.sim.enable()
        self.limits = self.sim.limits()

    def run(self, seconds: float, **frame) -> None:
        """Integrate ``seconds`` of simulated time at 200 Hz.

        When frame arguments are given they are streamed every tick, which is
        what the console does: a DM motor needs a continuous MIT stream, not one
        command and then silence.
        """
        for _ in range(int(seconds / constants.CTRL_DT)):
            if frame:
                self.sim.stream_frame(**frame)
            self.clock.advance(constants.CTRL_DT)
            self.sim.poll()

    def hold(self, mm: float, seconds: float = 1.0, kp: float = constants.KP_MOVE) -> None:
        self.run(
            seconds,
            q_rad=self.limits.to_rad(mm),
            kp=kp,
            kd=constants.KD_DEFAULT,
        )


class TestZeroTorqueIsNotAShutdown:
    """``zero_torque`` mirrors ``LiteGrip.stop()``: zero gain, motor still on.

    The distinction matters because the console uses it for both 零重力 and the
    gate-closing path, and because on the bench a stopped DM motor keeps
    listening — a console that had to re-enable before every command would work
    in the simulator and not on hardware.
    """

    def test_the_axis_can_be_commanded_again_after_a_stop(self) -> None:
        bench = Bench()
        bench.hold(0.0, seconds=0.3)
        bench.sim.zero_torque()

        # The regression this pins: a latched stop that silently refuses every
        # later frame, which turns the next move into a no-op with no error.
        assert bench.sim.stream_frame(
            bench.limits.to_rad(60.0), constants.KP_MOVE, constants.KD_DEFAULT
        )

        bench.hold(60.0, seconds=1.0)
        assert bench.sim.read().position_mm == pytest.approx(60.0, abs=2.0)

    def test_the_motor_keeps_reporting_after_a_stop(self) -> None:
        bench = Bench()
        bench.hold(0.0, seconds=0.3)
        bench.sim.zero_torque()
        bench.run(0.2)

        assert bench.sim.poll(), "a stopped motor still emits status frames"
        assert bench.sim.read().error_code in constants.OK_ERROR_CODES

    def test_a_stop_holds_position_rather_than_coasting(self) -> None:
        """Zero gain, not zero stiffness: the jaws stay where they were rather
        than drifting once the command goes away."""
        bench = Bench()
        bench.hold(60.0, seconds=1.0)
        bench.sim.zero_torque()
        bench.run(0.5)

        assert bench.sim.read().position_mm == pytest.approx(60.0, abs=1.0)


class TestDisableUnloadsThePlant:
    """A disabled driver stops applying what the last frame asked for.

    The plant has no notion of being enabled, so without an explicit unload it
    would go on squeezing at the last commanded torque for ever — and every test
    of a disabled gripper would be testing one that is still holding.
    """

    def test_the_squeeze_decays_once_the_motor_is_disabled(self) -> None:
        bench = Bench()
        bench.sim.plant.object_mm = 100.0
        bench.hold(0.0, seconds=1.0, kp=constants.KP_GRASP)
        squeezed = bench.sim.plant.tau
        assert abs(squeezed) > 1.0, "the gripper should be pressing on the object"

        bench.sim.disable()
        bench.run(1.0)

        assert abs(bench.sim.plant.tau) < 0.05 * abs(squeezed)

    def test_a_disabled_motor_stops_emitting_frames(self) -> None:
        bench = Bench()
        bench.run(0.2)
        bench.sim.disable()
        bench.run(0.2)

        assert not bench.sim.poll()
        assert bench.sim.read().position_mm is not None, "the snapshot stays readable"

    def test_a_disabled_motor_refuses_frames(self) -> None:
        bench = Bench()
        bench.sim.disable()

        assert not bench.sim.stream_frame(0.0, constants.KP_MOVE, constants.KD_DEFAULT)


class TestItsOwnCalibration:
    """Which calibration a simulator is running on, and which one it must not.

    The simulator keeps its own file so that a simulated calibration can never
    overwrite the bench gripper's.  These tests are the same guarantee read in
    the other direction — it must not *read* the bench file either — plus the
    one the console actually needs: a bare simulator is ready to move without
    the operator acknowledging anything.
    """

    #: Numbers that could not be mistaken for the plant's own, so a test can tell
    #: which of the two it ended up with.
    BENCH_RAW = {
        "zero_position_rad": 2.0,
        "max_position_rad": -0.4,
        "travel_range_rad": 2.4,
        "rad_to_mm": 40.0,
    }

    def test_a_bare_simulator_is_ready_to_move(self, tmp_path) -> None:
        """No acknowledgement, because there is nothing to acknowledge: the
        numbers are the ones the plant is running on, not data that might belong
        to a different gripper."""
        sim = SimBackend(clock=FakeClock(), calibration_path=tmp_path / "absent.json")
        info = sim.calibration_info()

        assert info.provenance == PROVENANCE_USER
        assert info.motion_allowed
        assert evaluate_gate(info)[0] is GateState.READY

    def test_connecting_does_not_change_the_answer(self, tmp_path, monkeypatch) -> None:
        """The regression, in the environment where it bit hardest: no calibration
        of its own, no bench calibration to borrow, and no factory data either —
        a machine with only this console on it.  Construction used to invent a
        default and the load on connect used to answer "missing", so the gate
        opened in front of the operator and shut again the moment they connected.
        """
        nowhere = tmp_path / "nowhere.json"
        monkeypatch.setattr(calibration, "default_user_path", lambda: nowhere)
        monkeypatch.setattr(calibration, "factory_path", lambda: nowhere)

        sim = SimBackend(clock=FakeClock(), calibration_path=tmp_path / "absent.json")
        before = sim.calibration_info()

        sim.connect()
        loaded = sim.load_calibration(None)
        after = sim.calibration_info()

        assert before.motion_allowed, "the console starts usable, as it should"
        assert loaded is True
        assert after.provenance == before.provenance
        assert after.limits.rad_to_mm == before.limits.rad_to_mm
        assert after.limits.closed_rad == before.limits.closed_rad

    def test_its_description_is_the_link_and_not_the_calibration(self, tmp_path) -> None:
        """The connection line is taken once, when the connection opens, and is
        never refreshed, so a calibration label in it is the state from before
        the load rather than a status — a console with a file loaded would go on
        saying 未标定.  The calibration page is where that belongs.

        Asserted on the *string changing*, not on a word: a label is free to be
        renamed, and the defect is that this line moves at all.
        """
        sim = SimBackend(clock=FakeClock(), calibration_path=tmp_path / "sim.json")
        before = sim.describe()
        sim.connect()

        sim.set_calibration_memory(1.780959, -0.069279, 64.86, max_stroke_mm=120.0)

        assert sim.describe() == before

    def test_it_never_reads_the_bench_grippers_calibration(
        self, tmp_path, monkeypatch
    ) -> None:
        """Reading it is the write hazard from the other side: the simulated
        geometry would silently become the real gripper's.

        The angles are the witness, not the scale: the scale is derived from the
        angles and the travel, so it differs from the file's on any file, and
        asserting on it here would pass for the wrong reason.
        """
        bench = tmp_path / "litegrip_calibration.json"
        bench.write_text(json.dumps(self.BENCH_RAW), encoding="utf-8")
        monkeypatch.setattr(calibration, "default_user_path", lambda: bench)

        sim = SimBackend(clock=FakeClock(), calibration_path=tmp_path / "absent.json")
        limits = sim.limits()

        assert sim.calibration_info().path != str(bench)
        assert limits.closed_rad != pytest.approx(self.BENCH_RAW["zero_position_rad"])
        assert limits.open_rad != pytest.approx(self.BENCH_RAW["max_position_rad"])

    def test_its_own_file_is_used_when_it_is_there(self, tmp_path) -> None:
        path = tmp_path / "sim.json"
        path.write_text(json.dumps(self.BENCH_RAW), encoding="utf-8")

        sim = SimBackend(clock=FakeClock(), calibration_path=path)
        sim.connect()
        assert sim.load_calibration(None) is True

        info = sim.calibration_info()
        assert info.path == str(path)
        assert info.provenance == PROVENANCE_USER
        limits = sim.limits()
        assert limits.closed_rad == pytest.approx(self.BENCH_RAW["zero_position_rad"])
        assert limits.open_rad == pytest.approx(self.BENCH_RAW["max_position_rad"])
        # Both the angles and the travel entering the scale derived from them,
        # which is the whole of what the file contributes: its own rad_to_mm is
        # its nominal stroke over this span and is deliberately not used.
        assert limits.rad_to_mm == pytest.approx(
            derive_scale(limits.travel_rad, limits.max_stroke_mm)
        )
        assert limits.rad_to_mm != pytest.approx(self.BENCH_RAW["rad_to_mm"])

    def test_an_unusable_file_of_its_own_is_not_papered_over(self, tmp_path) -> None:
        """A file with no travel in it must block, not quietly become the default.
        A fallback here would be the SDK's silent fallback wearing the simulator's
        name, and it would mean the console could not tell a good file from a bad
        one by looking at the gate.

        The unusable file chosen is one whose two extremes are the same angle,
        rather than one whose angles come in the other order: the second is a
        reverse-mounted gripper and is a perfectly good calibration.
        """
        path = tmp_path / "no_travel.json"
        path.write_text(
            json.dumps(
                {
                    "zero_position_rad": 1.0,
                    "max_position_rad": 1.0,
                    "rad_to_mm": 65.21,
                }
            ),
            encoding="utf-8",
        )

        sim = SimBackend(clock=FakeClock(), calibration_path=path)
        sim.connect()

        assert sim.load_calibration(None) is False
        info = sim.calibration_info()
        assert info.provenance == PROVENANCE_INVALID
        assert not info.motion_allowed
        assert evaluate_gate(info)[0] is GateState.BLOCKED

    def test_a_file_recording_the_other_ordering_is_applied(self, tmp_path) -> None:
        """The other order of the same two angles, and it is not a defect: the
        simulator has to be able to run on a gripper whose angle grows as the
        jaws open, or the console cannot be exercised against one."""
        path = tmp_path / "flipped.json"
        path.write_text(
            json.dumps(
                {
                    "zero_position_rad": -0.300793,
                    "max_position_rad": 1.421569,
                    "rad_to_mm": 49.93,
                }
            ),
            encoding="utf-8",
        )

        sim = SimBackend(clock=FakeClock(), calibration_path=path)
        sim.connect()

        assert sim.load_calibration(None) is True
        info = sim.calibration_info()
        assert info.provenance == PROVENANCE_USER
        assert info.motion_allowed
        limits = sim.limits()
        assert limits.direction == 1.0
        assert limits.to_rad(0.0) == pytest.approx(-0.300793)


class TestTheTravelSettingRescalesTheSimulator:
    """Changing the travel re-derives the scale, as a real reload does.

    The travel is what the millimetres per rad is derived from, so a simulator
    that kept the scale it was built with would report the same jaw positions in
    millimetres no matter what the setting said — the numbers on every page would
    move while the geometry behind them did not, and the simulator would stop
    being a check on the console.
    """

    def test_the_scale_follows_the_travel(self, tmp_path) -> None:
        sim = SimBackend(clock=FakeClock(), calibration_path=tmp_path / "absent.json")
        before = sim.limits()

        sim.set_travel_mm(120.0)

        after = sim.limits()
        assert after.max_stroke_mm == pytest.approx(120.0)
        assert after.travel_rad == pytest.approx(before.travel_rad), "the angles are the unit"
        assert after.rad_to_mm == pytest.approx(derive_scale(after.travel_rad, 120.0))
        assert after.rad_to_mm != pytest.approx(before.rad_to_mm)
        assert after.stroke_mm == pytest.approx(after.max_stroke_mm + 1.0)


class TestASavedProbeResultStopsBeingInMemory:
    """The end of the guided probe, on the simulator.

    A probe result lives in memory and blocks the gate until it is saved — the
    operator has to be able to see what they measured before it becomes what the
    console moves by.  Saving it is therefore the step that has to open the gate,
    and on the bench it is the re-read of the written file that does it.  A
    simulator that only wrote the file would leave an operator who had just
    calibrated and saved looking at a console that still refused to move.
    """

    PROBE = {"zero_rad": 1.780959, "open_rad": -0.069279, "rad_to_mm": 64.86}

    def test_it_is_saved_rather_than_still_pending(self, tmp_path) -> None:
        path = tmp_path / "sim.json"
        sim = SimBackend(clock=FakeClock(), calibration_path=path)
        sim.connect()
        pending = sim.set_calibration_memory(**self.PROBE, max_stroke_mm=120.0)
        assert pending.provenance == calibration.PROVENANCE_MEMORY
        assert evaluate_gate(pending)[0] is GateState.BLOCKED

        sim.save_calibration()

        info = sim.calibration_info()
        assert info.provenance == PROVENANCE_USER
        assert info.path == str(path)
        assert evaluate_gate(info)[0] is GateState.READY

    def test_what_comes_back_is_what_the_file_holds(self, tmp_path) -> None:
        """Read back off the disk, not carried over from memory: a simulator
        that kept its own copy would agree with itself while disagreeing with
        everything the file is for."""
        path = tmp_path / "sim.json"
        sim = SimBackend(clock=FakeClock(), calibration_path=path)
        sim.connect()
        sim.set_calibration_memory(**self.PROBE, max_stroke_mm=120.0)

        written = sim.save_calibration()

        on_disk = json.loads(path.read_text(encoding="utf-8"))
        info = sim.calibration_info()
        limits = info.limits
        assert written == str(path)
        assert info.path == str(path)
        assert limits.closed_rad == pytest.approx(self.PROBE["zero_rad"])
        assert limits.open_rad == pytest.approx(self.PROBE["open_rad"])
        assert limits.max_stroke_mm == pytest.approx(120.0)
        # What comes back is what the console moves by, and the file records
        # that: the scale is derived from the angles and the travel, where the
        # probe's own number was its nominal 120 mm over the span.
        assert limits.rad_to_mm == pytest.approx(
            derive_scale(limits.travel_rad, 120.0)
        )
        assert on_disk["rad_to_mm"] == pytest.approx(limits.rad_to_mm)
        assert on_disk["rad_to_mm"] != pytest.approx(self.PROBE["rad_to_mm"])
