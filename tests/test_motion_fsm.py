"""The motion state machine: one frame per tick, and never a blocking call.

The state machine is where the safety properties actually live, so most of what
is asserted here is a *refusal* as much as an action: that a gated move sends
nothing at all, that a plain move never quietly acquires a force setpoint, that
the commanded angle never leaves the calibrated travel, and that the frame which
holds a grip carries no position gain.

Two harnesses are used.  :class:`IdealPlant` tracks its command instantly, which
removes the servo from the picture so the logic is what fails when a test fails.
The ``rig`` fixture from ``conftest`` drives the real simulated plant, and is used
wherever the physics is load-bearing — force control and obstruction.
"""

from __future__ import annotations

import math
from typing import NamedTuple

import pytest

from litegrip_studio import constants
from litegrip_studio.backend.plant import Plant, PlantConfig
from litegrip_studio.core.motion import MotionFSM, MotionParams, MotionState
from litegrip_studio.telemetry import Telemetry
from litegrip_studio.units import Limits, clamp_force_torque, torque_from_force

DT = constants.CTRL_DT
#: The SDK's example 120 mm unit.  Pinned here rather than taken from the
#: defaults because these tests are about the motion law, and a limit that moved
#: with the travel setting would move the numbers they assert on with it.
BENCH = Limits(1.775959, -0.064279, 65.21, 120.0)


class Frame(NamedTuple):
    q_rad: float
    kp: float
    kd: float
    dq_rad_s: float
    tau_nm: float
    #: Whether the frame was sent with the calibration gate relaxed.  Recorded
    #: rather than ignored because it is the difference between a frame that
    #: reaches the motor on a console with an unusable file and one that does not.
    ungated: bool = False


class IdealPlant:
    """A backend whose jaws are exactly where they were last told to be.

    Removing the servo is the point: a failure here is a failure of the state
    machine, not of the tuning.
    """

    def __init__(self, limits: Limits = BENCH, start_mm: float = 0.0) -> None:
        self.limits = limits
        self.mm = start_mm
        self.frames: list[Frame] = []
        self.accept = True
        self.velocity_rad_s = 0.0
        self.torque_nm = 0.0

    def stream_frame(self, q_rad, kp, kd, dq_rad_s=0.0, tau_nm=0.0, *, ungated=False) -> bool:
        self.frames.append(Frame(q_rad, kp, kd, dq_rad_s, tau_nm, ungated))
        if not self.accept:
            return False
        self.mm = self.limits.clamp_mm(self.limits.to_mm(q_rad))
        return True

    def telemetry(self, **overrides) -> Telemetry:
        fields = dict(
            position_mm=self.mm,
            position_rad=self.limits.to_rad(self.mm),
            velocity_rad_s=self.velocity_rad_s,
            torque_nm=self.torque_nm,
        )
        fields.update(overrides)
        return Telemetry(**fields)

    # ── assertions shared by the scenarios ──────────────────────────────────
    def assert_within_travel(self) -> None:
        """Every commanded angle must be inside the calibrated stops.

        This is the promise that makes it safe to bypass the SDK's own ``goto_rad``
        clamp — which is computed from a config this console may already know to
        be wrong.
        """
        for f in self.frames:
            assert self.limits.rad_low - 1e-9 <= f.q_rad <= self.limits.rad_high + 1e-9, (
                f"commanded {f.q_rad} outside "
                f"[{self.limits.rad_low}, {self.limits.rad_high}]"
            )

    def assert_finite(self) -> None:
        for f in self.frames:
            assert all(math.isfinite(v) for v in f), f"non-finite frame {f}"


def run_ticks(fsm, backend, ticks, *, allow_motion=True, dt=DT, telemetry=None, on_tick=None):
    """Tick the FSM, collecting what it commanded."""
    outs = []
    for _ in range(ticks):
        tele = telemetry() if telemetry is not None else backend.telemetry()
        outs.append(fsm.tick(backend, tele, dt, allow_motion=allow_motion))
        if on_tick is not None:
            on_tick(backend)
    return outs


def servo(limits: Limits = BENCH, start_mm: float = 0.0, **kwargs) -> tuple[MotionFSM, IdealPlant]:
    fsm = MotionFSM(limits, MotionParams(**kwargs) if kwargs else None)
    return fsm, IdealPlant(limits, start_mm)


class TestPlainMove:
    def test_a_plain_move_reaches_hold_at_the_target(self) -> None:
        fsm, backend = servo(start_mm=0.0)
        fsm.move_to_mm(60.0, "slider")
        outs = run_ticks(fsm, backend, 6000)
        assert fsm.state is MotionState.HOLD
        assert backend.mm == pytest.approx(60.0, abs=constants.TOL_MM)
        assert any(o.note == "arrived" for o in outs)

    def test_a_plain_move_never_acquires_a_force_setpoint(self) -> None:
        """The regression: ``move_to_mm`` used to default ``force_n`` to the
        configured grasp force, so *every* move — including ``open()`` — ran in
        force mode, where the position gain is zero.  A plain move must have no
        force in it at all.
        """
        fsm, backend = servo(start_mm=120.0)
        fsm.move_to_mm(0.0, "slider")
        outs = run_ticks(fsm, backend, 6000)

        for out in outs:
            assert out.tau_nm == 0.0, "a plain move commands no feed-forward torque"
        assert fsm.state is MotionState.HOLD
        assert fsm.force_setpoint() == 0.0

    def test_open_and_close_use_the_whole_travel(self) -> None:
        fsm, backend = servo(start_mm=0.0)
        fsm.open()
        run_ticks(fsm, backend, 6000)
        assert backend.mm == pytest.approx(BENCH.max_stroke_mm, abs=constants.TOL_MM)

        fsm.close()
        run_ticks(fsm, backend, 6000)
        assert backend.mm == pytest.approx(0.0, abs=constants.TOL_MM)
        assert fsm.state is MotionState.HOLD

    def test_the_command_never_leaves_the_calibrated_travel(self) -> None:
        fsm, backend = servo(start_mm=60.0)
        fsm.open()
        run_ticks(fsm, backend, 6000)
        backend.assert_within_travel()

    def test_a_target_beyond_the_stop_is_clamped_not_refused(self) -> None:
        fsm, backend = servo(start_mm=60.0)
        fsm.move_to_mm(1e6, "slider")
        run_ticks(fsm, backend, 6000)
        assert backend.mm == pytest.approx(BENCH.max_stroke_mm, abs=constants.TOL_MM)
        backend.assert_within_travel()

    def test_narrowing_the_travel_mid_move_pulls_the_command_in(self) -> None:
        """A calibration change must take effect on the very next frame."""
        fsm, backend = servo(start_mm=60.0)
        fsm.open()
        run_ticks(fsm, backend, 100)
        narrow = Limits(1.775959, -0.064279, 65.21, max_stroke_mm=80.0)
        fsm.set_limits(narrow)
        backend.limits = narrow
        run_ticks(fsm, backend, 6000)
        assert backend.mm <= 80.0 + constants.TOL_MM
        backend.assert_within_travel()


class TestPreemption:
    def test_a_retarget_in_flight_is_adopted_at_once(self) -> None:
        """A slider drag must be able to redirect the jaws, not queue behind them."""
        fsm, backend = servo(start_mm=0.0)
        fsm.move_to_mm(120.0, "slider")
        run_ticks(fsm, backend, 40)
        fsm.move_to_mm(10.0, "slider")

        assert fsm.last_command_mm == 10.0
        assert fsm.profile.target_mm == 10.0
        run_ticks(fsm, backend, 6000)
        assert backend.mm == pytest.approx(10.0, abs=constants.TOL_MM)

    def test_the_reversal_is_bounded_by_the_deceleration_rate(self) -> None:
        """Adopted at once, but the jaws cannot reverse instantly.

        They carry ``v`` of speed, so turning around costs ``v/a`` to stop and
        another ``v/a`` to come back through the distance covered while stopping
        (``v²/2a`` = 3.1 mm here).  ``2·v/a`` is therefore the physical minimum,
        and the allowance below is twice that to leave room for servo lag.
        """
        fsm, backend = servo(start_mm=0.0)
        fsm.move_to_mm(120.0, "slider")
        run_ticks(fsm, backend, 200)  # well past the new target, so it is a reversal
        travelling = backend.mm
        assert travelling > 20.0
        fsm.move_to_mm(10.0, "slider")

        decel_ticks = int(
            constants.SPEED_DEFAULT_MM_S / constants.ACC_DEFAULT_MM_S2 / DT
        )
        outs = run_ticks(fsm, backend, decel_ticks * 4)
        assert any(
            o.q_cmd_mm is not None and o.q_cmd_mm < travelling for o in outs
        ), "the reference should have turned around within a couple of v/a"

    def test_stop_holds_where_it_is_with_no_feed_forward(self) -> None:
        fsm, backend = servo(start_mm=0.0)
        fsm.move_to_mm(120.0, "slider")
        run_ticks(fsm, backend, 40)
        backend.velocity_rad_s = 0.0

        fsm.hold(backend.mm)
        outs = run_ticks(fsm, backend, 3)
        assert fsm.state is MotionState.HOLD
        assert all(o.tau_nm == 0.0 for o in outs)
        assert outs[-1].q_cmd_mm == pytest.approx(backend.mm)

    def test_hold_keeps_sending_frames_so_the_pose_is_held(self) -> None:
        """Stopping is not the same as going quiet: the motor needs a frame."""
        fsm, backend = servo(start_mm=60.0)
        fsm.hold(60.0)
        outs = run_ticks(fsm, backend, 3)
        assert all(o.sent for o in outs)
        assert all(o.kp == constants.KP_MOVE for o in outs)

    def test_idle_sends_nothing(self) -> None:
        fsm, backend = servo(start_mm=60.0)
        fsm.idle()
        outs = run_ticks(fsm, backend, 3)
        assert not any(o.sent for o in outs)
        assert backend.frames == []


class TestGate:
    """A shut gate must leave the motor commanded by nothing at all.

    The first cut put the state machine into HOLD, which reads as harmless but is
    not: HOLD keeps sending frames so the pose is held, and every one of those
    frames is a position derived from the very limits the gate is doubtful about.
    Under a reversed calibration, "hold still" is a command to close hard.
    """

    def test_a_blocked_move_stops_commanding_anything(self) -> None:
        fsm, backend = servo(start_mm=0.0)
        fsm.move_to_mm(120.0, "slider")
        run_ticks(fsm, backend, 20)
        sent_before = len(backend.frames)

        outs = run_ticks(fsm, backend, 3, allow_motion=False)
        assert all(not o.sent for o in outs)
        assert len(backend.frames) == sent_before, "nothing may reach the motor"
        assert fsm.state is MotionState.BLOCKED

    def test_it_keeps_commanding_nothing_for_as_long_as_the_gate_is_shut(self) -> None:
        fsm, backend = servo(start_mm=0.0)
        fsm.move_to_mm(120.0, "slider")
        run_ticks(fsm, backend, 20)
        sent_before = len(backend.frames)
        run_ticks(fsm, backend, 2000, allow_motion=False)
        assert len(backend.frames) == sent_before

    def test_a_held_gripper_also_stops_commanding_when_the_gate_shuts(self) -> None:
        fsm, backend = servo(start_mm=60.0)
        fsm.hold(60.0)
        run_ticks(fsm, backend, 5)
        sent_before = len(backend.frames)

        outs = run_ticks(fsm, backend, 5, allow_motion=False)
        assert all(not o.sent for o in outs)
        assert len(backend.frames) == sent_before

    def test_the_blocked_reason_survives_into_the_note(self) -> None:
        fsm, _ = servo(start_mm=0.0)
        fsm.move_to_mm(120.0, "slider")
        out = fsm.tick(IdealPlant(), Telemetry(position_mm=0.0), DT, False, "标定未就绪")
        assert out.note == "标定未就绪"
        assert fsm.note == "标定未就绪"

    def test_a_reopened_gate_holds_the_pose_without_resuming_the_move(self) -> None:
        """The trajectory is stale by then, so the operator re-issues."""
        fsm, backend = servo(start_mm=0.0)
        fsm.move_to_mm(120.0, "slider")
        run_ticks(fsm, backend, 20)
        run_ticks(fsm, backend, 5, allow_motion=False)

        outs = run_ticks(fsm, backend, 3, allow_motion=True)
        assert fsm.state is MotionState.HOLD
        assert outs[0].sent and outs[0].tau_nm == 0.0
        assert outs[0].q_cmd_mm == pytest.approx(backend.mm)

    def test_a_released_gripper_needs_no_gate(self) -> None:
        """Zero stiffness has nothing to gate — the jaws cannot be driven."""
        fsm, backend = servo(start_mm=60.0)
        fsm.release()
        outs = run_ticks(fsm, backend, 3, allow_motion=False)
        assert all(o.sent for o in outs)

    def test_a_gripper_held_at_an_angle_needs_no_gate_either(self) -> None:
        """The one *hold* a shut gate cannot stop, and the difference from the
        millimetre hold above is the whole point: it names the angle the motor
        just reported, so there is no travel in it to be doubtful about.  An
        axis held through a probe's result being unsaved still has to be held —
        the alternative is an enabled drive told nothing at all, with the jaws
        free at the moment the press that was loading them is released."""
        fsm, backend = servo(start_mm=60.0)
        fsm.hold_rad(BENCH.to_rad(60.0))
        outs = run_ticks(fsm, backend, 3, allow_motion=False)
        assert all(o.sent for o in outs)
        assert all(o.q_cmd_mm is None for o in outs), "nothing in millimetres"


class TestFreeStates:
    @pytest.mark.parametrize(
        "enter", [lambda f: f.release(), lambda f: f.zero_gravity(True, 60.0)]
    )
    def test_the_jaws_can_be_moved_by_hand(self, enter) -> None:
        fsm, backend = servo(start_mm=60.0)
        enter(fsm)
        outs = run_ticks(fsm, backend, 3)
        assert all(o.kp == 0.0 and o.kd == 0.0 and o.tau_nm == 0.0 for o in outs)
        assert outs[-1].q_cmd_mm == pytest.approx(backend.mm)

    def test_leaving_zero_gravity_holds(self) -> None:
        fsm, backend = servo(start_mm=60.0)
        fsm.zero_gravity(True, 60.0)
        fsm.zero_gravity(False, 60.0)
        assert fsm.state is MotionState.HOLD

    def test_leaving_it_without_a_measurement_releases_instead(self) -> None:
        """Holding is a command built on a measurement.  With none, the only
        honest state is the one that commands no position at all — a hold would
        be frozen at the placeholder angle the motor never sent."""
        fsm, backend = servo(start_mm=60.0)
        fsm.zero_gravity(True)

        fsm.zero_gravity(False, None)

        assert fsm.state is MotionState.RELEASE
        assert fsm.source == "zero_g"
        outs = run_ticks(fsm, backend, 2)
        assert all(o.kp == 0.0 and o.kd == 0.0 for o in outs)

    def test_a_fault_command_zeroes_the_state_machine(self) -> None:
        fsm, backend = servo(start_mm=60.0)
        fsm.move_to_mm(0.0, "slider")
        run_ticks(fsm, backend, 20)
        fsm.fault()
        outs = run_ticks(fsm, backend, 3)
        assert fsm.state is MotionState.FAULT
        assert not any(o.sent for o in outs)

    @pytest.mark.parametrize(
        "enter", [lambda f: f.release(), lambda f: f.zero_gravity(True, 60.0)]
    )
    def test_a_free_state_is_marked_ungated_so_a_shut_gate_cannot_refuse_it(
        self, enter
    ) -> None:
        """Zero stiffness has nothing to gate, and on real hardware an ungated
        flag is the difference between 松力 working and doing nothing.

        松力 on a console whose file is unusable is how the operator gets the
        jaws into their hands to re-calibrate, which is exactly the console that
        has a shut gate.
        """
        fsm, backend = servo(start_mm=60.0)
        enter(fsm)

        run_ticks(fsm, backend, 3)

        assert backend.frames, "expected frames"
        assert all(f.ungated for f in backend.frames)

    def test_a_servo_frame_is_not_ungated(self) -> None:
        """The other half of the rule: a frame that names a position derived
        from the limits must still be refusable."""
        fsm, backend = servo(start_mm=0.0)
        fsm.move_to_mm(60.0, "slider")

        run_ticks(fsm, backend, 20)

        assert backend.frames
        assert not any(f.ungated for f in backend.frames)


class TestHoldAtAnAngle:
    """A hold that never goes near the calibration.

    It exists for the end of a probe, which is the one motion that deliberately
    leaves the calibrated travel — it looks for the mechanical stops and stops on
    one — so the only honest pose to hand back is the angle the encoder is
    reporting, not a millimetre computed from a travel that may not even be in
    force yet.
    """

    #: Past the open end of BENCH by more than the clamp would allow: the place
    #: a probe leaves the axis, and the place a millimetre cannot name.
    BEYOND = 1.9

    def test_the_angle_is_not_clamped_into_the_travel(self) -> None:
        fsm, backend = servo(start_mm=0.0)
        fsm.hold_rad(self.BEYOND)
        assert self.BEYOND > BENCH.rad_high, "the fixture must be outside the travel"

        outs = run_ticks(fsm, backend, 3)

        assert [f.q_rad for f in backend.frames] == [pytest.approx(self.BEYOND)] * 3
        assert outs[-1].q_cmd_mm is None, "no millimetre is being held"

    def test_it_holds_with_the_position_gain_and_no_feed_forward(self) -> None:
        fsm, backend = servo(start_mm=0.0)
        fsm.hold_rad(self.BEYOND)
        outs = run_ticks(fsm, backend, 1)
        assert outs[-1].kp == constants.KP_MOVE
        assert outs[-1].kd == constants.KD_DEFAULT
        assert outs[-1].tau_nm == 0.0
        assert outs[-1].sent

    def test_a_hold_in_millimetres_does_not_inherit_the_angle(self) -> None:
        """Both holds are ``HOLD``-shaped to a reader of the frames, so the one
        that must not win is pinned here."""
        fsm, backend = servo(start_mm=0.0)
        fsm.hold_rad(self.BEYOND)

        fsm.hold(60.0)
        outs = run_ticks(fsm, backend, 3)

        assert fsm.state is MotionState.HOLD
        assert outs[-1].q_cmd_mm == pytest.approx(60.0)
        assert backend.frames[-1].q_rad == pytest.approx(BENCH.to_rad(60.0))

    def test_a_hold_at_an_angle_is_a_hold_not_a_release(self) -> None:
        """RELEASE would also stream through a shut gate, and it is the wrong
        state to hand an axis back in: zero stiffness is exactly what lets a
        sprung mechanism open the jaws by itself."""
        fsm, backend = servo(start_mm=0.0)
        fsm.hold_rad(self.BEYOND)
        run_ticks(fsm, backend, 1)
        assert fsm.state is MotionState.HOLD_RAD
        assert backend.frames[-1].kp == constants.KP_MOVE

    def test_the_hold_reaches_a_motor_the_travel_check_would_refuse(self) -> None:
        """``BEYOND`` is outside BENCH by more than the clamp allows, which is
        where a probe leaves the axis — and behind a shut gate it is also where
        the travel check sits.

        Sent gated, this frame is refused and the axis is left free while the
        console reports it held: the one lie the operator cannot see, because
        the fingers only answer to the springs when nothing is commanding them.
        """
        fsm, backend = servo(start_mm=0.0)
        fsm.hold_rad(self.BEYOND)

        run_ticks(fsm, backend, 3)

        assert all(f.ungated for f in backend.frames)
        assert [f.q_rad for f in backend.frames] == [pytest.approx(self.BEYOND)] * 3


class TestFrameAssembly:
    def test_the_velocity_feed_forward_matches_the_jaw_direction(self) -> None:
        """Opening must produce a *negative* angular velocity reference."""
        fsm, backend = servo(start_mm=0.0)
        fsm.move_to_mm(120.0, "slider")
        run_ticks(fsm, backend, 40)
        moving = [f for f in backend.frames if abs(f.dq_rad_s) > 1e-6]
        assert moving, "expected feed-forward at cruise"
        assert all(f.dq_rad_s < 0.0 for f in moving)

    def test_hostile_telemetry_cannot_produce_a_non_finite_frame(self) -> None:
        """The last line of defence before the bus.  The plant does not sanitise,
        and neither would a real motor — so nothing non-finite may get past here."""
        for bad in (math.nan, math.inf, -math.inf):
            fsm, backend = servo(start_mm=60.0)
            fsm.move_to_mm(30.0, "slider")
            run_ticks(
                fsm,
                backend,
                20,
                telemetry=lambda: Telemetry(
                    position_mm=bad, position_rad=bad, velocity_rad_s=bad, torque_nm=bad
                ),
            )
            backend.assert_finite()


class TestObstructionWithoutForce:
    def test_a_blocked_move_stops_and_says_so(self, rig) -> None:
        assert rig.open()
        rig.sim.inject(obj_mm=60.0)
        rig.fsm.move_to_mm(0.0, "slider")
        rig.run_until(MotionState.HOLD, timeout_s=10.0)

        assert rig.fsm.state is MotionState.HOLD
        assert "堵转" in rig.fsm.note
        assert rig.position_mm > 55.0, "it must stop at the object, not pass through"

    def test_a_blocked_plain_move_does_not_crush_what_it_touches(self, rig) -> None:
        """A move with no force setpoint squeezes only by ``kp`` times the lost
        motion it tolerates before noticing.

        The squeeze is ``KP_MOVE / rad_to_mm × CONTACT_LOST_MM`` newtons, so it
        is a property of the unit rather than a constant: on this one's derived
        46.5 mm/rad it is 21.5 N.  That is the number the bound is built from —
        hard-coding it was wrong, because the derivation moved the millimetres
        per rad when the travel became a measurement.  The excess over it is the
        arrival impact, before the loss has had time to accumulate.

        What matters as much as the size is the shape: HOLD freezes the pose the
        contact was noticed at, so the spring eases the jaws back out and the
        force decays — a position move that meets something pushes and then lets
        go, where a grasp holds its torque until released.  Both stay under the
        40 N rating.
        """
        squeeze_n = (
            constants.KP_MOVE / rig.limits.rad_to_mm * constants.CONTACT_LOST_MM * 10.0
        )
        assert squeeze_n < constants.FORCE_MAX_N, "the derivation must fit the rating"

        assert rig.open()
        rig.sim.inject(obj_mm=60.0)
        rig.fsm.move_to_mm(0.0, "slider")
        rig.run_until(MotionState.HOLD, timeout_s=10.0)
        assert rig.peak_force < 1.5 * squeeze_n
        assert rig.peak_force < constants.FORCE_MAX_N

        rig.run(1.0)
        assert abs(rig.sim.read().force_n) < 0.25 * squeeze_n, "it must let go"


class Laggard(IdealPlant):
    """A mechanism that delivers a fixed fraction of the motion it is asked for.

    The gap this accumulates is the one a tight linkage, friction, a load or too
    little gain for the speed asked for all produce: the jaws sit behind where
    the trajectory's model says they should be, while travelling at a steady
    fraction of the speed that was commanded.  ``ratio=0`` is a mechanism that
    does not move at all.
    """

    def __init__(self, limits: Limits = BENCH, start_mm: float = 0.0, ratio: float = 0.8) -> None:
        super().__init__(limits, start_mm)
        self.ratio = ratio

    def stream_frame(self, q_rad, kp, kd, dq_rad_s=0.0, tau_nm=0.0, *, ungated=False) -> bool:
        self.frames.append(Frame(q_rad, kp, kd, dq_rad_s, tau_nm, ungated))
        target = self.limits.clamp_mm(self.limits.to_mm(q_rad))
        self.mm += (target - self.mm) * self.ratio
        self.velocity_rad_s = dq_rad_s * self.ratio
        return True


class StickyPlant:
    """The simulated mechanism driven directly, without the backend around it.

    ``SimBackend`` adds real-time pacing and a calibration file; a test about
    which moves get stopped wants neither, so this is the plant plus the two
    calls the state machine makes.
    """

    def __init__(self, config: PlantConfig) -> None:
        self.plant = Plant(config)
        self.limits = self.plant.limits

    def stream_frame(self, q_rad, kp, kd, dq_rad_s=0.0, tau_nm=0.0, *, ungated=False) -> bool:
        del ungated  # the plant underneath has no gate
        self.plant.stream(q_rad, kp, kd, dq_rad_s, tau_nm)
        return True

    def read(self) -> Telemetry:
        return self.plant.snapshot()


class TestWhetherAGapIsAnObstruction:
    """A gap between the jaws and the trajectory's model is not enough on its own.

    The gap says "they should have come this far by now", which assumes the
    mechanism keeps up once the reference is at cruise.  One that does not
    accumulates the same gap while working perfectly, and stopping it is a fault
    rather than a safety measure — the axis lurches a couple of millimetres per
    command instead of going where it was sent, worst at the slow end of the
    speed range.  These pin the two cases apart; see
    :data:`~litegrip_studio.constants.CONTACT_STILL_RATIO`.
    """

    def _run(self, fsm: MotionFSM, backend, ticks: int = 20000):
        """Tick to the end of the move, recording whether the gap ever tripped."""
        saw_contact = False
        for _ in range(ticks):
            fsm.tick(backend, backend.telemetry(), DT, allow_motion=True)
            out = fsm.last_profile_out
            saw_contact = saw_contact or (out is not None and out.contact)
            if fsm.state is not MotionState.SERVO:
                break
        return saw_contact

    def test_a_move_that_only_lags_is_not_stopped(self) -> None:
        """A 50 %-efficient mechanism at 150 mm/s: it delivers the motion but
        half of it, so the ideal path runs away from it by 0.38 mm a tick and the
        gap is unarguable.

        The ratio is what it is because the lead is *bounded* — at 20 mm/s and 0.8
        the console now closes the shortfall itself and no gap opens at all, which
        would make this assertion vacuous.  What the two settings have in common
        is the thing being pinned: a gap that large is still not an obstruction on
        its own, because the jaws are plainly moving.  It is also a second,
        independent check on the lead's bound — an unbounded one would have
        absorbed this too.
        """
        fsm = MotionFSM(BENCH, MotionParams(speed_mm_s=150.0))
        backend = Laggard(BENCH, ratio=0.5)

        fsm.move_to_mm(60.0, "slider")
        saw_contact = self._run(fsm, backend)

        assert saw_contact, "the lag must exceed the contact threshold, or this proves nothing"
        assert fsm.state is MotionState.HOLD
        assert backend.mm == pytest.approx(60.0, abs=constants.TOL_MM)
        assert fsm.note == "", "a move that is working must not be reported as stuck"

    def test_a_move_that_is_held_still_is_stopped(self) -> None:
        fsm = MotionFSM(BENCH, MotionParams(speed_mm_s=20.0))
        backend = Laggard(BENCH, ratio=0.0)

        fsm.move_to_mm(60.0, "slider")
        self._run(fsm, backend)

        assert fsm.state is MotionState.HOLD
        assert "堵转" in fsm.note
        assert backend.mm == pytest.approx(0.0, abs=0.5), "it must not have moved"

    def test_an_unreadable_velocity_is_not_an_obstruction_either(self) -> None:
        """The reading is what separates "lagging" from "held", so a velocity
        nobody can trust must not be read as "stopped".  The move deadline
        bounds the case instead."""
        fsm = MotionFSM(BENCH, MotionParams(speed_mm_s=20.0))
        backend = Laggard(BENCH, ratio=0.0)

        fsm.move_to_mm(60.0, "slider")
        run_ticks(
            fsm,
            backend,
            200,
            telemetry=lambda: backend.telemetry(velocity_rad_s=math.nan),
        )

        assert fsm.state is MotionState.SERVO

    def test_a_sticky_mechanism_still_reaches_its_target(self) -> None:
        """The case that prompted the rule, measured on the simulated unit.

        At 0.3 Nm of Coulomb friction a 20 mm/s move used to be declared stuck
        after 2.7 mm while covering 87% of the speed it had been asked for — the
        operator's "松手后它动两下就停" rather than a move.  Pinned against the
        plant's friction coefficient, which is a tuning choice: if that number
        moves, this says what it was pinned for.
        """
        backend = StickyPlant(PlantConfig(coulomb=0.3))
        fsm = MotionFSM(backend.limits, MotionParams(speed_mm_s=20.0))

        fsm.move_to_mm(60.0, "slider")
        for _ in range(20000):
            fsm.tick(backend, backend.read(), DT, allow_motion=True)
            backend.plant.step(DT)
            if fsm.state is not MotionState.SERVO:
                break

        assert backend.read().position_mm == pytest.approx(60.0, abs=constants.TOL_MM)
        assert fsm.note == ""


class TestASlowMoveAgainstFriction:
    """The failure an operator reports as "点击全部闭合时无法全部闭合，拖动时只能移动
    一小段": the reference is anchored to the measurement, so at cruise the servo
    is given one tick of travel and pushes with ``kp·v·dt`` — 0.2 Nm at 20 mm/s,
    twice that at 150 — and a mechanism drawing more friction than that stands
    still.  Every detector then reads the standstill as an obstruction, so the
    axis stops as well as stalling.

    :data:`~litegrip_studio.constants.CONTACT_LEAD_RAD` is sized for it.  These
    pin the two halves of that sizing against the simulated unit: 1.0 Nm of
    Coulomb friction arrives (1.2 does not — the constant's comment has the
    measurement), and the same mechanism with something in the way is still
    caught and still not crushed.  A lead that bought the first without the
    second would be a blindfold, not a fix.
    """

    #: The friction the lead is sized to cover.  The plant's own free travel
    #: draws 0.01 Nm, so this is a hundred times worse than the unit.
    FRICTION_NM = 1.0

    @staticmethod
    def _plant(coulomb: float, start_mm: float = 0.0):
        backend = StickyPlant(PlantConfig(coulomb=coulomb))
        backend.plant.q = backend.limits.to_rad(start_mm)
        backend.plant.dq = 0.0
        return backend

    @staticmethod
    def _drive(fsm, backend, ticks: int = 20000) -> float:
        """Tick until the move ends, and report the hardest push it made."""
        peak = 0.0
        for _ in range(ticks):
            fsm.tick(backend, backend.read(), DT, allow_motion=True)
            backend.plant.step(DT)
            peak = max(peak, abs(backend.read().force_n))
            if fsm.state is not MotionState.SERVO:
                break
        return peak

    def _run(self, backend, target_mm: float, ticks: int = 20000):
        fsm = MotionFSM(backend.limits, MotionParams(speed_mm_s=20.0))
        fsm.move_to_mm(target_mm, "slider")
        return fsm, self._drive(fsm, backend, ticks)

    def test_the_friction_the_lead_is_sized_for_still_arrives(self) -> None:
        backend = self._plant(self.FRICTION_NM)
        fsm, _ = self._run(backend, 60.0)

        assert fsm.state is MotionState.HOLD
        assert fsm.note == ""
        assert backend.read().position_mm == pytest.approx(60.0, abs=constants.TOL_MM)

    @pytest.mark.parametrize(
        "speed, coulomb",
        [
            (constants.SPEED_DEFAULT_MM_S, 1.0),
            (constants.SPEED_MIN_MM_S, 0.5),
        ],
    )
    def test_the_buttons_own_paths_arrive_with_friction(
        self, speed: float, coulomb: float
    ) -> None:
        """The same defect, driven the way the operator drives it.

        一键闭合, 一键张开 and a slider drop go through ``close()``, ``open()`` and
        ``move_to_mm``, and the speed is whatever the slider was left at — which
        is the point of the two rows.  The lead's share of the push is ``kp·lead``
        and does not move with the speed; the rest of it, ``kp·v·dt`` and
        ``kd·v``, does.  So the friction this mechanism can move at the top of
        the speed range is around 1.1 Nm and at the bottom around 0.55 (measured,
        in steps of 0.05); the rows sit inside both, and the slow one is where
        the lead is the difference between moving and being called jammed — with
        no lead at all, 5 mm/s stalls on 0.3 Nm, which is thirty times the
        simulated unit's own free travel.  That stall, at the start position, is
        the report this whole change exists to answer.
        """
        backend = self._plant(coulomb)
        top = backend.limits.max_stroke_mm
        for start_mm, target_mm, button in (
            (top, 0.0, "close"),
            (top, 30.0, "slider"),
            (0.0, top, "open"),
        ):
            backend = self._plant(coulomb, start_mm=start_mm)
            fsm = MotionFSM(backend.limits, MotionParams(speed_mm_s=speed))
            if button == "close":
                fsm.close()
            elif button == "open":
                fsm.open()
            else:
                fsm.move_to_mm(target_mm, "slider")
            self._drive(fsm, backend)

            where = f"{button} at {speed:.0f} mm/s on {coulomb} Nm"
            assert fsm.state is MotionState.HOLD, f"{where}: never settled"
            assert fsm.note == "", f"{where}: said {fsm.note!r}"
            assert backend.read().position_mm == pytest.approx(
                target_mm, abs=constants.TOL_MM
            ), f"{where}: stopped at {backend.read().position_mm:.2f} of {target_mm}"

    def test_the_same_mechanism_blocked_is_still_caught_and_still_light(self) -> None:
        """Blocked at 40 mm and closing from 60: the lead buys motion without
        blinding the detector, and the squeeze stays inside the derivation that
        sizes it.  The lead cannot make this case *harder* to see — it is
        titrated by the shortfall, which is small next to an obstacle met at
        20 mm/s — so the charge against the contact threshold is pinned where it
        is observable, in the profile (``test_profile.py``)."""
        backend = self._plant(self.FRICTION_NM, start_mm=60.0)
        backend.plant.object_mm = 40.0
        fsm, peak = self._run(backend, 0.0)

        assert fsm.state is MotionState.HOLD
        assert "堵转" in fsm.note
        assert backend.read().position_mm == pytest.approx(40.0, abs=1.0)
        # What a plain move may squeeze with: kp times the loss it tolerates,
        # including the lead it is allowed.  Well inside the 40 N rating.
        squeeze_n = (
            constants.KP_MOVE
            / backend.limits.rad_to_mm
            * (constants.CONTACT_LOST_MM + constants.CONTACT_LEAD_RAD * backend.limits.rad_to_mm)
            * 10.0
        )
        assert peak < squeeze_n
        assert peak < constants.FORCE_MAX_N


class TestStaleReadings:
    """A measurement is only evidence while it is current.

    Both detectors compare the trajectory against the jaws' position, and the
    position in hand is a cache: the SDK's ``get_state`` returns whatever the
    last poll brought.  A frame from several ticks ago accumulates lost motion at
    the reference speed while the jaws travel perfectly well, and its velocity
    field reads zero, so the veto that separates "lagging" from "held" cannot
    catch it either.  The worker marks the reading instead —
    :data:`~litegrip_studio.constants.CONTACT_FRESH_MS` — and what is left in
    that case is the deadline, which reports a move that did not finish rather
    than an obstruction that was never seen.
    """

    def test_a_reading_from_several_ticks_ago_is_not_an_obstruction(self) -> None:
        fsm = MotionFSM(BENCH, MotionParams(speed_mm_s=20.0))
        backend = Laggard(BENCH, ratio=0.0)

        fsm.move_to_mm(60.0, "slider")
        for _ in range(20000):
            fsm.tick(
                backend,
                backend.telemetry(),
                DT,
                allow_motion=True,
                telemetry_current=False,
            )
            if fsm.state is not MotionState.SERVO:
                break

        assert fsm.state is MotionState.HOLD
        assert "堵转" not in fsm.note
        assert "未到位" in fsm.note

    def test_the_same_frozen_axis_is_reported_once_the_reading_is_current(self) -> None:
        """The guard is a qualifier on the evidence, not a way to ignore it."""
        fsm = MotionFSM(BENCH, MotionParams(speed_mm_s=20.0))
        backend = Laggard(BENCH, ratio=0.0)

        fsm.move_to_mm(60.0, "slider")
        for _ in range(20000):
            fsm.tick(backend, backend.telemetry(), DT, allow_motion=True)
            if fsm.state is not MotionState.SERVO:
                break

        assert "堵转" in fsm.note


class TestForceControl:
    @pytest.mark.parametrize("force_n", [5.0, 12.0, 25.0, 40.0])
    def test_a_grasp_holds_the_setpoint_against_an_object(self, rig, force_n: float) -> None:
        assert rig.open()
        rig.sim.inject(obj_mm=60.0)
        rig.fsm.grasp(force_n)
        assert rig.run_until(MotionState.HOLD_FORCE, timeout_s=15.0), rig.fsm.note
        rig.run(0.5)

        assert rig.force_n == pytest.approx(force_n, abs=0.8)
        assert rig.position_mm > 55.0

    def test_the_frame_holding_a_grip_has_no_position_gain(self) -> None:
        """The deliberate divergence from the SDK.  Any position term is added to
        the feed-forward, so with a gain the delivered force would depend on how
        far the jaws sank into the object instead of on the setpoint.
        """
        fsm, backend = servo(start_mm=60.0)
        fsm.hold_force(20.0, 60.0)
        outs = run_ticks(fsm, backend, 60)
        assert all(o.kp == 0.0 for o in outs)
        assert all(o.tau_nm > 0.0 for o in outs)

    def test_the_torque_is_ramped_in_rather_than_stepped(self) -> None:
        """A torque step makes the fingers bounce off the object and the damping
        term fight the rebound — it spiked a 40 N grip to 56 N."""
        fsm, backend = servo(start_mm=60.0)
        fsm.hold_force(40.0, 60.0)
        outs = run_ticks(fsm, backend, 80)
        target = clamp_force_torque(40.0)

        assert outs[0].tau_nm < target * 0.2, "the first frame must not be a step"
        assert outs[-1].tau_nm == pytest.approx(target, rel=0.02)
        for prev, cur in zip(outs, outs[1:]):
            assert cur.tau_nm >= prev.tau_nm - 1e-12, "the ramp must be monotone"

    def test_the_velocity_reference_starts_at_the_measured_velocity(self) -> None:
        """Zeroing ``dq_cmd`` while the jaws are still moving is itself a torque
        step of ``kd·v`` — 15 N at 50 mm/s — applied to whatever was just touched.
        """
        fsm, backend = servo(start_mm=60.0)
        backend.velocity_rad_s = -2.0  # approaching at ~130 mm/s
        fsm.hold_force(20.0, 60.0)
        outs = run_ticks(fsm, backend, 40)

        velocity_refs = [f.dq_rad_s for f in backend.frames]
        assert velocity_refs[0] == pytest.approx(-2.0, rel=0.2)
        assert all(
            abs(cur) <= abs(prev) + 1e-12
            for prev, cur in zip(velocity_refs, velocity_refs[1:])
        ), "the reference must decay toward zero, never step to it"
        assert abs(velocity_refs[-1]) < 0.1
        assert len(outs) == len(velocity_refs)

    @pytest.mark.parametrize("asked", [60.0, 999.0, 1e9])
    def test_the_setpoint_cannot_exceed_the_mechanical_rating(self, asked: float) -> None:
        fsm, backend = servo(start_mm=60.0)
        fsm.hold_force(asked, 60.0)
        outs = run_ticks(fsm, backend, 80)
        assert all(o.tau_nm <= torque_from_force(constants.FORCE_MAX_N) + 1e-9 for o in outs)
        assert all(o.tau_nm > 0.0 for o in outs)
        assert fsm.force_setpoint() == constants.FORCE_MAX_N

    def test_a_negative_setpoint_is_treated_as_zero(self) -> None:
        fsm, _ = servo(start_mm=60.0)
        fsm.set_force(-10.0)
        assert fsm.params.force_n == 0.0


class TestTimeout:
    """The deadline is the third and last of the three "it is not moving" guards.

    A frozen axis trips the stall counter in 0.1 s; a jam met at cruise speed
    trips lost-motion contact detection in about 20 ms.  What is left is a move
    that never reaches cruise — so contact detection cannot see it — and that
    moves more than the stall threshold counts as progress.  That is the case
    these tests construct, because it is the only one the deadline is for.
    """

    def test_a_move_that_never_converges_stops_and_reports_it(self) -> None:
        fsm, backend = servo(start_mm=0.0)
        fsm.move_to_mm(2.0, "slider")  # far too short to reach cruise
        deadline = fsm._move_deadline(2.0)
        assert deadline == constants.STALL_TIMEOUT_MIN_S

        state = {"n": 0}

        def buzz(b: IdealPlant) -> None:
            """Buzzing in place: moving, but never any closer to the target."""
            state["n"] += 1
            b.mm = 0.05 if state["n"] % 2 else 0.0

        ticks = int(round((deadline + 0.5) / DT))
        outs = run_ticks(fsm, backend, ticks, on_tick=buzz)

        assert fsm.state is MotionState.HOLD
        assert "未到位" in fsm.note
        assert outs[-1].tau_nm == 0.0
        assert abs(fsm._elapsed - deadline) < 10 * DT

    def test_it_does_not_give_up_before_the_deadline(self) -> None:
        fsm, backend = servo(start_mm=0.0)
        fsm.move_to_mm(2.0, "slider")
        state = {"n": 0}

        def buzz(b: IdealPlant) -> None:
            state["n"] += 1
            b.mm = 0.05 if state["n"] % 2 else 0.0

        run_ticks(
            fsm, backend, int(round((constants.STALL_TIMEOUT_MIN_S - 0.2) / DT)), on_tick=buzz
        )
        assert fsm.state is MotionState.SERVO

    def test_the_slowest_offered_speed_can_complete_a_full_stroke_move(self) -> None:
        """The regression this deadline exists to fix.

        With a fixed 3 s cap, every move at the low end of the speed slider was
        aborted: a full-stroke move at 5 mm/s legitimately takes 24 s, and the
        axis would be stopped and reported as "never arrived" a fifth of the way
        through.  The deadline has to scale with the move.
        """
        fsm, backend = servo(start_mm=0.0, speed_mm_s=constants.SPEED_MIN_MM_S)
        fsm.move_to_mm(120.0, "slider")
        run_ticks(fsm, backend, 20)

        assert fsm._deadline_s > 2.0 * 120.0 / constants.SPEED_MIN_MM_S
        assert fsm._deadline_s > constants.STALL_TIMEOUT_MIN_S

        run_ticks(fsm, backend, int(round(40.0 / DT)))
        assert fsm.state is MotionState.HOLD
        assert "未到位" not in fsm.note
        assert backend.mm == pytest.approx(120.0, abs=constants.TOL_MM)

    @pytest.mark.parametrize(
        "speed,distance", [(5.0, 120.0), (50.0, 120.0), (150.0, 120.0), (150.0, 1.0)]
    )
    def test_the_deadline_always_exceeds_the_ideal_duration(
        self, speed: float, distance: float
    ) -> None:
        fsm, _ = servo(start_mm=0.0, speed_mm_s=speed)
        ideal = distance / speed + speed / constants.ACC_DEFAULT_MM_S2
        assert fsm._move_deadline(distance) >= ideal
        assert fsm._move_deadline(distance) >= constants.STALL_TIMEOUT_MIN_S
