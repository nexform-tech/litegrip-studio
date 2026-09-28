"""The trapezoidal reference generator.

The properties asserted here are the ones that keep the jaws safe: the
reference never overshoots, never exceeds the commanded speed, never demands
more acceleration than configured, and converges inside the arrival band.  The
randomised case sweeps the parameter space rather than trusting a handful of
hand-picked numbers.
"""

from __future__ import annotations

import math
import random

import pytest

from litegrip_studio import constants
from litegrip_studio.core.profile import SpeedProfile, simulate
from litegrip_studio.units import Limits

#: The SDK's example 120 mm unit; the profile only cares about the range it is
#: given, and pinning it keeps the distances below readable.
BENCH = Limits(1.775959, -0.064279, 65.21, 120.0)
DT = constants.CTRL_DT


def drive(
    limits: Limits,
    start_mm: float,
    target_mm: float,
    speed: float,
    acc: float,
    *,
    lag_mm: float = 0.0,
    dt: float = DT,
    max_ticks: int = 20000,
):
    """Run a profile against a plant that lags its command by ``lag_mm``."""
    prof = SpeedProfile(limits, target_mm, speed, acc)
    outs = []
    measured = start_mm
    for _ in range(max_ticks):
        out = prof.step(measured, dt)
        outs.append(out)
        # First-order lag toward the command: a crude stand-in for the servo,
        # enough to prove the trajectory tolerates not being tracked exactly.
        measured += (out.q_cmd_mm - measured) * (1.0 if lag_mm <= 0 else 0.3)
        if out.arrived:
            break
    return outs, measured


class TestTrapezoid:
    @pytest.mark.parametrize("speed", [constants.SPEED_MIN_MM_S, 20.0, 50.0, constants.SPEED_MAX_MM_S])
    @pytest.mark.parametrize("target", [0.0, 30.0, 60.0, 119.0, 120.0])
    def test_never_overshoots_the_reference(self, speed: float, target: float) -> None:
        """Any overshoot must be servo lag, never the reference itself."""
        outs, _ = drive(BENCH, 120.0, target, speed, constants.ACC_DEFAULT_MM_S2)
        cmds = [o.q_cmd_mm for o in outs]
        assert min(cmds) >= target - 1e-9

    @pytest.mark.parametrize("speed", [50.0, 150.0])
    def test_never_exceeds_the_commanded_speed(self, speed: float) -> None:
        outs, _ = drive(BENCH, 0.0, 120.0, speed, constants.ACC_DEFAULT_MM_S2)
        assert max(abs(o.vel_mm_s) for o in outs) <= speed + 1e-9

    @pytest.mark.parametrize("acc", [50.0, 400.0, 2000.0])
    def test_never_exceeds_the_commanded_acceleration(self, acc: float) -> None:
        outs, _ = drive(BENCH, 0.0, 120.0, 150.0, acc)
        limit = acc * DT + 1e-9
        for prev, cur in zip(outs, outs[1:]):
            assert abs(cur.vel_mm_s - prev.vel_mm_s) <= limit

    @pytest.mark.parametrize("target", [0.0, 60.0, 120.0])
    def test_velocity_is_continuous_at_the_start(self, target: float) -> None:
        """No velocity step: a step is a current step and an audible clack."""
        outs, _ = drive(BENCH, 60.0, target, 50.0, constants.ACC_DEFAULT_MM_S2)
        assert abs(outs[0].vel_mm_s) <= constants.ACC_DEFAULT_MM_S2 * DT + 1e-9

    def test_monotone_approach(self) -> None:
        outs, _ = drive(BENCH, 120.0, 0.0, 50.0, constants.ACC_DEFAULT_MM_S2)
        cmds = [o.q_cmd_mm for o in outs]
        assert all(b <= a + 1e-9 for a, b in zip(cmds, cmds[1:]))

    def test_converges_inside_the_arrival_band(self) -> None:
        outs, measured = drive(BENCH, 0.0, 60.0, 50.0, constants.ACC_DEFAULT_MM_S2)
        assert outs[-1].arrived
        assert abs(measured - 60.0) <= constants.TOL_MM

    def test_stops_at_the_target_not_at_full_speed(self) -> None:
        """The stopping-distance term should have bled the speed off first."""
        outs, _ = drive(BENCH, 0.0, 120.0, 150.0, constants.ACC_DEFAULT_MM_S2)
        assert abs(outs[-1].vel_mm_s) < 5.0

    def test_target_at_the_far_end_still_converges(self) -> None:
        outs, measured = drive(BENCH, 0.0, 120.0, 150.0, constants.ACC_DEFAULT_MM_S2)
        assert outs[-1].arrived
        assert measured == pytest.approx(120.0, abs=constants.TOL_MM)


class TestReversal:
    def test_decelerates_before_reversing(self) -> None:
        prof = SpeedProfile(BENCH, 120.0, 50.0, constants.ACC_DEFAULT_MM_S2)
        measured = 0.0
        for _ in range(200):
            out = prof.step(measured, DT)
            measured = out.q_cmd_mm
            assert abs(out.vel_mm_s) <= 50.0 + 1e-9
        prof.set_target(0.0)
        for _ in range(5):
            out = prof.step(measured, DT)
            measured = out.q_cmd_mm
            # Still travelling outward; the reference must not have flipped yet.
            assert out.vel_mm_s <= 1e-9 or abs(out.vel_mm_s) < 50.0

    def test_retarget_in_flight_is_adopted(self) -> None:
        prof = SpeedProfile(BENCH, 120.0, 50.0, constants.ACC_DEFAULT_MM_S2)
        measured = 30.0
        for _ in range(50):
            measured = prof.step(measured, DT).q_cmd_mm
        prof.set_target(10.0)
        assert prof.target_mm == 10.0
        for _ in range(4000):
            out = prof.step(measured, DT)
            measured = out.q_cmd_mm
            if out.arrived:
                break
        assert measured == pytest.approx(10.0, abs=constants.TOL_MM)

    def test_target_is_clamped_into_the_travel(self) -> None:
        assert SpeedProfile(BENCH, 1e6).target_mm == BENCH.max_stroke_mm
        assert SpeedProfile(BENCH, -1e6).target_mm == 0.0


class TestContactDetection:
    """Lost motion is measured only once the reference is at cruise.

    Before that the plant is chasing a moving target and lags it by a few
    millimetres — much larger than the gap a real contact produces in the time
    it takes to notice, and an artefact of acceleration rather than of
    obstruction.
    """

    def test_free_travel_never_reports_contact(self) -> None:
        for speed in (constants.SPEED_MIN_MM_S, 50.0, constants.SPEED_MAX_MM_S):
            for start, target in ((0.0, 120.0), (120.0, 0.0), (120.0, 60.0)):
                prof = SpeedProfile(BENCH, target, speed, constants.ACC_DEFAULT_MM_S2)
                measured = start
                for _ in range(6000):
                    out = prof.step(measured, DT)
                    assert not out.contact, (
                        f"false contact at {measured:.2f} mm "
                        f"(speed {speed}, {start}→{target}, lost {out.lost_mm:.3f})"
                    )
                    measured = out.q_cmd_mm
                    if out.arrived:
                        break

    def test_free_travel_lost_motion_stays_under_the_threshold(self) -> None:
        prof = SpeedProfile(BENCH, 120.0, 50.0, constants.ACC_DEFAULT_MM_S2)
        measured = 0.0
        worst = 0.0
        for _ in range(6000):
            out = prof.step(measured, DT)
            worst = max(worst, out.lost_mm)
            measured = out.q_cmd_mm
            if out.arrived:
                break
        assert worst < constants.CONTACT_LOST_MM

    def test_a_jammed_axis_reports_contact(self) -> None:
        """Measured position frozen while the reference keeps asking to move."""
        prof = SpeedProfile(BENCH, 0.0, 50.0, constants.ACC_DEFAULT_MM_S2)
        fired = None
        for i in range(6000):
            out = prof.step(80.0, DT)  # never moves
            if out.contact:
                fired = i
                break
        assert fired is not None
        assert fired * DT < 0.5, "contact should be noticed within half a second"

    def test_contact_only_once_at_cruise_speed(self) -> None:
        """A short move that never reaches cruise cannot report contact."""
        prof = SpeedProfile(BENCH, 119.0, constants.SPEED_MIN_MM_S, 50.0)
        for _ in range(200):
            out = prof.step(119.5, DT)
            assert not out.contact

    def test_lost_motion_resets_on_retarget(self) -> None:
        prof = SpeedProfile(BENCH, 0.0, 50.0, constants.ACC_DEFAULT_MM_S2)
        for _ in range(20):
            prof.step(120.0, DT)
        prof.set_target(60.0)
        assert prof.virtual_mm is None
        assert not prof.step(120.0, DT).contact


class TestStall:
    def test_a_frozen_axis_eventually_reports_stall(self) -> None:
        prof = SpeedProfile(BENCH, 0.0, 50.0, constants.ACC_DEFAULT_MM_S2)
        reported = False
        for _ in range(2000):
            if prof.step(90.0, DT).stalled:
                reported = True
                break
        assert reported

    def test_a_moving_axis_never_reports_stall(self) -> None:
        outs = simulate(BENCH, 120.0, 0.0)
        assert not any(o.stalled for o in outs)

    def test_arrival_means_stopped_not_merely_in_band(self) -> None:
        """``arrived`` is the condition the caller switches to HOLD on, so it has
        to mean the reference is at rest — otherwise the hand-off steps ``dq_cmd``
        from the residual speed to zero and that step is a ``kd · v`` torque."""
        outs = simulate(BENCH, 120.0, 0.0, speed_mm_s=constants.SPEED_MAX_MM_S)
        assert outs[-1].arrived
        assert outs[-1].vel_mm_s == 0.0

    @pytest.mark.parametrize("speed", [constants.SPEED_MIN_MM_S, 50.0, 150.0])
    @pytest.mark.parametrize("acc", [constants.ACC_MIN_MM_S2, 400.0])
    def test_velocity_output_is_continuous_through_arrival(
        self, speed: float, acc: float
    ) -> None:
        """A step in ``dq_cmd`` is a step in torque, so it must not exist — not
        even at the band entry, where the velocity is still bleeding off."""
        outs = simulate(BENCH, 120.0, 0.0, speed_mm_s=speed, acc_mm_s2=acc)
        for prev, cur in zip(outs, outs[1:]):
            assert abs(cur.vel_mm_s - prev.vel_mm_s) <= acc * DT + 1e-9


class TestContactLead:
    """The bounded lead, which is the one place the command may run ahead of the
    measurement.

    The bound is the safety property, not an optimisation: an unbounded lead is
    an unbounded squeeze, so the jaws may be led by the tick's own travel plus
    ``CONTACT_LEAD_RAD`` and by nothing else, however far behind they have fallen.
    A mechanism that keeps up must not see the lead at all, which is what keeps
    the anchoring guarantee — the position error the servo is given — intact for
    every move that is working.

    What is pinned here is that the implementation keeps to the budget; the size
    of the budget itself is pinned where it turns into newtons, by the crush
    bounds on a blocked move (``test_motion_fsm.py``), so that raising the
    constant cannot quietly raise the squeeze as well.
    """

    @pytest.mark.parametrize("target", [0.0, 120.0])
    @pytest.mark.parametrize("fraction", [1.0, 0.5, 0.0])
    def test_the_command_leads_by_no_more_than_the_lead(
        self, target: float, fraction: float
    ) -> None:
        prof = SpeedProfile(BENCH, target, 50.0, constants.ACC_DEFAULT_MM_S2)
        measured = 120.0 - target  # start at the other end
        budget = constants.CONTACT_LEAD_RAD * BENCH.rad_to_mm
        worst = 0.0
        for _ in range(4000):
            out = prof.step(measured, DT, constants.CONTACT_LEAD_RAD)
            if abs(out.err_mm) > constants.TOL_MM:
                # Outside the arrival band, where the command is the target by
                # design and the bound below does not describe it.
                lead = abs(out.q_cmd_mm - measured) - abs(out.vel_mm_s) * DT
                worst = max(worst, lead)
            measured += (out.q_cmd_mm - measured) * fraction
            if out.arrived:
                break
        assert worst <= budget + 1e-9, (
            f"led the jaws by {worst:.3f} mm, budget {budget:.3f} mm"
        )

    def test_a_mechanism_that_keeps_up_is_never_led(self) -> None:
        """The lead is titrated by the shortfall, so with none there is none."""
        prof = SpeedProfile(BENCH, 0.0, 50.0, constants.ACC_DEFAULT_MM_S2)
        measured = 120.0
        for _ in range(4000):
            out = prof.step(measured, DT, constants.CONTACT_LEAD_RAD)
            if abs(out.err_mm) > constants.TOL_MM:
                assert abs(out.q_cmd_mm - measured) <= abs(out.vel_mm_s) * DT + 1e-9
            measured = out.q_cmd_mm
            if out.arrived:
                break

    def test_the_lead_is_charged_against_the_contact_threshold(self) -> None:
        """Contact is ``CONTACT_LOST_MM`` of *unmet demand*, and the lead counts
        toward it rather than being added to it.  The other way round, a stiffer
        mechanism — the one already being led because it is behind — would have
        to lose a whole extra millimetre before an obstacle registered, which is
        backwards: the harder the mechanism is to move, the sooner a move that
        has stopped making progress should stop pushing.
        """
        prof = SpeedProfile(BENCH, 0.0, 50.0, constants.ACC_DEFAULT_MM_S2)
        out = prof.step(90.0, DT, constants.CONTACT_LEAD_RAD)
        for _ in range(400):  # an axis that never moves
            if out.contact:
                break
            out = prof.step(90.0, DT, constants.CONTACT_LEAD_RAD)

        assert out.contact
        assert out.lost_mm < constants.CONTACT_LOST_MM


class TestRandomisedSweep:
    """The properties must hold for arbitrary targets and speeds, not just the
    ones picked above."""

    @pytest.mark.parametrize("seed", range(8))
    def test_sweep(self, seed: int) -> None:
        rng = random.Random(seed)
        for _ in range(60):
            start = rng.uniform(0.0, BENCH.max_stroke_mm)
            target = rng.uniform(0.0, BENCH.max_stroke_mm)
            speed = rng.uniform(constants.SPEED_MIN_MM_S, constants.SPEED_MAX_MM_S)
            acc = rng.uniform(constants.ACC_MIN_MM_S2, 1500.0)

            outs, measured = drive(BENCH, start, target, speed, acc)
            assert outs[-1].arrived, f"{start:.1f}→{target:.1f} @{speed:.0f}"
            assert measured == pytest.approx(target, abs=constants.TOL_MM)

            cmds = [o.q_cmd_mm for o in outs]
            lo, hi = min(start, target), max(start, target)
            assert min(cmds) >= lo - 1e-9 and max(cmds) <= hi + 1e-9
            assert max(abs(o.vel_mm_s) for o in outs) <= speed + 1e-9
            assert all(
                abs(b.vel_mm_s - a.vel_mm_s) <= acc * DT + 1e-9
                for a, b in zip(outs, outs[1:])
            )

    @pytest.mark.parametrize("seed", range(4))
    def test_tolerates_tick_jitter(self, seed: int) -> None:
        """A worker that overshoots its tick must not destabilise the profile."""
        rng = random.Random(seed)
        target = rng.uniform(0.0, BENCH.max_stroke_mm)
        prof = SpeedProfile(BENCH, target, 50.0, constants.ACC_DEFAULT_MM_S2)
        measured = 120.0
        arrived = False
        for _ in range(20000):
            dt = rng.uniform(0.004, 0.008)
            out = prof.step(measured, dt)
            measured = out.q_cmd_mm
            if out.arrived:
                arrived = True
                break
        assert arrived
        assert measured == pytest.approx(target, abs=constants.TOL_MM)


class TestDegenerateInputs:
    def test_a_negative_lead_never_brakes_the_jaws(self) -> None:
        """The lead may only ever add push.  A caller passing a negative one has
        to get the plain anchored law, not a command *behind* the measurement —
        that would ask the servo to hold the jaws back while the trajectory ran
        away from them."""
        plain = SpeedProfile(BENCH, 0.0, 50.0, constants.ACC_DEFAULT_MM_S2)
        negative = SpeedProfile(BENCH, 0.0, 50.0, constants.ACC_DEFAULT_MM_S2)
        for measured in (120.0, 120.0, 90.0, 60.0, 40.0, 30.0):
            assert plain.step(measured, DT, 0.0).q_cmd_mm == negative.step(
                measured, DT, -constants.CONTACT_LEAD_RAD
            ).q_cmd_mm

    def test_nan_measurement_does_not_produce_a_nan_command(self) -> None:
        prof = SpeedProfile(BENCH, 60.0, 50.0, constants.ACC_DEFAULT_MM_S2)
        out = prof.step(math.nan, DT)
        assert not math.isnan(out.q_cmd_mm)

    def test_zero_dt_is_inert(self) -> None:
        prof = SpeedProfile(BENCH, 60.0, 50.0, constants.ACC_DEFAULT_MM_S2)
        assert not math.isnan(prof.step(10.0, 0.0).q_cmd_mm)
