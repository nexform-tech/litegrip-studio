"""The two calibration probes, driven through synthetic telemetry.

Both probes are pure: :meth:`tick` takes a measured angle and an elapsed time
and returns the frame to send.  Everything here is therefore a real run of the
probe — including the parts that on hardware are a stall against a hard stop or
a lost feedback frame, which are the parts that actually decide whether the
probe is safe.

The plant used throughout is deliberately trivial: ``Plant`` moves the jaws
toward the last commanded angle at a fixed rate, and stops dead at the limits —
which is exactly the behaviour that makes a probe necessary.
"""

from __future__ import annotations

import math

import pytest

from litegrip_studio import constants
from litegrip_studio.core.calibration_fsm import (
    FREE_FRAME,
    MANUAL_MAX_JUMP_RAD_PER_TICK,
    NO_FRAME,
    PROBE_MAX_JUMP_RAD_PER_TICK,
    CalibResult,
    GuidedCalibFSM,
    GuidedPhase,
    TwoPointCalibFSM,
    TwoPointPhase,
    summarise,
)
from litegrip_studio.units import Limits, torque_from_force

DT = constants.CTRL_DT
HZ = constants.CTRL_HZ

#: Where a real gripper's stops are, taken from the user calibration in
#: tests/test_calibration.py: closed is the larger angle, open the smaller.
CLOSED_RAD = 1.775959
OPEN_RAD = -0.064279

#: The same two stops on a gripper whose fingers are mounted the other way up,
#: so the angle *grows* as the jaws open.  These are the two readings the bench
#: unit this console was debugged against stands at, and on that unit the file in
#: force had them the other way round — which is why commanding 闭合 opened it.
REVERSED_CLOSED_RAD = -0.300793
REVERSED_OPEN_RAD = 1.421569


class Plant:
    """Jaws that follow the commanded angle, at a rate, up to hard stops."""

    def __init__(
        self,
        pos_rad: float,
        *,
        rate_rad_s: float = 0.5,
        open_rad: float = OPEN_RAD,
        closed_rad: float = CLOSED_RAD,
        frozen: bool = False,
    ) -> None:
        self.pos = pos_rad
        self.rate = rate_rad_s
        self.open_rad = open_rad
        self.closed_rad = closed_rad
        self.frozen = frozen
        self.last_q: float | None = None

    def apply(self, out) -> None:
        """Fold one probe frame in: a position command moves us, torque does not.

        Only ``kp`` terms move this plant.  That is a simplification of the MIT
        law, and the safe direction of one: it means a probe holding at the open
        stop with the reference one step past it still stops at the stop, so a
        test cannot pass by the plant wandering.
        """
        if out.q_rad is None:
            return
        self.last_q = out.q_rad
        if out.kp <= 0.0 or self.frozen:
            return
        error = out.q_rad - self.pos
        step = max(-self.rate * DT, min(self.rate * DT, error))
        # Order-free, because the two stops are named by role here rather than
        # by their position in the angle: a reverse-mounted plant has its closed
        # stop at the smaller angle, and a clamp written as ``min(closed, ...)``
        # would put that gripper's travel between its two stops the wrong way up
        # — the mirrors of each other, and the difference a sign test cannot see.
        low = min(self.open_rad, self.closed_rad)
        high = max(self.open_rad, self.closed_rad)
        self.pos = min(high, max(low, self.pos + step))

    def tick(self, fsm, dt: float = DT):
        """One control tick: send the probe's frame, then report where we are."""
        out = fsm.tick(self.pos, dt)
        self.apply(out)
        return out


def run(fsm, plant: Plant, *, seconds: float, until=None, dt: float = DT):
    """Run the loop, stopping early when ``until(fsm)`` holds.

    Returns the number of ticks, and the probe's own phase is the caller's to
    read — the point is to drive it until something happens, not to count.
    """
    ticks = int(seconds / dt)
    frames = 0
    for _ in range(ticks):
        frames += 1
        plant.tick(fsm, dt)
        if until is not None and until(fsm):
            break
    return frames


def run_to_terminal(fsm, plant: Plant, *, seconds: float = 60.0):
    run(fsm, plant, seconds=seconds, until=lambda f: f.is_terminal)
    assert fsm.is_terminal, f"probe did not finish within {seconds}s: {fsm.note}"


def walk(fsm, from_rad: float, to_rad: float, *, rate_rad_s: float = 1.0):
    """Feed the probe a hand-move from one angle to another, tick by tick.

    A manual probe is sampled every 5 ms, and its jump guard refuses a reading
    that moved further than the mechanism could in that time — so a test that
    teleports from one end of the travel to the other in a single tick is not
    testing the probe, it is testing the guard.  At a metre per second this takes
    the jaws across the whole travel in ~1.8 s.
    """
    pos = from_rad
    step = rate_rad_s * DT
    ticks = int(abs(to_rad - from_rad) / step) + 1
    for _ in range(ticks):
        pos += max(-step, min(step, to_rad - pos))
        fsm.tick(pos, DT)
    return pos


# ═══════════════════════════════════════════════════════════════════════════
# The result summary
# ═══════════════════════════════════════════════════════════════════════════
class TestSummarise:
    def test_a_normal_probe_produces_the_sdk_file_shape(self) -> None:
        """``travel = zero - open`` must hold exactly, as it does in the SDK's own
        files, or validation depends on which field it happens to read."""
        result = summarise(CLOSED_RAD, OPEN_RAD, 120.0, ())
        assert isinstance(result, CalibResult)
        assert result.zero_rad == round(CLOSED_RAD, 6)
        assert result.open_rad == round(OPEN_RAD, 6)
        assert result.travel_rad == result.zero_rad - result.open_rad
        assert result.as_raw()["travel_range_rad"] == result.travel_rad

    def test_the_conversion_factor_is_derived_from_the_measured_travel(self) -> None:
        """``(travel + inset) / travel``, not the SDK's hardcoded 120.0/travel and
        not the operator's travel over travel either: the probe presses a
        millimetre into the open stop, so the span it recorded is that much wider
        than the travel the jaws actually have."""
        result = summarise(CLOSED_RAD, OPEN_RAD, 200.0, ())
        assert isinstance(result, CalibResult)
        assert result.rad_to_mm == pytest.approx(
            (200.0 + constants.SPAN_INSET_MM) / result.travel_rad, abs=0.01
        )

    def test_the_recorded_span_is_the_travel_plus_the_inset(self) -> None:
        result = summarise(CLOSED_RAD, OPEN_RAD, 120.0, ())
        assert isinstance(result, CalibResult)
        assert result.stroke_mm == pytest.approx(120.0 + constants.SPAN_INSET_MM, abs=0.05)
        # Which is the same statement as "0 mm is the recorded closed extreme,
        # and the commanded range ends at the measured travel".
        assert result.as_raw()["rad_to_mm"] > 120.0 / result.travel_rad

    def test_a_reversed_pair_is_recorded_as_given(self) -> None:
        """The other mounting, and the one the two-point capture produces on it.

        Nothing here may normalise the two angles into the SDK's ordering: the
        file is the record of where the jaws stopped, and swapping them to look
        familiar is how a correct calibration becomes an inverted one.
        """
        result = summarise(OPEN_RAD, CLOSED_RAD, 120.0, ())
        assert isinstance(result, CalibResult)
        assert result.zero_rad == round(OPEN_RAD, 6), "the closed stop is the one given"
        assert result.open_rad == round(CLOSED_RAD, 6)
        assert result.travel_rad == result.open_rad - result.zero_rad
        assert result.travel_rad > 0.0
        assert Limits(OPEN_RAD, CLOSED_RAD, result.rad_to_mm).reversed_mount

    def test_the_same_two_angles_either_way_round_differ_only_in_order(self) -> None:
        """So the ordering carries no information the numbers do not already have,
        which is why refusing one of the two orderings refused a real gripper."""
        classic = summarise(CLOSED_RAD, OPEN_RAD, 120.0, ())
        flipped = summarise(OPEN_RAD, CLOSED_RAD, 120.0, ())
        assert isinstance(classic, CalibResult) and isinstance(flipped, CalibResult)
        assert classic.travel_rad == flipped.travel_rad
        assert classic.rad_to_mm == flipped.rad_to_mm

    def test_a_zero_range_is_refused(self) -> None:
        out = summarise(0.5, 0.5, 120.0, ())
        assert isinstance(out, str)
        assert "行程异常" in out


# ═══════════════════════════════════════════════════════════════════════════
# Guided probe
# ═══════════════════════════════════════════════════════════════════════════
class TestGuidedStepBound:
    """The probe's one safety-relevant number: how hard it can press."""

    def test_the_step_is_derived_from_the_force_rating(self) -> None:
        fsm = GuidedCalibFSM()
        assert fsm.step_rad == pytest.approx(
            torque_from_force(constants.FORCE_MAX_N) / constants.GUIDED_KP
        )

    def test_a_stalled_step_asks_for_no_more_than_the_rating(self) -> None:
        """At a stall the MIT law's torque is exactly kp × step — the velocity
        term has nothing left to damp.  The SDK's own 0.08 rad at kp=60 asks for
        4.8 Nm (48 N) against a 40 N rating."""
        fsm = GuidedCalibFSM()
        assert fsm.step_rad * fsm.kp <= torque_from_force(constants.FORCE_MAX_N) + 1e-12

    def test_the_derived_step_still_covers_a_full_stroke_in_max_iter(self) -> None:
        """The bound is only useful if it can still reach both stops."""
        fsm = GuidedCalibFSM()
        assert fsm.step_rad * fsm.max_iter > 1.9

    def test_a_gripper_with_a_lower_rating_gets_a_smaller_step(self) -> None:
        hard = GuidedCalibFSM(max_force_n=10.0)
        soft = GuidedCalibFSM(max_force_n=40.0)
        assert hard.step_rad < soft.step_rad

    def test_an_explicitly_smaller_step_is_honoured(self) -> None:
        fsm = GuidedCalibFSM(step_rad=0.01)
        assert fsm.step_rad == pytest.approx(0.01)

    def test_the_press_never_exceeds_the_rating_through_a_real_stall(self) -> None:
        """The bound above is arithmetic; this is the same claim measured over a
        probe that actually jams, which is where a regression would hide."""
        fsm = GuidedCalibFSM()
        plant = Plant(0.5, frozen=True)
        worst = 0.0
        for _ in range(HZ * 10):
            out = plant.tick(fsm)
            if out.kp > 0.0 and out.q_rad is not None:
                worst = max(worst, abs(out.q_rad - plant.pos) * out.kp)
            if fsm.is_terminal:
                break
        assert worst <= torque_from_force(constants.FORCE_MAX_N) + 1e-12


class TestGuidedProbe:
    def test_it_finds_both_stops_and_produces_a_usable_result(self) -> None:
        fsm = GuidedCalibFSM()
        assert fsm.start(0.5)
        plant = Plant(0.5)
        run_to_terminal(fsm, plant)
        assert fsm.phase is GuidedPhase.DONE, fsm.note
        result = fsm.result
        assert result is not None
        # The stalls are detected slightly before the true stop, so the limits
        # are inside the travel rather than exactly on it.
        assert result.open_rad >= OPEN_RAD - 0.05
        assert result.zero_rad <= CLOSED_RAD + 0.05
        assert result.travel_rad > 1.5
        assert result.stroke_mm == pytest.approx(
            fsm.max_stroke_mm + constants.SPAN_INSET_MM, abs=0.5
        )

    def test_the_probe_visits_open_before_closed(self) -> None:
        """Order matters: the back-off only makes sense after the open stop."""
        seen: list[GuidedPhase] = []
        fsm = GuidedCalibFSM()
        fsm.start(0.5)
        plant = Plant(0.5)
        for _ in range(HZ * 60):
            plant.tick(fsm)
            if not seen or seen[-1] is not fsm.phase:
                seen.append(fsm.phase)
            if fsm.is_terminal:
                break
        assert seen == [
            GuidedPhase.OPEN_PROBE,
            GuidedPhase.OPEN_BACKOFF,
            GuidedPhase.CLOSE_PROBE,
            GuidedPhase.DONE,
        ]

    def test_a_frozen_axis_terminates_on_the_stall_detector(self) -> None:
        """Nothing tells the probe the jaws have stopped except that they have
        stopped, so this is the only automatic ending there is."""
        fsm = GuidedCalibFSM()
        fsm.start(0.0)
        plant = Plant(0.0, frozen=True)
        plant.tick(fsm)
        assert fsm.phase is GuidedPhase.OPEN_PROBE, "should not end on one tick"
        run(fsm, plant, seconds=60.0, until=lambda f: f.phase is not GuidedPhase.OPEN_PROBE)
        # Read from the note rather than from the counter: entering the back-off
        # resets the counter, and the note is the durable record of why the limit
        # was taken — which is what the operator sees.
        assert "未移动" in fsm.note

    def test_the_stall_detector_takes_the_expected_number_of_steps(self) -> None:
        """Long enough to be sure the jaws really have stopped, short enough not
        to spend a minute pressing into a stop that the probe has already found."""
        fsm = GuidedCalibFSM()
        fsm.start(0.0)
        plant = Plant(0.0, frozen=True)
        run(fsm, plant, seconds=60.0, until=lambda f: f.phase is not GuidedPhase.OPEN_PROBE)
        assert f"{constants.GUIDED_STALL_CYCLES} 步未移动" in fsm.note
        assert constants.GUIDED_STALL_CYCLES * constants.GUIDED_STEP_INTERVAL_S < 2.0

    def test_an_axis_frozen_at_both_stops_fails_rather_than_saving_a_degenerate_file(
        self,
    ) -> None:
        """Both probes stall at the same place, so the "travel" is zero.  Saving
        that would give a gripper whose every position is the same position."""
        fsm = GuidedCalibFSM()
        fsm.start(0.0)
        plant = Plant(0.0, frozen=True)
        run_to_terminal(fsm, plant)
        assert fsm.phase is GuidedPhase.FAILED
        assert "行程异常" in fsm.note
        assert fsm.result is None

    def test_the_operator_can_confirm_a_limit_early(self) -> None:
        fsm = GuidedCalibFSM()
        fsm.start(0.5)
        plant = Plant(0.5)
        for _ in range(20):
            plant.tick(fsm)
        fsm.confirm()
        plant.tick(fsm)
        assert fsm.phase is GuidedPhase.OPEN_BACKOFF
        assert fsm.open_rad == pytest.approx(plant.pos, abs=0.02)
        assert fsm.open_rad > OPEN_RAD + 0.3, "confirmed well short of the real stop"

    def test_a_confirmation_before_a_move_is_recorded_is_still_consumed(self) -> None:
        """Otherwise the flag would survive into the close probe and take the
        wrong limit."""
        fsm = GuidedCalibFSM()
        fsm.start(0.5)
        fsm.confirm()
        plant = Plant(0.5)
        plant.tick(fsm)
        assert not fsm._confirmed
        assert fsm.phase is GuidedPhase.OPEN_BACKOFF

    def test_a_confirmation_outside_a_probe_is_ignored(self) -> None:
        fsm = GuidedCalibFSM()
        fsm.confirm()
        assert not fsm._confirmed
        fsm.start(0.5)
        fsm.cancel()
        fsm.confirm()
        assert not fsm._confirmed

    def test_the_reference_never_runs_ahead_of_the_measurement(self) -> None:
        """The anti-windup property, stated as the thing that matters: on a
        jammed axis the commanded position stays within one step of where the
        jaws actually are, so the press cannot grow."""
        fsm = GuidedCalibFSM()
        fsm.start(0.5)
        plant = Plant(0.5, frozen=True)
        for _ in range(HZ * 5):
            out = plant.tick(fsm)
            if out.q_rad is not None:
                assert abs(out.q_rad - plant.pos) <= fsm.step_rad + 1e-12
            if fsm.is_terminal:
                break

    def test_a_confirm_at_a_stall_ends_the_probe_immediately(self) -> None:
        """The operator's confirmation must beat the automatic detector, or the
        button would appear not to work on a gripper that is already jammed."""
        fsm = GuidedCalibFSM()
        fsm.start(0.5)
        plant = Plant(0.5, frozen=True)
        for _ in range(5):
            plant.tick(fsm)
        fsm.confirm()
        plant.tick(fsm)
        assert fsm.phase is GuidedPhase.OPEN_BACKOFF
        assert fsm.stalls == 0

    @pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
    def test_a_non_finite_reading_stops_the_probe(self, bad: float) -> None:
        fsm = GuidedCalibFSM()
        fsm.start(0.5)
        fsm.tick(bad, DT)
        assert fsm.phase is GuidedPhase.FAILED
        assert "读数无效" in fsm.note

    @pytest.mark.parametrize("bad", [float("nan"), float("inf")])
    def test_a_probe_cannot_start_on_a_non_finite_reading(self, bad: float) -> None:
        fsm = GuidedCalibFSM()
        assert fsm.start(bad) is False
        assert fsm.phase is GuidedPhase.FAILED

    def test_a_jumping_reading_stops_the_probe_before_it_is_adopted(self) -> None:
        """The probe steers by this number and derives the file from it, so a
        reading that could not have happened must not be used for either."""
        fsm = GuidedCalibFSM()
        fsm.start(0.0)
        for _ in range(10):
            fsm.tick(0.0, DT)
            assert not fsm.is_terminal
        fsm.tick(0.0, DT)
        fsm.tick(-3.0, DT)  # a wrap, or a frame from another CAN id
        for _ in range(HZ):
            fsm.tick(-3.0, DT)
            if fsm.is_terminal:
                break
        assert fsm.phase is GuidedPhase.FAILED
        assert "不可信" in fsm.note

    def test_a_reading_that_moves_at_the_rate_of_the_mechanism_is_fine(self) -> None:
        """The guard must not fire on the fastest legal move the console offers,
        which is where a threshold set by eye would."""
        fastest_rad_per_tick = (
            constants.SPEED_MAX_MM_S / constants.RAD_TO_MM_MIN
        ) * DT
        assert fastest_rad_per_tick < PROBE_MAX_JUMP_RAD_PER_TICK / 10

    @pytest.mark.parametrize("start_rad", [1.5, 0.9, 0.0, OPEN_RAD, CLOSED_RAD])
    def test_no_legitimate_probe_trips_the_jump_guard(self, start_rad: float) -> None:
        """Including a probe that starts *at* a stop, which is where an absolute
        bound on how far the probe may travel refuses a legal probe: the run
        legitimately covers a whole travel plus the stall overshoot."""
        fsm = GuidedCalibFSM()
        assert fsm.start(start_rad)
        plant = Plant(start_rad)
        run_to_terminal(fsm, plant, seconds=120.0)
        assert fsm.phase is GuidedPhase.DONE, fsm.note

    def test_a_legitimate_probe_does_not_trip_the_jump_guard_at_full_stroke(self) -> None:
        fsm = GuidedCalibFSM(max_stroke_mm=200.0)
        fsm.start(0.9)
        plant = Plant(0.9, rate_rad_s=1.0)
        run_to_terminal(fsm, plant, seconds=120.0)
        assert fsm.phase is GuidedPhase.DONE, fsm.note

    def test_the_back_off_moves_away_from_the_open_stop(self) -> None:
        """The SDK's own back-off commands ``open_rad - 0.15``, which is further
        into the stop — the sign is wrong and it presses at kp=80."""
        fsm = GuidedCalibFSM()
        fsm.start(0.5)
        plant = Plant(0.5)
        run(fsm, plant, seconds=60.0, until=lambda f: f.phase is GuidedPhase.OPEN_BACKOFF)
        assert fsm.phase is GuidedPhase.OPEN_BACKOFF
        at_stop = plant.pos
        run(fsm, plant, seconds=10.0, until=lambda f: f.phase is GuidedPhase.CLOSE_PROBE)
        assert plant.pos >= at_stop - 1e-9, "backed off further into the open stop"
        assert plant.pos - at_stop >= fsm.backoff_rad * 0.5

    def test_the_back_off_terminates_even_if_the_jaws_do_not_move(self) -> None:
        """Judged on the reference, so a seized axis cannot loop here forever."""
        fsm = GuidedCalibFSM()
        fsm.start(0.5)
        plant = Plant(0.5)
        run(fsm, plant, seconds=60.0, until=lambda f: f.phase is GuidedPhase.OPEN_BACKOFF)
        assert fsm.phase is GuidedPhase.OPEN_BACKOFF
        plant.frozen = True
        steps = int(fsm.backoff_rad / fsm.step_rad) + 4
        budget = int(steps * fsm.step_interval_s / DT) + 10
        for _ in range(budget):
            plant.tick(fsm)
            if fsm.phase is not GuidedPhase.OPEN_BACKOFF:
                break
        assert fsm.phase is not GuidedPhase.OPEN_BACKOFF, "back-off did not terminate"

    def test_cancelling_ends_the_probe_and_frees_the_axis(self) -> None:
        fsm = GuidedCalibFSM()
        fsm.start(0.5)
        plant = Plant(0.5)
        for _ in range(20):
            plant.tick(fsm)
        fsm.cancel()
        out = plant.tick(fsm)
        assert fsm.phase is GuidedPhase.CANCELLED
        assert out.kp == 0.0 and out.kd == 0.0 and out.tau_nm == 0.0

    def test_every_terminal_phase_sends_a_frame_that_lets_go(self) -> None:
        """Sending nothing would leave the motor executing the last frame, which
        during a probe is a press into a hard stop."""
        for end in ("cancel", "fail", "done"):
            fsm = GuidedCalibFSM()
            fsm.start(0.5)
            plant = Plant(0.5)
            if end == "fail":
                fsm.tick(float("nan"), DT)
            elif end == "cancel":
                fsm.cancel()
            else:
                run_to_terminal(fsm, plant)
            assert fsm.is_terminal
            out = fsm.tick(plant.pos, DT)
            assert out.q_rad is not None, f"{end}: terminal probe went silent"
            assert out.kp == 0.0 and out.kd == 0.0 and out.tau_nm == 0.0

    def test_an_idle_probe_sends_nothing(self) -> None:
        """The motor may be running someone else's frame; an unstarted probe has
        no business overwriting it."""
        assert GuidedCalibFSM().tick(0.5, DT) is NO_FRAME

    def test_it_does_not_move_the_jaws_before_it_is_started(self) -> None:
        fsm = GuidedCalibFSM()
        plant = Plant(0.5)
        for _ in range(100):
            plant.tick(fsm)
        assert plant.pos == 0.5

    def test_each_probe_moves_toward_its_own_stop(self) -> None:
        """Open is the numerically *smaller* angle and closed the larger, which
        is the reversal that makes an uncalibrated SDK config dangerous."""
        fsm = GuidedCalibFSM()
        fsm.start(0.5)
        plant = Plant(0.5)
        open_refs: list[float] = []
        close_refs: list[float] = []
        for _ in range(HZ * 120):
            tick_ref = fsm.target_rad
            plant.tick(fsm)
            if tick_ref is None:
                continue
            if fsm.phase is GuidedPhase.OPEN_PROBE:
                open_refs.append(tick_ref)
            elif fsm.phase is GuidedPhase.CLOSE_PROBE:
                close_refs.append(tick_ref)
            if fsm.is_terminal:
                break
        assert open_refs and close_refs
        assert open_refs[-1] < open_refs[0], "open probe did not move toward open"
        assert close_refs[-1] > close_refs[0], "close probe did not move toward closed"

    def test_progress_only_ever_advances(self) -> None:
        """A progress bar that goes backwards reads as a fault."""
        fsm = GuidedCalibFSM()
        fsm.start(0.5)
        plant = Plant(0.5, rate_rad_s=1.0)
        last = -1.0
        for _ in range(HZ * 90):
            plant.tick(fsm)
            assert fsm.progress >= last
            last = fsm.progress
            if fsm.is_terminal:
                break
        assert fsm.progress == 1.0

    def test_the_target_is_reported_for_the_gui(self) -> None:
        fsm = GuidedCalibFSM()
        assert fsm.target_rad is None
        fsm.start(0.5)
        assert fsm.target_rad is not None and fsm.target_rad != 0.5
        plant = Plant(0.5)
        run_to_terminal(fsm, plant)
        assert fsm.target_rad == pytest.approx(plant.pos)

    def test_is_active_tracks_start_and_finish(self) -> None:
        fsm = GuidedCalibFSM()
        assert not fsm.is_active
        fsm.start(0.5)
        assert fsm.is_active
        fsm.cancel()
        assert not fsm.is_active

    def test_a_restart_clears_the_previous_attempt(self) -> None:
        """The page offers "try again"; the second run must not inherit the
        first one's limits or its notes."""
        fsm = GuidedCalibFSM()
        fsm.start(0.5)
        plant = Plant(0.5, frozen=True)
        run_to_terminal(fsm, plant)
        assert fsm.is_terminal
        assert fsm.start(0.5)
        assert fsm.open_rad is None and fsm.close_rad is None and fsm.result is None
        assert fsm.iterations == 0 and fsm.stalls == 0

    @pytest.mark.parametrize("junk", [0.0, 1e300, -1e300, float("nan"), float("inf")])
    def test_tick_is_total(self, junk: float) -> None:
        """The worker calls this every 5 ms inside a ``BaseException`` guard, and
        an exception escaping ``QThread.run`` aborts the process — so a tick that
        cannot be reasoned about must still return a frame."""
        fsm = GuidedCalibFSM()
        assert fsm.tick(junk, DT) is NO_FRAME
        fsm.start(0.5)
        out = fsm.tick(junk, DT)
        assert out.q_rad is None or math.isfinite(out.q_rad)


class TestTheDeclaredMounting:
    """Which way the jaws open is the operator's to state, and it is the one
    input the probe cannot read.

    It has nothing to read it from: the probe exists because there is no usable
    file, and the stops it is walking toward are indistinguishable from each
    other until it has pressed one.  So the flag is not a hint the probe may
    second-guess — it is the sign every step is taken in, and a probe that
    assumed the SDK's direction instead would walk a reverse-mounted gripper
    into the stop it is not looking for, record the closed end as the open one,
    and produce a result that validates, saves, and drives the gripper inverted.
    """

    def test_the_open_probe_steps_the_way_the_declared_mounting_opens(self) -> None:
        """Both directions, from the same starting reading, or the test would
        pass on a probe that stepped one way regardless."""
        classic = GuidedCalibFSM()
        classic.start(0.5)
        reversed_ = GuidedCalibFSM(reversed_mount=True)
        reversed_.start(0.5)

        toward_classic = classic.tick(0.5, DT).q_rad
        toward_reversed = reversed_.tick(0.5, DT).q_rad

        assert toward_classic is not None and toward_reversed is not None
        assert toward_classic < 0.5 < toward_reversed, (
            "两个声明下第一步都朝同一个方向，说明方向没有真的被采用"
        )

    def test_the_flag_is_the_direction_it_says_it_is(self) -> None:
        """``+1`` means the angle grows toward open, which is the convention the
        rest of the console names ``Limits.reversed_mount``."""
        assert GuidedCalibFSM(reversed_mount=True).direction == 1.0
        assert GuidedCalibFSM().direction == -1.0
        # Defaults to the classic mounting, so every caller that has nothing to
        # say about it gets the SDK's own direction rather than an error.
        assert not GuidedCalibFSM().reversed_mount

    def test_the_status_line_names_the_mounting(self) -> None:
        """The operator has to be able to see, while it runs, which way the
        probe believes it is going — it is the one input they supplied."""
        fsm = GuidedCalibFSM(reversed_mount=True)
        fsm.start(0.0)

        assert "反向" in fsm.note

    def test_a_wrong_declaration_is_not_something_the_probe_can_notice(self) -> None:
        """Pinned as a limitation rather than left implied, because it is what
        the mounting question on the page is for: declared reversed on a
        classic gripper, the probe walks to the *closed* stop first, records it
        as the open one, and finishes with a complete, self-consistent,
        inverted result.  Nothing in the angles gives it away."""
        fsm = GuidedCalibFSM(reversed_mount=True)
        assert fsm.start(0.9)
        plant = Plant(0.9)  # the classic mounting: closed at the larger angle
        run_to_terminal(fsm, plant, seconds=120.0)

        assert fsm.phase is GuidedPhase.DONE, fsm.note
        result = fsm.result
        assert result is not None
        # "Open" is the closed stop and "closed" is the open one — the mirror
        # image of the right answer, and indistinguishable from it by inspection.
        assert result.open_rad > result.zero_rad
        assert Limits(
            result.zero_rad, result.open_rad, result.rad_to_mm
        ).reversed_mount, "错的方向应当产出一份反向标定，这正是它危险的地方"


class TestGuidedAgainstTheSimulatedStops:
    """The probe against the plant's real hard stops, both directions."""

    @pytest.mark.parametrize("start_rad", [1.5, 0.9, 0.5, 0.0, -0.05])
    def test_it_records_the_travel_from_any_starting_point(self, start_rad: float) -> None:
        fsm = GuidedCalibFSM()
        assert fsm.start(start_rad)
        plant = Plant(start_rad)
        run_to_terminal(fsm, plant, seconds=120.0)
        assert fsm.phase is GuidedPhase.DONE, fsm.note
        result = fsm.result
        assert result is not None
        assert result.travel_rad > 1.5
        assert OPEN_RAD <= result.open_rad < result.zero_rad <= CLOSED_RAD

    @pytest.mark.parametrize("start_rad", [1.0, 0.5, 0.0, -0.2])
    def test_a_reverse_mounted_gripper_is_probed_the_other_way_up(
        self, start_rad: float
    ) -> None:
        """The whole point of the flag: on a gripper whose angle grows as the
        jaws open, the probe has to record the *larger* angle as the open one,
        and that is the same probe with one sign flipped."""
        fsm = GuidedCalibFSM(reversed_mount=True)
        assert fsm.start(start_rad)
        plant = Plant(
            start_rad, open_rad=REVERSED_OPEN_RAD, closed_rad=REVERSED_CLOSED_RAD
        )
        run_to_terminal(fsm, plant, seconds=120.0)

        assert fsm.phase is GuidedPhase.DONE, fsm.note
        result = fsm.result
        assert result is not None
        assert result.travel_rad > 1.5
        # Both stops were found, and each on the right side of the other.
        assert result.zero_rad <= REVERSED_CLOSED_RAD + 0.05
        assert result.open_rad >= REVERSED_OPEN_RAD - 0.05
        # And the result is a reverse-mounted calibration, which is what the
        # console has to be able to save and move by.
        limits = Limits(result.zero_rad, result.open_rad, result.rad_to_mm)
        assert limits.reversed_mount
        assert limits.to_rad(0.0) == pytest.approx(result.zero_rad)
        assert limits.rad_to_mm > 0.0
        assert limits.stroke_mm == pytest.approx(
            fsm.max_stroke_mm + constants.SPAN_INSET_MM, abs=0.5
        )
        # Commanding the top of the range drives *up* in angle here, and the
        # jaws end up open — the check that the saved file is not inverted.
        assert limits.to_rad(limits.max_stroke_mm) > result.zero_rad

    @pytest.mark.parametrize("travel_mm", [85.0, 120.0, 200.0, 60.0])
    def test_the_coefficient_follows_the_configured_travel(self, travel_mm: float) -> None:
        """A probe run to calibrate a 60 mm gripper and a 200 mm one differ only
        in the millimetres they derive, because that is the one number the probes
        cannot measure."""
        fsm = GuidedCalibFSM(max_stroke_mm=travel_mm)
        fsm.start(0.9)
        plant = Plant(0.9)
        run_to_terminal(fsm, plant, seconds=120.0)
        assert fsm.phase is GuidedPhase.DONE, fsm.note
        result = fsm.result
        assert result is not None
        assert result.max_stroke_mm == travel_mm
        assert result.rad_to_mm == pytest.approx(
            (travel_mm + constants.SPAN_INSET_MM) / result.travel_rad, abs=0.02
        )
        assert result.stroke_mm == pytest.approx(travel_mm + constants.SPAN_INSET_MM, abs=0.5)




# ═══════════════════════════════════════════════════════════════════════════
# Two-point manual probe
# ═══════════════════════════════════════════════════════════════════════════
def press_open(fsm, at_rad: float) -> None:
    """Record the open point where the jaws are, the way the page does."""
    assert fsm.record_open(), "the probe was not waiting for the open point"
    fsm.tick(at_rad, DT)


def press_close(fsm, at_rad: float) -> None:
    """Record the closed point — the end that becomes 0 mm."""
    assert fsm.record_close(), "the probe was not waiting for the closed point"
    fsm.tick(at_rad, DT)


class TestTheTwoPointProbe:
    """The flow the operator drives by hand: two labelled points, no clock.

    Every test here presses the buttons the page presses, in the order the page
    offers them, and feeds the FSM a hand-move between them — because the two
    things that can go wrong are both about the order: a point recorded in the
    wrong place, and a point recorded under the wrong label.
    """

    def test_it_is_limp_throughout(self) -> None:
        """Zero torque is the whole mechanism: the operator drives the jaws."""
        fsm = TwoPointCalibFSM()
        fsm.start(0.5)
        for _ in range(200):
            out = fsm.tick(0.5, DT)
            assert out.kp == 0.0 and out.kd == 0.0 and out.tau_nm == 0.0

    def test_it_records_the_two_points_under_their_labels(self) -> None:
        fsm = TwoPointCalibFSM()
        assert fsm.start(CLOSED_RAD)
        walk(fsm, CLOSED_RAD, OPEN_RAD)
        assert fsm.record_open()
        fsm.tick(OPEN_RAD, DT)
        walk(fsm, OPEN_RAD, CLOSED_RAD)
        assert fsm.record_close()
        fsm.tick(CLOSED_RAD, DT)

        assert fsm.phase is TwoPointPhase.DONE, fsm.note
        assert fsm.open_rad == pytest.approx(OPEN_RAD)
        assert fsm.close_rad == pytest.approx(CLOSED_RAD)
        result = fsm.result
        assert result is not None
        assert result.travel_rad == pytest.approx(CLOSED_RAD - OPEN_RAD)
        assert result.stroke_mm == pytest.approx(
            fsm.max_stroke_mm + constants.SPAN_INSET_MM, abs=0.01
        )
        # And 0 mm is the end that was labelled closed, which is the whole
        # content of the operator's answer.
        assert result.zero_rad == pytest.approx(CLOSED_RAD)
        assert not Limits(result.zero_rad, result.open_rad, result.rad_to_mm).reversed_mount

    def test_a_reverse_mounted_gripper_needs_no_declaration(self) -> None:
        """The property the guided probe cannot have, and the reason this flow
        survives a gripper the SDK's formulas do not describe.

        Nothing here is told which way the jaws open.  The operator labels the
        two points as they record them, so the direction is not an assumption
        that has to be checked afterwards — it is the answer, and a gripper
        assembled either way round records correctly through the same buttons.
        """
        fsm = TwoPointCalibFSM()
        assert fsm.start(REVERSED_CLOSED_RAD)
        walk(fsm, REVERSED_CLOSED_RAD, REVERSED_OPEN_RAD)
        assert fsm.record_open()
        fsm.tick(REVERSED_OPEN_RAD, DT)
        walk(fsm, REVERSED_OPEN_RAD, REVERSED_CLOSED_RAD)
        assert fsm.record_close()
        fsm.tick(REVERSED_CLOSED_RAD, DT)

        assert fsm.phase is TwoPointPhase.DONE, fsm.note
        result = fsm.result
        assert result is not None
        limits = Limits(result.zero_rad, result.open_rad, result.rad_to_mm)
        assert limits.reversed_mount
        assert limits.to_rad(0.0) == pytest.approx(REVERSED_CLOSED_RAD)
        # The angle grows toward open on this one, and the travel is a magnitude
        # either way: the two mountings differ in the sign of the conversion,
        # not in the numbers the operator recorded.
        assert result.travel_rad == pytest.approx(REVERSED_OPEN_RAD - REVERSED_CLOSED_RAD)
        assert limits.to_rad(limits.max_stroke_mm) > limits.to_rad(0.0)

    def test_the_label_is_what_decides_the_direction(self) -> None:
        """The failure the two buttons exist to prevent, pinned rather than
        assumed: swap the labels and the same two angles produce a file whose
        every millimetre runs the other way, with nothing about it to see."""
        closed, opened = REVERSED_CLOSED_RAD, REVERSED_OPEN_RAD
        right = summarise(closed, opened, 85.0, ())
        wrong = summarise(opened, closed, 85.0, ())
        assert isinstance(right, CalibResult) and isinstance(wrong, CalibResult)
        # The same two angles, and the two files disagree about everything that
        # matters: where 0 mm is, and which way the millimetres run.
        assert right.zero_rad == wrong.open_rad
        assert right.open_rad == wrong.zero_rad
        right_limits = Limits(right.zero_rad, right.open_rad, right.rad_to_mm)
        wrong_limits = Limits(wrong.zero_rad, wrong.open_rad, wrong.rad_to_mm)
        assert right_limits.to_rad(0.0) == pytest.approx(closed)
        assert wrong_limits.to_rad(0.0) == pytest.approx(opened)
        assert right_limits.reversed_mount is not wrong_limits.reversed_mount

    def test_an_out_of_turn_record_is_refused(self) -> None:
        """Pressing the wrong button must do nothing at all.  Taking it as
        whichever point is due is how the labels get swapped, and the two
        buttons are the only thing standing between the operator and a file
        that drives the gripper inverted."""
        fsm = TwoPointCalibFSM()
        fsm.start(0.5)
        assert not fsm.record_close()
        assert fsm.phase is TwoPointPhase.RECORD_OPEN
        assert fsm.close_rad is None

        walk(fsm, 0.5, 1.0)
        press_open(fsm, 1.0)
        assert fsm.phase is TwoPointPhase.RECORD_CLOSE
        assert not fsm.record_open()
        assert fsm.open_rad == pytest.approx(1.0)

    def test_the_reading_taken_is_the_one_after_the_press(self) -> None:
        """The operator stops moving, then presses; the tick after the press is
        the first angle taken with the hand off the jaws."""
        fsm = TwoPointCalibFSM()
        fsm.start(0.5)
        walk(fsm, 0.5, 1.0)
        fsm.record_open()
        # The hand lets go and the jaws settle that last fraction of a degree.
        fsm.tick(1.0004, DT)
        assert fsm.open_rad == pytest.approx(1.0004)
        assert fsm.open_rad != pytest.approx(1.0)

    def test_a_reading_that_jumps_to_zero_is_not_recorded(self) -> None:
        """The hole the SDK's own guard leaves.  A dropped frame leaves the
        cached position at 0.0, which is inside ``|pos| < 50`` and inside a
        motor-rate threshold — but zero is beyond the open stop, so recording it
        would overstate the travel and understate every millimetre after it."""
        fsm = TwoPointCalibFSM()
        fsm.start(CLOSED_RAD)
        walk(fsm, CLOSED_RAD, 0.6)
        fsm.record_open()
        fsm.tick(0.0, DT)  # the feedback went away and came back as zero

        assert fsm.phase is TwoPointPhase.RECORD_OPEN, "zero was taken as the open stop"
        assert fsm.open_rad is None
        assert fsm.rejected == 1
        # And the operator is told, because the alternative is a button that
        # silently did nothing.
        assert "不可信" in fsm.note

        # A second press, with the reading back, records normally.  The hand
        # works back up to where it was: the return from the glitch is a jump
        # too, and one reading cannot tell a return from a second glitch.
        walk(fsm, 0.0, 0.6)
        press_open(fsm, 0.6)
        assert fsm.open_rad == pytest.approx(0.6)

    def test_an_out_of_range_reading_is_never_recorded(self) -> None:
        """The SDK's own guard (gripper.py:427), which is the one it applies to
        the samples its zero-gravity mode collects."""
        fsm = TwoPointCalibFSM()
        fsm.start(1.0)
        fsm.record_open()
        fsm.tick(constants.MANUAL_POS_GUARD_RAD + 0.1, DT)
        assert fsm.open_rad is None
        assert fsm.rejected == 1

    def test_the_guard_is_the_sdk_threshold(self) -> None:
        fsm = TwoPointCalibFSM(max_jump_rad=1e9)
        assert fsm.pos_guard_rad == constants.MANUAL_POS_GUARD_RAD
        fsm.start(0.0)
        fsm.tick(constants.MANUAL_POS_GUARD_RAD - 1e-9, DT)
        assert fsm.rejected == 0
        fsm.tick(constants.MANUAL_POS_GUARD_RAD, DT)
        assert fsm.rejected == 1

    @pytest.mark.parametrize("hand_speed", [100.0, 500.0, 1000.0])
    def test_a_legitimate_hand_move_is_never_treated_as_a_jump(
        self, hand_speed: float
    ) -> None:
        """Including a hard yank at the speed the guard is derived from."""
        fsm = TwoPointCalibFSM()
        fsm.start(CLOSED_RAD)
        walk(fsm, CLOSED_RAD, OPEN_RAD, rate_rad_s=hand_speed / 65.0)
        assert fsm.rejected == 0

    def test_the_manual_guard_is_tighter_than_the_guided_one(self) -> None:
        """A hand is slower than a motor, and the tighter bound is what catches
        the lost-frame jump above."""
        assert MANUAL_MAX_JUMP_RAD_PER_TICK < PROBE_MAX_JUMP_RAD_PER_TICK

    def test_both_points_recorded_in_the_same_place_fails(self) -> None:
        """Recording the same place twice is not a travel.  Saving it would give
        a gripper whose every position is one position."""
        fsm = TwoPointCalibFSM()
        fsm.start(0.5)
        fsm.record_open()
        fsm.tick(0.5, DT)
        fsm.record_close()
        fsm.tick(0.5, DT)

        assert fsm.phase is TwoPointPhase.FAILED
        assert "行程异常" in fsm.note
        assert fsm.result is None

    def test_a_point_that_is_never_recorded_fails_on_the_timeout(self) -> None:
        """The axis is limp for as long as this waits, so the wait has to end on
        its own — a console left holding an enabled, limp motor is a gripper that
        falls open on whatever is under it."""
        fsm = TwoPointCalibFSM(timeout_s=1.0)
        fsm.start(0.5)
        run(fsm, Plant(0.5), seconds=2.0, until=lambda f: f.is_terminal)

        assert fsm.phase is TwoPointPhase.FAILED
        assert "记录超时" in fsm.note and "张开" in fsm.note
        # Limp, not held: the reading a hold would use is one taken up to the
        # whole timeout ago, and the hand has been free to move the jaws since.
        assert fsm.tick(0.5, DT) is FREE_FRAME

    def test_the_timeout_restarts_for_the_second_point(self) -> None:
        """Otherwise a slow but successful first point would leave no time at
        all for the second, and the operator would be racing a clock they had
        already spent."""
        fsm = TwoPointCalibFSM(timeout_s=1.0)
        fsm.start(0.5)
        run(fsm, Plant(0.5), seconds=0.9, until=lambda f: f.is_terminal)
        assert fsm.phase is TwoPointPhase.RECORD_OPEN
        press_open(fsm, 0.5)

        assert fsm.remaining_s == pytest.approx(1.0)
        run(fsm, Plant(0.5), seconds=0.9, until=lambda f: f.is_terminal)
        assert fsm.phase is TwoPointPhase.RECORD_CLOSE

    def test_the_remaining_time_counts_down_and_stops_between_points(self) -> None:
        fsm = TwoPointCalibFSM(timeout_s=10.0)
        assert fsm.remaining_s == 0.0, "nothing is being waited for before the start"
        fsm.start(0.5)
        assert fsm.remaining_s == pytest.approx(10.0)
        for _ in range(HZ * 2):  # two seconds
            fsm.tick(0.5, DT)
        assert fsm.remaining_s == pytest.approx(8.0, abs=0.01)
        fsm.cancel()
        assert fsm.remaining_s == 0.0

    def test_progress_steps_at_the_record_rather_than_on_a_clock(self) -> None:
        """There is no clock to creep along: the operator decides when a point
        is recorded, so the bar counts the points."""
        fsm = TwoPointCalibFSM()
        fsm.start(0.5)
        assert fsm.progress == 0.0
        walk(fsm, 0.5, 1.5)
        press_open(fsm, 1.5)
        assert fsm.progress == 0.5
        walk(fsm, 1.5, 0.5)
        press_close(fsm, 0.5)
        assert fsm.phase is TwoPointPhase.DONE
        assert fsm.progress == 1.0

    def test_a_failed_probe_does_not_report_a_finished_bar(self) -> None:
        """A bar at 100% next to a failure reads as "done"; the phase label is
        the only thing saying otherwise."""
        fsm = TwoPointCalibFSM(timeout_s=1.0)
        fsm.start(0.5)
        run_to_terminal(fsm, Plant(0.5), seconds=2.0)

        assert fsm.phase is TwoPointPhase.FAILED
        assert fsm.progress == 0.0

        fsm.cancel()
        assert fsm.progress == 0.0

    def test_the_phase_order_is_the_order_the_buttons_are_offered_in(self) -> None:
        fsm = TwoPointCalibFSM()
        fsm.start(0.5)
        seen: list[TwoPointPhase] = []

        def watch() -> None:
            if not seen or seen[-1] is not fsm.phase:
                seen.append(fsm.phase)

        watch()
        walk(fsm, 0.5, 1.5)
        watch()
        press_open(fsm, 1.5)
        watch()
        walk(fsm, 1.5, 0.5)
        press_close(fsm, 0.5)
        watch()

        assert seen == [
            TwoPointPhase.RECORD_OPEN,
            TwoPointPhase.RECORD_CLOSE,
            TwoPointPhase.DONE,
        ]

    def test_a_restart_clears_the_previous_attempt(self) -> None:
        """The page offers "try again"; the second run must not inherit the
        first one's points, or the page would offer to save a calibration the
        new run never produced."""
        fsm = TwoPointCalibFSM()
        fsm.start(0.5)
        walk(fsm, 0.5, 1.5)
        press_open(fsm, 1.5)
        walk(fsm, 1.5, 0.5)
        press_close(fsm, 0.5)
        assert fsm.is_terminal and fsm.result is not None

        assert fsm.start(1.0)
        assert fsm.phase is TwoPointPhase.RECORD_OPEN
        assert fsm.open_rad is None and fsm.close_rad is None and fsm.result is None
        assert fsm.rejected == 0

    def test_cancelling_discards_the_result_and_frees_the_axis(self) -> None:
        fsm = TwoPointCalibFSM()
        fsm.start(1.0)
        fsm.tick(0.0, DT)
        fsm.cancel()
        out = fsm.tick(0.0, DT)

        assert fsm.phase is TwoPointPhase.CANCELLED
        assert fsm.result is None
        assert out is FREE_FRAME

    def test_the_finished_probe_holds_the_position_it_recorded(self) -> None:
        """It ends at the closed stop, with the operator's hand just off the
        jaws — so a hold here holds where they were left, rather than the press
        against a hard stop a guided probe ends on."""
        fsm = TwoPointCalibFSM()
        fsm.start(CLOSED_RAD)
        walk(fsm, CLOSED_RAD, OPEN_RAD)
        press_open(fsm, OPEN_RAD)
        walk(fsm, OPEN_RAD, CLOSED_RAD)
        press_close(fsm, CLOSED_RAD)

        out = fsm.tick(CLOSED_RAD, DT)
        assert out.q_rad == pytest.approx(CLOSED_RAD)
        assert out.kp == constants.KP_MOVE
        assert out.kd == constants.KD_DEFAULT

    def test_failing_from_outside_stops_it_and_keeps_the_reason(self) -> None:
        """The worker calls this when the frames stop reaching the motor: the
        operator can see the jaws move, so a probe that has stopped hearing the
        drive is recording the extremes of nothing."""
        fsm = TwoPointCalibFSM()
        fsm.start(0.5)
        walk(fsm, 0.5, 1.0)
        press_open(fsm, 1.0)
        fsm.fail("标定中止：电机未使能")

        assert fsm.phase is TwoPointPhase.FAILED
        assert fsm.note == "标定中止：电机未使能"
        assert fsm.result is None
        # The operator's hand is still on the jaws and they are about to start
        # again: stiffening at the last reading would push against it.
        assert fsm.tick(1.0, DT) is FREE_FRAME

    def test_a_failure_after_the_end_does_not_rewrite_the_result(self) -> None:
        fsm = TwoPointCalibFSM()
        fsm.start(0.5)
        walk(fsm, 0.5, 1.5)
        press_open(fsm, 1.5)
        walk(fsm, 1.5, 0.5)
        press_close(fsm, 0.5)
        note = fsm.note
        fsm.fail("标定中止：位置帧连续 0.5 s 未能发出")

        assert fsm.phase is TwoPointPhase.DONE
        assert fsm.note == note
        assert fsm.result is not None

    def test_the_notes_record_what_was_measured(self) -> None:
        """The log has to be able to say where the two numbers came from: a
        calibration is the one result on this console that nothing downstream
        can check."""
        fsm = TwoPointCalibFSM()
        fsm.start(0.5)
        walk(fsm, 0.5, 1.5)
        press_open(fsm, 1.5)
        walk(fsm, 1.5, 0.5)
        press_close(fsm, 0.5)

        result = fsm.result
        assert result is not None
        text = " ".join(result.notes)
        assert "1.500000" in text and "0.500000" in text

    @pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
    def test_a_non_finite_reading_stops_the_probe(self, bad: float) -> None:
        fsm = TwoPointCalibFSM()
        fsm.start(0.5)
        fsm.tick(bad, DT)

        assert fsm.phase is TwoPointPhase.FAILED
        assert "读数无效" in fsm.note

    def test_a_probe_that_never_had_a_reading_holds_nothing(self) -> None:
        """Holding at the ``0.0`` it was constructed with would command a move
        to zero — the opposite of what a failed probe should do."""
        fsm = TwoPointCalibFSM()
        assert not fsm.start(float("nan"))
        out = fsm.tick(0.5, DT)
        assert out is FREE_FRAME

    def test_an_idle_probe_sends_nothing(self) -> None:
        assert TwoPointCalibFSM().tick(0.5, DT) is NO_FRAME

    def test_it_does_not_move_the_jaws_before_it_is_started(self) -> None:
        fsm = TwoPointCalibFSM()
        plant = Plant(0.5)
        for _ in range(100):
            plant.tick(fsm)
        assert plant.pos == 0.5

    @pytest.mark.parametrize("junk", [0.0, 1e300, -1e300, float("nan"), float("inf")])
    def test_tick_is_total(self, junk: float) -> None:
        """The worker calls this every 5 ms inside a ``BaseException`` guard, and
        an exception escaping ``QThread.run`` aborts the process — so a tick that
        cannot be reasoned about must still return a frame."""
        fsm = TwoPointCalibFSM()
        assert fsm.tick(junk, DT) is NO_FRAME
        fsm.start(0.5)
        out = fsm.tick(junk, DT)
        assert out.q_rad is None or math.isfinite(out.q_rad)

    def test_the_coefficient_follows_the_configured_travel(self) -> None:
        """The two angles are measured and the millimetres are not: the operator
        has told the console how far the jaws travel, and that is what the
        coefficient comes out of."""
        fsm = TwoPointCalibFSM(max_stroke_mm=60.0)
        fsm.start(CLOSED_RAD)
        walk(fsm, CLOSED_RAD, OPEN_RAD)
        press_open(fsm, OPEN_RAD)
        walk(fsm, OPEN_RAD, CLOSED_RAD)
        press_close(fsm, CLOSED_RAD)

        result = fsm.result
        assert result is not None
        assert result.max_stroke_mm == 60.0
        assert result.rad_to_mm == pytest.approx(
            (60.0 + constants.SPAN_INSET_MM) / result.travel_rad, abs=0.02
        )


def ceil_div(a: float, b: float) -> int:
    return int(math.ceil(a / b))
