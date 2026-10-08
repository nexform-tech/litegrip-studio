"""mm ↔ rad conversion, limits, and force scaling.

These are the functions where a sign error drives a gripper into a hard stop,
so the expectations are checked against real calibrations — this bench's user
file, the SDK's shipped template, and its uncalibrated default pair — rather
than against the formulas themselves.
"""

from __future__ import annotations

import math
from types import SimpleNamespace

import pytest

from litegrip_studio import constants
from litegrip_studio.units import (
    Limits,
    clamp_force,
    clamp_force_torque,
    force_from_torque,
    frame_mismatch,
    mm_to_rad_per_s,
    rad_per_s_to_mm,
    torque_from_force,
)

# The three calibrations the SDK ships or produces on this bench.  The third is
# ``GripperConfig``'s untouched default pair, which the ordering alone used to
# disqualify; here it is kept as what it looks like from the numbers — a pair of
# angles recorded the other way round, and one whose range the real bench's
# encoder readings fall outside of.
#
# The middle one is the SDK's *template* (``calibration_normal.json``) and not
# its factory file, which is what it was labelled as until the two were
# compared.  The template is the nominal unit, so its mm/rad is the SDK's 120 mm
# over the 1.605 rad it recorded (74.8, rounded) and its span comes out as
# 120 mm — the gap ``TestStroke`` pins.  The factory file is this bench's, and
# its mm/rad already is the scale derived from its own travel, so it cannot show
# that gap at all; it is checked as a file in ``test_calibration.py``.
REVERSED = Limits(0.0, 1.14, 104.6)  # GripperConfig defaults
TEMPLATE = Limits(0.114, -1.491, 74.8)  # the SDK's calibration_normal.json
BENCH = Limits(1.775959, -0.064279, 65.21)  # example user calibration

#: What the real bench's encoder reports, closed and fully open — the two
#: numbers a calibration of that unit is taken from.  They are here because the
#: check that has to tell a file from the gripper in front of it needs a pair of
#: readings that a given file either contains or does not.
MEASURED_CLOSED_RAD = 1.421569
MEASURED_OPEN_RAD = -0.300793


class TestRoundTrip:
    @pytest.mark.parametrize("lim", [TEMPLATE, BENCH, REVERSED])
    @pytest.mark.parametrize("mm", [0.0, 0.001, 1.0, 37.5, 119.9, 120.0])
    def test_mm_rad_round_trip(self, lim: Limits, mm: float) -> None:
        assert lim.to_mm(lim.to_rad(mm)) == pytest.approx(mm, abs=1e-9)

    @pytest.mark.parametrize("lim", [TEMPLATE, BENCH, REVERSED])
    def test_rad_mm_round_trip(self, lim: Limits, ) -> None:
        for rad in (lim.closed_rad, lim.open_rad, (lim.closed_rad + lim.open_rad) / 2):
            assert lim.to_rad(lim.to_mm(rad)) == pytest.approx(rad, abs=1e-12)

    def test_zero_mm_is_the_closed_angle(self) -> None:
        assert BENCH.to_rad(0.0) == pytest.approx(BENCH.closed_rad)

    def test_full_stroke_is_the_open_angle(self) -> None:
        assert BENCH.to_rad(BENCH.stroke_mm) == pytest.approx(BENCH.open_rad)

    def test_angle_decreases_as_the_jaws_open(self) -> None:
        """The sign of this slope is the whole reason mm_to_rad_per_s negates."""
        assert BENCH.to_rad(10.0) < BENCH.to_rad(0.0)

    def test_angle_increases_as_the_jaws_open_when_recorded_the_other_way(self) -> None:
        """Same mapping, opposite slope — the formulas above are its
        ``direction == -1`` case, not the only case they can describe."""
        assert REVERSED.to_rad(10.0) > REVERSED.to_rad(0.0)


class TestStroke:
    def test_the_template_stroke_is_the_nominal_it_was_written_for(self) -> None:
        """1.605 rad of travel at the file's own 74.8 mm/rad is 120.05 mm.

        Which is the SDK's nominal stroke, not this bench's travel: a file that
        records its own 120 mm is describing the unit it was written for, and
        that gap is what ``validate_limits`` reports.
        """
        assert TEMPLATE.stroke_mm == pytest.approx(120.06, abs=0.01)

    def test_bench_stroke(self) -> None:
        assert BENCH.stroke_mm == pytest.approx(120.0, abs=0.01)

    def test_travel_is_absolute(self) -> None:
        for lim in (TEMPLATE, BENCH):
            assert lim.travel_rad > 0
            assert lim.rad_low < lim.rad_high


class TestFromConfig:
    """Reading an SDK ``GripperConfig``, which is where a missing travel lands.

    ``GripperConfig`` carries ``max_stroke_mm``; a config that does not is older
    than that field.  The fallback for it is the travel the operator measured,
    never the SDK's 120 mm spec figure — that would scale every reading by a
    number nobody measured and put the top of the slider in the middle of the
    travel, which is the defect the derivation exists to remove.
    """

    ANGLES = {"pos_closed_rad": 1.775959, "pos_open_rad": -0.064279, "rad_to_mm": 65.21}

    def test_a_config_with_no_travel_falls_back_to_the_measured_one(self) -> None:
        config = SimpleNamespace(**self.ANGLES)

        limits = Limits.from_config(config)

        assert limits.rad_to_mm == pytest.approx(65.21), "the config's scale is still read"
        assert limits.max_stroke_mm == pytest.approx(constants.DEFAULT_TRAVEL_MM)

    def test_a_config_with_a_travel_keeps_it(self) -> None:
        limits = Limits.from_config(SimpleNamespace(**self.ANGLES, max_stroke_mm=150.0))

        assert limits.max_stroke_mm == pytest.approx(150.0)


class TestDirectionCheck:
    """``direction`` is read from the recorded angles rather than assumed.

    The SDK's two formulas pin "the angle shrinks as the jaws open", which is how
    the units it was written for are assembled — an assembly detail, not a
    property of a calibration.  A pair of angles recorded the other way round is
    entirely valid and is converted by what it says; what refuses a file that
    does not belong to the hardware is :func:`frame_mismatch`, which reads the
    encoder and never looks at the ordering at all.
    """

    def test_the_two_recorded_the_usual_way_have_the_closed_stop_at_the_larger_angle(
        self,
    ) -> None:
        for lim in (TEMPLATE, BENCH):
            assert lim.direction == -1.0

    def test_the_defaults_read_as_the_ordering_they_have(self) -> None:
        assert REVERSED.direction == 1.0

    def test_equal_angles_are_degenerate_and_fall_back_to_one_case(self) -> None:
        equal = Limits(0.5, 0.5, 65.0)
        assert equal.travel_rad == 0.0
        assert equal.direction == -1.0, "no travel means no ordering to read"
        assert not frame_mismatch(equal, 0.5), "0.5 is inside the collapsed range"

    def test_the_direction_is_the_sign_of_the_conversion_slope(self) -> None:
        """Both orderings agree with their own recorded angles, which is the only
        thing that makes the other one safe to drive."""
        for lim in (TEMPLATE, BENCH, REVERSED):
            assert lim.to_rad(0.0) == pytest.approx(lim.closed_rad)
            step = lim.to_rad(10.0) - lim.to_rad(0.0)
            assert math.copysign(1.0, step) == lim.direction
            assert math.copysign(1.0, lim.to_mm(lim.open_rad)) == 1.0

    def test_the_range_check_is_blind_to_which_way_they_were_recorded(self) -> None:
        """Which is what lets it be the guard.  The same two angles, recorded
        either way round, both contain the readings they were taken from — and
        only the encoder knows which of the two this gripper has."""
        classic = Limits(MEASURED_CLOSED_RAD, MEASURED_OPEN_RAD, 49.93)
        flipped = Limits(MEASURED_OPEN_RAD, MEASURED_CLOSED_RAD, 49.93)
        for lim in (classic, flipped):
            for measured in (MEASURED_CLOSED_RAD, MEASURED_OPEN_RAD):
                assert not frame_mismatch(lim, measured)
        assert classic.direction == -1.0
        assert flipped.direction == 1.0

    def test_a_file_that_does_not_match_the_encoder_is_caught_by_its_range(self) -> None:
        """The check that replaced the refusal.  Whatever the ordering, readings
        from a gripper whose zero is not this file's land outside it."""
        for measured in (MEASURED_CLOSED_RAD, MEASURED_OPEN_RAD):
            assert frame_mismatch(REVERSED, measured)
        assert not frame_mismatch(REVERSED, 0.5), "and it is not simply always on"


class TestClamping:
    def test_mm_is_clamped_to_the_travel(self) -> None:
        assert BENCH.clamp_mm(-5.0) == 0.0
        assert BENCH.clamp_mm(1e6) == BENCH.max_stroke_mm
        assert BENCH.clamp_mm(60.0) == 60.0

    def test_rad_is_clamped_to_the_calibrated_range(self) -> None:
        assert BENCH.clamp_rad(1e6) == BENCH.rad_high
        assert BENCH.clamp_rad(-1e6) == BENCH.rad_low

    def test_nan_does_not_propagate_into_a_command(self) -> None:
        """A NaN that reaches a frame is a runaway motor, so it is caught here."""
        assert BENCH.clamp_mm(math.nan) == 0.0
        assert not math.isnan(BENCH.clamp_rad(math.nan))

    def test_speed_is_clamped_to_the_configured_band(self) -> None:
        assert BENCH.clamp_speed(0.0) == constants.SPEED_MIN_MM_S
        assert BENCH.clamp_speed(1e6) == constants.SPEED_MAX_MM_S
        assert BENCH.clamp_speed(math.nan) == constants.SPEED_DEFAULT_MM_S


class TestVelocityFeedForward:
    def test_opening_gives_a_negative_angular_velocity(self) -> None:
        """q = closed − mm/rad_to_mm, so dq/dmm < 0 and the sign must flip."""
        assert mm_to_rad_per_s(50.0, 65.21, direction=-1.0) < 0.0

    def test_closing_gives_a_positive_angular_velocity(self) -> None:
        assert mm_to_rad_per_s(-50.0, 65.21, direction=-1.0) > 0.0

    def test_opening_gives_a_positive_angular_velocity_on_the_other_ordering(self) -> None:
        assert mm_to_rad_per_s(50.0, 65.21, direction=1.0) > 0.0
        assert mm_to_rad_per_s(-50.0, 65.21, direction=1.0) < 0.0

    def test_magnitude_matches_the_conversion_rate(self) -> None:
        assert abs(mm_to_rad_per_s(65.21, 65.21, direction=-1.0)) == pytest.approx(1.0)
        assert abs(mm_to_rad_per_s(65.21, 65.21, direction=1.0)) == pytest.approx(1.0)

    @pytest.mark.parametrize("lim", [TEMPLATE, BENCH, REVERSED])
    def test_the_derivative_of_the_position_mapping(self, lim: Limits) -> None:
        """Cross-check the sign against the position mapping numerically.

        This is the check that matters: the velocity term and the position term
        are two branches of one mapping, and a sign error between them pins the
        jaws against a stop at full speed.  Both orderings, or the other branch
        is only ever tested by a formula that shares its mistake.
        """
        mm0, mm1, dt = 40.0, 40.1, 0.005
        dq_measured = (lim.to_rad(mm1) - lim.to_rad(mm0)) / dt
        speed_mm_s = (mm1 - mm0) / dt
        assert mm_to_rad_per_s(
            speed_mm_s, lim.rad_to_mm, direction=lim.direction
        ) == pytest.approx(dq_measured, rel=1e-9)

    def test_round_trips(self) -> None:
        for direction in (-1.0, 1.0):
            for v in (-150.0, -1.0, 0.0, 1.0, 150.0):
                assert rad_per_s_to_mm(
                    mm_to_rad_per_s(v, 65.21, direction=direction),
                    65.21,
                    direction=direction,
                ) == pytest.approx(v)

    def test_zero_scale_does_not_divide_by_zero(self) -> None:
        assert mm_to_rad_per_s(50.0, 0.0, direction=-1.0) == 0.0
        assert rad_per_s_to_mm(1.0, 0.0, direction=-1.0) == 0.0

    def test_a_nan_speed_does_not_become_a_nan_velocity_reference(self) -> None:
        """This output goes into the MIT velocity term with no clamp after it."""
        assert mm_to_rad_per_s(math.nan, 65.21, direction=-1.0) == 0.0
        assert mm_to_rad_per_s(math.inf, 65.21, direction=-1.0) == 0.0
        assert rad_per_s_to_mm(math.nan, 65.21, direction=-1.0) == 0.0
        assert rad_per_s_to_mm(math.inf, 65.21, direction=-1.0) == 0.0


class TestForceScaling:
    def test_force_and_torque_are_inverses(self) -> None:
        for f in (0.0, 5.0, 12.0, 40.0):
            assert force_from_torque(torque_from_force(f)) == pytest.approx(f)

    def test_rating_matches_the_sdk_conversion(self) -> None:
        assert torque_from_force(constants.FORCE_MAX_N) == pytest.approx(4.0)

    def test_the_rating_cannot_be_exceeded(self) -> None:
        """The cap is what makes FORCE_MAX_N a bound rather than a suggestion."""
        assert clamp_force(1e9) == constants.FORCE_MAX_N
        assert clamp_force(-5.0) == 0.0
        assert clamp_force(math.nan) == 0.0
        assert clamp_force_torque(1e9) == pytest.approx(
            constants.FORCE_MAX_N * constants.N_TO_NM
        )

    def test_theoretical_max_is_unreachable_through_the_cap(self) -> None:
        assert constants.FORCE_MAX_N < constants.FORCE_THEORETICAL_MAX_N
        assert clamp_force_torque(constants.FORCE_THEORETICAL_MAX_N) < (
            constants.FORCE_THEORETICAL_MAX_N * constants.N_TO_NM
        )
