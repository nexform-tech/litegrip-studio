"""mm ↔ rad conversion, limits, and force scaling.

These are the functions where a sign error drives a gripper into a hard stop,
so the expectations are checked against the three real calibration datasets
rather than against the formulas themselves.
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
    mm_to_rad_per_s,
    rad_per_s_to_mm,
    torque_from_force,
)

# The three calibrations the SDK ships or produces on this bench.
UNCALIBRATED = Limits(0.0, 1.14, 104.6)  # GripperConfig defaults — reversed
FACTORY = Limits(0.114, -1.491, 74.8)  # litegrip/factory_calibration.json
BENCH = Limits(1.775959, -0.064279, 65.21)  # example user calibration


class TestRoundTrip:
    @pytest.mark.parametrize("lim", [FACTORY, BENCH])
    @pytest.mark.parametrize("mm", [0.0, 0.001, 1.0, 37.5, 119.9, 120.0])
    def test_mm_rad_round_trip(self, lim: Limits, mm: float) -> None:
        assert lim.to_mm(lim.to_rad(mm)) == pytest.approx(mm, abs=1e-9)

    @pytest.mark.parametrize("lim", [FACTORY, BENCH])
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


class TestStroke:
    def test_factory_stroke(self) -> None:
        assert FACTORY.stroke_mm == pytest.approx(120.06, abs=0.01)

    def test_bench_stroke(self) -> None:
        assert BENCH.stroke_mm == pytest.approx(120.0, abs=0.01)

    def test_travel_is_absolute(self) -> None:
        for lim in (FACTORY, BENCH):
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
    """``is_reversed`` separates the dangerous default from real calibrations.

    It is the one check with no tolerance to tune, and the gate hard-blocks on
    it, so it is worth pinning against all three datasets.
    """

    def test_uncalibrated_defaults_are_reversed(self) -> None:
        assert UNCALIBRATED.is_reversed

    def test_factory_calibration_is_not_reversed(self) -> None:
        assert not FACTORY.is_reversed

    def test_bench_calibration_is_not_reversed(self) -> None:
        assert not BENCH.is_reversed

    def test_reversed_detected_even_when_angles_are_equal(self) -> None:
        assert Limits(0.5, 0.5, 65.0).is_reversed


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
        assert mm_to_rad_per_s(50.0, 65.21) < 0.0

    def test_closing_gives_a_positive_angular_velocity(self) -> None:
        assert mm_to_rad_per_s(-50.0, 65.21) > 0.0

    def test_magnitude_matches_the_conversion_rate(self) -> None:
        assert abs(mm_to_rad_per_s(65.21, 65.21)) == pytest.approx(1.0)

    def test_negative_of_the_position_derivative(self) -> None:
        """Cross-check the sign against the position mapping numerically."""
        lim = BENCH
        mm0, mm1, dt = 40.0, 40.1, 0.005
        dq_measured = (lim.to_rad(mm1) - lim.to_rad(mm0)) / dt
        speed_mm_s = (mm1 - mm0) / dt
        assert mm_to_rad_per_s(speed_mm_s, lim.rad_to_mm) == pytest.approx(dq_measured, rel=1e-9)

    def test_round_trips(self) -> None:
        for v in (-150.0, -1.0, 0.0, 1.0, 150.0):
            assert rad_per_s_to_mm(mm_to_rad_per_s(v, 65.21), 65.21) == pytest.approx(v)

    def test_zero_scale_does_not_divide_by_zero(self) -> None:
        assert mm_to_rad_per_s(50.0, 0.0) == 0.0
        assert rad_per_s_to_mm(1.0, 0.0) == 0.0

    def test_a_nan_speed_does_not_become_a_nan_velocity_reference(self) -> None:
        """This output goes into the MIT velocity term with no clamp after it."""
        assert mm_to_rad_per_s(math.nan, 65.21) == 0.0
        assert mm_to_rad_per_s(math.inf, 65.21) == 0.0
        assert rad_per_s_to_mm(math.nan, 65.21) == 0.0
        assert rad_per_s_to_mm(math.inf, 65.21) == 0.0


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
