"""The plots page: two decisions that are visible only when they are wrong.

The page is fed at half the rate the worker publishes at, and a sample that
cannot be drawn as a line is drawn as a gap rather than as a continuation.  Both
are tested here without a worker, a thread or a clock.
"""

from __future__ import annotations

import dataclasses

import pytest

from litegrip_studio import constants
from litegrip_studio.telemetry import EMPTY_FRAME, TelemetryFrame
from litegrip_studio.ui.plots_page import (
    CAPACITY,
    FORCE_AXIS_MIN_N,
    NO_VALUE,
    PLOT_INTERVAL_S,
    WINDOW_S,
    PlotsPage,
    _axis_bound,
)
from litegrip_studio.units import Limits

#: A derived calibration, so the recorded span (86 mm) and the commanded range
#: (85 mm) differ: the axis has to be ranged to the second, and with a file's own
#: scale the two coincide and the test could not tell them apart.
LIMITS = Limits(1.775959, -0.064279, 65.752365, 85.0)
FRAME_HZ = 50.0


def frame(t: float, **changes) -> TelemetryFrame:
    return dataclasses.replace(EMPTY_FRAME, t=t, **changes)


@pytest.fixture
def page(qapp) -> PlotsPage:
    return PlotsPage()


def feed(page, seconds: float, hz: float = FRAME_HZ, **changes) -> None:
    """Publish frames the way the worker does — on a fixed clock."""
    step = 1.0 / hz
    count = int(round(seconds * hz))
    for index in range(count):
        page.update_frame(frame(index * step, **changes))


class TestTheRate:
    def test_fifty_frames_a_second_become_twenty_five_samples(self, page) -> None:
        """Exactly twenty-five, not twenty: the publish rate is twice the plot
        rate, so every interval lands on the boundary and an exact comparison
        would drop two frames in three as the float error accumulated."""
        feed(page, 1.0)

        assert len(page.samples) == constants.PLOT_HZ

    def test_the_decimation_is_by_time_not_by_counting_frames(self, page) -> None:
        """A GUI that stalled publishes a burst of queued frames all at once.
        Halving by count would plot half of them at the same instant and push
        the window's worth of samples off the end of the buffer; the interval
        test plots the first and drops the rest, which is what they are."""
        for index in range(20):
            page.update_frame(frame(100.0 + index * 0.001, position_mm=float(index)))
        page.update_frame(frame(100.0 + PLOT_INTERVAL_S, position_mm=99.0))

        assert len(page.samples) == 2
        assert page.samples[-1][1] == pytest.approx(99.0)

    def test_a_slow_feed_is_not_decimated(self, page) -> None:
        """Frames arriving slower than the plot rate must all be drawn, or the
        plot would show fewer samples the slower the link got."""
        feed(page, 1.0, hz=10.0)

        assert len(page.samples) == 10


class TestTheBuffer:
    def test_the_window_holds_ten_seconds(self, page) -> None:
        assert CAPACITY >= WINDOW_S * constants.PLOT_HZ

    def test_old_samples_fall_off_the_end(self, page) -> None:
        feed(page, WINDOW_S * 2.0)

        assert len(page.samples) == CAPACITY
        newest = page.samples[-1][0]
        assert newest - page.samples[0][0] <= WINDOW_S + PLOT_INTERVAL_S

    def test_the_axis_starts_at_zero_wherever_the_clock_started(self, page) -> None:
        """``frame.t`` is a wall clock, so plotting it raw would put the labels
        somewhere around 1.7e9."""
        page.update_frame(frame(1_700_000_000.0, position_mm=1.0))
        page.update_frame(frame(1_700_000_001.0, position_mm=2.0))

        assert page.samples[0][0] == pytest.approx(0.0)
        assert page.samples[-1][0] == pytest.approx(1.0)

    def test_clearing_empties_it_and_restarts_the_axis(self, page) -> None:
        feed(page, 1.0)
        page.clear()
        page.update_frame(frame(50.0, position_mm=7.0))

        assert len(page.samples) == 1
        assert page.samples[0][0] == pytest.approx(0.0)
        assert page.samples[0][1] == pytest.approx(7.0)


class TestWhatIsDrawn:
    def test_the_position_the_target_and_the_force_are_all_kept(self, page) -> None:
        page.update_frame(frame(0.0, position_mm=42.0, cmd_mm=90.0, force_n=3.5))

        assert page.samples[0] == pytest.approx((0.0, 42.0, 90.0, 3.5))

    def test_a_frame_with_no_target_is_a_gap_not_a_continuation(self, page) -> None:
        """Joining the last known target to the next one draws a movement that
        was never commanded — and the operator reads the plot to see exactly
        that."""
        page.update_frame(frame(0.0, position_mm=42.0, cmd_mm=None))

        assert page.samples[0][2] != page.samples[0][2]  # NaN

    def test_the_gap_marker_is_nan(self) -> None:
        assert NO_VALUE != NO_VALUE

    def test_a_signed_force_is_kept_signed(self, page) -> None:
        page.update_frame(frame(0.0, force_n=-3.5))

        assert page.samples[0][3] == pytest.approx(-3.5)


class TestPausing:
    def test_a_paused_page_stops_recording(self, page) -> None:
        feed(page, 0.5)
        count = len(page.samples)
        page._pause.setChecked(True)
        feed(page, 1.0, **{})

        assert len(page.samples) == count

    def test_the_button_says_how_to_get_back(self, page) -> None:
        page._pause.click()

        assert page._pause.text() == "继续"
        assert page._note.text()

    def test_resuming_starts_a_new_buffer(self, page) -> None:
        """The samples either side of the pause are separated by an interval
        nobody measured; joining them would draw a movement that never
        happened."""
        feed(page, 1.0)
        page._pause.setChecked(True)
        feed(page, 1.0)
        page._pause.setChecked(False)

        assert page.samples == []
        assert page._note.text() == ""

    def test_a_resumed_page_records_again_from_zero(self, page) -> None:
        feed(page, 1.0)
        page._pause.setChecked(True)
        page._pause.setChecked(False)
        page.update_frame(frame(30.0, position_mm=5.0))

        assert page.samples[0][0] == pytest.approx(0.0)
        assert page.samples[0][1] == pytest.approx(5.0)


class TestTheAxes:
    def test_the_position_axis_is_ranged_to_the_calibrated_travel(self, page) -> None:
        page.set_limits(LIMITS)

        low, high = page._position_plot.getViewBox().viewRange()[1]
        assert low == pytest.approx(0.0, abs=0.5)
        assert high == pytest.approx(LIMITS.max_stroke_mm, abs=0.5)
        assert LIMITS.stroke_mm != pytest.approx(LIMITS.max_stroke_mm, abs=0.5)

    def test_without_a_calibration_the_axis_falls_back_to_the_full_range(self, page) -> None:
        page.set_limits(None)

        low, high = page._position_plot.getViewBox().viewRange()[1]
        assert high == pytest.approx(constants.STROKE_MAX_MM, abs=0.5)

    def test_the_force_axis_is_not_the_rating(self, page) -> None:
        """Pinned at 40 N it says what the mechanism may do rather than what it
        is doing, and a grip the operator is watching draws in the bottom third
        of the plot."""
        assert page._force_plot.getViewBox().viewRange()[1][1] < constants.FORCE_MAX_N

    def test_the_two_plots_share_a_time_axis(self, page) -> None:
        """Position and force are read together; two independently scrolling
        axes would make the reader do the alignment in their head."""
        assert page._force_plot.getViewBox().linkedView(0) is (
            page._position_plot.getViewBox()
        )


class TestTheForceAxis:
    """The force axis follows the trace — see the module docstring.

    The complaint it answers is specific: a 12 N grip on a 40 N axis, where the
    variation inside the grip — settling or creeping — is a couple of pixels.
    """

    def range(self, page) -> tuple[float, float]:
        low, high = page._force_plot.getViewBox().viewRange()[1]
        return low, high

    def test_a_grip_grows_the_axis_until_it_fits(self, page) -> None:
        feed(page, 1.0, force_n=12.0)

        low, high = self.range(page)
        assert low <= 0.0 <= high
        assert high >= 12.0, "a sample must never be drawn off the top"
        assert high < constants.FORCE_MAX_N

    def test_the_grip_uses_a_readable_part_of_the_plot(self, page) -> None:
        feed(page, 1.0, force_n=12.0)

        _low, high = self.range(page)
        assert 12.0 / high > 0.5, f"a 12 N grip on a {high:.0f} N axis is what was wrong"

    def test_an_empty_plot_does_not_magnify_the_noise(self, page) -> None:
        """A free move's braking torque is a fraction of a newton.  On an axis
        fitted to it exactly, that is a full-height waveform that reads as a
        grip; the floor keeps it the ripple it is."""
        feed(page, 1.0, force_n=0.05)

        low, high = self.range(page)
        assert high - low == pytest.approx(FORCE_AXIS_MIN_N, abs=0.01)

    def test_a_signed_force_is_bracketed_on_both_sides(self, page) -> None:
        """The SDK's torque has no sign guarantee, so a negative excursion is a
        real reading and must not be clipped at the bottom."""
        feed(page, 1.0, force_n=-8.0)

        low, high = self.range(page)
        assert low <= -8.0
        assert high >= 0.0

    def test_a_spike_that_would_not_fit_opens_the_axis_at_once(self, page) -> None:
        feed(page, 1.0, force_n=12.0)
        before = self.range(page)[1]
        page.update_frame(frame(2.0, force_n=37.5))

        assert self.range(page)[1] >= 37.5
        assert self.range(page)[1] > before

    def test_the_axis_closes_again_once_the_spike_has_scrolled_off(self, page) -> None:
        """Nothing to reset and no timer: the window is the memory."""
        page.update_frame(frame(0.0, force_n=37.5))
        feed(page, WINDOW_S + 1.0, force_n=9.0)

        assert self.range(page)[1] < 37.5

    def test_a_peak_on_a_step_boundary_does_not_flip_the_axis(self, page) -> None:
        """8.6 N rounds to a 10 N axis and 8.7 N to a 20 N one, so a grip
        wandering between them would repaint the axis at 25 Hz with the data
        barely moving.  The axis only closes once the peak is a quarter of the
        way inside it."""
        feed(page, 1.0, force_n=8.6)
        settled = self.range(page)
        page.update_frame(frame(2.0, force_n=8.7))

        assert self.range(page) == settled

    def test_clearing_returns_the_axis_to_the_floor(self, page) -> None:
        feed(page, 1.0, force_n=37.5)
        page.clear()

        low, high = self.range(page)
        assert high - low == pytest.approx(FORCE_AXIS_MIN_N, abs=0.01)


class TestTheAxisRounding:
    """The bounds the axis is allowed to take, in isolation from the page."""

    @pytest.mark.parametrize(
        ("peak", "expected"),
        [(0.05, 0.1), (1.0, 2.0), (8.6, 10.0), (11.0, 20.0), (37.5, 50.0)],
    )
    def test_it_rounds_up_to_one_two_or_five(self, peak: float, expected: float) -> None:
        assert _axis_bound(peak) == pytest.approx(expected)

    @pytest.mark.parametrize("peak", [0.001, 0.9, 3.0, 12.0, 39.9, 100.0])
    def test_the_rounded_axis_never_clips_the_peak(self, peak: float) -> None:
        assert _axis_bound(peak) >= peak

    @pytest.mark.parametrize("peak", [0.0, -5.0, float("nan"), float("inf")])
    def test_nothing_to_show_rounds_to_nothing(self, peak: float) -> None:
        assert _axis_bound(peak) == 0.0
