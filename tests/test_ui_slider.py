"""The position slider: drag it, and watch what it says while the jaws move.

This is the widget the whole console was asked for, so it is tested on its own
rather than through the page that hosts it.  The three claims being pinned:

* dragging commits exactly one position, on release, and the position is the
  one under the cursor;
* the handle does not follow the measurement while the operator is holding it,
  and does follow it when a button is driving the axis;
* a command that the slider owns leaves the handle at the target, so the gap
  between the handle and the green mark is the tracking error and it is live.
"""

from __future__ import annotations

import pytest

from PyQt5.QtCore import QPoint, Qt
from PyQt5.QtTest import QTest

from litegrip_studio.ui.slider import HANDLE_W, MIN_HEIGHT, StrokeSlider
from litegrip_studio.units import Limits

# The example user calibration: closed at 1.775959 rad, open at -0.064279.
LIMITS = Limits(1.775959, -0.064279, 65.21, 120.0)

#: Chosen so the groove is exactly 400 px wide with a 7 px margin, which makes
#: every position below an exact millimetre value rather than a rounding.
WIDTH = 414


def x_at(fraction: float) -> int:
    """The pixel ``fraction`` of the way along the bar."""
    return int(round(HANDLE_W / 2 + (WIDTH - HANDLE_W) * fraction))


@pytest.fixture
def slider(qapp):
    widget = StrokeSlider()
    widget.resize(WIDTH, MIN_HEIGHT)
    widget.set_limits(LIMITS)
    widget.set_actual(0.0)
    return widget


def drag(widget, to_x: int, *, front: int = 0) -> None:
    QTest.mousePress(widget, Qt.LeftButton, Qt.NoModifier, QPoint(front, 20))
    QTest.mouseMove(widget, QPoint(to_x, 20))
    QTest.mouseRelease(widget, Qt.LeftButton, Qt.NoModifier, QPoint(to_x, 20))


class TestTheTravel:
    def test_the_bar_spans_the_commanded_travel(self, slider) -> None:
        assert slider.maximum() == 1200
        assert slider.value_mm == pytest.approx(0.0)

    def test_the_two_ends_are_closed_and_fully_open(self, slider) -> None:
        """The bar's left end is 0 mm and its right end is the stroke, which is
        what makes the position readable off it without a scale."""
        slider.set_value_mm(0.0)
        assert slider.value_mm == pytest.approx(0.0)
        slider.set_value_mm(120.0)
        assert slider.value_mm == pytest.approx(120.0)

    def test_a_shorter_stroke_moves_the_open_end_in(self, qapp) -> None:
        widget = StrokeSlider()
        widget.set_limits(Limits(1.775959, -0.064279, 65.21, 80.0))

        assert widget.maximum() == 800

    def test_a_value_past_the_travel_is_clamped_by_the_slider(self, slider) -> None:
        slider.set_value_mm(500.0)

        assert slider.value_mm == pytest.approx(120.0)

    def test_no_calibration_means_no_control(self, qapp) -> None:
        widget = StrokeSlider()
        widget.set_limits(None)

        assert not widget.isEnabled()


class TestDragging:
    def test_a_drag_commits_one_position_on_release(self, slider) -> None:
        committed: list[float] = []
        slider.dragCommitted.connect(committed.append)

        drag(slider, x_at(0.25))

        assert committed == pytest.approx([30.0])

    def test_the_position_committed_is_the_one_under_the_cursor(self, slider) -> None:
        for fraction, expected in ((0.0, 0.0), (0.25, 30.0), (0.5, 60.0), (1.0, 120.0)):
            committed: list[float] = []
            slider.dragCommitted.connect(committed.append)
            drag(slider, x_at(fraction))
            slider.dragCommitted.disconnect(committed.append)
            assert committed == pytest.approx([expected]), f"at {fraction}"

    def test_moving_without_pressing_commits_nothing(self, slider) -> None:
        """Telemetry arrives at 50 Hz; a hover must not look like a drag."""
        committed: list[float] = []
        slider.dragCommitted.connect(committed.append)

        QTest.mouseMove(slider, QPoint(x_at(0.75), 20))

        assert committed == []

    def test_a_release_without_a_press_commits_nothing(self, slider) -> None:
        committed: list[float] = []
        slider.dragCommitted.connect(committed.append)

        QTest.mouseRelease(slider, Qt.LeftButton, Qt.NoModifier, QPoint(x_at(0.5), 20))

        assert committed == []

    def test_a_disabled_slider_cannot_be_dragged(self, slider) -> None:
        committed: list[float] = []
        slider.dragCommitted.connect(committed.append)
        slider.set_blocked("标定未就绪")

        drag(slider, x_at(0.8))

        assert committed == []
        assert slider.value_mm == pytest.approx(0.0)

    def test_a_keyboard_step_is_a_command_too(self, slider) -> None:
        """There is no release to wait for, so it commits at once."""
        committed: list[float] = []
        slider.dragCommitted.connect(committed.append)

        QTest.keyClick(slider, Qt.Key_Right)

        assert committed == pytest.approx([0.5])

    def test_the_scroll_wheel_is_refused(self, slider) -> None:
        """Qt's default is to obey it; on a page that scrolls, the wheel event
        aimed at the page would silently re-command the gripper."""
        from PyQt5.QtGui import QWheelEvent

        before = slider.value_mm
        event = QWheelEvent(
            QPoint(200, 20), QPoint(200, 20), QPoint(0, 0), QPoint(0, -120),
            Qt.NoButton, Qt.NoModifier, Qt.NoScrollPhase, False,
        )
        slider.wheelEvent(event)

        assert slider.value_mm == pytest.approx(before)


class TestLiveFollow:
    def test_off_by_default_it_sends_nothing_until_the_release(self, slider) -> None:
        moved, committed = [], []
        slider.dragMoved.connect(moved.append)
        slider.dragCommitted.connect(committed.append)

        drag(slider, x_at(0.5), front=x_at(0.1))

        assert moved == []
        assert len(committed) == 1

    def test_switched_on_it_sends_every_update(self, slider) -> None:
        moved, committed = [], []
        slider.set_live_follow(True)
        slider.dragMoved.connect(moved.append)
        slider.dragCommitted.connect(committed.append)

        drag(slider, x_at(0.5), front=x_at(0.1))

        assert moved, "live follow must emit while the hand is down"
        assert len(committed) == 1, "...and still commit once at the end"


class TestTheHandleFollowsTheJaws:
    """The heart of it: which of the two positions the handle shows, and when."""

    def test_while_dragging_the_measurement_does_not_pull_the_handle_away(self, slider) -> None:
        """A handle dragged out from under the finger is unusable."""
        QTest.mousePress(slider, Qt.LeftButton, Qt.NoModifier, QPoint(x_at(0.8), 20))
        try:
            slider.set_actual(20.0)
            assert slider.value_mm == pytest.approx(96.0)
        finally:
            QTest.mouseRelease(slider, Qt.LeftButton, Qt.NoModifier, QPoint(x_at(0.8), 20))

    def test_a_move_the_slider_did_not_start_carries_the_handle_along(self, slider) -> None:
        """This is the 开 / 合 buttons: the bar has to fill during the move."""
        slider.set_owner("button")

        slider.set_actual(45.0)

        assert slider.value_mm == pytest.approx(45.0)

    def test_a_move_the_slider_started_leaves_the_handle_at_the_target(self, slider) -> None:
        """So the gap between the handle and the green mark is the error, live."""
        slider.set_value_mm(90.0)
        slider.set_owner("slider")

        slider.set_actual(45.0)

        assert slider.value_mm == pytest.approx(90.0)
        assert slider.actual_mm == pytest.approx(45.0)

    def test_once_nobody_owns_the_axis_the_handle_follows_again(self, slider) -> None:
        slider.set_owner("slider")
        slider.set_actual(45.0)
        slider.set_owner(None)

        slider.set_actual(50.0)

        assert slider.value_mm == pytest.approx(50.0)

    def test_the_measurement_is_kept_even_when_the_handle_ignores_it(self, slider) -> None:
        slider.set_owner("slider")
        slider.set_actual(37.5)

        assert slider.actual_mm == pytest.approx(37.5)

    def test_no_measurement_yet_is_not_a_position_at_all(self, slider) -> None:
        """``None`` from the frame means the motor has not answered.  The
        handle has to stay put: zero is the closed stop, and the console has
        not been told the jaws are there."""
        slider.set_value_mm(84.0)

        slider.set_actual(None)

        assert slider.actual_mm is None
        assert slider.value_mm == pytest.approx(84.0)

    def test_the_mark_disappears_rather_than_sitting_at_zero(self, slider) -> None:
        """Painting it at 0 mm would draw a measurement that does not exist."""
        from PyQt5.QtGui import QPixmap

        slider.set_actual(None)
        pixmap = QPixmap(WIDTH, MIN_HEIGHT)

        slider.render(pixmap)  # draws without raising, with nothing to mark

        assert slider.actual_mm is None

    def test_the_handle_never_moves_itself_before_any_telemetry(self, qapp) -> None:
        """At startup there is no measurement; the handle must not jump to a
        position that was never measured."""
        widget = StrokeSlider()
        widget.resize(WIDTH, MIN_HEIGHT)
        widget.set_limits(LIMITS)
        widget.set_owner("button")

        assert widget.value_mm == pytest.approx(0.0)
        assert widget.actual_mm is None


class TestTheMarksAreWhereTheySay:
    """The painter and the mouse must agree, or the mark lies about the jaws."""

    @pytest.mark.parametrize("mm", [0.0, 30.0, 60.0, 119.9, 120.0])
    def test_a_position_and_its_pixel_round_trip(self, slider, mm: float) -> None:
        assert slider._value_for_x(slider._x_for_mm(mm)) == pytest.approx(mm, abs=0.05)

    def test_the_two_ends_land_on_the_ends_of_the_groove(self, slider) -> None:
        groove = slider._groove_rect()

        assert slider._x_for_mm(0.0) == pytest.approx(groove.left())
        assert slider._x_for_mm(120.0) == pytest.approx(groove.right())

    def test_a_position_beyond_the_travel_is_drawn_at_the_end(self, slider) -> None:
        """It means the calibration and the encoder disagree; the mark says so
        by sitting at the end rather than by being drawn off the widget."""
        groove = slider._groove_rect()

        assert slider._x_for_mm(-5.0) == pytest.approx(groove.left())
        assert slider._x_for_mm(999.0) == pytest.approx(groove.right())


class TestItPaints:
    """It draws itself without a display, and without raising."""

    def test_it_paints_in_every_state(self, slider) -> None:
        from PyQt5.QtGui import QPixmap

        for setup in (
            lambda: None,
            lambda: slider.set_target(70.0),
            lambda: slider.set_actual(42.0),
            lambda: slider.set_owner("slider"),
            lambda: slider.set_blocked("标定未就绪"),
        ):
            setup()
            pixmap = QPixmap(WIDTH, MIN_HEIGHT)
            slider.render(pixmap)

    def test_the_tooltip_explains_a_block(self, slider) -> None:
        slider.set_blocked("缺少标定文件")

        assert "缺少标定文件" in slider.toolTip()
        slider.set_blocked("")
        assert "拖动" in slider.toolTip()
