"""The control page: what the operator does, and what the console does about it.

The slider's own behaviour is pinned in :mod:`tests.test_ui_slider`; this file is
about the page — which command each gesture produces, and the two pieces of
bookkeeping that are invisible until they are wrong: the echo guard, which stops
a frame published before a command from undoing it, and the owner, which decides
whether the handle represents the target or the measurement.
"""

from __future__ import annotations

import dataclasses

import pytest
from PyQt5.QtCore import QPoint, Qt
from PyQt5.QtTest import QTest

from litegrip_studio import calibration, constants
from litegrip_studio.calibration import CalibrationInfo
from litegrip_studio.core import commands as cmd
from litegrip_studio.core.worker import GateState
from litegrip_studio.settings import Settings
from litegrip_studio.telemetry import EMPTY_FRAME, TelemetryFrame
from litegrip_studio.ui.control_page import ECHO_FRAMES, ControlPage
from litegrip_studio.ui.slider import HANDLE_W, MIN_HEIGHT
from litegrip_studio.units import Limits

#: Wide enough that the groove is a round 400 px, so a fraction of the travel
#: lands on an exact millimetre rather than on a rounding.
WIDTH = 414

READY_REASON = "用户标定：/tmp/cal.json"

# The example user calibration: closed at 1.775959 rad, open at -0.064279.
LIMITS = Limits(1.775959, -0.064279, 65.21, 120.0)
USER_CAL = CalibrationInfo(
    provenance=calibration.PROVENANCE_USER, limits=LIMITS, path="/tmp/cal.json"
)

# The same two angles as they are read under the derived scale, where the
# recorded span (121 mm) and the commanded range (85 mm) are two different
# numbers.  Either one quoted as "where the button goes" would be a bug, and
# with the fixture above they coincide and the test could not tell them apart.
MEASURED_LIMITS = Limits(1.775959, -0.064279, 65.752365, 85.0)
MEASURED_CAL = CalibrationInfo(
    provenance=calibration.PROVENANCE_USER, limits=MEASURED_LIMITS, path="/tmp/cal.json"
)


def frame(**changes) -> TelemetryFrame:
    return dataclasses.replace(EMPTY_FRAME, **changes)


def sweep(slider, to_fraction: float = 0.75) -> None:
    """A real drag from 10% of the travel to ``to_fraction`` of it.

    Real, rather than a hand-emitted ``dragMoved``: which signals a drag
    produces is exactly what the live-follow switch changes, and emitting one by
    hand would test the page's handler while skipping the widget's gate.
    """
    slider.resize(WIDTH, MIN_HEIGHT)
    x_at = lambda fraction: int(  # noqa: E731 - one expression, used four times
        round(HANDLE_W / 2 + (WIDTH - HANDLE_W) * fraction)
    )
    QTest.mousePress(slider, Qt.LeftButton, Qt.NoModifier, QPoint(x_at(0.1), 20))
    for fraction in (0.3, 0.5, 0.7, to_fraction):
        QTest.mouseMove(slider, QPoint(x_at(fraction), 20))
    QTest.mouseRelease(slider, Qt.LeftButton, Qt.NoModifier, QPoint(x_at(to_fraction), 20))


def drag(page, mm: float) -> None:
    """What a real drag does: the handle is already at ``mm`` when it commits."""
    page.slider.set_value_mm(mm)
    page.slider.dragCommitted.emit(mm)


class Recorder:
    def __init__(self) -> None:
        self.commands: list[object] = []

    def __call__(self, command) -> None:
        self.commands.append(command)

    def of(self, kind) -> list:
        return [c for c in self.commands if isinstance(c, kind)]

    def last(self):
        assert self.commands, "nothing was submitted"
        return self.commands[-1]


@pytest.fixture
def page(qapp):
    recorder = Recorder()
    widget = ControlPage(recorder)
    widget.set_calibration(USER_CAL)
    widget.set_gate(GateState.READY, READY_REASON)
    # No settings object, so no test writes into the operator's real file; the
    # tests that are about remembering pass a store of their own.
    widget.recorder = recorder  # type: ignore[attr-defined]
    return widget


def drain(page) -> None:
    """Deliver enough frames for the echo guard to have expired."""
    page.recorder.commands.clear()
    for _ in range(ECHO_FRAMES + 1):
        page.update_frame(frame(position_mm=0.0))


class TestMovingFromTheSlider:
    def test_a_drag_becomes_one_move_at_the_position_under_the_cursor(self, page) -> None:
        drag(page, 42.5)

        assert page.recorder.last() == cmd.MoveToMm(target_mm=42.5, source="slider")

    def test_the_handle_keeps_the_target_the_operator_chose(self, page) -> None:
        drag(page, 90.0)
        page.update_frame(frame(position_mm=45.0, cmd_mm=90.0, motion_state="SERVO"))

        assert page.slider.value_mm == pytest.approx(90.0)
        assert page.slider.actual_mm == pytest.approx(45.0)

    def test_a_frame_with_no_error_figure_is_not_an_arrival(self, page) -> None:
        """``err_mm is None`` means unknown, not zero.  Reading it as zero would
        hand the handle back mid-move on any frame that did not carry one."""
        drag(page, 90.0)
        page.update_frame(frame(position_mm=45.0, cmd_mm=90.0, err_mm=None))

        assert page.slider.value_mm == pytest.approx(90.0)

    def test_the_target_line_is_what_was_commanded(self, page) -> None:
        drag(page, 90.0)

        assert page.slider.target_mm == pytest.approx(90.0)

    def test_live_following_sends_a_command_per_update(self, page) -> None:
        """Off by default, so this needs switching on; the queue coalesces the
        burst, which is what makes it safe."""
        page.slider.set_live_follow(True)
        page.recorder.commands.clear()

        page.slider.dragMoved.emit(10.0)
        page.slider.dragMoved.emit(20.0)
        page.slider.dragMoved.emit(30.0)

        assert [c.target_mm for c in page.recorder.of(cmd.MoveToMm)] == [10.0, 20.0, 30.0]

    def test_the_readout_shows_the_error_while_a_move_is_running(self, page) -> None:
        page.update_frame(frame(position_mm=20.0, cmd_mm=80.0, motion_state="SERVO"))

        assert page.readout.error.isVisibleTo(page.readout)


class TestBeforeTheAxisHasAnswered:
    """A frame carries ``position_mm=None`` until a status frame has arrived.

    The page is the last place that could turn that back into a number, and a
    number here is read as a position by the person holding the gripper.
    """

    def test_the_slider_shows_no_measurement_rather_than_zero(self, page) -> None:
        page.slider.set_value_mm(84.0)
        page.update_frame(frame(position_mm=None))

        assert page.slider.actual_mm is None
        assert page.slider.value_mm == pytest.approx(84.0)

    def test_the_readout_says_so_instead_of_printing_a_number(self, page) -> None:
        page.update_frame(frame(position_mm=None))

        assert "—" in page.readout.actual.text()
        assert "0.0 mm" not in page.readout.actual.text()

    def test_a_later_frame_puts_the_measurement_back(self, page) -> None:
        page.update_frame(frame(position_mm=None))
        page.update_frame(frame(position_mm=37.5))

        assert page.slider.actual_mm == pytest.approx(37.5)
        assert "37.5" in page.readout.actual.text()


class TestTheButtonMoves:
    def test_open_and_close_send_their_commands(self, page) -> None:
        page._open.click()
        page._close.click()

        assert isinstance(page.recorder.of(cmd.Open)[0], cmd.Open)
        assert isinstance(page.recorder.of(cmd.Close)[0], cmd.Close)

    def test_the_two_buttons_say_they_go_all_the_way(self, page) -> None:
        """"全部" is the point of them: the operator asks for the end of the
        travel, not a distance, and the two are named so that neither is mistaken
        for the adjacent-stop buttons beside them."""
        assert page._open.text() == "全部张开"
        assert page._close.text() == "全部闭合"

    def test_the_end_they_go_to_is_readable_before_pressing(self, page) -> None:
        page.set_calibration(USER_CAL)

        assert "120.0 mm" in page._open.toolTip()
        assert "0.0 mm" in page._close.toolTip()

    def test_the_tooltip_quotes_the_commanded_range_not_the_recorded_span(
        self, page
    ) -> None:
        """The number has to be the one the bar is working to.  Under the
        derived scale the recorded span (121 mm here) is *longer* than the
        commanded travel (85 mm), so a tooltip named after the span would send
        the operator looking for 36 mm the axis was never going to reach."""
        page.set_calibration(MEASURED_CAL)

        assert "85.0 mm" in page._open.toolTip()
        assert "121" not in page._open.toolTip()
        # …and it is the same number the bar spans, in the bar's own units.
        assert page.slider.maximum() == pytest.approx(85.0 * 10)

    def test_without_a_travel_the_tooltip_quotes_no_number(self, page) -> None:
        """A millimetre figure the console is not working to is worse than none
        — 120 mm here would be the SDK's nominal stroke, not this unit's."""
        page.set_calibration(None)

        assert "120" not in page._open.toolTip()
        assert "尚未读到标定" in page._open.toolTip()
        assert "尚未读到标定" in page._close.toolTip()

    def test_grasp_carries_the_force_in_the_spinbox(self, page) -> None:
        page._force.setValue(22.5)
        page._grasp.click()

        grasp = page.recorder.of(cmd.Grasp)[-1]
        assert grasp.force_n == pytest.approx(22.5)

    def test_a_button_move_carries_the_handle_along(self, page) -> None:
        """The bar has to fill during an 开 / 合 move — that is the visible half
        of "the progress bar updates while the gripper moves"."""
        page._open.click()
        page.update_frame(frame(position_mm=64.0, cmd_mm=120.0, motion_state="SERVO"))

        assert page.slider.value_mm == pytest.approx(64.0)

    def test_the_release_after_a_grasp_sends_its_own_command(self, page) -> None:
        """``BackOff`` and not a computed ``MoveToMm``: the distance is measured
        from where the jaws are, which is the worker's reading to make."""
        page._back_off.click()

        assert len(page.recorder.of(cmd.BackOff)) == 1

    def test_the_release_sits_under_the_grasp(self, page) -> None:
        """One gesture, two halves — clamp it, then let go — and the column
        says which order they come in."""
        column = page._grasp.parentWidget().layout()

        assert column.indexOf(page._back_off) == column.indexOf(page._grasp) + 1

    def test_the_release_button_quotes_the_distance_it_opens(self, page) -> None:
        """The number is the one the worker will use, from the one constant the
        two share — a tooltip with its own ten millimetres would be a second
        copy to keep in step."""
        assert f"{constants.RELEASE_OPEN_MM:.1f} mm" in page._back_off.toolTip()
        assert cmd.BackOff().delta_mm == constants.RELEASE_OPEN_MM

    def test_stop_and_release_are_sent(self, page) -> None:
        page._stop.click()
        page._release.click()

        assert isinstance(page.recorder.of(cmd.Stop)[0], cmd.Stop)
        assert isinstance(page.recorder.of(cmd.Release)[0], cmd.Release)

    def test_stop_hands_the_handle_back_to_the_measurement(self, page) -> None:
        drag(page, 90.0)
        page._stop.click()
        drain(page)
        page.update_frame(frame(position_mm=45.0))

        assert page.slider.value_mm == pytest.approx(45.0)


class TestTheEchoGuard:
    """The frame published just after a command describes the state before it."""

    def test_a_stale_frame_does_not_pull_the_handle_out_of_the_hand(self, page) -> None:
        """Without the guard, the handle snaps back to the measured position the
        moment the operator lets go, which is the one thing the slider exists
        not to do."""
        drag(page, 90.0)

        page.update_frame(frame(position_mm=0.0, cmd_mm=None, motion_state="IDLE"))

        assert page.slider.value_mm == pytest.approx(90.0)

    def test_a_stale_frame_does_not_erase_the_target_line(self, page) -> None:
        drag(page, 90.0)

        page.update_frame(frame(position_mm=0.0, cmd_mm=None))

        assert page.slider.target_mm == pytest.approx(90.0)

    def test_once_the_command_is_echoed_the_worker_is_the_authority(self, page) -> None:
        drag(page, 90.0)
        page.update_frame(frame(position_mm=1.0, cmd_mm=88.0, motion_state="SERVO"))

        assert page.slider.target_mm == pytest.approx(88.0)

    def test_the_guard_expires_so_a_refused_command_does_not_wedge_the_slider(self, page) -> None:
        drag(page, 90.0)
        drain(page)

        page.update_frame(frame(position_mm=45.0, cmd_mm=None))

        assert page.slider.value_mm == pytest.approx(45.0)


class TestTheOwnerIsReleased:
    def test_arriving_hands_the_handle_back(self, page) -> None:
        drag(page, 90.0)
        page.update_frame(frame(position_mm=90.0, cmd_mm=90.0, err_mm=0.0))

        page.update_frame(frame(position_mm=90.2, cmd_mm=90.0, err_mm=-0.2))

        assert page.slider.value_mm == pytest.approx(90.2)

    def test_a_move_that_is_still_running_keeps_it(self, page) -> None:
        drag(page, 90.0)
        page.update_frame(
            frame(position_mm=50.0, cmd_mm=90.0, err_mm=40.0, moving=True,
                  motion_state="SERVO")
        )

        assert page.slider.value_mm == pytest.approx(90.0)


class TestTheGate:
    def test_a_fresh_page_accepts_nothing(self, qapp) -> None:
        """Before the worker has said anything about the calibration, the page
        must not offer to move the axis: the gate is the worker's to open, and
        the console's default has to be shut."""
        page = ControlPage(Recorder())

        assert not page.slider.isEnabled()
        assert not page._open.isEnabled()
        assert not page._close.isEnabled()
        assert not page._grasp.isEnabled()
        assert not page._back_off.isEnabled()
        assert "尚未" in page.slider.toolTip()

    def test_a_blocked_gate_refuses_the_slider_and_names_the_reason(self, page) -> None:
        page.set_gate(GateState.BLOCKED, "缺少标定文件")

        assert not page.slider.isEnabled()
        assert "缺少标定文件" in page.slider.toolTip()

    def test_a_blocked_gate_disables_the_moves_but_not_the_stops(self, page) -> None:
        """停止 and 松力 are the two controls whose purpose is to stop rather
        than to go anywhere; refusing them would be refusing to let go.

        放开 is *not* one of them: it is a millimetre command derived from the
        travel, so with the travel in doubt it stays refused, and the two above
        are what an operator reaches for instead.
        """
        page.set_gate(GateState.BLOCKED, "缺少标定文件")

        assert not page._open.isEnabled()
        assert not page._close.isEnabled()
        assert not page._grasp.isEnabled()
        assert not page._back_off.isEnabled()
        assert page._stop.isEnabled()
        assert page._release.isEnabled()

    def test_the_factory_gate_is_not_ready_either(self, page) -> None:
        page.set_gate(GateState.FACTORY, "正在使用出厂标定")

        assert not page.slider.isEnabled()

    def test_opening_the_gate_restores_the_control(self, page) -> None:
        page.set_gate(GateState.BLOCKED, "缺少标定文件")
        page.set_gate(GateState.READY, READY_REASON)

        assert page.slider.isEnabled()
        assert page._open.isEnabled()
        assert page._back_off.isEnabled()
        assert page.slider.toolTip().startswith("拖动")


class TestSpeedAndForce:
    def test_the_speed_slider_commands_the_worker(self, page) -> None:
        page.recorder.commands.clear()
        page._speed.setValue(77)

        assert page.recorder.of(cmd.SetSpeed)[-1].speed_mm_s == pytest.approx(77.0)

    def test_the_force_spinbox_commands_the_worker(self, page) -> None:
        page.recorder.commands.clear()
        page._force.setValue(18.0)

        assert page.recorder.of(cmd.SetForce)[-1].force_n == pytest.approx(18.0)

    def test_the_grasp_button_says_what_it_will_do(self, page) -> None:
        page._force.setValue(18.0)

        assert "18.0" in page._grasp.text()

    def test_a_force_near_the_rating_is_flagged_before_it_is_reached(self, page) -> None:
        page._force.setValue(constants.FORCE_SOFT_WARN_N + 1.0)

        assert "color" in page._force.styleSheet()

    def test_a_force_at_the_rating_is_flagged_harder(self, page) -> None:
        page._force.setValue(constants.FORCE_MAX_N)
        at_max = page._force.styleSheet()

        page._force.setValue(constants.FORCE_SOFT_WARN_N + 1.0)
        assert page._force.styleSheet() != at_max

    def test_the_speed_cannot_be_set_outside_the_profile_range(self, page) -> None:
        page._speed.setValue(9999)

        assert page.speed_mm_s == pytest.approx(constants.SPEED_MAX_MM_S)


class TestLiveFollowing:
    """The switch is the slider's to honour; these tests drive real gestures
    through it, because emitting ``dragMoved`` by hand would bypass the very
    gate under test."""

    def test_it_is_off_until_the_operator_asks(self, page) -> None:
        """Off by default: a console that commands a move for every pixel of a
        drag is a console that fills the log and the queue with intent."""
        assert not page.live_follow
        assert not page.slider._live_follow

    def test_ticking_it_reaches_the_slider(self, page) -> None:
        page._live.setChecked(True)

        assert page.slider._live_follow

    def test_a_drag_off_commits_once(self, page) -> None:
        page.recorder.commands.clear()

        sweep(page.slider)

        assert len(page.recorder.of(cmd.MoveToMm)) == 1
        assert page.recorder.of(cmd.MoveToMm)[-1].target_mm == pytest.approx(90.0)

    def test_a_live_drag_commands_every_step(self, page) -> None:
        page._live.setChecked(True)
        page.recorder.commands.clear()

        sweep(page.slider)

        assert len(page.recorder.of(cmd.MoveToMm)) > 1
        assert page.recorder.of(cmd.MoveToMm)[-1].target_mm == pytest.approx(90.0)

    def test_the_last_command_of_a_live_drag_is_still_the_release_point(self, page) -> None:
        """A queue coalesces the burst, so whatever the stream costs, what
        reaches the motor is the position the operator let go at."""
        page._live.setChecked(True)
        page.recorder.commands.clear()

        sweep(page.slider)

        assert page.recorder.of(cmd.MoveToMm)[-1] == cmd.MoveToMm(
            target_mm=90.0, source="slider"
        )

    def test_it_is_remembered_as_the_operator_leaves_it(self, qapp) -> None:
        from litegrip_studio.settings import KEY_LIVE_FOLLOW

        store = _Store(**{KEY_LIVE_FOLLOW: "true"})
        page = ControlPage(Recorder(), Settings(store))

        assert page.live_follow

        page._live.setChecked(False)

        assert store.value(KEY_LIVE_FOLLOW) is False

    def test_unticking_it_stops_the_stream(self, page) -> None:
        page._live.setChecked(True)
        page._live.setChecked(False)
        page.recorder.commands.clear()

        sweep(page.slider)

        assert len(page.recorder.of(cmd.MoveToMm)) == 1


class TestRemembering:
    def test_the_speed_and_force_are_restored_and_saved(self, qapp) -> None:
        from litegrip_studio.settings import KEY_FORCE, KEY_SPEED

        store = _Store(**{KEY_SPEED: "88", KEY_FORCE: "9.5"})
        page = ControlPage(Recorder(), Settings(store))

        assert page.speed_mm_s == pytest.approx(88.0)
        assert page.force_n == pytest.approx(9.5)

        page._speed.setValue(66)
        page._speed.sliderReleased.emit()
        assert store.value(KEY_SPEED) == 66

    def test_a_drag_does_not_write_the_file_on_every_step(self, qapp) -> None:
        """A drag is a hundred values; a hundred fsyncs is a lot of disk for one
        number."""
        store = _Store()
        page = ControlPage(Recorder(), Settings(store))
        store.syncs = 0

        for value in range(10, 60):
            page._speed.setValue(value)

        assert store.syncs == 0
        page._speed.sliderReleased.emit()
        assert store.syncs >= 1


class _Store:
    """A ``QSettings``-shaped dictionary, as :mod:`tests.test_settings` uses."""

    def __init__(self, **initial) -> None:
        self.data = dict(initial)
        self.syncs = 0

    def value(self, key, default=None):
        return self.data.get(key, default)

    def setValue(self, key, value) -> None:
        self.data[key] = value

    def remove(self, key) -> None:
        self.data.pop(key, None)

    def sync(self) -> None:
        self.syncs += 1
