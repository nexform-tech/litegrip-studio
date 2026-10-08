"""The window as a whole: that it builds, and that it is wired to the worker.

Every page is driven by a plain slot method, so the whole console can be
exercised here against a fake worker that emits the signals the real one emits.
No thread, no SDK, no sleeping — and, because the wiring is the thing most
likely to be wrong while every page's own tests pass, this is the file that
would notice a signal connected to the wrong slot or not at all.
"""

from __future__ import annotations

import dataclasses
import sys

import pytest
from PyQt5.QtCore import QObject, pyqtSignal
from PyQt5.QtWidgets import QPushButton

from litegrip_studio import calibration, constants
from litegrip_studio.calibration import CalibrationInfo
from litegrip_studio.core import commands as cmd
from litegrip_studio.core.calibration_fsm import GuidedPhase, TwoPointPhase
from litegrip_studio.core.worker import (
    CONN_CONNECTED,
    CONN_CONNECTING,
    CONN_DISCONNECTED,
    CONN_ERROR,
    GateState,
    GripperWorker,
)
from litegrip_studio.settings import KEY_TAB, KEY_THEME, Settings
from litegrip_studio.telemetry import EMPTY_FRAME, TelemetryFrame
from litegrip_studio.ui import theme
from litegrip_studio.ui.main_window import TAB_NAMES, MainWindow
from litegrip_studio.units import Limits

USER_LIMITS = Limits(1.775959, -0.064279, 65.21, 120.0)
USER_CAL = CalibrationInfo(
    provenance=calibration.PROVENANCE_USER, limits=USER_LIMITS, path="/tmp/cal.json"
)


def frame(**changes) -> TelemetryFrame:
    return dataclasses.replace(EMPTY_FRAME, **changes)


def _signal_names(cls) -> set[str]:
    """Every ``pyqtSignal`` the class or its own bases declare, by name.

    Read out of the class ``__dict__``\\ s rather than by ``dir``: on a class, a
    signal is an unbound ``pyqtSignal`` and never a ``pyqtBoundSignal``, so a
    ``dir`` scan that looks for the bound type finds nothing on either side and
    compares two empty sets.  The walk stops at ``QObject`` and ``QThread``
    because ``destroyed`` and ``started`` belong to Qt, and counting them would
    only pad both sides of the comparison.
    """
    from PyQt5.QtCore import QThread, pyqtSignal

    names: set[str] = set()
    for klass in cls.__mro__:
        if klass in (QObject, QThread, object):
            break
        names |= {n for n, v in vars(klass).items() if isinstance(v, pyqtSignal)}
    return names


class FakeSignals(QObject):
    """The worker's signals, with nothing behind them.

    Declared on the worker itself rather than on an attribute of it, because that
    is where :class:`~litegrip_studio.core.worker.GripperWorker` declares them
    and :class:`MainWindow` connects to whatever the worker actually has.  A fake
    with a different shape is a fake that passes while the console cannot start,
    which is how this file once missed exactly that.
    """

    telemetry = pyqtSignal(object)
    motion_state = pyqtSignal(str)
    conn_state = pyqtSignal(str, str)
    fault = pyqtSignal(int, str, str)
    gate_state = pyqtSignal(str, str)
    calib_info = pyqtSignal(object)
    calib_progress = pyqtSignal(str, float, str)
    log = pyqtSignal(str, str)
    alert = pyqtSignal(str, str)
    alert_cleared = pyqtSignal()
    busy = pyqtSignal(bool, str)


class FakeBackend:
    def describe(self) -> str:
        return "仿真 (Plant) | 用户标定"


class FakeWorker(FakeSignals):
    """Everything :class:`MainWindow` uses of the worker, and nothing else."""

    def __init__(self) -> None:
        super().__init__()
        self.backend = FakeBackend()
        self.commands: list[object] = []
        self.estopped = False
        self.estop_reasons: list[str] = []
        self.shutdowns = 0
        self.shutdown_ok = True

    def submit(self, command) -> int:
        self.commands.append(command)
        return len(self.commands)

    def estop(self, reason: str = "") -> None:
        self.estopped = True
        self.estop_reasons.append(reason)

    def shutdown(self, timeout_ms: int = 0) -> bool:
        self.shutdowns += 1
        return self.shutdown_ok

    def of(self, kind) -> list:
        return [c for c in self.commands if isinstance(c, kind)]


class _Store:
    def __init__(self, **initial) -> None:
        self.data = dict(initial)

    def value(self, key, default=None):
        return self.data.get(key, default)

    def setValue(self, key, value) -> None:
        self.data[key] = value

    def remove(self, key) -> None:
        self.data.pop(key, None)

    def sync(self) -> None:
        pass


@pytest.fixture
def worker() -> FakeWorker:
    return FakeWorker()


@pytest.fixture
def window(qapp, worker) -> MainWindow:
    widget = MainWindow(worker)
    widget.start()
    # Not shown: offscreen tests are about the wiring, and show() would start
    # the heartbeat timer against a worker that is not running.
    widget._heartbeat.stop()
    return widget


class TestItBuilds:
    def test_the_fake_worker_has_exactly_the_real_workers_signals(self) -> None:
        """The one test that can notice the fake and the worker drifting apart.

        Every other test in this file drives the window through the fake, so a
        signal the worker has and the fake does not — or the other way round — is
        invisible until the console is launched against the real thing.  It costs
        one comparison to make that impossible.
        """
        real = _signal_names(GripperWorker)
        fake = _signal_names(FakeWorker)

        assert fake == real

    def test_every_page_is_there_under_its_name(self, window) -> None:
        names = [window.tabs.tabText(i) for i in range(window.tabs.count())]

        assert tuple(names) == TAB_NAMES

    def test_the_backend_describes_itself_before_the_first_frame(self, window) -> None:
        assert "仿真" in window.connect_bar._target.text()

    def test_the_estop_is_visible_from_every_tab(self, window) -> None:
        for index in range(window.tabs.count()):
            window.tabs.setCurrentIndex(index)
            assert window.estop_button.isVisibleTo(window)


class TestTheWiring:
    def test_a_frame_reaches_every_page(self, window) -> None:
        window.worker.telemetry.emit(
            frame(position_mm=42.0, enabled=True, error_code=constants.ERROR_ENABLED)
        )

        assert window.control_page.slider.actual_mm == pytest.approx(42.0)
        assert window.status_page.value["位置"] == "42.00 mm"
        assert window.plots_page.samples
        assert "42.00" in window.calibration_page._position.text()
        assert "42.00 mm" in window._status_position.text()

    def test_a_frame_with_no_measurement_says_so_on_every_page(self, window) -> None:
        """The first frame after 使能 may not carry a position yet.  Every page
        that shows one has to render that as unknown — a page that showed
        0.00 mm would be showing the closed stop."""
        window.worker.telemetry.emit(frame(position_mm=None))

        assert window.control_page.slider.actual_mm is None
        assert window.status_page.value["位置"] == "—"
        assert "0.00" not in window.calibration_page._position.text()
        assert "0.00" not in window._status_position.text()

    def test_the_calibration_reaches_the_slider_and_the_plots(self, window, worker) -> None:
        """One signal, three pages: the slider spans it, the plots range to it,
        and the calibration page says where it came from.  The bar takes its
        *range* from the calibration and its *permission* from the gate, so it
        stays shut here — knowing how wide the gripper is is not licence to
        drive it."""
        worker.calib_info.emit(USER_CAL)

        assert not window.control_page.slider.isEnabled()
        low, high = window.plots_page._position_plot.getViewBox().viewRange()[1]
        assert high == pytest.approx(120.0, abs=0.5)

    def test_nothing_may_be_dragged_before_the_gate_is_known(self, window) -> None:
        """The console's own default, before the worker has said anything."""
        assert not window.control_page.slider.isEnabled()
        assert not window.control_page._open.isEnabled()
        assert "尚未" in window.control_page.slider.toolTip()

    def test_the_gate_reaches_every_page_that_shows_it(self, window, worker) -> None:
        worker.calib_info.emit(USER_CAL)
        worker.gate_state.emit(GateState.READY.value, "用户标定：/tmp/cal.json")

        assert window.control_page.slider.isEnabled()
        assert window.status_page.gate_alert[0] is None
        assert "就绪" in window.calibration_page._gate_label.text()
        assert "READY" in window._status_gate.text()

    def test_a_blocked_gate_reaches_the_slider(self, window, worker) -> None:
        worker.gate_state.emit(GateState.BLOCKED.value, "缺少标定文件")

        assert not window.control_page.slider.isEnabled()
        assert "缺少标定文件" in window.control_page.slider.toolTip()
        assert "缺少标定文件" in window.status_page.gate_alert[2]

    def test_the_connection_state_reaches_the_bar(self, window, worker) -> None:
        worker.conn_state.emit(CONN_CONNECTED, "已连接")

        assert "已连接" in window.connect_bar._conn_label.text()

    def test_a_connected_enabled_console_offers_the_guided_probe(
        self, window, worker
    ) -> None:
        """Connected and enabled are two signals, and the probe needs both.

        The page accepted them all along; what was missing was the window
        passing the first one on, which left 自动标定 greyed out on a console
        that was connected and enabled — the whole of the reported bug, in one
        assertion."""
        worker.conn_state.emit(CONN_CONNECTED, "已连接")
        worker.telemetry.emit(frame(enabled=True, error_code=constants.ERROR_ENABLED))

        assert window.calibration_page._guided_start.isEnabled()
        assert window.calibration_page._manual_start.isEnabled()

    def test_a_dropped_connection_offers_no_probe(self, window, worker) -> None:
        worker.conn_state.emit(CONN_CONNECTED, "已连接")
        worker.telemetry.emit(frame(enabled=True, error_code=constants.ERROR_ENABLED))

        worker.conn_state.emit(CONN_DISCONNECTED, "已断开")

        assert not window.calibration_page._guided_start.isEnabled()

    def test_a_fault_is_shown_without_opening_a_tab(self, window, worker) -> None:
        worker.fault.emit(
            constants.ERROR_UV, "欠压故障 (UV)", constants.FAULT_HINTS[constants.ERROR_UV]
        )

        assert window.alert_banner.severity == "error"
        assert "欠压" in window.alert_banner.headline

    def test_an_alert_reaches_the_window_banner(self, window, worker) -> None:
        worker.alert.emit("warn", "60 秒内多次欠压，已限速")

        assert window.alert_banner.severity == "warn"
        assert "欠压" in window.alert_banner.headline

    def test_the_worker_can_take_its_alert_back(self, window, worker) -> None:
        """The reported bug: an alert that stayed on screen after the console had
        recovered.  The worker watches the condition now and says when it is
        gone; the banner has to obey."""
        worker.alert.emit("error", "「移动」被拒绝：电机未使能")
        assert window.alert_banner.severity == "error"

        worker.alert_cleared.emit()

        assert window.alert_banner.severity is None
        assert not window.alert_banner.isVisible()

    def test_a_success_does_not_take_the_banner(self, window, worker) -> None:
        """``info`` is how the worker reports something that went right.  There is
        nothing to act on, and a strip saying the save worked is one more thing
        between the operator and the alert that matters."""
        worker.alert.emit("info", "标定已保存到 /home/qaz/.litegrip/calibration.json")

        assert window.alert_banner.severity is None
        assert any("标定已保存" in text for _level, text in window.lines)

    def test_an_estop_notice_is_not_a_fault_to_redden_the_window_over(
        self, window, worker
    ) -> None:
        """The latch has a banner of its own, with the reset button in it, and it
        is cleared by the motion state.  Painting the notification red as well
        left the E-stop's text up after the operator had reset it."""
        worker.fault.emit(0, "急停已复位", "请确认现场安全后再使能")

        assert window.alert_banner.severity is None
        assert any("急停已复位" in text for _level, text in window.lines)
        assert not window.estop_banner.isVisible()

    def test_a_probe_switches_to_the_page_that_can_be_watched(self, window, worker) -> None:
        """The slider is dead during a probe — there is no valid travel to span
        yet — so the operator has to be looking at the page that has the live
        reading."""
        window.tabs.setCurrentIndex(0)

        worker.calib_progress.emit(TwoPointPhase.RECORD_OPEN.value, 0.1, "")

        assert window.tabs.currentWidget() is window.calibration_page
        assert window.calibration_page.phase == TwoPointPhase.RECORD_OPEN.value

    def test_the_dock_is_not_forced_open_by_a_probe(self, window, worker) -> None:
        window.tabs.setCurrentIndex(1)

        worker.calib_progress.emit(GuidedPhase.OPEN_PROBE.value, 0.1, "")

        assert window.tabs.currentWidget() is window.tabs.widget(1)

    def test_busy_is_said_in_the_state_line_and_on_the_calibration_page(
        self, window, worker
    ) -> None:
        worker.busy.emit(True, "正在使能（可能需要数秒）…")

        assert "使能" in window._status_message.text()
        assert not window.calibration_page._guided_start.isEnabled()

    def test_the_busy_message_comes_down_again(self, window, worker) -> None:
        """The status bar this replaced only ever set the message —
        ``showMessage`` with no timeout stays up until something clears it — so
        the first busy spell of a session held the line for the rest of it, and
        with it the readouts that shared that slot."""
        worker.busy.emit(True, "正在使能（可能需要数秒）…")

        worker.busy.emit(False, "")

        assert window._status_message.text() == ""

    def test_a_motion_state_change_is_logged(self, window, worker) -> None:
        """Nothing else records when the axis left SERVO, which is the first
        question asked of the log after an unexpected movement."""
        worker.motion_state.emit("SERVO")

        assert ("state", "运动状态：SERVO") in window.lines


class TestTheLog:
    def test_the_debug_filter_lives_on_the_log_header(self, window) -> None:
        """The filter is about the log, so it belongs on the log's header.

        It used to sit on a row of its own inside the body, where it started
        10px to the left of the title above it: the style draws a title bar with
        its own padding and none of the body shares it, so the filter read as a
        control that had escaped its panel.  Moving it onto the header is what
        fixed that, and this is what stops it drifting back.
        """
        header = window._log_dock.titleBarWidget()

        assert header is not None, "the dock supplies its own title bar"
        assert header.isAncestorOf(window._show_debug)
        assert not window._log_dock.widget().isAncestorOf(window._show_debug)

    def test_it_shows_what_the_worker_sends(self, window, worker) -> None:
        worker.log.emit("info", "电机已使能")

        assert "电机已使能" in window._log.toPlainText()

    def test_debug_lines_are_kept_but_hidden(self, window, worker) -> None:
        """Every command is one, and they would bury the lines that describe
        what happened; the file still gets them."""
        worker.log.emit("debug", "命令: 移动到 42.0 mm")

        assert "命令" not in window._log.toPlainText()
        assert ("debug", "命令: 移动到 42.0 mm") in window.lines

    def test_turning_debug_on_replays_what_was_already_hidden(self, window, worker) -> None:
        worker.log.emit("debug", "命令: 移动到 42.0 mm")
        worker.log.emit("info", "电机已使能")

        window._show_debug.setChecked(True)

        text = window._log.toPlainText()
        assert "命令" in text
        assert "电机已使能" in text

    def test_turning_it_off_again_hides_only_the_debug_lines(self, window, worker) -> None:
        worker.log.emit("debug", "命令: 移动到 42.0 mm")
        worker.log.emit("warn", "命令队列已满")

        window._show_debug.setChecked(True)
        window._show_debug.setChecked(False)

        text = window._log.toPlainText()
        assert "命令队列已满" in text
        assert "移动到" not in text


class TestTheEstop:
    def test_the_button_stops_the_motor_without_queuing(self, window, worker) -> None:
        """Behind fifty coalesced drag commands is not an emergency stop."""
        window.estop_button.click()

        assert worker.estopped
        assert worker.estop_reasons
        assert not worker.of(cmd.Heartbeat)

    def test_the_keyboard_does_it_too(self, window, worker) -> None:
        window._estop_shortcut.activated.emit()

        assert worker.estopped

    def test_the_latch_shows_a_banner_with_the_way_out(self, window, worker) -> None:
        worker.motion_state.emit("ESTOP")

        assert window.estop_banner.severity == "error"
        assert "闩锁" in window.estop_banner.headline
        assert "复位" in window.estop_banner.detail
        assert window.estop_banner.findChild(QPushButton) is not None

    def test_the_reset_button_sends_the_command(self, window, worker) -> None:
        worker.motion_state.emit("ESTOP")
        window.estop_banner.findChild(QPushButton).click()

        assert isinstance(worker.commands[-1], cmd.ResetEStop)

    def test_the_banner_goes_when_the_latch_does(self, window, worker) -> None:
        worker.motion_state.emit("ESTOP")
        worker.motion_state.emit("IDLE")

        assert window.estop_banner.severity is None

    def test_a_motion_state_that_is_not_the_estop_does_not_clear_a_banner(
        self, window, worker
    ) -> None:
        """Only the latch coming off may clear it; a state change while it is
        still latched is not the operator having acknowledged anything."""
        worker.motion_state.emit("ESTOP")

        assert window.estop_banner.severity == "error"
        window.set_estopped(True)
        assert window.estop_banner.severity == "error"


class TestTheHeartbeat:
    def test_the_timer_is_running_so_the_worker_watchdog_is_fed(self, qapp, worker) -> None:
        window = MainWindow(worker)

        assert window._heartbeat.isActive()
        assert window._heartbeat.interval() == constants.HEARTBEAT_INTERVAL_MS
        window._heartbeat.stop()

    def test_it_sends_a_heartbeat(self, window, worker) -> None:
        window._heartbeat.timeout.emit()

        assert worker.of(cmd.Heartbeat)


class TestWhatIsRemembered:
    def test_the_open_tab_and_the_log_filter_come_back(self, qapp, worker) -> None:
        store = _Store(**{KEY_TAB: "2", "view/show_debug_log": "true"})

        window = MainWindow(worker, Settings(store))
        window._heartbeat.stop()

        assert window.tabs.currentIndex() == 2
        assert window._show_debug.isChecked()

    def test_the_window_state_is_written_on_close(self, qapp, worker) -> None:
        store = _Store()
        window = MainWindow(worker, Settings(store))
        window._heartbeat.stop()
        window.tabs.setCurrentIndex(3)

        window.close()

        assert store.value(KEY_TAB) == 3
        assert store.value("view/window_state") is not None

    def test_the_page_preferences_are_written_on_close(self, qapp, worker) -> None:
        """The speed is written on release; the close is the last chance for
        whatever the operator changed but never confirmed.  The travel was the
        other one of these until it stopped being a preference."""
        store = _Store()
        window = MainWindow(worker, Settings(store))
        window._heartbeat.stop()
        window.control_page._speed.setValue(77)

        window.close()

        assert store.value("motion/speed_mm_s") == 77
        assert store.value("calibration/travel_mm") is None


class TestClosing:
    def test_the_worker_is_shut_down_once(self, qapp, worker) -> None:
        window = MainWindow(worker)
        window._heartbeat.stop()

        window.close()

        assert worker.shutdowns == 1
        assert not window._heartbeat.isActive()

    def test_a_thread_that_will_not_stop_is_reported_rather_than_hidden(
        self, qapp, worker
    ) -> None:
        """The window closing is not the same as the motor being off."""
        worker.shutdown_ok = False
        window = MainWindow(worker)
        window._heartbeat.stop()

        window.close()

        assert any("未在超时内退出" in text for _level, text in window.lines)


class TestTheThemeIsSwitchedFromTheWindow:
    """The palette moves from one control, and nothing else moves with it.

    A theme is the one preference that touches every widget at once, so the two
    ways it can go wrong are both worth a test: a control that changes how
    everything looks without being reachable from the page you are on, and a
    repaint that goes through a slot which also does something — the speed and
    force fields are re-rendered by the same code that commands the motor, and
    a switch that reached the commanding half would re-command the gripper.
    """

    @pytest.fixture(autouse=True)
    def restore(self):
        before = theme.current_theme()
        yield
        theme.set_theme(before)

    def test_asking_for_a_theme_moves_the_palette_and_remembers_it(self, qapp, worker) -> None:
        store = _Store()
        window = MainWindow(worker, Settings(store))
        window._heartbeat.stop()
        target = theme.LIGHT if theme.current_theme() == theme.DARK else theme.DARK

        window.theme_switch.button(target).click()

        assert theme.current_theme() == target
        assert store.value(KEY_THEME) == target
        assert window.theme_switch.current == target
        window.close()

    def test_both_names_are_on_screen_at_once(self, window) -> None:
        """The whole point of the pair: a single toggle labelled with the state
        it is in cannot say whether it describes now or what a click will do."""
        for name in (theme.DARK, theme.LIGHT):
            button = window.theme_switch.button(name)
            assert button.isVisibleTo(window)
            assert button.text() == theme.PALETTES[name].label

    def test_the_control_is_reachable_from_every_tab(self, window) -> None:
        for index in range(window.tabs.count()):
            window.tabs.setCurrentIndex(index)
            assert window.theme_switch.isVisibleTo(window)

    def test_switching_commands_nothing(self, window, worker) -> None:
        worker.commands.clear()

        window.theme_switch.choose(theme.LIGHT)

        assert worker.commands == []

    def test_it_reaches_the_application_and_not_only_the_tokens(
        self, qapp, window
    ) -> None:
        """Moving the tokens is not the same as telling the application, and a
        console that only did the first would keep the old chrome on every
        widget that is styled by the sheet rather than by an inline one."""
        from PyQt5.QtGui import QPalette

        window.theme_switch.choose(
            theme.LIGHT if theme.current_theme() == theme.DARK else theme.DARK
        )

        assert qapp.styleSheet() == theme.stylesheet()
        assert qapp.palette().color(QPalette.Window).name() == theme.BACKGROUND

    def test_every_page_is_repainted(self, window, worker) -> None:
        """Driven through the worker first, so each page is holding content it
        painted itself rather than the state it booted in."""
        worker.conn_state.emit(CONN_CONNECTED, "已连接")
        worker.calib_info.emit(USER_CAL)
        worker.gate_state.emit(GateState.READY.value, "")
        worker.telemetry.emit(frame(position_mm=42.0, enabled=True))
        worker.alert.emit("warn", "夹持力接近上限")

        def painted() -> tuple:
            return (
                window.status_page._values["位置"].styleSheet(),
                window.calibration_page._gate_label.text(),
                window.status_page._link_dot.styleSheet(),
                window.alert_banner.styleSheet(),
                window._status_gate.text(),
            )

        before = painted()
        window.theme_switch.choose(
            theme.LIGHT if theme.current_theme() == theme.DARK else theme.DARK
        )
        after = painted()

        labels = ("status value", "gate label", "link dot", "alert banner",
                  "status bar")
        for name, was, now in zip(labels, before, after):
            assert was != now, f"{name} kept the old palette"


class TestEveryHandlerReachesItsEnd:
    """A slot that raises is a slot that stops, and Qt says nothing about it.

    PyQt prints an exception out of a slot and carries on, so a handler that
    dies halfway leaves the window looking perfectly alive while one page
    silently stops receiving anything.  That is exactly how the guided probe
    came to be unusable: ``conn_state`` reached the connection bar — the first
    statement — and then raised on the second, before the calibration page's
    turn, which is a symptom no page's own tests could ever see.  It took a
    signal-level test that watches for the exception itself.
    """

    @pytest.fixture
    def raised(self, monkeypatch) -> list[BaseException]:
        """Everything Qt would otherwise have printed and swallowed."""
        caught: list[BaseException] = []
        monkeypatch.setattr(
            sys, "excepthook", lambda kind, value, tb: caught.append(value)
        )
        return caught

    def test_no_slot_raises_on_any_of_the_workers_signals(
        self, window, worker, raised
    ) -> None:
        worker.telemetry.emit(frame(enabled=True, error_code=constants.ERROR_ENABLED))
        worker.motion_state.emit("SERVO")
        worker.motion_state.emit("ESTOP")
        # All four connection states, because the handler branches on them and
        # the branch that raised was the one the common path does not take.
        for state, detail in (
            (CONN_CONNECTING, ""),
            (CONN_CONNECTED, "已连接"),
            (CONN_DISCONNECTED, "已断开"),
            (CONN_ERROR, "can0 不存在"),
        ):
            worker.conn_state.emit(state, detail)
        worker.fault.emit(
            constants.ERROR_UV,
            "欠压故障 (UV)",
            constants.FAULT_HINTS[constants.ERROR_UV],
        )
        worker.gate_state.emit(GateState.READY.value, "用户标定：/tmp/cal.json")
        worker.calib_info.emit(USER_CAL)
        worker.calib_progress.emit(GuidedPhase.OPEN_PROBE.value, 0.2, "等待确认")
        worker.log.emit("info", "已连接")
        worker.alert.emit("warn", "60 秒内多次欠压，已限速")
        worker.busy.emit(True, "正在使能（可能需要数秒）…")

        assert raised == []


class TestTheGateIsNotTheOnlyThingShown:
    def test_a_disconnected_console_still_offers_every_page(self, window, worker) -> None:
        """Nothing is hidden when the link drops: the operator needs the status
        page and the log more then, not less."""
        worker.conn_state.emit(CONN_DISCONNECTED, "")
        worker.conn_state.emit(CONN_ERROR, "can0 不存在")

        assert window.tabs.count() == len(TAB_NAMES)
        assert "失败" in window.connect_bar._conn_label.text()


class TestTheWindowCanBeMadeSmallEnoughToLiveOnTheScreen:
    """The console has to be movable, and a window taller than the display puts
    its own title bar past the edge of it.

    Qt will not shrink a window below its tallest page's minimum size, so one
    page's content can raise the floor for the whole console.  It did: the
    calibration page's stack made the minimum 1089 pixels, on a screen that has
    fewer than that, and the window opened with nothing to grab.

    That is the failure this pins — not a size that looks nice, but a size the
    window can actually be dragged down to.  A page that grows past it is
    forcing the operator to work on a window they cannot position.
    """

    def test_it_can_shrink_into_a_screen_with_room_to_spare(self, window) -> None:
        # 1080 is the shortest screen this console is plausibly run on, and a
        # desktop panel takes another thirty-odd of it.
        assert window.minimumSizeHint().height() <= 900
