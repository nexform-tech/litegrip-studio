"""The calibration page: provenance, the two probes, and what it refuses.

The page exists because the SDK resolves two calibration files silently and
returns ``True`` either way.  Most of these tests are therefore about what the
operator is *told*, and the rest are about the probe buttons being offered
exactly when a probe could actually run — the page is the one place in the
console that has to stay usable while the motion gate is shut, since a shut gate
is what a console with no calibration looks like.
"""

from __future__ import annotations

import dataclasses
from pathlib import Path

import pytest
from PyQt5.QtWidgets import QLabel, QScrollArea

from litegrip_studio import calibration, constants
from litegrip_studio.calibration import CalibrationInfo
from litegrip_studio.core import commands as cmd
from litegrip_studio.core.calibration_fsm import GuidedPhase, ManualPhase
from litegrip_studio.core.worker import (
    CONN_CONNECTED,
    CONN_DISCONNECTED,
    GateState,
)
from litegrip_studio.settings import KEY_ALLOW_FACTORY, Settings
from litegrip_studio.telemetry import EMPTY_FRAME, TelemetryFrame
from litegrip_studio.ui.calibration_page import (
    CalibrationPage,
    guided_active,
    manual_active,
)
from litegrip_studio.units import Limits

USER_LIMITS = Limits(1.775959, -0.064279, 65.21, 120.0)
FACTORY_LIMITS = Limits(0.114, -1.491, 74.8, 120.0)

#: A stand-in for where the SDK was checked out.  The shortened form of a
#: factory path is relative to that root, so the tests that assert on it
#: point ``calibration.sdk_root`` here: they then say the same thing on a
#: machine with the SDK installed and on one without it.
SDK_ROOT = Path("/opt/litegrip")

USER_CAL = CalibrationInfo(
    provenance=calibration.PROVENANCE_USER,
    limits=USER_LIMITS,
    path=str(Path.home() / ".litegrip" / "litegrip_calibration.json"),
)
FACTORY_CAL = CalibrationInfo(
    provenance=calibration.PROVENANCE_FACTORY,
    limits=FACTORY_LIMITS,
    path=str(SDK_ROOT / "litegrip" / "factory_calibration.json"),
)
REVERSED_CAL = CalibrationInfo(
    provenance=calibration.PROVENANCE_INVALID,
    limits=None,
    path="/tmp/broken.json",
    problems=("闭合角 0.000000 rad 不大于张开角 1.140000 rad：方向相反，疑似未标定",),
)
MEMORY_CAL = CalibrationInfo(
    provenance=calibration.PROVENANCE_MEMORY,
    limits=USER_LIMITS,
    path=None,
    warnings=("结果尚未保存",),
)


def frame(**changes) -> TelemetryFrame:
    return dataclasses.replace(EMPTY_FRAME, **changes)


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
def page(qapp) -> CalibrationPage:
    recorder = Recorder()
    widget = CalibrationPage(recorder)
    widget.recorder = recorder  # type: ignore[attr-defined]
    widget.set_conn_state(CONN_CONNECTED)
    widget.update_frame(frame(enabled=True))
    return widget


class TestTheProvenanceIsShown:
    def test_a_user_calibration_is_stated_plainly(self, page) -> None:
        page.set_calibration(USER_CAL)

        assert page._banner.severity == "info"
        assert calibration.PROVENANCE_LABELS[calibration.PROVENANCE_USER] in (
            page._banner.headline
        )

    def test_the_factory_file_is_a_warning_and_offers_the_acknowledgement(self, page) -> None:
        """The silent fallback is the failure this page exists for: the numbers
        look fine and belong to a different gripper."""
        page.set_calibration(FACTORY_CAL)

        assert page._banner.severity == "warn"
        assert page._allow.isVisibleTo(page)

    def test_a_reversed_calibration_is_an_error_naming_the_problem(self, page) -> None:
        page.set_calibration(REVERSED_CAL)

        assert page._banner.severity == "error"
        assert "方向相反" in page._banner.detail

    def test_the_acknowledgement_is_hidden_when_it_is_moot(self, page) -> None:
        page.set_calibration(FACTORY_CAL)
        page.set_calibration(USER_CAL)

        assert not page._allow.isVisibleTo(page)

    def test_the_table_carries_the_numbers_the_decision_rests_on(self, page) -> None:
        page.set_calibration(USER_CAL)

        values = [page._table.itemAt(i).widget().text()
                  for i in range(page._table.count())
                  if page._table.itemAt(i).widget() is not None]
        assert any("1.775959" in text for text in values)
        assert any("65.21" in text for text in values)
        assert any("120.00 mm" in text for text in values)

    def test_the_banner_is_a_sentence_and_not_a_row_of_radians(self, page) -> None:
        """It is read while deciding whether to move the gripper, so it carries
        the stroke and the provenance and nothing else."""
        page.set_calibration(USER_CAL)

        assert page._banner.headline == "用户标定：行程 120.0 mm"

    def test_the_expert_numbers_are_not_on_screen_by_default(self, page) -> None:
        page.set_calibration(USER_CAL)

        assert not page._detail.isVisibleTo(page)

    def test_the_toggle_reveals_them(self, page) -> None:
        page.set_calibration(USER_CAL)

        page._detail_toggle.setChecked(True)

        assert page._detail.isVisibleTo(page)
        texts = [child.text() for child in page._detail.findChildren(QLabel)]
        assert any("1.775959" in text for text in texts)

    def test_the_summary_stays_visible_when_the_detail_is_hidden(self, page) -> None:
        """The two rows that remain are the load-bearing ones, so hiding the
        detail must not hide them."""
        page.set_calibration(USER_CAL)

        assert page._summary_holder.isVisibleTo(page)
        summary = [child.text() for child in page._summary_holder.findChildren(QLabel)]
        assert "用户标定" in summary
        assert "120.0 mm" in summary

    def test_the_file_is_shown_relative_rather_than_absolute(
        self, page, monkeypatch
    ) -> None:
        monkeypatch.setattr(calibration, "sdk_root", lambda: SDK_ROOT)
        page.set_calibration(FACTORY_CAL)

        assert "litegrip/factory_calibration.json" in page._file_label.text()
        assert str(SDK_ROOT) not in page._file_label.text()

    def test_reloading_does_not_leave_the_previous_table_behind(self, page) -> None:
        """Two calibrations on screen at once is the exact confusion this page
        exists to prevent, so the old rows have to leave the widget tree — not
        merely the layout."""
        page.set_calibration(FACTORY_CAL)
        page.set_calibration(USER_CAL)

        texts = [child.text() for child in page._table_box.findChildren(QLabel)]
        assert not any("factory_calibration.json" in text for text in texts)
        assert any("litegrip_calibration.json" in text for text in texts)

    def test_no_calibration_at_all_is_an_error_rather_than_a_blank(self, page) -> None:
        page.set_calibration(None)

        assert page._banner.severity == "error"


class TestTheGateIsExplained:
    def test_a_blocked_gate_says_why(self, page) -> None:
        page.set_gate(GateState.BLOCKED, "缺少标定文件")

        assert "已阻断" in page._gate_label.text()
        assert "缺少标定文件" in page._gate_label.text()

    def test_a_ready_gate_says_so(self, page) -> None:
        page.set_gate(GateState.READY, "用户标定")

        assert "就绪" in page._gate_label.text()

    def test_the_factory_gate_is_a_warning_rather_than_a_block(self, page) -> None:
        page.set_gate(GateState.FACTORY, "正在使用出厂标定")

        assert "待确认" in page._gate_label.text()


class TestTheProbeButtons:
    def test_a_probe_needs_a_connected_enabled_motor(self, page) -> None:
        page.set_calibration(REVERSED_CAL)
        page.set_gate(GateState.BLOCKED, "缺少标定文件")

        assert page._guided_start.isEnabled()
        assert page._manual_start.isEnabled()

        page.update_frame(frame(enabled=False))

        assert not page._guided_start.isEnabled()
        assert not page._manual_start.isEnabled()

    def test_a_disconnected_console_offers_no_probe(self, page) -> None:
        page.set_conn_state(CONN_DISCONNECTED)

        assert not page._guided_start.isEnabled()

    def test_a_busy_worker_offers_no_probe(self, page) -> None:
        """``enable()`` can take ten seconds; a probe submitted during it would
        be refused, and a button that refuses is a button to remove."""
        page.set_busy(True, "正在使能（可能需要数秒）…")

        assert not page._guided_start.isEnabled()
        page.set_busy(False)

        assert page._guided_start.isEnabled()

    def test_once_a_probe_runs_the_other_start_is_withdrawn(self, page) -> None:
        page.set_progress(GuidedPhase.OPEN_PROBE.value, 0.1)

        assert not page._guided_start.isEnabled()
        assert not page._manual_start.isEnabled()
        assert page._guided_cancel.isEnabled()

    def test_the_manual_probe_withdraws_the_guided_one_too(self, page) -> None:
        page.set_progress(ManualPhase.RECORDING.value, 0.3)

        assert not page._guided_start.isEnabled()
        assert page._manual_stop.isEnabled()

    def test_a_finished_probe_offers_a_new_one(self, page) -> None:
        page.set_progress(GuidedPhase.DONE.value, 1.0)

        assert page._guided_start.isEnabled()
        assert not page._guided_cancel.isEnabled()

    def test_a_failed_probe_offers_a_new_one(self, page) -> None:
        page.set_progress(GuidedPhase.FAILED.value, 1.0)

        assert page._guided_start.isEnabled()


class TestTheConfirmation:
    def test_the_button_says_which_limit_is_being_confirmed(self, page) -> None:
        """The SDK waits for the Enter key, which tells the operator nothing
        about what they are agreeing to."""
        page.set_progress(GuidedPhase.OPEN_PROBE.value, 0.2)
        opening = page._guided_confirm.text()

        page.set_progress(GuidedPhase.CLOSE_PROBE.value, 0.6)

        assert "张开" in opening
        assert "闭合" in page._guided_confirm.text()
        assert page._guided_confirm.text() != opening

    def test_it_is_offered_only_while_a_limit_is_being_probed(self, page) -> None:
        page.set_progress(GuidedPhase.OPEN_BACKOFF.value, 0.4)

        assert not page._guided_confirm.isEnabled()

    def test_confirming_sends_the_confirmation(self, page) -> None:
        page.set_progress(GuidedPhase.OPEN_PROBE.value, 0.2)
        page.recorder.commands.clear()

        page._guided_confirm.click()

        assert isinstance(page.recorder.last(), cmd.ConfirmProbeLimit)


class TestTheCommandsItSends:
    def test_each_probe_button_sends_its_command(self, page) -> None:
        page._guided_start.click()
        assert isinstance(page.recorder.last(), cmd.StartGuidedCalibration)

        page.set_progress(GuidedPhase.OPEN_PROBE.value, 0.2)
        page._guided_cancel.click()
        assert isinstance(page.recorder.last(), cmd.CancelCalibration)

    def test_the_manual_probe_carries_the_duration(self, page) -> None:
        page._manual_duration.setValue(45.0)
        page.recorder.commands.clear()

        page._manual_start.click()

        assert page.recorder.of(cmd.StartManualCalibration)[-1].duration_s == 45.0

    def test_stopping_the_recording_keeps_the_samples(self, page) -> None:
        """Distinct from cancelling: the SDK's Ctrl+C equivalent settles and
        validates what it captured, and a worker thread can never receive the
        signal itself."""
        page.set_progress(ManualPhase.RECORDING.value, 0.5)
        page.recorder.commands.clear()

        page._manual_stop.click()

        assert isinstance(page.recorder.last(), cmd.StopManualRecording)

    def test_reloading_and_saving_are_commands(self, page) -> None:
        page.recorder.commands.clear()
        page._reload.click()
        assert isinstance(page.recorder.of(cmd.LoadCalibration)[-1], cmd.LoadCalibration)

    def test_choosing_a_file_loads_that_path(self, page) -> None:
        """The dialog cannot be driven in a test, so the path handling is
        separated from it."""
        page.recorder.commands.clear()

        page._load_file("/tmp/other.json")

        assert page.recorder.of(cmd.LoadCalibration)[-1].path == "/tmp/other.json"

    def test_the_travel_mm_travels_as_a_command(self, page) -> None:
        page._travel.setValue(140.0)
        page.recorder.commands.clear()

        page._travel_apply.click()

        assert page.recorder.of(cmd.SetTravel)[-1].max_stroke_mm == 140.0


class TestSaving:
    def test_a_user_calibration_can_be_rewritten(self, page) -> None:
        page.set_calibration(USER_CAL)

        assert page._save.isEnabled()

    def test_an_unsaved_result_can_be_saved(self, page) -> None:
        page.set_calibration(MEMORY_CAL)

        assert page._save.isEnabled()

    def test_the_factory_file_is_not_the_operator_s_to_overwrite(self, page) -> None:
        """Saving the factory numbers into the user path would launder them into
        a user calibration, and the next launch would show them as this
        gripper's own."""
        page.set_calibration(FACTORY_CAL)

        assert not page._save.isEnabled()

    def test_an_unusable_file_is_not_worth_writing_out(self, page) -> None:
        page.set_calibration(REVERSED_CAL)

        assert not page._save.isEnabled()

    def test_nothing_is_offered_while_a_probe_runs(self, page) -> None:
        page.set_calibration(USER_CAL)
        page.set_progress(GuidedPhase.OPEN_PROBE.value, 0.1)

        assert not page._save.isEnabled()
        assert not page._reload.isEnabled()
        assert not page._load.isEnabled()


class TestTheFactoryAcknowledgement:
    def test_ticking_it_tells_the_worker(self, qapp) -> None:
        seen: list[bool] = []
        page = CalibrationPage(Recorder(), seen.append)
        page.set_calibration(FACTORY_CAL)

        page._allow.setChecked(True)

        assert seen == [True]

    def test_it_is_remembered(self, qapp) -> None:
        store = _Store()
        page = CalibrationPage(Recorder(), None, Settings(store))

        page._allow.setChecked(True)

        assert store.value(KEY_ALLOW_FACTORY) in (True, "true", 1)

    def test_it_is_restored(self, qapp) -> None:
        store = _Store(**{KEY_ALLOW_FACTORY: True})
        page = CalibrationPage(Recorder(), None, Settings(store))

        assert page.allows_factory


class TestThePhaseHelpers:
    @pytest.mark.parametrize(
        "phase, active",
        [
            (GuidedPhase.OPEN_PROBE.value, True),
            (GuidedPhase.OPEN_BACKOFF.value, True),
            (GuidedPhase.CLOSE_PROBE.value, True),
            (GuidedPhase.IDLE.value, False),
            (GuidedPhase.DONE.value, False),
            (GuidedPhase.FAILED.value, False),
            (GuidedPhase.CANCELLED.value, False),
        ],
    )
    def test_a_guided_phase_is_active_or_not(self, phase: str, active: bool) -> None:
        assert guided_active(phase) is active

    @pytest.mark.parametrize(
        "phase, active",
        [
            (ManualPhase.RECORDING.value, True),
            (ManualPhase.SETTLE.value, True),
            (ManualPhase.RECOVER.value, True),
            (ManualPhase.IDLE.value, False),
            (ManualPhase.DONE.value, False),
            (ManualPhase.FAILED.value, False),
            (ManualPhase.CANCELLED.value, False),
        ],
    )
    def test_a_manual_phase_is_active_or_not(self, phase: str, active: bool) -> None:
        assert manual_active(phase) is active

    def test_the_two_phase_sets_share_the_idle_names(self) -> None:
        """The worker emits one signal for both probes, so "no probe running"
        has to be recognisable whichever one the phase came from."""
        for phase in GuidedPhase:
            assert (phase.value in {p.value for p in ManualPhase}) == (
                phase.value in {"IDLE", "DONE", "FAILED", "CANCELLED"}
            )


class TestTheProgress:
    def test_the_bar_follows_the_progress(self, page) -> None:
        page.set_progress(GuidedPhase.OPEN_PROBE.value, 0.4)

        assert page._progress.value() == 40

    def test_the_phase_is_named_in_words(self, page) -> None:
        page.set_progress(GuidedPhase.CLOSE_PROBE.value, 0.6)

        assert "闭合" in page._phase_label.text()

    def test_the_note_is_shown(self, page) -> None:
        page.set_progress(ManualPhase.RECORDING.value, 0.3, "剩余 20 s")

        assert page._note.text() == "剩余 20 s"

    def test_the_live_position_is_shown_while_probing(self, page) -> None:
        """A probe is the one time the operator cannot see the slider: it is
        disabled, because there is no valid travel to span yet."""
        page.update_frame(frame(position_mm=12.5, enabled=True))

        assert "12.50" in page._position.text()

    def test_out_of_range_progress_cannot_break_the_bar(self, page) -> None:
        page.set_progress(GuidedPhase.OPEN_PROBE.value, 1.7)

        assert page._progress.value() == 100

    def test_an_unknown_phase_is_still_shown(self, page) -> None:
        page.set_progress("SOMETHING_NEW", 0.5)

        assert "SOMETHING_NEW" in page._phase_label.text()


class TestTheTravel:
    """The one number on this page that is a measurement rather than a file's.

    It is what the millimetres per rad is derived from and what the slider tops
    out at, so it is editable — but only inside a band, because a mistyped travel
    scales every reading on the console.
    """

    def test_it_defaults_to_the_measured_travel(self, page) -> None:
        assert page.travel_mm == pytest.approx(constants.DEFAULT_TRAVEL_MM)

    def test_it_is_restored_from_the_settings(self, qapp) -> None:
        store = _Store(**{"calibration/travel_mm": "140.0"})
        page = CalibrationPage(Recorder(), None, Settings(store))

        assert page.travel_mm == pytest.approx(140.0)

    def test_it_cannot_be_set_outside_the_plausible_range(self, page) -> None:
        page._travel.setValue(9999.0)

        assert page.travel_mm == pytest.approx(constants.STROKE_MAX_MM)


@pytest.fixture
def shown_page(page: CalibrationPage, qapp) -> CalibrationPage:
    """The page on screen.  The size a window is held to is computed from the
    widgets it is showing; one that was never shown is not yet evidence."""
    page.resize(900, 520)
    page.show()
    qapp.processEvents()
    return page


class TestThePageDoesNotDecideHowBigTheWindowIs:
    """The page is the tallest thing in the window, and Qt will not shrink a
    window below its tallest page's minimum size.

    A page whose height is a function of everything it has to say therefore sets
    the floor for the whole console — and a floor taller than the screen puts the
    window's own title bar past the edge of the display, which is the same as a
    window that cannot be moved.  These pin the two ways this page did that: the
    natural height of a six-box stack, and the detail table raising the minimum
    by four hundred pixels when it was expanded.
    """

    def test_its_own_floor_is_a_rounding_error(self, shown_page) -> None:
        """It holds a lot of content and may not require room for all of it.

        The window spends about four hundred pixels on its own chrome, the log
        and the tab bar before any page is drawn, so a page that asks for two
        hundred still leaves the console inside a 1080-pixel screen.
        """
        assert shown_page.minimumSizeHint().height() <= 200

    def test_the_detail_table_does_not_grow_the_floor_when_it_is_expanded(
        self, shown_page
    ) -> None:
        """The rows are real and worth reading, so they may make the page
        scroll — but not make the window bigger."""
        shown_page.set_calibration(USER_CAL)
        before = shown_page.minimumSizeHint().height()

        shown_page._detail_toggle.setChecked(True)

        assert shown_page.minimumSizeHint().height() == before

    def test_the_running_probe_is_not_something_to_scroll_to(self, page) -> None:
        """The moment this page is watched rather than read is the moment a
        probe is running, so its readout is pinned outside the scrolling part.
        A progress bar below the fold is a probe the operator is running blind.
        """
        scroll = page.findChild(QScrollArea)

        assert scroll is not None, "the page must scroll rather than force the window"
        for readout in (page._phase_label, page._progress, page._position, page._note):
            assert not scroll.isAncestorOf(readout)
