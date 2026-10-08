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
from PyQt5.QtWidgets import QLabel, QScrollArea, QWidget

from litegrip_studio import calibration
from litegrip_studio.calibration import CalibrationInfo
from litegrip_studio.core import commands as cmd
from litegrip_studio.core.calibration_fsm import GuidedPhase, TwoPointPhase
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
#: The factory file this console falls back to, as the console reads it.  The
#: file carries its own 52.69 mm/rad; what is built from it is the scale derived
#: from its 1.651026 rad span over ``DEFAULT_TRAVEL_MM`` plus the probe's inset
#: instead, which is ``derive_scale(1.651026, 85.0)`` — the two differ by 1.2 %,
#: and this side is the one the console moves by.
FACTORY_LIMITS = Limits(0.0, -1.651026, 52.08882234440887, 85.0)

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
INVALID_CAL = CalibrationInfo(
    provenance=calibration.PROVENANCE_INVALID,
    limits=None,
    path="/tmp/broken.json",
    problems=(
        "行程 (travel_range_rad) 必须为正",
        "由行程 0.000000 rad 与设定行程 120.0 mm 推出的 mm/rad 无效；请检查标定页上的行程设定",
    ),
)
#: A good calibration whose two angles were recorded the other way round.  It is
#: the case the page must not make anything of: the file is usable, the banner
#: carries the stroke, and no word about a mounting appears anywhere.
FLIPPED_LIMITS = Limits(-0.300793, 1.421569, 49.93, 85.0)
FLIPPED_CAL = CalibrationInfo(
    provenance=calibration.PROVENANCE_USER,
    limits=FLIPPED_LIMITS,
    path=str(Path.home() / ".litegrip" / "litegrip_calibration.json"),
)
MEMORY_CAL = CalibrationInfo(
    provenance=calibration.PROVENANCE_MEMORY,
    limits=USER_LIMITS,
    path=None,
    warnings=("结果尚未保存",),
)
#: A probe result whose numbers did not survive validation.  ``in_memory``
#: drops the limits when a hard check fails, so this is what a result that
#: cannot be saved actually looks like — not merely one that is unsaved.
BROKEN_MEMORY_CAL = CalibrationInfo(
    provenance=calibration.PROVENANCE_MEMORY,
    limits=None,
    path=None,
    problems=("由行程 0.003824 rad 与设定行程 85.0 mm 推出的 mm/rad 超出合理范围",),
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

    def test_the_factory_file_is_shown_like_any_other_source(self, page) -> None:
        """It is the console's default, not a fallback to be noticed and
        accepted: what would make it wrong for this gripper is the live reading,
        and the gate says so in its own words rather than this banner."""
        page.set_calibration(FACTORY_CAL)

        assert page._banner.severity == "info"
        assert calibration.PROVENANCE_LABELS[calibration.PROVENANCE_FACTORY] in (
            page._banner.headline
        )

    def test_an_unusable_calibration_is_an_error_naming_the_problem(self, page) -> None:
        page.set_calibration(INVALID_CAL)

        assert page._banner.severity == "error"
        assert "行程" in page._banner.detail

    def test_a_calibration_recorded_the_other_way_is_shown_without_comment(self, page) -> None:
        """Not an error, and not a mounting question either: the file is usable
        and self-consistent, and the direction is read from its two angles."""
        page.set_calibration(FLIPPED_CAL)

        assert page._banner.severity == "info"
        assert page._banner.headline == "用户标定：行程 85.0 mm"
        assert "装配" not in page._banner.detail

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


class TestTheProbeButtons:
    def test_a_click_is_answered_before_the_motor_is_enabled(self, page) -> None:
        """The reported bug: 开始手动标定 did nothing on the first press.

        The button was disabled on the motor, and Qt drops a click on a disabled
        button without a word — so the operator, who had just powered the bench
        up and had not enabled it yet, pressed again.  The page asks the worker
        instead, and the worker's refusal names the reason, which is the answer
        they were missing."""
        page.set_calibration(INVALID_CAL)
        page.set_gate(GateState.BLOCKED, "缺少标定文件")
        page.update_frame(frame(enabled=False))

        assert page._guided_start.isEnabled()
        assert page._manual_start.isEnabled()

        page._manual_start.click()

        assert isinstance(page.recorder.last(), cmd.StartManualCalibration)

    def test_the_button_reads_the_same_as_the_log_line_it_writes(self, page) -> None:
        """A probe writes its own name into the log when it starts, and the log
        is read afterwards to find out which of the two was run.  A second name
        for the same probe is a name nobody thinks to search for."""
        assert page._guided_start.text() in cmd.StartGuidedCalibration().describe()

    def test_the_start_buttons_say_the_motor_has_to_be_enabled(self, page) -> None:
        """A click that is answered is still a click the operator could have
        been spared, and the probe is started by someone who is looking at the
        gripper rather than at the window."""
        assert "使能" in page._manual_start.toolTip()
        assert "使能" in page._guided_start.toolTip()

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
        page.set_progress(TwoPointPhase.RECORD_OPEN.value, 0.0)

        assert not page._guided_start.isEnabled()
        assert page._manual_open.isEnabled()
        # Including the guided probe's own controls: a cancel button that is
        # live during the *other* probe cancels nothing, and the operator finds
        # that out at the moment they most need it to work.
        assert not page._guided_cancel.isEnabled()
        assert not page._guided_confirm.isEnabled()

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


class TestTheDirectionIsNotAskedFor:
    """The guided probe takes no mounting declaration, and the page asks for none.

    A declaration is a second opinion about what the probe is on its way to
    measure, and a wrong one is caught nowhere downstream: it records the closed
    stop as the open one and produces a file that validates, saves and runs the
    gripper inverted.  The direction is read from the two angles that were
    recorded, and a unit that behaves differently is what manual two-point
    calibration is for.
    """

    def test_the_command_carries_nothing_else(self, page) -> None:
        page.recorder.commands.clear()

        page._guided_start.click()

        assert page.recorder.of(cmd.StartGuidedCalibration)[-1] == (
            cmd.StartGuidedCalibration()
        )

    def test_the_page_has_no_such_control(self, page) -> None:
        assert not hasattr(page, "reversed_mount")
        assert not hasattr(page, "_guided_reversed")

    def test_no_widget_on_the_page_mentions_a_mounting(self, page) -> None:
        """Read off the widgets rather than the source: a wording that crept back
        into a label or a tooltip is what the operator would actually see."""

        def said(widget: QWidget) -> list[str]:
            found = [widget.toolTip()]
            for name in ("text", "title"):
                value = getattr(widget, name, None)
                if callable(value):
                    found.append(value())
                elif isinstance(value, str):
                    found.append(value)
            return found

        texts = [text for widget in page.findChildren(QWidget) for text in said(widget)]

        assert not [text for text in texts if "装配" in text or "反向" in text]


class TestTheCommandsItSends:
    def test_each_probe_button_sends_its_command(self, page) -> None:
        page._guided_start.click()
        assert isinstance(page.recorder.last(), cmd.StartGuidedCalibration)

        page.set_progress(GuidedPhase.OPEN_PROBE.value, 0.2)
        page._guided_cancel.click()
        assert isinstance(page.recorder.last(), cmd.CancelCalibration)

    def test_starting_the_manual_probe_carries_nothing_else(self, page) -> None:
        """There is no duration to send any more: the operator decides when each
        point is recorded, so the probe has nothing to be told in advance."""
        page.recorder.commands.clear()

        page._manual_start.click()

        assert page.recorder.of(cmd.StartManualCalibration)[-1] == (
            cmd.StartManualCalibration()
        )

    def test_each_record_button_sends_its_own_label(self, page) -> None:
        """The command carries the label rather than the point being inferred
        from the step the probe is in — the label is the operator's whole answer
        to which end is 0 mm, and the two must not be able to disagree."""
        page.set_progress(TwoPointPhase.RECORD_OPEN.value, 0.0)
        page.recorder.commands.clear()
        page._manual_open.click()
        assert isinstance(page.recorder.last(), cmd.RecordOpenLimit)

        page.set_progress(TwoPointPhase.RECORD_CLOSE.value, 0.5)
        page.recorder.commands.clear()
        page._manual_close.click()
        assert isinstance(page.recorder.last(), cmd.RecordCloseLimit)

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

    def test_there_is_no_travel_widget_to_set(self, page) -> None:
        """The travel was editable here until 2026-09-28, when 10 mm was typed
        into it and the derived scale fell below the plausible band — the console
        then refused every move and the page could not explain why.  A
        measurement of the bench is not a preference, so the control is gone."""
        assert not hasattr(page, "_travel")
        assert not hasattr(page, "travel_mm")


class TestSaving:
    def test_a_user_calibration_can_be_rewritten(self, page) -> None:
        page.set_calibration(USER_CAL)

        assert page._save.isEnabled()

    def test_an_unsaved_result_can_be_saved(self, page) -> None:
        """An in-memory result blocks motion, and that is precisely why the
        button has to stay live: a finished probe writes itself out, so what is
        left for the button is the retry after that write failed — and a retry
        on a result that is merely unsaved is the whole case it exists for."""
        page.set_calibration(MEMORY_CAL)

        assert not MEMORY_CAL.motion_allowed, "still gated, and still savable"
        assert page._save.isEnabled()

    def test_a_broken_memory_result_offers_nothing_to_save(self, page) -> None:
        """A result that failed validation comes back with no limits, and a
        file written from it would only look like a calibration — the next
        launch would load it as this gripper's own and refuse to move."""
        page.set_calibration(BROKEN_MEMORY_CAL)

        assert not page._save.isEnabled()

    def test_the_factory_file_is_not_the_operator_s_to_overwrite(self, page) -> None:
        """Saving the factory numbers into the user path would launder them into
        a user calibration, and the next launch would show them as this
        gripper's own.

        The backend refuses the factory path by name as well, but that guard
        cannot see this case: a factory calibration the SDK fell back to has no
        path of its own, so the target would be the *user* file."""
        page.set_calibration(FACTORY_CAL)

        assert not page._save.isEnabled()

    def test_an_unusable_file_is_not_worth_writing_out(self, page) -> None:
        page.set_calibration(INVALID_CAL)

        assert not page._save.isEnabled()

    def test_the_button_says_it_is_a_retry(self, page) -> None:
        """No probe ever needs it pressed, so it must not read like a step of
        the procedure."""
        assert page._save.text() == "重新保存标定…"

    def test_nothing_is_offered_while_a_probe_runs(self, page) -> None:
        page.set_calibration(USER_CAL)
        page.set_progress(GuidedPhase.OPEN_PROBE.value, 0.1)

        assert not page._save.isEnabled()
        assert not page._reload.isEnabled()
        assert not page._load.isEnabled()


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
            (TwoPointPhase.RECORD_OPEN.value, True),
            (TwoPointPhase.RECORD_CLOSE.value, True),
            (TwoPointPhase.IDLE.value, False),
            (TwoPointPhase.DONE.value, False),
            (TwoPointPhase.FAILED.value, False),
            (TwoPointPhase.CANCELLED.value, False),
        ],
    )
    def test_a_manual_phase_is_active_or_not(self, phase: str, active: bool) -> None:
        assert manual_active(phase) is active

    def test_neither_probe_claims_the_other_probes_phases(self) -> None:
        """The two probes report through one progress signal and spell their
        idle phases the same way, so "is a probe running" is never the question
        a widget is asking — "is *this* probe running" is.  A predicate that
        answered with "not idle in my set" would say yes to every phase of the
        other probe; the phase *name* is the only thing that tells them apart.
        """
        idle = {"IDLE", "DONE", "FAILED", "CANCELLED"}
        for phase in (*GuidedPhase, *TwoPointPhase):
            guided = phase in GuidedPhase
            live = phase.value not in idle
            assert guided_active(phase.value) is (guided and live)
            assert manual_active(phase.value) is (not guided and live)

    def test_the_two_phase_sets_share_the_idle_names(self) -> None:
        """The worker emits one signal for both probes, so "no probe running"
        has to be recognisable whichever one the phase came from."""
        for phase in GuidedPhase:
            assert (phase.value in {p.value for p in TwoPointPhase}) == (
                phase.value in {"IDLE", "DONE", "FAILED", "CANCELLED"}
            )


class TestTheProgress:
    def test_the_bar_follows_the_progress(self, page) -> None:
        page.set_progress(GuidedPhase.OPEN_PROBE.value, 0.4)

        assert page._progress.value() == 40

    def test_an_empty_bar_is_not_shown_before_any_probe_has_run(self, page) -> None:
        """An empty bar is a claim that something is nought per cent done, and
        before a probe runs there is nothing for that claim to be about.  It sat
        across the page as an unlabelled grey slab until a reviewer asked what it
        was — which is the test: nobody should have to ask."""
        assert not page._progress.isVisibleTo(page)
        assert page._phase_label.text(), "the strip still says what state it is in"

    def test_the_bar_comes_back_when_a_probe_runs(self, page) -> None:
        page.set_progress(GuidedPhase.OPEN_PROBE.value, 0.1)

        assert page._progress.isVisibleTo(page)

    def test_a_finished_probe_leaves_its_bar_up(self, page) -> None:
        """100% is worth seeing: it is how the operator knows the probe finished
        rather than stopped."""
        page.set_progress(GuidedPhase.DONE.value, 1.0)

        assert page._progress.isVisibleTo(page)
        assert page._progress.value() == 100

    def test_the_phase_is_named_in_words(self, page) -> None:
        page.set_progress(GuidedPhase.CLOSE_PROBE.value, 0.6)

        assert "闭合" in page._phase_label.text()

    def test_the_note_is_shown(self, page) -> None:
        page.set_progress(TwoPointPhase.RECORD_OPEN.value, 0.3, "剩余 20 s")

        assert page._note.text() == "剩余 20 s"

    def test_the_live_position_is_shown_while_probing(self, page) -> None:
        """A probe is the one time the operator cannot see the slider: it is
        disabled, because there is no valid travel to span yet."""
        page.update_frame(frame(position_mm=12.5, enabled=True))

        assert "12.50" in page._position.text()

    def test_the_angle_is_shown_beside_the_millimetres(self, page) -> None:
        """The millimetres are computed through the calibration under suspicion,
        so they cannot answer the one question the page has to ask the operator
        — which way the jaws open.  The angle can: it is what the encoder
        reports, and it is the same number whichever calibration is loaded."""
        page.update_frame(frame(position_mm=12.5, position_rad=-0.300793, enabled=True))

        text = page._position.text()
        assert "12.50" in text
        assert "-0.3008" in text

    def test_a_frame_without_an_angle_still_reads(self, page) -> None:
        """``position_rad`` is newer than any frame a test may have been built
        from, and a missing reading must not print as a number."""
        page.update_frame(frame(position_mm=None, position_rad=None, enabled=True))

        assert "—" in page._position.text()

    def test_out_of_range_progress_cannot_break_the_bar(self, page) -> None:
        page.set_progress(GuidedPhase.OPEN_PROBE.value, 1.7)

        assert page._progress.value() == 100

    def test_an_unknown_phase_is_still_shown(self, page) -> None:
        page.set_progress("SOMETHING_NEW", 0.5)

        assert "SOMETHING_NEW" in page._phase_label.text()


class TestTheTravelIsNotOnThisPage:
    """The travel used to be a spinbox here.  It is
    :data:`constants.DEFAULT_TRAVEL_MM` now, and the page must not offer a way to
    move it: the console derives every millimetre from it, and one that can be
    told the wrong number reports every reading wrong while looking healthy."""

    def test_the_page_does_not_read_a_stored_travel(self, qapp) -> None:
        """An older console's value must not leak in through the settings."""
        store = _Store(**{"calibration/travel_mm": "140.0"})
        page = CalibrationPage(Recorder(), Settings(store))

        assert not hasattr(page, "travel_mm")

    def test_a_stored_acknowledgement_is_dropped_rather_than_honoured(self, qapp) -> None:
        """The gate it armed is gone, and the file it was about is the console's
        own default now.  Left in the file it would read as a standing decision
        the console is still taking notice of, which is worse than absent."""
        store = _Store(**{KEY_ALLOW_FACTORY: "true"})
        Settings(store)

        assert KEY_ALLOW_FACTORY not in store.data

    def test_the_retired_key_is_dropped_from_the_store(self, qapp) -> None:
        store = _Store(**{"calibration/travel_mm": "10.0"})
        Settings(store)

        assert "calibration/travel_mm" not in store.data


@pytest.fixture
def shown_page(page: CalibrationPage, qapp) -> CalibrationPage:
    """The page on screen.  The size a window is held to is computed from the
    widgets it is showing; one that was never shown is not yet evidence."""
    page.resize(900, 520)
    page.show()
    qapp.processEvents()
    return page


class TestWhatSitsWhere:
    """Which box is where, which on this page is a decision rather than a
    rendering detail.

    The two probes are the page's actions, so they share the top row: one per
    kind of calibration, neither of them the "other" one by position.  What they
    are there to produce — the calibration in force and the file it came from —
    is the row underneath them, which is where the eye already is when a probe
    finishes.  It used to be one column of probes on the right and one column of
    results on the left, so a finished probe was read across the page from the
    button that started it.
    """

    @staticmethod
    def _rows(page) -> list:
        """The page's content rows, top to bottom, one layout per row."""
        inner = page._banner.parentWidget().layout()
        return [
            inner.itemAt(index).layout()
            for index in range(inner.count())
            if inner.itemAt(index).layout() is not None
        ]

    def test_the_two_probes_share_the_top_row(self, page) -> None:
        probes = self._rows(page)[0]

        assert probes.indexOf(page._guided_start.parentWidget()) >= 0
        assert probes.indexOf(page._manual_start.parentWidget()) >= 0

    def test_the_automatic_probe_is_the_left_of_the_two(self, page) -> None:
        """It is the one an operator runs first on a gripper nobody has
        calibrated yet, and the left of a row is where that belongs."""
        probes = self._rows(page)[0]

        assert probes.indexOf(page._guided_start.parentWidget()) < probes.indexOf(
            page._manual_start.parentWidget()
        )

    def test_the_calibration_in_force_is_the_row_under_the_probes(self, page) -> None:
        """The result is read after pressing one of the two buttons, so it goes
        below them rather than beside them."""
        probes, current = self._rows(page)[:2]

        assert current.indexOf(page._table_box) >= 0
        assert current.indexOf(page._file_label.parentWidget()) >= 0
        assert probes.indexOf(page._guided_start.parentWidget()) >= 0


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
