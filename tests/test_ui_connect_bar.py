"""The connection bar: what it says about the link, and what it offers.

The state derivations are pure and tested as such.  The enablement is tested
because a console that offers 断开 while it is disconnected, or 使能 while the
motor is already enabled, is a console that teaches the operator to ignore its
buttons.
"""

from __future__ import annotations

import dataclasses

import pytest

from litegrip_studio import constants
from litegrip_studio.core import commands as cmd
from litegrip_studio.core.worker import (
    CONN_CONNECTED,
    CONN_CONNECTING,
    CONN_DISCONNECTED,
    CONN_ERROR,
)
from litegrip_studio.telemetry import EMPTY_FRAME, TelemetryFrame
from litegrip_studio.ui.connect_bar import (
    CONN_LABELS,
    ConnectBar,
    connection_dot,
    fault_state,
    link_state,
)


def frame(**changes) -> TelemetryFrame:
    return dataclasses.replace(EMPTY_FRAME, **changes)


class Recorder:
    def __init__(self) -> None:
        self.commands: list[object] = []

    def __call__(self, command) -> None:
        self.commands.append(command)

    def last(self):
        assert self.commands, "nothing was submitted"
        return self.commands[-1]


@pytest.fixture
def bar(qapp):
    return ConnectBar(Recorder())


class TestTheLinkState:
    def test_silence_while_not_enabled_is_not_a_fault(self) -> None:
        """A console that has not addressed the motor receives nothing, and
        calling that a dropped link would be a permanent false alarm."""
        assert link_state(frame(enabled=False, stale_ms=9999.0)) == ("idle", "未使能")

    def test_a_live_link_reports_its_rate(self) -> None:
        state, text = link_state(frame(enabled=True, stale_ms=4.0, rx_hz=250.0))

        assert state == "ok"
        assert "250" in text

    def test_a_slow_link_is_a_warning_before_it_is_a_fault(self) -> None:
        state, _text = link_state(frame(enabled=True, stale_ms=150.0, rx_hz=6.0))

        assert state == "warn"

    def test_a_stale_link_is_an_error(self) -> None:
        state, text = link_state(
            frame(enabled=True, stale_ms=constants.LINK_STALE_MS, rx_hz=0.0)
        )

        assert state == "error"
        assert "链路超时" in text


class TestTheConnectionDot:
    """The dot the status page shows once the frames have stopped.

    It says the same words as the bar's own label, because the two are read in
    the same window: a bar reading 未连接 above a dot reading 250 Hz is the
    contradiction this exists to prevent.
    """

    @pytest.mark.parametrize("state", [CONN_DISCONNECTED, CONN_CONNECTING, CONN_ERROR])
    def test_it_says_what_the_bar_says(self, state) -> None:
        dot, text = connection_dot(state)

        assert text == CONN_LABELS[state][0]
        assert dot in ("idle", "warn", "error")

    def test_a_failure_carries_its_reason(self) -> None:
        dot, text = connection_dot(CONN_ERROR, "can0 不存在")

        assert dot == "error"
        assert "can0 不存在" in text

    def test_a_plain_disconnection_is_not_given_a_reason_it_does_not_have(
        self,
    ) -> None:
        """The worker emits 已断开 with it, and "未连接（已断开）" would say the
        same thing twice."""
        _dot, text = connection_dot(CONN_DISCONNECTED, "已断开")

        assert text == "未连接"

    def test_an_unknown_state_is_still_shown(self) -> None:
        """A dot that went blank on a state this build does not know about would
        hide the very thing it is there to report."""
        dot, text = connection_dot("SOMETHING_NEW")

        assert dot == "idle"
        assert text == "SOMETHING_NEW"


class TestTheFaultState:
    def test_the_two_benign_codes_are_not_faults(self) -> None:
        for code in constants.OK_ERROR_CODES:
            assert fault_state(frame(error_code=code))[0] == "ok"

    def test_a_fault_uses_the_sdk_text_and_adds_the_thing_to_do(self) -> None:
        state, text = fault_state(frame(error_code=constants.ERROR_UV))

        assert state == "error"
        assert "欠压" in text
        assert constants.FAULT_HINTS[constants.ERROR_UV] in text

    def test_a_fault_with_no_hint_is_still_named(self) -> None:
        state, text = fault_state(frame(error_code=0x7F))

        assert state == "error"
        assert text


class TestWhatItOffers:
    def test_before_connecting_only_连接_is_offered(self, bar) -> None:
        assert bar._connect.isEnabled()
        assert not bar._disconnect.isEnabled()
        assert not bar._enable.isEnabled()
        assert not bar._disable.isEnabled()
        assert not bar._clear.isEnabled()
        assert not bar._reset.isEnabled()

    def test_while_connecting_the_button_says_so_and_is_not_clickable_again(self, bar) -> None:
        bar.set_conn_state(CONN_CONNECTING)

        assert not bar._connect.isEnabled()
        assert bar._connect.text() == "连接中…"

    def test_connected_but_disabled_offers_enable_and_disconnect(self, bar) -> None:
        bar.set_conn_state(CONN_CONNECTED)

        assert bar._enable.isEnabled()
        assert bar._disconnect.isEnabled()
        assert not bar._disable.isEnabled()
        assert not bar._connect.isEnabled()

    def test_enabled_offers_disable_instead(self, bar) -> None:
        bar.set_conn_state(CONN_CONNECTED)
        bar.update_frame(frame(enabled=True, error_code=constants.ERROR_ENABLED))

        assert bar._disable.isEnabled()
        assert not bar._enable.isEnabled()

    def test_a_fault_offers_clear(self, bar) -> None:
        bar.set_conn_state(CONN_CONNECTED)
        bar.update_frame(frame(error_code=constants.ERROR_OC))

        assert bar._clear.isEnabled()

    def test_the_latched_estop_offers_reset_and_no_enable(self, bar) -> None:
        """A latched E-stop disables the motor, so 使能 would be offered by the
        rule above and refused by the worker.  It must not be offered."""
        bar.set_conn_state(CONN_CONNECTED)
        bar.update_frame(frame(enabled=False, motion_state="ESTOP"))

        assert bar._reset.isEnabled()
        assert not bar._enable.isEnabled()

    def test_a_failed_connection_can_be_retried(self, bar) -> None:
        """The failure the operator most needs to recover from is the one that
        just happened, so a refused connect must not disable the button."""
        bar.set_conn_state(CONN_ERROR)

        assert bar._connect.isEnabled()
        assert not bar._disconnect.isEnabled()


class TestWhatItSends:
    @pytest.mark.parametrize(
        "attribute, expected, conn, changes",
        [
            ("_connect", cmd.Connect, "disconnected", {}),
            ("_disconnect", cmd.Disconnect, CONN_CONNECTED, {}),
            ("_enable", cmd.Enable, CONN_CONNECTED, {}),
            (
                "_disable",
                cmd.Disable,
                CONN_CONNECTED,
                {"enabled": True, "error_code": constants.ERROR_ENABLED},
            ),
            ("_clear", cmd.ClearFault, CONN_CONNECTED, {"error_code": constants.ERROR_UV}),
            ("_reset", cmd.ResetEStop, CONN_CONNECTED, {"motion_state": "ESTOP"}),
        ],
    )
    def test_each_button_sends_its_command(
        self, qapp, attribute: str, expected, conn: str, changes: dict
    ) -> None:
        recorder = Recorder()
        bar = ConnectBar(recorder)
        bar.set_conn_state(conn)
        bar.update_frame(frame(**changes))
        assert getattr(bar, attribute).isEnabled(), "the scenario must offer this button"

        getattr(bar, attribute).click()

        assert isinstance(recorder.last(), expected)

    def test_the_description_is_shown_without_the_connection_state(self, bar) -> None:
        bar.set_backend_description("仿真 (Plant) | 用户标定")
        bar.set_conn_state(CONN_CONNECTED, "已连接")

        assert "仿真" in bar._target.text()
        assert "已连接" in bar._conn_label.text()
