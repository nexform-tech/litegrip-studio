"""The status page: the numbers, and the two things it must not claim.

Nothing here is interactive, so the tests are about honesty rather than about
commands: a motor that is not being addressed has no fault information and no
link health, and a page that rendered both as "fine" would be a page that reads
green while the gripper is unplugged.
"""

from __future__ import annotations

import dataclasses

import pytest

from litegrip_studio import constants
from litegrip_studio.core.worker import (
    CONN_CONNECTED,
    CONN_DISCONNECTED,
    CONN_ERROR,
    GateState,
)
from litegrip_studio.telemetry import EMPTY_FRAME, TelemetryFrame
from litegrip_studio.ui import theme
from litegrip_studio.ui.status_page import StatusPage

READY_REASON = "用户标定：/tmp/cal.json"


def frame(**changes) -> TelemetryFrame:
    return dataclasses.replace(EMPTY_FRAME, **changes)


@pytest.fixture
def page(qapp) -> StatusPage:
    return StatusPage()


class TestTheReadings:
    def test_every_field_of_the_frame_has_somewhere_to_go(self, page) -> None:
        """A field added to the frame but forgotten here is a reading the
        operator cannot see; the page names all of them."""
        rendered = set(page.value)
        covered = {
            "位置", "速度", "夹持力", "力矩", "速度参考", "误差",
            "MOS 温度", "线圈温度", "运动状态", "已抓取", "状态帧", "帧速率",
            "链路静默", "tick 耗时", "tick 超时", "错误码",
        }

        assert rendered == covered

    def test_a_frame_is_rendered_into_the_labels(self, page) -> None:
        page.update_frame(
            frame(
                position_mm=62.35,
                velocity_mm_s=-12.5,
                force_n=3.25,
                torque_nm=0.325,
                temperature_mos=31,
                temperature_coil=30,
                rx_frames=1234,
                rx_hz=248.7,
                stale_ms=4.0,
                cycle_ms=0.45,
                overruns=2,
                motion_state="SERVO",
                enabled=True,
                error_code=constants.ERROR_ENABLED,
            )
        )

        values = page.value
        assert values["位置"] == "62.35 mm"
        assert values["速度"] == "-12.5 mm/s"
        assert values["夹持力"] == "+3.25 N"
        assert values["力矩"] == "+0.325 Nm"
        assert values["MOS 温度"] == "31 °C"
        assert values["状态帧"] == "1234"
        assert values["帧速率"] == "249 Hz"
        assert values["tick 耗时"] == "0.45 ms"
        assert values["tick 超时"] == "2"
        assert values["运动状态"] == "SERVO"

    def test_an_unknown_error_figure_is_a_dash_not_a_zero(self, page) -> None:
        """``err_mm is None`` means the axis is not running a profile, which is
        not the same as being exactly on target."""
        page.update_frame(frame(err_mm=None))

        assert page.value["误差"] == "—"

    def test_zero_error_is_still_shown_as_a_number(self, page) -> None:
        page.update_frame(frame(err_mm=0.0))

        assert page.value["误差"] == "+0.00 mm"

    def test_an_unmeasured_position_is_a_dash_not_a_zero(self, page) -> None:
        """Before the first status frame there is no position, and 0.00 mm is
        the closed stop — a reading of a place the jaws are not."""
        page.update_frame(frame(position_mm=None))

        assert page.value["位置"] == "—"

    def test_a_measured_zero_is_still_shown_as_a_number(self, page) -> None:
        """The other side of it: jaws that really are shut read 0.00 mm."""
        page.update_frame(frame(position_mm=0.0))

        assert page.value["位置"] == "0.00 mm"


class TestWhatIsColoured:
    def test_a_hot_mos_is_flagged(self, page) -> None:
        page.update_frame(frame(temperature_mos=constants.TEMP_MOS_WARN_C))

        assert theme.ERROR in page._values["MOS 温度"].styleSheet()

    def test_a_warm_mos_is_flagged_softer(self, page) -> None:
        page.update_frame(frame(temperature_mos=constants.TEMP_MOS_WARN_C - 1))

        assert theme.WARN in page._values["MOS 温度"].styleSheet()

    def test_a_cool_mos_is_plain(self, page) -> None:
        page.update_frame(frame(temperature_mos=25))

        assert theme.WARN not in page._values["MOS 温度"].styleSheet()
        assert theme.ERROR not in page._values["MOS 温度"].styleSheet()

    def test_the_coil_has_its_own_threshold(self, page) -> None:
        """They are different sensors with different limits; one threshold for
        both would flag the wrong one."""
        page.update_frame(
            frame(temperature_coil=constants.TEMP_MOS_WARN_C + 1)
        )

        assert theme.ERROR not in page._values["线圈温度"].styleSheet()

    def test_a_stale_link_is_flagged_on_the_number_itself(self, page) -> None:
        page.update_frame(frame(stale_ms=constants.LINK_STALE_MS, enabled=True))

        assert theme.ERROR in page._values["链路静默"].styleSheet()

    def test_a_fault_colours_the_code(self, page) -> None:
        page.update_frame(frame(error_code=constants.ERROR_OC, enabled=True))

        assert theme.ERROR in page._values["错误码"].styleSheet()
        assert page.value["错误码"] == "0xA"


class TestTheLinkDot:
    def test_a_live_link_reports_its_rate(self, page) -> None:
        page.update_frame(frame(enabled=True, stale_ms=4.0, rx_hz=250.0))

        assert "250" in page.link_text

    def test_silence_while_disabled_is_not_a_dropped_link(self, page) -> None:
        """A console that has not addressed the motor receives nothing; calling
        that a fault would be a permanent false alarm on a good setup."""
        page.update_frame(frame(enabled=False, stale_ms=9999.0, rx_hz=0.0))

        assert "未使能" in page.link_text
        assert "超时" not in page.link_text

    def test_a_quiet_link_while_addressed_is_a_fault(self, page) -> None:
        page.update_frame(
            frame(enabled=True, stale_ms=constants.LINK_STALE_MS, rx_hz=0.0)
        )

        assert "超时" in page.link_text


class TestTheDotsOutliveTheirEvidence:
    """A frame is evidence, and evidence outlives its subject.

    The last frame received goes on saying "250 Hz" and "无故障" forever, so a
    page driven by frames alone reads green over a cable that has been pulled —
    in the same window as a bar that says 未连接.  The connection state is the
    one thing here that needs no frame to be true.
    """

    def test_a_dropped_link_takes_the_green_dot_with_it(self, page) -> None:
        page.update_frame(frame(enabled=True, stale_ms=4.0, rx_hz=250.0))

        page.set_conn_state(CONN_DISCONNECTED, "已断开")

        assert page.link_text == "未连接"

    def test_the_fault_dot_stops_claiming_health_too(self, page) -> None:
        """The verdict goes back to being unknown, in the same words the
        disabled case already uses for it: ``error_code`` 0 means "no fault"
        only while there are status frames to read it out of."""
        page.update_frame(frame(enabled=True, error_code=constants.ERROR_ENABLED))
        assert page.fault_text == "无故障"

        page.set_conn_state(CONN_DISCONNECTED, "已断开")

        assert page.fault_text == "未连接（无故障信息）"

    def test_a_failed_connection_says_why(self, page) -> None:
        page.set_conn_state(CONN_ERROR, "can0 不存在")

        assert "连接失败" in page.link_text
        assert "can0 不存在" in page.link_text

    def test_a_new_frame_puts_the_dots_back(self, page) -> None:
        """Reconnecting is nothing this method has to undo: the frames that
        follow it are what restores the dot, which is why connecting is a
        no-op here rather than a second source of truth."""
        page.set_conn_state(CONN_DISCONNECTED, "已断开")
        page.set_conn_state(CONN_CONNECTED, "已连接")

        page.update_frame(frame(enabled=True, stale_ms=4.0, rx_hz=250.0))

        assert "250" in page.link_text
        assert page.fault_text == "无故障"

    def test_connecting_does_not_erase_a_live_reading(self, page) -> None:
        """The worker says 已连接 before the first frame of the new session has
        arrived, and the reading on screen came from a real one."""
        page.update_frame(frame(enabled=True, stale_ms=4.0, rx_hz=250.0))

        page.set_conn_state(CONN_CONNECTED, "已连接")

        assert "250" in page.link_text

    def test_a_fault_that_already_happened_is_still_reported(self, page) -> None:
        """The banner reports a fault that did happen; the dot claims the
        present.  Under-reporting trouble is the harmless direction, so only
        the dot is cleared."""
        page.update_frame(frame(enabled=True, error_code=constants.ERROR_UV))

        page.set_conn_state(CONN_DISCONNECTED, "已断开")

        assert page.fault_alert[0] == "error"
        assert "欠压" in page.fault_alert[1]


class TestTheFaultDot:
    def test_a_disabled_motor_reports_nothing_rather_than_no_fault(self, page) -> None:
        """``error_code`` 0 on a disabled DM4310 means "disabled", not "well".
        The console is not receiving status frames in that state, so it has no
        grounds for the reassuring reading."""
        page.update_frame(frame(enabled=False, error_code=constants.ERROR_DISABLED))

        assert "未使能" in page.fault_text
        assert page.fault_alert[0] is None

    def test_a_benign_code_on_a_live_motor_is_fine(self, page) -> None:
        page.update_frame(frame(enabled=True, error_code=constants.ERROR_ENABLED))

        assert page.fault_text == "无故障"
        assert page.fault_alert[0] is None

    def test_a_fault_names_itself_and_what_to_do(self, page) -> None:
        page.update_frame(frame(enabled=True, error_code=constants.ERROR_UV))

        severity, headline, detail = page.fault_alert
        assert severity == "error"
        assert "欠压" in headline
        assert detail == constants.FAULT_HINTS[constants.ERROR_UV]

    def test_recovering_clears_the_banner(self, page) -> None:
        page.update_frame(frame(enabled=True, error_code=constants.ERROR_UV))
        page.update_frame(frame(enabled=True, error_code=constants.ERROR_ENABLED))

        assert page.fault_alert[0] is None

    def test_a_fault_with_no_hint_still_shows_the_fault(self, page) -> None:
        """The banner is worth showing without advice; hiding it because the
        hint table has no entry would hide the fault."""
        page.update_frame(frame(enabled=True, error_code=0x7F))

        assert page.fault_alert[0] == "error"
        assert page.fault_alert[1]


class TestTheGate:
    def test_a_ready_gate_says_nothing(self, page) -> None:
        page.set_gate(GateState.READY, READY_REASON)

        assert page.gate_alert[0] is None

    def test_a_blocked_gate_names_the_reason(self, page) -> None:
        page.set_gate(GateState.BLOCKED, "缺少标定文件")

        severity, headline, detail = page.gate_alert
        assert severity == "error"
        assert "阻断" in headline
        assert detail == "缺少标定文件"

    def test_the_factory_gate_is_a_warning_rather_than_an_error(self, page) -> None:
        """It is overridable and the operator has to acknowledge it; painting it
        the same red as a hard block would spend the colour that means "stop"."""
        page.set_gate(GateState.FACTORY, "正在使用出厂标定")

        assert page.gate_alert[0] == "warn"

    def test_opening_the_gate_clears_the_banner(self, page) -> None:
        page.set_gate(GateState.BLOCKED, "缺少标定文件")
        page.set_gate(GateState.READY, READY_REASON)

        assert page.gate_alert[0] is None

    def test_the_gate_is_not_disturbed_by_frames(self, page) -> None:
        """The gate changes when the calibration does, not fifty times a
        second."""
        page.set_gate(GateState.BLOCKED, "缺少标定文件")
        for _ in range(5):
            page.update_frame(frame(enabled=True, error_code=constants.ERROR_ENABLED))

        assert page.gate_alert[2] == "缺少标定文件"
