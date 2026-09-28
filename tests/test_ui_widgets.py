"""The theme and the shared widgets.

The formatting rules are tested without Qt — they are the part that decides what
an operator reads, and they are pure.  The widgets themselves are only built and
exercised for the properties that are about *visibility*, which is the one thing
here that is a decision rather than a rendering.
"""

from __future__ import annotations

import pytest

from litegrip_studio.ui import theme
from litegrip_studio.ui.widgets import (
    TARGET_VISIBLE_MM,
    UNKNOWN,
    Banner,
    PositionReadout,
    StatusDot,
    format_readout,
)


class TestTheReadoutText:
    def test_the_actual_position_is_always_shown(self) -> None:
        assert format_readout(62.4, None).actual == "  62.4 mm"
        assert format_readout(0.0, None).actual == "   0.0 mm"

    def test_a_target_with_no_command_hides_the_other_two_lines(self) -> None:
        text = format_readout(62.4, None)

        assert text.target == ""
        assert text.error == ""
        assert not text.moving

    def test_the_target_and_error_appear_once_the_jaws_are_away_from_it(self) -> None:
        text = format_readout(62.4, 71.0)

        assert text.target == "  71.0 mm"
        assert text.error == "  +8.6 mm"
        assert text.moving

    def test_the_error_keeps_its_sign(self) -> None:
        """Which way the jaws have to travel is the whole content of the sign."""
        assert format_readout(71.0, 62.4).error.startswith("  -")

    @pytest.mark.parametrize("offset", [0.0, 0.1, -0.1, TARGET_VISIBLE_MM, -TARGET_VISIBLE_MM])
    def test_a_move_that_has_arrived_settles_to_one_number(self, offset: float) -> None:
        """The readout going quiet IS the arrival signal, so the band has to
        include the arrival tolerance rather than sit inside it."""
        assert not format_readout(60.0, 60.0 + offset).moving

    def test_it_is_not_hysteretic_between_the_two_bands(self) -> None:
        """Just outside the band the target comes back; a readout that flickered
        between showing and hiding while the loop settled would be worse than
        either choice."""
        assert format_readout(60.0, 60.0 + TARGET_VISIBLE_MM + 0.01).moving

    def test_a_position_nobody_measured_is_not_drawn_as_a_number(self) -> None:
        """Before the first status frame the position is unknown.  ``0.0 mm``
        would be the closed stop, which is a place, and the wrong one."""
        text = format_readout(None, None)
        assert text.actual == UNKNOWN
        assert text.actual != "   0.0 mm"

    def test_without_a_measurement_there_is_no_error_to_show(self) -> None:
        """A target can only be judged against a measured position, so asking
        for one is not a move that can be reported on."""
        text = format_readout(None, 71.0)
        assert (text.target, text.error) == ("", "")
        assert not text.moving


class TestTheReadoutWidget:
    def test_the_target_lines_are_hidden_when_the_move_has_arrived(self, qapp) -> None:
        readout = PositionReadout()
        readout.update_position(60.0, 60.1)

        assert not readout.target.isVisibleTo(readout)
        assert not readout.error.isVisibleTo(readout)
        assert readout.actual.isVisibleTo(readout)

    def test_they_come_back_during_a_move(self, qapp) -> None:
        readout = PositionReadout()
        readout.update_position(60.0, 90.0)

        assert readout.target.isVisibleTo(readout)
        assert readout.error.isVisibleTo(readout)

    def test_a_grasp_shows_only_where_the_jaws_are(self, qapp) -> None:
        """Holding an object at 28 mm with the grasp targeting 0 mm: the target
        and the error are the object's width, and read as a move that has gone
        badly wrong rather than as a grasp that is working."""
        readout = PositionReadout()
        readout.update_position(28.0, 0.0, grasping=True)

        assert readout.actual.isVisibleTo(readout)
        assert not readout.target.isVisibleTo(readout)
        assert not readout.error.isVisibleTo(readout)

    def test_letting_go_puts_the_two_lines_back(self, qapp) -> None:
        """The state is the frame's, not a mode the readout latches: the first
        frame after the grasp ends is a normal move again."""
        readout = PositionReadout()
        readout.update_position(28.0, 0.0, grasping=True)

        readout.update_position(28.0, 38.0)

        assert readout.target.isVisibleTo(readout)
        assert readout.error.isVisibleTo(readout)


class TestTheBanner:
    def test_a_severity_colours_the_edge(self, qapp) -> None:
        banner = Banner()
        banner.set("error", "标定已阻断")

        assert "标定已阻断" in banner.title
        assert theme.ERROR in banner.styleSheet()

    def test_none_hides_it(self, qapp) -> None:
        banner = Banner()
        banner.set("warn", "注意")
        banner.set(None, "")

        assert not banner.isVisible()
        assert banner.title == ""

    def test_a_detail_line_is_optional(self, qapp) -> None:
        banner = Banner()
        banner.set("info", "标题")

        assert "标题" in banner.title
        banner.set("info", "标题", "细节")
        assert "细节" in banner.title

    def test_an_action_can_be_put_inside_it(self, qapp) -> None:
        from PyQt5.QtWidgets import QPushButton

        button = QPushButton("确认")
        banner = Banner()
        banner.add_action(button)

        assert button.parent() is banner._action


class TestTheTheme:
    def test_every_level_gets_a_colour(self) -> None:
        for level in ("info", "debug", "warn", "warning", "error", "fatal"):
            assert theme.level_colour(level).startswith("#")

    def test_an_unknown_level_is_muted_rather_than_alarming(self) -> None:
        """Something arriving that this module has not been taught about is not
        the operator's emergency."""
        assert theme.level_colour("wat") == theme.TEXT_MUTED

    def test_the_stylesheet_covers_the_panels_and_is_applied(self, qapp) -> None:
        theme.apply(qapp)
        applied = qapp.styleSheet()

        assert theme.PANEL in applied
        assert "QGroupBox" in applied
        assert "QTabBar::tab" in applied

    def test_the_slider_and_the_plot_share_their_colours(self) -> None:
        """One green means "the jaws are here", in every widget that shows it."""
        assert theme.ACTUAL == theme.OK


class TestTheStatusDot:
    def test_it_carries_a_colour_and_a_word(self, qapp) -> None:
        dot = StatusDot("ok", "已连接")
        assert theme.OK in dot.text()
        assert "已连接" in dot.text()

        dot.set("error", "链路断开")
        assert theme.ERROR in dot.text()
        assert "链路断开" in dot.toolTip()

    def test_an_unknown_state_reads_as_inactive(self, qapp) -> None:
        dot = StatusDot("ok", "x")
        dot.set("something-new")

        assert theme.DISABLED in dot.text()
        assert "x" in dot.text(), "the label is kept when only the state changes"
