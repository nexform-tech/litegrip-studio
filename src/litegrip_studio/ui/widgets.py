"""Small shared widgets: readouts, the banner, and a status dot.

The formatting rules live in module-level functions rather than in the widgets,
so what an operator reads can be tested without instantiating Qt at all.  That
matters for the readout in particular: "is the target line shown?" is a policy
decision about when a number is worth showing, not a painting concern.
"""

from __future__ import annotations

from dataclasses import dataclass

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from . import theme

#: How far the jaws may be from the target before the console starts showing
#: that they are on their way there.
#:
#: Larger than :data:`~litegrip_studio.constants.TOL_MM` on purpose: the arrival
#: band is what the motion planner calls "arrived", and a readout that hid the
#: target at exactly 0.4 mm would flicker between showing and hiding while the
#: loop settles into it.  The gap between the two numbers is the hysteresis.
TARGET_VISIBLE_MM = 0.5

#: What a position readout says when there is no measurement behind it.  Not a
#: zero: zero is the closed stop, and a number nobody has measured must not read
#: as a place the jaws are.
UNKNOWN = "—"


@dataclass(frozen=True)
class ReadoutText:
    """The three lines beside the slider, already formatted."""

    actual: str
    target: str
    error: str

    @property
    def moving(self) -> bool:
        return bool(self.target)


def format_readout(actual_mm: float | None, target_mm: float | None) -> ReadoutText:
    """Decide what the position readout says.

    The target and the error are hidden once the jaws are close enough: a line
    reading "目标 62.1 mm / 误差 +0.0 mm" beside an actual of 62.1 mm is noise,
    and it is noise exactly when the operator is watching for the move to
    finish.  Hidden, the readout settles to a single number, which is the
    signal that the move is over.

    ``actual_mm`` of ``None`` means no status frame has arrived, so the position
    is unknown — there is nothing to compare a target against, and the readout
    says so instead of printing a millimetre figure for one.
    """
    if actual_mm is None:
        return ReadoutText(UNKNOWN, "", "")
    actual = f"{actual_mm:6.1f} mm"
    if target_mm is None:
        return ReadoutText(actual, "", "")
    error = target_mm - actual_mm
    if abs(error) <= TARGET_VISIBLE_MM:
        return ReadoutText(actual, "", "")
    return ReadoutText(actual, f"{target_mm:6.1f} mm", f"{error:+6.1f} mm")


class Readout(QLabel):
    """A number with a caption above it, in a font that does not twitch."""

    def __init__(self, caption: str, value: str = UNKNOWN, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._caption = caption
        self.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.setMinimumWidth(96)
        self.set_value(value)
        self.setToolTip(caption)

    def set_value(self, value: str, colour: str = theme.TEXT) -> None:
        # Rich text rather than two widgets: the caption and the value have to
        # stay on one baseline and inside one width, and the readout column is
        # what keeps the slider from resizing as the numbers change.
        self.setText(
            f'<span style="color:{theme.TEXT_MUTED};font-size:11px">{self._caption}</span>'
            f'<br><span style="color:{colour};font-family:{theme.MONO_FAMILY}">{value}</span>'
        )

    def set_caption(self, caption: str) -> None:
        self._caption = caption


class PositionReadout(QWidget):
    """The block to the right of the slider: actual, target, error."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.actual = Readout("实际位置", "—")
        self.target = Readout("目标", "")
        self.error = Readout("误差", "")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        for widget in (self.actual, self.target, self.error):
            layout.addWidget(widget)
        layout.addStretch(1)

        # Fixed width: the slider is the thing that takes the space, and a
        # readout that widened as the numbers grew would move the slider under
        # the operator's hand.
        self.setFixedWidth(112)

    def update_position(self, actual_mm: float, target_mm: float | None = None) -> None:
        text = format_readout(actual_mm, target_mm)
        self.actual.set_value(text.actual, theme.ACTUAL)
        self.target.set_value(text.target, theme.TARGET)
        self.error.set_value(text.error, theme.WARN if text.moving else theme.TEXT_MUTED)
        self.target.setVisible(bool(text.target))
        self.error.setVisible(bool(text.error))


class Banner(QFrame):
    """A strip that stays put, for a fact the operator has to keep seeing.

    Used for the calibration source.  A banner rather than a dialog because the
    fact does not go away: a console running on the factory file says so for as
    long as it is running, and one that said it once in a modal that was clicked
    away would be worse than silent.
    """

    SEVERITY_COLOURS = {
        "ok": theme.OK,
        "info": theme.TARGET,
        "warn": theme.WARN,
        "error": theme.ERROR,
    }

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setFrameShape(QFrame.NoFrame)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)

        self._severity: str | None = None
        self._headline = ""
        self._detail = ""
        self._title = QLabel()
        self._title.setTextFormat(Qt.RichText)
        self._title.setWordWrap(True)

        self._action = QWidget()
        action_layout = QHBoxLayout(self._action)
        action_layout.setContentsMargins(0, 0, 0, 0)
        action_layout.setSpacing(6)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(10)
        layout.addWidget(self._title, 1)
        layout.addWidget(self._action, 0)
        self.set(None)

    def set(self, severity: str | None, title: str = "", detail: str = "") -> None:
        """Show ``title`` (and an optional second line) at ``severity``.

        ``severity=None`` hides the banner, which only the "everything is fine
        and there is nothing to say" case uses.
        """
        self._severity = severity
        self._headline = title
        self._detail = detail
        if severity is None:
            self.setVisible(False)
            self._title.clear()
            return
        colour = self.SEVERITY_COLOURS.get(severity, theme.TEXT)
        text = f'<span style="color:{colour};font-weight:bold">{title}</span>'
        if detail:
            text += f'<br><span style="color:{theme.TEXT_MUTED}">{detail}</span>'
        self._title.setText(text)
        self.setStyleSheet(
            f"Banner {{ background: {theme.PANEL};"
            f" border-left: 4px solid {colour};"
            f" border-radius: {theme.RADIUS}px; }}"
        )
        self.setVisible(True)

    @property
    def title(self) -> str:
        """The visible text, markup and all, for tests."""
        return self._title.text()

    @property
    def severity(self) -> str | None:
        """What it was last set to, or None while hidden."""
        return self._severity

    @property
    def headline(self) -> str:
        """The first line on its own, without markup or the detail line."""
        return self._headline

    @property
    def detail(self) -> str:
        return self._detail

    def add_action(self, widget: QWidget) -> None:
        """Put a control inside the banner — a "确认" button, typically."""
        self._action.layout().addWidget(widget)


class StatusDot(QLabel):
    """A coloured dot and a word."""

    def __init__(self, state: str = "idle", text: str = "", parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._label = ""
        self.set(state=state, text=text)

    def set(self, state: str, text: str | None = None) -> None:
        colour = {
            "ok": theme.OK,
            "idle": theme.DISABLED,
            "warn": theme.WARN,
            "error": theme.ERROR,
        }.get(state, theme.DISABLED)
        label = text if text is not None else self._label
        self._label = label
        self.setText(
            f'<span style="color:{colour}">●</span> '
            f'<span style="color:{theme.TEXT_MUTED}">{label}</span>'
        )
        self.setToolTip(label)

    @property
    def label(self) -> str:
        """The words, without the markup.  ``text()`` returns the rich text."""
        return self._label


def separator() -> QFrame:
    """A horizontal rule, for grouping inside a panel."""
    line = QFrame()
    line.setFrameShape(QFrame.HLine)
    line.setStyleSheet(f"color: {theme.BORDER};")
    return line
