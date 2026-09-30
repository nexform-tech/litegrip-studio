"""Small shared widgets: readouts, the banner, and a status chip.

The formatting rules live in module-level functions rather than in the widgets,
so what an operator reads can be tested without instantiating Qt at all.  That
matters for the readout in particular: "is the target line shown?" is a policy
decision about when a number is worth showing, not a painting concern.

The three widgets here are the console's recurring *composed* shapes — a
caption over a monospaced number, a severity strip, a state chip — and they are
written as litearm-studio writes its badges: a tinted wash for the fill, the
same colour one step darker for the edge, and the colour itself for the text.
A severity therefore reads the same whether it is a banner across the window or
a chip on a card, and the fill is what carries the meaning rather than a single
saturated block.

Because a widget here paints a colour rather than asking for one, each of them
can re-render itself from what it was last told to show.  That is what
:func:`theme.subscribe` calls when the theme changes; without it, a banner set
while the console was dark would still be dark on a light console.
"""

from __future__ import annotations

from dataclasses import dataclass

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
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
        self._value = value
        #: None means "the ordinary ink", resolved at paint time.  Held as a
        #: colour it would be whichever palette was in force when this readout
        #: was built — and a readout that has not been given a number yet is
        #: exactly the one that has nothing to re-render it from.
        self._colour: str | None = None
        self.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.setMinimumWidth(96)
        self.setTextFormat(Qt.RichText)
        self.setToolTip(caption)
        self.restyle()

    def set_value(self, value: str, colour: str | None = None) -> None:
        self._value = value
        self._colour = colour
        self.restyle()

    def set_caption(self, caption: str) -> None:
        self._caption = caption

    def restyle(self) -> None:
        """Re-render in the current palette.

        Rich text rather than two widgets: the caption and the value have to
        stay on one baseline and inside one width, and the readout column is
        what keeps the slider from resizing as the numbers change.
        """
        colour = self._colour if self._colour is not None else theme.TEXT
        self.setText(
            f'<span style="color:{theme.TEXT_MUTED};font-size:{theme.FONT_SMALL_PX}px">'
            f"{self._caption}</span>"
            f'<br><span style="color:{colour};'
            f'font-family:{theme.MONO_FAMILY};font-size:{theme.FONT_PX + 2}px;'
            f'font-weight:600">{self._value}</span>'
        )

    @property
    def value(self) -> str:
        """The number, without the caption or the markup."""
        return self._value


class PositionReadout(QWidget):
    """The block to the right of the slider: actual, target, error."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.actual = Readout("实际位置", UNKNOWN)
        self.target = Readout("目标", "")
        self.error = Readout("误差", "")
        #: The last frame handed to :meth:`update_position`, or None before there
        #: has been one.  None rather than a tuple of Nones: a frame that
        #: carried no measurement *is* a rendering — it is the em dash — and it
        #: has to be re-rendered on a theme change just like any other.
        self._last: tuple[float | None, float | None, bool] | None = None

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        for widget in (self.actual, self.target, self.error):
            layout.addWidget(widget)
        layout.addStretch(1)

        # Fixed width: the slider is the thing that takes the space, and a
        # readout that widened as the numbers grew would move the slider under
        # the operator's hand.
        self.setFixedWidth(116)
        theme.subscribe(self.restyle)

    def update_position(
        self, actual_mm: float | None, target_mm: float | None = None, *,
        grasping: bool = False,
    ) -> None:
        """Draw the three lines, or one of them while the jaws hold something.

        ``grasping`` is the grasp that has been made and is being held under a
        force cap.  Its target is the closed end and its error is the width of
        the object between the jaws — both true, neither a place the jaws are
        going — and printed together they read as "28.0 mm from where you asked
        and not closing", which is the most alarming pair of numbers on the
        panel describing a grasp that is working exactly as intended.  The
        actual position stays, because it is the one number that is about the
        gripper rather than about the command.
        """
        self._last = (actual_mm, target_mm, grasping)
        text = format_readout(actual_mm, target_mm)
        self.actual.set_value(text.actual, theme.ACTUAL)
        self.target.set_value(text.target, theme.TARGET)
        self.error.set_value(text.error, theme.WARN if text.moving else theme.TEXT_MUTED)
        self.target.setVisible(not grasping and bool(text.target))
        self.error.setVisible(not grasping and bool(text.error))

    def restyle(self) -> None:
        """Repaint the last frame's numbers in the current palette."""
        if self._last is None:
            for readout in (self.actual, self.target, self.error):
                readout.restyle()
            return
        actual_mm, target_mm, grasping = self._last
        self.update_position(actual_mm, target_mm, grasping=grasping)


class Banner(QFrame):
    """A strip that stays put, for a fact the operator has to keep seeing.

    Used for the calibration source and for an alert in force.  A banner rather
    than a dialog because the fact does not go away: a console running on the
    factory file says so for as long as it is running, and one that said it once
    in a modal that was clicked away would be worse than silent.

    Drawn as litearm's alert is: a tinted wash for the fill, a matching edge, and
    the accent colour on the text.  The left edge is kept heavier than the other
    three because a strip this wide is read at a glance, and the edge is what
    says how bad it is before any of the words do.

    The fill, the edge and the text all come from :func:`theme.severity_colours`
    at paint time rather than from a table captured at import: a table built
    here would hold whichever palette happened to be in force when the module
    loaded, and would go on painting it after the console switched.
    """

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("Banner")
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
        layout.setContentsMargins(12, 9, 12, 9)
        layout.setSpacing(10)
        layout.addWidget(self._title, 1)
        layout.addWidget(self._action, 0)
        self.set(None)
        # Subscribed rather than driven by whoever owns it: a banner can hang off
        # the window or off a page, and one that relied on its owner remembering
        # to repaint it would be left wearing the old palette by the owner that
        # forgot.  Held weakly, so the banner going away unsubscribes it.
        theme.subscribe(self.restyle)

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
        self.restyle()
        self.setVisible(True)

    def restyle(self) -> None:
        """Repaint the banner, and its strip, in the current palette."""
        if self._severity is None:
            return
        colour, fill, line = theme.severity_colours(self._severity)
        text = f'<span style="color:{colour};font-weight:600">{self._headline}</span>'
        if self._detail:
            text += f'<br><span style="color:{theme.TEXT_MUTED}">{self._detail}</span>'
        self._title.setText(text)
        self.setStyleSheet(
            f"#Banner {{ background: {fill};"
            f" border: 1px solid {line};"
            f" border-left: 3px solid {colour};"
            f" border-radius: {theme.RADIUS}px; }}"
        )

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
    """A state chip: a coloured dot and a word, on a tinted wash.

    Still called a dot because that is what it is on the bar — the colour is
    read before the word — but drawn as a chip so that a row of them on a card
    reads as a row of states rather than as a column of loose text.

    The state-to-colour table is read at paint time.  Held as a class attribute
    it would be built once, against whichever palette was in force at import,
    and a theme switch would leave every chip on the old one.
    """

    def __init__(self, state: str = "idle", text: str = "", parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._state = state
        self._label = ""
        self.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        self.set(state=state, text=text)
        theme.subscribe(self.restyle)

    def set(self, state: str, text: str | None = None) -> None:
        self._state = state
        if text is not None:
            self._label = text
        self.restyle()
        self.setToolTip(self._label)

    @staticmethod
    def _state_colour(state: str) -> str:
        """The dot's colour: the four known states, and muted ink for the rest.

        A state this module has not been taught about is not the operator's
        emergency, so it reads as inactive rather than as an error.
        """
        return {
            "ok": theme.OK,
            "idle": theme.DISABLED,
            "warn": theme.WARN,
            "error": theme.ERROR,
        }.get(state, theme.DISABLED)

    def restyle(self) -> None:
        """Repaint the chip, dot and wash, in the current palette."""
        # ``ok`` and ``idle`` are the two states a bar sits in most of the time
        # and neither is a warning, so they take the neutral wash; the two that
        # are warnings take their own colour, which is what makes a link that
        # has gone quiet visible from across the room.
        if self._state in ("warn", "error"):
            colour, fill, line = theme.severity_colours(self._state)
        else:
            colour = self._state_colour(self._state)
            fill = theme.PANEL_ALT
            line = theme.BORDER
        self.setText(
            f'<span style="color:{colour}">●</span> '
            f'<span style="color:{theme.TEXT_MUTED}">{self._label}</span>'
        )
        self.setStyleSheet(
            f"background: {fill}; border: 1px solid {line};"
            f" border-radius: {theme.RADIUS_PILL}px;"
            f" padding: 3px 12px 3px 10px;"
        )

    @property
    def label(self) -> str:
        """The words, without the markup.  ``text()`` returns the rich text."""
        return self._label

    @property
    def state(self) -> str:
        """What it was last set to, for a restyle and for tests."""
        return self._state



class ThemeSwitch(QFrame):
    """The theme, as two pills with the one in force lit.

    Not one button.  A single toggle has to be labelled either with what is on
    now or with what a click will do, and "主题：深色" reads perfectly well as
    either — the operator cannot tell which without clicking it, which is the
    one thing a control should never require.  Showing both names and lighting
    the active one removes the question: there is nothing left to guess.

    It is also the shape the tab strip already uses, so the console has one
    segmented control rather than two ideas about one.
    """

    #: The name of the theme the operator asked for.
    chosen = pyqtSignal(str)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("Segmented")
        self.setFrameShape(QFrame.NoFrame)
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

        self._buttons: dict[str, QPushButton] = {}
        layout = QHBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(2)
        for name in (theme.DARK, theme.LIGHT):
            button = QPushButton(theme.PALETTES[name].label)
            button.setCheckable(True)
            button.setProperty("segment", True)
            button.setCursor(Qt.PointingHandCursor)
            button.setToolTip(f"使用{theme.PALETTES[name].label}主题")
            button.clicked.connect(lambda _checked=False, key=name: self.chosen.emit(key))
            layout.addWidget(button)
            self._buttons[name] = button

        self.set_current(theme.current_theme())

    def set_current(self, name: str) -> None:
        """Light ``name`` without emitting anything — for a repaint, or for the
        window telling the switch what the rest of the console is already on."""
        for key, button in self._buttons.items():
            button.setChecked(key == name)

    def choose(self, name: str) -> None:
        """Ask for ``name`` exactly as a click would.  For a test, or a shortcut."""
        self.set_current(name)
        self.chosen.emit(name)

    @property
    def current(self) -> str:
        return next(key for key, button in self._buttons.items() if button.isChecked())

    def button(self, name: str) -> QPushButton:
        return self._buttons[name]


def separator() -> QFrame:
    """A horizontal rule, for grouping inside a panel."""
    line = QFrame()
    line.setFrameShape(QFrame.HLine)
    line.setStyleSheet(f"color: {theme.BORDER_SOFT}; background: {theme.BORDER_SOFT};")
    line.setFixedHeight(1)
    return line
