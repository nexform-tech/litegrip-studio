"""The position bar: drag it to command the gripper, watch it track the jaws.

Three values, three visual elements, and keeping them apart is the whole design:

============  ==========================================  ====================
element       where it comes from                          what it means
============  ==========================================  ====================
handle        the operator's drag, or the measured         where the jaws are
              position when the console is driving them    being told to go
target line   the last position that was commanded          what was asked for
actual bar    telemetry, 50 Hz                             where they really are
============  ==========================================  ====================

The rule that makes the difference visible is in :meth:`set_actual`: the handle
follows the measured position *unless* the operator is dragging it or the axis
is going where the slider asked.  So a move started from the 闭合 button carries
the handle along with it — the progress bar fills during the move, which is the
thing that was asked for — while a move started from the slider leaves the
handle where the hand put it, and the gap between the handle and the green mark
is the tracking error, live.

Committing on release rather than on every pixel is the plan's default,
following litearm-studio's split between ``onValueChange`` and ``onValueCommit``.
It is what makes a drag across the bar one move instead of a hundred.  Live
following is available and is safe — every command is rate-limited by the motion
profile, so a drag that spans the whole travel still moves at the set speed —
but it is off by default because a hundred queued moves is a worse description
of "put them here" than one.

The widget also refuses to move on a scroll wheel.  Qt's default is to obey it,
and on a page that scrolls, a wheel event aimed at the page would silently
re-command the gripper.
"""

from __future__ import annotations

from PyQt5.QtCore import QPointF, QRectF, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import QSizePolicy, QSlider, QWidget

from ..units import Limits
from . import theme

#: Slider steps per millimetre.  0.1 mm, which is finer than the travel can be
#: commanded to anyway: the 16-bit encoder quantum is about 0.025 mm and the
#: arrival band is 0.4 mm, so this is display resolution rather than a limit.
RESOLUTION = 10

#: The owner value meaning "the axis is going where this slider asked".
OWNER_SLIDER = "slider"

# The vertical layout, top to bottom: handle, groove, the actual-position
# triangle below it, then the two end labels.  Written as a stack with explicit
# offsets rather than as fractions of the widget height, because every element
# has a fixed size and a fraction-based layout only lines up at one height.
HANDLE_W = 14.0
HANDLE_H = 22.0
HANDLE_TOP = 8.0
GROOVE_H = 8.0
MARKER_H = 7.0
LABEL_H = 14.0
TRIANGLE_TOP = HANDLE_TOP + HANDLE_H + 4.0
LABEL_TOP = TRIANGLE_TOP + MARKER_H + 2.0
MIN_HEIGHT = int(LABEL_TOP + LABEL_H + 4.0)


class StrokeSlider(QSlider):
    """A horizontal QSlider that draws the jaws as well as the command."""

    #: Every update while the hand is down.  Only emitted when the operator has
    #: asked for live following; see the module docstring.
    dragMoved = pyqtSignal(float)
    #: The operator finished: a release, or a keyboard step that landed.
    dragCommitted = pyqtSignal(float)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(Qt.Horizontal, parent)
        self._limits: Limits | None = None
        self._dragging = False
        self._owner: str | None = None
        self._actual_mm: float | None = None
        self._target_mm: float | None = None
        self._blocked_reason = ""
        self._live_follow = False

        self.setMinimum(0)
        self.setMaximum(int(round(120.0 * RESOLUTION)))
        self.setSingleStep(5)  # 0.5 mm
        self.setPageStep(50)  # 5 mm
        self.setTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.setMinimumHeight(MIN_HEIGHT)
        self.set_value_mm(0.0)

    # ── configuration ───────────────────────────────────────────────────────
    def set_limits(self, limits: Limits | None) -> None:
        """Set the travel the bar spans.

        The range is the *commanded* travel, ``max_stroke_mm``, which is what
        :meth:`~litegrip_studio.units.Limits.clamp_mm` enforces and what the
        motion planner will therefore honour.  The travel the calibration
        implies is a different number and may not match; the calibration page is
        where the two are compared, because silently reconciling them here would
        hide exactly the disagreement worth seeing.
        """
        self._limits = limits
        if limits is None:
            self.setEnabled(False)
            return
        self.setMaximum(int(round(limits.max_stroke_mm * RESOLUTION)))
        self._sync_enabled()
        self.update()

    def set_live_follow(self, live: bool) -> None:
        """Whether a drag commands continuously, or only on release."""
        self._live_follow = bool(live)

    def set_owner(self, owner: str | None) -> None:
        """Say who is driving the axis now.

        The control page sets this when it issues a command, because the slider
        cannot see the command queue.  ``None`` means nobody — the axis is at
        rest and the handle may follow the measurement again.
        """
        self._owner = owner
        self.update()

    def set_blocked(self, reason: str) -> None:
        """Refuse to be dragged, and say why on hover.

        Disabling the widget is the first of the four gate layers; the other
        three are in the worker, the profile and the backend.  It is here so the
        operator is not offered a control that would do nothing.
        """
        self._blocked_reason = reason
        if reason:
            self.setToolTip(f"位置控制不可用：{reason}")
        else:
            self.setToolTip("拖动设定位置（松手后执行）")
        self._sync_enabled()

    def _sync_enabled(self) -> None:
        self.setEnabled(self._limits is not None and not self._blocked_reason)

    # ── the three values ────────────────────────────────────────────────────
    @property
    def value_mm(self) -> float:
        """Where the handle is — the position the operator has selected."""
        return self.value() / RESOLUTION

    def set_value_mm(self, mm: float) -> None:
        """Move the handle without emitting a command."""
        self._set_value_mm(mm)

    def set_actual(self, mm: float | None) -> None:
        """The measured position, from telemetry at 50 Hz.

        The handle only follows when the operator is not holding it and the
        axis is not going where the slider asked: a handle dragged out from
        under the finger is unusable, and a handle that chased the jaws during a
        slider-owned move would hide the tracking error, which is the one thing
        this mark exists to show.

        ``None`` is "not measured yet" rather than zero: the mark is not drawn
        at all, and the handle stays where it is instead of being moved to a
        position no frame has reported.
        """
        self._actual_mm = mm
        if mm is not None and not self._dragging and self._owner != OWNER_SLIDER:
            self._set_value_mm(mm)
        self.update()

    def set_target(self, mm: float | None) -> None:
        """The position last commanded, drawn as the thin line."""
        self._target_mm = mm
        self.update()

    @property
    def actual_mm(self) -> float | None:
        return self._actual_mm

    @property
    def target_mm(self) -> float | None:
        return self._target_mm

    @property
    def dragging(self) -> bool:
        return self._dragging

    def _set_value_mm(self, mm: float) -> None:
        """Set the handle, without letting ``valueChanged`` look like a drag.

        Programmatic moves are not commands, and a listener that treated them as
        such would command the gripper every time telemetry arrived.
        """
        blocked = self.blockSignals(True)
        try:
            self.setValue(int(round(mm * RESOLUTION)))
        finally:
            self.blockSignals(blocked)

    # ── interaction ─────────────────────────────────────────────────────────
    def mousePressEvent(self, event) -> None:
        if event.button() != Qt.LeftButton or not self.isEnabled():
            super().mousePressEvent(event)
            return
        self._dragging = True
        self._jump_to(event.pos().x())
        self._emit_move()
        event.accept()

    def mouseMoveEvent(self, event) -> None:
        if not self._dragging:
            return
        self._jump_to(event.pos().x())
        self._emit_move()
        event.accept()

    def mouseReleaseEvent(self, event) -> None:
        if not self._dragging:
            super().mouseReleaseEvent(event)
            return
        self._dragging = False
        self._jump_to(event.pos().x())
        self.dragCommitted.emit(self.value_mm)
        event.accept()

    def keyPressEvent(self, event) -> None:
        """Arrow keys are a command too.

        A single key press is one deliberate position, and there is no release
        to wait for, so it commits immediately — the same rule as a drag, minus
        the drag.
        """
        before = self.value()
        super().keyPressEvent(event)
        if self.isEnabled() and self.value() != before:
            self.dragCommitted.emit(self.value_mm)

    def wheelEvent(self, event) -> None:
        """Deliberately ignored — see the module docstring."""
        event.ignore()

    def _emit_move(self) -> None:
        if self._live_follow:
            self.dragMoved.emit(self.value_mm)

    def _jump_to(self, x: float) -> None:
        self._set_value_mm(self._value_for_x(x))

    # ── geometry, shared by the painter and the mouse ───────────────────────
    def _groove_rect(self) -> QRectF:
        margin = HANDLE_W / 2.0
        top = HANDLE_TOP + (HANDLE_H - GROOVE_H) / 2.0
        return QRectF(margin, top, max(1.0, self.width() - 2 * margin), GROOVE_H)

    def _x_for_mm(self, mm: float) -> float:
        """Where ``mm`` sits on the bar, clamped to the bar.

        Clamped here rather than at each call site so no mark can be drawn
        outside the travel: a position beyond either end means the calibration
        and the encoder disagree, and drawing it off the end would say that
        while pointing at nothing.
        """
        groove = self._groove_rect()
        span = self.maximum() - self.minimum()
        if span <= 0:
            return groove.left()
        steps = min(max(mm * RESOLUTION, self.minimum()), self.maximum())
        fraction = (steps - self.minimum()) / span
        return groove.left() + fraction * groove.width()

    def _value_for_x(self, x: float) -> float:
        """Inverse of :meth:`_x_for_mm`, in mm, clamped into the travel.

        Deliberately the same geometry function as the painter rather than Qt's
        own style-based mapping: if the two disagreed, the mark would sit
        somewhere other than where the jaws are, and that is the failure this
        widget exists to avoid.
        """
        groove = self._groove_rect()
        span = self.maximum() - self.minimum()
        fraction = (x - groove.left()) / groove.width() if groove.width() > 0 else 0.0
        fraction = min(max(fraction, 0.0), 1.0)
        return (self.minimum() + fraction * span) / RESOLUTION

    # ── painting ────────────────────────────────────────────────────────────
    def paintEvent(self, event) -> None:
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        groove = self._groove_rect()
        enabled = self.isEnabled()

        # 1. the groove
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(theme.GROOVE))
        painter.drawRoundedRect(groove, GROOVE_H / 2, GROOVE_H / 2)

        # 2. how much of the travel the jaws have actually covered.  Filled from
        #    the closed end, so the bar reads as a progress bar for the move —
        #    and it is the measurement, not the command, that fills it.
        if self._actual_mm is not None:
            filled = QRectF(
                groove.left(), groove.top(),
                self._x_for_mm(self._actual_mm) - groove.left(), groove.height(),
            )
            if filled.width() > 0.5:
                colour = QColor(theme.ACTUAL if enabled else theme.DISABLED)
                colour.setAlpha(90 if enabled else 50)
                painter.setBrush(colour)
                painter.drawRoundedRect(filled, GROOVE_H / 2, GROOVE_H / 2)

        # 3. the target: a thin line, which stays where it is when the jaws are
        #    blocked short of it.
        if self._target_mm is not None:
            x = self._x_for_mm(self._target_mm)
            painter.setPen(QPen(QColor(theme.TARGET if enabled else theme.DISABLED), 2))
            painter.drawLine(QPointF(x, groove.top() - 5), QPointF(x, groove.bottom() + 5))

        # 4. the jaws.  A triangle rather than a line so it cannot be confused
        #    with the target even in a screenshot, where neither is moving.
        if self._actual_mm is not None:
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor(theme.ACTUAL if enabled else theme.DISABLED))
            painter.drawPolygon(QPolygonF(_triangle(self._x_for_mm(self._actual_mm), TRIANGLE_TOP)))

        # 5. the handle, last, so it is never hidden by a mark.
        centre = self._x_for_mm(self.value_mm)
        handle = QRectF(
            centre - HANDLE_W / 2, groove.center().y() - HANDLE_H / 2, HANDLE_W, HANDLE_H
        )
        painter.setPen(QPen(QColor(theme.BORDER), 1))
        painter.setBrush(QColor(theme.TEXT if enabled else theme.DISABLED))
        painter.drawRoundedRect(handle, 3, 3)
        painter.setPen(QPen(QColor(theme.BACKGROUND), 1))
        painter.drawLine(
            QPointF(handle.center().x(), handle.top() + 3),
            QPointF(handle.center().x(), handle.bottom() - 3),
        )

        # 6. the two ends of the travel, named, because "the left end is closed"
        #    is a fact about this mechanism rather than something to infer.
        painter.setPen(QPen(QColor(theme.TEXT_MUTED if enabled else theme.DISABLED)))
        font = QFont(painter.font())
        font.setPointSize(8)
        painter.setFont(font)
        painter.drawText(
            QRectF(0.0, LABEL_TOP, 90.0, LABEL_H),
            Qt.AlignLeft | Qt.AlignVCenter,
            "闭合 0",
        )
        if self._limits is not None:
            painter.drawText(
                QRectF(self.width() - 90.0, LABEL_TOP, 90.0, LABEL_H),
                Qt.AlignRight | Qt.AlignVCenter,
                f"张开 {self._limits.max_stroke_mm:.0f} mm",
            )

    def sizeHint(self):
        hint = super().sizeHint()
        hint.setHeight(MIN_HEIGHT)
        return hint


def _triangle(x: float, y: float) -> list[QPointF]:
    """A downward marker centred on ``x``, with its apex at ``y``."""
    return [
        QPointF(x, y),
        QPointF(x - MARKER_H, y + MARKER_H),
        QPointF(x + MARKER_H, y + MARKER_H),
    ]
