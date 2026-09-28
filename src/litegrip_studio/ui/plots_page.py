"""The plots page: the last few seconds of position and force.

Three decisions shape this page.

*The plots are fed at their own rate.*  Frames arrive at 50 Hz and the plot is
drawn at 25 Hz, because the extra twenty samples a second cost repaints that no
eye can resolve.  The decimation is by arrival time rather than by counting
every second frame, so a burst of queued frames — which is what a stalled GUI
looks like when it catches up — does not push the window's worth of samples off
the end of the buffer.

*A gap is drawn as a gap.*  The commanded position exists only while the FSM is
running a profile, and the honest rendering of "no target" is a break in the
line, not a line drawn through the last known value.

*The force axis follows the trace.*  Fixed at the 40 N rating it says what the
mechanism *may* do rather than what it is doing: a 12 N grip then draws in the
bottom third of the plot, and the variation inside a grip — the thing an
operator watches to see whether the jaws are settling or creeping — is a couple
of pixels of it.  So the axis is fitted to the window on every redraw, rounded up
to a readable number and never clipped.  See :func:`_axis_bound` for the rounding
and :meth:`PlotsPage._range_the_force_axis` for the hysteresis that keeps it from
chattering.  The position axis deliberately does *not* do this: it is ranged to
the travel, which is what makes the same 10 s of motion look the same on every
gripper.
"""

from __future__ import annotations

import math
from collections import deque

import pyqtgraph as pg
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPen
from PyQt5.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from .. import constants
from ..telemetry import TelemetryFrame
from . import theme

#: Seconds of history on screen.
WINDOW_S = 10.0
#: Interval between plotted samples, from the frame rate the worker publishes at.
PLOT_INTERVAL_S = 1.0 / constants.PLOT_HZ
#: Slack on that interval.  The publish clock and the plot clock are two
#: different lattices — 50 Hz against 25 Hz means every interval lands exactly
#: on the boundary, where a float two ulps short of the threshold turns the
#: sample rate into an irregular 20–25 Hz.  A millisecond is far below anything
#: the eye can see and far above the error.
PLOT_JITTER_S = 1e-3
#: One extra so the oldest point leaves the window rather than clipping at it.
CAPACITY = int(WINDOW_S * constants.PLOT_HZ) + 1

#: A target that does not exist, as pyqtgraph understands it.
NO_VALUE = float("nan")

#: Smallest force axis to draw, in newtons, as a full span.  Force is plotted
#: signed (the SDK's torque has no sign guarantee), so this is split either side
#: of zero.  Below it the axis would not be an axis: the noise of a free move —
#: a fraction of a newton of braking torque — would be magnified to fill the
#: plot and read as a grip.
FORCE_AXIS_MIN_N = 2.0
#: How much clear air to leave above the peak before rounding, so the trace does
#: not touch the top edge and look clipped.
FORCE_AXIS_HEADROOM = 1.15
#: How far the axis must be from where the data now wants it, as a fraction of
#: the current span, before it is moved.  Rounding to 1/2/5 already absorbs most
#: of the jitter; this absorbs the rest at the boundary between two steps, where
#: a peak oscillating either side of it would otherwise flip the axis between
#: e.g. 10 N and 20 N at 25 Hz.
FORCE_AXIS_HYSTERESIS = 0.25


def _axis_bound(peak: float) -> float:
    """Round a peak magnitude up to 1, 2 or 5 times a power of ten.

    Readable numbers only: an axis that ends at 13.2 N gives the reader no
    reference point, and one that changes by a random amount every time the
    peak moves is worse than a fixed one.  The rounding is also what makes the
    axis stable — a grip that wanders between 11.9 N and 12.1 N lands on 20 N
    either way and the axis does not move at all.
    """
    if peak <= 0.0 or not math.isfinite(peak):
        return 0.0
    scaled = peak * FORCE_AXIS_HEADROOM
    decade = 10.0 ** math.floor(math.log10(scaled))
    for multiple in (1.0, 2.0, 5.0):
        if scaled <= multiple * decade:
            return round(multiple * decade, 6)
    return round(10.0 * decade, 6)


def _configure() -> None:
    pg.setConfigOptions(antialias=True, background=theme.PANEL, foreground=theme.TEXT_MUTED)


_configure()


def _pen(colour: str, *, dashed: bool = False, width: int = 2) -> QPen:
    pen = pg.mkPen(colour, width=width)
    if dashed:
        pen.setStyle(Qt.DashLine)
    return pen


class PlotsPage(QWidget):
    """A rolling position plot and a rolling force plot, sharing a time axis."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._t: deque[float] = deque(maxlen=CAPACITY)
        self._position: deque[float] = deque(maxlen=CAPACITY)
        self._target: deque[float] = deque(maxlen=CAPACITY)
        self._force: deque[float] = deque(maxlen=CAPACITY)
        self._last_plot_t: float | None = None
        #: Wall-clock time of the first sample on screen.  The axis counts from
        #: it so the labels read "12.4 s" rather than a monotonic clock value.
        self._origin: float | None = None
        self._paused = False

        self._position_plot = self._make_plot("位置 (mm)")
        self._force_plot = self._make_plot("夹持力 (N)")

        self._actual_curve = self._position_plot.plot(
            pen=_pen(theme.ACTUAL), name="实测"
        )
        self._target_curve = self._position_plot.plot(
            pen=_pen(theme.TARGET, dashed=True), name="目标"
        )
        self._force_curve = self._force_plot.plot(pen=_pen(theme.WARN), name="力")

        self._position_plot.setYRange(0.0, constants.STROKE_MAX_MM, padding=0.0)
        self._force_plot.setYRange(
            -FORCE_AXIS_MIN_N / 2.0, FORCE_AXIS_MIN_N / 2.0, padding=0.0
        )
        self._force_plot.setXLink(self._position_plot)

        self._pause = QPushButton("暂停")
        self._pause.setCheckable(True)
        self._pause.setToolTip("冻结曲线；恢复时清空后重新开始，避免跨越暂停画出一条直线")
        self._pause.toggled.connect(self._on_pause)
        self._clear = QPushButton("清空")
        self._clear.clicked.connect(self.clear)

        self._note = QLabel()
        self._note.setTextFormat(Qt.RichText)

        controls = QHBoxLayout()
        controls.setSpacing(8)
        controls.addWidget(self._pause)
        controls.addWidget(self._clear)
        controls.addWidget(self._note, 1)

        layout = QVBoxLayout(self)
        layout.setSpacing(6)
        layout.addLayout(controls)
        layout.addWidget(self._position_plot, 1)
        layout.addWidget(self._force_plot, 1)

    def _make_plot(self, title: str) -> pg.PlotWidget:
        plot = pg.PlotWidget()
        plot.setTitle(title, color=theme.TEXT, size="10pt")
        plot.showGrid(x=True, y=True, alpha=0.15)
        plot.setLabel("bottom", "时间", units="s")
        # The x axis scrolls itself, so panning it would be undone by the next
        # sample; the y axis is the operator's to zoom.
        plot.setMouseEnabled(x=False, y=True)
        plot.hideButtons()
        return plot

    # ── slots ───────────────────────────────────────────────────────────────
    def update_frame(self, frame: TelemetryFrame) -> None:
        if self._paused:
            return
        if self._last_plot_t is not None:
            if frame.t - self._last_plot_t < PLOT_INTERVAL_S - PLOT_JITTER_S:
                return
        self._last_plot_t = frame.t
        self.append(frame.t, frame.position_mm, frame.cmd_mm, frame.force_n)

    def append(self, t: float, position_mm: float | None, cmd_mm: float | None,
               force_n: float) -> None:
        """Add one sample.  Split out from :meth:`update_frame` so the tests can
        fill the buffer without owning a clock.

        A missing sample is a gap rather than a zero, for the same reason the
        target is: pyqtgraph breaks the line at NaN, so a position nobody has
        measured cannot be read as the closed stop.
        """
        if self._origin is None:
            self._origin = t
        self._t.append(t - self._origin)
        self._position.append(NO_VALUE if position_mm is None else position_mm)
        self._target.append(NO_VALUE if cmd_mm is None else cmd_mm)
        self._force.append(force_n)
        self._redraw()

    def clear(self) -> None:
        self._t.clear()
        self._position.clear()
        self._target.clear()
        self._force.clear()
        self._last_plot_t = None
        self._origin = None
        self._redraw()

    def set_limits(self, limits) -> None:
        """Range the position axis to the travel the calibration describes.

        A plot fixed at 0–300 mm shows a 120 mm gripper using half its height,
        which hides exactly the detail the plot exists to show.
        """
        if limits is None:
            self._position_plot.setYRange(0.0, constants.STROKE_MAX_MM, padding=0.0)
            return
        # Millimetres run from nought at the closed end to the travel at the
        # open end, which is the same axis the slider spans.
        self._position_plot.setYRange(0.0, limits.max_stroke_mm, padding=0.0)

    # ── rendering ───────────────────────────────────────────────────────────
    def _on_pause(self, paused: bool) -> None:
        self._paused = paused
        self._pause.setText("继续" if paused else "暂停")
        if paused:
            self._note.setText(
                f'<span style="color:{theme.WARN}">'
                f"已暂停（恢复后曲线清空重画）</span>"
            )
            return
        self._note.clear()
        # Resuming starts a new buffer.  The samples either side of the pause
        # are separated by an interval nobody measured, and joining them with a
        # straight line would draw a movement that never happened.
        self.clear()

    def _redraw(self) -> None:
        times = list(self._t)
        self._actual_curve.setData(times, list(self._position))
        self._target_curve.setData(times, list(self._target))
        self._force_curve.setData(times, list(self._force))
        if times:
            # A fixed-width window whose right edge is the newest sample: it
            # fills as the first samples arrive and then scrolls.
            start = max(times[-1] - WINDOW_S, times[0])
            self._position_plot.setXRange(start, start + WINDOW_S, padding=0.0)
        self._range_the_force_axis()

    def _range_the_force_axis(self) -> None:
        """Fit the force axis to the samples in the window — see the docstring.

        Read from the window rather than from a running maximum, so the axis
        comes back down on its own a few seconds after a spike has scrolled off,
        with no timer to get wrong.  A sample that cannot be read is skipped
        rather than treated as zero: a force nobody has measured is not a force
        of nothing, and letting one set the axis would collapse it to the floor
        for as long as the sample stayed on screen.

        Both decisions are taken against the *peaks*, not against the rounded
        bounds: the axis opens the moment a peak would not fit, and closes only
        once both peaks are a quarter of the way inside it.  Comparing rounded
        bounds instead leaves a peak sitting on a step boundary — 8.7 N rounds to
        20 N, 8.6 N to 10 N — flipping the axis between two heights at 25 Hz with
        the data barely moving.
        """
        samples = [f for f in self._force if math.isfinite(f)]
        peak_up = max((v for v in samples if v > 0.0), default=0.0)
        peak_down = max((-v for v in samples if v < 0.0), default=0.0)
        low, high = self._force_plot.getViewBox().viewRange()[1]

        fits = peak_up <= high and peak_down <= -low
        roomy = (
            peak_up <= high * (1.0 - FORCE_AXIS_HYSTERESIS)
            and peak_down <= -low * (1.0 - FORCE_AXIS_HYSTERESIS)
        )
        if fits and not roomy:
            return

        top = max(_axis_bound(peak_up), FORCE_AXIS_MIN_N / 2.0)
        bottom = -max(_axis_bound(peak_down), FORCE_AXIS_MIN_N / 2.0)
        if (bottom, top) != (low, high):
            self._force_plot.setYRange(bottom, top, padding=0.0)

    @property
    def paused(self) -> bool:
        return self._paused

    @property
    def samples(self) -> list[tuple[float, float, float, float]]:
        """The buffer as ``(t, position, target, force)`` rows, for tests."""
        return list(zip(self._t, self._position, self._target, self._force))
