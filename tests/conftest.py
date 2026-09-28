"""Fixtures shared by the suite.

The path bootstrap and the offscreen Qt setting live in the repository-root
``conftest.py``, which runs before any of this.

Two things make these tests runnable with no hardware and no installed SDK: the
simulated backend is driven by an injected clock, so a 120-second thermal run
costs milliseconds and produces bit-identical results on every machine; and
``pytest-qt`` is not a dependency, so the ``qapp`` fixture below stands in for
it with a plain session-scoped ``QApplication``.
"""

from __future__ import annotations

import pytest

from litegrip_studio import constants
from litegrip_studio.backend.plant import Plant, PlantConfig
from litegrip_studio.backend.sim import SimBackend
from litegrip_studio.core.motion import MotionFSM, MotionParams, MotionState
from litegrip_studio.units import Limits

CTRL_DT = constants.CTRL_DT


class FakeClock:
    """A clock the test moves by hand.

    Injected into :class:`SimBackend` so the plant integrates simulated time
    rather than wall time — deterministic, and it makes long runs free.
    """

    def __init__(self, start: float = 1000.0) -> None:
        self.t = start

    def __call__(self) -> float:
        return self.t

    def advance(self, seconds: float) -> None:
        self.t += seconds


class Rig:
    """A connected, enabled simulated gripper driven at 200 Hz by hand.

    Holds the loop that every physics test would otherwise repeat: read
    telemetry, tick the FSM, advance the clock, poll.  ``run`` drives it for a
    simulated duration; ``run_until`` stops at a state or a deadline.
    """

    def __init__(
        self,
        *,
        stroke_mm: float | None = None,
        config: PlantConfig | None = None,
        params: MotionParams | None = None,
        connect: bool = True,
    ) -> None:
        self.clock = FakeClock()
        self.sim = SimBackend(config, clock=self.clock)
        if stroke_mm is not None:
            self.sim.plant.limits = self.sim.plant.limits.with_max_stroke(stroke_mm)
        if connect:
            self.sim.connect()
            self.sim.enable()
        self.limits = self.sim.limits()
        self.fsm = MotionFSM(self.limits, params or MotionParams())
        self.t = 0.0
        self.peak_force = 0.0

    # ── driving ─────────────────────────────────────────────────────────────
    def tick(self, allow_motion: bool = True):
        out = self.fsm.tick(self.sim, self.sim.read(), CTRL_DT, allow_motion=allow_motion)
        self.clock.advance(CTRL_DT)
        self.sim.poll()
        self.t += CTRL_DT
        # Tracked over the whole run, not just at the end: the interesting force
        # during a grasp is the transient at contact, which has come and gone by
        # the time the state machine settles into HOLD_FORCE.
        self.peak_force = max(self.peak_force, abs(self.sim.read().force_n))
        return out

    def run(self, seconds: float, allow_motion: bool = True) -> None:
        for _ in range(int(round(seconds / CTRL_DT))):
            self.tick(allow_motion)

    def run_until(
        self, state: MotionState, timeout_s: float = 30.0, allow_motion: bool = True
    ) -> bool:
        """Drive until ``state`` is reached.  True if it was, False on timeout."""
        for _ in range(int(round(timeout_s / CTRL_DT))):
            if self.fsm.state is state:
                return True
            self.tick(allow_motion)
        return self.fsm.state is state

    def settle(self, seconds: float = 0.5) -> None:
        """Run on with no further commands, letting the state machine finish."""
        self.run(seconds)

    # ── convenience ─────────────────────────────────────────────────────────
    @property
    def position_mm(self) -> float:
        return self.sim.read().position_mm

    @property
    def force_n(self) -> float:
        return abs(self.sim.read().force_n)

    def open(self, timeout_s: float = 40.0) -> bool:
        self.fsm.open()
        return self.run_until(MotionState.HOLD, timeout_s)

    def close(self, force_n: float | None = None, timeout_s: float = 40.0) -> bool:
        self.fsm.close(force_n=force_n)
        return self.run_until(MotionState.HOLD, timeout_s)


@pytest.fixture
def plant() -> Plant:
    """A bare plant, no clock and no backend."""
    return Plant()


@pytest.fixture
def limits(plant: Plant) -> Limits:
    return plant.limits


@pytest.fixture
def rig() -> Rig:
    """A connected simulator and its motion FSM."""
    return Rig()


@pytest.fixture(scope="session")
def qapp():
    """One QApplication for the whole session.

    ``pytest-qt`` is not installed here, so this stands in for its ``qapp``
    fixture.  Skipped rather than failed if PyQt5 is absent, so the pure-logic
    tests still run on a machine without a GUI stack.
    """
    qt_widgets = pytest.importorskip("PyQt5.QtWidgets")
    app = qt_widgets.QApplication.instance() or qt_widgets.QApplication([])
    yield app
