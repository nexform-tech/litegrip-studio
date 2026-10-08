"""Fixtures shared by the suite, and the home directory the suite runs in.

The path bootstrap and the offscreen Qt setting live in the repository-root
``conftest.py``, which runs before any of this.  The home directory is moved
aside here, next to the fixtures that would otherwise reach the operator's.

Two things make these tests runnable with no hardware: the simulated backend is
driven by an injected clock, so a 120-second thermal run costs milliseconds and
produces bit-identical results on every machine; and ``pytest-qt`` is not a
dependency, so the ``qapp`` fixture below stands in for it with a plain
session-scoped ``QApplication``.  Nothing has to be installed for the SDK
either — it is vendored under ``src/litegrip`` and the root ``conftest.py`` puts
it on the path.
"""

from __future__ import annotations

import atexit
import os
import shutil
import tempfile
from pathlib import Path

import pytest

from litegrip_studio import constants
from litegrip_studio.backend.plant import Plant, PlantConfig
from litegrip_studio.backend.sim import SimBackend
from litegrip_studio.core.motion import MotionFSM, MotionParams, MotionState
from litegrip_studio.units import Limits

CTRL_DT = constants.CTRL_DT

# ── the home the suite runs in ─────────────────────────────────────────────
# Everything the console owns lives under ``Path.home()``: the bench gripper's
# calibration, the simulator's, the preferences, the log.  Most tests name the
# file they mean, but several construct a backend with no path at all — the
# ``Rig`` below does, and so does every test that builds a ``SimBackend`` to
# exercise the physics — and those resolve the defaults.
#
# On a machine where a console has been run, resolving the defaults means
# reading the operator's data, and a test whose numbers come out of it stops
# being a statement about the code.  That is not hypothetical: a simulated
# calibration saved from a console running alongside the suite was picked up
# this way and moved the millimetres per rad far enough to fail
# ``test_motion_fsm``, a file in which no calibration is mentioned at all.
#
# Pointing ``HOME`` at a directory of the suite's own closes every one of those
# doors at once, including the SDK's ``DEFAULT_CALIB`` (gripper.py:39), and it
# does so without stubbing the functions that compute the paths: a test may
# still assert that a default resolves to ``~/.litegrip/...``, because that is
# still exactly what it does.  The two environment variables are dropped for the
# same reason — they point the same two files outside the redirect, so an
# operator who exported one in their shell would otherwise still be testing
# against their own gripper.
#
# ``OPERATOR_HOME`` is kept so a test can say what it must not be reading.
OPERATOR_HOME = Path.home()
TEST_HOME = Path(tempfile.mkdtemp(prefix="litegrip-studio-tests-"))
atexit.register(shutil.rmtree, TEST_HOME, ignore_errors=True)
os.environ["HOME"] = str(TEST_HOME)
for _var in ("LITEGRIP_CALIB", "LITEGRIP_FACTORY_CALIB"):
    os.environ.pop(_var, None)

REPO_ROOT = Path(__file__).resolve().parents[1]


def shipped_factory_calibrations() -> list[Path]:
    """Every copy of the factory calibration this repository ships.

    Exactly one copy exists, and where it lives depends on the checkout: the
    console carries its own, and the SDK vendored into ``src/litegrip`` carries
    the same file.  Found by search so a test comparing against it can say "the
    file this repository ships" without naming a directory only one layout has.

    The numbers in it are the ones the console falls back to on a machine with
    no user calibration, which is why a fixture claiming to be this file has to
    be compared against it rather than trusted.
    """
    return sorted((REPO_ROOT / "src").rglob("factory_calibration.json"))


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
