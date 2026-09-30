"""The suite must not read the operator's calibration, preferences or log.

:mod:`tests.test_motion_fsm` is physics: it pins how the jaws move, and nothing
in it mentions a calibration file.  It failed anyway — ``assert 40.33975573809752
< 40.0`` — because the rig driving it builds a simulator with no calibration path,
the simulator resolved ``~/.litegrip/litegrip_calibration.sim.json``, and a console
running next to the suite had just saved that file from the bench.  The
millimetres per rad changed and the physics test moved with it.

This file pins the property that makes the rest of the suite trustworthy: the home
the tests resolve their defaults in is the suite's own, not the one the operator's
console writes.
"""

from __future__ import annotations

import os
import subprocess
import sys
import tempfile
from pathlib import Path

from litegrip_studio import calibration, logging_setup, settings
from litegrip_studio.backend.sim import default_sim_calibration_path

from conftest import OPERATOR_HOME, TEST_HOME, Rig

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
#: Mirrors the root ``conftest.py``, which is what puts these on the path when
#: the suite runs; a subprocess has to be told as well.
SDK = Path(os.environ.get("LITEGRIP_SDK_PATH") or REPO.parent / "lite-grip")


class TestTheSuiteHasAHomeOfItsOwn:
    def test_the_home_is_not_the_operators(self) -> None:
        """The one thing every other test in this file rests on."""
        assert TEST_HOME != OPERATOR_HOME
        assert Path.home() == TEST_HOME

    def test_every_default_lands_inside_it(self) -> None:
        """The four files a console keeps under the home directory.

        All four, rather than the calibration alone: they are the same defect in
        the same shape, and one left behind would be read by some later test that
        has nothing to do with the file.
        """
        defaults = {
            "the gripper's calibration": calibration.default_user_path(),
            "the simulator's calibration": default_sim_calibration_path(),
            "the preferences": settings.default_settings_path(),
            "the log": logging_setup.default_log_path(),
        }

        assert Path.home() != OPERATOR_HOME, "the suite is reading the operator's home"
        for what, path in defaults.items():
            assert TEST_HOME in path.parents, f"{what} resolved outside the suite's home: {path}"

    def test_an_exported_calibration_path_does_not_reach_the_suite(self) -> None:
        """A shell that exports one of these points a real file somewhere the home
        redirect does not cover, so the variables are dropped as well.

        Run in a subprocess, because the suite is what is under test here: by the
        time a test in this process runs, ``conftest`` has already dropped the
        variables, and asserting they are absent would pass on any machine that
        never exported them.
        """
        theirs = "/tmp/somebody-elses-calibration.json"
        env = dict(os.environ)
        env["PYTHONPATH"] = os.pathsep.join([str(REPO / "src"), str(SDK)])
        env["LITEGRIP_CALIB"] = theirs
        env["LITEGRIP_FACTORY_CALIB"] = theirs

        resolved = subprocess.run(
            [sys.executable, "-c",
             "import conftest;"
             " from litegrip_studio.calibration import default_user_path;"
             " print(default_user_path())"],
            cwd=HERE, env=env, capture_output=True, text=True, check=True,
        ).stdout.strip()

        assert theirs not in resolved
        assert resolved.startswith(str(Path(tempfile.gettempdir()) / "litegrip-studio-tests-")), (
            f"an exported LITEGRIP_CALIB reached the suite: {resolved}"
        )

    def test_a_rig_with_no_path_of_its_own_cannot_see_the_bench_file(self) -> None:
        """The defect, on the fixture that caused it: ``Rig`` names no file, so
        the file it ends up running on is whatever the default resolves to."""
        sim = Rig(connect=False).sim

        assert Path(sim.calibration_path) == default_sim_calibration_path()
        assert TEST_HOME in Path(sim.calibration_path).parents
