"""The self-test, and the claim that it is worth running.

A smoke test that cannot fail is worse than no smoke test, so the checks
themselves are pinned here: each one is shown to be non-vacuous by defeating the
thing it exists to catch.  That is the only property that matters about a check —
whether it passes on a healthy machine is not interesting.
"""

from __future__ import annotations

import ast
import dataclasses
import io
from pathlib import Path

import pytest

from litegrip_studio import constants, selftest
from litegrip_studio.units import Limits


def run(stream=None):
    stream = stream if stream is not None else io.StringIO()
    code = selftest.run(stream)
    return code, stream.getvalue()


class TestTheReport:
    def test_a_healthy_machine_passes_every_check(self) -> None:
        code, text = run()

        assert code == 0, text
        assert "FAIL" not in text
        assert text.count("PASS") == len(selftest.CHECKS)

    def test_it_counts_what_it_ran(self) -> None:
        _code, text = run()

        assert f"{len(selftest.CHECKS)}/{len(selftest.CHECKS)} 项通过" in text

    def test_a_failing_check_is_reported_and_changes_the_exit_code(
        self, monkeypatch
    ) -> None:
        """And does not stop the ones after it: a self-test is read by someone
        looking for the whole picture, not the first problem."""

        def explode() -> None:
            raise RuntimeError("boom")

        monkeypatch.setattr(
            selftest,
            "CHECKS",
            (
                selftest.Check("先通过的一项", lambda: None),
                selftest.Check("炸掉的一项", explode),
                selftest.Check("后面的一项", lambda: None),
            ),
        )

        code, text = run()

        assert code == 1
        assert "FAIL  炸掉的一项" in text
        assert "PASS  后面的一项" in text
        assert "RuntimeError: boom" in text
        assert "2/3 项通过" in text

    def test_the_names_are_distinct(self) -> None:
        """Two checks with one name would make a FAIL line ambiguous."""
        names = [check.name for check in selftest.CHECKS]

        assert len(set(names)) == len(names)

    def test_it_writes_where_it_is_told(self) -> None:
        stream = io.StringIO()

        selftest.run(stream)

        assert stream.getvalue().startswith("LiteGrip 控制台自检")


class TestTheChecksAreNotVacuous:
    """Each check is defeated, and must notice."""

    def test_the_direction_check_fails_if_the_reversal_rule_is_defeated(
        self, monkeypatch
    ) -> None:
        """The uncalibrated default is the reason the console has a gate at all;
        a check that cannot see it reversed is a check that would wave it
        through."""
        monkeypatch.setattr(Limits, "is_reversed", property(lambda self: False))

        with pytest.raises(AssertionError, match="未标定"):
            selftest.check_direction_is_caught()

    def test_the_travel_check_fails_when_the_file_scale_is_used(
        self, monkeypatch
    ) -> None:
        """Defeated by putting the file's own millimetres per rad back, which is
        what the SDK does and what the check exists to catch: the recorded
        extremes would then read as the file's 120 mm nominal stroke, and the
        slider would cover the middle of a travel whose ends nobody has seen."""
        from litegrip_studio import calibration, units

        monkeypatch.setattr(
            calibration,
            "derive_scale",
            lambda travel_rad, travel_mm=0.0, inset_mm=0.0: 65.21,
        )
        monkeypatch.setattr(
            units, "derive_scale", lambda travel_rad, travel_mm=0.0, inset_mm=0.0: 65.21
        )

        with pytest.raises(AssertionError, match="记录跨度"):
            selftest.check_the_travel_is_derived_from_the_recorded_angles()

    def test_the_travel_check_fails_when_the_inset_is_not_what_is_applied(
        self, monkeypatch
    ) -> None:
        """And defeated the other way: the console's inset and the one its scale
        was derived with disagreeing.  The span the check computes then cannot be
        the one the recorded angles convert to, which is the shape of a console
        that has stopped applying the inset altogether — the jaws would press the
        hard stop at the end of every full open."""
        monkeypatch.setattr(constants, "SPAN_INSET_MM", 40.0)

        with pytest.raises(AssertionError, match="记录跨度"):
            selftest.check_the_travel_is_derived_from_the_recorded_angles()

    def test_the_can_probe_check_fails_when_the_expected_bitrate_is_off(
        self, monkeypatch
    ) -> None:
        """Defeated by making the console expect a different bus speed, which is
        the state the operator is in when they have set the adapter to 500 kbit
        and the console still believes 1 Mbit: it must notice, because the
        alternative is a message claiming the interface is fine."""
        monkeypatch.setattr(constants, "CAN_BITRATE", 500_000)

        with pytest.raises(AssertionError, match="被判成需要改动"):
            selftest.check_the_can_probe_is_read_correctly()

    def test_the_estop_check_fails_on_a_motor_left_enabled(self, monkeypatch) -> None:
        """It asserts on the reading, so defeating the reading must break it."""
        from litegrip_studio.backend import sim as sim_module

        monkeypatch.setattr(sim_module.SimBackend, "disable", lambda self: None)

        with pytest.raises(AssertionError, match="仍是使能"):
            selftest.check_the_estop_zeroes_the_torque()

    def test_the_arrival_check_reads_the_measurement_not_the_plan(
        self, monkeypatch
    ) -> None:
        """The one check that is the whole console in miniature.  It is defeated
        by moving the *reading*, not the target: a check that compared the target
        against itself would pass with the gripper stuck at either end."""
        from litegrip_studio.backend import sim as sim_module

        measured = sim_module.SimBackend.read

        def elsewhere(self):
            telemetry = measured(self)
            return dataclasses.replace(
                telemetry, position_mm=telemetry.position_mm + 5.0
            )

        monkeypatch.setattr(sim_module.SimBackend, "read", elsewhere)

        with pytest.raises(AssertionError, match="停在"):
            selftest.check_a_move_arrives_and_stops()


class TestItNeedsNoQt:
    def test_the_module_imports_nothing_from_qt(self) -> None:
        """Structural, and read from the source rather than from ``sys.modules``:
        by the time this test runs, other tests have imported Qt, so looking at
        the module table here could only ever prove that Qt is importable."""
        tree = ast.parse(Path(selftest.__file__).read_text(encoding="utf-8"))

        imported: list[str] = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported += [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported.append(node.module)

        assert not [name for name in imported if "PyQt" in name or "pyqtgraph" in name]
