"""The command line: what the flags mean, and what they must not do.

The interesting claims here are negative ones.  ``--selftest`` must work where Qt
does not, which is only checkable in a process that has never imported Qt — so
one test runs a child interpreter and looks at its ``sys.modules``.  And the
console must not apply a calibration at startup: the file is loaded when the
worker connects, so the provenance check describes the file that actually
reached the SDK rather than whatever a command line said.
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import types
import textwrap
from pathlib import Path

import pytest

from litegrip_studio import cli, constants, version
from litegrip_studio.can_link import CanLink

REPO_ROOT = Path(__file__).resolve().parent.parent


class Recorder:
    """A backend stand-in that records what the factory did to it."""

    def __init__(self, **kwargs) -> None:
        self.kwargs = kwargs
        self.strokes: list[float] = []

    def set_travel_mm(self, max_stroke_mm: float) -> None:
        self.strokes.append(max_stroke_mm)

    def describe(self) -> str:
        return "recorder"


class FakeSettings:
    def __init__(self, calibration_path=None, travel_mm=120.0) -> None:
        self.calibration_path = calibration_path
        self.travel_mm = travel_mm


@pytest.fixture
def real_backend(monkeypatch):
    """Replace :class:`RealBackend` with a recorder, and the SDK with nothing.

    The point is the argument plumbing, without a CAN interface; what the real
    backend then does with those arguments has its own tests.
    """
    import litegrip_studio.backend.real as real

    monkeypatch.setattr(cli, "ensure_sdk", lambda: True)
    monkeypatch.setattr(real, "RealBackend", Recorder)
    return Recorder


@pytest.fixture
def sim_backend(monkeypatch):
    import litegrip_studio.backend.sim as sim

    monkeypatch.setattr(sim, "SimBackend", Recorder)
    return Recorder


def parse(*argv):
    return cli.build_parser().parse_args(list(argv))


class TestTheArguments:
    def test_it_defaults_to_the_real_gripper(self) -> None:
        args = parse()

        assert args.backend == "real"
        assert args.can_channel == "can0"
        assert args.selftest is False

    def test_the_can_ids_are_optional_so_the_file_can_supply_them(self) -> None:
        """``load_calibration`` overwrites can_id and mst_id from the file, so a
        default here would be a value that silently loses."""
        assert parse().can_id is None
        assert parse().mst_id is None

    def test_they_can_be_given(self) -> None:
        args = parse("--backend", "sim", "--can-id", "7", "--mst-id", "1")

        assert args.backend == "sim"
        assert (args.can_id, args.mst_id) == (7, 1)

    def test_an_unknown_backend_is_refused(self) -> None:
        with pytest.raises(SystemExit):
            parse("--backend", "hardware")

    def test_the_travel_mm_defaults_to_whatever_the_page_remembers(self) -> None:
        """None and not 120: the flag is an override, and the value stored by the
        calibration page is the one that should win when the flag is absent."""
        assert parse().travel_mm is None

    def test_help_names_the_default_stroke(self) -> None:
        assert f"{constants.DEFAULT_TRAVEL_MM:.0f}" in cli.build_parser().format_help()


class TestBuildingTheBackend:
    def test_the_simulator_is_built_with_no_sdk(self, sim_backend) -> None:
        backend = cli.make_backend(parse("--backend", "sim"))

        assert isinstance(backend, Recorder)
        assert backend.kwargs["realtime"] is True

    def test_the_channel_and_ids_reach_the_real_backend(self, real_backend) -> None:
        backend = cli.make_backend(parse("--can-channel", "can1", "--can-id", "3"))

        assert backend.kwargs["channel"] == "can1"
        assert backend.kwargs["can_id"] == 3

    def test_a_missing_sdk_is_refused_rather_than_traced_back(
        self, monkeypatch, real_backend
    ) -> None:
        monkeypatch.setattr(cli, "ensure_sdk", lambda: False)

        with pytest.raises(SystemExit):
            cli.make_backend(parse())

    def test_the_stroke_is_applied_to_whichever_backend_was_chosen(
        self, sim_backend
    ) -> None:
        backend = cli.make_backend(parse("--backend", "sim", "--travel-mm", "130"))

        assert backend.strokes == [130.0]

    def test_no_stroke_flag_leaves_the_backend_alone(self, sim_backend) -> None:
        """Because the calibration page has already set it, and re-applying the
        default here would undo that on every launch."""
        backend = cli.make_backend(parse("--backend", "sim"))

        assert backend.strokes == []

    def test_the_calibration_path_reaches_the_backend(self, sim_backend) -> None:
        backend = cli.make_backend(
            parse("--backend", "sim", "--calibration", "/tmp/cal.json")
        )

        assert backend.kwargs["calibration_path"] == "/tmp/cal.json"


class TestWhichInterfaceIsPrepared:
    """The CAN bring-up is wired here for the same reason the backend is: which
    bus this console talks to is decided in one place, and the worker stays a
    thing that talks to whatever it was handed."""

    def test_the_real_backend_gets_one(self) -> None:
        link = cli.make_can_link(parse())

        assert isinstance(link, CanLink)
        assert link.channel == constants.CAN_CHANNEL
        assert link.bitrate == constants.CAN_BITRATE

    def test_the_flags_reach_it(self) -> None:
        link = cli.make_can_link(
            parse("--can-channel", "can1", "--can-bitrate", "500000")
        )

        assert (link.channel, link.bitrate) == ("can1", 500_000)

    def test_the_simulator_gets_none(self) -> None:
        """There is no interface behind it, and a password dialog for a bus that
        does not exist would be theatre."""
        assert cli.make_can_link(parse("--backend", "sim")) is None

    def test_the_operator_can_turn_it_off(self) -> None:
        """For the console whose interface is raised by a unit file, or by hand:
        the prep is a convenience, and a convenience nobody asked for is a
        dialog on every connect."""
        assert cli.make_can_link(parse("--no-can-setup")) is None

    def test_it_is_handed_to_the_worker_rather_than_constructed_inside_it(self) -> None:
        """A source-level assertion, because the claim is about where a decision
        lives: the worker must not know how a bus is chosen, only that it was
        given one to prepare."""
        source = Path(cli.__file__).read_text(encoding="utf-8")
        worker = Path(cli.__file__).parent.joinpath("core/worker.py").read_text(
            encoding="utf-8"
        )

        assert "can_link=make_can_link(args)" in source
        assert "import subprocess" not in worker
        assert "pkexec" not in worker


class TestWhichCalibrationFileIsChosen:
    """Nothing is loaded here; this only decides which file the worker will load
    when it connects, and that choice has one asymmetric rule in it."""

    def test_the_flag_wins(self) -> None:
        args = parse("--backend", "real", "--calibration", "/tmp/now.json")

        assert cli.resolve_calibration_path(
            args, FakeSettings(calibration_path="/tmp/earlier.json")
        ) == "/tmp/now.json"

    def test_the_real_backend_falls_back_to_the_remembered_path(self) -> None:
        args = parse("--backend", "real")

        assert cli.resolve_calibration_path(
            args, FakeSettings(calibration_path="/tmp/earlier.json")
        ) == "/tmp/earlier.json"

    def test_the_simulator_ignores_the_remembered_path(self) -> None:
        """It is the bench gripper's file.  Letting a simulator run start from it
        is how a simulated calibration ends up written over the real one."""
        args = parse("--backend", "sim")

        assert cli.resolve_calibration_path(
            args, FakeSettings(calibration_path="/tmp/bench.json")
        ) is None

    def test_an_explicit_path_is_honoured_even_for_the_simulator(self) -> None:
        args = parse("--backend", "sim", "--calibration", "/tmp/sim.json")

        assert cli.resolve_calibration_path(args, FakeSettings()) == "/tmp/sim.json"


class TestStartupNeverAppliesACalibration:
    def test_the_source_has_no_load_or_save_command(self) -> None:
        """A source-level assertion, because the claim is about absence: the
        worker loads on connect, and a console that also loaded at startup would
        have two loads with one provenance check between them."""
        source = Path(cli.__file__).read_text(encoding="utf-8")

        assert "LoadCalibration" not in source
        assert "SaveCalibration" not in source


class TestSelftest:
    def test_it_runs_and_reports_success(self, capsys) -> None:
        assert cli.main(["--selftest"]) == 0

        assert "项通过" in capsys.readouterr().out

    def test_it_needs_no_qt(self) -> None:
        """The machine with a broken Qt is the machine this is for, so the check
        runs in a child interpreter that has never imported PyQt5 — a test in this
        process could only prove that Qt happens to be importable."""
        code = textwrap.dedent(
            """
            import sys
            from litegrip_studio import cli
            code = cli.main(["--selftest"])
            qt = sorted(m for m in sys.modules if "PyQt5" in m or "pyqtgraph" in m)
            print("QT:", qt)
            raise SystemExit(code)
            """
        )
        result = subprocess.run(
            [sys.executable, "-c", code],
            cwd=REPO_ROOT,
            # The child starts clean, so it gets the same path bootstrap the
            # repository's conftest gives this process.
            env={
                **os.environ,
                "PYTHONPATH": os.pathsep.join(
                    p for p in sys.path if p and Path(p).is_dir()
                ),
            },
            capture_output=True,
            text=True,
        )

        assert result.returncode == 0, result.stderr
        assert "QT: []" in result.stdout


class TestEnsureSdk:
    @staticmethod
    def _hide_the_sdk(monkeypatch) -> None:
        """Make ``import litegrip`` fail, however the SDK got on the path.

        Stripping ``sys.path`` is not enough to say "the SDK is not here": the
        SDK can be *installed*, and an editable install resolves it through a
        meta-path finder rather than through a ``sys.path`` entry, so the import
        would still succeed and ``ensure_sdk()`` would return before it looked at
        anything.  A ``None`` in ``sys.modules`` halts the import whatever the
        finders say, which is the condition these two tests are about.
        """
        monkeypatch.setitem(sys.modules, "litegrip", None)
        # A copy, so the path ``ensure_sdk()`` inserts is undone with the rest.
        monkeypatch.setattr(sys, "path", list(sys.path))

    def test_it_finds_the_checkout_on_the_path(self) -> None:
        assert cli.ensure_sdk() is True

    def test_it_reports_failure_and_says_where_it_looked(
        self, monkeypatch, tmp_path, capsys
    ) -> None:
        monkeypatch.setattr(cli, "DEFAULT_SDK_PATH", str(tmp_path))
        monkeypatch.delenv("LITEGRIP_SDK_PATH", raising=False)
        self._hide_the_sdk(monkeypatch)

        assert cli.ensure_sdk() is False
        assert str(tmp_path) in capsys.readouterr().err
        assert str(tmp_path) in sys.path

    def test_the_environment_overrides_the_built_in_path(
        self, monkeypatch, tmp_path, capsys
    ) -> None:
        elsewhere = tmp_path / "sdk-elsewhere"
        monkeypatch.setattr(cli, "DEFAULT_SDK_PATH", str(tmp_path / "nowhere"))
        monkeypatch.setenv("LITEGRIP_SDK_PATH", str(elsewhere))
        self._hide_the_sdk(monkeypatch)

        cli.ensure_sdk()

        assert str(elsewhere) in sys.path


class TestTheSignalHandler:
    def test_it_quits_the_application(self) -> None:
        class App:
            def __init__(self) -> None:
                self.quits = 0

            def quit(self) -> None:
                self.quits += 1

        app = App()
        cli.quit_handler(app)(signal.SIGINT, None)

        assert app.quits == 1

    def test_installing_it_claims_both_signals(self, qapp) -> None:
        """Saved and put back by hand: ``monkeypatch`` restores attributes, and a
        signal handler is not an attribute."""
        saved = {
            sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)
        }
        try:
            cli.install_signal_handlers(qapp)

            for sig in (signal.SIGINT, signal.SIGTERM):
                assert signal.getsignal(sig) is not signal.SIG_DFL
        finally:
            for sig, handler in saved.items():
                signal.signal(sig, handler)


class TestTheVersion:
    def test_a_source_run_says_so_rather_than_inventing_a_number(self) -> None:
        assert version.resolve_version() == version.BASE_VERSION + version.SOURCE_SUFFIX

    def test_a_build_stamp_wins(self, monkeypatch) -> None:
        """``build.sh`` writes ``_version.py`` next to the package, and a built
        artifact must report the build rather than the base."""
        stamped = types.ModuleType("litegrip_studio._version")
        stamped.__version__ = "0.1.0.42+gdeadbee.20260924"
        monkeypatch.setitem(sys.modules, "litegrip_studio._version", stamped)

        assert version.resolve_version() == "0.1.0.42+gdeadbee.20260924"

    def test_the_flag_prints_it_and_exits(self, capsys) -> None:
        with pytest.raises(SystemExit) as exit_info:
            cli.build_parser().parse_args(["--version"])

        assert exit_info.value.code == 0
        assert version.resolve_version() in capsys.readouterr().out
