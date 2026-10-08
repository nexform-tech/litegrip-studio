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

#: The SDK that ships with this repository, as opposed to one somewhere else.
VENDORED_SDK = REPO_ROOT / "src" / "litegrip"


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
    def __init__(self, calibration_path=None) -> None:
        self.calibration_path = calibration_path


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

    def test_the_help_offers_no_way_to_change_the_travel(self) -> None:
        """The travel is a property of the bench, not a command-line choice: a
        console told the wrong one reports every millimetre wrong, so the hole is
        closed here rather than bounded."""
        assert "--travel-mm" not in cli.build_parser().format_help()


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

    def test_the_backend_is_left_at_the_measured_travel(self, sim_backend) -> None:
        """Nothing on the command line can move it, so the backend keeps whatever
        the stack gave it — which is :data:`constants.DEFAULT_TRAVEL_MM`."""
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

    def test_the_source_applies_nothing_either(self) -> None:
        """Applying writes a file and opens the gate, and both are answers to a
        question only the operator can be asked — the startup path has nobody to
        ask, so it must not reach for either.  Refreshing is excluded for the
        same reason: there is nothing in force to re-read before a connection."""
        source = Path(cli.__file__).read_text(encoding="utf-8")

        assert "ApplyCalibration" not in source
        assert "DiscardCalibration" not in source

    def test_the_path_it_resolves_is_only_ever_handed_to_the_backend(self) -> None:
        """The remembered path is a *request*, not a load: it reaches the worker
        as the path the backend is constructed with, and the load happens on
        connect — after which the file is checked and the gate reports it."""
        args = parse("--backend", "real")
        settings = FakeSettings(calibration_path="/tmp/bench.json")

        assert cli.resolve_calibration_path(args, settings) == "/tmp/bench.json"
        assert not hasattr(cli, "load_calibration")


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
    def test_it_finds_the_vendored_sdk_with_nothing_set(
        self, monkeypatch, capsys
    ) -> None:
        """The whole point of vendoring: no environment, no install, it imports."""
        monkeypatch.delenv("LITEGRIP_SDK_PATH", raising=False)

        assert cli.ensure_sdk() is True
        assert not capsys.readouterr().err

        import litegrip

        # The one under this repository's own src/, not some installed copy that
        # happens to be found first.
        assert Path(litegrip.__file__).resolve().is_relative_to(VENDORED_SDK)

    def test_a_set_path_that_holds_no_sdk_is_reported(
        self, monkeypatch, tmp_path, capsys
    ) -> None:
        """An override set on purpose and wrong is a mistake, not a fallback.

        It does not quietly keep the vendored copy: the operator asked for that
        checkout, and finding out later — on the bench — is worse than not
        starting.
        """
        monkeypatch.setenv("LITEGRIP_SDK_PATH", str(tmp_path))

        assert cli.ensure_sdk() is False
        assert str(tmp_path) in capsys.readouterr().err

    def test_a_set_path_comes_ahead_of_the_vendored_copy(
        self, monkeypatch, tmp_path, capsys
    ) -> None:
        elsewhere = tmp_path / "sdk-elsewhere"
        (elsewhere / "litegrip").mkdir(parents=True)
        (elsewhere / "litegrip" / "__init__.py").write_text(".", encoding="utf-8")
        monkeypatch.setenv("LITEGRIP_SDK_PATH", str(elsewhere))
        monkeypatch.setattr(sys, "path", list(sys.path))

        assert cli.ensure_sdk() is True

        # First, not merely present: `src` is already on the path, so anything
        # later would still import the vendored copy and the override would be a
        # silent no-op. tests/test_vendored_sdk.py proves the import follows.
        assert sys.path[0] == str(elsewhere)
        assert not capsys.readouterr().err

    def test_a_source_tree_layout_is_accepted_too(
        self, monkeypatch, tmp_path, capsys
    ) -> None:
        """`<repo>` or `<repo>/src`, whichever holds the package.

        The SDK upstream uses the `src/` layout; pointing the override at the
        checkout root is the obvious mistake, and accepting it costs one `is_file`.
        """
        root = tmp_path / "sdk-checkout"
        (root / "src" / "litegrip").mkdir(parents=True)
        (root / "src" / "litegrip" / "__init__.py").write_text(".", encoding="utf-8")
        monkeypatch.setenv("LITEGRIP_SDK_PATH", str(root))
        monkeypatch.setattr(sys, "path", list(sys.path))

        assert cli.ensure_sdk() is True

        assert sys.path[0] == str(root / "src")
        assert not capsys.readouterr().err


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

    def test_the_base_cannot_be_mistaken_for_a_release(self) -> None:
        """``0.1.0+source`` was what a checkout reported, and ``v0.1.0`` is a real
        tag in this repository — so the current source read as the first release
        to anyone comparing versions across machines, which is how "the other
        computers are running an old version" gets reported.

        ``0.0.0`` is the "no release yet" placeholder, the same one
        ``pyproject.toml`` carries: semantic-release never writes a ``v0.0.0``
        tag, so this cannot collide with one.
        """
        assert version.BASE_VERSION == "0.0.0"

    def test_a_build_stamp_wins(self, monkeypatch) -> None:
        """``build.sh`` writes ``_version.py`` next to the package, and a built
        artifact must report the build rather than the base."""
        stamped = types.ModuleType("litegrip_studio._version")
        stamped.__version__ = "0.9.3.42+gdeadbee.20260924"
        monkeypatch.setitem(sys.modules, "litegrip_studio._version", stamped)

        assert version.resolve_version() == "0.9.3.42+gdeadbee.20260924"

    def test_the_flag_prints_it_and_exits(self, capsys) -> None:
        with pytest.raises(SystemExit) as exit_info:
            cli.build_parser().parse_args(["--version"])

        assert exit_info.value.code == 0
        assert version.resolve_version() in capsys.readouterr().out
