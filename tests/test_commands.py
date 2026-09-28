"""The command set and its coalescing.

The coalescing rule is small enough to look obviously correct, and it is the
part of the console that decides which of the operator's intentions survives a
burst — so it is tested against the sequences that a rule written carelessly
would get wrong, not just against a burst of identical ones.
"""

from __future__ import annotations

import dataclasses

import pytest

from litegrip_studio.core import commands
from litegrip_studio.core.commands import (
    Close,
    ConfirmProbeLimit,
    Connect,
    Grasp,
    Heartbeat,
    Inject,
    LoadCalibration,
    MoveToMm,
    Open,
    Release,
    SaveCalibration,
    SetForce,
    SetTravel,
    SetSpeed,
    StartGuidedCalibration,
    StartManualCalibration,
    Stop,
)


class TestFrozen:
    @pytest.mark.parametrize(
        "cmd",
        [
            Connect(),
            MoveToMm(50.0),
            Close(force_n=12.0),
            Grasp(),
            SetSpeed(80.0),
            LoadCalibration("/tmp/x.json"),
            Inject({"obj_mm": 40.0}),
        ],
    )
    def test_a_command_cannot_be_mutated_after_it_is_queued(self, cmd) -> None:
        """The widget it was built from may have moved on by the time the worker
        drains it; the worker has to act on what was asked when it was asked."""
        with pytest.raises(dataclasses.FrozenInstanceError):
            cmd.source = "something else"  # type: ignore[misc]

    def test_every_command_is_a_command(self) -> None:
        for name in commands.AnyCommand.__args__:  # type: ignore[attr-defined]
            assert issubclass(name, commands.Command)

    def test_the_union_covers_every_command_class(self) -> None:
        """Otherwise a command could be added, posted, and silently never
        matched by the worker's dispatch."""
        declared = {
            obj
            for obj in vars(commands).values()
            if isinstance(obj, type)
            and issubclass(obj, commands.Command)
            and obj is not commands.Command
        }
        assert declared == set(commands.AnyCommand.__args__)  # type: ignore[attr-defined]


class TestDescribe:
    """Every command is logged, so every command needs a line that reads."""

    @pytest.mark.parametrize(
        "cmd",
        [
            Connect(),
            Open(),
            Close(),
            Close(force_n=12.0),
            Grasp(force_n=20.0),
            MoveToMm(62.4, source="slider"),
            Stop(),
            Release(),
            SetSpeed(80.0),
            SetForce(15.0),
            SetTravel(200.0),
            LoadCalibration(),
            LoadCalibration("/tmp/x.json"),
            SaveCalibration(),
            StartGuidedCalibration(),
            ConfirmProbeLimit(),
            StartManualCalibration(),
            StartManualCalibration(30.0),
            Heartbeat(),
            Inject({"uv": True, "obj_mm": 40.0}),
        ],
    )
    def test_it_is_non_empty_and_is_not_the_class_name(self, cmd) -> None:
        text = cmd.describe()
        assert text
        assert text != type(cmd).__name__

    def test_a_requested_cap_is_visible_in_the_line(self) -> None:
        """A close that was capped at 12 N and one that was not look identical
        in a log otherwise, and the cap is the safety-relevant part."""
        assert "12.0 N" in Close(force_n=12.0).describe()
        assert "N" not in Close().describe()

    def test_the_source_is_carried_into_the_line(self) -> None:
        """An unexpected motion is only diagnosable if the log says who asked."""
        assert "slider" in MoveToMm(10.0, source="slider").describe()
        assert "button" in MoveToMm(10.0, source="button").describe()

    def test_a_bare_load_says_it_is_the_default_path(self) -> None:
        assert "默认路径" in LoadCalibration().describe()

    def test_the_injection_reports_its_values(self) -> None:
        text = Inject({"uv": True, "obj_mm": 40.0}).describe()
        assert "uv=True" in text and "obj_mm=40.0" in text


class TestCoalesce:
    def test_a_burst_of_moves_collapses_to_the_last(self) -> None:
        """Fifty drag events between two ticks describe one position: where the
        operator's finger ended up."""
        burst = [MoveToMm(float(mm)) for mm in range(50)]
        assert commands.coalesce(burst) == [MoveToMm(49.0)]

    def test_a_burst_of_speeds_collapses_to_the_last(self) -> None:
        assert commands.coalesce([SetSpeed(10.0), SetSpeed(20.0), SetSpeed(30.0)]) == [
            SetSpeed(30.0)
        ]

    @pytest.mark.parametrize(
        "kind,args",
        [
            (MoveToMm, (10.0,)),
            (SetSpeed, (50.0,)),
            (SetForce, (10.0,)),
            (SetTravel, (120.0,)),
            (Heartbeat, ()),
        ],
    )
    def test_every_declared_coalescable_kind_actually_coalesces(
        self, kind, args: tuple
    ) -> None:
        """Guards the list itself: a kind added there but not handled would
        quietly stop collapsing."""
        assert len(commands.coalesce([kind(*args), kind(*args)])) == 1

    def test_the_coalescable_list_names_only_real_commands(self) -> None:
        for kind in commands.COALESCABLE:
            assert issubclass(kind, commands.Command)

    def test_commands_of_different_kinds_are_all_kept(self) -> None:
        """Each one does something the others do not."""
        seq = [Connect(), MoveToMm(10.0), SetSpeed(50.0)]
        assert commands.coalesce(seq) == seq

    def test_a_move_after_a_button_press_is_not_folded_into_it(self) -> None:
        seq = [Open(), MoveToMm(10.0)]
        assert commands.coalesce(seq) == seq

    def test_a_separated_run_is_not_folded_across_the_gap(self) -> None:
        """``SetSpeed(10), MoveTo(50), SetSpeed(80)`` is a move that starts at
        10 mm/s and speeds up.  Folding the speeds together would silently pick
        one of them and change what the sequence means."""
        seq = [SetSpeed(10.0), MoveToMm(50.0), SetSpeed(80.0)]
        assert commands.coalesce(seq) == seq

    def test_the_same_kind_reappearing_after_another_is_a_new_run(self) -> None:
        seq = [MoveToMm(1.0), Stop(), MoveToMm(2.0)]
        assert commands.coalesce(seq) == seq

    def test_a_run_after_a_gap_still_collapses(self) -> None:
        seq = [MoveToMm(1.0), Stop(), MoveToMm(2.0), MoveToMm(3.0), MoveToMm(4.0)]
        assert commands.coalesce(seq) == [MoveToMm(1.0), Stop(), MoveToMm(4.0)]

    def test_nothing_in_means_nothing_out(self) -> None:
        assert commands.coalesce([]) == []

    def test_a_non_coalescable_command_is_never_dropped(self) -> None:
        """A calibration step or a fault clear has an effect that a later one of
        the same kind does not supersede."""
        seq = [
            StartGuidedCalibration(),
            ConfirmProbeLimit(),
            ConfirmProbeLimit(),
            ConfirmProbeLimit(),
        ]
        assert commands.coalesce(seq) == seq

    def test_ordering_is_preserved(self) -> None:
        seq = [SetSpeed(10.0), SetSpeed(20.0), MoveToMm(5.0), Release()]
        assert commands.coalesce(seq) == [SetSpeed(20.0), MoveToMm(5.0), Release()]

    def test_the_result_is_a_subsequence_never_a_superset(self) -> None:
        """Coalescing may drop commands; it must never invent one."""
        seq = [MoveToMm(1.0), SetSpeed(5.0), MoveToMm(2.0), MoveToMm(3.0), Heartbeat()]
        out = commands.coalesce(seq)
        assert [type(c) for c in out] == [MoveToMm, SetSpeed, MoveToMm, Heartbeat]
        assert out[2] == MoveToMm(3.0)
