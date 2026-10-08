"""Calibration provenance, validation, and the two defences with no backstop.

Deliberately imports nothing from :mod:`litegrip_studio.backend` — the module
under test is pure, and keeping it that way is what lets this file run before
any transport exists.

Two failures are pinned here harder than the rest, because neither has another
line of defence:

*The silent fallback.*  The SDK's ``load_calibration`` reads the user file and
then the factory file, and returns ``True`` for both.  A console that only
checks the return value cannot tell them apart, so a factory file belonging to
a different gripper silently makes every mm and every force wrong.  The tests
below assert that an unusable user file stays *unusable* rather than becoming a
factory one, and that the SDK is only ever handed a path we have already read.

*The direction reversal.*  The SDK's uncalibrated defaults have ``closed``
numerically below ``open``, which is the ordering a reverse-mounted gripper has
as well — the two are indistinguishable from the angles alone, and a check that
refuses the first makes the second impossible to calibrate.  So the ordering is
reported and the operator is asked to confirm it; what refuses a file is the
``frame_mismatch`` against the live encoder reading, which is tested in
``test_units.py``.
"""

from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path

import litegrip
import pytest

from litegrip_studio import calibration, constants
from litegrip_studio.calibration import (
    PROVENANCE_FACTORY,
    PROVENANCE_INVALID,
    PROVENANCE_MEMORY,
    PROVENANCE_MISSING,
    PROVENANCE_USER,
)
from litegrip_studio.units import Limits

from conftest import REPO_ROOT, shipped_factory_calibrations

# The three real datasets, copied from the SDK tree rather than invented, so a
# change in the SDK's shipped numbers shows up as a test failure here.
#
# ``FACTORY_RAW`` is the SDK's ``factory_calibration.json``, verbatim, including
# ``work_stroke_mm`` — a key this console does not read and must still carry, or
# the comparison below would be against a file of its own invention.
#
# Its ``rad_to_mm`` is upstream's scale for the unit that file was recorded on:
# 1.651026 rad of span times 52.69 is 87 mm.  It is deliberately *not* the scale
# the console moves by — that one is derived from the travel the operator
# measured, and over this span ``DEFAULT_TRAVEL_MM`` gives 52.0888, about 1.2 %
# away.  The derivation is the whole point of the console's limits, so a fixture
# that assumed the file agreed with it would be pinning the wrong number.
# ``TestTheFixtureIsTheFileItClaims`` below compares this against the file the
# repository ships.
FACTORY_RAW = {
    "channel": "can0",
    "can_id": 8,
    "mst_id": 24,
    "canfd_mode": False,
    "zero_position_rad": 0.0,
    "max_position_rad": -1.651026,
    "travel_range_rad": 1.651026,
    "rad_to_mm": 52.69,
    "work_stroke_mm": 80.0,
    "motor_type": "DM4310",
    "kp": 5.0,
    "kd": 2.0,
    "grasp_torque_threshold": 0.5,
}

USER_RAW = {
    "channel": "can0",
    "can_id": 8,
    "mst_id": 18,
    "canfd_mode": False,
    "zero_position_rad": 1.775959,
    "max_position_rad": -0.064279,
    "travel_range_rad": 1.840238,
    "rad_to_mm": 65.21,
    "motor_type": "DM4310",
    "kp": 100.0,
    "kd": 2.0,
    "grasp_torque_threshold": 0.5,
}

# ``GripperConfig``'s untouched defaults: closed at 0.0, open at 1.14.  This is
# what the SDK uses when nothing has been loaded.  Read as a file it describes a
# reverse-mounted gripper and validates; what it cannot survive is the encoder,
# because this bench's readings are nowhere inside that range.
UNCALIBRATED_RAW = {
    "zero_position_rad": 0.0,
    "max_position_rad": 1.14,
    "rad_to_mm": 65.21,
}

#: A file that is unusable for a reason no operator could mistake for a mounting
#: direction: both extremes are the same angle, so the travel is zero and the
#: scale derived from it is zero, which collapses every target to one angle.
ZERO_TRAVEL_RAW = dict(USER_RAW, max_position_rad=USER_RAW["zero_position_rad"])

#: This bench's own recorded extremes, with the closed stop at the *smaller*
#: angle — what the two-point capture writes on a reverse-mounted unit.  The
#: file's scale agrees with the one derived from the travel, so the only thing
#: distinguishing it from ``USER_RAW`` is the ordering.
REVERSED_RAW = dict(
    USER_RAW,
    zero_position_rad=-0.300793,
    max_position_rad=1.421569,
    travel_range_rad=1.722362,
    rad_to_mm=49.93,
)

#: The angular travel the user file records, used to place a travel setting
#: exactly on one of the plausibility-band edges.
TRAVEL_RAD = USER_RAW["zero_position_rad"] - USER_RAW["max_position_rad"]


def _write(path: Path, data: object) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    if isinstance(data, str):
        path.write_text(data, encoding="utf-8")
    else:
        path.write_text(json.dumps(data), encoding="utf-8")
    return path


@pytest.fixture
def env(tmp_path, monkeypatch):
    """An isolated calibration environment.

    Both lookups are redirected into ``tmp_path``, so ``resolve()`` can be tested
    through its real defaults — the environment variables are how the paths are
    overridden in production, and testing around them would leave the override
    itself unexercised.
    """
    user = tmp_path / "user.json"
    factory = tmp_path / "factory.json"
    monkeypatch.setenv("LITEGRIP_CALIB", str(user))
    monkeypatch.setenv("LITEGRIP_FACTORY_CALIB", str(factory))
    return type("Env", (), {"user": user, "factory": factory, "dir": tmp_path})()


# ── schema ──────────────────────────────────────────────────────────────────
class TestSchemaValidation:
    def test_a_valid_file_parses_clean(self) -> None:
        raw, problems = calibration.parse_calibration_json(json.dumps(USER_RAW))
        assert problems == []
        assert raw == USER_RAW

    def test_broken_json_reports_a_problem_rather_than_raising(self) -> None:
        raw, problems = calibration.parse_calibration_json("{not json,")
        assert raw is None
        assert len(problems) == 1
        assert "JSON" in problems[0]

    def test_a_bare_string_is_not_json(self) -> None:
        # ``json.loads`` accepts this, and the SDK would then index it unguarded.
        raw, problems = calibration.parse_calibration_json('"hello"')
        assert raw is None
        assert "顶层应为对象" in problems[0]

    def test_a_json_array_is_not_a_calibration(self) -> None:
        raw, problems = calibration.parse_calibration_json("[1, 2, 3]")
        assert raw is None
        assert "顶层应为对象" in problems[0]

    @pytest.mark.parametrize("missing", calibration.REQUIRED_KEYS)
    def test_a_missing_required_key_is_named_not_raised(self, missing: str) -> None:
        """The SDK indexes these three unguarded (gripper.py:745), so a file
        missing one raises ``KeyError`` from inside the SDK — with no way to
        tell which file, or why."""
        data = dict(USER_RAW)
        del data[missing]
        raw, problems = calibration.parse_calibration_json(json.dumps(data))
        assert raw == data, "the parsed data is still returned, for the UI to show"
        assert any(missing in p for p in problems)

    @pytest.mark.parametrize("key", calibration.REQUIRED_KEYS)
    @pytest.mark.parametrize("bad", ["abc", None, [1], {"a": 1}])
    def test_a_non_numeric_value_is_named(self, key: str, bad: object) -> None:
        data = dict(USER_RAW)
        data[key] = bad
        _, problems = calibration.parse_calibration_json(json.dumps(data))
        assert any(key in p for p in problems)

    def test_a_numeric_string_is_accepted(self) -> None:
        """The SDK calls ``float()``, so a quoted number works there too."""
        data = dict(USER_RAW)
        data["rad_to_mm"] = "65.21"
        _, problems = calibration.parse_calibration_json(json.dumps(data))
        assert problems == []

    def test_extra_keys_are_ignored(self) -> None:
        data = dict(USER_RAW, future_field=1)
        raw, problems = calibration.parse_calibration_json(json.dumps(data))
        assert problems == []
        assert raw["future_field"] == 1


# ── provenance ──────────────────────────────────────────────────────────────
class TestProvenance:
    def test_a_user_file_is_the_user_calibration(self, env) -> None:
        _write(env.user, USER_RAW)
        _write(env.factory, FACTORY_RAW)
        info = calibration.resolve()
        assert info.provenance == PROVENANCE_USER
        assert info.usable and info.is_user and not info.is_factory
        assert info.path == str(env.user)
        assert info.limits is not None
        assert info.limits.closed_rad == pytest.approx(1.775959)
        # The scale is derived from the measured travel, not read from the file,
        # so the recorded extremes span the travel plus the inset and the
        # commanded range is the travel.
        assert info.limits.max_stroke_mm == pytest.approx(constants.DEFAULT_TRAVEL_MM)
        assert info.limits.stroke_mm == pytest.approx(
            constants.DEFAULT_TRAVEL_MM + constants.SPAN_INSET_MM, abs=0.01
        )

    def test_an_absent_user_file_falls_back_to_factory_and_says_so(self, env) -> None:
        _write(env.factory, FACTORY_RAW)
        info = calibration.resolve()
        assert info.provenance == PROVENANCE_FACTORY
        assert info.usable and info.is_factory
        assert info.path == str(env.factory)
        joined = " ".join(info.warnings)
        assert "未找到用户标定文件" in joined
        assert "同一台" in joined, "the operator must be told the risk, not just the fact"

    def test_nothing_anywhere_is_missing_and_blocks(self, env) -> None:
        info = calibration.resolve()
        assert info.provenance == PROVENANCE_MISSING
        assert not info.usable
        assert info.limits is None
        assert info.path is None
        assert "请先执行标定" in " ".join(info.problems)

    def test_an_unusable_user_file_does_not_fall_back_to_factory(self, env) -> None:
        """The single most important behaviour in this module.

        A corrupt user file means *this gripper is not calibrated*.  The SDK
        would fall through to the factory file and report success; doing the
        same here would put a nominal unit's numbers behind a real machine's
        readings, with nothing on screen to show it.
        """
        _write(env.user, "{ truncated")
        _write(env.factory, FACTORY_RAW)
        info = calibration.resolve()
        assert info.provenance == PROVENANCE_INVALID
        assert not info.usable
        assert info.limits is None
        assert info.path == str(env.user), "the bad file, not the one we did not use"

    def test_an_unreadable_user_file_is_reported_as_such(self, env, monkeypatch) -> None:
        _write(env.user, USER_RAW)
        _write(env.factory, FACTORY_RAW)

        def boom(*args, **kwargs):
            raise OSError("permission denied")

        monkeypatch.setattr(Path, "read_text", boom)
        info = calibration.resolve()
        assert info.provenance == PROVENANCE_INVALID
        assert "无法读取" in " ".join(info.problems)

    def test_an_invalid_factory_file_is_reported_as_invalid(self, env) -> None:
        _write(env.factory, [1, 2])
        info = calibration.resolve()
        assert info.provenance == PROVENANCE_INVALID
        assert not info.usable

    def test_resolve_accepts_an_explicit_path(self, env) -> None:
        other = _write(env.dir / "other.json", USER_RAW)
        info = calibration.resolve(other)
        assert info.provenance == PROVENANCE_USER
        assert info.path == str(other)

    def test_an_explicit_path_that_does_not_exist_falls_back(self, env) -> None:
        _write(env.factory, FACTORY_RAW)
        info = calibration.resolve(env.dir / "nope.json")
        assert info.provenance == PROVENANCE_FACTORY


# ── the direction check ─────────────────────────────────────────────────────
class TestTheDirectionTheAnglesWereRecordedIn:
    """The ordering of the two angles is read, not judged and not asked about.

    ``closed <= open`` used to be a hard problem here: it is the signature of the
    SDK's uncalibrated defaults, and every conversion downstream assumes the
    ordering of the units on this bench.  A file whose two angles were recorded
    the other way round cannot be told apart from the defaults by the angles
    alone — so the check made such a file impossible to load, which is the
    deadlock this replaces.  The ordering is now simply read out of the pair, and
    the encoder has the final say, in ``frame_mismatch``.
    """

    def test_the_uncalibrated_defaults_are_loaded_and_read_as_given(self, env) -> None:
        _write(env.user, UNCALIBRATED_RAW)
        info = calibration.resolve()
        assert info.usable, "the angles are self-consistent; only the encoder can refute them"
        assert info.limits is not None and info.limits.direction == 1.0
        assert info.problems == ()
        assert not info.warnings, "the ordering is not something to warn about any more"

    def test_a_calibration_recorded_the_other_way_round_may_be_moved_under(self, env) -> None:
        """The whole point: after the two-point capture on a unit whose angle
        grows as the jaws open, the operator is not left with a dead console."""
        _write(env.user, dict(REVERSED_RAW))
        info = calibration.resolve()
        assert info.usable and info.motion_allowed
        assert info.limits is not None and info.limits.direction == 1.0

    def test_the_real_files_are_read_the_other_way_and_do_not_warn(self, env) -> None:
        _write(env.user, USER_RAW)
        info = calibration.resolve()
        assert info.limits is not None and info.limits.direction == -1.0
        assert not info.warnings

    def test_equal_angles_are_refused(self, env) -> None:
        """Zero travel is a problem whichever way it is read: the scale derived
        from it is zero, so every target clamps to one angle."""
        _write(env.user, ZERO_TRAVEL_RAW)
        info = calibration.resolve()
        assert not info.usable
        assert info.limits is None
        assert any("行程" in p for p in info.problems)

    @pytest.mark.parametrize(
        "raw", [FACTORY_RAW, USER_RAW], ids=["factory", "user"]
    )
    def test_both_real_calibrations_pass(self, env, raw: dict) -> None:
        _write(env.user, raw)
        info = calibration.resolve()
        assert info.provenance == PROVENANCE_USER
        assert info.problems == ()
        assert info.limits is not None
        assert info.limits.direction == -1.0


class TestMotionAllowed:
    """The predicate the worker's gate and the backend both consult.

    Strictly narrower than ``usable``, and every case where the two differ is a
    case where someone would otherwise be told the gripper is ready when it is
    not.
    """

    def test_a_user_calibration_opens_the_gate(self, env) -> None:
        _write(env.user, USER_RAW)
        info = calibration.resolve()
        assert info.usable and info.motion_allowed

    def test_the_factory_fallback_is_allowed_here_and_gated_by_the_worker(self, env) -> None:
        """This property is a statement about the file, so it says the file is
        usable.  Requiring the operator to accept the risk is the worker's job,
        because that is a decision rather than a fact."""
        _write(env.factory, FACTORY_RAW)
        info = calibration.resolve()
        assert info.motion_allowed
        assert not info.is_user, "it is still not this gripper's calibration"

    def test_an_unsaved_probe_result_does_not(self) -> None:
        """Nothing about it survives a restart, so a move planned against it
        cannot be reproduced."""
        info = calibration.in_memory(1.775959, -0.064279, 65.21)
        assert info.usable, "the numbers are still worth displaying"
        assert not info.motion_allowed

    def test_a_reverse_mounted_calibration_does(self, env) -> None:
        """The mounting direction is not a reason to refuse motion; it is a
        reason to say which way round the gripper is."""
        _write(env.user, REVERSED_RAW)
        info = calibration.resolve()
        assert info.usable and info.motion_allowed

    def test_a_zero_travel_calibration_does_not(self, env) -> None:
        _write(env.user, ZERO_TRAVEL_RAW)
        info = calibration.resolve()
        assert not info.usable and not info.motion_allowed

    def test_a_missing_calibration_does_not(self, env) -> None:
        assert not calibration.resolve().motion_allowed

    def test_a_calibration_that_failed_the_cross_check_does_not(self, env) -> None:
        """It loaded, it is self-consistent, and the hardware is not running on
        it.  The backend records the mismatch as problems for exactly this
        reason."""
        _write(env.user, USER_RAW)
        info = calibration.resolve()
        mismatch = calibration.cross_check(info, Limits(0.0, -1.651026, 52.69))
        assert mismatch
        assert not replace(info, problems=info.problems + tuple(mismatch)).motion_allowed

    @pytest.mark.parametrize(
        "provenance_allowed",
        [
            (PROVENANCE_USER, True),
            (PROVENANCE_FACTORY, True),
            (PROVENANCE_MEMORY, False),
            (PROVENANCE_INVALID, False),
            (PROVENANCE_MISSING, False),
        ],
    )
    def test_every_provenance_has_a_decided_answer(
        self, env, provenance_allowed: tuple[str, bool]
    ) -> None:
        """Exhaustive over the provenances so a new one cannot be added without
        deciding what it means for motion."""
        provenance, expected = provenance_allowed
        writers = {
            PROVENANCE_USER: lambda: _write(env.user, USER_RAW),
            PROVENANCE_FACTORY: lambda: _write(env.factory, FACTORY_RAW),
            PROVENANCE_MEMORY: lambda: None,
            PROVENANCE_INVALID: lambda: _write(env.user, ZERO_TRAVEL_RAW),
            PROVENANCE_MISSING: lambda: None,
        }
        writers[provenance]()

        if provenance == PROVENANCE_MEMORY:
            info = calibration.in_memory(1.775959, -0.064279, 65.21)
        else:
            info = calibration.resolve()
        assert info.provenance == provenance
        assert info.motion_allowed is expected


# ── numeric sanity ──────────────────────────────────────────────────────────
class TestNumericSanity:
    """The file's own ``rad_to_mm`` is evidence now, not a conversion factor.

    The scale in force is derived from the two angles and the measured travel,
    so an impossible coefficient in the file cannot make a millimetre reading
    wrong — it can only make the file unschedulable, which is a warning.  What
    blocks motion is a *travel* that cannot belong to these angles, because that
    is the number the millimetres are derived from.
    """

    @pytest.mark.parametrize("bad", [0.0, -65.21])
    def test_a_non_positive_rad_to_mm_in_the_file_is_ignored_with_a_warning(
        self, env, bad: float
    ) -> None:
        _write(env.user, dict(USER_RAW, rad_to_mm=bad))
        info = calibration.resolve()
        assert info.usable, "the readings come from the derived scale either way"
        assert any("rad_to_mm" in w for w in info.warnings)

    @pytest.mark.parametrize(
        "implied_mm", [constants.STROKE_MIN_MM / 2.0, constants.STROKE_MAX_MM * 2.0, 1000.0]
    )
    def test_an_implausible_stroke_in_the_file_warns_about_another_gripper(
        self, env, implied_mm: float
    ) -> None:
        """The file's own scale is judged by the stroke it implies, and only when
        that is not even the right order of magnitude — see the test below for
        why the comparison cannot be tighter."""
        _write(env.user, dict(USER_RAW, rad_to_mm=implied_mm / TRAVEL_RAD))
        info = calibration.resolve()
        assert info.usable, "the scale in force is derived; the file's is evidence only"
        assert any("另一台夹爪" in w for w in info.warnings)

    @pytest.mark.parametrize(
        "nominal_mm", [120.0, 80.0, 200.0]
    )
    def test_a_file_written_around_a_nominal_stroke_does_not_warn(
        self, env, nominal_mm: float
    ) -> None:
        """Every file the SDK or an older console wrote carries the *nominal*
        stroke it was configured with over the span it recorded — the operator's
        own file says 120 mm on an 85 mm gripper — so the implied stroke reports
        a setting rather than a measurement and reads the same for a 60 mm unit
        and a 120 mm unit alike.  Warning about it would mean warning about every
        file there is, including this gripper's own, which is how a warning
        becomes something the operator scrolls past."""
        _write(env.user, dict(USER_RAW, rad_to_mm=nominal_mm / TRAVEL_RAD))
        info = calibration.resolve()
        assert info.usable, info.problems
        assert not any("另一台夹爪" in w for w in info.warnings), info.warnings

    @pytest.mark.parametrize("travel_mm,ok", [(60.0, True), (300.0, True), (10.0, False), (400.0, False)])
    def test_a_travel_that_cannot_be_this_gripper_is_a_problem(
        self, env, travel_mm: float, ok: bool
    ) -> None:
        """The band on the derived scale is what catches a mistyped travel: with
        these angles, 85 mm is 46.7 mm/rad and 10 mm would be 6.0 mm/rad, which
        no gripper of this shape can be."""
        _write(env.user, USER_RAW)
        info = calibration.resolve(max_stroke_mm=travel_mm)
        assert info.usable is ok, info.problems
        if not ok:
            assert any("超出合理范围" in p for p in info.problems)

    def test_the_band_edges_are_inclusive(self, env) -> None:
        """Both comparisons are non-strict, and the edges are reachable: this
        unit's angles put the lower one at a travel of 54.2 mm."""
        edge = constants.RAD_TO_MM_MIN
        travel_mm = edge * TRAVEL_RAD - constants.SPAN_INSET_MM
        _write(env.user, USER_RAW)
        info = calibration.resolve(max_stroke_mm=travel_mm)
        assert info.usable, info.problems
        assert info.limits is not None
        assert info.limits.rad_to_mm == pytest.approx(edge)

    def test_zero_travel_is_a_problem(self, env) -> None:
        _write(env.user, dict(USER_RAW, zero_position_rad=0.5, max_position_rad=0.5))
        assert any("行程" in p for p in calibration.resolve().problems)

    def test_a_stroke_far_from_nominal_warns_but_does_not_block(self, env) -> None:
        """The signature of a factory file belonging to another gripper: the
        angles are self-consistent, so only the stroke gives it away."""
        _write(env.user, dict(USER_RAW, rad_to_mm=180.0, max_position_rad=-0.064279))
        info = calibration.resolve()
        assert info.usable, "a warning must not stop the operator working"
        assert any("另一台夹爪" in w for w in info.warnings)

    def test_a_file_the_console_wrote_does_not_warn_about_itself(self, env) -> None:
        """The round trip that has to be silent: probe → save → reload."""
        info = calibration.resolve(_write(env.dir / "a.json", USER_RAW))
        assert info.limits is not None
        target = _write(env.user, calibration.build_save_dict(info.limits))
        reloaded = calibration.resolve(target)
        assert reloaded.warnings == ()
        assert reloaded.usable

    def test_the_tolerance_is_measured_against_the_travel_not_a_constant(
        self, env
    ) -> None:
        """A 200 mm gripper calibrated at 200 mm must not be warned about."""
        _write(env.user, dict(USER_RAW, rad_to_mm=108.68))  # 1.840238 × 108.68 ≈ 200
        info = calibration.resolve(max_stroke_mm=200.0)
        assert info.usable
        assert not any("另一台夹爪" in w for w in info.warnings)
        assert info.max_stroke_mm == 200.0


# ── the silent-fallback defence ─────────────────────────────────────────────
class TestSilentFallbackDefence:
    """The SDK cannot tell us which file it loaded, so we never let it choose."""

    @pytest.mark.parametrize(
        "raw,provenance",
        [(USER_RAW, PROVENANCE_USER), (FACTORY_RAW, PROVENANCE_FACTORY)],
        ids=["user", "factory"],
    )
    def test_a_usable_calibration_yields_an_explicit_readable_path(
        self, env, raw: dict, provenance: str
    ) -> None:
        """Both are handed over as a concrete path — including the factory one.

        Passing ``None`` for the factory case would let the SDK reach its own
        fallback branch, where it would pick the file itself and give no signal
        that it had.  We name the file so the branch is never entered.
        """
        if provenance == PROVENANCE_FACTORY:
            _write(env.factory, raw)
        else:
            _write(env.user, raw)
        info = calibration.resolve()
        assert info.provenance == provenance

        handed = calibration.sdk_load_path(info)
        assert handed is not None
        assert Path(handed).is_file(), "never hand over a path we have not read"
        assert Path(handed).read_text(encoding="utf-8") == Path(info.path).read_text(
            encoding="utf-8"
        )

    @pytest.mark.parametrize(
        "setup",
        ["missing", "corrupt", "zero_travel", "memory"],
    )
    def test_no_path_is_handed_when_motion_must_not_proceed(self, env, setup: str) -> None:
        if setup == "corrupt":
            _write(env.user, "{ truncated")
        elif setup == "zero_travel":
            _write(env.user, ZERO_TRAVEL_RAW)
        elif setup == "memory":
            info = calibration.in_memory(1.775959, -0.064279, 65.21)
            assert info.provenance == PROVENANCE_MEMORY
            assert calibration.sdk_load_path(info) is None
            return
        info = calibration.resolve()
        assert not info.usable
        assert calibration.sdk_load_path(info) is None

    def test_a_memory_calibration_is_still_not_a_user_calibration(self) -> None:
        """A fresh probe result does not survive a restart, so it does not
        unlock the gate on its own."""
        info = calibration.in_memory(1.775959, -0.064279, 65.21)
        assert info.provenance == PROVENANCE_MEMORY
        assert info.is_user is False
        assert info.limits is not None, "the numbers are still usable for display"
        assert any("尚未保存" in w for w in info.warnings)

    def test_a_memory_calibration_of_a_probe_recorded_either_way_is_displayable(self) -> None:
        """Which way the two angles came out does not change the rule for an
        unsaved result: shown, and gated until it is written to a file."""
        info = calibration.in_memory(-0.300793, 1.421569, 49.93)
        assert info.usable
        assert info.limits is not None and info.limits.direction == 1.0
        assert not info.motion_allowed

    def test_a_memory_calibration_of_a_zero_travel_probe_is_refused(self) -> None:
        info = calibration.in_memory(0.5, 0.5, 65.21)
        assert not info.usable
        assert info.limits is None
        assert any("行程" in p for p in info.problems)

    def test_a_memory_calibration_carries_its_extra_fields(self) -> None:
        info = calibration.in_memory(1.775959, -0.064279, 65.21, source="guided")
        assert info.raw["source"] == "guided"
        assert info.raw["travel_range_rad"] == pytest.approx(1.840238)


class TestCrossCheck:
    """Observing the outcome, which is the only defence that cannot be bypassed.

    The applied limits these build are what the SDK's config holds *after* a
    load — the file's own numbers — because that is the state the cross-check
    exists to read.  The derived scale is written into the config afterwards,
    by the backend, so it is deliberately not what this compares against.
    """

    @staticmethod
    def _as_loaded(raw: dict) -> Limits:
        return Limits(
            closed_rad=raw["zero_position_rad"],
            open_rad=raw["max_position_rad"],
            rad_to_mm=raw["rad_to_mm"],
        )

    def test_agreement_produces_no_warning(self, env) -> None:
        _write(env.user, USER_RAW)
        info = calibration.resolve()
        assert calibration.cross_check(info, self._as_loaded(USER_RAW)) == []

    def test_a_silent_fallback_to_the_factory_file_is_caught(self, env) -> None:
        """The exact scenario: we asked for the user file, the SDK loaded its
        own.  Without this the operator sees a working gripper and wrong mm."""
        _write(env.user, USER_RAW)
        info = calibration.resolve()
        warnings = calibration.cross_check(info, self._as_loaded(FACTORY_RAW))
        assert warnings
        assert "交叉核对失败" in warnings[0]
        assert any("zero_position_rad" in w for w in warnings)

    def test_the_derived_scale_alone_is_a_mismatch(self, env) -> None:
        """A config holding the console's own scale was not loaded from the file.

        Which is the point of comparing against the file's coefficient: the
        derived one is what the backend applies *after* this check, and it is
        also what a config left over from another source would look like.
        """
        _write(env.user, USER_RAW)
        info = calibration.resolve()
        assert calibration.cross_check(info, info.limits)

    def test_a_small_angle_difference_is_caught(self, env) -> None:
        _write(env.user, USER_RAW)
        info = calibration.resolve()
        applied = Limits(
            USER_RAW["zero_position_rad"] + 0.01,
            USER_RAW["max_position_rad"],
            USER_RAW["rad_to_mm"],
        )
        assert calibration.cross_check(info, applied)

    def test_a_small_rad_to_mm_difference_is_caught(self, env) -> None:
        _write(env.user, USER_RAW)
        info = calibration.resolve()
        applied = Limits(
            USER_RAW["zero_position_rad"],
            USER_RAW["max_position_rad"],
            USER_RAW["rad_to_mm"] * 1.01,
        )
        assert calibration.cross_check(info, applied)

    def test_the_conversion_factor_is_compared_relatively(self, env) -> None:
        """0.001 mm/rad is noise at 65 and a rounding error at 1; only the
        relative difference is meaningful."""
        _write(env.user, USER_RAW)
        info = calibration.resolve()
        applied = Limits(
            USER_RAW["zero_position_rad"],
            USER_RAW["max_position_rad"],
            USER_RAW["rad_to_mm"] + 1e-6,
        )
        assert calibration.cross_check(info, applied) == []

    def test_an_unusable_calibration_that_was_applied_anyway_is_caught(self, env) -> None:
        """Motion is refused here, so what the backend happens to hold is not
        just a mismatch — it is a config nobody vetted."""
        _write(env.user, ZERO_TRAVEL_RAW)
        info = calibration.resolve()
        applied = Limits(0.5, 0.5, 65.21)
        warnings = calibration.cross_check(info, applied)
        assert warnings and "运动已拒绝" in warnings[0]

    def test_a_backend_that_reports_nothing_is_caught(self, env) -> None:
        _write(env.user, USER_RAW)
        info = calibration.resolve()
        assert "无法确认标定是否生效" in calibration.cross_check(info, None)[0]

    def test_an_unusable_calibration_with_no_limits_is_reported(self, env) -> None:
        _write(env.user, "{ truncated")
        info = calibration.resolve()
        assert calibration.cross_check(info, None) == ["请求载入的标定不可用（标定无效）"]


# ── writing ─────────────────────────────────────────────────────────────────
class TestRoundTrip:
    def test_the_saved_schema_matches_the_sdks(self) -> None:
        """A file we write must be loadable by ``load_calibration`` unchanged."""
        data = calibration.build_save_dict(Limits(1.775959, -0.064279, 65.21))
        assert set(data) == set(calibration._SAVED_KEYS)
        _, problems = calibration.parse_calibration_json(json.dumps(data))
        assert problems == []

    def test_a_saved_file_reloads_to_the_same_limits(self, env) -> None:
        original = calibration.resolve(_write(env.dir / "a.json", USER_RAW))
        target = env.dir / "nested" / "b.json"
        calibration.write_calibration(target, calibration.build_save_dict(original.limits))

        reloaded = calibration.resolve(target)
        assert reloaded.provenance == PROVENANCE_USER
        assert reloaded.limits == original.limits

    def test_writing_creates_parent_directories(self, env) -> None:
        target = env.dir / "deep" / "deeper" / "c.json"
        written = calibration.write_calibration(target, calibration.build_save_dict(
            Limits(1.775959, -0.064279, 65.21)
        ))
        assert Path(written).is_file()

    def test_the_closed_and_open_names_are_mapped_not_copied(self) -> None:
        """``max_position_rad`` reads as "largest angle" but means "fully open";
        the mapping onto ``Limits`` is where that is disambiguated."""
        limits = calibration.limits_from_raw(USER_RAW)
        assert limits.closed_rad == USER_RAW["zero_position_rad"]
        assert limits.open_rad == USER_RAW["max_position_rad"]

    def test_a_custom_stroke_is_passed_through(self) -> None:
        assert calibration.limits_from_raw(USER_RAW, 200.0).max_stroke_mm == 200.0


# ── presentation ────────────────────────────────────────────────────────────
class TestPresentation:
    @pytest.mark.parametrize(
        "raw",
        [USER_RAW, FACTORY_RAW, UNCALIBRATED_RAW, None],
        ids=["user", "factory", "reverse_mounted", "missing"],
    )
    def test_describe_and_as_table_never_raise(self, env, raw) -> None:
        """The status bar and the calibration page call these unconditionally,
        including for calibrations that have no limits at all."""
        if raw is None:
            info = calibration.resolve()
        else:
            _write(env.user, raw)
            info = calibration.resolve()
        assert info.describe()
        rows = info.as_table()
        assert rows and all(len(r) == 2 for r in rows)

    def test_the_table_reports_the_numbers_and_the_travel(self, env) -> None:
        """Both coefficients are on show: the one in force, derived from the
        travel, and the one the file carries, which is how much they disagree."""
        _write(env.user, USER_RAW)
        table = dict(calibration.resolve().as_table())
        assert table["来源"] == "用户标定"
        assert table["zero_position_rad (闭合)"] == "1.775959"
        assert table["rad_to_mm (按行程推导)"] == "46.733086"
        assert table["rad_to_mm (文件自带)"] == "65.210000"
        assert table["记录极限跨度"] == "86.00 mm"
        assert table["行程上限 (可命令)"] == "85.0 mm"
        assert table["can_id"] == "0x08"

    def test_a_missing_can_id_shows_a_dash(self, env) -> None:
        _write(env.user, UNCALIBRATED_RAW)
        table = dict(calibration.resolve().as_table())
        assert table["can_id"] == "—"

    def test_the_summary_is_two_rows_and_no_radians(self, env) -> None:
        """The dict stays small on purpose: every extra row is another number
        between the operator and the two that matter.

        One decimal, which is the slider's own resolution — the six decimals the
        detail table carries are for checking a file, not for reading a gripper.
        """
        _write(env.user, USER_RAW)
        info = calibration.resolve()
        assert info.summary() == [("来源", "用户标定"), ("行程", "85.0 mm")]
        assert not any("rad" in key.lower() for key, _ in info.summary())
        assert dict(info.as_table())["行程上限 (可命令)"] == "85.0 mm"

    def test_the_summary_says_it_has_no_stroke_rather_than_a_wrong_one(self, env) -> None:
        info = calibration.resolve()
        assert dict(info.summary())["行程"] == "—"

    def test_the_headline_is_a_sentence_not_a_dump(self, env) -> None:
        _write(env.user, USER_RAW)
        headline = calibration.resolve().headline()
        assert headline == "用户标定：行程 85.0 mm"
        # The log line keeps the radians; the banner is read while deciding
        # whether to move, so it must not carry them.
        assert "rad" not in headline
        assert "rad" in calibration.resolve().describe()

    def test_neither_the_headline_nor_the_table_names_a_mounting(self, env) -> None:
        """The declaration is gone, and so is every word it used to put on
        screen: a row that says 正向装配 is a question the operator is no longer
        asked, and an answer nothing reads."""
        _write(env.user, REVERSED_RAW)
        info = calibration.resolve()
        rows = dict(info.as_table())
        assert "方向" not in info.headline()
        assert not [key for key in rows if "方向" in key or "装配" in key]
        assert not [value for value in rows.values() if "装配" in value]

    def test_every_provenance_has_a_chinese_label(self) -> None:
        for name in (
            PROVENANCE_USER,
            PROVENANCE_FACTORY,
            PROVENANCE_INVALID,
            PROVENANCE_MISSING,
            PROVENANCE_MEMORY,
        ):
            assert name in calibration.PROVENANCE_LABELS


# ── paths ───────────────────────────────────────────────────────────────────
class TestPaths:
    def test_the_user_path_follows_the_environment(self, monkeypatch, tmp_path) -> None:
        monkeypatch.setenv("LITEGRIP_CALIB", str(tmp_path / "x.json"))
        assert calibration.default_user_path() == tmp_path / "x.json"

    def test_the_user_path_defaults_to_the_sdk_location(self, monkeypatch) -> None:
        """Mirrors ``DEFAULT_CALIB`` (gripper.py:39) — if these diverge, the file
        the console validates is not the file the SDK loads."""
        monkeypatch.delenv("LITEGRIP_CALIB", raising=False)
        assert calibration.default_user_path() == (
            Path.home() / ".litegrip" / "litegrip_calibration.json"
        )

    def test_the_factory_path_points_inside_the_sdk(self, monkeypatch) -> None:
        monkeypatch.delenv("LITEGRIP_FACTORY_CALIB", raising=False)
        assert calibration.factory_path() == (
            Path(litegrip.__file__).resolve().parent / "factory_calibration.json"
        )

    def test_the_vendored_factory_file_is_usable(self, monkeypatch) -> None:
        """The fallback is only a fallback if it is valid — otherwise the console
        would refuse to start on a machine that has no problem.

        It is *data*, which is the part that gets left out of a package: a wheel
        built without the SDK's package data, a bundle built without
        ``--collect-data litegrip``, both leave the code that reads it intact. So
        this asserts on the file's content, not on its presence alone — and it
        does not skip when the file is absent, because shipping without it is the
        failure it is here to catch.
        """
        monkeypatch.delenv("LITEGRIP_FACTORY_CALIB", raising=False)
        path = calibration.factory_path()
        assert path.is_file(), f"出厂标定文件不在产物里：{path}"

        raw, problems = calibration.parse_calibration_json(
            path.read_text(encoding="utf-8")
        )
        assert problems == []
        limits = calibration.limits_from_raw(raw, constants.DEFAULT_TRAVEL_MM)
        hard, _soft = calibration.validate_limits(
            limits, constants.DEFAULT_TRAVEL_MM, calibration.file_scale(raw)
        )
        assert hard == []

        info = calibration.resolve(path)
        assert info.problems == ()
        assert info.usable

    def test_the_only_fallback_is_the_vendored_sdk_file(self, monkeypatch) -> None:
        """One candidate, and it is the SDK's own file.

        The console used to prefer the SDK's file and keep a copy of the same
        bytes as a second candidate. Now that the SDK is vendored, that second
        candidate resolved to the same file on every machine — two ways to name
        one fallback is a way to load the wrong millimetres without anything on
        screen to say so.
        """
        monkeypatch.delenv("LITEGRIP_FACTORY_CALIB", raising=False)
        sdk = Path(litegrip.__file__).resolve().parent / "factory_calibration.json"

        assert calibration.factory_candidates() == (sdk,)
        assert calibration.factory_path() == sdk

    def test_an_override_replaces_the_sdk_file_rather_than_joining_it(
        self, monkeypatch, tmp_path
    ) -> None:
        """A path set on purpose is the answer, not a suggestion.

        Joining it would keep reading the SDK's file behind the operator's back
        when the override turns out to be absent — and the machine this is set on
        is one somebody is trying to bring up with a file of their own.
        """
        mine = tmp_path / "factory.json"
        mine.write_text(json.dumps(FACTORY_RAW), encoding="utf-8")
        monkeypatch.setenv("LITEGRIP_FACTORY_CALIB", str(mine))

        assert calibration.factory_candidates() == (mine,)
        assert calibration.factory_path() == mine


class TestPathsAreShownRelative:
    """Display only.  Loading keeps the absolute path: the SDK finds its factory
    file through ``dirname(__file__)`` and a frozen build finds it under
    ``sys._MEIPASS``, and neither survives being made relative."""

    def test_the_factory_file_is_shown_relative_to_the_sdk(self) -> None:
        """It says where the file lives, not which machine it was checked out on."""
        assert calibration.friendly_path(calibration.factory_path()) == (
            "litegrip/factory_calibration.json"
        )

    def test_a_file_under_home_gets_a_tilde(self, tmp_path, monkeypatch) -> None:
        monkeypatch.setattr(Path, "home", classmethod(lambda cls: tmp_path))
        # Not under the SDK, so the home rule is the one that applies.
        target = tmp_path / ".litegrip" / "litegrip_calibration.json"
        assert calibration.friendly_path(target) == (
            "~/.litegrip/litegrip_calibration.json"
        )

    def test_a_path_with_no_meaningful_home_is_left_alone(self) -> None:
        """Shortening a path to ``../../`` would be worse than showing it."""
        assert calibration.friendly_path("/opt/somewhere/cal.json") == (
            "/opt/somewhere/cal.json"
        )

    def test_a_missing_path_is_a_dash(self) -> None:
        """An in-memory probe result has no file, and the page shows it."""
        assert calibration.friendly_path(None) == "—"
        assert calibration.friendly_path("") == "—"


# ── the fixture itself ──────────────────────────────────────────────────────
class TestTheFixtureIsTheFileItClaims:
    """``FACTORY_RAW`` stands in for *the* factory calibration in a dozen tests,
    so a wrong one is worse than a missing one: every test still passes, and the
    suite then describes a file no machine loads.

    That is what happened.  The numbers here were a fork's factory file
    (``0.114 / -1.491 / 74.8``, the SDK's nominal pair for a 120 mm unit) under a
    comment naming the SDK this console was built against, so the file the
    console actually falls back to had never been through these tests at all.
    """

    def test_it_matches_every_copy_the_repository_ships(self) -> None:
        copies = shipped_factory_calibrations()
        assert copies, "this repository ships no factory calibration at all"

        for path in copies:
            assert json.loads(path.read_text(encoding="utf-8")) == FACTORY_RAW, (
                f"{path.relative_to(REPO_ROOT)} 与 FACTORY_RAW 已经不一致了，两份要一起改"
            )
