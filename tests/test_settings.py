"""The preferences file, which is on disk and editable by hand.

Two things are being pinned here, and only one of them is about preferences.
The ordinary one is that a stored value survives a round trip and an
out-of-range one is clamped.  The one that matters is that the store is *hostile
input*: it can be corrupted, hand-edited, or written by an older version, and
none of those may raise, and none of them may be handed to the motor.  In
particular a garbled ``safety/allow_factory_calibration`` has to read as False —
the failure direction for that setting is the whole point of it.
"""

from __future__ import annotations

import math

import pytest

from litegrip_studio import constants
from litegrip_studio.settings import (
    KEY_ALLOW_FACTORY,
    KEY_FORCE,
    KEY_SPEED,
    Settings,
    default_settings_path,
)


class Store:
    """A ``QSettings``-shaped dictionary, including its stringly-typed habits.

    Values round-trip as whatever was written, but ``_raw`` lets a test plant
    what a hand-edit would: the strings an INI file actually holds.
    """

    def __init__(self, **initial) -> None:
        self.data: dict[str, object] = dict(initial)
        self.syncs = 0
        self.removed: list[str] = []

    def value(self, key, default=None):
        return self.data.get(key, default)

    def setValue(self, key, value) -> None:
        self.data[key] = value

    def remove(self, key) -> None:
        self.removed.append(key)
        self.data.pop(key, None)

    def sync(self) -> None:
        self.syncs += 1


def raw(**pairs) -> Store:
    """A store as an INI file holds it: every value already a string."""
    return Store(**{k: str(v) for k, v in pairs.items()})


class TestDefaults:
    def test_an_empty_store_gives_the_documented_defaults(self) -> None:
        settings = Settings(Store())

        assert settings.speed_mm_s == constants.SPEED_DEFAULT_MM_S
        assert settings.force_n == constants.FORCE_DEFAULT_N
        assert settings.travel_mm == constants.DEFAULT_TRAVEL_MM
        assert settings.live_follow is False
        assert settings.calibration_path is None
        assert settings.active_tab == 0
        assert settings.show_debug_log is False

    def test_the_factory_acknowledgement_starts_off(self) -> None:
        """The safety-relevant default, on its own so a change to it is loud."""
        assert Settings(Store()).allow_factory_calibration is False

    def test_the_default_path_sits_with_the_calibration_files(self) -> None:
        from litegrip_studio import calibration

        assert default_settings_path().parent == calibration.default_user_path().parent


class TestRoundTrip:
    def test_every_setting_survives_a_write_and_a_read(self) -> None:
        store = Store()
        settings = Settings(store)
        settings.speed_mm_s = 80.0
        settings.force_n = 25.0
        settings.live_follow = True
        settings.allow_factory_calibration = True
        settings.travel_mm = 150.0
        settings.calibration_path = "/tmp/cal.json"
        settings.active_tab = 2
        settings.show_debug_log = True

        reopened = Settings(store)
        assert reopened.speed_mm_s == pytest.approx(80.0)
        assert reopened.force_n == pytest.approx(25.0)
        assert reopened.live_follow is True
        assert reopened.allow_factory_calibration is True
        assert reopened.travel_mm == pytest.approx(150.0)
        assert reopened.calibration_path == "/tmp/cal.json"
        assert reopened.active_tab == 2
        assert reopened.show_debug_log is True

    def test_a_write_is_flushed(self) -> None:
        """A console that is killed rather than closed must not lose the
        acknowledgement the operator gave it."""
        store = Store()
        Settings(store).allow_factory_calibration = True

        assert store.syncs >= 1

    def test_clearing_the_calibration_path_removes_the_key(self) -> None:
        store = raw(**{"calibration/path": "/tmp/cal.json"})
        Settings(store).calibration_path = None

        assert "calibration/path" in store.removed
        assert Settings(store).calibration_path is None

    def test_the_window_geometry_is_stored_opaquely(self) -> None:
        """Qt's own format; the console must not try to understand it."""
        blob = b"\x01\x02\x03"
        store = Store()
        Settings(store).geometry = blob

        assert Settings(store).geometry == blob


class TestTheStoreIsHostileInput:
    """Nothing in the file may raise, and nothing in it may widen what the
    gripper is allowed to do."""

    @pytest.mark.parametrize("junk", ["abc", "", "  ", "nan", "inf", "-inf", "1e999", None, []])
    def test_an_unparseable_number_falls_back_to_the_default(self, junk) -> None:
        store = Store(**{KEY_SPEED: junk, KEY_FORCE: junk})
        settings = Settings(store)

        assert settings.speed_mm_s == constants.SPEED_DEFAULT_MM_S
        assert settings.force_n == constants.FORCE_DEFAULT_N

    @pytest.mark.parametrize(
        "stored, expected",
        [("9999", constants.SPEED_MAX_MM_S), ("-40", constants.SPEED_MIN_MM_S), ("0", constants.SPEED_MIN_MM_S)],
    )
    def test_an_out_of_range_speed_is_clamped(self, stored: str, expected: float) -> None:
        """The file may say 9999 mm/s; the motion planner never sees it."""
        assert Settings(raw(**{KEY_SPEED: stored})).speed_mm_s == pytest.approx(expected)

    @pytest.mark.parametrize(
        "stored, expected",
        [("999", constants.FORCE_MAX_N), ("-5", 0.0)],
    )
    def test_an_out_of_range_force_is_clamped(self, stored: str, expected: float) -> None:
        assert Settings(raw(**{KEY_FORCE: stored})).force_n == pytest.approx(expected)

    def test_the_write_setters_clamp_too(self) -> None:
        settings = Settings(Store())
        settings.speed_mm_s = 1.0e6
        settings.force_n = -1.0

        assert settings.speed_mm_s == pytest.approx(constants.SPEED_MAX_MM_S)
        assert settings.force_n == pytest.approx(0.0)

    def test_a_nan_write_is_refused_rather_than_stored(self) -> None:
        """NaN is what a division by a zero calibration would produce, and it
        would poison every comparison downstream."""
        settings = Settings(Store())
        settings.speed_mm_s = math.nan

        assert settings.speed_mm_s == pytest.approx(constants.SPEED_DEFAULT_MM_S)

    @pytest.mark.parametrize("stored", ["maybe", "2", "yess", "TRUEE", "1.5"])
    def test_an_ambiguous_boolean_does_not_become_true(self, stored: str) -> None:
        """Only the spellings that unambiguously mean yes are yes.  "2" and
        "maybe" are a hand-edit, and the safe reading of a hand-edit is no."""
        assert Settings(raw(**{KEY_ALLOW_FACTORY: stored})).allow_factory_calibration is False

    @pytest.mark.parametrize("stored", ["true", "TRUE", "1", "yes", "on"])
    def test_the_unambiguous_spellings_of_yes_are_honoured(self, stored: str) -> None:
        assert Settings(raw(**{KEY_ALLOW_FACTORY: stored})).allow_factory_calibration is True

    def test_a_store_that_raises_on_read_is_survivable(self) -> None:
        class Broken(Store):
            def value(self, key, default=None):
                raise OSError("home directory went away")

        settings = Settings(Broken())
        assert settings.speed_mm_s == constants.SPEED_DEFAULT_MM_S
        assert settings.allow_factory_calibration is False

    def test_a_store_that_cannot_be_written_is_survivable(self) -> None:
        class ReadOnly(Store):
            def setValue(self, key, value) -> None:
                raise OSError("read-only file system")

            def sync(self) -> None:
                raise OSError("read-only file system")

        settings = Settings(ReadOnly())
        settings.speed_mm_s = 70.0  # must not raise

    def test_a_geometry_blob_that_was_never_written_is_none(self) -> None:
        """``restoreGeometry(None)`` is how Qt is told to use the default size,
        so an unset geometry has to be None rather than an empty value."""
        settings = Settings(Store())

        assert settings.geometry is None
        assert settings.window_state is None
