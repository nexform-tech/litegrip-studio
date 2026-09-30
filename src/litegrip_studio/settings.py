"""The operator's choices, remembered between runs.

An INI file under ``~/.litegrip``, next to the calibration it belongs with.
Written through :class:`QSettings` so it lands in the platform's usual place, but
read defensively: the file is on disk and editable by hand, so a value that has
been corrupted, hand-edited to something out of range, or written by an older
version has to fall back to the default rather than raise — or worse, be handed
to the motor.

The one setting that is not a convenience is
:attr:`Settings.allow_factory_calibration`.  It is the operator's standing
acknowledgement that the SDK's bundled factory file may be used, it defaults to
*off*, and it is read by the worker's gate rather than by a widget, so a console
that has never been told otherwise cannot be talked into moving an uncalibrated
gripper.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

from . import constants


def default_settings_path() -> Path:
    """Where the console keeps its preferences.

    Deliberately the same directory as the calibration files
    (:func:`litegrip_studio.calibration.default_user_path`): the two are read
    together when something has gone wrong, and an operator looking for one will
    find the other.
    """
    return Path.home() / ".litegrip" / "litegrip_studio.ini"


#: Keys, spelled out so a typo is a NameError rather than a silently ignored
#: setting that leaves the console behaving as though the operator never
#: changed anything.
KEY_ALLOW_FACTORY = constants.FACTORY_CALIBRATION_GATE_KEY
KEY_SPEED = "motion/speed_mm_s"
KEY_FORCE = "motion/force_n"
KEY_LIVE_FOLLOW = "motion/live_follow"
KEY_CALIBRATION_PATH = "calibration/path"
#: The measured travel used to be a preference here.  It is a property of the
#: bench unit rather than a choice, so it is :data:`constants.DEFAULT_TRAVEL_MM`
#: now — and this key is retired.  Only :data:`RETIRED_KEYS` reads it.
KEY_TRAVEL_MM = "calibration/travel_mm"
KEY_GEOMETRY = "view/geometry"
KEY_WINDOW_STATE = "view/window_state"
KEY_TAB = "view/tab"
KEY_SHOW_DEBUG = "view/show_debug_log"
#: ``dark`` or ``light``; absent until the operator picks one.
KEY_THEME = "view/theme"
KEY_LOG_LEVEL = "logging/level"

#: Keys a console older than this one may have written and nothing reads any
#: more.  They are deleted on open rather than left alone, because the file is
#: what an operator (or a support engineer) reads when something is wrong: a
#: ``travel_mm=10`` sitting in it would look like the console's own belief about
#: the bench, and there is no way to tell from the file that it is inert.
RETIRED_KEYS = (KEY_TRAVEL_MM,)

_TRUE = frozenset({"1", "true", "yes", "on", "y", "t"})
_FALSE = frozenset({"0", "false", "no", "off", "n", "f", ""})


class Settings:
    """Typed accessors over a key/value store.

    ``store`` is anything with ``QSettings``'s ``value``/``setValue``/``sync``.
    Tests pass a dictionary-backed double, which is also why nothing here
    imports Qt at module scope.
    """

    def __init__(self, store: Any = None, path: str | Path | None = None) -> None:
        self.path = Path(path) if path is not None else default_settings_path()
        if store is None:
            from PyQt5.QtCore import QSettings

            store = QSettings(str(self.path), QSettings.IniFormat)
        self._store = store
        self._drop_retired_keys()

    # ── motion ──────────────────────────────────────────────────────────────
    @property
    def speed_mm_s(self) -> float:
        return self._float(KEY_SPEED, constants.SPEED_DEFAULT_MM_S,
                           constants.SPEED_MIN_MM_S, constants.SPEED_MAX_MM_S)

    @speed_mm_s.setter
    def speed_mm_s(self, value: float) -> None:
        self._write(KEY_SPEED, self._clamp(value, constants.SPEED_MIN_MM_S,
                                           constants.SPEED_MAX_MM_S,
                                           constants.SPEED_DEFAULT_MM_S))

    @property
    def force_n(self) -> float:
        return self._float(KEY_FORCE, constants.FORCE_DEFAULT_N, 0.0, constants.FORCE_MAX_N)

    @force_n.setter
    def force_n(self, value: float) -> None:
        self._write(KEY_FORCE, self._clamp(value, 0.0, constants.FORCE_MAX_N,
                                           constants.FORCE_DEFAULT_N))

    @property
    def live_follow(self) -> bool:
        """Whether dragging the slider moves the jaws while the hand is down.

        Off by default: committing on release is what litearm-studio does, and it
        means a drag across the bar is one move rather than a hundred.
        """
        return self._bool(KEY_LIVE_FOLLOW, False)

    @live_follow.setter
    def live_follow(self, value: bool) -> None:
        self._write(KEY_LIVE_FOLLOW, bool(value))

    # ── calibration ─────────────────────────────────────────────────────────
    @property
    def allow_factory_calibration(self) -> bool:
        """The standing acknowledgement that the factory file may be used.

        Defaults to False, and the failure direction matters: a missing or
        unreadable setting must leave the gripper blocked, not permit it.
        """
        return self._bool(KEY_ALLOW_FACTORY, False)

    @allow_factory_calibration.setter
    def allow_factory_calibration(self, value: bool) -> None:
        self._write(KEY_ALLOW_FACTORY, bool(value))

    @property
    def calibration_path(self) -> str | None:
        """The file the operator last loaded, or None to use the SDK's default."""
        return self._text(KEY_CALIBRATION_PATH) or None

    @calibration_path.setter
    def calibration_path(self, value: str | None) -> None:
        if value:
            self._write(KEY_CALIBRATION_PATH, str(value))
        else:
            self._store.remove(KEY_CALIBRATION_PATH)
            self._sync()

    # ── view ────────────────────────────────────────────────────────────────
    @property
    def geometry(self) -> Any:
        """Opaque ``QByteArray``; only :meth:`QWidget.restoreGeometry` reads it."""
        return self._store.value(KEY_GEOMETRY, None)

    @geometry.setter
    def geometry(self, value: Any) -> None:
        self._write(KEY_GEOMETRY, value)

    @property
    def window_state(self) -> Any:
        return self._store.value(KEY_WINDOW_STATE, None)

    @window_state.setter
    def window_state(self, value: Any) -> None:
        self._write(KEY_WINDOW_STATE, value)

    @property
    def active_tab(self) -> int:
        return int(self._float(KEY_TAB, 0.0, 0.0, 99.0))

    @active_tab.setter
    def active_tab(self, value: int) -> None:
        self._write(KEY_TAB, max(0, int(value)))

    @property
    def theme(self) -> str | None:
        """``"dark"`` or ``"light"``, or None while the operator has not chosen.

        ``None`` rather than the resolved name, so that "nobody has chosen" is
        still readable a restart later.  A console that wrote down the theme it
        opened in on its first launch could not tell that apart from a choice,
        and the two want different answers if the default ever moves.
        """
        name = self._text(KEY_THEME)
        return name if name in ("dark", "light") else None

    @theme.setter
    def theme(self, value: str | None) -> None:
        if value:
            self._write(KEY_THEME, str(value))
        else:
            self._store.remove(KEY_THEME)
            self._sync()

    @property
    def show_debug_log(self) -> bool:
        """Whether the log dock shows the per-command debug lines.

        Off by default: every command the console sends is one of them, and they
        would bury the lines that describe what actually happened.
        """
        return self._bool(KEY_SHOW_DEBUG, False)

    @show_debug_log.setter
    def show_debug_log(self, value: bool) -> None:
        self._write(KEY_SHOW_DEBUG, bool(value))

    # ── internals ───────────────────────────────────────────────────────────
    def _drop_retired_keys(self) -> None:
        """Delete keys this console no longer reads, if any are present.

        Only writes when there is something to remove: opening the console must
        not rewrite a file nothing has changed.  A store that cannot be cleaned
        is not fatal — the values are inert either way.
        """
        for key in RETIRED_KEYS:
            try:
                present = self._store.value(key, None) is not None
            except Exception:  # noqa: BLE001 - an unreadable store is not fatal
                continue
            if not present:
                continue
            try:
                self._store.remove(key)
            except Exception:  # noqa: BLE001
                continue
            self._sync()

    def _read(self, key: str, default: Any) -> Any:
        try:
            value = self._store.value(key, default)
        except Exception:  # noqa: BLE001 - an unreadable store is not fatal
            return default
        return default if value is None else value

    def _text(self, key: str) -> str:
        value = self._read(key, "")
        return value if isinstance(value, str) else str(value)

    def _bool(self, key: str, default: bool) -> bool:
        value = self._read(key, default)
        if isinstance(value, bool):
            return value
        text = str(value).strip().lower()
        if text in _TRUE:
            return True
        if text in _FALSE:
            return False
        return default

    def _float(self, key: str, default: float, lo: float, hi: float) -> float:
        return self._clamp(self._read(key, default), lo, hi, default)

    @staticmethod
    def _clamp(value: Any, lo: float, hi: float, default: float) -> float:
        try:
            number = float(value)
        except (TypeError, ValueError):
            return default
        if not math.isfinite(number):
            return default
        return min(max(number, lo), hi)

    def _write(self, key: str, value: Any) -> None:
        # Failing to remember a preference is not a reason to stop the console:
        # every one of these is a convenience, and the setting that is not —
        # the factory acknowledgement — fails safe, because its absence reads
        # as False.
        try:
            self._store.setValue(key, value)
        except Exception:  # noqa: BLE001
            return
        self._sync()

    def _sync(self) -> None:
        try:
            self._store.sync()
        except Exception:  # noqa: BLE001 - a read-only home directory is not fatal
            pass
