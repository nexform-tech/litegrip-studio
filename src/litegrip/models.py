"""LiteGrip SDK — data models."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum, IntFlag
import time
from typing import Optional

from .constants import GripperGeometry, UnitConversion


class GripperMode(IntEnum):
    """Gripper control mode."""
    MIT = 0
    POSITION = 1
    VELOCITY = 2
    FORCE = 3


class GripperStatus(IntFlag):
    """Gripper status flags (bitmask)."""
    NONE = 0x00
    ENABLED = 0x01
    MOVING = 0x02
    AT_TARGET = 0x04
    GRASPED = 0x08
    ERROR = 0x10


@dataclass
class GripperState:
    """Live gripper state snapshot.

    All fields are updated on each :meth:`LiteGrip.get_state` call.
    """

    position_rad: float = 0.0
    velocity_rad_s: float = 0.0
    torque_nm: float = 0.0
    temperature_mos: int = 0
    temperature_coil: int = 0
    error_code: int = 0
    timestamp: float = field(default_factory=time.time)

    # Convenience — computed from raw values with unit conversion
    position_mm: float = 0.0
    force_n: float = 0.0

    @property
    def is_enabled(self) -> bool:
        """True if the motor is enabled (error_code == 1)."""
        return self.error_code == 1

    @property
    def is_error(self) -> bool:
        """True if a fault is active (error_code ∉ {0, 1})."""
        return self.error_code not in (0, 1)

    @property
    def is_moving(self) -> bool:
        """True if velocity exceeds a small threshold."""
        return abs(self.velocity_rad_s) > 0.01

    @property
    def aperture_mm(self) -> float:
        """Opening distance in mm (single-side displacement).

        For total jaw separation multiply by 2.
        """
        return self.position_mm


@dataclass
class GripperConfig:
    """Gripper configuration — tune these for your hardware."""

    # CAN
    can_channel: str = "can0"
    can_id: int = 0x08
    mst_id: Optional[int] = None          # None = auto-detect
    canfd_mode: bool = False

    # Control gains (MIT mode)
    kp: float = 100.0                      # position stiffness [0, 500]
    kd: float = 2.0                        # velocity damping [0, 5]

    # Position limits (rad) — updated by calibrate() / load_calibration().
    # Which of the two is numerically larger is NOT assumed: a gripper whose
    # motor is mounted the other way round (a "reverse mount") simply has
    # pos_closed_rad < pos_open_rad.  See :attr:`close_sign`.
    pos_closed_rad: float = 1.14           # closed → 0 mm
    pos_open_rad: float = 0.0              # open  → full stroke

    # True once the limits above come from a real calibration (or from a
    # calibration template) rather than from the placeholder defaults.  The
    # motion actions refuse to run while this is False, because with the
    # defaults still in place every direction is a guess.
    calibrated: bool = False

    # Jaw travel (mm) — the caliper measurement of your gripper's travel.
    # :meth:`LiteGrip.calibrate` derives ``rad_to_mm`` from it plus the probe
    # inset, because the recorded extremes span that inset more than the jaws
    # actually open; see :class:`~litegrip.constants.GripperGeometry`.
    max_stroke_mm: float = GripperGeometry.JAW_TRAVEL_MM

    # Working stroke (mm) — the opening the motions are allowed to reach, from
    # the closed zero.  ``0`` (or anything ≥ the jaw travel) means "no limit":
    # :meth:`actions.open` then presses onto the open mechanical stop as before.
    # Set it below the jaw travel to keep a margin at the open end (e.g. 80 mm on
    # an 85 mm travel leaves 5 mm), so a plain open no longer drives into the
    # stop.
    work_stroke_mm: float = 0.0

    # Unit conversion — update after calibration
    rad_to_mm: float = UnitConversion.RAD_TO_MM   # rad → mm
    nm_to_n: float = 10.0                  # Nm → N (approximate)

    # Grasp detection
    grasp_torque_threshold: float = 0.5    # Nm

    @property
    def close_sign(self) -> float:
        """Which way the motor counts when the jaws close.

        ``+1.0`` when closing means increasing radians (the usual mounting),
        ``-1.0`` for a reverse mount.  Derived from the ordering of the two
        calibrated limits, so direction is data, not a separate switch — and
        it is only meaningful once :attr:`calibrated` is True.
        """
        return 1.0 if self.pos_closed_rad >= self.pos_open_rad else -1.0

    @property
    def mount(self) -> Optional[str]:
        """``"normal"`` / ``"reverse"`` — the name the limits spell out.

        ``None`` while :attr:`calibrated` is False: the placeholder defaults
        also happen to order close above open, so reporting ``"normal"``
        before a calibration has been loaded would be a claim, not a reading.
        Handy for a caller that has to show the operator which way this unit
        is mounted.  Compare with the ``mount=`` argument of
        :class:`~litegrip.LiteGrip`, which *declares* a mount by loading a
        template rather than reading one back.
        """
        if not self.calibrated:
            return None
        return "normal" if self.close_sign > 0 else "reverse"


@dataclass
class GripperInfo:
    """Static device information."""

    model: str = "LiteGrip"
    motor_type: str = "DM4310"
    can_id: int = 0x08
    mst_id: int = 0x18
    firmware_version: str = ""
    serial_number: str = ""


@dataclass
class CalibrationData:
    """Result of a gripper calibration run."""

    zero_position: float = 1.14            # closed limit (rad)
    max_position: float = 0.0              # open limit (rad)
    travel_range: float = 1.14             # |max - zero| (rad)
    rad_to_mm: float = UnitConversion.RAD_TO_MM   # calibrated conversion
    motor_type: str = "DM4310"
    can_id: int = 0x08
    mst_id: int = 0x18
    calibration_time: str = ""

    @property
    def travel_mm(self) -> float:
        """Full stroke in mm."""
        return self.travel_range * self.rad_to_mm
