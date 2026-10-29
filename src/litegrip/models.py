"""LiteGrip SDK — data models."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum, IntFlag
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


#: Age past which a :class:`GripperState` counts as stale, in seconds.  The same
#: threshold the C++ SDK uses, so a guard reads the same on either side.
STALE_AFTER_S = 0.5


@dataclass
class GripperState:
    """Gripper state snapshot, and how fresh it is.

    The readings are a cache, updated only by a status frame.  Whether such a
    frame arrived is what :attr:`fresh`, :attr:`data_age_s` and
    :meth:`is_stale` report — a snapshot from a bus that said nothing is not a
    measurement, and it must not look like one.  Check them before trusting the
    numbers; :meth:`LiteGrip.refresh_status` elicits a frame when the bus is
    idle.
    """

    position_rad: float = 0.0
    velocity_rad_s: float = 0.0
    torque_nm: float = 0.0
    temperature_mos: int = 0
    temperature_coil: int = 0
    error_code: int = 0
    #: Wall-clock time the frame behind this snapshot arrived — not the time of
    #: the call that returned it.  ``0.0`` means no frame has ever arrived, so
    #: every reading above is still a constructor default.
    timestamp: float = 0.0

    # Convenience — computed from raw values with unit conversion
    position_mm: float = 0.0
    force_n: float = 0.0

    #: Whether a status frame answered *this* read.  False means the readings
    #: are the ones from the previous frame, however old that is.
    fresh: bool = False
    #: Seconds since the frame behind this snapshot arrived, or ``None`` when
    #: there is no such frame at all.
    data_age_s: Optional[float] = None

    def is_stale(self, stale_after_s: float = STALE_AFTER_S) -> bool:
        """True when the newest frame is older than *stale_after_s*, or absent.

        This is about the snapshot's age, not about the read that produced it:
        a read no frame answered has ``fresh=False`` while the snapshot may
        still be younger than the threshold.  Ask both questions when the
        difference matters — "how old is this" and "did anyone answer just now".

        ``stale_after_s`` defaults to :data:`STALE_AFTER_S`, the same threshold
        the C++ SDK uses.
        """
        return self.data_age_s is None or self.data_age_s > stale_after_s

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

        ``+1.0`` when closing means increasing radians (the shipped
        mounting), ``-1.0`` when the limits run the other way.  Derived from
        the ordering of the two calibrated limits, so direction is data, not
        a separate switch — and it is only meaningful once :attr:`calibrated`
        is True.
        """
        return 1.0 if self.pos_closed_rad >= self.pos_open_rad else -1.0

    @property
    def mount(self) -> Optional[str]:
        """Legacy mount label (``"normal"`` / ``"reverse"``), or ``None``.

        **Deprecated.**  Kept for backward compatibility.  Shipped grippers
        have one mounting; on a calibrated unit this reports ``"normal"``.
        Read ``pos_closed_rad`` / ``pos_open_rad`` directly when you need the
        travel.

        ``None`` while :attr:`calibrated` is False: the placeholder defaults
        also happen to order close above open, so reporting ``"normal"``
        before a calibration has been loaded would be a claim, not a reading.
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
