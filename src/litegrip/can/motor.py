"""Per-motor state container — tracks decoded status from CAN frames.

Each motor instance holds the latest position, velocity, torque, error code,
and temperature values.  Updated by the MotorController when status frames
arrive on the motor's mst_id.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum

from .protocol import ControlMode, MotorLimits, get_motor_limits


class MotorType(IntEnum):
    """Damiao motor model indices (matching DM_Motor_Type)."""
    DM3507 = 0
    DM4310 = 1
    DM4310_48V = 2
    DM4340 = 3
    DM4340_48V = 4
    DM6006 = 5
    DM6248P = 6
    DM8006 = 7
    DM8009 = 8
    DM10010L = 9
    DM10010 = 10
    DMH3510 = 11
    DMH6215 = 12
    DMS3519 = 13
    DMG6220 = 14


@dataclass
class MotorParams:
    """Configuration parameters for a single motor."""
    motor_type: MotorType = MotorType.DM4310
    can_id: int = 0x08
    mst_id: int = 0x18          # Master ID (status frame CAN ID)
    control_mode: ControlMode = ControlMode.MIT_MODE
    limits: MotorLimits = field(default=None)

    def __post_init__(self):
        if self.limits is None:
            self.limits = get_motor_limits(int(self.motor_type))


class MotorState:
    """Runtime state of a single DM motor, updated from status frames.

    Thread-unsafe — caller must serialize updates.
    """

    __slots__ = (
        "_params",
        "_position",
        "_velocity",
        "_torque",
        "_error",
        "_t_mos",
        "_t_coil",
        "rx_count",
        "_last_update",
    )

    def __init__(self, params: MotorParams):
        self._params = params
        self._position: float = 0.0
        self._velocity: float = 0.0
        self._torque: float = 0.0
        self._error: int = 0
        self._t_mos: int = 0
        self._t_coil: int = 0
        self.rx_count: int = 0
        self._last_update: float = 0.0

    # ── properties ──────────────────────────────────────────────────────

    @property
    def can_id(self) -> int:
        return self._params.can_id

    @property
    def mst_id(self) -> int:
        return self._params.mst_id

    @property
    def control_mode(self) -> ControlMode:
        return self._params.control_mode

    @property
    def motor_type(self) -> MotorType:
        return self._params.motor_type

    @property
    def limits(self) -> MotorLimits:
        return self._params.limits

    @property
    def position(self) -> float:
        return self._position

    @property
    def velocity(self) -> float:
        return self._velocity

    @property
    def torque(self) -> float:
        return self._torque

    @property
    def error(self) -> int:
        return self._error

    @property
    def t_mos(self) -> int:
        return self._t_mos

    @property
    def t_coil(self) -> int:
        return self._t_coil

    @property
    def is_enabled(self) -> bool:
        return self._error == 1

    @property
    def is_fault(self) -> bool:
        return self._error not in (0, 1)

    @property
    def last_update(self) -> float:
        return self._last_update

    # ── control mode management ─────────────────────────────────────────

    def set_mode(self, mode: ControlMode) -> None:
        """Change the control mode (does not send CAN command)."""
        self._params.control_mode = mode

    def mode_offset(self) -> int:
        """Get the CAN ID offset for the current control mode."""
        return int(self._params.control_mode)

    # ── status update ───────────────────────────────────────────────────

    def update_from_status(self, position: float, velocity: float,
                           torque: float, error: int,
                           t_mos: int, t_coil: int,
                           timestamp: float = 0.0) -> None:
        """Called by MotorController when a status frame for this motor arrives."""
        self._position = position
        self._velocity = velocity
        self._torque = torque
        self._error = error
        self._t_mos = t_mos
        self._t_coil = t_coil
        self._last_update = timestamp
        self.rx_count += 1

    def __repr__(self) -> str:
        return (f"MotorState(can_id=0x{self.can_id:02X}, mst=0x{self.mst_id:02X}, "
                f"pos={self._position:.4f}, vel={self._velocity:.3f}, "
                f"tau={self._torque:.3f}, err={self._error}, "
                f"rx={self.rx_count})")
