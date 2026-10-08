"""LiteGrip SDK — constants and protocol definitions.

Self-contained. Uses litegrip.can for DM motor types; does NOT import
damiao_socketcan or any arm library.
"""

from typing import Final

from .can.motor import MotorType
from .can.protocol import ControlMode as _ControlMode


# ═════════════════════════════════════════════════════════════════════════
# Re-export DM protocol types for convenience
# ═════════════════════════════════════════════════════════════════════════

DM_Motor_Type = MotorType
Control_Mode = _ControlMode


# ═════════════════════════════════════════════════════════════════════════
# LiteGrip gripper parameters
# ═════════════════════════════════════════════════════════════════════════

class GripperParams:
    """LiteGrip default parameters."""

    CAN_ID: Final = 0x08
    MST_ID: Final = 0x18
    MOTOR_TYPE: Final = MotorType.DM4310
    CONTROL_MODE: Final = Control_Mode.MIT_MODE

    # Position limits (rad) — nominal placeholders only.  The real limits come
    # from the calibration; do not treat these as a direction convention, since
    # a reverse-mounted motor has them the other way round.
    POS_CLOSED_RAD: Final = 1.14
    POS_OPEN_RAD: Final = 0.0

    # MIT quantization limits (DM4310)
    Q_MAX: Final = 12.5      # rad
    DQ_MAX: Final = 30.0     # rad/s
    TAU_MAX: Final = 10.0    # Nm

    # Default control gains
    DEFAULT_KP: Final = 100.0
    DEFAULT_KD: Final = 2.0

    # Fault recovery
    FAULT_CLEAR_RETRIES: Final = 5
    FAULT_CLEAR_DELAY_S: Final = 0.02


# ═════════════════════════════════════════════════════════════════════════
# Unit conversion (nominal — calibrate for accuracy)
# ═════════════════════════════════════════════════════════════════════════

class UnitConversion:
    """Unit conversion coefficients.

    Nominal values for a 120 mm stroke gripper.  Run calibrate() to get
    accurate per-unit values.
    """
    RAD_TO_MM: Final = 120.0 / 1.14   # ≈ 105.26 mm/rad
    MM_TO_RAD: Final = 1.14 / 120.0   # ≈ 0.0095 rad/mm
    NM_TO_N: Final = 10.0             # approximate N per Nm
    N_TO_NM: Final = 0.1              # approximate Nm per N


# ═════════════════════════════════════════════════════════════════════════
# Error codes
# ═════════════════════════════════════════════════════════════════════════

class ErrorCode:
    """Damiao motor error codes (extracted from status frame data[0] >> 4)."""
    DISABLED: Final = 0
    ENABLED: Final = 1
    UV_FAULT: Final = 0x9
    OC_FAULT: Final = 0xA
    MOS_OT: Final = 0xB
    COIL_OT: Final = 0xC


ERROR_DESCRIPTIONS = {
    0x0: "已失能",
    0x1: "已使能",
    0x9: "欠压故障 (UV)",
    0xA: "过流故障 (OC)",
    0xB: "MOS 过温故障",
    0xC: "线圈过温故障",
}


def describe_error(code: int) -> str:
    """Return a human-readable description for a motor error code."""
    return ERROR_DESCRIPTIONS.get(code, f"未知错误 (0x{code:X})")


# ═════════════════════════════════════════════════════════════════════════
# Default configuration
# ═════════════════════════════════════════════════════════════════════════

class DefaultParams:
    """System defaults."""
    CAN_CHANNEL: Final = "can0"
    CAN_BITRATE: Final = 1_000_000    # 1 Mbps classic CAN
    CANFD_MODE: Final = False
    TIMEOUT_S: Final = 0.1
    INIT_TIMEOUT_S: Final = 2.0
