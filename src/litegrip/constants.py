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
# Gripper geometry and unit conversion
# ═════════════════════════════════════════════════════════════════════════

class GripperGeometry:
    """The LiteGrip two-finger gripper's nominal geometry.

    These are the numbers every uncalibrated default in this package comes
    from.  They are the measured geometry of the reference unit, not a
    tolerance to be picked per gripper: an ``85 mm`` jaw travel is what the
    calipers read, and the ``1 mm`` inset is how far the probe presses into
    the open stop while looking for it.

    **Keep the two apart; do not fold the inset into the travel.** The
    calibration probe measures the span *between the two stops*, which is
    ``SPAN_MM`` (86 mm) — the jaws at rest on the closed stop, and pressed
    ~1 mm past where the jaws have run out of travel on the open one.  The
    millimetres-per-radian scale is that span over the measured travel in
    radians (``rad_to_mm``); deriving it from the 85 mm jaw travel instead
    would leave every mm-based move 1.2% short — 1 mm lost per full stroke.
    The factory calibration's own numbers show the pair: 1.409552 rad of
    travel is 86 mm at ``61.01229326764816`` mm/rad, and 85 mm at
    ``60.303``.

    ``work_stroke_mm`` is measured from the closed zero like
    :attr:`JAW_TRAVEL_MM`, so a full-stroke ``work_stroke_mm`` is 85 mm, not
    ``SPAN_MM``.
    """

    #: Jaw travel measured with calipers, from the closed stop (mm).
    JAW_TRAVEL_MM: Final = 85.0

    #: How far the calibration probe presses into the open stop (mm).
    STOP_INSET_MM: Final = 1.0

    #: What the recorded extremes span (mm) — the numerator of the scale.
    SPAN_MM: Final = JAW_TRAVEL_MM + STOP_INSET_MM


class UnitConversion:
    """Unit conversion coefficients.

    Nominal values for this gripper — see :class:`GripperGeometry`.  They are
    the span over the uncalibrated placeholder travel
    (:attr:`~litegrip.models.GripperConfig.pos_closed_rad`, ``1.14 rad``), and
    every motion is refused until a real calibration replaces them.  Run
    ``calibrate()`` to get accurate per-unit values.
    """
    RAD_TO_MM: Final = GripperGeometry.SPAN_MM / 1.14   # ≈ 75.44 mm/rad
    MM_TO_RAD: Final = 1.14 / GripperGeometry.SPAN_MM   # ≈ 0.01326 rad/mm
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
