"""Damiao motor CAN protocol encoder/decoder — pure functions, no state.

Implements the complete DM motor protocol:
  - MIT frame pack/unpack (q/dq/kp/kd/tau → 8 bytes)
  - Command frames (enable/disable/clear fault/set zero)
  - Parameter read/write/save frames (broadcast to 0x7FF)
  - Status frame parsing
  - Parameter response frame parsing

Reference: DM4310/DM4340/DM6248P CAN protocol specification.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional

# ── CAN ID constants ─────────────────────────────────────────────────────
BROADCAST_ID = 0x7FF


class ControlMode(IntEnum):
    """CAN ID offsets for control frames."""
    MIT_MODE = 0x000
    POS_VEL_MODE = 0x100
    VEL_MODE = 0x200
    POS_FORCE_MODE = 0x300


class ControlModeCode(IntEnum):
    """CTRL_MODE register values."""
    MIT = 1
    POS_VEL = 2
    VEL = 3
    POS_FORCE = 4


class DM_REG(IntEnum):
    """Damiao motor register IDs."""
    UV_Value = 0
    KT_Value = 1
    OT_Value = 2
    OC_Value = 3
    ACC = 4
    DEC = 5
    MAX_SPD = 6
    MST_ID = 7
    ESC_ID = 8
    TIMEOUT = 9
    CTRL_MODE = 10
    Damp = 11
    Inertia = 12
    hw_ver = 13
    sw_ver = 14
    SN = 15
    NPP = 16
    Rs = 17
    LS = 18
    Flux = 19
    Gr = 20
    PMAX = 21
    VMAX = 22
    TMAX = 23
    I_BW = 24
    KP_ASR = 25
    KI_ASR = 26
    KP_APR = 27
    KI_APR = 28
    OV_Value = 29
    GREF = 30
    Deta = 31
    V_BW = 32
    IQ_c1 = 33
    VL_c1 = 34
    can_br = 35
    sub_ver = 36


# ── Integer register ranges (big-endian decode on read response) ─────────
_INT_REG_RANGES = (
    (7, 10),
    (13, 16),
    (35, 36),
)


def _is_int_register(rid: int) -> bool:
    """Return True if the register ID stores an integer value."""
    for lo, hi in _INT_REG_RANGES:
        if lo <= rid <= hi:
            return True
    return False


# ── Quantization helpers ─────────────────────────────────────────────────

def float_to_uint(value: float, value_min: float, value_max: float,
                  bits: int) -> int:
    """Quantize a float to unsigned integer of given bit width."""
    value = max(value_min, min(value_max, value))
    span = value_max - value_min
    offset = value - value_min
    return int(offset * ((1 << bits) - 1) / span)


def uint_to_float(value: int, value_min: float, value_max: float,
                  bits: int) -> float:
    """Dequantize an unsigned integer back to float."""
    span = value_max - value_min
    return float(value) * span / float((1 << bits) - 1) + value_min


# ── MIT control frame ────────────────────────────────────────────────────

def pack_mit_frame(
    q: float,
    dq: float,
    kp: float,
    kd: float,
    tau: float,
    q_max: float = 12.5,
    dq_max: float = 30.0,
    tau_max: float = 10.0,
) -> bytes:
    """Pack position/torque targets into an 8-byte MIT control frame.

    Args:
        q:   Target position (rad), mapped to [-q_max, +q_max] in 16 bits.
        dq:  Target velocity (rad/s), mapped to [-dq_max, +dq_max] in 12 bits.
        kp:  Position stiffness (0–500), mapped to [0, 500] in 12 bits.
        kd:  Velocity damping (0–5), mapped to [0, 5] in 12 bits.
        tau: Feed-forward torque (Nm), mapped to [-tau_max, +tau_max] in 12 bits.

    Returns:
        8-byte CAN payload.
    """
    q_uint = float_to_uint(q, -q_max, q_max, 16)
    dq_uint = float_to_uint(dq, -dq_max, dq_max, 12)
    kp_uint = float_to_uint(kp, 0.0, 500.0, 12)
    kd_uint = float_to_uint(kd, 0.0, 5.0, 12)
    tau_uint = float_to_uint(tau, -tau_max, tau_max, 12)

    # Bit packing per protocol spec
    data = bytearray(8)
    data[0] = (q_uint >> 8) & 0xFF
    data[1] = q_uint & 0xFF
    data[2] = (dq_uint >> 4) & 0xFF
    data[3] = ((dq_uint & 0x0F) << 4) | ((kp_uint >> 8) & 0x0F)
    data[4] = kp_uint & 0xFF
    data[5] = (kd_uint >> 4) & 0xFF
    data[6] = ((kd_uint & 0x0F) << 4) | ((tau_uint >> 8) & 0x0F)
    data[7] = tau_uint & 0xFF

    return bytes(data)


# ── Status frame (motor → PC) ────────────────────────────────────────────

@dataclass
class ParsedStatus:
    """Parsed motor status frame."""
    err: int          # 4-bit error code (0=disabled, 1=enabled, 0x9=UV, etc.)
    can_id: int       # 4-bit CAN ID (low nibble of data[0])
    q: float          # Position (rad)
    dq: float         # Velocity (rad/s)
    tau: float        # Torque (Nm)
    t_mos: int        # MOS temperature (°C)
    t_coil: int       # Coil temperature (°C)


def unpack_status_frame(
    data: bytes,
    q_max: float = 12.5,
    dq_max: float = 30.0,
    tau_max: float = 10.0,
) -> ParsedStatus:
    """Parse an 8-byte motor status frame.

    Byte layout::

        [0] err(4) | can_id(4)
        [1] q[15:8]   [2] q[7:0]
        [3] dq[11:4]  [4] dq[3:0] | tau[11:8]
        [5] tau[7:0]
        [6] t_mos      [7] t_coil
    """
    if len(data) < 8:
        raise ValueError(f"Status frame needs 8 bytes, got {len(data)}")

    err = (data[0] >> 4) & 0x0F
    cid = data[0] & 0x0F

    q_uint = ((data[1] << 8) | data[2]) & 0xFFFF
    dq_uint = ((data[3] << 4) | (data[4] >> 4)) & 0xFFF
    tau_uint = (((data[4] & 0x0F) << 8) | data[5]) & 0xFFF

    q = uint_to_float(q_uint, -q_max, q_max, 16)
    dq = uint_to_float(dq_uint, -dq_max, dq_max, 12)
    tau = uint_to_float(tau_uint, -tau_max, tau_max, 12)

    t_mos = data[6]
    t_coil = data[7]

    return ParsedStatus(err=err, can_id=cid, q=q, dq=dq, tau=tau,
                        t_mos=t_mos, t_coil=t_coil)


# ── Command frames (PC → motor) ──────────────────────────────────────────

# Command byte values (placed in byte 7)
CMD_ENABLE = 0xFC
CMD_DISABLE = 0xFD
CMD_CLEAR_FAULT = 0xFB
CMD_SET_ZERO = 0xFE


def pack_command_frame(cmd: int) -> bytes:
    """Pack a command frame (enable/disable/clear fault/set zero).

    All 8 bytes are 0xFF except byte 7 which holds the command.
    """
    data = bytearray([0xFF] * 7)
    data.append(cmd)
    return bytes(data)


# ── Status refresh frame ────────────────────────────────────────────────

def pack_refresh_frame(can_id: int) -> bytes:
    """Pack a status refresh request (sent to broadcast ID 0x7FF).

    Returns:
        4-byte frame: [can_id_lo, can_id_hi, 0xCC, 0x00]
    """
    return bytes([can_id & 0xFF, (can_id >> 8) & 0xFF, 0xCC, 0x00])


# ── Parameter read/write/save frames ────────────────────────────────────

def pack_read_param_frame(can_id: int, rid: int) -> bytes:
    """Pack a parameter read request (sent to broadcast ID 0x7FF).

    Returns:
        8-byte frame: [can_id_lo, can_id_hi, 0x33, RID, 0, 0, 0, 0]
    """
    return bytes([can_id & 0xFF, (can_id >> 8) & 0xFF, 0x33, rid & 0xFF,
                  0, 0, 0, 0])


def pack_write_param_frame(can_id: int, rid: int, value) -> bytes:
    """Pack a parameter write request (sent to broadcast ID 0x7FF).

    Args:
        can_id: Motor CAN ID.
        rid:    Register ID.
        value:  int or float — int registers packed as little-endian uint32,
                float registers packed as IEEE 754 little-endian float32.

    Returns:
        8-byte frame: [can_id_lo, can_id_hi, 0x55, RID, data[0..3]]
    """
    if _is_int_register(rid):
        data_bytes = int(value).to_bytes(4, "little", signed=False)
    else:
        data_bytes = struct.pack("<f", float(value))

    return bytes([can_id & 0xFF, (can_id >> 8) & 0xFF, 0x55, rid & 0xFF,
                  data_bytes[0], data_bytes[1], data_bytes[2], data_bytes[3]])


def pack_save_param_frame(can_id: int) -> bytes:
    """Pack a parameter save-to-flash request (sent to broadcast ID 0x7FF).

    Note: Motor must be disabled before saving.

    Returns:
        8-byte frame: [can_id_lo, can_id_hi, 0xAA, 0x01, 0, 0, 0, 0]
    """
    return bytes([can_id & 0xFF, (can_id >> 8) & 0xFF, 0xAA, 0x01,
                  0, 0, 0, 0])


# ── Parameter response frame (motor → PC) ───────────────────────────────

@dataclass
class ParsedParamResponse:
    """Parsed parameter read/write response."""
    can_id: int      # Motor CAN ID (from data[0] & 0x0F)
    opcode: int      # 0x33 = read response, 0x55 = write response, 0xAA = save response
    rid: int         # Register ID
    value: float     # Decoded value (float for float regs, int cast to float for int regs)


def unpack_param_response(data: bytes) -> Optional[ParsedParamResponse]:
    """Parse a parameter response frame.

    Byte layout::

        [0] motor_id (low nibble = can_id)
        [1] can_id_hi
        [2] opcode (0x33=read, 0x55=write, 0xAA=save)
        [3] RID
        [4..7] data (4 bytes)

    Integer registers: little-endian uint32 decode (matching damiao firmware
    convention on the wire).
    Float registers:   little-endian IEEE 754 float32.
    """
    if len(data) < 8:
        return None

    can_id = data[0] & 0x0F
    opcode = data[2]
    rid = data[3]

    if opcode not in (0x33, 0x55, 0xAA):
        return None

    if _is_int_register(rid):
        value = float((data[7] << 24) | (data[6] << 16) |
                      (data[5] << 8) | data[4])
    else:
        try:
            value = struct.unpack("<f", bytes(data[4:8]))[0]
        except struct.error:
            value = 0.0

    return ParsedParamResponse(can_id=can_id, opcode=opcode, rid=rid,
                               value=value)


# ── Motor type parameters ────────────────────────────────────────────────

@dataclass
class MotorLimits:
    """MIT frame quantization limits for a motor type."""
    q_max: float      # rad
    dq_max: float     # rad/s
    tau_max: float    # Nm


# Per-motor-type limits (matching DM4310/DM4340/DM6248P datasheet)
MOTOR_LIMITS: dict[int, MotorLimits] = {
    1: MotorLimits(q_max=12.5, dq_max=30.0, tau_max=10.0),     # DM4310
    3: MotorLimits(q_max=12.5, dq_max=10.0, tau_max=28.0),     # DM4340
    6: MotorLimits(q_max=12.566, dq_max=20.0, tau_max=120.0),  # DM6248P
}


def get_motor_limits(motor_type: int) -> MotorLimits:
    """Get quantization limits for a given motor type index.

    Args:
        motor_type: DM_Motor_Type enum value (1=DM4310, 3=DM4340, 6=DM6248P, etc.)

    Returns:
        MotorLimits for that type. Falls back to DM4310 limits for unknown types.
    """
    return MOTOR_LIMITS.get(motor_type, MOTOR_LIMITS[1])
