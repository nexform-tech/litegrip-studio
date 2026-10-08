"""LiteGrip CAN subpackage — self-contained DM motor protocol over SocketCAN.

No dependency on damiao_socketcan or any arm-specific library.
Uses only Python stdlib + socket (Linux SocketCAN).
"""

from .transport import CanTransport, CanMode, CanFrame
from .protocol import (
    pack_mit_frame,
    unpack_status_frame,
    pack_command_frame,
    pack_refresh_frame,
    pack_read_param_frame,
    pack_write_param_frame,
    pack_save_param_frame,
    unpack_param_response,
    float_to_uint,
    uint_to_float,
)
from .motor import MotorState, MotorType, MotorParams
from .controller import MotorController

__all__ = [
    "CanTransport",
    "CanMode",
    "CanFrame",
    "pack_mit_frame",
    "unpack_status_frame",
    "pack_command_frame",
    "pack_refresh_frame",
    "pack_read_param_frame",
    "pack_write_param_frame",
    "pack_save_param_frame",
    "unpack_param_response",
    "float_to_uint",
    "uint_to_float",
    "MotorState",
    "MotorType",
    "MotorParams",
    "MotorController",
]
