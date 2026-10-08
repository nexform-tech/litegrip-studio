"""LiteGrip SDK — communication protocols.

The CAN bus layer (LiteGripCAN) is self-contained: it implements the
Damiao motor protocol directly over Linux SocketCAN with zero
dependencies on damiao_socketcan or any arm-specific library.
"""

from .can_bus import LiteGripCAN

__all__ = ["LiteGripCAN"]
