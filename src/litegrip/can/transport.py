"""Raw SocketCAN transport — no dependencies beyond Python stdlib + Linux SocketCAN.

Supports classic CAN (MTU 16) and CAN FD (MTU 72) with automatic mode
detection.  Provides send/receive with optional timeout.
"""

from __future__ import annotations

import fcntl
import logging
import select
import socket
import struct
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional

log = logging.getLogger("litegrip.can.transport")

# ── SocketCAN constants ──────────────────────────────────────────────────
CAN_RAW = getattr(socket, "CAN_RAW", 1)
CAN_RAW_FD_FRAMES = getattr(socket, "CAN_RAW_FD_FRAMES", 5)
CAN_RAW_FILTER = getattr(socket, "CAN_RAW_FILTER", 1)
SOL_CAN_RAW = getattr(socket, "SOL_CAN_RAW", 101)
CAN_SFF_MASK = 0x7FF
CAN_EFF_MASK = 0x1FFFFFFF

# struct can_filter { canid_t can_id; canid_t can_mask; } — two native uint32.
_CAN_FILTER_FMT = "=II"

# Classic CAN frame: can_id(4B) + dlc(1B) + padding(3B) + data(8B) = 16B
CAN_FRAME_FMT = "=IB3x8s"
CAN_MTU = 16

# CAN FD frame: can_id(4B) + dlc(1B) + flags(1B) + len8_flag(1B) + reserved(1B) + data(64B) = 72B
CANFD_FRAME_FMT = "=IBBBB64s"
CANFD_FRAME_SIZE = struct.calcsize(CANFD_FRAME_FMT)
CANFD_BRS = 0x01
CANFD_MTU = 72

_SIOCGIFMTU = 0x8921


def _iface_mtu(sock: socket.socket, iface: str) -> Optional[int]:
    """Read interface MTU (16=classic CAN, 72=CAN FD). Returns None on failure."""
    try:
        ifr = struct.pack("16sH", iface.encode("utf-8")[:15], 0)
        res = fcntl.ioctl(sock.fileno(), _SIOCGIFMTU, ifr)
        return struct.unpack("16sH", res)[1]
    except OSError:
        return None


class CanMode(IntEnum):
    """CAN interface mode."""
    CAN = 0
    CANFD = 1


@dataclass
class CanFrame:
    """A single CAN frame."""
    can_id: int
    data: bytes
    is_extended: bool = False
    is_fd: bool = False
    timestamp: float = 0.0

    @property
    def dlc(self) -> int:
        return len(self.data)


class CanTransport:
    """SocketCAN transport — opens a CAN socket and provides send/recv.

    Usage::

        transport = CanTransport("can0")
        transport.open()
        transport.send(0x108, bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]))
        frame = transport.recv(timeout_s=0.1)
        transport.close()
    """

    def __init__(self, channel: str, mode: CanMode = CanMode.CAN):
        self._channel = channel
        self._mode = mode
        self._sock: Optional[socket.socket] = None
        self._is_open = False
        # Set of CAN IDs (SFF) we want the kernel to deliver. Empty = accept
        # all (no hardware filter). Populated via set_id_filter() once the
        # motor's mst_id is known, so that on a shared bus (e.g. a full arm
        # chattering on other IDs) foreign frames don't flood the RX buffer
        # and starve/drop the motor's status frames.
        self._accept_ids: set = set()

    # ── properties ──────────────────────────────────────────────────────

    @property
    def channel(self) -> str:
        return self._channel

    @property
    def mode(self) -> CanMode:
        return self._mode

    @property
    def is_open(self) -> bool:
        return self._is_open

    # ── open / close ────────────────────────────────────────────────────

    def open(self) -> None:
        """Open and bind the CAN socket.

        Raises:
            OSError: if the interface does not exist or is down.
        """
        if self._is_open:
            return

        sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, CAN_RAW)

        # Check interface MTU.  Some USB-CAN adapters (gs_usb) and Linux
        # kernel versions require CAN FD socket mode on FD-capable
        # interfaces — classic CAN sockets may silently drop frames.
        # Match the old damiao_socketcan behaviour: upgrade CAN→CANFD
        # when the interface is FD, downgrade CANFD→CAN when not.
        mtu = _iface_mtu(sock, self._channel)
        if mtu is not None:
            if mtu >= CANFD_MTU and self._mode == CanMode.CAN:
                log.info(
                    "Interface %s is CAN FD (MTU=%d); switching to CANFD mode.",
                    self._channel, mtu,
                )
                self._mode = CanMode.CANFD
            elif mtu < CANFD_MTU and self._mode == CanMode.CANFD:
                log.warning(
                    "Interface %s MTU is %d (classic CAN) but CanMode.CANFD "
                    "requested; switching to classic CAN.",
                    self._channel, mtu,
                )
                self._mode = CanMode.CAN

        if self._mode == CanMode.CANFD:
            try:
                sock.setsockopt(SOL_CAN_RAW, CAN_RAW_FD_FRAMES, 1)
            except OSError:
                log.warning("Failed to enable CAN FD on %s; falling back to classic CAN.", self._channel)
                self._mode = CanMode.CAN

        try:
            socket.if_nametoindex(self._channel)  # verify interface exists
        except OSError as e:
            sock.close()
            raise OSError(f"CAN interface '{self._channel}' not found: {e}")

        sock.bind((self._channel,))
        sock.setblocking(False)

        self._sock = sock
        self._is_open = True
        # Re-apply any ID filter requested before the socket existed.
        if self._accept_ids:
            self._apply_id_filter()
        log.info("CAN transport opened on %s (mode=%s)", self._channel, self._mode.name)

    # ── receive filtering ─────────────────────────────────────────────────

    def set_id_filter(self, can_ids) -> None:
        """Restrict which CAN IDs the kernel delivers to this socket.

        On a shared bus (e.g. a full arm), other devices chatter on their own
        IDs at high rate. Without a hardware filter every one of those frames
        lands in this socket's RX buffer; a caller that reads one frame per
        control cycle cannot keep up, the buffer saturates, and the motor's
        own status frames are delayed or dropped — making read-back appear
        "frozen" while the motor is actually moving.

        Args:
            can_ids: Iterable of SFF CAN IDs to accept. Empty/None clears the
                     filter (accept all frames again).
        """
        self._accept_ids = {int(i) & CAN_SFF_MASK for i in (can_ids or [])}
        if self._is_open:
            self._apply_id_filter()

    def add_id_filter(self, can_id: int) -> None:
        """Add a single CAN ID to the accept set (see :meth:`set_id_filter`)."""
        self.set_id_filter(self._accept_ids | {int(can_id) & CAN_SFF_MASK})

    def _apply_id_filter(self) -> None:
        """Push the current accept-id set to the socket via CAN_RAW_FILTER.

        An empty set installs a zero-length filter list, which SocketCAN
        interprets as "receive nothing" — so instead we remove the filter by
        installing a single match-all entry (id=0, mask=0).
        """
        if self._sock is None:
            return
        if self._accept_ids:
            fdata = b"".join(
                struct.pack(_CAN_FILTER_FMT, cid, CAN_SFF_MASK)
                for cid in sorted(self._accept_ids)
            )
        else:
            fdata = struct.pack(_CAN_FILTER_FMT, 0, 0)  # match-all
        try:
            self._sock.setsockopt(SOL_CAN_RAW, CAN_RAW_FILTER, fdata)
            log.info("CAN RX filter set on %s: %s", self._channel,
                     [hex(i) for i in sorted(self._accept_ids)] or "all")
        except OSError as e:
            log.warning("Failed to set CAN RX filter on %s: %s", self._channel, e)

    def close(self) -> None:
        """Close the CAN socket."""
        if not self._is_open or self._sock is None:
            return
        try:
            self._sock.close()
        except OSError:
            pass
        self._sock = None
        self._is_open = False

    # ── send ────────────────────────────────────────────────────────────

    def send(self, can_id: int, data: bytes, timeout: float = 0.1) -> None:
        """Send a CAN frame.

        Args:
            can_id: 11-bit or 29-bit CAN ID.
            data: Payload (0–8 bytes classic, 0–64 bytes FD).
            timeout: Max wait time if send buffer is full (seconds).

        Raises:
            OSError: on send failure after timeout.
        """
        if not self._is_open or self._sock is None:
            raise RuntimeError("Transport not open")

        if self._mode == CanMode.CANFD:
            flags = 0
            frame = struct.pack(CANFD_FRAME_FMT,
                                can_id & CAN_EFF_MASK,
                                len(data), flags, 0, 0,
                                data.ljust(64, b'\x00'))
        else:
            if len(data) > 8:
                raise ValueError(f"Classic CAN frames max 8 bytes, got {len(data)}")
            frame = struct.pack(CAN_FRAME_FMT,
                                can_id & CAN_SFF_MASK,
                                len(data),
                                data.ljust(8, b'\x00'))

        # Retry on BlockingIOError (send buffer full under high load).
        deadline = time.monotonic() + timeout
        while True:
            try:
                self._sock.send(frame)
                return
            except BlockingIOError:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    log.warning(
                        "CAN send timeout on %s (buffer full >%.1fs), "
                        "frame to 0x%03X dropped",
                        self._channel, timeout, can_id & CAN_SFF_MASK,
                    )
                    return
                select.select([], [self._sock], [], remaining)

    def send_multi(self, can_id: int, data: bytes, count: int,
                   interval_s: float = 0.002) -> None:
        """Send the same frame multiple times (for reliability of config commands)."""
        for _ in range(count):
            self.send(can_id, data)
            if count > 1 and interval_s > 0:
                time.sleep(interval_s)

    # ── recv ────────────────────────────────────────────────────────────

    def recv(self, timeout_s: float = 0.0) -> Optional[CanFrame]:
        """Receive a single CAN frame.

        Args:
            timeout_s: Max wait time in seconds. 0 = non-blocking poll.

        Returns:
            CanFrame if a frame was received, None if timeout.
        """
        if not self._is_open or self._sock is None:
            return None

        try:
            ready, _, _ = select.select([self._sock], [], [], timeout_s)
            if not ready:
                return None
        except (OSError, ValueError):
            return None

        try:
            raw = self._sock.recv(CANFD_FRAME_SIZE)
        except OSError:
            return None

        ts = time.monotonic()

        if self._mode == CanMode.CANFD:
            can_id, dlc, flags, _len8, _res, data = struct.unpack(
                CANFD_FRAME_FMT, raw)
            is_fd = True
            is_extended = bool(can_id & CAN_EFF_MASK)
            data = data[:dlc]
        else:
            can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, raw)
            is_fd = False
            is_extended = bool(can_id & CAN_EFF_MASK)
            data = data[:dlc]

        return CanFrame(
            can_id=can_id & CAN_SFF_MASK,
            data=data,
            is_extended=is_extended,
            is_fd=is_fd,
            timestamp=ts,
        )

    def drain(self) -> None:
        """Read and discard all pending frames from the receive buffer."""
        while self.recv(timeout_s=0.0) is not None:
            pass

    # ── context manager ─────────────────────────────────────────────────

    def __enter__(self) -> "CanTransport":
        self.open()
        return self

    def __exit__(self, *args) -> None:
        self.close()

    def __repr__(self) -> str:
        state = "open" if self._is_open else "closed"
        return f"CanTransport({self._channel!r}, {self._mode.name}, {state})"
