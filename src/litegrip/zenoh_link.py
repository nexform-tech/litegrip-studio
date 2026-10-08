"""Pure point-to-point zenoh link for gripper teleoperation.

This is a port of ``litearm-teleop-isomorphic/liteteleop/link.py`` — the transport
structure the arm teleoperation runs on in the field today.  It is not a new
design; the function names, the config keys and the endpoint roles are kept
identical so the two can be compared line by line.  What is new here is only the
:class:`ZenohTeleopTransport` adapter, which presents the same behaviour through
this SDK's :class:`~litegrip.TeleopTransport` interface.

Both ends run ``mode="peer"`` with **all discovery off** (no multicast scouting,
no gossip): the only way two peers find each other is the explicit
``listen``/``connect`` endpoints given here.  The leader listens, the follower
connects.

Two rules worth losing a session over:

* **The process will not exit until ``close()`` is called.**  This was measured,
  not assumed — an un-closed session leaves the interpreter hanging and only
  ``timeout`` kills it.
* **The topic is shared with the litearm arm/gripper stack**
  (``litearm/v4/{id}/gripper_teleop``).  The frame *format* is byte-identical to
  that stack, so the two interoperate — deliberately.  Sharing a topic across
  divergent formats would decode silently wrong, which is why the format is
  pinned in :mod:`litegrip.teleop`.

zenoh is an optional dependency: install it with ``pip install litegrip[zenoh]``.
This module is imported lazily so the base SDK still works without it.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Callable, Optional

import zenoh

from .teleop import (DEFAULT_GRIP_PORT, TeleopError, TeleopSubscription,
                     TeleopTransport)

log = logging.getLogger("litegrip.zenoh")

__all__ = ["DEFAULT_GRIP_PORT", "Listener", "Connector", "LatestSlot",
           "ZenohTeleopTransport"]


def _base_config() -> zenoh.Config:
    """The shared point-to-point config: **every discovery mechanism off**."""
    c = zenoh.Config()
    c.insert_json5("scouting/multicast/enabled", "false")
    c.insert_json5("scouting/gossip/enabled", "false")
    c.insert_json5("mode", '"peer"')
    return c


class _Endpoint:
    """Common lifecycle: an idempotent :meth:`close` that must be called."""

    def __init__(self) -> None:
        self._session: Optional[zenoh.Session] = None
        self._closed = False

    def close(self) -> None:
        """Release the session.  Idempotent, but **must** be called — without it
        the process never exits."""
        if self._closed:
            return
        self._closed = True
        s, self._session = self._session, None
        if s is not None:
            try:
                s.close()
            except Exception:                       # noqa: BLE001 - exit path never raises
                pass

    def __enter__(self):
        return self

    def __exit__(self, *exc) -> bool:
        self.close()
        return False


class Listener(_Endpoint):
    """Leader side: listen on a port and wait for a follower to connect."""

    def __init__(self, port: int, key: str) -> None:
        super().__init__()
        cfg = _base_config()
        cfg.insert_json5("listen/endpoints", f'["tcp/0.0.0.0:{int(port)}"]')
        cfg.insert_json5("connect/endpoints", "[]")
        self._session = zenoh.open(cfg)
        self._pub = self._session.declare_publisher(key)
        self._matching = False

        def _on_match(status) -> None:
            self._matching = bool(status.matching)

        self._pub.declare_matching_listener(_on_match)

    def put(self, payload: bytes) -> None:
        """Publish one frame.  Non-blocking."""
        self._pub.put(payload)

    @property
    def matching(self) -> bool:
        """Whether any subscriber is matched.

        This is a **boolean, not a count** — ``zenoh.MatchingStatus`` exposes only
        ``.matching``.  The UI shows "matched / not matched".
        """
        return self._matching


class Connector(_Endpoint):
    """Follower side: connect to the leader's host:port and subscribe to its stream.

    ``on_frame`` is called on **zenoh's own thread** ⇒ it must only write a
    latest slot.  Blocking work there (e.g. a ``goto_rad`` that waits for an ACK)
    stalls the zenoh thread.
    """

    def __init__(self, host: str, port: int, key: str,
                 on_frame: Optional[Callable[[bytes], None]] = None) -> None:
        super().__init__()
        cfg = _base_config()
        cfg.insert_json5("listen/endpoints", "[]")
        cfg.insert_json5("connect/endpoints", f'[\"tcp/{host}:{int(port)}\"]')
        self._session = zenoh.open(cfg)
        cb = on_frame or (lambda _b: None)
        #: Read-only frame counter, for the status snapshot.
        #: Its safety comes from *a single subscriber's callbacks running serially
        #: on one zenoh thread*, not from ``+=`` being atomic under the GIL —
        #: attribute ``+=`` is not an atomic bytecode sequence.
        self.received = 0

        def _handler(sample) -> None:
            self.received += 1
            cb(bytes(sample.payload))

        self._sub = self._session.declare_subscriber(key, _handler)

    def latest(self) -> int:
        """Frames received so far (used to display a rate)."""
        return self.received


class LatestSlot:
    """A latest-wins slot — the only hand-off between the zenoh callback and the
    servo loop.

    Only the **newest** frame is kept: a late frame overwrites, nothing queues.
    Teleoperation wants the latest pose only; a backlog would make the follower
    chase a stale trajectory.

    ⚠ **"Never received" and "just received" must be distinguishable.**  An
    earlier version used ``0.0`` as the "never received" sentinel, so once
    ``take()`` cleared the timestamp ``peek_age()`` reported "never received"
    right after a frame was taken ⇒ a caller that takes a frame and then asks its
    age is instantly judged dead and drops into HOLDING, triggering a blocking
    realign — a single read order could stall the follower mid-motion.  Now
    ``_last_recv_ts is None`` means "never received", and ``take()`` clears only
    the payload, never the timestamp.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._payload: Optional[bytes] = None
        self._last_recv_ts: Optional[float] = None      # None = never received
        self.dropped = 0

    def put(self, payload: bytes, now: float) -> None:
        with self._lock:
            if self._payload is not None:
                self.dropped += 1
            self._payload = payload
            self._last_recv_ts = now

    def take(self):
        """Take the newest payload, returning ``(payload_or_None, last_recv_ts_or_None)``.

        ⚠ **The timestamp is left alone** — it is the fact of "when we last
        received", which taking the payload must not erase.
        """
        with self._lock:
            p = self._payload
            self._payload = None
            return p, self._last_recv_ts

    @property
    def ever_received(self) -> bool:
        """Whether *any* frame ever arrived.

        The "no first frame within N seconds of start" diagnostic needs this, and
        it is a **different question** from "the steady-state watchdog timed out"
        (the latter only applies after a first frame).
        """
        with self._lock:
            return self._last_recv_ts is not None

    def peek_age(self, now: float) -> Optional[float]:
        """Local seconds since the newest frame; **``None`` if never received**.

        ⚠ Returning ``None`` rather than ``0.0`` is deliberate: ``0.0`` is a
        *legitimate* age ("just arrived"), so using it for "never received" would
        void the diagnostic above.
        ⚠ The result is clamped to ``≥ 0``: the loop may read ``now`` and then a
        frame carrying a larger ``now`` lands, making the difference negative —
        which is a hair away from a spurious watchdog trip.
        ⚠ This is a **local** time difference, unrelated to the frame's own ``ts``
        (the leader's clock); the two machines' clocks are not a common source.
        """
        with self._lock:
            t = self._last_recv_ts
        if t is None:
            return None
        return max(0.0, now - t)


class _SlotSubscription(TeleopSubscription):
    """A :class:`LatestSlot` presented as a pull-style subscription."""

    def __init__(self, slot: LatestSlot) -> None:
        self._slot = slot

    def try_recv(self) -> Optional[bytes]:
        payload, _ts = self._slot.take()
        return payload

    @property
    def ever_received(self) -> bool:
        return self._slot.ever_received

    def peek_age(self, now: float) -> Optional[float]:
        return self._slot.peek_age(now)

    @property
    def dropped(self) -> int:
        return self._slot.dropped


class ZenohTeleopTransport(TeleopTransport):
    """A :class:`~litegrip.TeleopTransport` carried over the point-to-point zenoh link.

    One instance is one endpoint: ``role="master"`` wraps a :class:`Listener`
    (publish only), ``role="slave"`` wraps a :class:`Connector` (subscribe only).
    The follower's frames land in a :class:`LatestSlot` and are drained by the
    servo loop, so the zenoh callback never blocks.

    ⚠ The leader's :class:`Listener` must be **resident across teleoperation
    sessions**: rebuilding it per session both leaves the port bound and makes
    publisher↔subscriber matching fail intermittently (5 of 12 restarts when
    rebuilt; 0 of 12 when resident).  The follower's :class:`Connector` is
    per-session and must be closed when the session ends — see
    :meth:`~litegrip.LiteGrip.teleop_stop`.

    Args:
        role: ``"master"`` (listen/publish) or ``"slave"`` (connect/subscribe).
        key: The zenoh topic, normally ``teleop_topic(grip_id)``.
        port: TCP port.  The leader listens on it, the follower connects to it.
        host: Follower only — the leader's address.  Ignored by the master.
    """

    def __init__(self, role: str, key: str, port: int = DEFAULT_GRIP_PORT,
                 host: Optional[str] = None) -> None:
        if role not in ("master", "slave"):
            raise ValueError(f"role must be 'master' or 'slave', got {role!r}")
        if role == "slave" and not host:
            raise ValueError("host is required for the slave (connecting) end")
        self._role = role
        self._key = key
        self._endpoint: Optional[_Endpoint] = None
        self._slot: Optional[LatestSlot] = None
        self._sub: Optional[_SlotSubscription] = None

        if role == "master":
            self._endpoint = Listener(int(port), key)
        else:
            self._slot = LatestSlot()

            def _on_wire(payload: bytes) -> None:
                # Wrapped, not ``slot.put`` directly: the callback has one
                # parameter, ``put`` needs two.  Runs on zenoh's thread ⇒ only
                # the slot is touched.
                self._slot.put(payload, time.monotonic())

            self._endpoint = Connector(host, int(port), key, on_frame=_on_wire)

    @property
    def matching(self) -> bool:
        """Master only — whether a subscriber is matched."""
        if isinstance(self._endpoint, Listener):
            return self._endpoint.matching
        return False

    @property
    def received(self) -> int:
        """Slave only — frames received."""
        if isinstance(self._endpoint, Connector):
            return self._endpoint.received
        return 0

    def pub(self, topic: str, payload: bytes) -> None:
        if self._role != "master":
            raise TeleopError("this zenoh transport subscribes; it cannot publish")
        self._check_topic(topic)
        self._endpoint.put(payload)

    def sub(self, topic: str) -> TeleopSubscription:
        if self._role != "slave":
            raise TeleopError("this zenoh transport publishes; it cannot subscribe")
        self._check_topic(topic)
        if self._sub is None:
            self._sub = _SlotSubscription(self._slot)
        return self._sub

    def _check_topic(self, topic: str) -> None:
        # The zenoh key is fixed when the endpoint is opened; a mismatched topic
        # here means two ends were wired to different keys, which would otherwise
        # look like "the leader just never publishes".
        if topic != self._key:
            raise TeleopError(
                f"topic {topic!r} does not match this transport's key {self._key!r}")

    def close(self) -> None:
        endpoint, self._endpoint = self._endpoint, None
        if endpoint is not None:
            endpoint.close()
