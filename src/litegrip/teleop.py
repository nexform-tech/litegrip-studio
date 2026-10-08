"""Leader/follower gripper teleoperation — single-DOF position mirroring.

Ported from ``litearm_device.gripper_teleop`` (the implementation that drove the
same feature inside the litearm server stack), minus the parts that only made
sense there.  The algorithm is unchanged; what is new is that the transport is
pluggable and the module depends on nothing outside the standard library, so a
bare ``LiteGrip`` on a CAN bus is all it takes.

Topology::

    leader  (zero-gravity, hand-back-driven)  --pub-->  follower (MIT follow)

``master`` streams zero-torque frames so the jaws can be pushed by hand, and
publishes its normalised opening at ``rate_hz``.  ``slave`` subscribes, aligns
once, then streams MIT position frames toward the received opening.

Why the wire carries ``openness`` and not radians
-------------------------------------------------
Each gripper has its own zero, direction and calibration (one unit opens at
-1.42 rad, another at +1.14 rad), so a raw angle is meaningless on the far
side.  ``openness`` is the opening normalised by the *local* travel and is
therefore dimensionless and direction-free; each side converts on its own.

Frame layout (big-endian, four doubles, 32 bytes) — byte-compatible with the
litearm implementation so the two can interoperate::

    openness[0..1] | position_mm | force_n | timestamp

Safety notes
------------
* ``openness`` is clamped to ``[0, 1]``, which keeps every commanded target
  inside the calibrated travel.  That clamp bounds *where* the follower may go;
  ``torque_limit_nm`` bounds *how hard* it may push.
* When ``torque_limit_nm`` is set (``0``, the default, disables it) the follower
  watches its own torque and releases when it stays over the limit for
  :data:`TORQUE_TRIP_CYCLES` cycles — see
  :meth:`GripperTeleop._update_torque_guard`.  Position is deliberately not used
  for this: a jam partway through the travel is indistinguishable from slow
  motion, and the calibrated travel comes from two hand-measured limits, which
  makes it the least trustworthy number in the loop.
* A **non-finite** frame (NaN / ±inf) is *dropped*, never clamped.  ``_clamp01``
  passes NaN through and ``min(hi, NaN)`` returns ``hi``, so clamping a NaN
  target silently commands the follower to its closed stop.  Bad readings are
  rejected at the wire boundary on both ends; the follower then holds, which is
  the safe side.
* A ``slave`` whose leader goes quiet **holds** its last target at the follow
  gains (it does not relax to zero torque).  The jaws therefore keep pressing
  whatever is between them — the same behaviour as the litearm original.  A
  tripped torque guard overrides that and stays released.
* Teleoperation is exclusive: stop any motion you started elsewhere before
  calling :meth:`~litegrip.LiteGrip.teleop_start`.
* :class:`UdpTeleopTransport` is plain, unauthenticated UDP.  Use it only on a
  trusted network.  For real deployments use the point-to-point zenoh link in
  :mod:`litegrip.zenoh_link` (``pip install litegrip[zenoh]``), which is what the
  field-validated litearm teleoperation runs on.
"""

from __future__ import annotations

import logging
import math
import socket
import struct
import threading
import time
from collections import deque
from typing import Any, Callable, Dict, Optional, Tuple, Union

from .exceptions import LiteGripError

log = logging.getLogger("litegrip.teleop")

# ── Frame codec ───────────────────────────────────────────────────────────

_FRAME = struct.Struct(">4d")

#: Size of one teleop frame in bytes (four doubles).
FRAME_SIZE = _FRAME.size

#: Default grip id and TCP port.  The topic and the port come from the litearm
#: teleoperation namespace: the arm uses ``armA``/17447, the gripper
#: ``gripA``/17448, so one machine can run both without a collision.
DEFAULT_GRIP_ID = "gripA"
DEFAULT_GRIP_PORT = 17448

#: Default ceiling on the leader velocity the follower feeds forward, in rad/s.
#: The wire frame carries no velocity, so the follower recovers one by finite
#: difference (see :meth:`GripperTeleop._estimate_dq`); this bounds what a bad
#: estimate can demand.  A hand sweep is well under 5 rad/s, and the DM4310's own
#: ``dq`` range is ±30, so this is generous for motion and tight for a glitch.
DEFAULT_DQ_MAX = 10.0

#: Longest interval a finite-difference velocity is trusted over, in seconds.
#: Past this the "velocity" would be an average across a dropout — refuse it and
#: fall back to position-only control for that cycle.
MAX_FRAME_GAP_S = 0.05

#: Default ceiling on the follower's own torque, in Nm.  ``0`` disables the guard,
#: which is the default: it changes the follower's behaviour under load, so a
#: caller opts in rather than inheriting it.
#:
#: Exceeding the limit means the jaws are pressing harder than a normal grasp,
#: which on a printed jaw or mount is how parts break.  The status frame reports
#: torque (there is no raw milliamp field); the motor derives it from its coil
#: current, so that is the current signal to judge on.  Pick the value from
#: observed torque on the machine: the 0.5 Nm ``grasp_torque_threshold`` and the
#: 2.0 Nm calibration probe ceiling bracket the useful range, and the follow
#: gains set how much position error that much torque corresponds to.
DEFAULT_TORQUE_LIMIT_NM = 0.0

#: Consecutive over-limit cycles before the guard trips.  At the 50 Hz default
#: this is 60 ms, which rides out a single noisy sample without leaving a real
#: jam pressing for long.
TORQUE_TRIP_CYCLES = 3

#: Opening the leader must recover, in normalised openness, before a tripped
#: follower re-arms.  Without it the release would re-engage on the next cycle
#: and press again — the guard would chatter instead of letting go.  A trip
#: within this distance of the open stop has no travel left to reopen into, so
#: there the requirement is the open stop itself; see
#: :meth:`GripperTeleop._update_torque_guard`.
TORQUE_REARM_OPENNESS = 0.05


def encode_frame(openness: float, position_mm: float, force_n: float,
                 timestamp: float) -> bytes:
    """Pack one teleop frame.

    Args:
        openness: Normalised opening in ``[0, 1]`` (0 = closed, 1 = fully
            open).  The quantity that actually drives the follower.
        position_mm: Leader opening in mm — diagnostic only.
        force_n: Leader gripping force in N — diagnostic only.
        timestamp: Sender clock in seconds — diagnostic only; the follower
            judges liveness from its own receive time, not from this field.
    """
    return _FRAME.pack(float(openness), float(position_mm), float(force_n),
                       float(timestamp))


def decode_frame(payload: bytes) -> Tuple[float, float, float, float]:
    """Unpack a teleop frame into ``(openness, position_mm, force_n, timestamp)``.

    Raises:
        ValueError: ``payload`` is not exactly :data:`FRAME_SIZE` bytes.
    """
    if len(payload) != FRAME_SIZE:
        raise ValueError(
            f"teleop frame must be {FRAME_SIZE} bytes, got {len(payload)}")
    return _FRAME.unpack(payload)


def teleop_topic(grip_id: str = DEFAULT_GRIP_ID) -> str:
    """Topic the leader publishes and the follower subscribes to.

    ⚠ This is the **litearm teleoperation namespace**, shared with the arm and
    gripper teleoperation stacks: ``litearm/v4/{grip_id}/gripper_teleop``.  The
    frame format is byte-identical, so the two interoperate — deliberately.
    Both ends must agree on ``grip_id``; it defaults to ``"gripA"`` so a single
    pair needs no configuration.  Use a distinct id per pair when more than one
    teleoperation runs on the same transport.
    """
    return f"litearm/v4/{grip_id}/gripper_teleop"


# ── Transport abstraction ─────────────────────────────────────────────────


class TeleopSubscription:
    """A non-blocking subscription handle."""

    def try_recv(self) -> Optional[bytes]:
        """Return the next payload, or ``None`` if none is waiting."""
        raise NotImplementedError

    def drain_latest(self) -> Optional[bytes]:
        """Discard all but the newest queued payload and return it.

        Teleoperation only ever wants the latest sample, so a slow consumer
        skips history instead of replaying it.
        """
        latest = None
        while True:
            msg = self.try_recv()
            if msg is None:
                return latest
            latest = msg


class TeleopTransport:
    """Publish/subscribe transport between a leader and a follower.

    Implement this to carry teleop frames over anything (a different network
    stack, an in-process bus, ...).  Three implementations ship with the SDK:
    :class:`UdpTeleopTransport`, :class:`InProcTeleopTransport`, and — for real
    deployments — the point-to-point zenoh link in :mod:`litegrip.zenoh_link`.
    """

    def pub(self, topic: str, payload: bytes) -> None:
        raise NotImplementedError

    def sub(self, topic: str) -> TeleopSubscription:
        raise NotImplementedError

    def close(self) -> None:
        """Release any resources.  Idempotent."""


def _parse_addr(addr: Union[str, Tuple[str, int]]) -> Tuple[str, int]:
    if isinstance(addr, str):
        host, _, port = addr.rpartition(":")
        if not host or not port:
            raise ValueError(f"address must be 'host:port', got {addr!r}")
        return host, int(port)
    host, port = addr
    return str(host), int(port)


class _UdpSubscription(TeleopSubscription):
    def __init__(self, sock: socket.socket) -> None:
        self._sock = sock

    def try_recv(self) -> Optional[bytes]:
        try:
            data, _ = self._sock.recvfrom(2048)
            return data
        except (BlockingIOError, InterruptedError):
            return None
        except OSError as e:
            log.debug("udp recv failed: %s", e)
            return None


class UdpTeleopTransport(TeleopTransport):
    """Plain UDP transport — the zero-dependency default.

    The leader sends frames to ``pub_addr``; the follower receives on
    ``bind_addr``.  Either or both may be given, so one object can both send
    and receive (not needed for a single leader/follower pair).  Same machine:
    ``"127.0.0.1:17448"``.  Across machines: the peer's real address,
    ``"0.0.0.0:<port>"`` to accept on every interface.

    Unauthenticated and unencrypted — trusted networks only.  A dropped
    datagram is simply the next sample being late, which the follower's
    watchdog already tolerates.
    """

    def __init__(self, pub_addr: Union[str, Tuple[str, int], None] = None,
                 bind_addr: Union[str, Tuple[str, int], None] = None) -> None:
        self._pub_addr = _parse_addr(pub_addr) if pub_addr is not None else None
        self._sock: Optional[socket.socket] = None
        self._sub: Optional[_UdpSubscription] = None

        if bind_addr is not None:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(_parse_addr(bind_addr))
            sock.setblocking(False)
            self._sock = sock
        elif self._pub_addr is not None:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        if self._pub_addr is None and bind_addr is None:
            raise ValueError("UdpTeleopTransport needs pub_addr, bind_addr or both")

    def pub(self, topic: str, payload: bytes) -> None:
        if self._pub_addr is None or self._sock is None:
            return
        try:
            self._sock.sendto(payload, self._pub_addr)
        except OSError as e:
            log.debug("udp send failed: %s", e)

    def sub(self, topic: str) -> TeleopSubscription:
        if self._sock is None:
            raise TeleopError(
                "UdpTeleopTransport was built without bind_addr; it cannot subscribe")
        if self._sub is None:
            self._sub = _UdpSubscription(self._sock)
        return self._sub

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        self._sub = None


class _InProcSubscription(TeleopSubscription):
    def __init__(self, queue: "deque[bytes]") -> None:
        self._queue = queue

    def try_recv(self) -> Optional[bytes]:
        try:
            return self._queue.popleft()
        except IndexError:
            return None


class InProcTeleopTransport(TeleopTransport):
    """In-process bus — for tests and for two grippers in one program.

    A single instance is shared by both ends; nothing crosses a process
    boundary.  Each topic keeps a bounded FIFO per subscriber, so a subscriber
    that falls behind skips frames rather than growing without bound.
    """

    def __init__(self, fifo_depth: int = 16) -> None:
        self._depth = fifo_depth
        self._queues: Dict[str, list] = {}
        self._lock = threading.Lock()

    def pub(self, topic: str, payload: bytes) -> None:
        with self._lock:
            for queue in self._queues.get(topic, []):
                queue.append(payload)
                while len(queue) > self._depth:
                    queue.popleft()

    def sub(self, topic: str) -> TeleopSubscription:
        queue: "deque[bytes]" = deque()
        with self._lock:
            self._queues.setdefault(topic, []).append(queue)
        return _InProcSubscription(queue)

    def close(self) -> None:
        with self._lock:
            self._queues.clear()


# ── openness <-> radians conversion ───────────────────────────────────────


def travel_mm(cfg: Any) -> float:
    """Full calibrated stroke in mm (``|open - closed| * rad_to_mm``)."""
    return abs(cfg.pos_open_rad - cfg.pos_closed_rad) * cfg.rad_to_mm


def rad_to_openness(position_rad: float, cfg: Any) -> float:
    """Motor angle -> normalised opening in ``[0, 1]``.

    Uses the same sign convention as :meth:`LiteGrip.get_state`, so it is
    correct for both mountings (``cfg.close_sign`` carries the direction).
    """
    stroke = travel_mm(cfg)
    if stroke <= 0.0:
        return 0.0
    position_mm = ((cfg.pos_closed_rad - position_rad)
                   * cfg.close_sign * cfg.rad_to_mm)
    return _clamp01(position_mm / stroke)


def openness_to_rad(openness: float, cfg: Any) -> float:
    """Normalised opening in ``[0, 1]`` -> motor angle.

    Mirrors :meth:`LiteGrip.goto_mm` including its ``close_sign`` factor, so a
    reverse-mounted follower moves the correct way.  ``openness`` is clamped
    to ``[0, 1]`` first, which bounds the target to the calibrated travel.
    """
    if cfg.rad_to_mm <= 0.0:
        return cfg.pos_closed_rad
    openness = _clamp01(openness)
    return (cfg.pos_closed_rad
            - cfg.close_sign * openness * travel_mm(cfg) / cfg.rad_to_mm)


def clamp_to_calibrated(cfg: Any, q: float) -> float:
    """Clamp a motor angle into the calibrated travel.

    Either limit may be the larger one (a reverse mount swaps them), so this
    takes ``min``/``max`` rather than assuming an order.  Same source as the
    SDK's own ``goto_rad`` clamp.
    """
    lo = min(float(cfg.pos_closed_rad), float(cfg.pos_open_rad))
    hi = max(float(cfg.pos_closed_rad), float(cfg.pos_open_rad))
    return max(lo, min(hi, float(q)))


def _clamp01(x: float) -> float:
    """Clamp to ``[0, 1]``.

    ⚠ **This does not sanitise NaN** — every comparison against NaN is false, so
    NaN passes straight through, and a later ``min(hi, NaN)`` yields ``hi``.
    Callers must reject non-finite values *before* clamping, not rely on the
    clamp to bound them.
    """
    return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)


# ── exceptions ────────────────────────────────────────────────────────────


class TeleopError(LiteGripError):
    """Base class for teleoperation errors."""


class TeleopBusyError(TeleopError):
    """Raised when teleop is started while it is already running."""


class TeleopNotActiveError(TeleopError):
    """Raised when an operation needs an active session but none is running."""


class TeleopNotReady(TeleopError):
    """Raised when the gripper cannot safely be teleoperated yet.

    ``send_mit_frame`` and ``goto_rad`` do **not** check ``calibrated`` — only
    ``open``/``close``/``grasp`` do.  Without this precheck an uncalibrated unit
    is driven from placeholder limits of unknown direction.
    """


def check_ready(cfg: Any) -> None:
    """Verify the calibration teleoperation depends on.  **Call before enable.**

    Raises:
        TeleopNotReady: uncalibrated, zero travel, or ``rad_to_mm == 0``.

    The zero-travel test is written in **rad space** (matching the SDK); testing
    ``travel_mm == 0`` in mm space is a strictly weaker condition.
    """
    if not bool(getattr(cfg, "calibrated", False)):
        raise TeleopNotReady(
            "gripper is not calibrated: pos_closed_rad / pos_open_rad are still "
            "placeholder defaults and the direction is a guess. Run "
            "load_calibration() / load_template(), or zero() first.")
    if abs(float(cfg.pos_closed_rad) - float(cfg.pos_open_rad)) <= 1e-6:
        raise TeleopNotReady(
            "zero travel: pos_closed_rad equals pos_open_rad — recalibrate.")
    if not float(cfg.rad_to_mm):
        raise TeleopNotReady(
            "rad_to_mm is 0: the openness<->radian conversion would divide by zero.")


# ── the algorithm ─────────────────────────────────────────────────────────


class GripperTeleop:
    """One side of a gripper teleoperation, driven by a background thread.

    ``mode="master"`` streams zero-torque frames (so the jaws can be moved by
    hand) and publishes the opening.  ``mode="slave"`` subscribes, aligns to
    the first frame with a single :meth:`~litegrip.LiteGrip.goto_rad`, then
    follows every fresh sample with ``send_mit_frame``, feeding the leader's
    finite-difference velocity forward as ``dq`` (:meth:`_estimate_dq`).

    One loop thread per side, sampling and sending in the same cycle — no
    shared buffers and no contention, which is all a single-DOF gripper needs
    (see the litearm original for the reasoning).

    Args:
        gripper: The ``LiteGrip`` this side drives.
        transport: Transport used to publish (master) and, unless
            ``sub_transport`` is given, to subscribe (slave).
        mode: ``"master"`` or ``"slave"``.
        topic: Topic to publish/subscribe.
        rate_hz: Loop rate.  ~50 Hz is plenty; the CAN frame stream and the
            publish share the same cycle.
        kp, kd: Follow gains (slave).  ``None`` uses the calibration's ``kp`` /
            ``kd``, falling back to ``100.0`` / ``2.0``.
        align: Slave only — align to the first frame before following.
        watchdog_s: Slave only — seconds without a fresh frame before the
            follower is considered stale and starts holding.  Must be > 0: a
            non-positive watchdog makes the follower permanently stale, which is
            a session that starts and then silently does nothing.
        dq_max: Slave only — ceiling in rad/s on the leader velocity fed forward
            as the follower's ``dq`` target.  ``0`` disables the feedforward, in
            which case the follower biases on position error alone and trails a
            moving leader.  See :meth:`_estimate_dq`.
        torque_limit_nm: Slave only — ceiling in Nm on the follower's own torque.
            Held over the limit for :data:`TORQUE_TRIP_CYCLES` cycles it
            releases in place and stays released until the leader reopens by
            :data:`TORQUE_REARM_OPENNESS`.  ``0`` disables the guard.  See
            :meth:`_update_torque_guard`.
        sub_transport: Slave only — a separate transport to subscribe on when
            the leader is remote (the master's transport is local-only).
        sleep_fn, time_fn: Timing seams for tests.  ``time_fn`` must be
            monotonic.
    """

    def __init__(
        self,
        gripper: Any,
        transport: TeleopTransport,
        mode: str,
        topic: str,
        rate_hz: float = 50.0,
        kp: Optional[float] = None,
        kd: Optional[float] = None,
        align: bool = True,
        watchdog_s: float = 0.2,
        dq_max: float = DEFAULT_DQ_MAX,
        torque_limit_nm: float = DEFAULT_TORQUE_LIMIT_NM,
        sub_transport: Optional[TeleopTransport] = None,
        sleep_fn: Callable[[float], None] = time.sleep,
        time_fn: Callable[[], float] = time.monotonic,
    ) -> None:
        if mode not in ("master", "slave"):
            raise ValueError(f"mode must be 'master' or 'slave', got {mode!r}")
        if not rate_hz > 0.0:
            raise ValueError("rate_hz must be > 0")
        if not watchdog_s > 0.0:
            raise ValueError("watchdog_s must be > 0")
        if float(torque_limit_nm) < 0.0:
            raise ValueError(
                f"torque_limit_nm must be >= 0 (0 disables the guard), "
                f"got {torque_limit_nm!r}")
        self._g = gripper
        self._tp = transport
        self._mode = mode
        self._topic = topic
        self._dt = 1.0 / rate_hz
        self._kp = kp
        self._kd = kd
        self._align = align
        self._watchdog_s = watchdog_s
        self._dq_max = float(dq_max)
        self._torque_limit_nm = float(torque_limit_nm)
        self._sub_tp = sub_transport
        self._sleep_fn = sleep_fn
        self._time_fn = time_fn

        self._thread: Optional[threading.Thread] = None
        self._running = False

        # Diagnostics.
        self._frames = 0
        self._last_openness = 0.0
        #: Last ``position_mm`` / ``force_n`` the session saw — the master reads
        #: them off its own state, the slave takes them from the frame it
        #: followed.  Reported by :meth:`status` so a caller can show what the
        #: jaws are doing without a second CAN reader (the teleop loop owns the
        #: bus while it runs).
        self._last_position_mm = 0.0
        self._last_force_n = 0.0
        self._last_frame_ts = 0.0
        #: Explicit "have we ever received a frame" flag.  The watchdog only
        #: applies after the first frame, and this must not be inferred from a
        #: ``0.0`` timestamp — see the ``LatestSlot`` note in the design spec.
        self._ever_received = False
        self._stale = False
        self._rejected = 0
        self._send_failed = 0
        self._fault = ""
        #: Torque guard (slave).  ``_torque_nm`` is the last torque read;
        #: ``_over_torque`` is latched once the limit is held for
        #: ``TORQUE_TRIP_CYCLES`` cycles and clears on re-arm.  See
        #: :meth:`_update_torque_guard`.
        self._torque_nm = 0.0
        self._over_torque = False
        self._torque_trips = 0
        self._torque_over_cycles = 0
        self._trip_openness = 0.0
        #: Leader velocity fed forward as this cycle's ``dq`` target, and the
        #: two samples it is differenced from.  Seeded on the first good frame;
        #: see :meth:`_estimate_dq`.
        self._dq_cmd = 0.0
        self._prev_q: Optional[float] = None
        self._prev_rx_ts: Optional[float] = None
        self._loops = 0
        self._loop_hz = 0.0
        self._hz_t0 = 0.0
        self._hz_n0 = 0

    # ── lifecycle ─────────────────────────────────────────────────────

    @property
    def is_running(self) -> bool:
        return self._running

    def start(self) -> None:
        """Start the background loop.  Raises :class:`TeleopBusyError` if it
        is already running."""
        if self._running:
            raise TeleopBusyError("teleop already running")
        self._running = True
        self._thread = threading.Thread(
            target=self._run, name=f"litegrip-teleop-{self._mode}", daemon=True)
        self._thread.start()
        log.info("teleop started: mode=%s topic=%s rate=%.0fHz",
                 self._mode, self._topic, 1.0 / self._dt)

    def _run(self) -> None:
        # Whatever ends the loop — a stop, a CAN error, a bad transport — the
        # session is no longer active once the thread returns.
        try:
            if self._mode == "master":
                self._master_loop()
            else:
                self._slave_loop()
        finally:
            self._running = False

    def stop(self, timeout: float = 2.0) -> None:
        """Stop the loop and leave the gripper holding its position.

        Neither side disables the motor: the master leaves zero-gravity mode and
        the slave sends one final frame at its current angle, so both hold under
        gain and whatever is between the jaws stays there.
        """
        self._running = False
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=timeout)
        self._thread = None
        self._handoff()
        log.info("teleop stopped: mode=%s frames=%d", self._mode, self._frames)

    def _handoff(self) -> None:
        """Leave the gripper holding position — never disable (§8 rule 4/6).

        A tripped torque guard is the exception: re-applying the follow gains
        would press the very thing the guard just let go of, so the final frame
        stays at zero torque.
        """
        try:
            if self._mode == "master":
                # Internally one frame at the current angle under the config
                # gains, which is exactly the hand-off we want.
                self._g.exit_zero_gravity()
            else:
                q = clamp_to_calibrated(
                    self._g.config, self._g.get_state(wait=False).position_rad)
                if self._over_torque:
                    self._send(q, 0.0, 0.0)
                else:
                    self._send(q, self._resolve_kp(), self._resolve_kd())
        except Exception as e:  # noqa: BLE001
            log.debug("hand-off on stop failed: %s", e)

    def status(self) -> dict:
        """A snapshot of the session, for logging and diagnostics."""
        age_ms = None
        if self._mode == "slave" and self._ever_received:
            age_ms = (self._time_fn() - self._last_frame_ts) * 1000.0
        matching = getattr(self._tp, "matching", None)
        return {
            "active": self._running,
            "mode": self._mode,
            "topic": self._topic,
            "frames": self._frames,
            "last_frame_age_ms": age_ms,
            "stale": self._stale,
            "openness": round(self._last_openness, 4),
            # Position and force behind that opening — the master's own state,
            # or (slave) the leader's values from the followed frame.
            "position_mm": round(self._last_position_mm, 4),
            "force_n": round(self._last_force_n, 4),
            "loop_hz": round(self._loop_hz, 1),
            # Slave only: the leader velocity last fed forward, in rad/s.
            "dq_cmd": round(self._dq_cmd, 4),
            # Frames dropped at the protocol boundary (non-finite values).
            "rejected": self._rejected,
            # ``send_mit_frame`` returned False — the motor is not following.
            "send_failed": self._send_failed,
            # The gripper's own error_code was not "enabled".
            "fault": self._fault,
            # Slave only: the follower's own torque (Nm), and whether the guard
            # has released it.  ``torque_trips`` counts trips this session.
            "torque_nm": round(self._torque_nm, 4),
            "over_torque": self._over_torque,
            "torque_trips": self._torque_trips,
            # Master only: whether a subscriber is matched.
            "matching": matching if isinstance(matching, bool) else None,
        }

    # ── master ────────────────────────────────────────────────────────

    def _master_loop(self) -> None:
        log.info("[master] zero-gravity, publishing to %s", self._topic)
        try:
            while self._running:
                t0 = self._time_fn()
                # Keep the frame stream alive: the DM motor self-locks a
                # "communication loss" fault ~100 ms after frames stop, so a
                # zero-torque frame goes out every cycle even though nothing
                # is being commanded.
                try:
                    self._send(0.0, 0.0, 0.0, quiet=True)
                    state = self._g.get_state(wait=False)
                except Exception:  # noqa: BLE001
                    log.exception("[master] CAN error; loop exiting")
                    break
                self._note_grip_fault(state)
                openness = rad_to_openness(state.position_rad, self._g.config)
                if not all(math.isfinite(v) for v in
                           (openness, state.position_mm, state.force_n)):
                    # Bad reading: publish nothing.  The follower's watchdog then
                    # times out and **holds**, which is the safe side.  Sending
                    # the frame would hand the follower a NaN target.
                    self._count_rejected(openness)
                else:
                    self._last_openness = openness
                    self._last_position_mm = float(state.position_mm)
                    self._last_force_n = float(state.force_n)
                    try:
                        self._tp.pub(self._topic, encode_frame(
                            openness, state.position_mm, state.force_n,
                            self._time_fn()))
                    except Exception as e:  # noqa: BLE001
                        log.debug("[master] publish failed: %s", e)
                    self._frames += 1
                self._sleep_rest(t0)
        finally:
            log.info("[master] loop exited (%d frames sent)", self._frames)

    # ── slave ─────────────────────────────────────────────────────────

    def _slave_loop(self) -> None:
        transport = self._sub_tp or self._tp
        sub = transport.sub(self._topic)
        cfg = self._g.config
        log.info("[slave] subscribed %s (align=%s watchdog=%.0fms)",
                 self._topic, self._align, self._watchdog_s * 1000.0)

        # Until a frame arrives, hold wherever the jaws already are.  A ``0.0``
        # target here would be a real position command — the open or the closed
        # stop, depending on the mount.
        q_cmd = clamp_to_calibrated(cfg, self._g.get_state(wait=False).position_rad)

        if self._align:
            first = self._wait_first_frame(sub, timeout_s=5.0, hold_q=q_cmd)
            if first is not None:
                q_cmd = clamp_to_calibrated(
                    cfg, openness_to_rad(_clamp01(first[0]), cfg))
                log.info("[slave] aligning to first frame: openness=%.3f -> %+.4f rad",
                         _clamp01(first[0]), q_cmd)
                try:
                    self._g.goto_rad(q_cmd, kp=self._resolve_kp(),
                                     kd=self._resolve_kd(), duration=1.0)
                except Exception as e:  # noqa: BLE001
                    log.warning("[slave] align goto_rad failed: %s", e)
                self._last_frame_ts = self._time_fn()
                self._ever_received = True
                self._last_openness = _clamp01(first[0])
                self._last_position_mm = float(first[1])
                self._last_force_n = float(first[2])
            else:
                log.warning("[slave] no frame within align timeout; "
                            "holding current position")

        try:
            while self._running:
                t0 = self._time_fn()
                # Only a fresh frame sets a velocity; a hold cycle is ``dq = 0``.
                dq_cmd = 0.0
                msg = sub.drain_latest()
                if msg is not None:
                    try:
                        openness, _mm, _force, _ts = decode_frame(msg)
                    except ValueError as e:
                        log.debug("[slave] ignoring bad frame: %s", e)
                    else:
                        # Protocol boundary: reject non-finite values.  Clamping
                        # a NaN target folds it onto a hard stop, silently.
                        if not all(math.isfinite(v) for v in (openness, _mm, _force)):
                            self._count_rejected(openness)
                        else:
                            rx_ts = self._time_fn()
                            q_cmd = openness_to_rad(_clamp01(openness), cfg)
                            dq_cmd = self._estimate_dq(q_cmd, rx_ts)
                            self._prev_q = q_cmd
                            self._prev_rx_ts = rx_ts
                            self._last_openness = _clamp01(openness)
                            self._last_position_mm = float(_mm)
                            self._last_force_n = float(_force)
                            self._last_frame_ts = rx_ts
                            self._ever_received = True
                            self._frames += 1
                            self._stale = False
                elif (self._ever_received
                      and (self._time_fn() - self._last_frame_ts) > self._watchdog_s):
                    if not self._stale:
                        log.warning("[slave] frames stale (>%.0fms); holding position",
                                    self._watchdog_s * 1000.0)
                    self._stale = True

                # Always send — including while stale.  The frame both holds
                # the position and keeps the motor from self-locking.
                q_cmd = clamp_to_calibrated(cfg, q_cmd)
                self._dq_cmd = dq_cmd
                try:
                    # Read the state first: this cycle's send is decided from
                    # this cycle's torque.  The read is the same cached poll the
                    # fault check always did, so the guard costs no extra traffic
                    # and runs at the loop rate.
                    state = self._g.get_state(wait=False)
                    self._note_grip_fault(state)
                    self._update_torque_guard(state)
                    if self._over_torque:
                        # Released in place: zero stiffness and damping, but the
                        # frames keep flowing so the motor does not latch its
                        # comm-loss fault.
                        self._send(q_cmd, 0.0, 0.0)
                    else:
                        self._send(q_cmd, self._resolve_kp(), self._resolve_kd(),
                                   dq=dq_cmd)
                except Exception:  # noqa: BLE001
                    log.exception("[slave] CAN error; loop exiting")
                    break
                self._sleep_rest(t0)
        finally:
            log.info("[slave] loop exited (%d frames received)", self._frames)

    # ── helpers ───────────────────────────────────────────────────────

    def _wait_first_frame(self, sub: TeleopSubscription, timeout_s: float,
                          hold_q: float) -> Optional[Tuple[float, float, float, float]]:
        """Wait for the first frame, holding position while we wait.

        ⚠ A **non-finite** frame is skipped and the wait continues, never
        returned: this value feeds ``goto_rad``, and a NaN folds onto an end
        stop — one bad frame would pull the follower to the closed limit, on the
        ``align=True`` default path, before the loop's own guard could see it.
        ⚠ The wait **keeps sending hold frames**: staying silent for up to 5 s
        contradicts this module's own "stop sending ⇒ lose force" rule, and the
        gripper may be holding something.
        """
        deadline = self._time_fn() + timeout_s
        while self._running and self._time_fn() < deadline:
            msg = sub.drain_latest()
            if msg is not None:
                try:
                    values = decode_frame(msg)
                except ValueError:
                    continue
                if all(math.isfinite(v) for v in values):
                    return values
                self._count_rejected(values[0])
            try:
                self._send(hold_q, self._resolve_kp(), self._resolve_kd())
                self._note_grip_fault(self._g.get_state(wait=False))
            except Exception:  # noqa: BLE001
                log.exception("[slave] CAN error during align; loop exiting")
                return None
            self._sleep_fn(0.01)
        return None

    def _estimate_dq(self, q: float, rx_ts: float) -> float:
        """Leader velocity in rad/s, differenced from the previous good frame.

        The gripper wire frame carries no velocity field, so the follower
        recovers one here and feeds it forward as its own ``dq`` target — this
        stands in for the ``dq`` the arm teleoperation sends outright.  Without
        it the follower biases on position error alone and visibly trails a
        moving leader (the lag scales with speed / ``kp``).

        Deliberately conservative: nothing to difference against on the first
        frame, a degenerate or dropout-sized interval is refused, and the result
        is clamped to ``dq_max``.  ``kd * (dq - dq_measured)`` is a real torque
        term, so an unbounded estimate could command a large one.
        """
        if (self._dq_max <= 0.0 or self._prev_q is None
                or self._prev_rx_ts is None):
            return 0.0
        dt = rx_ts - self._prev_rx_ts
        if not 1e-4 < dt < MAX_FRAME_GAP_S:
            return 0.0
        dq = (q - self._prev_q) / dt
        if not math.isfinite(dq):
            return 0.0
        return max(-self._dq_max, min(self._dq_max, dq))

    def _count_rejected(self, got: Any) -> None:
        """Record a frame dropped at the protocol boundary (§8 rule 9)."""
        self._rejected += 1
        if self._rejected == 1:
            log.warning("[%s] dropped a non-finite frame (openness=%r) — "
                        "holding position rather than folding onto a stop",
                        self._mode, got)

    def _send(self, q: float, kp: float, kd: float, dq: float = 0.0,
              quiet: bool = False) -> None:
        """Send one MIT frame and **check the return value** (§8 rule 10).

        ``dq`` is the follower's velocity target: zero for a hold (and for the
        master's slack frames), the leader's finite-difference velocity for a
        follower cycle — see :meth:`_estimate_dq`.

        ``send_mit_frame`` returns ``False`` when the motor is not enabled or the
        CAN write fails — it does not raise, so ignoring the result means
        believing we are driving a gripper that is not moving.
        """
        if not self._g.send_mit_frame(q=q, kp=kp, kd=kd, dq=dq):
            self._send_failed += 1
            if self._send_failed == 1 and not quiet:
                log.warning("[%s] send_mit_frame returned False — the motor is "
                            "not enabled or the CAN write failed; the gripper may "
                            "not be moving at all", self._mode)

    def _note_grip_fault(self, state: Any) -> None:
        """Consume the gripper's own ``error_code`` (§8 rule 10).

        Polling without reading ``error_code`` is the same as not polling: ``1``
        means enabled, ``0`` disabled, anything else is a real fault (over-temp,
        over-current).  Only the first one is reported.
        """
        code = int(getattr(state, "error_code", 1))
        if code != 1 and not self._fault:
            self._fault = (f"gripper reports error_code={code}"
                           + (" (disabled)" if code == 0 else " (fault)")
                           + " — still streaming hold frames")
            log.warning("[%s] %s", self._mode, self._fault)

    def _update_torque_guard(self, state: Any) -> None:
        """Release the follower when its torque stays over the limit.

        Torque is the follower's own reading, not the leader's ``force_n`` on the
        wire: the jaws press whatever is between them, and the status frame's
        ``tau`` (derived from coil current) is the only signal that sees it.
        Position is not used — a jam partway through the travel looks like slow
        motion, and the calibrated limits are the loop's least trustworthy
        numbers.

        Latching matters: releasing in place drops the torque to zero, so a
        non-latching guard would re-engage on the very next cycle and chatter.
        The follower re-arms only once the leader has reopened by
        :data:`TORQUE_REARM_OPENNESS`, i.e. once the operator has backed off.

        That target is capped at the open stop.  A trip within the margin of
        full open has no travel left to reopen into, and an uncapped target
        there is unreachable: the guard stayed off for the rest of the session
        while its own log line asked for a movement the stroke does not allow.
        Capped, such a trip re-arms when the leader reaches the open stop —
        the most back-off that exists at that end.

        ``torque_limit_nm == 0`` disables the guard entirely.
        """
        self._torque_nm = float(getattr(state, "torque_nm", 0.0) or 0.0)
        if self._torque_limit_nm <= 0.0:
            return
        if self._over_torque:
            reopen_needed = min(self._trip_openness + TORQUE_REARM_OPENNESS, 1.0)
            if self._last_openness >= reopen_needed:
                self._over_torque = False
                self._torque_over_cycles = 0
                log.info("[%s] torque guard re-armed (leader reopened to %.3f; "
                         "needed %.3f)", self._mode, self._last_openness,
                         reopen_needed)
            return
        if abs(self._torque_nm) >= self._torque_limit_nm:
            self._torque_over_cycles += 1
            if self._torque_over_cycles >= TORQUE_TRIP_CYCLES:
                self._over_torque = True
                self._torque_trips += 1
                self._trip_openness = self._last_openness
                log.warning(
                    "[%s] torque %.3f Nm >= limit %.3f Nm for %d cycles — "
                    "releasing in place (reopen the leader to %.2f to re-arm)",
                    self._mode, self._torque_nm, self._torque_limit_nm,
                    TORQUE_TRIP_CYCLES,
                    min(self._last_openness + TORQUE_REARM_OPENNESS, 1.0))
        else:
            self._torque_over_cycles = 0

    def _resolve_kp(self) -> float:
        """Follow gain: the explicit one, else the calibration's ``kp``."""
        if self._kp is not None:
            return float(self._kp)
        return float(getattr(self._g.config, "kp", 100.0))

    def _resolve_kd(self) -> float:
        if self._kd is not None:
            return float(self._kd)
        return float(getattr(self._g.config, "kd", 2.0))

    def _sleep_rest(self, t0: float) -> None:
        self._loops += 1
        now = self._time_fn()
        if self._hz_t0 == 0.0:
            self._hz_t0 = now
            self._hz_n0 = self._loops
        elif now - self._hz_t0 >= 1.0:
            self._loop_hz = (self._loops - self._hz_n0) / (now - self._hz_t0)
            self._hz_t0 = now
            self._hz_n0 = self._loops
        rest = self._dt - (self._time_fn() - t0)
        if rest > 0.0:
            self._sleep_fn(rest)
