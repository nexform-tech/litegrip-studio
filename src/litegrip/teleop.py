"""Leader/follower gripper teleoperation — single-DOF position mirroring.

Ported from ``litearm_device.gripper_teleop`` (the implementation that drove the
same feature inside the litearm server stack), minus the parts that only made
sense there.  The algorithm is unchanged; what is new is that the transport is
pluggable and the module depends on nothing outside the standard library, so a
bare ``LiteGrip`` on a CAN bus is all it takes.

Topology::

    leader  (zero-gravity, hand-back-driven)  --pub-->  follower (MIT follow)
    leader  (holds under gain until ready)    <--pub--  follower (at target)

``master`` publishes its normalised opening at ``rate_hz``.  ``slave``
subscribes, ramps once to the first frame's opening, then streams MIT position
frames toward the received opening.  On a transport that carries the reverse
direction the follower also announces when it has arrived
(:func:`ready_topic`), and the master holds its own jaws under gain — **not**
hand-movable — until it hears that.  A master that cannot subscribe to the ready
topic, or whose peer never announces, behaves as before: it goes slack
immediately, warning if it waited past :data:`DEFAULT_READY_TIMEOUT_S`.

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
  ``lead_cap_mm`` bounds *how hard* the align may push, and ``torque_limit_nm``
  is the backstop for the follow.
* The align move goes out through :meth:`GripperTeleop._ramp_to`, which caps how
  far each frame's command may lead the measured position
  (:meth:`GripperTeleop._cap_lead`).  Torque is ``kp * (q_cmd - q_measured)``, so
  bounding the lead bounds the commanded torque by construction — without it the
  first align frame demands ``kp`` times the whole error, and at the shipped
  ``kp`` of 100 Nm/rad anything past ~0.1 rad saturates the DM4310.  The *follow*
  loop is deliberately left uncapped so the follower stays responsive; the
  ``torque_limit_nm`` guard is what protects it under load.
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
* The leader's readiness gate is about **when it hands back**, never about
  withholding frames: it publishes from the first cycle either way, so a
  follower always has something to align to and neither end can block on the
  other.  ``require_ready=False`` disables the gate; ``ready_timeout_s=0`` waits
  indefinitely.  See :meth:`GripperTeleop._master_loop`.
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

# ── [遥操对齐块] 常量 ──────────────────────────────────────────────────────
# 遥操从机启动对齐的「定速斜坡 + 每帧领先上限」。整块的作用、清单，以及
# 临时停用 / 整块移除的做法，见 GripperTeleop._ramp_to() 上方那段说明。
# 用 `grep -n '\[遥操对齐块\]' src/litegrip/teleop.py src/litegrip/gripper.py`
# 可定位本块的全部位置。
# ──────────────────────────────────────────────────────────────────────────
#: Speed the follower travels at when it aligns to the leader's opening, in mm/s
#: of jaw travel.  It used to travel at whatever stiffness would get it there:
#: ``goto_rad(..., duration=1.0)`` sends ``q = q_target`` from the first frame,
#: so the whole error was commanded at once.  Same number
#: ``actions.MotionConfig.speed_mm_s`` gives ``open``/``close``.  Must be > 0.
DEFAULT_ALIGN_SPEED_MM_S = 50.0

#: Ceiling on how far the *align* command may lead the measured position, in mm
#: of jaw travel.  Torque is ``kp * (q_cmd - q_measured)``, so bounding the lead
#: bounds the commanded torque by construction.  4 mm is 0.054 rad at the shipped
#: ``rad_to_mm`` of ~83, i.e. ~5.4 Nm at ``kp`` 100 — the bracket ``open``/
#: ``close`` already travel with (``MotionConfig.max_lead_mm``).  ``0`` disables
#: the cap.  The follow loop does not use this: it commands the leader's opening
#: outright so it stays responsive.
DEFAULT_LEAD_CAP_MM = 4.0

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

#: Follower arrival tolerance, in mm of the follower's own travel.  The follower
#: announces itself ready once its measured position is this close to the target
#: it was last commanded, so the leader can hand back knowing the jaws are where
#: it thinks they are.  Tight enough that the hand-off does not jump, loose
#: enough that a stiff position loop with a little following error still
#: qualifies.
DEFAULT_READY_TOLERANCE_MM = 2.0

#: How often the follower republishes its ready state, in seconds — ~10 Hz.  The
#: state is also resent whenever it changes, so the leader sees a transition at
#: once; the periodic resend is for a leader that starts *after* the follower
#: arrived and would otherwise never learn it.
DEFAULT_READY_PERIOD_S = 0.1

#: How long the leader holds under gain for a ready signal before proceeding
#: anyway, in seconds.  ``0`` waits indefinitely.  A bounded wait is the
#: backward-compatibility answer: an older follower that never announces ready
#: must not leave a new leader holding forever, so the leader warns and relaxes
#: after this.  See :meth:`GripperTeleop._relaxed`.
DEFAULT_READY_TIMEOUT_S = 10.0


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


# ── Ready codec ───────────────────────────────────────────────────────────

_READY_FRAME = struct.Struct(">B")

#: Size of one ready frame in bytes (a single state byte).
READY_FRAME_SIZE = _READY_FRAME.size


def encode_ready_frame(ready: bool) -> bytes:
    """Pack one follower-ready frame — a single state byte.

    Deliberately tiny and separate from the teleop frame: the teleop frame
    carries the *leader's* pose and is byte-identical to the litearm stack's, so
    a readiness bit cannot ride on it without diverging that format.  This
    channel is the follower -> leader direction only.
    """
    return _READY_FRAME.pack(1 if ready else 0)


def decode_ready_frame(payload: bytes) -> bool:
    """Unpack a ready frame into ``True`` (ready) or ``False`` (not ready).

    ⚠ Any non-zero byte reads as ready: a truncation or a stray value should not
    silently look like "not ready".  Both of those are caught at the wire
    boundary instead — the leader treats an undecodable frame as no frame.

    Raises:
        ValueError: ``payload`` is not exactly :data:`READY_FRAME_SIZE` bytes.
    """
    if len(payload) != READY_FRAME_SIZE:
        raise ValueError(
            f"ready frame must be {READY_FRAME_SIZE} byte, got {len(payload)}")
    return _READY_FRAME.unpack(payload)[0] != 0


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


def ready_topic(grip_id: str = DEFAULT_GRIP_ID) -> str:
    """Topic the follower publishes and the leader subscribes to.

    A sibling of :func:`teleop_topic` on the same transport, in the reverse
    direction, carrying the one-byte readiness state.  It is *not* part of the
    litearm namespace the teleop topic shares — that link is one-way and its
    frame format is pinned — so a peer that does not know this topic simply
    never matches it, which is the intended "old peer" behaviour.
    """
    return f"litearm/v4/{grip_id}/gripper_ready"


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
        self._bind_addr = _parse_addr(bind_addr) if bind_addr is not None else None
        self._sock: Optional[socket.socket] = None
        self._sub: Optional[_UdpSubscription] = None

        if self._bind_addr is not None:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(self._bind_addr)
            sock.setblocking(False)
            self._sock = sock
        elif self._pub_addr is not None:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        if self._pub_addr is None and self._bind_addr is None:
            raise ValueError("UdpTeleopTransport needs pub_addr, bind_addr or both")

    def pub(self, topic: str, payload: bytes) -> None:
        if self._pub_addr is None or self._sock is None:
            return
        try:
            self._sock.sendto(payload, self._pub_addr)
        except OSError as e:
            log.debug("udp send failed: %s", e)

    def sub(self, topic: str) -> TeleopSubscription:
        # Keyed on the *bind*, not on the socket: a pub-only transport still
        # holds a socket, but it was never bound, is left in blocking mode, and
        # a recv on it would block the caller forever rather than returning
        # "nothing yet".
        if self._bind_addr is None:
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

    ``mode="master"`` publishes its opening and, once the follower reports ready
    (or ``ready_timeout_s`` passes), streams zero-torque frames so the jaws
    can be moved by hand; until then it holds its own position under gain.
    ``mode="slave"`` subscribes, ramps to the first frame (:meth:`_ramp_to`),
    then follows every fresh sample with ``send_mit_frame``, feeding the leader's
    finite-difference velocity forward as ``dq`` (:meth:`_estimate_dq`).  The
    ramp caps how far each frame's command may lead the measurement
    (:meth:`_cap_lead`); the follow loop is uncapped so it tracks the leader
    sample-for-sample.  It announces ready on :func:`ready_topic` once it has
    arrived.

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
        align_speed_mm_s: Slave only — speed of that align move, in mm/s of jaw
            travel.  Must be > 0.  See :meth:`_ramp_to`.
        watchdog_s: Slave only — seconds without a fresh frame before the
            follower is considered stale and starts holding.  Must be > 0: a
            non-positive watchdog makes the follower permanently stale, which is
            a session that starts and then silently does nothing.
        dq_max: Slave only — ceiling in rad/s on the leader velocity fed forward
            as the follower's ``dq`` target.  ``0`` disables the feedforward, in
            which case the follower biases on position error alone and trails a
            moving leader.  See :meth:`_estimate_dq`.
        lead_cap_mm: Slave only — ceiling in mm on how far the *align* command
            may lead the measured position, which bounds the align torque at
            ``kp * lead_cap_mm / rad_to_mm``.  The follow loop is not capped.
            ``0`` disables the cap.  See :meth:`_cap_lead`.
        torque_limit_nm: Slave only — ceiling in Nm on the follower's own torque.
            Held over the limit for :data:`TORQUE_TRIP_CYCLES` cycles it
            releases in place and stays released until the leader reopens by
            :data:`TORQUE_REARM_OPENNESS`.  ``0`` disables the guard.  See
            :meth:`_update_torque_guard`.
        ready_topic: The follower-ready channel, or ``None`` to run without a
            handshake (the master then goes slack from its first cycle).  When
            set, the master holds under gain until a ready frame arrives and the
            slave announces its arrival.
        require_ready: Master only — gate the hand-back on a ready frame.  Only
            meaningful with ``ready_topic`` set; ``False`` restores the
            immediate hand-back.
        ready_timeout_s: Master only — seconds to hold for a ready signal before
            proceeding anyway with a warning; ``0`` waits indefinitely.  Ignored
            when ``require_ready`` is ``False``.
        ready_tolerance_mm: Slave only — announce ready once the measured
            position is within this many mm of the commanded one.
        ready_period_s: Slave only — republish the ready state at least this
            often (it is also resent on every change).
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
        align_speed_mm_s: float = DEFAULT_ALIGN_SPEED_MM_S,   # [遥操对齐块]
        watchdog_s: float = 0.2,
        dq_max: float = DEFAULT_DQ_MAX,
        torque_limit_nm: float = DEFAULT_TORQUE_LIMIT_NM,
        lead_cap_mm: float = DEFAULT_LEAD_CAP_MM,             # [遥操对齐块]
        ready_topic: Optional[str] = None,
        require_ready: bool = True,
        ready_timeout_s: float = DEFAULT_READY_TIMEOUT_S,
        ready_tolerance_mm: float = DEFAULT_READY_TOLERANCE_MM,
        ready_period_s: float = DEFAULT_READY_PERIOD_S,
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
        if not float(align_speed_mm_s) > 0.0:
            raise ValueError(
                f"align_speed_mm_s must be > 0, got {align_speed_mm_s!r}")
        if float(torque_limit_nm) < 0.0:
            raise ValueError(
                f"torque_limit_nm must be >= 0 (0 disables the guard), "
                f"got {torque_limit_nm!r}")
        if float(lead_cap_mm) < 0.0:
            raise ValueError(
                f"lead_cap_mm must be >= 0 (0 disables the cap), "
                f"got {lead_cap_mm!r}")
        if float(ready_timeout_s) < 0.0:
            raise ValueError(
                f"ready_timeout_s must be >= 0 (0 waits indefinitely), "
                f"got {ready_timeout_s!r}")
        if float(ready_tolerance_mm) < 0.0:
            raise ValueError(
                f"ready_tolerance_mm must be >= 0, got {ready_tolerance_mm!r}")
        if not float(ready_period_s) > 0.0:
            raise ValueError("ready_period_s must be > 0")
        self._g = gripper
        self._tp = transport
        self._mode = mode
        self._topic = topic
        self._dt = 1.0 / rate_hz
        self._kp = kp
        self._kd = kd
        self._align = align
        self._align_speed_mm_s = float(align_speed_mm_s)
        self._watchdog_s = watchdog_s
        self._dq_max = float(dq_max)
        self._torque_limit_nm = float(torque_limit_nm)
        self._lead_cap_mm = float(lead_cap_mm)
        self._ready_topic = ready_topic
        self._require_ready = bool(require_ready)
        self._ready_timeout_s = float(ready_timeout_s)
        self._ready_tolerance_mm = float(ready_tolerance_mm)
        self._ready_period_s = float(ready_period_s)
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
        #: Readiness handshake.  Master side: ``_follower_ready`` is the last
        #: state heard, ``_master_relaxed`` latches once the gate opens so a
        #: later not-ready does not snatch control back from the operator, and
        #: ``_master_t0``/``_ready_timed_out`` implement the bounded wait.  Slave
        #: side: ``_ready_state`` is the state it last published (``None`` =
        #: nothing sent yet) and ``_ready_pub_ts`` paces the republish.
        self._follower_ready = False
        self._master_relaxed = False
        self._master_t0: Optional[float] = None
        self._ready_timed_out = False
        self._ready_rx = 0
        #: Master only — the live ready subscription, or ``None`` when the gate
        #: is disabled or the transport cannot carry it.  Set by
        #: :meth:`_master_loop`.
        self._ready_sub: Optional[TeleopSubscription] = None
        self._ready_state: Optional[bool] = None
        self._ready_pub_ts = 0.0
        self._ready_pubs = 0
        #: Slave only — the transport its ready frames go out on (the one that
        #: reaches the leader), set by :meth:`_slave_loop`.
        self._ready_tp: Optional[TeleopTransport] = None
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
        if self._mode == "slave":
            # Withdraw the announcement: the next session aligns again, and a
            # leader still up must not be told the follower is ready through a
            # gap between sessions.  Forced — the change might fall inside the
            # republish period and be throttled away.
            self._publish_ready(False, force=True)
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
            # Readiness handshake.  ``ready`` is this side's answer to "has the
            # follower arrived?" — on the slave, what it last announced; on the
            # master, the last state heard.  ``ready_timed_out`` is the master
            # proceeding without one (see :meth:`_master_loop`), and the two
            # counters are the frames sent/received on the ready channel.
            "ready": (bool(self._ready_state) if self._mode == "slave"
                      else self._follower_ready),
            "ready_rx": self._ready_rx,
            "ready_pubs": self._ready_pubs,
            "ready_timed_out": self._ready_timed_out,
        }

    # ── master ────────────────────────────────────────────────────────

    def _master_loop(self) -> None:
        self._ready_sub = self._open_ready_sub()
        if self._ready_sub is None:
            log.info("[master] zero-gravity, publishing to %s", self._topic)
        else:
            wait = ("indefinitely" if self._ready_timeout_s <= 0.0
                    else f"{self._ready_timeout_s:.0f}s")
            log.info("[master] holding under gain, publishing to %s; releasing "
                     "when %s reports ready (waiting %s)",
                     self._topic, self._ready_topic, wait)
        self._master_t0 = self._time_fn()
        try:
            while self._running:
                t0 = self._time_fn()
                # Read before sending: a held master needs its own angle to
                # hold, and a relaxed one still wants the state for the frame.
                try:
                    state = self._g.get_state(wait=False)
                except Exception:  # noqa: BLE001
                    log.exception("[master] CAN error; loop exiting")
                    break
                self._note_grip_fault(state)
                if self._ready_sub is not None and not self._follower_ready:
                    self._read_ready(self._ready_sub)
                # Keep the frame stream alive: the DM motor self-locks a
                # "communication loss" fault ~100 ms after frames stop, so a
                # frame goes out every cycle even though nothing is commanded.
                # While the gate is shut it is a *hold* at the current angle —
                # a zero-torque frame there would make the jaws hand-movable,
                # which is the very thing the gate exists to prevent.
                try:
                    if self._relaxed():
                        self._send(0.0, 0.0, 0.0, quiet=True)
                    else:
                        self._send(clamp_to_calibrated(
                            self._g.config, state.position_rad),
                            self._resolve_kp(), self._resolve_kd())
                except Exception:  # noqa: BLE001
                    log.exception("[master] CAN error; loop exiting")
                    break
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
        # The ready frames go back the same way the teleop frames came in, so
        # they travel the link that is already proven to reach the leader.
        self._ready_tp = transport
        sub = transport.sub(self._topic)
        cfg = self._g.config
        log.info("[slave] subscribed %s (align=%s watchdog=%.0fms)",
                 self._topic, self._align, self._watchdog_s * 1000.0)

        # Until a frame arrives, hold wherever the jaws already are.  A ``0.0``
        # target here would be a real position command — the open or the closed
        # stop, depending on the mount.
        here = self._g.get_state(wait=False)
        q_cmd = clamp_to_calibrated(cfg, here.position_rad)
        # Announce "not ready" up front: a leader already holding must not read
        # a stale ready from the previous session as "this one is aligned".
        self._publish_ready(False, force=True)

        if self._align:
            first = self._wait_first_frame(sub, timeout_s=5.0, hold_q=q_cmd)
            if first is not None:
                # ``q_cmd`` becomes the align target *before* the ramp so the
                # follow loop keeps holding it if the leader then goes quiet;
                # leaving it at the pre-align position would drive the jaws back
                # the moment the ramp finished.
                q_cmd = clamp_to_calibrated(
                    cfg, openness_to_rad(_clamp01(first[0]), cfg))
                self._last_frame_ts = self._time_fn()
                self._ever_received = True
                self._last_openness = _clamp01(first[0])
                self._last_position_mm = float(first[1])
                self._last_force_n = float(first[2])
                # [遥操对齐块] 定速斜坡 + 领先上限对齐（说明见 _ramp_to() 上方）。
                # 退回旧行为就把下一行换成：
                # self._g.goto_rad(q_cmd, kp=self._resolve_kp(),
                #                  kd=self._resolve_kd(), duration=1.0)
                self._ramp_to(q_cmd)
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
                    self._announce_ready(state, cfg, q_cmd)
                    if self._over_torque:
                        # Released in place: zero stiffness and damping, but the
                        # frames keep flowing so the motor does not latch its
                        # comm-loss fault.
                        self._send(q_cmd, 0.0, 0.0)
                    else:
                        # Deliberately uncapped: the follower commands the
                        # leader's opening outright so it stays responsive.  Only
                        # the align ramp caps its lead (:meth:`_ramp_to`); the
                        # guard above is what protects the follow under load.
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
        returned: this value feeds the align ramp, and a NaN folds onto an end
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

    # ── readiness handshake ───────────────────────────────────────────

    def _gate_enabled(self) -> bool:
        """Whether this session *wants* the leader-side gate."""
        return (self._mode == "master" and self._require_ready
                and self._ready_topic is not None)

    def _open_ready_sub(self) -> Optional[TeleopSubscription]:
        """Subscribe to the ready channel, or ``None`` if the gate is off.

        ⚠ A transport that cannot subscribe must not be fatal.  The UDP master
        publishes without binding — there is no receive path on it at all — so
        the gate is simply disabled and the leader goes slack as it always did.
        Treating "no channel" as "not ready" would hold the leader under gain
        for the full timeout on a link that can never carry the answer.
        """
        if not self._gate_enabled():
            return None
        try:
            return (self._sub_tp or self._tp).sub(self._ready_topic)
        except Exception as e:  # noqa: BLE001
            log.warning("[master] cannot subscribe to %s (%s); the readiness "
                        "gate is off", self._ready_topic, e)
            return None

    def _read_ready(self, sub: TeleopSubscription) -> None:
        """Drain the ready channel.  The newest frame wins."""
        msg = sub.drain_latest()
        if msg is None:
            return
        try:
            ready = decode_ready_frame(msg)
        except ValueError as e:
            log.debug("[master] ignoring bad ready frame: %s", e)
            return
        self._ready_rx += 1
        if ready != self._follower_ready:
            log.info("[master] follower %s", "ready" if ready else "not ready")
        self._follower_ready = ready

    def _relaxed(self) -> bool:
        """Whether the leader may hand back this cycle — **latched**.

        The gate opens on the first ready frame, or, so that an older follower
        that never announces cannot deadlock a new leader, once
        ``ready_timeout_s`` has passed — with a warning.  It then stays open:
        taking the jaws away from the operator mid-session because a frame went
        missing would be worse than the transient the gate guards against.
        """
        if self._master_relaxed:
            return True
        if self._ready_sub is None:
            return True
        if self._follower_ready:
            self._master_relaxed = True
            log.info("[master] follower ready — handing back zero-gravity")
            return True
        if self._ready_timeout_s <= 0.0:
            return False
        if (self._master_t0 is not None
                and (self._time_fn() - self._master_t0) >= self._ready_timeout_s):
            self._master_relaxed = True
            self._ready_timed_out = True
            log.warning("[master] no ready signal within %.0fs; releasing the "
                        "hand-back anyway (an older follower may not announce)",
                        self._ready_timeout_s)
            return True
        return False

    def _announce_ready(self, state: Any, cfg: Any, q_cmd: float) -> None:
        """Publish the follower's readiness for this cycle (slave)."""
        if self._ready_topic is None:
            return
        measured = float(state.position_rad)
        at_target = (abs(float(q_cmd) - measured) * float(cfg.rad_to_mm)
                     <= self._ready_tolerance_mm)
        self._publish_ready(self._ever_received and not self._stale
                            and not self._over_torque and at_target)

    def _publish_ready(self, ready: bool, force: bool = False) -> None:
        """Send one ready frame, throttled to a change or ``ready_period_s``.

        Best-effort: the channel is advisory, so a transport that cannot carry
        it (or drops this one frame) must not affect the follow loop.
        """
        if self._ready_topic is None or self._ready_tp is None:
            return
        now = self._time_fn()
        if (not force and ready == self._ready_state
                and (now - self._ready_pub_ts) < self._ready_period_s):
            return
        try:
            self._ready_tp.pub(self._ready_topic, encode_ready_frame(ready))
        except Exception as e:  # noqa: BLE001
            log.debug("[slave] ready publish failed: %s", e)
            return
        if ready != self._ready_state:
            log.info("[slave] %s", "ready — the leader may hand back" if ready
                     else "not ready")
        self._ready_state = ready
        self._ready_pub_ts = now
        self._ready_pubs += 1

    # ═══════════════════════════════════════════════════════════════════════
    # [遥操对齐块] 遥操从机启动对齐的安全措施 —— 可整块注释 / 删除
    #
    # 作用：把"对齐到 leader 第一帧"从「一帧 goto_rad 把整个误差一次发出去」
    # 改成「定速斜坡（align_speed_mm_s）+ 每帧把指令领先夹到 lead_cap_mm」。
    # 力矩 = kp × (q_cmd − q_measured)，夹住领先就从构造上夹住了力矩 —— 开盘
    # 那一帧不再以满力矩顶过去（曾有从机被这样怼进硬限位、撞断打印限位）。
    # 顺带把对齐纳入 _update_torque_guard：卡住会像正常一样失力释放。
    #
    # 本块清单（grep -n '\[遥操对齐块\]' src/litegrip/teleop.py src/litegrip/gripper.py）：
    #   - 常量   DEFAULT_ALIGN_SPEED_MM_S / DEFAULT_LEAD_CAP_MM
    #   - 形参   __init__ 的 align_speed_mm_s / lead_cap_mm（含校验与赋值）
    #   - 方法   _lead_cap_rad / _cap_lead / _ramp_to（本段）
    #   - 调用点 self._ramp_to(q_cmd)（slave 对齐段）
    #   - 透传   gripper.py 的 teleop_start(align_speed_mm_s=, lead_cap_mm=)
    #
    # 临时停用（不动代码）：teleop_start(lead_cap_mm=0) 关掉领先上限；
    #   align_speed_mm_s 只调对齐速度。
    # 整块移除：删掉上面清单，并把 _ramp_to(q_cmd) 换回旧行为 ——
    #   self._g.goto_rad(q_cmd, kp=self._resolve_kp(),
    #                    kd=self._resolve_kd(), duration=1.0)
    #   （注意：旧的 duration 只是 deadline、不是斜坡，第一帧即满误差；这正是
    #    本块要修的隐患。）
    # ═══════════════════════════════════════════════════════════════════════

    def _lead_cap_rad(self) -> float:
        """The lead cap in radians, or ``0.0`` when it is disabled.

        ``rad_to_mm`` is the only conversion the cap needs and teleop already
        refuses to start without it (``check_ready``); treating a missing or zero
        one as "disabled" keeps this from dividing by zero on a caller that
        bypassed that check.
        """
        rad_to_mm = float(getattr(self._g.config, "rad_to_mm", 0.0) or 0.0)
        if self._lead_cap_mm <= 0.0 or rad_to_mm <= 0.0:
            return 0.0
        return self._lead_cap_mm / rad_to_mm

    def _cap_lead(self, q_target: float, pos_rad: float) -> float:
        """Pull a position command back to within the lead cap of the measurement.

        The follower is a position loop whose gain is in Nm/rad, so the torque it
        commands is ``kp * (q_target - pos_rad)``.  Bounding that difference
        bounds the torque by construction, which is the point: the align's first
        frame hands over the leader's whole opening at once, and an uncapped one
        would demand ``kp`` times the full error — at the shipped ``kp`` of 100
        an error over ~0.1 rad saturates the DM4310 (10 Nm).  Capping does not
        slow the move down for free: the jaws still travel, they just press with
        a bounded torque while they catch up.  Only the align ramp calls this.
        """
        cap = self._lead_cap_rad()
        if cap <= 0.0:
            return q_target
        lead = q_target - pos_rad
        if lead > cap:
            return pos_rad + cap
        if lead < -cap:
            return pos_rad - cap
        return q_target

    def _ramp_to(self, target_rad: float) -> None:
        """Travel to ``target_rad`` at :attr:`_align_speed_mm_s`, capped every frame.

        The align used to be one ``goto_rad(..., duration=1.0)``.  ``duration``
        reads like a ramp, but the CAN layer sends ``q = q_target`` from the
        first frame and merely holds it until the deadline
        (``protocols/can_bus.py``'s ``control_mit_stream``), so the first frame
        demanded ``kp`` times the whole error — a full-torque step wherever the
        jaws happened to start, which drove a follower into the closed hard stop
        hard enough to break its printed limit.  Instead this is the profile
        ``open``/``close`` already travel with (``actions.MotionConfig``): a
        constant-speed schedule, the same speed fed forward as ``dq``, and
        :meth:`_cap_lead` on every frame.

        Sending through this module's own path also puts the align under
        :meth:`_update_torque_guard`, which the blocking ``goto_rad`` was outside
        of — a jam during the align now releases like any other.
        """
        cfg = self._g.config
        try:
            state = self._g.get_state(wait=False)
        except Exception as e:  # noqa: BLE001
            log.warning("[slave] align state read failed: %s", e)
            return
        start = clamp_to_calibrated(cfg, state.position_rad)
        dist_rad = target_rad - start
        rad_to_mm = float(getattr(cfg, "rad_to_mm", 0.0) or 0.0)
        speed_mm_s = self._align_speed_mm_s
        if rad_to_mm <= 0.0 or abs(dist_rad) < 1e-9:
            # Nothing to schedule against.  One capped frame still bounds the
            # torque; an uncapped one would be the bug this method exists to fix.
            self._send(self._cap_lead(target_rad, state.position_rad),
                       self._resolve_kp(), self._resolve_kd())
            return
        speed_rad_s = speed_mm_s / rad_to_mm
        sign = 1.0 if dist_rad >= 0.0 else -1.0
        steps = max(1, int(round(abs(dist_rad) / speed_rad_s / self._dt)))
        log.info("[slave] aligning: %+.4f -> %+.4f rad at %.1f mm/s "
                 "(%d frames, cap %.4f rad)",
                 start, target_rad, speed_mm_s, steps, self._lead_cap_rad())
        for i in range(1, steps + 1):
            if not self._running:
                return
            t0 = self._time_fn()
            try:
                state = self._g.get_state(wait=False)
                self._note_grip_fault(state)
                self._update_torque_guard(state)
                if self._over_torque:
                    # Released in place, exactly as the follow loop would.  Hold
                    # where the jaws are; do not keep driving into whatever
                    # tripped the guard.
                    self._send(clamp_to_calibrated(cfg, state.position_rad),
                               0.0, 0.0)
                    return
                q_sched = start + dist_rad * (i / steps)
                self._send(self._cap_lead(q_sched, state.position_rad),
                           self._resolve_kp(), self._resolve_kd(),
                           dq=sign * speed_rad_s)
            except Exception:  # noqa: BLE001
                log.exception("[slave] CAN error during align; loop exiting")
                return
            rest = self._dt - (self._time_fn() - t0)
            if rest > 0.0:
                self._sleep_fn(rest)

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
