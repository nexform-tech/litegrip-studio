"""Trajectory record and replay — teach the jaws a motion once, repeat it later.

Read this if you want to capture a motion off a real gripper and play it back:
the approach path that seats a part, the wiggle that shakes it loose, the squeeze
profile you found by hand.  ``gripper.record(...)`` puts the motor into
zero-gravity, lets you push the jaws through the motion, and samples what they
did; ``gripper.play(trajectory)`` streams it back as MIT command frames.

Two halves, two classes:

* :class:`TrajectoryRecorder` — a background sampling loop.
* :class:`TrajectoryPlayer` — a background command loop.

and :class:`Trajectory` / :class:`TrajectorySample` — the captured data and its
on-disk format.

Why the recording stores ``openness`` and not radians
----------------------------------------------------
Each gripper has its own zero, direction and calibration (one unit opens toward
-1.42 rad, another toward +1.14 rad), so a raw angle is meaningless on a
different unit.  A sample therefore carries the opening normalised by the
*recording* unit's travel — dimensionless, direction-free — and replay converts
it back with the *local* calibration.  A trajectory taught on a normal-mount
gripper replays correctly on a reverse-mounted one.  The recorded angle,
velocity and torque are kept as diagnostics only.

What replay does not reproduce
------------------------------
Replay commands **position**, not force.  The opening is clamped to the local
calibrated travel, and the recorded torque is never fed forward, so a squeeze
that was recorded against an object replays as a position path that presses
with whatever ``kp`` yields — the grasp force you taught is *not* preserved.
For a repeatable grip force, replay the motion and then call
:meth:`~litegrip.LiteGrip.grasp` with an explicit ``force_n``.

Traps this module works around
------------------------------
* The DM motor self-locks a communication-loss fault roughly 100 ms after the
  frames stop.  A recorder that only *sampled* would therefore fault the motor
  it is teaching, so zero-gravity recording streams a zero-torque frame every
  cycle — the same rule as the teleop master loop.
* A blocking :meth:`~litegrip.LiteGrip.record` returns only once the capture is
  complete; a capture that did not fill raises and reports how many samples
  landed, rather than handing back a short recording as if it were whole.
* A file whose length does not match the sample count in its header is rejected,
  not parsed into half a trajectory.
* A sample whose timestamp did not advance is not stored, and a clock that will
  not advance aborts either loop — rather than filling a "successful" recording
  with rows that all claim the same instant, or replaying forever.
"""

from __future__ import annotations

import logging
import os
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, List, Optional, Tuple

from .exceptions import LiteGripError
from .teleop import openness_to_rad, rad_to_openness

log = logging.getLogger("litegrip.trajectory")

__all__ = [
    "TrajectorySample",
    "Trajectory",
    "TrajectoryRecorder",
    "TrajectoryPlayer",
    "trajectory_dir",
    "TrajectoryError",
    "TrajectoryBusyError",
    "TrajectoryNotActiveError",
    "TrajectoryEmptyError",
    "TrajectoryRecordingError",
    "TrajectoryFormatError",
    "DEFAULT_RATE_HZ",
]

#: Default sampling rate for a hand-taught recording.  Fast enough to keep the
#: shape of a motion a human made, slow enough that a CAN round trip per sample
#: always fits: 100 Hz means a 10 ms budget, and a status frame takes ~1 ms.
DEFAULT_RATE_HZ = 100.0

#: Two clock readings closer together than this count as "the clock did not
#: advance" — see :data:`_STALL_CYCLES`.  One nanosecond, because the target
#: platform is Linux, where ``time.monotonic`` has nanosecond resolution: a
#: running loop always moves further than this between two cycles, and a
#: stopped clock never moves at all.
_CLOCK_EPS = 1e-9

#: How many consecutive non-advancing cycles abort a sampling or replay loop.
#: A real monotonic clock never does this — five cycles of Python and a CAN
#: round trip cannot land inside one nanosecond — so it only ever fires on a
#: clock that has stopped, where without the guard the loop would spin forever.
_STALL_CYCLES = 5


# ── file format ───────────────────────────────────────────────────────────
#
# Little-endian throughout, matching the fixed-layout samples litearm-core
# writes.  (litegrip's teleop wire frames are big-endian for byte compatibility
# with the litearm implementation; nothing outside this SDK reads this file, so
# there is no interop to preserve and the native order wins.)
#
#   header  magic 8s | version u16 | n_samples u32 | sample_hz f64 |
#           created f64 | pos_closed_rad f64 | pos_open_rad f64 |
#           rad_to_mm f64 | can_id u32 | mount 8s   = 66 B
#   sample  t f64 | openness f64 | position_rad f64 | velocity_rad_s f64 |
#           torque_nm f64                            = 40 B

_MAGIC = b"LGRTRJ01"
_VERSION = 1
_HEADER = struct.Struct("<8sHI5dI8s")
_SAMPLE = struct.Struct("<5d")
_MOUNT_FIELD = 8
_MOUNTS = (None, "normal", "reverse")


def trajectory_dir() -> str:
    """Directory a trajectory is saved to when only a name is given.

    ``~/.litegrip/trajectories`` — next to the per-channel calibration files
    (see :func:`~litegrip.default_calib_path`), because a trajectory belongs to
    the machine, not to the working directory the program happened to start in.
    ``LITEGRIP_TRAJ_DIR`` overrides it, for tests and for a controller with a
    read-only home.
    """
    env = os.environ.get("LITEGRIP_TRAJ_DIR")
    if env:
        return env
    return os.path.join(os.path.expanduser("~"), ".litegrip", "trajectories")


def resolve_path(path: str) -> str:
    """A bare name lands in :func:`trajectory_dir`; anything else is a path.

    ``"pick"`` → ``~/.litegrip/trajectories/pick.lgt``; ``"out/pick.lgt"`` and
    ``"/tmp/pick.lgt"`` are used as written.
    """
    if os.sep in path or (os.altsep and os.altsep in path):
        return path
    name = path if path.endswith(".lgt") else path + ".lgt"
    return os.path.join(trajectory_dir(), name)


# ── exceptions ────────────────────────────────────────────────────────────


class TrajectoryError(LiteGripError):
    """Base class for trajectory record/replay errors."""


class TrajectoryBusyError(TrajectoryError):
    """Raised when a recording or replay is started while one is already running."""


class TrajectoryNotActiveError(TrajectoryError):
    """Raised when a stop is asked for but nothing is running."""


class TrajectoryEmptyError(TrajectoryError):
    """Raised when a capture or a replay would deal with zero samples."""


class TrajectoryRecordingError(TrajectoryError):
    """Raised when the recording loop died, or a timed capture did not fill."""


class TrajectoryFormatError(TrajectoryError):
    """Raised when a byte stream is not a well-formed trajectory file."""


# ── data ──────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class TrajectorySample:
    """One sample of a recorded motion.

    Attributes:
        t: Seconds since the recording started.  A trimmed clock reading of the
            *recording* machine — it means nothing after a reboot or on another
            host, and replay only ever uses the differences between samples.
        openness: Opening in ``[0, 1]`` (0 = closed, 1 = fully open).  The
            channel replay actually follows; see the module docstring.
        position_rad: Raw motor angle at that instant — diagnostic.  Only
            meaningful against the calibration recorded alongside it.
        velocity_rad_s: Motor velocity — diagnostic.
        torque_nm: Motor torque — diagnostic.  Also the closest thing to a
            force record, but replay does not feed it forward.
    """

    t: float
    openness: float
    position_rad: float
    velocity_rad_s: float = 0.0
    torque_nm: float = 0.0


@dataclass
class Trajectory:
    """A recorded motion: samples plus the calibration they were taken against.

    The geometry fields describe the gripper that *recorded* the trajectory and
    travel with it in the file, so a loaded trajectory still reports which unit
    it came from and how its ``openness`` values were derived.  Replay ignores
    them in favour of the local gripper's own calibration.
    """

    samples: List[TrajectorySample] = field(default_factory=list)
    sample_hz: float = DEFAULT_RATE_HZ
    created: float = field(default_factory=time.time)
    can_id: int = 0x08
    pos_closed_rad: float = 0.0
    pos_open_rad: float = 0.0
    rad_to_mm: float = 0.0
    mount: Optional[str] = None

    def __len__(self) -> int:
        return len(self.samples)

    @property
    def duration(self) -> float:
        """Seconds from the first sample to the last (0.0 for a short one).

        The span, not the last sample's timestamp: a trajectory whose first
        sample does not sit at ``t = 0`` still lasts only as long as its own
        samples cover, and reporting the raw end stamp would make replay hold
        its opening for the whole offset before starting to move.
        """
        if not self.samples:
            return 0.0
        return float(self.samples[-1].t - self.samples[0].t)

    def openness_at(self, t: float) -> float:
        """Opening at time *t*, linearly interpolated between samples.

        Clamped at both ends: before the first sample and after the last one the
        nearest sample's opening is returned.  Signals that move are sampled far
        faster than they move, so linear interpolation between neighbours is
        well below the mechanical resolution — a smoother curve would be
        inventing detail the recording does not contain.
        """
        samples = self.samples
        if not samples:
            raise TrajectoryEmptyError("轨迹没有采样点")
        if t <= samples[0].t:
            return samples[0].openness
        if t >= samples[-1].t:
            return samples[-1].openness
        lo, hi = 0, len(samples) - 1
        while hi - lo > 1:                      # bisect on t
            mid = (lo + hi) // 2
            if samples[mid].t <= t:
                lo = mid
            else:
                hi = mid
        a, b = samples[lo], samples[hi]
        span = b.t - a.t
        if span <= 0.0:
            return b.openness
        return a.openness + (t - a.t) / span * (b.openness - a.openness)

    # ── serialisation ─────────────────────────────────────────────────

    def to_bytes(self) -> bytes:
        """Serialise to the binary format described in this module's header.

        Raises:
            TrajectoryError: The mount name does not fit its field.
        """
        mount = (self.mount or "").encode("ascii")
        if len(mount) > _MOUNT_FIELD:
            raise TrajectoryError(
                f"mount 名 {self.mount!r} 超过 {_MOUNT_FIELD} 字节")
        parts = [_HEADER.pack(
            _MAGIC, _VERSION, len(self.samples), float(self.sample_hz),
            float(self.created), float(self.pos_closed_rad),
            float(self.pos_open_rad), float(self.rad_to_mm), int(self.can_id),
            mount.ljust(_MOUNT_FIELD, b"\x00"))]
        for s in self.samples:
            parts.append(_SAMPLE.pack(
                float(s.t), float(s.openness), float(s.position_rad),
                float(s.velocity_rad_s), float(s.torque_nm)))
        return b"".join(parts)

    @classmethod
    def from_bytes(cls, blob: bytes) -> "Trajectory":
        """Parse a trajectory file, or say precisely why it is not one.

        Every check here exists to stop a corrupt file from becoming a
        plausible-looking motion: the header's sample count has to match the
        payload exactly (a truncated download, a half-written file and a file
        with another file's tail appended are all caught by the same rule), and
        the samples have to be finite, in range and ordered in time.

        Raises:
            TrajectoryFormatError: Magic, version, length, header fields or
                sample values are not valid.
        """
        blob = bytes(blob)
        if len(blob) < _HEADER.size:
            raise TrajectoryFormatError(
                f"文件只有 {len(blob)}B, 连 {_HEADER.size}B 的头都不够")
        (magic, version, n, hz, created, pos_closed, pos_open, rad_to_mm,
         can_id, mount_raw) = _HEADER.unpack_from(blob, 0)
        if magic != _MAGIC:
            raise TrajectoryFormatError(
                f"magic 不对: {magic!r} (期望 {_MAGIC!r}) —— 不是轨迹文件")
        if version != _VERSION:
            raise TrajectoryFormatError(
                f"格式版本 {version} 不是本 SDK 能读的版本 {_VERSION}")
        want = _HEADER.size + n * _SAMPLE.size
        if len(blob) != want:
            raise TrajectoryFormatError(
                f"文件长度 {len(blob)}B 与头部声明的 {n} 拍 ({want}B) 不符 "
                f"(差 {len(blob) - want:+d}B) —— 文件被截断或尾部多了数据, "
                f"拒绝解析出半截轨迹")
        if not (hz > 0.0):
            raise TrajectoryFormatError(f"采样率非法: {hz}")
        if not (rad_to_mm > 0.0):
            raise TrajectoryFormatError(f"rad_to_mm 非法: {rad_to_mm}")
        mount = mount_raw.split(b"\x00", 1)[0].decode("ascii", "replace")
        if mount not in ("", "normal", "reverse"):
            raise TrajectoryFormatError(f"装法名非法: {mount!r}")
        if not (abs(pos_open - pos_closed) * rad_to_mm > 0.0):
            raise TrajectoryFormatError(
                f"行程为零 (closed={pos_closed}, open={pos_open})")

        samples: List[TrajectorySample] = []
        last_t = float("-inf")
        for i in range(n):
            values = _SAMPLE.unpack_from(blob, _HEADER.size + i * _SAMPLE.size)
            t, openness, position_rad, velocity, torque = values
            for name, value in zip(
                    ("t", "openness", "position_rad", "velocity_rad_s",
                     "torque_nm"), values):
                if value != value or value in (float("inf"), float("-inf")):
                    raise TrajectoryFormatError(
                        f"第 {i} 拍的 {name} 不是有限数: {value}")
            if not 0.0 <= openness <= 1.0:
                raise TrajectoryFormatError(
                    f"第 {i} 拍的 openness={openness} 不在 [0, 1] 内")
            if t < last_t:
                raise TrajectoryFormatError(
                    f"第 {i} 拍的 t={t} 比上一拍 {last_t} 还早 —— 时间未单调")
            last_t = t
            samples.append(TrajectorySample(
                t=t, openness=openness, position_rad=position_rad,
                velocity_rad_s=velocity, torque_nm=torque))

        return cls(samples=samples, sample_hz=hz, created=created, can_id=can_id,
                   pos_closed_rad=pos_closed, pos_open_rad=pos_open,
                   rad_to_mm=rad_to_mm, mount=mount or None)

    def save(self, path: str) -> str:
        """Write to *path*, creating the parent directory.  Returns the path.

        A bare name goes to :func:`trajectory_dir` (see :func:`resolve_path`).
        """
        target = resolve_path(path)
        parent = os.path.dirname(target)
        if parent:
            os.makedirs(parent, exist_ok=True)
        with open(target, "wb") as f:
            f.write(self.to_bytes())
        log.info("trajectory saved: %s (%d samples)", target, len(self.samples))
        return target

    @classmethod
    def load(cls, path: str) -> "Trajectory":
        """Read a trajectory written by :meth:`save`."""
        with open(resolve_path(path), "rb") as f:
            return cls.from_bytes(f.read())


# ── loop pacing ───────────────────────────────────────────────────────────


class _Pacer:
    """Frame pacing and loop-rate measurement, shared by both loops.

    ``rest()`` sleeps whatever is left of a cycle's budget, so the loop holds
    its rate without drifting when a cycle overruns.  ``loop_hz`` is the
    measured rate over the last second — the number to look at when a recording
    sounds wrong, because a loop that cannot keep up drops samples rather than
    stretching time.
    """

    def __init__(self, dt: float, sleep_fn: Callable[[float], None],
                 monotonic_fn: Callable[[], float]) -> None:
        self._dt = dt
        self._sleep_fn = sleep_fn
        self._monotonic_fn = monotonic_fn
        self._loops = 0
        self._hz_t0 = 0.0
        self._hz_n0 = 0
        self.loop_hz = 0.0

    def rest(self, t0: float) -> None:
        self._loops += 1
        now = self._monotonic_fn()
        if self._hz_t0 == 0.0:
            self._hz_t0 = now
            self._hz_n0 = self._loops
        elif now - self._hz_t0 >= 1.0:
            self.loop_hz = (self._loops - self._hz_n0) / (now - self._hz_t0)
            self._hz_t0 = now
            self._hz_n0 = self._loops
        rest = self._dt - (self._monotonic_fn() - t0)
        if rest > 0.0:
            self._sleep_fn(rest)


def _resolve_seams(
    gripper: Any,
    sleep_fn: Optional[Callable[[float], None]],
    monotonic_fn: Optional[Callable[[], float]],
) -> Tuple[Callable[[float], None], Callable[[], float]]:
    """Fill in the timing seams from the gripper's :class:`MotionConfig`.

    The motion engine already takes ``sleep_fn`` / ``monotonic_fn`` from there,
    and the fixture every test builds its gripper with stubs them.  Defaulting
    to the same place is what lets a test drive a whole recording without
    stubbing this module — and letting a caller pass them explicitly is what
    keeps the classes usable on their own.
    """
    cfg = getattr(gripper, "motion_config", None)
    if sleep_fn is None:
        sleep_fn = getattr(cfg, "sleep_fn", None) or time.sleep
    if monotonic_fn is None:
        monotonic_fn = getattr(cfg, "monotonic_fn", None) or time.monotonic
    return sleep_fn, monotonic_fn


# ── recording ─────────────────────────────────────────────────────────────


class TrajectoryRecorder:
    """Samples the gripper's state into a :class:`Trajectory`, on a thread.

    With ``zero_gravity=True`` the loop streams ``q=0, kp=0, kd=0`` every cycle,
    which leaves the motor energised but torque-free so the jaws can be pushed
    by hand — that is the hand-teaching mode, and the zero-torque frame is
    mandatory, not a convenience (see the module docstring).  With
    ``zero_gravity=False`` the loop only *reads*, so the caller is free to drive
    the gripper from another thread while it records: that is how a
    programmatic ``grasp()`` or a move sequence gets captured.

    Args:
        gripper: The ``LiteGrip`` to sample.
        rate_hz: Samples per second.  ``DEFAULT_RATE_HZ`` (100) unless the
            motion is fast.
        zero_gravity: Stream zero-torque frames so the jaws can be hand-driven.
        max_samples: Stop by itself after this many samples.  ``None`` records
            until :meth:`stop`.  Bounded recordings are what make a stuck clock
            visible as a timeout instead of a running process.
        sleep_fn, monotonic_fn: Timing seams for tests, matching
            :class:`~litegrip.MotionConfig`'s names.
    """

    def __init__(
        self,
        gripper: Any,
        rate_hz: float = DEFAULT_RATE_HZ,
        zero_gravity: bool = True,
        max_samples: Optional[int] = None,
        sleep_fn: Optional[Callable[[float], None]] = None,
        monotonic_fn: Optional[Callable[[], float]] = None,
    ) -> None:
        if rate_hz <= 0.0:
            raise ValueError(f"rate_hz must be > 0, got {rate_hz!r}")
        self._g = gripper
        self._rate_hz = float(rate_hz)
        self._zero_gravity = bool(zero_gravity)
        self._max_samples = None if max_samples is None else int(max_samples)
        self._sleep_fn, self._monotonic_fn = _resolve_seams(
            gripper, sleep_fn, monotonic_fn)
        self._pacer = _Pacer(1.0 / self._rate_hz, self._sleep_fn,
                             self._monotonic_fn)

        # Appended by the loop thread, read by the caller.  No lock, for the
        # same reason teleop's frame counter has none: list append and len are
        # atomic under the GIL, and a torn sample is impossible once appended.
        self._samples: List[TrajectorySample] = []
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._error: Optional[BaseException] = None
        self._t0 = 0.0

    # ── lifecycle ─────────────────────────────────────────────────────

    @property
    def is_recording(self) -> bool:
        return self._running

    @property
    def sample_count(self) -> int:
        return len(self._samples)

    def start(self) -> None:
        """Start sampling.  Raises :class:`TrajectoryBusyError` if already on."""
        if self._running:
            raise TrajectoryBusyError("trajectory recording is already running")
        self._samples = []
        self._error = None
        self._pacer.loop_hz = 0.0
        # Sample 0 is taken here, before the thread exists, with t = 0.0: the
        # recording then starts at a commanded instant instead of whenever the
        # scheduler happened to first run the loop.
        self._t0 = self._monotonic_fn()
        self._samples.append(self._sample_now(0.0))
        self._running = True
        self._thread = threading.Thread(
            target=self._run, name="litegrip-trajectory-record", daemon=True)
        self._thread.start()
        log.info("recording started: %.0fHz zero_gravity=%s max_samples=%s",
                 self._rate_hz, self._zero_gravity, self._max_samples)

    def _run(self) -> None:
        try:
            self._loop()
        except BaseException as e:  # noqa: BLE001 — surfaces through result()
            self._error = e
            log.warning("recording loop stopped: %s", e)
        finally:
            self._running = False

    def _loop(self) -> None:
        stall = 0
        last = self._t0
        while self._running:
            t0 = self._monotonic_fn()
            # A cycle whose clock did not move produced no sample: two rows with
            # the same timestamp are not two measurements, and appending one
            # anyway is how a stopped clock becomes a "successful" recording of
            # zero length.  So the clock has to move before anything is stored —
            # and a clock that will not move is an error, not a slow capture.
            if t0 - last <= _CLOCK_EPS:
                stall += 1
                if stall >= _STALL_CYCLES:
                    raise TrajectoryRecordingError(
                        f"采样时钟连续 {stall} 拍没有前进 (t={t0}) —— "
                        f"拒绝在停住的时钟上无限追加采样")
                last = t0
                self._pacer.rest(t0)
                continue
            stall = 0
            last = t0

            # Checked before storing, not after: start() already stored the
            # reference sample, so a check after the append would return
            # max_samples + 1 rows — and for a cap the reference sample alone
            # already meets, it would depend on which thread got there first.
            if (self._max_samples is not None
                    and len(self._samples) >= self._max_samples):
                return

            if self._zero_gravity:
                self._g.send_mit_frame(q=0.0, kp=0.0, kd=0.0)
            self._samples.append(self._sample_now(t0 - self._t0))
            self._pacer.rest(t0)

    def _sample_now(self, t: float) -> TrajectorySample:
        state = self._g.get_state(wait=False)
        return TrajectorySample(
            t=float(t),
            openness=rad_to_openness(state.position_rad, self._g.config),
            position_rad=float(state.position_rad),
            velocity_rad_s=float(state.velocity_rad_s),
            torque_nm=float(state.torque_nm),
        )

    def wait_for(self, n_samples: int, timeout: float,
                 poll: float = 0.005) -> int:
        """Block until *n_samples* have been captured; return the count.

        Polls the real clock, not the loop's ``monotonic_fn`` seam, so a test
        that stubs the loop's clock still gets a bounded wait.  A loop that died
        raises its own error here immediately rather than after the timeout.

        Raises:
            TrajectoryRecordingError: The loop died, or *timeout* elapsed with
                fewer samples — the message names the count, so a short capture
                is visible rather than silently accepted.
        """
        n = int(n_samples)
        deadline = time.monotonic() + float(timeout)
        while True:
            if self._error is not None:
                raise self._recording_error(len(self._samples))
            got = len(self._samples)
            if got >= n:
                return got
            if not self._running:
                raise TrajectoryRecordingError(
                    f"录制提前结束: 只录到 {got}/{n} 拍, 循环已停止")
            if time.monotonic() >= deadline:
                raise TrajectoryRecordingError(
                    f"录制未在 {float(timeout):.1f}s 内录满 {n} 拍 "
                    f"(只录到 {got} 拍) —— 可调大 duration_s 或调小 rate_hz")
            time.sleep(poll)

    def stop(self, timeout: float = 2.0) -> None:
        """Stop the loop and leave the gripper holding its position.

        The hold happens even when the loop died on a CAN error: a failed
        recording must not leave the jaws slack.
        """
        self._running = False
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=timeout)
        self._thread = None
        if self._zero_gravity:
            try:
                self._g.exit_zero_gravity()
            except Exception as e:  # noqa: BLE001
                log.debug("exit_zero_gravity on stop failed: %s", e)
        log.info("recording stopped: %d samples", len(self._samples))

    # ── results ───────────────────────────────────────────────────────

    def _recording_error(self, got: int) -> TrajectoryRecordingError:
        """Wrap whatever killed the loop, without double-wrapping our own error."""
        if isinstance(self._error, TrajectoryRecordingError):
            return self._error
        return TrajectoryRecordingError(
            f"录制失败, 已录到的 {got} 拍不予返回: {self._error}")

    def result(self, allow_empty: bool = False) -> Trajectory:
        """The captured trajectory.

        Refuses to hand back a capture that did not work: a dead loop raises
        :class:`TrajectoryRecordingError` (carrying the original error and the
        sample count) rather than returning the partial capture as if it were a
        whole recording.

        :class:`TrajectoryEmptyError` is for the one way to get nothing at all —
        :meth:`result` on a recorder that was never started.  A started capture
        always holds at least the reference sample :meth:`start` takes.

        Raises:
            TrajectoryRecordingError: The sampling loop died.
            TrajectoryEmptyError: Nothing was ever captured.
        """
        if self._error is not None:
            raise self._recording_error(len(self._samples))
        if not self._samples and not allow_empty:
            raise TrajectoryEmptyError(
                "录制没有采到任何样本 (0 拍) —— 起停之间没有留出采样时间")
        cfg = self._g.config
        return Trajectory(
            samples=list(self._samples),
            sample_hz=self._rate_hz,
            created=time.time(),
            can_id=int(self._g.can_id),
            pos_closed_rad=float(cfg.pos_closed_rad),
            pos_open_rad=float(cfg.pos_open_rad),
            rad_to_mm=float(cfg.rad_to_mm),
            mount=cfg.mount,
        )

    def status(self) -> dict:
        """A snapshot of the recording session, for logging and diagnostics."""
        return {
            "active": self._running,
            "kind": "record",
            "samples": len(self._samples),
            "rate_hz": round(self._rate_hz, 1),
            "zero_gravity": self._zero_gravity,
            "loop_hz": round(self._pacer.loop_hz, 1),
            "error": None if self._error is None else str(self._error),
        }


# ── replay ────────────────────────────────────────────────────────────────


class TrajectoryPlayer:
    """Streams a :class:`Trajectory` back to the gripper, on a thread.

    The target at each cycle is interpolated from the trajectory by **wall
    clock**: ``u = (now - t0) * speed``, then ``trajectory.openness_at(u)``.
    Advancing an index once per cycle instead would tie playback speed to the
    loop rate and accumulate drift, so a cycle that overruns would make every
    later sample late and a 2 s recording would take longer and longer to play.
    Here a slow cycle skips ahead, and the motion stays the length it was
    taught.

    The commanded angle is ``openness_to_rad(openness, local_config)``, so the
    trajectory plays on a gripper with a different mount or calibration.  Only
    position is commanded: the recorded velocity and torque are never sent as
    ``dq``/``tau`` feed-forward, because both are tied to the sign convention of
    the unit that recorded them.

    Args:
        gripper: The ``LiteGrip`` to drive.
        trajectory: What to play.  Rejected up front if it has no samples.
        speed: Multiplier on the recorded timing.  ``0.5`` is half speed.
        kp, kd: Gains for the replay frames.  ``None`` uses the gripper's
            configured gains.
        loop: Restart from the beginning instead of stopping at the end.  A
            trajectory with a single sample is a pose with no length to
            restart, so looping it keeps holding that opening.
        align: Move to the trajectory's first opening (one ``goto_rad``) before
            following.  Without it the first frame steps to sample 0 from
            wherever the jaws happen to be, which is a torque spike.
        rate_hz: Frame rate.  ``None`` uses
            :attr:`~litegrip.MotionConfig.frame_interval` (200 Hz by default).
        sleep_fn, monotonic_fn: Timing seams for tests.
    """

    def __init__(
        self,
        gripper: Any,
        trajectory: Trajectory,
        speed: float = 1.0,
        kp: Optional[float] = None,
        kd: Optional[float] = None,
        loop: bool = False,
        align: bool = True,
        rate_hz: Optional[float] = None,
        sleep_fn: Optional[Callable[[float], None]] = None,
        monotonic_fn: Optional[Callable[[], float]] = None,
    ) -> None:
        if speed <= 0.0:
            raise ValueError(f"speed must be > 0, got {speed!r}")
        if len(trajectory) == 0:
            raise TrajectoryEmptyError(
                "轨迹里没有任何采样点, 没有可回放的动作")
        self._g = gripper
        self._traj = trajectory
        self._speed = float(speed)
        self._kp = float(kp) if kp is not None else float(gripper.config.kp)
        self._kd = float(kd) if kd is not None else float(gripper.config.kd)
        self._loop = bool(loop)
        self._align = bool(align)
        # Where this trajectory's own clock starts.  Zero for anything this
        # SDK recorded; a hand-built or foreign one may not be, and
        # openness_at() indexes by the absolute stamp, so the phase is kept
        # separately from the elapsed time the pacing works in.
        self._origin = float(trajectory.samples[0].t)
        if rate_hz is None:
            interval = float(gripper.motion_config.frame_interval)
            rate_hz = 1.0 / interval if interval > 0.0 else 200.0
        self._rate_hz = float(rate_hz)
        self._dt = 1.0 / self._rate_hz
        self._sleep_fn, self._monotonic_fn = _resolve_seams(
            gripper, sleep_fn, monotonic_fn)
        self._pacer = _Pacer(self._dt, self._sleep_fn, self._monotonic_fn)

        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._error: Optional[BaseException] = None
        self._frames = 0
        self._last_openness = trajectory.samples[0].openness
        self._completed = False

    # ── lifecycle ─────────────────────────────────────────────────────

    @property
    def is_playing(self) -> bool:
        return self._running

    @property
    def is_finished(self) -> bool:
        """True once the trajectory has played through to its end."""
        return self._completed

    def start(self) -> None:
        """Start replaying.  Raises :class:`TrajectoryBusyError` if already on."""
        if self._running:
            raise TrajectoryBusyError("trajectory replay is already running")
        self._running = True
        self._thread = threading.Thread(
            target=self._run, name="litegrip-trajectory-play", daemon=True)
        self._thread.start()
        log.info("replay started: %d samples, %.2fs, speed=%.2f loop=%s",
                 len(self._traj), self._traj.duration, self._speed, self._loop)

    def _run(self) -> None:
        try:
            if self._align:
                self._align_to_start()
            self._loop_frames()
        except BaseException as e:  # noqa: BLE001 — surfaces through status()
            self._error = e
            log.warning("replay loop stopped: %s", e)
        finally:
            self._running = False

    def _align_to_start(self) -> None:
        """Walk to sample 0 at a bounded speed, so following starts in place.

        This used to be one ``goto_rad(..., duration=1.0)``.  ``duration`` reads
        like a ramp but is only a deadline: the CAN layer sends ``q = q_target``
        from the first frame and holds it, so that frame demanded ``kp`` times
        the whole error.  That is the same full-torque step #22 reports for the
        teleop follower — at the shipped ``kp`` of 100 Nm/rad any error past
        ~0.1 rad saturates the DM4310.

        The target is walked in at ``motion_config.speed_mm_s`` with that same
        speed fed forward as ``dq``, and each frame is capped to
        ``motion_config.max_lead_mm`` of lead.  Torque is
        ``kp * (q_cmd - q_measured)``, so capping the lead caps the commanded
        torque by construction — the rule ``open``/``close`` already travel
        under.
        """
        gcfg = self._g.config
        mcfg = self._g.motion_config
        target = openness_to_rad(self._traj.samples[0].openness, gcfg)
        log.info("replay aligning to first sample: openness=%.3f -> %.3f rad",
                 self._traj.samples[0].openness, target)
        try:
            start = self._g.get_state(wait=False).position_rad
        except Exception as e:  # noqa: BLE001
            log.warning("replay align state read failed: %s", e)
            return
        rad_to_mm = float(getattr(gcfg, "rad_to_mm", 0.0) or 0.0)
        speed_mm_s = float(getattr(mcfg, "speed_mm_s", 0.0) or 0.0)
        dist_rad = target - start
        if rad_to_mm <= 0.0 or speed_mm_s <= 0.0 or abs(dist_rad) < 1e-9:
            # Nothing to schedule against: the jaws are already at the target,
            # or the config cannot give us a speed.  One frame hands over.
            self._send_rad(target)
            return
        speed_rad_s = speed_mm_s / rad_to_mm
        sign = 1.0 if dist_rad >= 0.0 else -1.0
        cap_rad = float(getattr(mcfg, "max_lead_mm", 0.0) or 0.0) / rad_to_mm
        steps = max(1, int(round(abs(dist_rad) / speed_rad_s / self._dt)))
        log.info("replay align: %+.4f -> %+.4f rad at %.1f mm/s "
                 "(%d frames, cap %.4f rad)",
                 start, target, speed_mm_s, steps, cap_rad)
        for i in range(1, steps + 1):
            if not self._running:
                return
            cycle_start = self._monotonic_fn()
            q_sched = start + dist_rad * (i / steps)
            pos = self._g.get_state(wait=False).position_rad
            lead = q_sched - pos
            if cap_rad > 0.0:
                if lead > cap_rad:
                    q_sched = pos + cap_rad
                elif lead < -cap_rad:
                    q_sched = pos - cap_rad
            self._send_rad(q_sched, dq=sign * speed_rad_s)
            self._pacer.rest(cycle_start)

    def _loop_frames(self) -> None:
        duration = self._traj.duration
        t0 = self._monotonic_fn()
        last = t0
        stall = 0
        while self._running:
            cycle_start = self._monotonic_fn()
            # The recorder's rule, for the same reason: trajectory time is
            # read off this clock, so a clock that will not move means the
            # replay can never reach its end — and a loop that keeps sending
            # frames it cannot advance past is flooding the bus, not playing.
            if cycle_start - last <= _CLOCK_EPS:
                stall += 1
                if stall >= _STALL_CYCLES:
                    raise TrajectoryError(
                        f"回放时钟连续 {stall} 拍没有前进 (t={cycle_start}) —— "
                        f"回放推不动, 拒绝空转刷帧")
            else:
                stall = 0
            last = cycle_start
            t0 = self._emit_cycle(cycle_start, t0, duration)
            if t0 is None:
                return
            self._pacer.rest(cycle_start)

    def _emit_cycle(self, now: float, t0: float,
                    duration: float) -> Optional[float]:
        """Emit one frame; return the (possibly rewound) epoch, or None to stop."""
        if duration <= 0.0:
            # A one-sample trajectory is a pose, not a path: there is no time
            # to advance along.  Looping it means holding that pose, which is
            # the only reading that keeps `loop=True` meaning "keeps going
            # until play_stop" — a pose is exactly what a hold is for.
            if self._loop:
                self._emit(self._traj.samples[0].openness)
                return t0
            self._emit(self._traj.samples[-1].openness)
            self._completed = True
            log.info("replay finished: %d frames", self._frames)
            return None

        # Elapsed seconds along the trajectory, from the wall clock — never an
        # index stepped once per cycle, which would tie the speed to the loop
        # rate and make a recorded 2 s path take longer every time.
        elapsed = (now - t0) * self._speed
        if elapsed >= duration:
            if self._loop:
                # Rewind by whole loops rather than resetting to `now`: a cycle
                # that overran keeps its phase instead of shifting the loop.
                elapsed %= duration
                t0 = now - elapsed / self._speed
            else:
                self._emit(self._traj.samples[-1].openness)
                self._completed = True
                log.info("replay finished: %d frames", self._frames)
                return None
        self._emit(self._traj.openness_at(self._origin + elapsed))
        return t0

    def _emit(self, openness: float) -> None:
        self._send_rad(openness_to_rad(openness, self._g.config))
        self._last_openness = openness

    def _send_rad(self, q: float, dq: float = 0.0) -> None:
        """Send one MIT frame at ``q`` (radians), counting it.

        Shared by the replay loop (``dq = 0``) and the align ramp, which feeds
        its travel speed forward as ``dq``.  A failed send aborts: dropping
        frames silently would leave the jaws somewhere the status does not show.
        """
        if not self._g.send_mit_frame(q=q, kp=self._kp, kd=self._kd, dq=dq):
            raise TrajectoryError(
                "MIT 帧下发失败 (未连接或未使能?) —— 回放中止, 不静默丢帧")
        self._frames += 1

    def stop(self, timeout: float = 2.0) -> None:
        """Stop replaying and leave the gripper holding its last target.

        One MIT frame is sent at the current position under the configured
        gains.  That holds the jaws for as long as frames keep arriving — the
        motor self-locks a comm-loss fault about 100 ms after they stop — so
        call the next action promptly, or keep the player running with
        ``loop=True`` if the hold has to last.
        """
        self._running = False
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=timeout)
        self._thread = None
        try:
            self._g._hold_position()
        except Exception as e:  # noqa: BLE001
            log.debug("hold on replay stop failed: %s", e)
        log.info("replay stopped: %d frames", self._frames)

    def wait(self, timeout: float) -> bool:
        """Block until the replay finishes.  True if it did, False on timeout.

        A wall-clock playback of a stalling clock would otherwise spin forever,
        so the blocking wrapper gives it a deadline and this is how it asks.
        """
        thread = self._thread
        if thread is None:
            return True
        thread.join(timeout)
        return not thread.is_alive()

    def status(self) -> dict:
        """A snapshot of the replay session, for logging and diagnostics."""
        return {
            "active": self._running,
            "kind": "play",
            "samples": len(self._traj),
            "frames": self._frames,
            "speed": round(self._speed, 3),
            "loop": self._loop,
            "completed": self._completed,
            "openness": round(self._last_openness, 4),
            "loop_hz": round(self._pacer.loop_hz, 1),
            "error": None if self._error is None else str(self._error),
        }
