"""LiteGrip SDK — high-level gripper API.

The LiteGrip class is the primary entry point.  It manages a single
gripper motor (DM4310 by default) over CAN, exposing an intuitive
open/close/grasp interface with automatic unit conversion.

Example::

    from litegrip import LiteGrip

    with LiteGrip(channel="can0", can_id=0x08) as gripper:
        gripper.load_calibration()
        gripper.enable()          # 反复重试直到状态帧确认 err == 1
        gripper.open()
        gripper.grasp(force_n=20.0, hold_s=3.0)
        state = gripper.get_state()
        print(f"Position: {state.position_mm:.1f} mm, Force: {state.force_n:.1f} N")

``enable()`` / ``disable()`` / ``open()`` / ``close()`` / ``grasp()`` /
``zero()`` 都转发到 :class:`~litegrip.actions.GripperActions`（见
:attr:`LiteGrip.actions`），运动参数在 :attr:`LiteGrip.motion_config` 上。
"""

from __future__ import annotations

import logging
import os as _os
import select as _select_mod
import sys
import threading
import time
from datetime import datetime
from typing import Callable, List, Optional

from .can.motor import MotorType
from .protocols.can_bus import LiteGripCAN
from .actions import (
    EnableResult,
    GraspResult,
    GripperActions,
    MotionConfig,
    MoveProgress,
    MoveResult,
)
from .teleop import (DEFAULT_DQ_MAX, DEFAULT_GRIP_ID, DEFAULT_GRIP_PORT,
                     DEFAULT_TORQUE_LIMIT_NM)


def _zenoh_transport(role: str, key: str, port: int,
                     host: Optional[str]) -> "TeleopTransport":
    """Build a point-to-point zenoh transport, or explain how to get one.

    zenoh is an optional dependency, so a bare ``import`` failure is turned into
    an actionable message rather than a ModuleNotFoundError naming a module the
    user never asked for.
    """
    try:
        from .zenoh_link import ZenohTeleopTransport
    except ImportError as e:
        raise ImportError(
            "the zenoh teleoperation link needs the optional zenoh dependency — "
            "install it with `pip install litegrip[zenoh]`") from e
    return ZenohTeleopTransport(role, key, port=port, host=host)


# Path to built-in factory calibration (ships with the package, read-only fallback).
_FACTORY_CALIB = _os.path.join(_os.path.dirname(__file__), "factory_calibration.json")

# Pre-made calibration templates shipped with the package.  Each is a pair of
# nominal limits whose *ordering* declares the mounting direction: "normal"
# closes at the larger rad value, "reverse" at the smaller one.  They are a
# labelling seed, not a per-unit calibration — the limits are nominal, so run
# zero() to measure the real travel.  Pick one by eye after watching which way
# the jaws move::
#
#     gripper.load_calibration(CALIB_TEMPLATES["reverse"])   # by path
#     gripper.load_template("reverse")                       # or by name
#
# They deliberately carry no `channel`, `can_id`, `mst_id` or gain values: a
# template declares a *direction*, and adopting a device identity or a tuned
# kp/kd from it would silently rewrite what the caller set.  Use
# :func:`list_templates` to enumerate the names for a UI.
CALIB_TEMPLATES = {
    "normal": _os.path.join(_os.path.dirname(__file__), "calibration_normal.json"),
    "reverse": _os.path.join(_os.path.dirname(__file__), "calibration_reverse.json"),
}

# The pre-per-channel location, kept so a calibration saved by an older
# version still loads.  DEFAULT_CALIB keeps its historical value (env override
# included); it is now the *legacy* entry of the automatic load chain rather
# than the primary one.
_LEGACY_CALIB_PATH = _os.path.join(
    _os.path.expanduser("~"), ".litegrip", "litegrip_calibration.json")

# Default user calibration path — a stable absolute location so that a
# calibration saved without an explicit path is picked up on the next load
# without an explicit path, regardless of the process working directory.
# Override with the LITEGRIP_CALIB env var if desired.
DEFAULT_CALIB = _os.environ.get("LITEGRIP_CALIB", _LEGACY_CALIB_PATH)


from .models import (
    GripperState,
    GripperConfig,
    GripperInfo,
    GripperStatus,
    CalibrationData,
)
from .constants import (
    GripperParams,
    UnitConversion,
    DefaultParams,
    describe_error,
)
from .exceptions import (
    CommandError,
    ConnectError,
    CommError,
    HardwareError,
    NotInitializedError,
)


def default_calib_path(channel: str = DefaultParams.CAN_CHANNEL) -> str:
    """Where a calibration for *channel* is saved and read back by default.

    One gripper per channel, one file per channel —
    ``~/.litegrip/<channel>_calibration.json``.  A single shared file let two
    grippers on one machine (``can0`` / ``can1``, both at CAN ID 0x08)
    overwrite each other's direction and travel.

    ``LITEGRIP_CALIB``, when set, overrides this with one explicit path for
    every channel.  ``HOME`` and the env var are read per call, so a caller
    (or a test) can redirect them.
    """
    env = _os.environ.get("LITEGRIP_CALIB")
    if env:
        return env
    return _os.path.join(_os.path.expanduser("~"), ".litegrip",
                         f"{channel}_calibration.json")


def list_templates() -> List[str]:
    """The calibration template names, in declaration order (normal first).

    For a UI that has to offer the mount as a choice.
    """
    return list(CALIB_TEMPLATES)


def _resolve_template(name: str) -> str:
    """Template *name* → path.  Unknown names are a hard error."""
    try:
        return CALIB_TEMPLATES[name]
    except KeyError:
        raise CommandError(
            f"未知的标定模板 {name!r}；可用的是 "
            f"{', '.join(repr(n) for n in CALIB_TEMPLATES)}"
            f"（也可以直接给文件路径）。") from None


def _dedupe(paths: List[str]) -> List[str]:
    """Drop repeats, keep order — ``LITEGRIP_CALIB`` can alias several entries."""
    seen = set()
    out: List[str] = []
    for p in paths:
        if p not in seen:
            seen.add(p)
            out.append(p)
    return out


log = logging.getLogger("litegrip")


class LiteGrip:
    """LiteGrip adaptive two-finger gripper.

    Parameters
    ----------
    channel:
        CAN interface name (``"can0"``, ``"vcan0"``, etc.).
    can_id:
        Motor CAN ID (default 0x08).
    mst_id:
        Motor master ID for status frames.  ``None`` = auto-detect.
    canfd_mode:
        ``True`` to prefer CAN FD.  ``None`` = auto-detect from interface MTU.
    motor_type:
        Damiao motor model (default DM4310).
    config:
        Full GripperConfig for advanced tuning.
    motion_config:
        Motion parameters for the high-level moves (see
        :class:`~litegrip.actions.MotionConfig`).  Also settable at runtime
        via the :attr:`motion_config` property.
    disable_on_disconnect:
        ``True`` (default) → :meth:`disconnect` sends a disable command first,
        so the gripper goes limp when the object is released.  Set ``False``
        to leave the motor enabled after disconnecting.
    mount:
        Template name declaring which way this unit is mounted —
        ``"normal"`` or ``"reverse"`` (see :data:`CALIB_TEMPLATES` and
        :func:`list_templates`).  Loads that template immediately, so
        ``LiteGrip("can1", mount="reverse")`` is all a two-gripper setup
        needs.  ``None`` (default) loads nothing; call
        :meth:`load_calibration` yourself.  This *declares* the mount; read
        the resulting direction back from :attr:`GripperConfig.mount`.

    The gripper works as a context manager::

        with LiteGrip("can0") as gripper:
            gripper.enable()
            gripper.open()
    """

    def __init__(
        self,
        channel: str = DefaultParams.CAN_CHANNEL,
        can_id: int = GripperParams.CAN_ID,
        mst_id: Optional[int] = None,
        canfd_mode: Optional[bool] = None,
        motor_type: MotorType = MotorType.DM4310,
        config: Optional[GripperConfig] = None,
        motion_config: Optional[MotionConfig] = None,
        disable_on_disconnect: bool = True,
        mount: Optional[str] = None,
    ):
        cfg = config or GripperConfig()
        self._channel = channel if channel != DefaultParams.CAN_CHANNEL or config is None else cfg.can_channel
        self._can_id = can_id if can_id != GripperParams.CAN_ID or config is None else cfg.can_id
        self._mst_id = mst_id if mst_id is not None else cfg.mst_id
        self._canfd_mode = canfd_mode if canfd_mode is not None else cfg.canfd_mode
        self._motor_type = motor_type
        self._config = cfg
        self._disable_on_disconnect = disable_on_disconnect

        self._can: Optional[LiteGripCAN] = None
        self._enabled = False
        self._connected = False
        self._status_flags = GripperStatus.NONE
        self._actions = GripperActions(self, motion_config)

        # Reentrant lock around the low-level CAN I/O that a teleop background
        # thread shares with the caller (send/poll/get_state).  A no-op for the
        # single-threaded use this SDK assumed before teleop existed.
        self._io_lock = threading.RLock()
        self._teleop: Optional["GripperTeleop"] = None
        # Transport teleop_start built itself (as opposed to one the caller
        # injected), so teleop_stop knows what it is allowed to close.
        self._teleop_transport: Optional["TeleopTransport"] = None
        # The leader's zenoh publisher endpoint, kept for the life of this
        # gripper rather than per session — rebuilding it per session leaves the
        # port bound and makes matching fail intermittently.
        self._teleop_pub: Optional["TeleopTransport"] = None

        # One long-running session at a time.  Teleoperation and a trajectory
        # both own the CAN I/O for their whole duration and both run in their
        # own thread, so "is teleop running? no → start recording" is a
        # check-then-act race that lets two loops interleave frames on one
        # motor.  Claiming and releasing under one lock closes it.  The owner
        # string names the session, so the refusal says who is in the way.
        self._session_lock = threading.Lock()
        self._session_owner: Optional[str] = None
        self._trajectory_recorder: Optional["TrajectoryRecorder"] = None
        self._trajectory_player: Optional["TrajectoryPlayer"] = None

        # Declaring the mount is just loading the matching template, so it
        # costs no CAN traffic and is safe this early.  The template carries
        # only a direction and geometry, so it cannot clobber the identity
        # arguments above or a tuned config.
        if mount is not None:
            self.load_template(mount)

    # ═══════════════════════════════════════════════════════════════════
    # Properties
    # ═══════════════════════════════════════════════════════════════════

    @property
    def channel(self) -> str:
        return self._channel

    @property
    def can_id(self) -> int:
        return self._can_id

    @property
    def mst_id(self) -> Optional[int]:
        return self._mst_id

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_enabled(self) -> bool:
        return self._enabled

    @property
    def config(self) -> GripperConfig:
        return self._config

    @property
    def mount(self) -> Optional[str]:
        """``"normal"`` / ``"reverse"``, or ``None`` while uncalibrated.

        Read back from the loaded limits (see
        :attr:`~litegrip.GripperConfig.mount`) — not necessarily the name
        that was *declared*, since a calibration run keeps the direction but
        rewrites the travel.
        """
        return self._config.mount

    @property
    def actions(self) -> GripperActions:
        """High-level motion API: ``open`` / ``close`` / ``grasp`` / ``zero`` /
        ``enable`` / ``disable``."""
        return self._actions

    @property
    def motion_config(self) -> MotionConfig:
        """Motion parameters used by the high-level moves.  Settable."""
        return self._actions.config

    @motion_config.setter
    def motion_config(self, value: MotionConfig) -> None:
        self._actions.config = value

    @property
    def disable_on_disconnect(self) -> bool:
        return self._disable_on_disconnect

    @disable_on_disconnect.setter
    def disable_on_disconnect(self, value: bool) -> None:
        self._disable_on_disconnect = value

    # ═══════════════════════════════════════════════════════════════════
    # Connection
    # ═══════════════════════════════════════════════════════════════════

    def connect(self) -> bool:
        """Open CAN bus and register the gripper motor.

        If *mst_id* was ``None`` at construction, it is auto-detected here.
        """
        if self._connected:
            return True

        try:
            self._can = LiteGripCAN(
                channel=self._channel,
                canfd_mode=self._canfd_mode,  # None → auto-detect in transport
            )
            self._can.connect()

            # Register motor; mst_id=None triggers auto-detect
            actual_mst = self._can.register_gripper(
                can_id=self._can_id,
                mst_id=self._mst_id,
                motor_type=self._motor_type,
            )
            if self._mst_id is None:
                self._mst_id = actual_mst

            self._connected = True
            self._status_flags = GripperStatus.NONE
            log.info("LiteGrip connected: %s", self)
            return True
        except ConnectError:
            raise
        except Exception as e:
            raise ConnectError(f"连接失败: {e}")

    def disconnect(self) -> None:
        """Close the CAN transport.

        Sends a disable command first unless :attr:`disable_on_disconnect`
        is ``False`` (in which case the motor stays enabled).
        """
        if not self._connected:
            return

        if self._teleop is not None:
            self.teleop_stop()
        self._close_teleop_pub()
        # A recording left running would keep sampling a bus that is about to
        # close, and a replay would keep commanding one.  stop() also leaves the
        # gripper holding rather than slack.
        if self._trajectory_recorder is not None:
            self._trajectory_recorder.stop()
            self._release_session("record")
            self._trajectory_recorder = None
        if self._trajectory_player is not None:
            self._trajectory_player.stop()
            self._release_session("play")
            self._trajectory_player = None

        if self._can:
            self._can.disconnect(disable=self._disable_on_disconnect)
            self._can = None

        self._connected = False
        self._enabled = False
        self._status_flags = GripperStatus.NONE

    # ═══════════════════════════════════════════════════════════════════
    # Enable / disable / fault
    # ═══════════════════════════════════════════════════════════════════

    def enable(self, retries: Optional[int] = None) -> EnableResult:
        """Enable the gripper motor and verify it actually took.

        Retries ``enable`` until a status frame reports ``err == 1`` (真使能)
        — the frame must be read back, because ``enable`` is a one-way CAN
        command and a dropped frame would otherwise go unnoticed.  Real
        faults (err ∉ {0, 1}) are cleared before retrying.

        Args:
            retries: Attempts.  ``None`` = ``MotionConfig.enable_retries``.

        Returns:
            :class:`~litegrip.actions.EnableResult` — truthy when enabled.
        """
        result = self._actions.enable(retries)
        # 以「回读到的状态帧」为准，而不是以某一次 initialize() 的返回值为准
        self._enabled = result.ok
        if result.ok:
            self._status_flags |= GripperStatus.ENABLED
        else:
            self._status_flags &= ~GripperStatus.ENABLED
        return result

    def _enable_once(self) -> bool:
        """One ``enable`` attempt: full init (disable → MIT → enable →
        feedback).  No retry, no state-frame verification — that is
        :meth:`actions.enable` / :meth:`enable`'s job.

        Raises:
            HardwareError: the motor did not end up enabled.
        """
        self._check_connected()

        err = self.get_error()
        if err not in (0, 1):
            self.clear_fault()

        try:
            self._enabled = self._can.initialize()
            if self._enabled:
                self._status_flags |= GripperStatus.ENABLED
            return self._enabled
        except HardwareError:
            raise
        except Exception as e:
            raise HardwareError(f"使能失败: {e}")

    def disable(self) -> bool:
        """Disable the gripper motor."""
        return self._actions.disable()

    def _disable_once(self) -> bool:
        """Send a single disable command and clear the enabled flags."""
        self._check_connected()
        try:
            result = self._can.disable()
            self._enabled = False
            self._status_flags &= ~GripperStatus.ENABLED
            return result
        except Exception:
            self._enabled = False
            return False

    def clear_fault(self) -> bool:
        """Clear latched faults (UV / OC / OT).

        Sequence: disable → clear(0xFB) → enable → verify.
        Retries up to *FAULT_CLEAR_RETRIES* times.
        """
        self._check_connected()
        if self._can is None:
            return False

        cleared = self._can.clear_fault()
        if not cleared:
            err = self._can.get_error()
            raise HardwareError(
                f"故障清除失败: {describe_error(err)} (错误码 0x{err:X})",
                error_code=err,
            )
        self._enabled = True
        self._status_flags |= GripperStatus.ENABLED
        return True

    def stop(self) -> None:
        """Emergency stop — send zero-torque MIT frame.

        Does NOT disable the motor; the motor stays enabled but exerts
        zero torque so it can be back-driven.
        """
        if self._can is not None and self._enabled:
            self._can.control_mit(q_target=0, kp=0, kd=0)
            self._can.update_state(timeout_s=0.02)

    def send_mit_frame(
        self,
        q: float,
        kp: float,
        kd: float,
        dq: float = 0.0,
        tau: float = 0.0,
    ) -> bool:
        """Send a single MIT control frame — expert/low-level use.

        For sustained motion use :meth:`goto_rad` or :meth:`move_at_speed`
        instead.  This method is exposed for custom control loops that
        manage their own timing (e.g. force-feedback monitors).

        Args:
            q: Target position in rad.
            kp: Position stiffness.
            kd: Velocity damping.
            dq: Target velocity in rad/s.
            tau: Feed-forward torque in Nm.

        Returns:
            True if the frame was sent.
        """
        if self._can is None or not self._enabled:
            return False
        with self._io_lock:
            return self._can.control_mit(
                q_target=q, kp=kp, kd=kd, dq_target=dq, tau_feedforward=tau)

    def poll(self, timeout_s: float = 0.0) -> bool:
        """Poll for one CAN frame and update cached motor state.

        Args:
            timeout_s: Max wait time in seconds (0 = non-blocking).

        Returns:
            True if a status frame for this gripper was received.
        """
        if self._can is None:
            return False
        with self._io_lock:
            return self._can.poll(timeout_s=timeout_s)

    # ═══════════════════════════════════════════════════════════════════
    # Zero-gravity mode (manual back-driving)
    # ═══════════════════════════════════════════════════════════════════

    def enter_zero_gravity(self, duration: float = 0.0) -> None:
        """Enter zero-gravity mode — motor stays enabled but exerts no torque.

        The gripper can be freely moved by hand.  Call :meth:`exit_zero_gravity`
        or any motion command to resume normal control.

        Args:
            duration: Seconds to sustain zero-gravity.  If > 0, blocks for
                      that duration while continuously streaming kp=0, kd=0
                      frames.  If 0 (default), the caller must call
                      :meth:`update_state` or poll manually to sustain the
                      mode (single frame sent as bootstrap).
        """
        self._check_connected()
        self._check_enabled()
        if self._can is None:
            return

        if duration > 0:
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                self._can.control_mit(q_target=0, kp=0, kd=0, tau_feedforward=0)
                self._can.poll(timeout_s=0.0)
                time.sleep(0.005)
            log.info("Zero-gravity mode ended (duration=%.1fs)", duration)
        else:
            self._can.control_mit(q_target=0, kp=0, kd=0, tau_feedforward=0)
            log.info("Zero-gravity mode: gripper is freely back-drivable "
                     "(caller must poll/sustain)")

    def exit_zero_gravity(self) -> None:
        """Exit zero-gravity mode and hold current position."""
        if self._can is not None and self._enabled:
            self._hold_position()
            log.info("Zero-gravity mode exited; holding position")

    def _hold_position(self) -> None:
        """Send one MIT frame holding the current position under the configured gains.

        The one "hold, do not go slack" primitive in this SDK: the teleop
        master and a stopping trajectory replay both need it, and neither wants
        zero-gravity mode's log line.  It is one frame, not a sustained hold —
        the motor self-locks a communication-loss fault about 100 ms after the
        frames stop, so a caller that needs the jaws held longer has to keep
        sending (or call a motion action).
        """
        if self._can is None or not self._enabled:
            return
        # Under the same lock as every other CAN operation: a caller normally
        # joins its loop before holding, but a join that times out would
        # otherwise let this frame overtake one still being written.
        with self._io_lock:
            current_pos = self._can.get_position()
            self._can.control_mit(q_target=current_pos, kp=self._config.kp,
                                  kd=self._config.kd, tau_feedforward=0)

    # ═══════════════════════════════════════════════════════════════════
    # Manual calibration (zero-gravity assisted)
    # ═══════════════════════════════════════════════════════════════════

    def calibrate_manual(
        self,
        duration: float = 30.0,
        settle_time: float = 2.0,
        sample_interval: float = 0.01,
    ) -> CalibrationData:
        """Calibrate by manually moving the gripper in zero-gravity mode.

        The motor enters zero-torque mode so you can freely push/pull the
        gripper jaws through their full range.  The SDK records the two
        extremes reached — which one is "closed" depends on the mounting
        direction declared by the current configuration (see
        :attr:`GripperConfig.close_sign`), not on which is numerically larger.

        Usage::

            with LiteGrip("can0") as gripper:
                gripper.enable()
                result = gripper.calibrate_manual(duration=30.0)
                print(f"行程: {result.travel_mm:.1f} mm")

        **Procedure:**

        1. Call this method — the gripper goes limp (zero torque).
        2. Manually push the jaws fully closed, then fully open.
        3. Repeat a few times to ensure limits are captured.
        4. The method returns after *duration* seconds, or press Ctrl+C
           to stop early and keep the best readings so far.

        Args:
            duration: Max recording time in seconds.
            settle_time: Extra seconds at the end to hold position before
                         exiting zero-gravity.
            sample_interval: Polling interval in seconds (~100 Hz default).

        Returns:
            CalibrationData with zero_position, max_position, travel_range,
            and updated rad_to_mm.  The internal :attr:`config` is also updated.
        """
        self._check_connected()
        self._check_enabled()
        if self._can is None:
            raise NotInitializedError("未连接")

        print("=" * 60)
        print("  零重力手动标定")
        print("=" * 60)
        print()
        print("  夹爪已进入零重力模式，可以自由用手掰动。")
        print()
        print("  操作步骤：")
        print("    1. 将夹爪推到完全闭合位置")
        print("    2. 将夹爪拉到完全张开位置")
        print("    3. 反复推拉几次确保极限被抓到")
        print()
        print(f"  标定将持续 {duration:.0f} 秒（可按 Ctrl+C 提前结束）")
        print("=" * 60)
        print()

        # Track the two extremes; the mounting direction (close_sign) decides
        # which of them is the closed limit.  A normal mount closes at the
        # larger rad value, a reverse mount at the smaller one.
        s = self._config.close_sign
        lo_rad = float("+inf")     # most negative rad seen
        hi_rad = float("-inf")     # most positive rad seen
        sample_count = 0

        def _label(rad: float) -> str:
            at_lo = rad <= lo_rad
            if s > 0:
                return "张开极限" if at_lo else "闭合极限"
            return "闭合极限" if at_lo else "张开极限"

        deadline = time.monotonic() + duration

        try:
            while time.monotonic() < deadline:
                # Stream zero-torque MIT frame
                self._can.control_mit(q_target=0, kp=0, kd=0, tau_feedforward=0)

                # Poll for latest position
                self._can.poll(timeout_s=0.0)

                pos = self._can.get_position()
                # Only count positions that look like real feedback
                if abs(pos) < 50.0:
                    sample_count += 1
                    if pos < lo_rad:
                        lo_rad = pos
                        print(f"  ★ 新{_label(pos)}: {pos:.6f} rad")
                    elif pos > hi_rad:
                        hi_rad = pos
                        print(f"  ★ 新{_label(pos)}: {pos:.6f} rad")

                # Progress indicator (every ~1 second)
                remaining = deadline - time.monotonic()
                if sample_count % 100 == 0 and sample_count > 0:
                    print(f"  ... 剩余 {remaining:.0f}s  |  当前 pos={pos:.4f} rad  "
                          f"|  lo={lo_rad:.4f}  hi={hi_rad:.4f}")

                time.sleep(sample_interval)

        except KeyboardInterrupt:
            print("\n  ⏎ 用户提前结束标定")

        # ── Settle ──────────────────────────────────────────────────────
        print(f"\n  保持零重力 {settle_time:.0f} 秒（稳定位置）...")
        settle_deadline = time.monotonic() + settle_time
        while time.monotonic() < settle_deadline:
            self._can.control_mit(q_target=0, kp=0, kd=0, tau_feedforward=0)
            self._can.poll(timeout_s=0.0)
            pos = self._can.get_position()
            lo_rad = min(lo_rad, pos)
            hi_rad = max(hi_rad, pos)
            time.sleep(sample_interval)

        # ── Exit zero-gravity ───────────────────────────────────────────
        self.exit_zero_gravity()
        time.sleep(0.1)

        # ── Validate ────────────────────────────────────────────────────
        if lo_rad == float("+inf"):
            raise RuntimeError(
                "标定失败：未能捕获有效的位置范围。"
                "请确保夹爪使能正常且有反馈。"
            )

        close_rad, open_rad = (hi_rad, lo_rad) if s > 0 else (lo_rad, hi_rad)

        travel = abs(close_rad - open_rad)
        if travel <= 0:
            raise RuntimeError(
                f"标定失败：行程异常 ({travel:.6f} rad)。"
                "请重新标定。"
            )

        rad_to_mm = self._config.max_stroke_mm / travel if travel > 0 else 105.26

        result = CalibrationData(
            zero_position=round(close_rad, 6),      # closed → 0 mm
            max_position=round(open_rad, 6),        # open → max mm
            travel_range=round(travel, 6),
            rad_to_mm=round(rad_to_mm, 2),
            motor_type=self._motor_type.name,
            can_id=self._can_id,
            mst_id=self._mst_id or 0,
            calibration_time=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        )

        # Update config.  The ordering the two values end up in is the
        # direction declaration: close on the close_sign side, open on the
        # other, whatever the mount.
        self._config.pos_closed_rad = result.zero_position
        self._config.pos_open_rad = result.max_position
        self._config.rad_to_mm = result.rad_to_mm
        self._config.calibrated = True

        print(f"\n{'=' * 60}")
        print(f"  标定完成")
        print(f"{'=' * 60}")
        print(f"  样本数:     {sample_count}")
        print(f"  闭合极限:   {result.zero_position:.6f} rad  (0.0 mm)")
        print(f"  张开极限:   {result.max_position:.6f} rad  "
              f"({result.travel_mm:.1f} mm)")
        print(f"  行程:       {result.travel_range:.6f} rad  "
              f"({result.travel_mm:.1f} mm)")
        print(f"  转换系数:   {result.rad_to_mm:.1f} mm/rad")
        print(f"{'=' * 60}")

        return result

    # ═══════════════════════════════════════════════════════════════════
    # Guided calibration (step-by-step, user confirms each limit)
    # ═══════════════════════════════════════════════════════════════════

    def calibrate_guided(
        self,
        kp: float = 20.0,
        kd: float = 2.0,
        step_rad: float = 0.05,
        stall_delta: float = 0.0015,
        stall_cycles: int = 5,
        max_iter: int = 200,
        tau_limit: Optional[float] = 2.0,
    ) -> CalibrationData:
        """Guided two-step calibration with user confirmation at each limit.

        The gripper moves itself (low stiffness) toward each limit.  You
        press Enter when the limit is reached.

        **Procedure:**

        1. Gripper steps toward the OPEN direction.  Watch the position.
           Press Enter when fully open (or when it stalls at the hard stop).
        2. Gripper steps toward the CLOSE direction.  Press Enter when
           fully closed.
        3. Calibration is saved to :attr:`config` automatically.

        Args:
            kp: Probing stiffness (low = gentle).
            kd: Probing damping.
            step_rad: Step size per iteration (rad).
            stall_delta: Position delta for auto-stall detection (rad).
            stall_cycles: Consecutive stalls to auto-confirm limit.
            max_iter: Max steps per direction.
            tau_limit: Torque ceiling in Nm; the probe stops when ``|tau|``
                reaches it (``None`` disables the ceiling).  The command lead is
                already bounded to ``step_rad`` here, so this is the second,
                independent guard.

        Returns:
            CalibrationData; :attr:`config` is updated in-place.
        """
        self._check_connected()
        self._check_enabled()
        if self._can is None:
            raise NotInitializedError("未连接")

        print("=" * 60)
        print("  引导式标定")
        print("=" * 60)
        print()
        print("  夹爪将自动缓慢移动。到达极限时按 Enter 确认。")

        s = self._config.close_sign
        print(f"  方向：{'正向' if s > 0 else '反装'}（由当前配置的限位顺序推出）")
        print()

        def _step_to_limit(direction: str, sign: float, label: str) -> float:
            """Step in *direction* until user presses Enter or stall."""
            print(f"  [{direction}] 正在{label}...")
            print(f"    按 Enter 确认到达{label}，或等待自动检测堵转")
            print()

            self._can.update_state(timeout_s=0.05)
            current = self._can.get_position()
            stall = 0

            for i in range(max_iter):
                target = current + sign * step_rad
                self._can.control_mit_stream(
                    target, kp, kd, duration_s=0.3, interval_s=0.005)
                self._can.update_state(timeout_s=0.1)

                new_pos = self._can.get_position()
                delta = abs(new_pos - current)
                tau = self._can.get_torque()

                # Non-blocking keyboard check
                hit_enter = False
                if sys.stdin.isatty():
                    r, _, _ = _select_mod.select([sys.stdin], [], [], 0)
                    if r:
                        sys.stdin.readline()
                        hit_enter = True

                print(f"    [{i}] pos={new_pos:.4f} rad  d={delta:.5f}  "
                      f"tau={tau:+.3f}  stall={stall}", end="")
                if hit_enter:
                    print("  ← 用户确认")
                    return new_pos

                if tau_limit is not None and abs(tau) >= tau_limit:
                    print(f"  ← 力矩达到上限 {tau_limit:.2f} Nm（{tau:+.3f}）")
                    print(f"    → 停止推进并保持: {new_pos:.6f} rad")
                    return new_pos

                if delta < stall_delta:
                    stall += 1
                    print(f"  (堵转检测中)")
                    if stall >= stall_cycles:
                        print(f"    → 自动检测到{label}: {new_pos:.6f} rad")
                        return new_pos
                else:
                    stall = 0
                    print()

                current = new_pos

            print(f"    → 安全停止（达到最大步数 {max_iter}）: {current:.4f} rad")
            return current

        # ── Step 1: Open ─────────────────────────────────────────────────
        print("━" * 60)
        print("  第 1 步：张开")
        print("━" * 60)
        open_rad = _step_to_limit("open", sign=-s, label="张开极限")

        # Small back-off, away from the open stop toward the close side
        print(f"\n  回退一小段...")
        self._can.control_mit_stream(open_rad + s * 0.15, kp=80, kd=kd,
                                     duration_s=0.5, interval_s=0.005)
        time.sleep(0.1)

        # ── Step 2: Close ────────────────────────────────────────────────
        print(f"\n{'━' * 60}")
        print("  第 2 步：闭合")
        print("━" * 60)
        close_rad = _step_to_limit("close", sign=+s, label="闭合极限")

        # ── Compute ──────────────────────────────────────────────────────
        travel = abs(close_rad - open_rad)
        if travel <= 0:
            raise RuntimeError(f"行程异常: close={close_rad:.4f} == open={open_rad:.4f}")

        rad_to_mm = self._config.max_stroke_mm / travel

        result = CalibrationData(
            zero_position=round(close_rad, 6),
            max_position=round(open_rad, 6),
            travel_range=round(travel, 6),
            rad_to_mm=round(rad_to_mm, 2),
            motor_type=self._motor_type.name,
            can_id=self._can_id,
            mst_id=self._mst_id or 0,
            calibration_time=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        )

        # Update config
        self._config.pos_closed_rad = result.zero_position
        self._config.pos_open_rad = result.max_position
        self._config.rad_to_mm = result.rad_to_mm
        self._config.calibrated = True

        print(f"\n{'=' * 60}")
        print(f"  标定完成")
        print(f"{'=' * 60}")
        print(f"  闭合(0mm):   {result.zero_position:.6f} rad")
        print(f"  张开({self._config.max_stroke_mm:.0f}mm): "
              f"{result.max_position:.6f} rad")
        print(f"  行程:        {result.travel_range:.6f} rad  "
              f"({result.travel_mm:.1f} mm)")
        print(f"  转换系数:    {result.rad_to_mm:.1f} mm/rad")
        print(f"{'=' * 60}")

        return result

    # ═══════════════════════════════════════════════════════════════════
    # Calibration persistence
    # ═══════════════════════════════════════════════════════════════════

    def save_calibration(self, path: Optional[str] = None) -> str:
        """Save current calibration and settings to a JSON file.

        Args:
            path: Destination file. When omitted, saves to this channel's own
                  path (:func:`default_calib_path`, i.e.
                  ``~/.litegrip/<channel>_calibration.json``) — the file
                  :meth:`load_calibration` reads by default, so a calibration
                  saved here is picked up automatically next run.  One file
                  per channel is what keeps two grippers on one machine from
                  overwriting each other.  The factory calibration shipped
                  with the SDK is never overwritten; back it up separately if
                  needed.

        Returns:
            The absolute path the calibration was written to.
        """
        import json
        if path is None:
            path = default_calib_path(self._channel)
        data = {
            "channel": self._channel,
            "can_id": self._can_id,
            # Only stamp mst_id when it is actually known.  Writing a falsy 0
            # would pin auto-detection: load_calibration would then install a
            # RX filter of 0x000, every reply from the motor would be dropped,
            # and enable() would fail after a long retry loop.  Omitting it
            # keeps the value "unknown", which connect() reads as auto-detect.
            "canfd_mode": self._canfd_mode or False,
            "calibrated": True,
            # Which of zero/max is numerically larger is what carries the
            # mounting direction; there is no separate field for it.
            "zero_position_rad": self._config.pos_closed_rad,
            "max_position_rad": self._config.pos_open_rad,
            "travel_range_rad": abs(self._config.pos_open_rad - self._config.pos_closed_rad),
            "rad_to_mm": self._config.rad_to_mm,
            "motor_type": self._motor_type.name,
            "kp": self._config.kp,
            "kd": self._config.kd,
            "grasp_torque_threshold": self._config.grasp_torque_threshold,
        }
        if self._mst_id:
            data["mst_id"] = self._mst_id
        parent = _os.path.dirname(path)
        if parent:
            _os.makedirs(parent, exist_ok=True)
        with open(path, "w") as f:
            json.dump(data, f, indent=2)
        log.info("Calibration saved to %s", path)
        return path

    def load_calibration(self, path: Optional[str] = None,
                         template: Optional[str] = None) -> bool:
        """Load calibration from a JSON file into :attr:`config`.

        Three ways to say which file, and they are mutually exclusive:

        * ``template="reverse"`` — a name from :data:`CALIB_TEMPLATES`
          (:func:`list_templates`).  Strict: if that template cannot be read
          this raises instead of falling back, because the fallback would be
          the factory file, and that file is a *normal* mount — answering a
          request for reverse with normal is the one failure the name exists
          to prevent.
        * ``path=...`` — an explicit file, which may be anywhere (including a
          template's path).  Falls back to the built-in factory calibration
          if it cannot be read.
        * neither — this channel's own file (:func:`default_calib_path`),
          then the legacy single-file location (:data:`DEFAULT_CALIB`), then
          the factory one.  A candidate whose ``channel`` names a different
          interface is *skipped*, so a ``can1`` unit with no calibration of
          its own fails loudly rather than silently adopting ``can0``'s.

        Call this after :meth:`connect` but before :meth:`enable`.

        The limits in the file decide the direction: whichever of the two is
        numerically larger is the closed side.  Loading a template is how a
        reverse-mounted gripper is declared, and the direction a file implies
        is logged.

        Args:
            path: JSON file path.
            template: Template name (``"normal"`` / ``"reverse"``).

        Returns:
            True if loaded successfully.

        Raises:
            CommandError: both *path* and *template* given, an unknown
                template name, or a readable-but-missing template file.
        """
        import json

        if path is not None and template is not None:
            raise CommandError(
                "load_calibration() 的 path 与 template 只能给一个：path 是显式"
                "文件，template 是 CALIB_TEMPLATES 里的名字。")

        if template is not None:
            sources = [_resolve_template(template)]
            strict = True
            skip_other_channels = False
        elif path is not None:
            sources = [path, _FACTORY_CALIB]
            strict = False
            skip_other_channels = False
        else:
            sources = _dedupe([default_calib_path(self._channel),
                               DEFAULT_CALIB, _FACTORY_CALIB])
            strict = False
            skip_other_channels = True

        data = None
        loaded_from: str = ""
        for src in sources:
            try:
                with open(src, "r") as f:
                    candidate = json.load(f)
            except (FileNotFoundError, json.JSONDecodeError):
                continue
            if skip_other_channels:
                file_channel = candidate.get("channel")
                if file_channel and self._channel and file_channel != self._channel:
                    log.info("跳过 %s：它声明的 channel=%s 不是本实例的 %s",
                             src, file_channel, self._channel)
                    continue
            data = candidate
            loaded_from = src
            break

        if data is None:
            if strict:
                raise CommandError(
                    f"标定模板 {template!r} 读不出来（{sources[0]}）。不作出厂"
                    f"回退 —— 出厂文件是正装，用它会静默把方向换成正装。")
            log.warning("No calibration found (tried: %s). "
                        "Run zero() first, or load a template.",
                        ", ".join(sources))
            return False

        log.info("Calibration loaded from %s", loaded_from)

        self._config.pos_closed_rad = float(data["zero_position_rad"])
        self._config.pos_open_rad = float(data["max_position_rad"])
        self._config.rad_to_mm = float(data["rad_to_mm"])

        # Optional fields (present in newer calibration files)
        for key, attr in [
            ("can_id", "can_id"),
            ("mst_id", "mst_id"),
            ("channel", "can_channel"),
            ("canfd_mode", "canfd_mode"),
            ("kp", "kp"),
            ("kd", "kd"),
            ("grasp_torque_threshold", "grasp_torque_threshold"),
        ]:
            if key in data:
                setattr(self._config, attr, data[key])

        # Older calibration files predate the flag and always came from a real
        # calibration run, so absence means calibrated.
        self._config.calibrated = bool(data.get("calibrated", True))

        # The channel is the only thing that tells two grippers apart when both
        # sit at CAN ID 0x08, so a file that names another channel is worth
        # flagging — but it is not fatal, since older files may omit the key.
        file_channel = data.get("channel")
        if file_channel and self._channel and file_channel != self._channel:
            log.warning("标定文件的 channel=%s 与本实例的 %s 不一致 —— "
                        "同一台电脑上多台夹爪共用 CAN ID 时，通道是唯一身份键，"
                        "确认没有指错文件。", file_channel, self._channel)

        # Also update instance-level IDs if present.  A falsy value counts as
        # "unknown", not as ID 0: files written before this SDK started omitting
        # an unset mst_id (and hand-edited ones) may carry ``"mst_id": 0``, which
        # would otherwise pin the CAN RX filter to 0x000 and drop every reply.
        if data.get("can_id"):
            self._can_id = int(data["can_id"])
        if data.get("mst_id"):
            self._mst_id = int(data["mst_id"])

        s = self._config.close_sign
        log.info("Calibration loaded from %s: range=%.1f mm，方向=%s",
                 loaded_from,
                 abs(self._config.pos_open_rad - self._config.pos_closed_rad)
                 * self._config.rad_to_mm,
                 "正向（rad 增大 = 闭合）" if s > 0 else "反装（rad 减小 = 闭合）")
        return True

    def load_template(self, name: str) -> bool:
        """Load a pre-made calibration template by name, declaring the mount.

        Names come from :func:`list_templates` (``"normal"`` / ``"reverse"``).
        Thin wrapper over ``load_calibration(template=name)`` so the name is
        resolved in one place; see that method for why a template never falls
        back to the factory file.

        Args:
            name: Template name.

        Returns:
            True once loaded.

        Raises:
            CommandError: unknown name, or the template file is unreadable.
        """
        return self.load_calibration(template=name)

    # ═══════════════════════════════════════════════════════════════════
    # Motion — high-level
    # ═══════════════════════════════════════════════════════════════════

    def home(self) -> bool:
        """Move to the closed (zero) position.

        Uses this instance's calibrated closed limit, so a reverse-mounted
        gripper homes to the correct end.
        """
        self._check_connected()
        self._check_enabled()
        return self.move_to(self._config.pos_closed_rad, duration=1.0)

    def open(
        self,
        speed_mm_s: Optional[float] = None,
        *,
        progress: Optional[Callable[[MoveProgress], None]] = None,
    ) -> MoveResult:
        """Open the gripper fully.

        A continuous ramp (velocity feed-forward, one frame per
        :attr:`MotionConfig.frame_interval`) that drives *past* the calibrated
        open limit and lets the mechanical stop end the move. The command lead
        is narrowed to :attr:`MotionConfig.stop_lead_mm` inside
        :attr:`MotionConfig.press_zone_mm` of the limit, so the pressing
        torque stays around ``kp × stop_lead_mm``.

        Args:
            speed_mm_s: Opening speed; ``None`` = ``MotionConfig.speed_mm_s``.
            progress: Optional callback, called with a
                :class:`~litegrip.actions.MoveProgress` per sample.

        Returns:
            :class:`~litegrip.actions.MoveResult` — truthy when it pressed
            onto the stop (``stalled`` and parked within
            :attr:`MotionConfig.stop_tol` of the limit).  Stalling far from
            the limit means something blocked the travel, and is falsy.
        """
        self._check_connected()
        return self._actions.open(speed_mm_s, progress=progress)

    def close(
        self,
        speed_mm_s: Optional[float] = None,
        *,
        progress: Optional[Callable[[MoveProgress], None]] = None,
    ) -> MoveResult:
        """Close the gripper.

        Same ramp as :meth:`open`, pressing onto the closed-side mechanical
        stop.  Use :meth:`grasp` for a power grasp (closing onto an object and
        squeezing) — that one stops on the object, not on the empty stop.

        Args:
            speed_mm_s: Closing speed; ``None`` = ``MotionConfig.speed_mm_s``.
            progress: Optional progress callback.

        Returns:
            :class:`~litegrip.actions.MoveResult`.
        """
        self._check_connected()
        return self._actions.close(speed_mm_s, progress=progress)

    def grasp(
        self,
        force_n: Optional[float] = None,
        hold_s: float = 0.0,
        *,
        progress: Optional[Callable[[MoveProgress], None]] = None,
    ) -> GraspResult:
        """Adaptive grasp — close until stall, then hold with a set force.

        Blocking.  Closes on a ramp (stall detection catches the object),
        then keeps streaming MIT frames holding the position with a
        feed-forward torque of ``force_n × 0.1`` Nm, re-reading the
        position every ``MotionConfig.hold_interval``.

        Args:
            force_n: Gripping force in N; ``None`` = ``MotionConfig.force_n``.
            hold_s: Hold time in seconds; ``0`` = hold until a fault or
                Ctrl+C.
            progress: Optional progress callback.

        Returns:
            :class:`~litegrip.actions.GraspResult` — truthy when the hold
            ended normally.  ``stalled=True`` with ``reached=False`` means
            it closed onto an object, which is the expected outcome.
        """
        self._check_connected()
        return self._actions.grasp(force_n, hold_s, progress=progress)

    def zero(self) -> CalibrationData:
        """Full calibration: probe both mechanical limits and save.

        The gripper is driven against each end stop with low stiffness.
        Make sure the travel is clear.  Writes the result to the default
        user calibration path (see :meth:`save_calibration`).

        Returns:
            :class:`CalibrationData`.
        """
        self._check_connected()
        self._check_enabled()
        return self._actions.zero()

    # ═══════════════════════════════════════════════════════════════════
    # Motion — mid-level
    # ═══════════════════════════════════════════════════════════════════

    def goto(
        self,
        position_mm: float,
        kp: Optional[float] = None,
        kd: Optional[float] = None,
        duration: float = 0.5,
    ) -> bool:
        """Move to an absolute position in millimetres."""
        self._check_connected()
        self._check_enabled()
        # Opening may increase or decrease the motor rad count depending on the
        # mount; close_sign carries that, so this works for both.
        position_rad = (self._config.pos_closed_rad
                        - self._config.close_sign * position_mm
                        / self._config.rad_to_mm)
        return self.goto_rad(position_rad, kp=kp, kd=kd, duration=duration)

    def goto_rad(
        self,
        position_rad: float,
        kp: Optional[float] = None,
        kd: Optional[float] = None,
        dq_target: float = 0.0,
        tau_feedforward: float = 0.0,
        duration: float = 0.5,
    ) -> bool:
        """Move to an absolute position in radians (streams MIT frames)."""
        self._check_connected()
        self._check_enabled()
        if self._can is None:
            return False

        kp = kp if kp is not None else self._config.kp
        kd = kd if kd is not None else self._config.kd

        # Clamp between the two calibrated limits, whichever is numerically
        # larger (the ordering flips on a reverse mount).
        lo = min(self._config.pos_closed_rad, self._config.pos_open_rad)
        hi = max(self._config.pos_closed_rad, self._config.pos_open_rad)
        position_rad = max(lo, min(hi, position_rad))

        try:
            return self._can.control_mit_stream(
                q_target=position_rad,
                kp=kp, kd=kd,
                duration_s=duration,
                dq_target=dq_target,
                tau_feedforward=tau_feedforward,
            )
        except Exception as e:
            raise CommError(f"位置控制失败: {e}")

    def move_to(
        self,
        target_rad: float,
        kp: Optional[float] = None,
        kd: Optional[float] = None,
        tau_feedforward: float = 0.0,
        duration: float = 1.0,
    ) -> bool:
        """Sustained move to a target position (longer default duration)."""
        return self.goto_rad(target_rad, kp=kp, kd=kd,
                             tau_feedforward=tau_feedforward,
                             duration=duration)

    def set_force(self, force_n: float, duration: float = 0.3) -> bool:
        """Apply a gripping force at the current position.

        Sends MIT frames with feed-forward torque while holding position.

        Args:
            force_n: Target force in newtons.
            duration: Hold time in seconds.
        """
        self._check_connected()
        self._check_enabled()
        if self._can is None:
            return False

        # Squeezing may mean increasing or decreasing rad depending on the
        # mount; close_sign carries that.
        tau_nm = self._config.close_sign * force_n * UnitConversion.N_TO_NM
        current_pos = self._can.get_position()

        try:
            return self._can.control_mit_stream(
                q_target=current_pos,
                kp=150.0, kd=2.0,
                duration_s=duration,
                tau_feedforward=tau_nm,
            )
        except Exception as e:
            raise CommError(f"力控失败: {e}")

    # ═══════════════════════════════════════════════════════════════════
    # Velocity-controlled moves
    # ═══════════════════════════════════════════════════════════════════

    def move_at_speed(
        self,
        target_mm: float,
        speed_mm_s: float = 30.0,
        kp: Optional[float] = None,
        kd: Optional[float] = None,
    ) -> bool:
        """Move to a target position at a constant linear speed (mm/s).

        Uses MIT mode with velocity feedforward.  The trajectory is a
        linear ramp from current position to *target_mm*.

        Args:
            target_mm: Target position in mm (0=closed, 120=open).
            speed_mm_s: Travel speed in mm/s (default 30).
            kp: Position stiffness (default from config).
            kd: Velocity damping.

        Returns:
            True on success.
        """
        self._check_connected()
        self._check_enabled()

        current_mm = self.get_state().position_mm
        distance_mm = abs(target_mm - current_mm)
        if distance_mm < 0.01 or speed_mm_s <= 0:
            return True

        duration_s = distance_mm / speed_mm_s
        # Convert to rad (close_sign handles the reverse mount)
        s = self._config.close_sign
        current_rad = (self._config.pos_closed_rad
                       - s * current_mm / self._config.rad_to_mm)
        target_rad = (self._config.pos_closed_rad
                      - s * target_mm / self._config.rad_to_mm)
        speed_rad_s = speed_mm_s / self._config.rad_to_mm

        return self._move_at_speed_rad(
            current_rad, target_rad, speed_rad_s, duration_s, kp, kd)

    def move_at_speed_rad(
        self,
        target_rad: float,
        speed_rad_s: float = 0.5,
        kp: Optional[float] = None,
        kd: Optional[float] = None,
    ) -> bool:
        """Move to a target position at a constant motor speed (rad/s).

        Args:
            target_rad: Target position in rad.
            speed_rad_s: Motor speed in rad/s (default 0.5).
            kp: Position stiffness (default from config).
            kd: Velocity damping.

        Returns:
            True on success.
        """
        self._check_connected()
        self._check_enabled()

        current_rad = self.get_position_rad()
        distance_rad = abs(target_rad - current_rad)
        if distance_rad < 0.0001 or speed_rad_s <= 0:
            return True

        duration_s = distance_rad / speed_rad_s
        return self._move_at_speed_rad(
            current_rad, target_rad, speed_rad_s, duration_s, kp, kd)

    def _move_at_speed_rad(
        self,
        start_rad: float,
        target_rad: float,
        speed_rad_s: float,
        duration_s: float,
        kp: Optional[float],
        kd: Optional[float],
    ) -> bool:
        """Core: linear ramp from start to target at constant speed."""
        if self._can is None:
            return False

        kp = kp if kp is not None else self._config.kp
        kd = kd if kd is not None else self._config.kd

        # Clamp target
        lo = min(self._config.pos_closed_rad, self._config.pos_open_rad)
        hi = max(self._config.pos_closed_rad, self._config.pos_open_rad)
        target_rad = max(lo, min(hi, target_rad))

        direction = 1.0 if target_rad > start_rad else -1.0

        interval = 0.005  # 200 Hz
        steps = max(1, int(duration_s / interval))
        actual_interval = duration_s / steps

        log.info("move_at_speed: %.4f → %.4f rad @ %.2f rad/s (%.2f s, %d steps)",
                 start_rad, target_rad, speed_rad_s, duration_s, steps)

        for i in range(steps + 1):
            frac = i / steps
            q = start_rad + (target_rad - start_rad) * frac
            dq = direction * speed_rad_s if i < steps else 0.0

            self._can.control_mit(
                q_target=q, kp=kp, kd=kd,
                dq_target=dq, tau_feedforward=0.0)

            if i < steps:
                self._can.poll(timeout_s=0.0)
                time.sleep(actual_interval)

        # Hold at target briefly
        for _ in range(20):
            self._can.control_mit(
                q_target=target_rad, kp=kp, kd=kd,
                dq_target=0.0, tau_feedforward=0.0)
            time.sleep(0.005)

        return True

    # ═══════════════════════════════════════════════════════════════════
    # Calibration
    # ═══════════════════════════════════════════════════════════════════

    def calibrate(
        self,
        kp: float = 20.0,
        kd: float = 2.0,
        step_rad: float = 0.05,
        stall_delta: float = 0.0015,
        stall_cycles: int = 5,
        max_iter: int = 200,
        tau_limit: Optional[float] = 2.0,
    ) -> CalibrationData:
        """Calibrate the gripper: find close and open mechanical limits.

        Process:
        1. Back off a small amount from current position
        2. Step toward close direction until stall → zero_position
        3. Back off
        4. Step toward open direction until stall → max_position
        5. Compute travel_range and update internal conversion factors

        A stall only says "something stopped me", never *which* end was hit —
        both ends are hard stops.  So the direction is not discovered here: it
        comes from :attr:`GripperConfig.close_sign` (load a calibration
        template first for a reverse mount), and this routine preserves it.

        Safety: at a hard stop the encoder keeps creeping (backlash, elastic
        deformation, micro-slip), so the position-based stall test alone can
        never fire — the command would keep advancing and ``kp × error`` would
        keep growing until the structure breaks.  Two independent guards are
        therefore in place: the command lead is re-derived from the *measured*
        position every cycle (bounded to ``step_rad``, so pressing torque is at
        most ``kp × step_rad``), and the probe aborts the moment ``|tau|``
        reaches ``tau_limit``.

        Args:
            kp: Probing stiffness (low = gentle).
            kd: Probing damping.
            step_rad: Step size per iteration (rad); also the command lead cap.
            stall_delta: Position change threshold for stall detection (rad).
            stall_cycles: Consecutive stalls to confirm limit.
            max_iter: Max steps per direction (safety cap).
            tau_limit: Torque ceiling in Nm.  The probe stops as soon as
                ``|tau|`` reaches this; ``None`` disables the ceiling.

        Returns:
            CalibrationData with zero/max position, travel range, and
            rad-to-mm conversion.
        """
        self._check_connected()
        self._check_enabled()
        if self._can is None:
            raise NotInitializedError("未连接")

        s = self._config.close_sign
        init_pos = self._can.get_position()
        self._can.update_state(timeout_s=0.1)
        init_pos = self._can.get_position()
        print(f"标定开始  初始位置: {init_pos:.4f} rad  "
              f"（方向：{'正向' if s > 0 else '反装'}）")

        def _find_limit(direction: str) -> float:
            sign = s if direction == "close" else -s
            label = "闭合限位" if direction == "close" else "张开限位"
            print(f"  寻找{label}...")

            self._can.update_state(timeout_s=0.05)
            current = self._can.get_position()
            stall = 0

            for i in range(max_iter):
                # Re-derive the command from the *measured* position each cycle.
                # Accumulating a running target (``target += sign * step_rad``)
                # lets the command lead grow without bound once the stop is
                # reached, and the pressing torque (kp × lead) grows with it.
                # Bounding the lead to one step caps it at kp × step_rad.
                target = current + sign * step_rad
                self._can.control_mit_stream(target, kp, kd, duration_s=0.3, interval_s=0.005)
                self._can.update_state(timeout_s=0.1)

                new_pos = self._can.get_position()
                delta = abs(new_pos - current)
                tau = self._can.get_torque()

                print(f"    [{i}] tgt={target:+.3f} pos={new_pos:.4f} "
                      f"d={delta:.5f} tau={tau:+.3f} st={stall}")

                if tau_limit is not None and abs(tau) >= tau_limit:
                    print(f"    → 力矩达到上限 {tau_limit:.2f} Nm（{tau:+.3f}），"
                          f"停止推进并保持: {new_pos:.6f} rad")
                    return new_pos

                if delta < stall_delta:
                    stall += 1
                    if stall >= stall_cycles:
                        print(f"    → 到达{label}: {new_pos:.6f} rad")
                        return new_pos
                else:
                    stall = 0
                current = new_pos

            print(f"    → 安全停止（达到最大步数 {max_iter}）: {current:.4f} rad")
            return current

        def _bounded_move(target: float, label: str) -> None:
            """Move toward *target*, guarded the same way as the probe.

            The plain ``goto_rad`` streams one constant command for half a
            second with no ceiling.  If the jaws already sit at the stop that
            command is heading for, the pressing torque is ``kp × 0.2`` (16 Nm at
            ``kp=80``) held for the whole duration — the same kind of unguarded
            push that broke a jaw.  Leading by one step at a time and watching
            ``|tau|`` bounds it to the probe's ceiling instead.
            """
            current = self._can.get_position()
            sign = 1.0 if target >= current else -1.0
            for _ in range(max_iter):        # same cap as the probe
                if abs(target - current) <= step_rad:
                    return
                self._can.control_mit_stream(
                    current + sign * step_rad, kp, kd,
                    duration_s=0.1, interval_s=0.005)
                self._can.update_state(timeout_s=0.1)
                new_pos = self._can.get_position()
                tau = self._can.get_torque()
                if tau_limit is not None and abs(tau) >= tau_limit:
                    print(f"    → {label}：力矩达到上限 {tau_limit:.2f} Nm"
                          f"（{tau:+.3f}），停止")
                    return
                if abs(new_pos - current) < stall_delta:
                    print(f"    → {label}：位置不再变化，停止")
                    return
                current = new_pos

        # 1. Safe back-off (a nudge toward the close side, as before, flipped
        #    with the mount)
        print("  安全回退...")
        _bounded_move(init_pos + s * 0.2, "安全回退")

        # 2. Find zero (close direction)
        zero_pos = _find_limit("close")

        # 3. Back off, away from the close stop toward the open side
        print("  回退...")
        _bounded_move(zero_pos - s * 0.3, "回退")

        # 4. Find max (open direction)
        max_pos = _find_limit("open")

        # 5. Compute results
        # zero_pos = close limit, max_pos = open limit; which is numerically
        # larger depends on the mount, so take the magnitude.
        travel = abs(zero_pos - max_pos)
        rad_to_mm = self._config.max_stroke_mm / travel if travel > 0 else 105.26

        result = CalibrationData(
            zero_position=round(zero_pos, 6),      # closed → 0 mm
            max_position=round(max_pos, 6),        # open → max mm
            travel_range=round(travel, 6),
            rad_to_mm=round(rad_to_mm, 2),
            motor_type=self._motor_type.name,
            can_id=self._can_id,
            mst_id=self._mst_id or 0,
            calibration_time=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        )

        # Update config
        self._config.pos_closed_rad = result.zero_position
        self._config.pos_open_rad = result.max_position
        self._config.rad_to_mm = result.rad_to_mm
        self._config.calibrated = True

        print(f"\n  标定结果:")
        print(f"    闭合(0mm): {result.zero_position:.6f} rad")
        print(f"    张开(120mm): {result.max_position:.6f} rad")
        print(f"    行程:     {result.travel_range:.6f} rad  "
              f"({result.travel_mm:.1f} mm)")
        print(f"    转换系数: {result.rad_to_mm:.1f} mm/rad")

        return result

    # ═══════════════════════════════════════════════════════════════════
    # State
    # ═══════════════════════════════════════════════════════════════════

    def get_state(self, wait: bool = True) -> GripperState:
        """Return the current gripper state.

        Args:
            wait: If True (default), waits up to 50 ms for a fresh status
                  frame.  If False, returns immediately with the last cached
                  state (suitable for high-frequency control loops).

        Returns:
            GripperState snapshot.
        """
        self._check_connected()
        if self._can is None:
            return GripperState()

        with self._io_lock:
            if wait:
                self._can.update_state(timeout_s=0.05)
            else:
                self._can.poll(timeout_s=0.0)

            position_rad = self._can.get_position()
            velocity_rad_s = self._can.get_velocity()
            torque_nm = self._can.get_torque()
            error_code = self._can.get_error()
            t_mos, t_coil = self._can.get_temperature()

            # pos_closed_rad = closed (0 mm), pos_open_rad = open (max mm).
            # Which way the rad count runs depends on the mount, so close_sign
            # sets the sign; the result is 0 mm at the closed limit and +stroke
            # at the open limit for both mountings.
            s = self._config.close_sign
            position_mm = ((self._config.pos_closed_rad - position_rad)
                           * s * self._config.rad_to_mm)
            # Squeeze is positive force: the sign flips with the mount too.
            force_n = s * torque_nm * UnitConversion.NM_TO_N

            return GripperState(
                position_rad=position_rad,
                velocity_rad_s=velocity_rad_s,
                torque_nm=torque_nm,
                temperature_mos=t_mos,
                temperature_coil=t_coil,
                error_code=error_code,
                timestamp=time.time(),
                position_mm=position_mm,
                force_n=force_n,
            )

    def get_position(self) -> float:
        """Current position in mm."""
        return self.get_state().position_mm

    def get_position_rad(self) -> float:
        """Current position in rad."""
        self._check_connected()
        if self._can is None:
            return 0.0
        self._can.update_state(timeout_s=0.05)
        return self._can.get_position()

    def get_force(self) -> float:
        """Estimated gripping force in N."""
        return self.get_state().force_n

    def get_torque(self) -> float:
        """Current motor torque in Nm."""
        self._check_connected()
        if self._can is None:
            return 0.0
        self._can.update_state(timeout_s=0.05)
        return self._can.get_torque()

    def get_error(self) -> int:
        """Motor error code (0=disabled, 1=enabled, 0x9=UV, ...)."""
        self._check_connected()
        if self._can is None:
            return -1
        self._can.update_state(timeout_s=0.05)
        return self._can.get_error()

    def get_temperature(self) -> tuple[int, int]:
        """Return (MOS_temp, Coil_temp) in °C."""
        self._check_connected()
        if self._can is None:
            return 0, 0
        self._can.update_state(timeout_s=0.05)
        return self._can.get_temperature()

    def get_info(self) -> GripperInfo:
        """Return device metadata."""
        return GripperInfo(
            motor_type=self._motor_type.name,
            can_id=self._can_id,
            mst_id=self._mst_id or 0,
        )

    # ═══════════════════════════════════════════════════════════════════
    # State predicates
    # ═══════════════════════════════════════════════════════════════════

    def is_moving(self) -> bool:
        """True if the gripper is currently in motion."""
        state = self.get_state()
        return state.is_moving

    def is_grasped(self) -> bool:
        """True if torque exceeds the grasp-detection threshold."""
        state = self.get_state()
        return abs(state.torque_nm) > self._config.grasp_torque_threshold

    def wait_for_ready(self, timeout: float = 5.0) -> bool:
        """Block until the motor is enabled and not moving."""
        if self._can is None:
            return False
        start = time.time()
        while time.time() - start < timeout:
            self._can.update_state(timeout_s=0.05)
            err = self._can.get_error()
            if err == 1 and not self.is_moving():
                return True
            time.sleep(0.05)
        return False

    # ═══════════════════════════════════════════════════════════════════
    # Parameter access (expert)
    # ═══════════════════════════════════════════════════════════════════

    def read_param(self, rid: int, timeout_s: float = 0.5) -> float:
        """Read a motor register by its RID (expert use).

        See :class:`litegrip.can.protocol.DM_REG` for available registers.
        """
        self._check_connected()
        if self._can is None:
            raise NotInitializedError("未连接")
        return self._can.read_param(rid, timeout_s=timeout_s)

    # ═══════════════════════════════════════════════════════════════════
    # Teleoperation (leader / follower)
    # ═══════════════════════════════════════════════════════════════════

    def teleop_start(
        self,
        mode: str,
        *,
        transport: Optional["TeleopTransport"] = None,
        link: str = "zenoh",
        host: Optional[str] = None,
        port: int = DEFAULT_GRIP_PORT,
        grip_id: str = DEFAULT_GRIP_ID,
        kp: Optional[float] = None,
        kd: Optional[float] = None,
        align: bool = True,
        watchdog_s: float = 0.2,
        dq_max: float = DEFAULT_DQ_MAX,
        torque_limit_nm: float = DEFAULT_TORQUE_LIMIT_NM,
        rate_hz: float = 50.0,
    ) -> dict:
        """Start leader/follower teleoperation on this gripper.

        ``mode="master"`` (leader) makes the motor slack — the jaws can be
        pushed by hand — and publishes the opening.  ``mode="slave"``
        (follower) receives the opening and follows it.

        Both ends must agree on ``grip_id``.  Teleoperation is exclusive: the
        background loop owns the CAN I/O until :meth:`teleop_stop`, so do not
        drive this gripper from the caller while it runs.

        Args:
            mode: ``"master"`` or ``"slave"``.
            transport: A :class:`~litegrip.TeleopTransport`.  When omitted, one
                is built from ``link``.  An injected transport is never closed
                by this class.
            link: ``"zenoh"`` (default) builds the point-to-point zenoh link
                used by the field teleoperation — see
                :mod:`litegrip.zenoh_link`, needs ``pip install litegrip[zenoh]``.
                ``"udp"`` builds the plain-UDP transport, for a trusted LAN.
            host: The peer's address.  Required for a ``slave``; the master only
                listens, so it needs no host.
            port: TCP (zenoh) or UDP port.
            grip_id: Topic id shared by both ends.
            kp, kd: Follow gains (slave).  ``None`` uses the calibration's own.
            align: Slave only — align to the first received frame before
                following.
            watchdog_s: Slave only — hold position after this long without a
                fresh frame.
            dq_max: Slave only — ceiling in rad/s on the leader velocity fed
                forward to the follower.  ``0`` disables the feedforward.
            torque_limit_nm: Slave only — ceiling in Nm on the follower's own
                torque; held over it the follower releases in place.  ``0``
                disables the guard.  See :class:`~litegrip.GripperTeleop`.
            rate_hz: Loop rate.

        Returns:
            The initial :meth:`teleop_status` snapshot.

        Raises:
            TeleopNotReady: uncalibrated, zero travel, or ``rad_to_mm == 0``.
            TeleopBusyError: teleoperation is already running.
            NotInitializedError: not connected or not enabled.
            ValueError: ``torque_limit_nm`` is negative.
        """
        from .teleop import (GripperTeleop, TeleopBusyError, check_ready,
                             teleop_topic)
        from .teleop import UdpTeleopTransport

        self._check_connected()
        self._check_enabled()
        if mode not in ("master", "slave"):
            raise ValueError(f"mode must be 'master' or 'slave', got {mode!r}")
        # Before anything is enabled or driven: ``send_mit_frame`` and
        # ``goto_rad`` do not check ``calibrated`` themselves.
        check_ready(self.config)
        if self._teleop is not None and self._teleop.is_running:
            raise TeleopBusyError("teleop is already running")
        self._claim_session("teleop", TeleopBusyError)

        # Everything from here to the running loop is inside one try: the
        # session was claimed above, and a transport that refuses (a bad link,
        # a missing host, a port already bound) would otherwise leave the slot
        # claimed for the life of the process, refusing every later teleop,
        # record and replay in turn.
        try:
            key = teleop_topic(grip_id)
            created_transport = None
            if transport is None:
                if link == "zenoh":
                    if mode == "master":
                        # ⚠ 常驻 publisher **不登记**成 per-session transport：登记了
                        #    `teleop_stop` 就会 close 它，而 `_teleop_pub` 仍指向这个
                        #    已死的端点 ⇒ 此后每一轮都在往死会话里 put。（真机实测：
                        #    第一次配对正常，之后每次从端 0 帧。）常驻端点的生命周期
                        #    只归 `disconnect()` → `_close_teleop_pub()`。
                        transport = self._open_teleop_pub(port, key)
                    else:
                        transport = _zenoh_transport("slave", key, port, host)
                        created_transport = transport
                elif link == "udp":
                    if host is None:
                        raise ValueError("host is required for the udp link")
                    addr = f"{host}:{port}"
                    if mode == "master":
                        transport = UdpTeleopTransport(pub_addr=addr)
                    else:
                        transport = UdpTeleopTransport(bind_addr=addr)
                    created_transport = transport
                else:
                    raise ValueError(
                        f"link must be 'zenoh' or 'udp', got {link!r}")

            manager = GripperTeleop(
                self, transport, mode, key,
                rate_hz=rate_hz, kp=kp, kd=kd, align=align,
                watchdog_s=watchdog_s, dq_max=dq_max,
                torque_limit_nm=torque_limit_nm)
            manager.start()
        except BaseException:
            self._release_session("teleop")
            raise
        self._teleop = manager
        self._teleop_transport = created_transport
        return manager.status()

    def _open_teleop_pub(self, port: int, key: str) -> "TeleopTransport":
        """Return the leader's resident publisher, building it on first use.

        ⚠ **Resident, not per session.**  Rebuilding the zenoh listener on every
        session leaves the port bound and makes publisher↔subscriber matching
        fail intermittently; keeping one for the life of the gripper removes
        both.
        """
        if self._teleop_pub is None:
            self._teleop_pub = _zenoh_transport("master", key, port, None)
        return self._teleop_pub

    def _close_teleop_pub(self) -> None:
        pub, self._teleop_pub = self._teleop_pub, None
        if pub is not None:
            try:
                pub.close()
            except Exception as e:  # noqa: BLE001
                log.debug("teleop publisher close failed: %s", e)

    def teleop_stop(self, timeout: float = 2.0) -> dict:
        """Stop teleoperation and leave the gripper holding its position.

        Neither side disables: the master leaves zero-gravity mode and the slave
        sends one final frame at its current angle, so the jaws hold under gain.
        The leader's resident publisher is **not** closed here — only the
        per-session subscriber is.
        """
        manager = self._teleop
        if manager is None:
            return {"active": False, "mode": None}
        manager.stop(timeout=timeout)
        self._teleop = None
        self._release_session("teleop")
        if self._teleop_transport is not None:
            try:
                self._teleop_transport.close()
            except Exception as e:  # noqa: BLE001
                log.debug("teleop transport close failed: %s", e)
            self._teleop_transport = None
        return manager.status()

    def teleop_status(self) -> dict:
        """Snapshot of the running teleoperation, or ``{"active": False}``."""
        if self._teleop is None:
            return {"active": False, "mode": None}
        return self._teleop.status()

    # ═══════════════════════════════════════════════════════════════════
    # Trajectory record and replay
    # ═══════════════════════════════════════════════════════════════════
    #
    # Teach a motion once, repeat it later.  See litegrip.trajectory for what
    # the recording stores and, importantly, what a replay does *not*
    # reproduce (force).

    def record_start(
        self,
        rate_hz: float = 100.0,
        zero_gravity: bool = True,
        max_samples: Optional[int] = None,
    ) -> dict:
        """Begin recording this gripper's motion, in the background.

        With ``zero_gravity=True`` — the hand-teaching mode — the recorder
        itself streams zero-torque frames so the jaws can be pushed by hand.
        Do not drive the gripper from the caller while that runs.  With
        ``zero_gravity=False`` the recorder only *reads* state, so the caller is
        free to drive the gripper from another thread and capture a
        programmatic move; only the sampling touches the bus.

        Recording is exclusive with teleoperation and replay.

        Args:
            rate_hz: Samples per second (default 100).
            zero_gravity: Stream zero-torque frames, leaving the jaws
                back-drivable by hand.
            max_samples: Stop by itself after this many samples; ``None``
                records until :meth:`record_stop`.

        Returns:
            The initial :meth:`trajectory_status` snapshot.

        Raises:
            TrajectoryBusyError: another session (teleop, record, play) is running.
            NotInitializedError: not connected or not enabled.
            TrajectoryError: the unit is not calibrated, so the normalised
                opening a sample stores would be a guess.
        """
        from .trajectory import (TrajectoryBusyError, TrajectoryError,
                                 TrajectoryRecorder)

        self._check_connected()
        self._check_enabled()
        if not self._config.calibrated:
            raise TrajectoryError(
                "未标定 —— 轨迹记录的是按行程归一化的张开度, 没有标定就算不出来; "
                "先 load_calibration() 或 zero()")
        self._claim_session("record", TrajectoryBusyError)
        try:
            recorder = TrajectoryRecorder(
                self, rate_hz=rate_hz, zero_gravity=zero_gravity,
                max_samples=max_samples)
            recorder.start()
        except BaseException:
            self._release_session("record")
            raise
        self._trajectory_recorder = recorder
        return recorder.status()

    def record_stop(self, allow_empty: bool = False) -> "Trajectory":
        """Stop recording and return the captured trajectory.

        Args:
            allow_empty: Return an empty trajectory instead of raising when
                nothing was captured.  For a deliberate start-then-immediately-
                stop; a capture that was *meant* to contain motion should be
                allowed to raise.

        Returns:
            The recorded :class:`~litegrip.Trajectory`.

        Raises:
            TrajectoryNotActiveError: nothing is being recorded.
            TrajectoryRecordingError: the sampling loop died — a partial
                capture is never returned as if it were whole.
            TrajectoryEmptyError: no samples were captured.
        """
        from .trajectory import TrajectoryNotActiveError

        recorder = self._trajectory_recorder
        if recorder is None:
            raise TrajectoryNotActiveError("没有正在进行的录制")
        recorder.stop()
        self._trajectory_recorder = None
        self._release_session("record")
        return recorder.result(allow_empty=allow_empty)

    def record(
        self,
        duration_s: float,
        rate_hz: float = 100.0,
        zero_gravity: bool = True,
    ) -> "Trajectory":
        """Hand-teach a motion: record for *duration_s* seconds, return it.

        Blocking.  Zero-gravity is on, so the jaws go slack and you push them
        through the motion by hand while the samples are taken.  The call
        returns once the capture is complete; if it did not fill, it raises and
        says how many samples it got rather than returning a short recording.

        Args:
            duration_s: How many seconds to record.  The gripper is slack for
                that long — keep a hand on it, and be aware that the jaws hold
                nothing while it is slack.
            rate_hz: Samples per second (default 100).
            zero_gravity: ``False`` only if something else drives the gripper
                during the recording — see :meth:`record_start`.

        Raises:
            ValueError: ``duration_s <= 0``.
            TrajectoryBusyError: another session is running.
            NotInitializedError: not connected or not enabled.
            TrajectoryError: the unit is not calibrated.
            TrajectoryRecordingError: the capture did not fill.

        Example::

            with LiteGrip("can0") as gripper:
                gripper.load_calibration()
                gripper.enable()
                taught = gripper.record(5.0)      # push the jaws by hand
                taught.save("pick")               # ~/.litegrip/trajectories/pick.lgt
                gripper.play(taught)              # repeat it
        """
        duration_s = float(duration_s)
        if duration_s <= 0.0:
            raise ValueError(f"duration_s 需 > 0 (给的是 {duration_s})")
        target = max(1, int(round(duration_s * float(rate_hz))))

        self.record_start(rate_hz=rate_hz, zero_gravity=zero_gravity,
                          max_samples=target)
        recorder = self._trajectory_recorder
        try:
            recorder.wait_for(target, timeout=duration_s * 1.5 + 3.0)
        except BaseException:
            # Never leave the jaws slack and the session claimed because the
            # capture went wrong — clean up, then let the error through.
            recorder.stop()
            self._trajectory_recorder = None
            self._release_session("record")
            raise
        return self.record_stop()

    def play_start(
        self,
        trajectory: "Trajectory",
        speed: float = 1.0,
        kp: Optional[float] = None,
        kd: Optional[float] = None,
        loop: bool = False,
        align: bool = True,
    ) -> dict:
        """Replay a trajectory in the background.

        Only position is replayed: the recorded torque and velocity are
        diagnostics, never feed-forward, so a motion recorded while gripping an
        object replays as a position path and *not* as the same gripping force.
        Follow it with :meth:`grasp` if the force matters.

        Args:
            trajectory: A :class:`~litegrip.Trajectory`, from :meth:`record_stop`
                or :meth:`~litegrip.Trajectory.load`.
            speed: Timing multiplier; ``0.5`` plays at half speed.
            kp, kd: Gains for the replay frames; ``None`` uses the configured ones.
            loop: Restart at the end instead of stopping.  A one-sample
                trajectory is a pose, so looping it holds that opening.
            align: Move to the trajectory's first opening before following, so
                the first frame is not a step from wherever the jaws are.

        Returns:
            The initial :meth:`trajectory_status` snapshot.

        Raises:
            TrajectoryBusyError: another session is running.
            TrajectoryEmptyError: the trajectory has no samples.
            ValueError: ``speed <= 0``.
            NotInitializedError: not connected or not enabled.
            TrajectoryError: the unit is not calibrated — the opening has to be
                converted back through *this* gripper's travel, and with the
                placeholder limits that conversion is a guess.
        """
        from .trajectory import (TrajectoryBusyError, TrajectoryError,
                                 TrajectoryPlayer)

        self._check_connected()
        self._check_enabled()
        if not self._config.calibrated:
            raise TrajectoryError(
                "未标定 —— 回放要把张开度按本机行程换算回角度, 没有标定就是瞎走; "
                "先 load_calibration() 或 zero()")
        self._claim_session("play", TrajectoryBusyError)
        try:
            player = TrajectoryPlayer(
                self, trajectory, speed=speed, kp=kp, kd=kd, loop=loop,
                align=align)
            player.start()
        except BaseException:
            self._release_session("play")
            raise
        self._trajectory_player = player
        return player.status()

    def play(
        self,
        trajectory: "Trajectory",
        speed: float = 1.0,
        kp: Optional[float] = None,
        kd: Optional[float] = None,
        loop: bool = False,
        align: bool = True,
    ) -> dict:
        """Replay a trajectory once, blocking until it finishes.

        The last frame holds the final position under the configured gains, but
        it is *one* frame: the motor self-locks a communication-loss fault about
        100 ms after the frames stop.  Call the next action promptly, or use
        :meth:`play_start` with ``loop=True`` for a hold that lasts until
        :meth:`play_stop`.

        Args:
            loop: Must be ``False``.  A blocking replay of a looping trajectory
                never returns; use :meth:`play_start` for that.

        Returns:
            The final :meth:`trajectory_status` snapshot.

        Raises:
            ValueError: ``loop`` is true, or ``speed <= 0``.
            TrajectoryBusyError: another session is running.
            TrajectoryEmptyError: the trajectory has no samples.
            TrajectoryError: the unit is not calibrated, or the replay stopped
                early — a send failed, or the sampling clock stalled.
            NotInitializedError: not connected or not enabled.
        """
        from .trajectory import TrajectoryError

        if loop:
            raise ValueError(
                "loop=True 的阻塞回放永远不会返回; 要循环播放用 "
                "play_start(loop=True), 再用 play_stop() 停")
        self.play_start(trajectory, speed=speed, kp=kp, kd=kd, loop=False,
                        align=align)
        player = self._trajectory_player
        # Wall-clock pacing plus one align move; the margin covers a slow first
        # frame.  A stall guard of its own, so a stopped clock cannot hang here.
        budget = abs(float(trajectory.duration)) / float(speed) * 1.5 + 4.0
        try:
            finished = player.wait(budget)
        except BaseException:
            # Ctrl+C during a long replay is the ordinary way out of this call.
            # Without the stop the player keeps commanding the motor and the
            # session stays claimed, so every later record/play is refused as
            # busy until the process ends.
            self.play_stop()
            raise
        status = self.play_stop()
        if not finished:
            raise TrajectoryError(
                f"回放未在 {budget:.1f}s 内结束 (已发 {status.get('frames', 0)} 帧) "
                f"—— 采样时钟可能停住了")
        if status.get("error") is not None:
            raise TrajectoryError(f"回放中止: {status['error']}")
        return status

    def play_stop(self, timeout: float = 2.0) -> dict:
        """Stop a replay and leave the gripper holding its last target."""
        player = self._trajectory_player
        if player is None:
            return {"active": False, "kind": None}
        player.stop(timeout=timeout)
        self._trajectory_player = None
        self._release_session("play")
        return player.status()

    def trajectory_status(self) -> dict:
        """Snapshot of the running recording or replay, or ``{"active": False}``.

        One call for both directions: the ``kind`` key says which
        (``"record"`` / ``"play"``), and only one of them can be running.
        """
        if self._trajectory_recorder is not None:
            return self._trajectory_recorder.status()
        if self._trajectory_player is not None:
            return self._trajectory_player.status()
        return {"active": False, "kind": None}

    # ═══════════════════════════════════════════════════════════════════
    # Internal
    # ═══════════════════════════════════════════════════════════════════

    def _check_connected(self) -> None:
        if not self._connected:
            raise NotInitializedError("未连接 — 请先调用 connect() 或使用 with 上下文")

    def _check_enabled(self) -> None:
        if not self._enabled:
            raise NotInitializedError("未使能 — 请先调用 enable()")

    def _claim_session(self, owner: str, error: type) -> None:
        """Take the single long-running session slot, or raise *error*.

        Teleoperation, trajectory recording and trajectory replay all own the
        CAN I/O for their duration.  Claiming happens under one lock, so two
        threads starting different sessions at the same instant cannot both
        pass the check and then both start.
        """
        with self._session_lock:
            if self._session_owner is not None:
                raise error(
                    f"已有会话在运行 ({self._session_owner}) —— "
                    f"teleop、录制、回放同一时刻只能有一个")
            self._session_owner = owner

    def _release_session(self, owner: str) -> None:
        """Give the session slot back.  Harmless if *owner* no longer holds it."""
        with self._session_lock:
            if self._session_owner == owner:
                self._session_owner = None

    @property
    def session(self) -> Optional[str]:
        """Which long-running session is active: ``None``, ``"teleop"``,
        ``"record"`` or ``"play"``.  For diagnostics and for a UI that has to
        disable the controls that cannot run at the same time."""
        return self._session_owner

    # ═══════════════════════════════════════════════════════════════════
    # Context manager
    # ═══════════════════════════════════════════════════════════════════

    def __enter__(self) -> "LiteGrip":
        self.connect()
        return self

    def __exit__(self, *args) -> None:
        self.disconnect()

    def __repr__(self) -> str:
        status = "已连接" if self._connected else "未连接"
        enabled = "已使能" if self._enabled else "未使能"
        mst = f"0x{self._mst_id:02X}" if self._mst_id else "auto"
        return (f"LiteGrip(ch={self._channel}, CAN_ID=0x{self._can_id:02X}, "
                f"MST_ID={mst}, {status}, {enabled})")

    def __str__(self) -> str:
        return self.__repr__()
