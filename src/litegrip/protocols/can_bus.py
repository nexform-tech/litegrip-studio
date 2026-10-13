"""LiteGrip CAN bus layer — self-contained, no damiao_socketcan dependency.

Wraps the litegrip.can subpackage (transport + protocol + motor + controller)
into a simplified interface tailored for a single gripper motor.
"""

from __future__ import annotations

import logging
import math
import time
from typing import Optional

from ..can.transport import CanTransport, CanMode as TransportMode
from ..can.controller import MotorController
from ..can.motor import MotorState, MotorType
from ..can.protocol import (
    ControlMode,
    ControlModeCode,
    DM_REG,
)
from ..constants import GripperParams, DefaultParams
from ..exceptions import (CommandError, CommError, ConnectError, HardwareError,
                          NotInitializedError)

log = logging.getLogger("litegrip.can_bus")


class LiteGripCAN:
    """LiteGrip CAN communication client.

    Manages a single gripper motor (DM4310 by default) over SocketCAN.
    Entirely self-contained — does NOT import damiao_socketcan or any
    arm-specific library.

    Typical usage::

        can = LiteGripCAN(channel="can0")
        can.connect()
        can.register_gripper(can_id=0x08)
        can.initialize()
        can.control_mit_stream(target_position, kp=100, kd=2, duration_s=1.0)
    """

    def __init__(
        self,
        channel: str = DefaultParams.CAN_CHANNEL,
        canfd_mode: bool = DefaultParams.CANFD_MODE,
    ):
        self._channel = channel
        self._canfd_mode = canfd_mode
        self._transport: Optional[CanTransport] = None
        self._controller: Optional[MotorController] = None
        self._motor: Optional[MotorState] = None
        self._connected = False
        self._initialized = False

    # ═══════════════════════════════════════════════════════════════════
    # Connection
    # ═══════════════════════════════════════════════════════════════════

    def connect(self) -> bool:
        """Open the CAN transport. Auto-detects CAN/CAN-FD mode."""
        if self._connected:
            return True

        # None → let CanTransport auto-detect via interface MTU
        mode = TransportMode.CANFD if self._canfd_mode else TransportMode.CAN

        try:
            self._transport = CanTransport(self._channel, mode)
            self._transport.open()
        except OSError as e:
            raise ConnectError(f"CAN 接口 '{self._channel}' 不可用: {e}")

        self._controller = MotorController(self._transport)
        self._connected = True
        log.info("LiteGripCAN connected on %s (mode=%s)",
                 self._channel, self._transport.mode.name)
        return True

    def disconnect(self, disable: bool = True) -> None:
        """Close the CAN transport and release resources.

        Args:
            disable: Send a disable command first (default).  ``False``
                leaves the motor enabled after the transport is closed.
        """
        if not self._connected:
            return

        if self._initialized and disable:
            try:
                self.disable()
            except Exception:
                pass

        if self._controller:
            self._controller.close()
        elif self._transport:
            self._transport.close()

        self._transport = None
        self._controller = None
        self._motor = None
        self._connected = False
        self._initialized = False

    # ═══════════════════════════════════════════════════════════════════
    # Motor registration
    # ═══════════════════════════════════════════════════════════════════

    def register_gripper(
        self,
        can_id: int = GripperParams.CAN_ID,
        mst_id: Optional[int] = None,
        motor_type: MotorType = MotorType.DM4310,
    ) -> int:
        """Register the gripper motor.

        Args:
            can_id: Motor CAN ID (usually 0x08).
            mst_id: Master ID. None = auto-detect via register read / scan.
            motor_type: Motor model (default DM4310).

        Returns:
            The actual (possibly auto-detected) MST_ID.
        """
        if not self._connected or self._controller is None:
            raise CommError("CAN 未连接")

        try:
            self._motor = self._controller.add_motor(
                can_id=can_id,
                mst_id=mst_id,
                motor_type=motor_type,
                control_mode=ControlMode.MIT_MODE,
            )
            return self._motor.mst_id
        except Exception as e:
            raise CommError(f"注册夹爪电机失败: {e}")

    # ═══════════════════════════════════════════════════════════════════
    # Initialization / enable / disable
    # ═══════════════════════════════════════════════════════════════════

    def initialize(self) -> bool:
        """Full initialization: disable → switch to MIT → enable → verify feedback.

        Returns:
            True if the motor reports ``err == 1`` (enabled).  ``err == 0``
            means the enable command did not take effect — the attempt is
            retried rather than reported as success.

        Raises:
            HardwareError: on timeout, persistent fault, no feedback, or if
                every attempt leaves the motor not enabled (``err == 0``).
        """
        if not self._connected or self._controller is None or self._motor is None:
            raise NotInitializedError("未连接或未注册")

        ctrl = self._controller
        m = self._motor
        last_err = -1

        for attempt in range(GripperParams.FAULT_CLEAR_RETRIES):
            try:
                # 1. Disable
                ctrl.disable(m)
                time.sleep(0.01)

                # 2. Switch to MIT mode
                if not ctrl.switch_control_mode(m, ControlModeCode.MIT):
                    log.warning("switchControlMode verification failed; proceeding anyway")
                time.sleep(0.05)

                # 3. Enable
                ctrl.enable(m)

                # 4. Send zero-torque MIT frame and poll for feedback
                prev_rx = m.rx_count
                ctrl.control_mit(m, kp=0, kd=0, q=0, dq=0, tau=0)

                deadline = time.monotonic() + DefaultParams.INIT_TIMEOUT_S
                while time.monotonic() < deadline:
                    ctrl.poll(timeout_s=0.01)
                    if m.rx_count > prev_rx:
                        err = m.error
                        if err == 1:
                            self._initialized = True
                            return True
                        # err == 0 → the enable frame was lost; retry.
                        # anything else → a real fault; retry after clearing.
                        last_err = err
                        break
                    time.sleep(0.005)
                else:
                    last_err = -2  # timeout, no feedback

            except Exception:
                last_err = -3

            # Retry: clear fault before next attempt
            if attempt < GripperParams.FAULT_CLEAR_RETRIES - 1:
                ctrl.clear_fault(m)
                time.sleep(0.005)
                ctrl.enable(m)
                time.sleep(0.01)

        # All retries exhausted → raise
        if last_err == 0x9:
            raise HardwareError(
                "电机欠压故障 (UV_FAULT)，请检查夹爪 24V 供电", error_code=0x9)
        elif last_err == 0:
            raise HardwareError(
                "使能未生效（err=0，enable 命令可能丢帧）", error_code=0)
        elif last_err == -2:
            raise HardwareError(
                f"电机无反馈（超时 {DefaultParams.INIT_TIMEOUT_S}s），"
                "请检查夹爪供电和 CAN 连接")
        elif last_err != -3:
            raise HardwareError(
                f"电机故障 0x{last_err:X} 无法清除", error_code=last_err)
        else:
            raise HardwareError("初始化失败：所有重试耗尽")

    def enable(self) -> bool:
        """Send enable command only (no full init). Prefer initialize()."""
        if self._controller is None or self._motor is None:
            return False
        try:
            self._controller.enable(self._motor)
            self._controller.control_mit(self._motor, kp=0, kd=0, q=0)
            self._controller.poll(timeout_s=0.05)
            return self._motor.error in (0, 1)
        except Exception:
            return False

    def disable(self) -> bool:
        """Send disable command."""
        if self._controller is None or self._motor is None:
            return False
        try:
            self._controller.disable(self._motor)
            self._initialized = False
            return True
        except Exception:
            return False

    def set_zero(self) -> bool:
        """Make the current position the motor's encoder zero (CMD 0xFE).

        This is the raw frame only.  A DM motor **ignores 0xFE while it is
        enabled**, so the caller must send :meth:`disable` first (and re-enable
        afterwards if it wants to keep moving) — that is the order the proven
        bench tool uses.  The motor must already be sitting at the position you
        want to call zero before this is sent; this method sends the command but
        does not verify, so the caller reads the position back to confirm.
        """
        if self._controller is None or self._motor is None:
            return False
        try:
            self._controller.set_zero(self._motor)
            return True
        except Exception:
            return False

    # ═══════════════════════════════════════════════════════════════════
    # Fault handling
    # ═══════════════════════════════════════════════════════════════════

    def _clear_fault(self) -> bool:
        """[Deprecated] Use :meth:`clear_fault` instead."""
        return self.clear_fault()

    def clear_fault(self) -> bool:
        """Clear latched fault and re-enable the motor.

        Tries two strategies:
        1. Direct clear (0xFB) + enable (0xFC) — works for UV/OC/OT faults
           where the motor is already effectively disabled.
        2. Full disable → clear → enable cycle.
        """
        if self._controller is None or self._motor is None:
            return False

        ctrl = self._controller
        m = self._motor

        for _ in range(GripperParams.FAULT_CLEAR_RETRIES):
            # Strategy 1: Direct clear + enable (proven for UV_FAULT)
            ctrl.clear_fault(m)
            time.sleep(0.005)
            ctrl.enable(m)
            time.sleep(0.01)

            # Verify
            ctrl.control_mit(m, kp=0, kd=0, q=0)
            for _ in range(20):
                ctrl.poll(timeout_s=0.01)
                time.sleep(0.003)
            if m.error in (0, 1):
                return True

            # Strategy 2: Full disable → clear → enable
            ctrl.disable(m)
            time.sleep(0.01)
            ctrl.clear_fault(m)
            time.sleep(0.01)
            ctrl.enable(m)
            time.sleep(0.01)

            ctrl.control_mit(m, kp=0, kd=0, q=0)
            for _ in range(20):
                ctrl.poll(timeout_s=0.01)
                time.sleep(0.003)
            if m.error in (0, 1):
                return True

        return False

    # ═══════════════════════════════════════════════════════════════════
    # Motion control
    # ═══════════════════════════════════════════════════════════════════

    def _check_motion_command(self, q_target: float, kp: float, kd: float,
                              dq_target: float,
                              tau_feedforward: float) -> None:
        """拒收一帧 MIT 不该承载的指令（见 :meth:`control_mit`）。

        这一层是**所有**下发的必经之路 —— 引擎的每一帧、``goto_rad`` 的整段
        流、teleop 的连续帧，最后都落在这里 —— 所以数值边界放在这里一次
        就够。上层的判据（领先上限、保力预算）管的是「走多快、压多硬」，这里
        管的是「这一帧本身是不是一个指令」。
        """
        for name, value in (("q_target", q_target), ("kp", kp), ("kd", kd),
                            ("dq_target", dq_target),
                            ("tau_feedforward", tau_feedforward)):
            if not math.isfinite(value):
                raise CommandError(
                    f"MIT 指令的 {name} 不是有限数：{value!r}。非有限值在编码"
                    "器里会被映射到量程的一端（如 q → +12.5 rad、tau → "
                    "+10 Nm），那是一个调用方没要过的真实指令，所以拒绝下发。")

        # 帧量程按型号来（DM4310 是 10 Nm，DM4340 是 28 Nm）。鸭子类型的替身
        # 没带 limits，退回 DM4310 —— 本 SDK 的夹爪就是它。
        limits = getattr(self._motor, "limits", None)
        tau_max = limits.tau_max if limits is not None else 10.0
        if abs(tau_feedforward) > tau_max:
            raise CommandError(
                f"前馈力矩 {tau_feedforward} Nm 超出本型号的帧量程 "
                f"±{tau_max} Nm。超量程会被静默钳到端值 —— 那是另一个力，"
                "不是要的那个，所以拒绝下发。")

    def control_mit(self, q_target: float, kp: float, kd: float,
                    dq_target: float = 0.0,
                    tau_feedforward: float = 0.0) -> bool:
        """Send a single MIT control frame.

        Args:
            q_target: Target position (rad).
            kp: Position stiffness.
            kd: Velocity damping.
            dq_target: Target velocity (rad/s).
            tau_feedforward: Feed-forward torque (Nm).

        Returns:
            True if the frame was sent, False when there is no connection or the
                send failed.  A failed send leaves a warning naming the reason —
                ``False`` alone cannot tell "not enabled" from "the buffer was
                full for 100 ms", and those need different fixes.

        Raises:
            CommandError: a field is NaN/infinite, or the torque exceeds the
                frame's range.  Those are caller bugs rather than transport
                conditions, so they keep a diagnosable reason instead of being
                folded into the ``False`` return.
        """
        self._check_motion_command(q_target, kp, kd, dq_target,
                                   tau_feedforward)
        if self._controller is None or self._motor is None:
            return False
        try:
            self._controller.control_mit(
                self._motor, kp=kp, kd=kd,
                q=q_target, dq=dq_target, tau=tau_feedforward)
            return True
        except Exception as e:
            log.warning("MIT 帧下发失败（%s）：%r", self._channel, e)
            return False

    def poll(self, timeout_s: float = 0.0) -> bool:
        """Poll for one CAN frame and update motor state if relevant.

        Args:
            timeout_s: Max wait time (0 = non-blocking).

        Returns:
            True if a frame for our motor was received.
        """
        if self._controller is None:
            return False
        return self._controller.poll(timeout_s=timeout_s) is not None

    def control_mit_stream(
        self,
        q_target: float,
        kp: float,
        kd: float,
        duration_s: float,
        dq_target: float = 0.0,
        tau_feedforward: float = 0.0,
        interval_s: float = 0.005,
    ) -> bool:
        """Stream MIT frames for a duration (blocking).

        DM motors require continuous MIT frames to sustain motion.

        Returns:
            True only when every frame of the stream went out.  A stream with
            failed frames returns False and logs how many — a single dropped
            frame no longer aborts the stream, because the motor needs the rest
            of the frames, but it must not come back as success either.

        Raises:
            CommandError: a field is NaN/infinite, the torque exceeds the
                frame's range, or the timing is not usable — the same rule as
                :meth:`control_mit`, checked once before the first frame rather
                than per frame.
            CommError: not one frame could be sent.  The bus or the enable is
                broken, and streaming nothing for the whole duration while
                returning success is exactly the silent failure this method used
                to have.
        """
        self._check_motion_command(q_target, kp, kd, dq_target,
                                   tau_feedforward)
        for name, value in (("duration_s", duration_s),
                            ("interval_s", interval_s)):
            if not math.isfinite(value):
                raise CommandError(
                    f"{name} 不是有限数：{value!r}。拒绝下发：duration_s 为 "
                    "NaN 时 ``while now < deadline`` 恒为假 —— 整段一帧都不发"
                    "却返回 True，调用方以为夹爪动了。")
        if duration_s <= 0.0:
            # 与上面 NaN 同一个洞：0 或负数让循环体一次都不进，一帧不发。
            raise CommandError(
                f"duration_s 必须为正：{duration_s!r}。这个时长里一帧都不会"
                "下发，而返回成功就是谎报运动。")
        if interval_s < 0.0:
            raise CommandError(f"interval_s 不能为负：{interval_s!r}。")
        if self._controller is None or self._motor is None:
            return False

        ctrl = self._controller
        m = self._motor
        deadline = time.monotonic() + duration_s

        attempts = 0
        sent = 0
        last_error: Optional[Exception] = None
        while time.monotonic() < deadline:
            attempts += 1
            try:
                ctrl.control_mit(m, kp=kp, kd=kd,
                                 q=q_target, dq=dq_target,
                                 tau=tau_feedforward)
                sent += 1
            except Exception as e:
                last_error = e
            ctrl.poll(timeout_s=0.0)
            time.sleep(interval_s)

        failed = attempts - sent
        if failed:
            log.warning("%s：%d 帧里 %d 帧没发出去（最后一次：%r）",
                        self._channel, attempts, failed, last_error)
        if sent == 0:
            raise CommError(
                f"MIT 流一帧都没发出去（{failed} 次尝试全部失败，最后一次："
                f"{last_error!r}）。整段 {duration_s:.3f} s 一帧都没下发 —— "
                "报成功就是谎报运动。")
        return failed == 0

    # ═══════════════════════════════════════════════════════════════════
    # Status
    # ═══════════════════════════════════════════════════════════════════

    def update_state(self, timeout_s: float = 0.05) -> bool:
        """Poll until at least one new status frame arrives."""
        if self._controller is None or self._motor is None:
            return False
        return self._controller.poll_until(self._motor, timeout_s=timeout_s)

    def get_position(self) -> float:
        """Current position in rad."""
        if self._motor is None:
            return 0.0
        return self._motor.position

    def get_velocity(self) -> float:
        """Current velocity in rad/s."""
        if self._motor is None:
            return 0.0
        return self._motor.velocity

    def get_torque(self) -> float:
        """Current torque in Nm."""
        if self._motor is None:
            return 0.0
        return self._motor.torque

    def get_error(self) -> int:
        """Error code (0=disabled, 1=enabled, 0x9=UV, etc.)."""
        if self._motor is None:
            return -1
        return self._motor.error

    def get_temperature(self) -> tuple[int, int]:
        """MOS and coil temperatures in °C."""
        if self._motor is None:
            return 0, 0
        return self._motor.t_mos, self._motor.t_coil

    def read_param(self, rid: int, timeout_s: float = 0.5) -> float:
        """Read a motor register by RID."""
        if self._controller is None or self._motor is None:
            raise NotInitializedError("未连接或未注册")
        return self._controller.read_param(self._motor, DM_REG(rid),
                                           timeout_s=timeout_s)

    # ═══════════════════════════════════════════════════════════════════
    # Properties
    # ═══════════════════════════════════════════════════════════════════

    @property
    def is_initialized(self) -> bool:
        return self._initialized

    @property
    def motor(self) -> Optional[MotorState]:
        """Direct access to the underlying MotorState (expert use)."""
        return self._motor
