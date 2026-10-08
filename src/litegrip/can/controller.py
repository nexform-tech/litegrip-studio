"""Motor controller — manages one or more DM motors over a CAN transport.

Replaces the damiao_socketcan Motor_Control class with a self-contained
implementation that has zero dependencies outside this package.

Supports:
  - Motor registration / deregistration
  - MIT control frame streaming
  - Command dispatch (enable / disable / clear fault / set zero)
  - Parameter read / write / save
  - Status polling with automatic state decode
  - Control mode switching (MIT ↔ POS_VEL ↔ VEL ↔ POS_FORCE)
"""

from __future__ import annotations

import logging
import time
from typing import Dict, Optional

from ..exceptions import CANTimeoutError
from .transport import CanTransport
from .protocol import (
    BROADCAST_ID,
    ControlMode,
    ControlModeCode,
    DM_REG,
    pack_mit_frame,
    pack_command_frame,
    pack_refresh_frame,
    pack_read_param_frame,
    pack_write_param_frame,
    pack_save_param_frame,
    unpack_status_frame,
    unpack_param_response,
    CMD_ENABLE,
    CMD_DISABLE,
    CMD_CLEAR_FAULT,
    CMD_SET_ZERO,
)
from .motor import MotorState, MotorParams, MotorType

log = logging.getLogger("litegrip.can.controller")


class MotorController:
    """Manages DM motors over a CAN transport.

    Typical usage for a single gripper motor::

        transport = CanTransport("can0")
        transport.open()

        controller = MotorController(transport)
        motor = controller.add_motor(can_id=0x08)

        controller.enable(motor)
        controller.control_mit(motor, q=0.5, kp=100, kd=2)
    """

    def __init__(self, transport: CanTransport):
        self._transport = transport
        self._motors: Dict[int, MotorState] = {}   # keyed by mst_id

    # ── motor management ────────────────────────────────────────────────

    def add_motor(
        self,
        can_id: int,
        mst_id: Optional[int] = None,
        motor_type: MotorType = MotorType.DM4310,
        control_mode: ControlMode = ControlMode.MIT_MODE,
    ) -> MotorState:
        """Register a motor. If mst_id is None, auto-detect via register read.

        Returns:
            The MotorState instance (also accessible via get_motor()).
        """
        if mst_id is None:
            mst_id = self._detect_mst_id(can_id)

        params = MotorParams(
            motor_type=motor_type,
            can_id=can_id,
            mst_id=mst_id,
            control_mode=control_mode,
        )
        motor = MotorState(params)
        self._motors[mst_id] = motor
        log.info("Motor registered: can_id=0x%02X mst_id=0x%02X type=%s",
                 can_id, mst_id, motor_type.name)

        # Install a hardware RX filter for all registered motors' mst_ids.
        # Done AFTER mst_id detection (detection needs unfiltered RX). On a
        # shared bus this stops foreign frames from flooding the RX buffer and
        # starving this motor's status frames — the root cause of "state
        # read-back freezes while the motor keeps moving". Param responses for
        # our motor also arrive on its mst_id, so read_param still works.
        try:
            self._transport.set_id_filter(self._motors.keys())
        except Exception as e:  # never let filtering break registration
            log.warning("Could not set CAN RX filter: %s", e)

        return motor

    def remove_motor(self, motor: MotorState) -> None:
        """Deregister a motor."""
        self._motors.pop(motor.mst_id, None)
        try:
            self._transport.set_id_filter(self._motors.keys())
        except Exception:
            pass

    def get_motor(self, mst_id: int) -> Optional[MotorState]:
        """Get a motor by its mst_id."""
        return self._motors.get(mst_id)

    def get_motor_by_can_id(self, can_id: int) -> Optional[MotorState]:
        """Get the first motor with the given can_id."""
        for m in self._motors.values():
            if m.can_id == can_id:
                return m
        return None

    @property
    def motors(self) -> Dict[int, MotorState]:
        return dict(self._motors)

    # ── command dispatch ────────────────────────────────────────────────

    def send_command(self, motor: MotorState, cmd: int, count: int = 5,
                     interval_s: float = 0.002) -> None:
        """Send a command (enable/disable/clear fault/set zero) to a motor.

        Commands are sent multiple times for reliability (per DM protocol).
        """
        can_id = motor.can_id + motor.mode_offset()
        data = pack_command_frame(cmd)
        self._transport.send_multi(can_id, data, count, interval_s)

    def enable(self, motor: MotorState) -> None:
        """Send enable command (0xFC)."""
        self.send_command(motor, CMD_ENABLE)

    def disable(self, motor: MotorState) -> None:
        """Send disable command (0xFD)."""
        self.send_command(motor, CMD_DISABLE)

    def clear_fault(self, motor: MotorState) -> None:
        """Send clear-fault command (0xFB)."""
        self.send_command(motor, CMD_CLEAR_FAULT)

    def set_zero(self, motor: MotorState) -> None:
        """Set current position as zero (0xFE)."""
        self.send_command(motor, CMD_SET_ZERO)

    def refresh_status(self, motor: MotorState) -> None:
        """Request a status frame refresh (0xCC). Does not change motor output."""
        data = pack_refresh_frame(motor.can_id)
        self._transport.send(BROADCAST_ID, data)

    # ── control modes ───────────────────────────────────────────────────

    def switch_control_mode(self, motor: MotorState,
                            mode_code: ControlModeCode) -> bool:
        """Switch motor control mode (MIT ↔ POS_VEL ↔ VEL ↔ POS_FORCE).

        Writes the CTRL_MODE register and verifies the change.
        """
        self.write_param(motor, DM_REG.CTRL_MODE, int(mode_code))
        time.sleep(0.02)
        # Verify
        try:
            actual = int(self.read_param(motor, DM_REG.CTRL_MODE))
            if actual == int(mode_code):
                # Map code → mode offset
                mode_map = {
                    ControlModeCode.MIT: ControlMode.MIT_MODE,
                    ControlModeCode.POS_VEL: ControlMode.POS_VEL_MODE,
                    ControlModeCode.VEL: ControlMode.VEL_MODE,
                    ControlModeCode.POS_FORCE: ControlMode.POS_FORCE_MODE,
                }
                motor.set_mode(mode_map.get(mode_code, ControlMode.MIT_MODE))
                return True
        except Exception:
            pass
        return False

    # ── MIT control ─────────────────────────────────────────────────────

    def control_mit(self, motor: MotorState, kp: float, kd: float,
                    q: float, dq: float = 0.0, tau: float = 0.0) -> None:
        """Send a single MIT control frame.

        Call in a loop at ≥200 Hz for smooth motion.
        """
        limits = motor.limits
        data = pack_mit_frame(q, dq, kp, kd, tau,
                              q_max=limits.q_max,
                              dq_max=limits.dq_max,
                              tau_max=limits.tau_max)
        can_id = motor.can_id + motor.mode_offset()
        self._transport.send(can_id, data)

    # ── parameter access ────────────────────────────────────────────────

    def read_param(self, motor: MotorState, rid: DM_REG,
                   timeout_s: float = 0.5) -> float:
        """Read a motor register. Returns the decoded value (float).

        Raises:
            CANTimeoutError: if no response within timeout.
        """
        data = pack_read_param_frame(motor.can_id, rid)
        self._transport.send(BROADCAST_ID, data)

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            frame = self._transport.recv(timeout_s=0.01)
            if frame is None:
                continue
            # Check if frame is from our motor
            resp = unpack_param_response(frame.data)
            if resp is not None and resp.can_id == (motor.can_id & 0x0F):
                if resp.opcode in (0x33, 0x55) and resp.rid == rid:
                    return resp.value
        raise CANTimeoutError(
            f"read_param timeout: can_id=0x{motor.can_id:02X} rid={rid}")

    def write_param(self, motor: MotorState, rid: DM_REG,
                    value) -> None:
        """Write a motor register. Does not wait for confirmation."""
        data = pack_write_param_frame(motor.can_id, rid, value)
        self._transport.send(BROADCAST_ID, data)

    def save_params(self, motor: MotorState) -> None:
        """Save all parameters to flash. Motor must be disabled first."""
        data = pack_save_param_frame(motor.can_id)
        self._transport.send(BROADCAST_ID, data)

    # ── polling ──────────────────────────────────────────────────────────

    def poll(self, timeout_s: float = 0.0) -> Optional[MotorState]:
        """Poll for one CAN frame. If it's a status frame for a registered
        motor, decode it and update that MotorState.

        Returns:
            The MotorState that was updated, or None if no relevant frame arrived.
        """
        frame = self._transport.recv(timeout_s=timeout_s)
        if frame is None:
            return None

        # Match frame.can_id against registered motors' mst_id
        motor = self._motors.get(frame.can_id)
        if motor is None:
            return None

        if len(frame.data) < 8:
            return None

        # Guard: skip param response frames (0x33/0x55/0xAA) — they share the
        # motor's mst_id but have a different byte layout and would corrupt
        # motor state if parsed as status.
        #
        # A status frame's data[2] is the velocity low byte, which can collide
        # with an opcode value (e.g. 0x55) — so data[2] alone is NOT a reliable
        # discriminator. A param response is structurally distinct:
        #   data[0] = can_id (high nibble 0)   status: (err<<4)|can_id
        #   data[1] = can_id high byte         status: position high byte
        #   data[2] = opcode                   status: velocity low byte
        # Require all three to match before discarding, so a normal (enabled,
        # err>=1) status frame is never dropped regardless of its position bytes.
        opcode = frame.data[2] if len(frame.data) > 2 else 0
        if (opcode in (0x33, 0x55, 0xAA)
                and (frame.data[0] >> 4) == 0
                and frame.data[0] == (motor.can_id & 0xFF)
                and frame.data[1] == ((motor.can_id >> 8) & 0xFF)):
            return None

        # Decode as a status frame
        try:
            limits = motor.limits
            status = unpack_status_frame(
                frame.data,
                q_max=limits.q_max,
                dq_max=limits.dq_max,
                tau_max=limits.tau_max,
            )
            motor.update_from_status(
                position=status.q,
                velocity=status.dq,
                torque=status.tau,
                error=status.err,
                t_mos=status.t_mos,
                t_coil=status.t_coil,
                timestamp=frame.timestamp,
            )
            return motor
        except ValueError:
            pass

        return None

    def poll_until(self, motor: MotorState,
                   timeout_s: float = 1.0) -> bool:
        """Poll until at least one new status frame arrives for the given motor.

        Returns:
            True if a frame was received, False on timeout.
        """
        prev_rx = motor.rx_count
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            updated = self.poll(timeout_s=0.01)
            if updated is motor and motor.rx_count > prev_rx:
                return True
            time.sleep(0.001)
        return False

    # ── internal ────────────────────────────────────────────────────────

    # Default MST_ID fallback — LiteGrip ships with mst_id=0x18 (24).
    _DEFAULT_MST_ID = 0x18

    def _detect_mst_id(self, can_id: int,
                       timeout_s: float = 0.5) -> int:
        """Auto-detect motor MST_ID.

        Strategy 1: Send read-param for MST_ID register (RID=7) to broadcast.
        The motor responds on its real mst_id; we listen for ALL frames and
        match by the can_id embedded in the parameter response.

        Strategy 2: Send a status-refresh command (0xCC) and listen for the
        status frame — its arrival CAN ID *is* the mst_id.

        Falls back to a hardcoded default if neither strategy succeeds.

        Note: detection must see frames on IDs not yet known, so any active
        RX id-filter is cleared for the duration and restored afterward.
        """
        can_id_low = can_id & 0x0F

        # Clear any active RX filter so detection can see frames on the
        # not-yet-known mst_id; add_motor() reinstalls the filter afterward.
        try:
            self._transport.set_id_filter(None)
        except Exception:
            pass

        # Strategy 1: Read MST_ID register
        try:
            self._transport.drain()
            self._transport.send(BROADCAST_ID,
                                 pack_read_param_frame(can_id, DM_REG.MST_ID))

            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                frame = self._transport.recv(timeout_s=0.01)
                if frame is None:
                    continue
                resp = unpack_param_response(frame.data)
                if resp is not None and resp.rid == DM_REG.MST_ID:
                    if resp.can_id == can_id_low:
                        mst = int(resp.value)
                        if 1 <= mst <= 0xFE:
                            log.info("Detected MST_ID=0x%02X via register read", mst)
                            return mst
        except Exception:
            pass

        # Strategy 2: Send status-refresh; catch the status frame.
        # The status frame arrives on the motor's mst_id → frame.can_id = mst_id.
        log.info("Register read failed; trying status refresh...")
        try:
            self._transport.drain()
            self._transport.send(BROADCAST_ID, pack_refresh_frame(can_id))

            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                frame = self._transport.recv(timeout_s=0.01)
                if frame is None:
                    continue
                if len(frame.data) < 8:
                    continue
                # Status frame: data[0] low nibble = can_id
                if (frame.data[0] & 0x0F) == can_id_low:
                    mst = frame.can_id & 0x7FF
                    if 1 <= mst <= 0xFE:
                        log.info("Detected MST_ID=0x%02X via status refresh", mst)
                        return mst
        except Exception:
            pass

        log.warning(
            "Could not auto-detect MST_ID for can_id=0x%02X; "
            "falling back to default 0x%02X.  "
            "Use --mst-id to override if this is wrong.",
            can_id, self._DEFAULT_MST_ID,
        )
        return self._DEFAULT_MST_ID

    # ── teardown ────────────────────────────────────────────────────────

    def close(self) -> None:
        """Close the underlying transport."""
        self._transport.close()

    def __enter__(self) -> "MotorController":
        return self

    def __exit__(self, *args) -> None:
        self.close()
