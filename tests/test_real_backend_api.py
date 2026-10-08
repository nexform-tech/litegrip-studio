"""The real backend: which SDK calls it may make, and what it does with them.

Two halves, and they check different things.

``TestSdkSurface`` reads ``real.py`` as source.  The rule it enforces — never
call the SDK's convenience motion methods — cannot be checked at runtime,
because the paths that would call them are exactly the ones that only run
against hardware.  An AST walk does check it, and it also catches the case a
test never would: a method that is reachable but not exercised.

Everything else drives a recording stub in place of ``LiteGrip``.  That is what
makes the calibration paths testable without a CAN interface — and those paths
are where the dangerous behaviour lives, since ``load_calibration`` returns
``True`` whether it loaded the file it was given or fell back to the factory
one.  The stub reproduces that, so the cross-check has something real to catch.
"""

from __future__ import annotations

import ast
import json
import math
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest

import litegrip

#: Everything here is about the surface of the real SDK: which of its names the
#: backend may call, and what it does with the errors that SDK raises.  A
#: stand-in would test the stand-in.  The SDK is vendored under ``src/litegrip``
#: rather than being a dependency, so it is always importable.
GripperConfig = litegrip.GripperConfig
LiteGripError = litegrip.LiteGripError

from litegrip_studio import calibration, constants
from litegrip_studio.backend import (
    BackendError,
    ConnectFailed,
    EnableFailed,
    FaultActive,
    LinkDown,
    NotReady,
)
from litegrip_studio.backend.real import RealBackend
from litegrip_studio.units import Limits

from conftest import shipped_factory_calibrations

REAL_SOURCE = Path(__file__).resolve().parent.parent / "src/litegrip_studio/backend/real.py"


def _sdk_link_error(fault_code: int | None = None) -> LiteGripError:
    """What ``LiteGrip.enable()`` raises when the interface cannot send.

    Reproduced from the SDK rather than invented: the send inside
    ``can_bus.initialize`` raises ``OSError``, the retry path calls
    ``clear_fault`` *outside* the ``try`` that would have swallowed it, and
    ``gripper.enable``'s ``except Exception as e: raise HardwareError(f"使能失败:
    {e}")`` turns it into this — with the errno surviving only in the implicit
    ``__context__``.
    """
    with pytest.raises(LiteGripError) as excinfo:
        try:
            raise OSError(100, "Network is down")
        except OSError as cause:
            if fault_code is None:
                raise LiteGripError(f"使能失败: {cause}") from cause
            raise LiteGripError("使能失败: 电机故障 0x9", error_code=fault_code) from cause
    return excinfo.value

# Every convenience method the README and the SDK's own examples reach for, and
# every one of them is unusable here: the motion methods block inside
# ``control_mit_stream`` (no abort hook), ``move_at_speed`` appends a ~100 ms
# hold per call, ``home`` uses a hardcoded closed angle, and the calibration
# methods read stdin or wait for Ctrl+C.
FORBIDDEN = frozenset(
    {
        "open",
        "close",
        "grasp",
        "release",
        "move_to",
        "move_at_speed",
        "goto",
        "goto_rad",
        "home",
        "calibrate",
        "calibrate_guided",
        "calibrate_manual",
        "calibrate_auto",
        "enter_zero_gravity",
        "exit_zero_gravity",
        "control_mit_stream",
    }
)

USER_RAW = {
    "channel": "can0",
    "can_id": 8,
    "mst_id": 18,
    "canfd_mode": False,
    "zero_position_rad": 1.775959,
    "max_position_rad": -0.064279,
    "travel_range_rad": 1.840238,
    "rad_to_mm": 65.21,
    "motor_type": "DM4310",
    "kp": 100.0,
    "kd": 2.0,
    "grasp_torque_threshold": 0.5,
}

#: The factory calibration the console falls back to, read from the copy this
#: repository ships rather than retyped.  Nothing below depends on the numbers,
#: only on the file being a valid factory calibration — and retyping them is how
#: this fixture came to hold another SDK's nominal pair while still being loaded
#: as if it were the file the console would really fall back to.
FACTORY_RAW = json.loads(
    shipped_factory_calibrations()[0].read_text(encoding="utf-8")
)

#: The same shape with the closed stop at the smaller angle, which is what a
#: reverse-mounted gripper records.  Its scale is the one derived from its own
#: travel and the console's travel setting, so nothing about it is a warning.
REVERSED_RAW = dict(
    USER_RAW,
    zero_position_rad=-0.300793,
    max_position_rad=1.421569,
    travel_range_rad=1.722362,
    rad_to_mm=49.93,
)


class StubGripper:
    """The SDK's public surface, recorded.

    Only the methods :class:`RealBackend` is allowed to call are implemented;
    anything else would show up as an ``AttributeError`` in a test rather than
    as a silent pass.
    """

    def __init__(self, config: GripperConfig | None = None) -> None:
        self.config = config or GripperConfig()
        self.channel = self.config.can_channel
        self.is_connected = False
        self.is_enabled = False

        self.calls: list[tuple] = []
        self.frames: list[dict] = []
        self.loaded: list[str | None] = []
        self.saved: list[str] = []
        self.waits: list[bool] = []

        self.connect_error: Exception | None = None
        self.enable_error: Exception | None = None
        self.enable_result = True
        self.fault_cleared = True
        # When set, ``load_calibration`` returns True but leaves the config
        # alone — the SDK's silent fallback, faithfully reproduced.
        self.ignore_calibration = False

        # ``position_mm`` is a sentinel: the backend must not use it.
        self.state = SimpleNamespace(
            position_rad=self.config.pos_closed_rad,
            velocity_rad_s=0.0,
            torque_nm=0.0,
            temperature_mos=30,
            temperature_coil=31,
            error_code=constants.ERROR_ENABLED,
            position_mm=-999.0,
        )

    # ── lifecycle ───────────────────────────────────────────────────────────
    def connect(self) -> bool:
        if self.connect_error is not None:
            raise self.connect_error
        self.is_connected = True
        return True

    def disconnect(self) -> None:
        self.is_connected = False
        self.is_enabled = False

    def enable(self) -> bool:
        self.calls.append(("enable",))
        if self.enable_error is not None:
            raise self.enable_error
        self.is_enabled = self.enable_result
        return self.enable_result

    def disable(self) -> bool:
        self.is_enabled = False
        return True

    def clear_fault(self) -> bool:
        if not self.fault_cleared:
            raise LiteGripError("故障清除失败", error_code=constants.ERROR_UV)
        return True

    # ── control ─────────────────────────────────────────────────────────────
    def send_mit_frame(self, q: float, kp: float, kd: float, dq: float, tau: float) -> bool:
        if not self.is_enabled:
            return False
        self.frames.append({"q": q, "kp": kp, "kd": kd, "dq": dq, "tau": tau})
        return True

    def poll(self, timeout_s: float = 0.0) -> bool:
        assert timeout_s == 0.0, "poll must never block"
        return self.is_connected

    def get_state(self, wait: bool = True) -> SimpleNamespace:
        self.waits.append(wait)
        return self.state

    def stop(self) -> None:
        self.calls.append(("stop",))

    # ── calibration ─────────────────────────────────────────────────────────
    def load_calibration(self, path: str | None = None) -> bool:
        self.calls.append(("load_calibration", path))
        self.loaded.append(path)
        if self.ignore_calibration:
            return True
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        self.config.pos_closed_rad = float(data["zero_position_rad"])
        self.config.pos_open_rad = float(data["max_position_rad"])
        self.config.rad_to_mm = float(data["rad_to_mm"])
        for key, attr in (("can_id", "can_id"), ("mst_id", "mst_id"), ("kp", "kp"), ("kd", "kd")):
            if key in data:
                setattr(self.config, attr, data[key])
        return True

    def save_calibration(self, path: str | None = None) -> str:
        self.saved.append(str(path))
        data = {
            "channel": self.config.can_channel,
            "can_id": self.config.can_id,
            "mst_id": self.config.mst_id or 0,
            "canfd_mode": self.config.canfd_mode,
            "zero_position_rad": self.config.pos_closed_rad,
            "max_position_rad": self.config.pos_open_rad,
            "travel_range_rad": abs(self.config.pos_open_rad - self.config.pos_closed_rad),
            "rad_to_mm": self.config.rad_to_mm,
            "motor_type": "DM4310",
            "kp": self.config.kp,
            "kd": self.config.kd,
            "grasp_torque_threshold": self.config.grasp_torque_threshold,
        }
        Path(path).write_text(json.dumps(data, indent=2), encoding="utf-8")
        return str(path)


@pytest.fixture
def env(tmp_path, monkeypatch):
    """A calibration environment isolated to ``tmp_path``."""
    user = tmp_path / "user.json"
    factory = tmp_path / "factory.json"
    monkeypatch.setenv("LITEGRIP_CALIB", str(user))
    monkeypatch.setenv("LITEGRIP_FACTORY_CALIB", str(factory))

    def write(path: Path, data: dict) -> Path:
        path.write_text(json.dumps(data), encoding="utf-8")
        return path

    return SimpleNamespace(user=user, factory=factory, dir=tmp_path, write=write)


@pytest.fixture
def backend(env):
    """A backend over a stub, connected and enabled — with nothing loaded.

    Deliberately uncalibrated: that is the state the console starts in, and it
    is the state in which every motion refusal has to hold.
    """
    stub = StubGripper()
    backend = RealBackend(gripper=stub)
    backend.connect()
    backend.enable()
    return backend


@pytest.fixture
def armed(backend, env):
    """A connected, enabled backend with a usable user calibration loaded."""
    env.write(env.user, USER_RAW)
    assert backend.load_calibration(str(env.user)) is True
    return backend


def _stub(backend: RealBackend) -> StubGripper:
    return backend._gripper


# ── which SDK calls are allowed ─────────────────────────────────────────────
class TestSdkSurface:
    """Source-level, because the paths that would misuse the SDK need hardware."""

    @pytest.fixture
    def tree(self) -> ast.Module:
        return ast.parse(REAL_SOURCE.read_text(encoding="utf-8"))

    @pytest.fixture
    def attributes(self, tree: ast.Module) -> set[str]:
        return {
            node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)
        }

    def test_no_convenience_motion_method_is_ever_called(self, attributes) -> None:
        """Any of these would defeat the E-stop, the speed setting, or both.

        Read from the AST rather than the text, because the module docstring
        and the comments below it name every one of them while explaining why
        they are not used.
        """
        assert not (attributes & FORBIDDEN), (
            f"real.py calls {sorted(attributes & FORBIDDEN)}, which the console "
            "cannot interrupt"
        )

    def test_the_primitives_it_must_use_are_present(self, attributes) -> None:
        """Guards the test above: an empty or mis-parsed file would pass it."""
        assert {
            "send_mit_frame",
            "poll",
            "get_state",
            "stop",
            "load_calibration",
            "save_calibration",
        } <= attributes

    def test_only_public_sdk_names_are_imported(self, tree: ast.Module) -> None:
        import litegrip

        public = set(litegrip.__all__)
        imported: set[str] = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom) and node.module == "litegrip":
                imported |= {alias.name for alias in node.names}
        assert imported, "the module stopped importing the SDK"
        assert imported <= public, f"non-public SDK imports: {sorted(imported - public)}"

    def test_no_private_attribute_of_the_gripper_is_touched(self, tree: ast.Module) -> None:
        """``_can`` and friends are how a console ends up coupled to the SDK's
        internals — and how ``move_at_speed`` looked like the easy option."""
        offenders = []
        for node in ast.walk(tree):
            if not isinstance(node, ast.Attribute) or not node.attr.startswith("_"):
                continue
            value = node.value
            if isinstance(value, ast.Attribute) and value.attr.startswith("_gripper"):
                offenders.append(node.attr)
        assert not offenders, f"private SDK attributes used: {offenders}"


# ── lifecycle ───────────────────────────────────────────────────────────────
class TestLifecycle:
    def test_a_connect_failure_is_reported_as_ours(self, env) -> None:
        backend = RealBackend(gripper=StubGripper())
        _stub(backend).connect_error = LiteGripError("no such device")
        with pytest.raises(ConnectFailed, match="no such device"):
            backend.connect()

    def test_a_disabled_motor_reports_the_disabled_code(self, env) -> None:
        backend = RealBackend(gripper=StubGripper())
        tele = backend.read()
        assert tele.error_code == constants.ERROR_DISABLED
        assert tele.position_rad == 0.0

    def test_enable_does_not_load_a_calibration(self, backend) -> None:
        """The SDK would quietly fall back to its factory file and report
        success; the operator has to see the provenance first."""
        assert _stub(backend).loaded == []

    def test_a_latched_fault_surfaces_as_a_fault_not_a_failure(self, env) -> None:
        backend = RealBackend(gripper=StubGripper())
        backend.connect()
        _stub(backend).enable_error = LiteGripError(
            "driver refused", error_code=constants.ERROR_UV
        )
        with pytest.raises(FaultActive) as excinfo:
            backend.enable()
        assert excinfo.value.code == constants.ERROR_UV
        assert "欠压" in str(excinfo.value)

    def test_an_enable_that_returns_false_is_a_failure(self, env) -> None:
        backend = RealBackend(gripper=StubGripper())
        backend.connect()
        _stub(backend).enable_result = False
        with pytest.raises(EnableFailed):
            backend.enable()

    def test_a_dead_interface_is_a_link_problem_not_a_motor_one(self, env) -> None:
        """The failure the operator actually meets on 使能: the interface is down
        or the controller is bus-off, so the first frame that has to leave the
        machine never does.  Reporting it as a motor failure would send them to
        the 24 V supply for a problem that is at the adapter."""
        backend = RealBackend(gripper=StubGripper())
        backend.connect()
        _stub(backend).enable_error = _sdk_link_error()

        with pytest.raises(LinkDown) as excinfo:
            backend.enable()

        assert "can0" in str(excinfo.value), "the interface is named"
        assert "ENETDOWN" in str(excinfo.value)
        assert "欠压" not in str(excinfo.value), "no motor advice for a link fault"

    def test_a_transport_errno_outranks_a_stale_fault_code(self, env) -> None:
        """When both are present the link wins, because a fault code can only
        have come from a frame that arrived *before* the interface stopped
        carrying them, while the errno describes this attempt."""
        backend = RealBackend(gripper=StubGripper())
        backend.connect()
        _stub(backend).enable_error = _sdk_link_error(fault_code=0x9)

        with pytest.raises(LinkDown):
            backend.enable()

    def test_enable_before_connect_is_refused(self, env) -> None:
        backend = RealBackend(gripper=StubGripper())
        with pytest.raises(ConnectFailed):
            backend.enable()

    def test_a_failed_clear_keeps_the_fault(self, env) -> None:
        backend = RealBackend(gripper=StubGripper())
        backend.connect()
        _stub(backend).fault_cleared = False
        with pytest.raises(FaultActive):
            backend.clear_fault()

    def test_disconnect_swallows_a_failing_teardown(self, backend) -> None:
        """There is nothing to recover at shutdown, and raising would mask the
        reason the session is ending."""

        def boom() -> None:
            raise LiteGripError("transport already gone")

        _stub(backend).disconnect = boom
        backend.disconnect()

    def test_describe_names_the_link_and_the_motor_and_nothing_that_changes(self, backend) -> None:
        """It is taken once, when the connection opens, and never refreshed.

        So it may only carry what cannot go out of date: the calibration it used
        to end with was read *before* the load — the console says 未标定 on the
        connection line for the whole session, file loaded or not — and the motor
        id was the motor's own, not the file's, so it was the same whichever file
        was in force.
        """
        text = backend.describe()

        assert "can0" in text
        assert "0x08" in text
        assert "未标定" not in text
        assert "mst_id" not in text


# ── the primitives ──────────────────────────────────────────────────────────
class TestPrimitives:
    def test_a_frame_reaches_the_sdk_unchanged(self, armed) -> None:
        assert armed.stream_frame(1.5, 100.0, 2.0, -0.5, 0.3) is True
        assert _stub(armed).frames == [
            {"q": 1.5, "kp": 100.0, "kd": 2.0, "dq": -0.5, "tau": 0.3}
        ]

    def test_nothing_is_sent_while_disabled(self, backend) -> None:
        backend.disable()
        assert backend.stream_frame(1.5, 100.0, 2.0) is False
        assert _stub(backend).frames == []

    def test_reading_never_blocks(self, backend) -> None:
        """Every other accessor on the SDK waits 50 ms for a fresh frame — ten
        ticks — so a display built on one would stall the control loop."""
        backend.read()
        assert _stub(backend).waits == [False]

    def test_position_mm_comes_from_our_limits_not_the_sdks_config(self, backend, env) -> None:
        """The SDK's ``get_state().position_mm`` is derived from its own config.
        Using it would mean the position the FSM reads back was converted by a
        different mapping from the one it used to write the command.

        Our mapping is the derived scale, so this is also where a reading in the
        file's own millimetres — the SDK's nominal 120 over the span, which is
        what makes the jaws read 120 mm wide — is kept out of the display.
        """
        env.write(env.user, USER_RAW)
        assert backend.load_calibration(str(env.user)) is True
        _stub(backend).state.position_rad = 1.5

        limits = backend.limits()
        tele = backend.read()
        assert tele.position_mm == pytest.approx(limits.to_mm(1.5))
        assert tele.position_mm != pytest.approx(-999.0)
        by_the_file = (
            USER_RAW["zero_position_rad"] - 1.5
        ) * USER_RAW["rad_to_mm"]
        assert limits.rad_to_mm != pytest.approx(USER_RAW["rad_to_mm"])
        assert tele.position_mm != pytest.approx(by_the_file)

    def test_position_mm_is_zero_with_no_calibration(self, backend) -> None:
        """Millimetres are undefined until the travel is known, and the SDK's
        fallback number would come from the reversed default config."""
        backend._info = None
        _stub(backend).state.position_rad = 1.5
        assert backend.read().position_mm == 0.0

    def test_force_is_reported_even_uncalibrated(self, backend) -> None:
        """It is a torque times a constant — and the probe needs it before any
        calibration exists."""
        backend._info = None
        _stub(backend).state.torque_nm = 1.25
        assert backend.read().force_n == pytest.approx(12.5)

    def test_zero_torque_reaches_the_sdk(self, backend) -> None:
        backend.zero_torque()
        assert ("stop",) in _stub(backend).calls

    def test_poll_is_a_fresh_frame_signal(self, backend) -> None:
        assert backend.poll() is True
        backend.disconnect()
        assert backend.poll() is False


class TestFrameRefusal:
    """The last line of defence before the motor."""

    def test_a_legal_frame_is_sent(self, armed) -> None:
        assert armed.stream_frame(1.0, 100.0, 2.0) is True
        assert len(_stub(armed).frames) == 1

    @pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
    def test_a_non_finite_value_is_never_transmitted(self, armed, bad: float) -> None:
        for args in ((bad, 100.0, 2.0), (1.0, bad, 2.0), (1.0, 100.0, bad), (1.0, 100.0, 2.0, bad)):
            assert armed.stream_frame(*args) is False
        assert _stub(armed).frames == []

    def test_a_target_outside_the_travel_is_refused(self, armed, env) -> None:
        """Refused, not clamped: a clamp would silently convert a bug into a
        legal-looking command heading for the hard stop."""
        limits = armed.limits()
        assert armed.stream_frame(limits.rad_high + 0.01, 100.0, 2.0) is False
        assert armed.stream_frame(limits.rad_low - 0.01, 100.0, 2.0) is False
        assert _stub(armed).frames == []

    def test_the_travel_endpoints_themselves_are_allowed(self, armed) -> None:
        limits = armed.limits()
        assert armed.stream_frame(limits.rad_low, 100.0, 2.0) is True
        assert armed.stream_frame(limits.rad_high, 100.0, 2.0) is True

    def test_an_ungated_frame_may_leave_the_travel_it_is_measuring(self, armed) -> None:
        """The guided probe exists to find the mechanical stops, and those sit
        outside the calibrated travel by design — the red lines are the margin.

        Applying the travel check to it trapped the probe inside the very band
        it was measuring: it recorded the last step it was allowed to command as
        the open limit, a step short of the stop, without ever pressing hard
        enough to stall.  A wrong answer that reads as a successful measurement
        of a stop nobody touched.
        """
        beyond = armed.limits().rad_low - 0.08
        assert armed.stream_frame(beyond, 60.0, 2.0) is False
        assert armed.stream_frame(beyond, 60.0, 2.0, ungated=True) is True
        assert _stub(armed).frames[-1]["q"] == beyond

    def test_an_ungated_frame_is_still_refused_when_it_is_nonsense(self, backend) -> None:
        """``ungated`` relaxes the calibration requirement and the travel
        check, not the sanity checks: those are wrong whatever a calibration
        says, and on that path the frame is the one thing that presses."""
        assert backend.stream_frame(math.nan, 100.0, 2.0, ungated=True) is False
        assert backend.stream_frame(1.0, -1.0, 2.0, ungated=True) is False
        assert _stub(backend).frames == []
        # Uncalibrated is what a probe is for, so that one is sent.
        assert backend.stream_frame(1.0, 100.0, 2.0, ungated=True) is True

    def test_a_negative_gain_is_refused(self, armed) -> None:
        assert armed.stream_frame(1.0, -1.0, 2.0) is False
        assert armed.stream_frame(1.0, 100.0, -1.0) is False
        assert _stub(armed).frames == []

    def test_a_zero_gain_frame_reaches_an_uncalibrated_motor(self, backend) -> None:
        """零重力 (``RELEASE``) and the wizard's zero gravity (``ZERO_G``) carry
        no pose: kp and kd are zero, so there is no
        position for a calibration to have got wrong, and the frame cannot move
        the axis whatever the angle field says.

        They used to be refused here, and that is what made a bad file
        un-escapable: with the gate shut, 零重力 did nothing, the operator could
        not push the jaws by hand, and re-calibrating needs the jaws to move.
        """
        backend._info = None
        assert backend.stream_frame(0.9, 0.0, 0.0, 0.0, 0.0, ungated=True) is True
        assert len(_stub(backend).frames) == 1

    def test_an_ungated_hold_may_sit_outside_a_stale_travel(self, armed) -> None:
        """The pose a probe is left in is the one the encoder reported, and it
        is outside the old file's travel by exactly the amount that made the
        file wrong.  Refusing it leaves the axis free while the log says held."""
        stale = armed.limits().rad_low - 0.9
        assert armed.stream_frame(stale, 60.0, 2.0) is False
        assert armed.stream_frame(stale, 60.0, 2.0, ungated=True) is True
        assert _stub(armed).frames[-1]["q"] == stale

    def test_a_position_frame_is_still_refused_without_a_calibration(
        self, backend
    ) -> None:
        """The relaxation belongs to the flag, not to a frame that happens to
        have no gain: the same call without it must still be refused, or every
        move on an uncalibrated console would reach the motor."""
        backend._info = None
        assert backend.stream_frame(0.9, 0.0, 0.0) is False
        assert backend.stream_frame(0.9, 100.0, 2.0) is False
        assert _stub(backend).frames == []

    def test_zero_gains_are_allowed(self, armed) -> None:
        """Zero stiffness is how the gripper is made back-drivable, so it must
        not be caught by a check meant for negative stiffness."""
        assert armed.stream_frame(1.0, 0.0, 0.0) is True

    def test_nothing_is_sent_without_a_usable_calibration(self, backend) -> None:
        """The gate blocks motion in this state; if a frame arrives anyway then
        something above has failed, and the frame is what would hurt."""
        backend._info = None
        assert backend.stream_frame(1.0, 100.0, 2.0) is False
        assert _stub(backend).frames == []

    def test_an_unusable_calibration_is_not_a_licence_to_transmit(self, env) -> None:
        backend = RealBackend(gripper=StubGripper())
        backend.connect()
        backend.enable()
        env.write(env.user, dict(USER_RAW, max_position_rad=USER_RAW["zero_position_rad"]))
        assert backend.load_calibration(str(env.user)) is False
        assert backend.stream_frame(0.5, 100.0, 2.0) is False
        assert _stub(backend).frames == []

    def test_a_reverse_mounted_calibration_is(self, env) -> None:
        """The ordering of the two angles is not what refuses a file — the
        encoder is, and this backend has not read one.  A gripper whose angle
        grows as the jaws open is a gripper, and refusing to command it was the
        deadlock that made such a unit impossible to calibrate."""
        backend = RealBackend(gripper=StubGripper())
        backend.connect()
        backend.enable()
        env.write(env.user, REVERSED_RAW)
        assert backend.load_calibration(str(env.user)) is True
        assert backend.stream_frame(0.5, 100.0, 2.0) is True

    def test_a_repeating_refusal_is_logged_once(self, armed, caplog) -> None:
        """This sits on a 200 Hz path; 200 identical lines a second would bury
        the first one, which is the only one worth reading."""
        for _ in range(50):
            armed.stream_frame(math.nan, 100.0, 2.0)
        records = [r for r in caplog.records if "拒绝发送" in r.getMessage()]
        assert len(records) == 1

    def test_a_different_refusal_is_logged_again(self, armed, caplog) -> None:
        armed.stream_frame(math.nan, 100.0, 2.0)
        armed.stream_frame(99.0, 100.0, 2.0)
        records = [r for r in caplog.records if "拒绝发送" in r.getMessage()]
        assert len(records) == 2

    def test_a_successful_frame_clears_the_refusal_memory(self, armed, caplog) -> None:
        armed.stream_frame(math.nan, 100.0, 2.0)
        armed.stream_frame(1.0, 100.0, 2.0)
        armed.stream_frame(math.nan, 100.0, 2.0)
        records = [r for r in caplog.records if "拒绝发送" in r.getMessage()]
        assert len(records) == 2


# ── calibration ─────────────────────────────────────────────────────────────
class TestLoadCalibration:
    def test_a_good_user_file_is_applied_and_usable(self, backend, env) -> None:
        env.write(env.user, USER_RAW)
        assert backend.load_calibration(str(env.user)) is True

        info = backend.calibration_info()
        assert info is not None
        assert info.provenance == calibration.PROVENANCE_USER
        assert info.usable
        assert backend.limits().closed_rad == pytest.approx(USER_RAW["zero_position_rad"])

    def test_the_sdk_is_handed_the_path_we_validated(self, backend, env) -> None:
        env.write(env.user, USER_RAW)
        backend.load_calibration(str(env.user))
        assert _stub(backend).loaded == [str(env.user)]

    def test_the_factory_file_is_passed_explicitly_too(self, backend, env) -> None:
        """Passing ``None`` here would let the SDK choose the file itself, which
        is the silent fallback this module exists to prevent."""
        env.write(env.factory, FACTORY_RAW)
        assert backend.load_calibration() is True
        assert _stub(backend).loaded == [str(env.factory)]
        assert backend.calibration_info().provenance == calibration.PROVENANCE_FACTORY

    def test_an_unusable_file_is_never_handed_to_the_sdk(self, backend, env) -> None:
        env.write(env.user, dict(USER_RAW, max_position_rad=USER_RAW["zero_position_rad"]))
        env.write(env.factory, FACTORY_RAW)
        assert backend.load_calibration(str(env.user)) is False
        assert _stub(backend).loaded == [], "the SDK would have loaded the factory file"

    def test_a_missing_calibration_is_never_handed_to_the_sdk(self, backend) -> None:
        assert backend.load_calibration() is False
        assert _stub(backend).loaded == []

    def test_a_broken_file_is_never_handed_to_the_sdk(self, backend, env) -> None:
        env.user.write_text("{ truncated", encoding="utf-8")
        assert backend.load_calibration(str(env.user)) is False
        assert _stub(backend).loaded == []

    def test_a_silent_fallback_is_caught(self, backend, env) -> None:
        """The SDK returns True for a file it did not actually apply — it read
        ours, found something it preferred, and said nothing.  Without the
        cross-check the console would run on numbers nobody vetted."""
        env.write(env.user, USER_RAW)
        _stub(backend).ignore_calibration = True

        assert backend.load_calibration(str(env.user)) is False
        info = backend.calibration_info()
        assert info is not None
        assert any("交叉核对失败" in p for p in info.problems)
        assert not info.usable
        assert not info.motion_allowed

    def test_a_cross_check_failure_blocks_motion_at_the_backend(self, backend, env) -> None:
        """Belt and braces: the worker's gate refuses on the same condition, but
        the backend must not be willing either."""
        env.write(env.user, USER_RAW)
        _stub(backend).ignore_calibration = True
        backend.load_calibration(str(env.user))
        assert backend.stream_frame(1.0, 100.0, 2.0) is False
        assert _stub(backend).frames == []

    def test_a_disputed_calibration_still_reports_its_numbers(self, backend, env) -> None:
        """The UI has to show what it tried to load, or the mismatch is
        invisible — which is the failure the cross-check exists to end."""
        env.write(env.user, USER_RAW)
        _stub(backend).ignore_calibration = True
        backend.load_calibration(str(env.user))
        assert backend.limits().closed_rad == pytest.approx(USER_RAW["zero_position_rad"])

    def test_a_rejected_load_is_reported_as_a_problem(self, backend, env) -> None:
        env.write(env.user, USER_RAW)
        _stub(backend).load_calibration = lambda path=None: False
        assert backend.load_calibration(str(env.user)) is False
        assert any("未能载入" in p for p in backend.calibration_info().problems)

    def test_the_file_may_move_the_can_ids_and_say_so(self, backend, env) -> None:
        """``load_calibration`` overwrites can_id/mst_id/channel/kp/kd from the
        file, so a file describing a different link silently retargets it."""
        env.write(env.user, dict(USER_RAW, can_id=0x09, mst_id=0x19))
        backend.load_calibration(str(env.user))
        config = _stub(backend).config
        assert config.can_id == 0x09
        assert config.mst_id == 0x19

    def test_limits_fall_back_to_the_config_when_unusable(self, backend, env) -> None:
        """The world stays describable while motion is refused."""
        env.write(env.user, dict(USER_RAW, max_position_rad=USER_RAW["zero_position_rad"]))
        backend.load_calibration(str(env.user))
        assert backend.limits() == Limits.from_config(_stub(backend).config)


class TestSaveCalibration:
    def test_saving_promotes_an_in_memory_result_to_a_user_file(self, backend, env) -> None:
        info = backend.set_calibration_memory(1.775959, -0.064279, 65.21)
        assert info.provenance == calibration.PROVENANCE_MEMORY
        assert not info.is_user, "an unsaved result is not a calibration yet"
        assert backend.stream_frame(1.0, 100.0, 2.0) is False, "still gated"

        target = env.dir / "saved.json"
        written = backend.save_calibration(str(target))
        assert Path(written).is_file()

        after = backend.calibration_info()
        assert after.provenance == calibration.PROVENANCE_USER
        assert after.usable
        assert after.limits == info.limits
        assert backend.stream_frame(1.0, 100.0, 2.0) is True, "the gate is open"

    def test_the_config_carries_our_numbers_before_the_sdk_writes(self, backend, env) -> None:
        """The SDK writes the file from its own config, so anything we forgot to
        push across would be saved over.

        Including the scale, and it is the derived one rather than the number the
        probe handed in: the file records what this console moves by, so that a
        reload agrees with itself instead of re-deriving a different value.
        """
        backend.set_calibration_memory(1.775959, -0.064279, 65.21)
        backend.save_calibration(str(env.dir / "saved.json"))

        written = json.loads((env.dir / "saved.json").read_text(encoding="utf-8"))
        assert written["zero_position_rad"] == pytest.approx(1.775959)
        assert written["max_position_rad"] == pytest.approx(-0.064279)
        assert written["rad_to_mm"] == pytest.approx(backend.limits().rad_to_mm)
        assert written["rad_to_mm"] != pytest.approx(65.21)

    def test_the_factory_file_is_never_the_save_target(self, backend, env) -> None:
        """The SDK's bundled file describes whichever unit it was taken on.

        Overwriting it would replace the fallback every later install of this
        console depends on.  It is also reachable without meaning to: the target
        is the stored path, which is whatever the console was told to load at
        launch.  The UI used to keep it out of reach by offering the save only
        for a USER or MEMORY provenance — a finished probe now writes itself
        out, so the refusal has to live here.
        """
        backend.set_calibration_memory(1.775959, -0.064279, 65.21)

        with pytest.raises(BackendError, match="出厂标定"):
            backend.save_calibration(str(env.factory))

        assert not env.factory.exists()
        assert _stub(backend).saved == [], "nothing reached the SDK"

    def test_a_symlink_to_the_factory_file_is_the_factory_file(
        self, backend, env
    ) -> None:
        """The target arrives as a string the operator typed, so the same file
        can be named more than one way — and the ways that look least like the
        factory file are the ones a check on the string would miss."""
        env.factory.write_text("{}", encoding="utf-8")
        innocent = env.dir / "innocent.json"
        innocent.symlink_to(env.factory)
        backend.set_calibration_memory(1.775959, -0.064279, 65.21)

        with pytest.raises(BackendError, match="出厂标定"):
            backend.save_calibration(str(innocent))

        assert env.factory.read_text(encoding="utf-8") == "{}"

    def test_saving_with_nothing_to_save_is_refused(self, backend) -> None:
        with pytest.raises(NotReady):
            backend.save_calibration(str(backend._calibration_path or "/tmp/x.json"))

    def test_nothing_is_written_after_saving_nothing(self, backend) -> None:
        with pytest.raises(NotReady):
            backend.save_calibration("/tmp/never-written.json")
        assert _stub(backend).saved == []


class TestTheTravelSetting:
    def test_it_is_written_through_to_the_config(self, backend) -> None:
        backend.set_travel_mm(200.0)
        assert _stub(backend).config.max_stroke_mm == 200.0

    def test_the_default_travel_is_ours_not_the_sdks(self, backend) -> None:
        """With no ``max_stroke_mm`` on the SDK's config — an older SDK, or one
        built by hand — the fallback is the travel the operator measured, never
        the SDK's 120 mm spec figure.  120 would scale every reading by a number
        nobody measured and put the top of the slider in the wrong place.

        A namespace without the field rather than ``del``: the dataclass keeps
        its default on the class, so deleting the instance attribute only exposes
        the 120 again.
        """
        fields = dict(vars(_stub(backend).config))
        fields.pop("max_stroke_mm")
        _stub(backend).config = SimpleNamespace(**fields)

        backend.set_calibration_memory(1.775959, -0.064279, 65.21)

        assert backend.limits().max_stroke_mm == pytest.approx(
            constants.DEFAULT_TRAVEL_MM
        )

    def test_what_the_console_moves_by_reaches_the_sdk_config(self, backend, env) -> None:
        """After a load, the SDK's config holds the console's numbers — the
        derived scale, not the one the file carries.

        It is not cosmetic: the SDK writes its config to the calibration file, so
        without this a save would put the file's nominal back and the next load
        would disagree with the display the operator is looking at.
        """
        env.write(env.user, USER_RAW)

        backend.load_calibration(str(env.user))

        limits = backend.limits()
        config = _stub(backend).config
        assert config.pos_closed_rad == pytest.approx(limits.closed_rad)
        assert config.pos_open_rad == pytest.approx(limits.open_rad)
        assert config.max_stroke_mm == pytest.approx(limits.max_stroke_mm)
        assert config.rad_to_mm == pytest.approx(limits.rad_to_mm)
        assert config.rad_to_mm != pytest.approx(USER_RAW["rad_to_mm"])

    def test_a_file_from_another_gripper_is_warned_about(self, backend, env) -> None:
        """A 200 mm gripper judged against a nominal 120 warns; judged against
        its own nominal it does not."""
        env.write(env.user, dict(USER_RAW, rad_to_mm=108.68))
        backend.set_travel_mm(200.0)
        assert backend.load_calibration(str(env.user)) is True
        assert not any("另一台夹爪" in w for w in backend.calibration_info().warnings)

    def test_it_reaches_the_limits(self, backend, env) -> None:
        env.write(env.user, USER_RAW)
        backend.set_travel_mm(150.0)
        backend.load_calibration(str(env.user))
        assert backend.limits().max_stroke_mm == 150.0
        assert backend.calibration_info().max_stroke_mm == 150.0


class TestThreadOwnership:
    def test_a_call_from_another_thread_is_refused(self, backend) -> None:
        """The SDK is not thread-safe, so this is enforced rather than
        documented: a race here interleaves CAN frames."""
        errors: list[BaseException] = []

        def worker() -> None:
            try:
                backend.read()
            except BaseException as exc:  # noqa: BLE001 - re-raised below
                errors.append(exc)

        thread = threading.Thread(target=worker)
        thread.start()
        thread.join()

        assert len(errors) == 1
        assert isinstance(errors[0], RuntimeError)
        assert "two threads" in str(errors[0])

    def test_the_owning_thread_keeps_working(self, backend) -> None:
        assert backend.read() is not None
        assert backend.owner_tid is not None
