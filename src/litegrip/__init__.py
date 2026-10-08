"""LiteGrip SDK — self-contained gripper library for the LiteGrip adaptive
two-finger hand.

Built on the Damiao motor MIT protocol over SocketCAN.  Zero dependencies
outside Python's standard library + Linux SocketCAN — no damiao_socketcan
or robot-arm libraries required.

Quick start::

    from litegrip import LiteGrip

    with LiteGrip(channel="can0", can_id=0x08) as gripper:
        gripper.enable()          # retries until the status frame says err == 1
        gripper.open()
        gripper.grasp(force_n=20.0, hold_s=3.0)
        state = gripper.get_state()
        print(state)

Subpackages
-----------
- ``litegrip.can`` — Raw SocketCAN transport + DM motor protocol codec.
  Use directly if you need fine-grained control or multi-motor setups.
"""

# Version of a package that is on disk but not installed as a distribution — a
# source checkout, a vendored copy, or the files dropped in by a system package.
# The ``+`` local segment keeps it valid PEP 440 and makes it obvious in a bug
# report that nobody is looking at an official build.
_VERSION_SOURCE_TREE = "0.0.0+source"


def _detect_version(dist_name: str = "litegrip") -> str:
    """Report the installed distribution's version, or a marker for a raw checkout.

    The repository never commits a version back (AGENTS.md §3): semantic-release
    derives it from the commit prefixes and writes it only into the publish
    workspace, so the git tag is the single source of truth. A literal
    ``__version__`` in this file would therefore be a second, permanently stale
    copy of that number. Reading the installed metadata instead keeps whatever the
    user actually installed in one place.

    Returns:
        The installed distribution version, or ``"0.0.0+source"`` when this module
        was imported from loose files rather than an installed distribution.
    """
    from importlib.metadata import PackageNotFoundError, version

    try:
        return version(dist_name)
    except PackageNotFoundError:
        return _VERSION_SOURCE_TREE


__version__ = _detect_version()

# This package deliberately installs no handler and does not call
# ``logging.disable()``: a WARNING such as "this calibration file belongs to
# another channel" is a safety signal, and must reach the user even when the
# embedding program never configured logging.  With no handler, Python's
# ``logging.lastResort`` prints WARNING and above to stderr.  Applications that
# want to capture the records can still attach their own handler to the
# ``litegrip`` logger.

# ── High-level API ──────────────────────────────────────────────────────
from .gripper import (LiteGrip, DEFAULT_CALIB, CALIB_TEMPLATES,
                      default_calib_path, list_templates)

# ── Motion actions ──────────────────────────────────────────────────────
from .actions import (
    MotionConfig,
    MoveProgress,
    MoveResult,
    GraspResult,
    EnableResult,
    GripperActions,
    limit_target,
    press_target,
)

# ── Data models ─────────────────────────────────────────────────────────
from .models import (
    GripperState,
    GripperConfig,
    GripperInfo,
    GripperStatus,
    GripperMode,
    CalibrationData,
)

# ── Constants & enums ───────────────────────────────────────────────────
from .constants import (
    GripperParams,
    UnitConversion,
    ErrorCode,
    DefaultParams,
    describe_error,
    DM_Motor_Type,
    Control_Mode,
)

# ── Exceptions ──────────────────────────────────────────────────────────
from .exceptions import (
    LiteGripError,
    CommError,
    ConnectError,
    CommandError,
    CANTimeoutError,
    HardwareError,
    NotInitializedError,
)

# ── Teleoperation (leader / follower) ───────────────────────────────────
from .teleop import (
    GripperTeleop,
    TeleopTransport,
    TeleopSubscription,
    UdpTeleopTransport,
    InProcTeleopTransport,
    TeleopError,
    TeleopBusyError,
    TeleopNotActiveError,
    TeleopNotReady,
    check_ready,
    clamp_to_calibrated,
    DEFAULT_GRIP_ID,
    DEFAULT_GRIP_PORT,
    DEFAULT_DQ_MAX,
    DEFAULT_TORQUE_LIMIT_NM,
    FRAME_SIZE,
    encode_frame,
    decode_frame,
    teleop_topic,
)

# ── Trajectory record and replay ────────────────────────────────────────
from .trajectory import (
    Trajectory,
    TrajectorySample,
    TrajectoryRecorder,
    TrajectoryPlayer,
    trajectory_dir,
    resolve_path,
    DEFAULT_RATE_HZ,
    TrajectoryError,
    TrajectoryBusyError,
    TrajectoryNotActiveError,
    TrajectoryEmptyError,
    TrajectoryRecordingError,
    TrajectoryFormatError,
)

# ── CAN subpackage (expert) ─────────────────────────────────────────────
from . import can


# The zenoh link needs the optional ``zenoh`` dependency, so it is resolved on
# first access rather than at import time: ``import litegrip`` must work on a
# bare robot controller that will never teleoperate.  Install the extra with
# ``pip install litegrip[zenoh]``.
_ZENOH_EXPORTS = frozenset(
    {"ZenohTeleopTransport", "Listener", "Connector", "LatestSlot"})


def __getattr__(name: str):
    if name in _ZENOH_EXPORTS:
        try:
            from . import zenoh_link
        except ImportError as e:
            raise ImportError(
                f"litegrip.{name} needs the optional zenoh dependency — install "
                "it with `pip install litegrip[zenoh]`") from e
        return getattr(zenoh_link, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__():
    return sorted(set(globals()) | _ZENOH_EXPORTS)

__all__ = [
    "__version__",
    # High-level
    "LiteGrip",
    "DEFAULT_CALIB",
    "CALIB_TEMPLATES",
    "default_calib_path",
    "list_templates",
    # Motion actions
    "MotionConfig",
    "MoveProgress",
    "MoveResult",
    "GraspResult",
    "EnableResult",
    "GripperActions",
    "limit_target",
    "press_target",
    # Models
    "GripperState",
    "GripperConfig",
    "GripperInfo",
    "GripperStatus",
    "GripperMode",
    "CalibrationData",
    # Constants
    "GripperParams",
    "UnitConversion",
    "ErrorCode",
    "DefaultParams",
    "describe_error",
    "DM_Motor_Type",
    "Control_Mode",
    # Exceptions
    "LiteGripError",
    "CommError",
    "ConnectError",
    "CommandError",
    "CANTimeoutError",
    "HardwareError",
    "NotInitializedError",
    # Teleoperation
    "GripperTeleop",
    "TeleopTransport",
    "TeleopSubscription",
    "UdpTeleopTransport",
    "InProcTeleopTransport",
    "TeleopError",
    "TeleopBusyError",
    "TeleopNotActiveError",
    "TeleopNotReady",
    "check_ready",
    "clamp_to_calibrated",
    "DEFAULT_GRIP_ID",
    "DEFAULT_GRIP_PORT",
    "DEFAULT_DQ_MAX",
    "DEFAULT_TORQUE_LIMIT_NM",
    "FRAME_SIZE",
    "encode_frame",
    "decode_frame",
    "teleop_topic",
    # Teleoperation — zenoh link (resolved lazily; needs litegrip[zenoh])
    "ZenohTeleopTransport",
    "Listener",
    "Connector",
    "LatestSlot",
    # Trajectory record and replay
    "Trajectory",
    "TrajectorySample",
    "TrajectoryRecorder",
    "TrajectoryPlayer",
    "trajectory_dir",
    "resolve_path",
    "DEFAULT_RATE_HZ",
    "TrajectoryError",
    "TrajectoryBusyError",
    "TrajectoryNotActiveError",
    "TrajectoryEmptyError",
    "TrajectoryRecordingError",
    "TrajectoryFormatError",
    # Subpackages
    "can",
]
