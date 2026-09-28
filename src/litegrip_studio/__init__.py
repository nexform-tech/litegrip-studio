"""LiteGrip Console — a PyQt5 host application for the LiteGrip gripper.

Layered so that the parts worth testing do not need Qt, a CAN bus, or the SDK:

``constants``, ``units``, ``calibration``
    Pure policy and arithmetic.  No imports beyond the standard library.
``core``
    The trajectory, the control state machine and the calibration state
    machines.  Pure; no threads, no Qt, no I/O.
``backend``
    The only layer that touches hardware.  ``plant`` is a pure dynamics model,
    ``sim`` puts a real-time clock and fault injection behind it, ``real``
    wraps the SDK.
``ui``
    Qt.  Thin by design — it renders telemetry and posts commands, and makes no
    safety decisions of its own.
"""

from __future__ import annotations

__all__ = ["__version__"]

# Replaced at build time; the git tag is the real source of truth.
__version__ = "0.0.0-semantic-release"
