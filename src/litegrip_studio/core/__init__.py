"""Pure control logic: trajectory generation and the control state machines.

No module here imports the SDK or Qt, and each is a function of the values
passed to it, which is what makes the whole control layer testable without
hardware — and what lets the simulator exercise the same code the real backend
runs, rather than a parallel implementation of it.

The one qualified exception is :mod:`~litegrip_studio.core.worker`, which does
import Qt: it is where the console crosses from logic into a thread, and Qt is
what the thread is.  The Qt-free part of it, :class:`WorkerLoop`, holds the
entire loop, so even that module's behaviour is tested without a QApplication.
"""
