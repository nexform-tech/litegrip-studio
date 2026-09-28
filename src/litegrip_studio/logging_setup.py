"""Where the console's output goes, and what catches the output nobody wrote.

Three sources of text end up in one place:

* the console's own loggers,
* Qt, which writes warnings to stderr and nowhere else unless a message handler
  is installed — and the warnings that matter here ("QBackingStore::endPaint()
  called with a painter that has not been ended") are the ones that precede a
  widget going wrong,
* the last words of a process that dies, which is what :mod:`faulthandler`
  writes.

They share a file because the point of it is that after something has gone
wrong, one file explains what the console was doing.  The file rotates at a
small size on purpose: this is meant to be read after an incident, and a console
left running for a week must not have buried the incident under a week of
normal operation.

A missing log file is never fatal.  The console is a control panel, and refusing
to open it because ``~/.litegrip`` could not be written would be a worse failure
than losing the log.
"""

from __future__ import annotations

import faulthandler
import logging
import logging.handlers
import sys
from pathlib import Path

#: Root of the console's own package, which is what gets DEBUG in the file.
PACKAGE_LOGGER = "litegrip_studio"

FILE_LEVEL = logging.DEBUG
CONSOLE_LEVEL = logging.INFO

MAX_BYTES = 512 * 1024
BACKUP_COUNT = 3

_configured: Path | None = None


def default_log_path() -> Path:
    return Path.home() / ".litegrip" / "logs" / "litegrip-studio.log"


def setup(level: str = "INFO", *, path: str | Path | None = None,
          console: bool = True) -> Path | None:
    """Configure logging.  Returns the file written, or None if none could be.

    Idempotent: calling it twice does not double every line.
    """
    global _configured

    target = Path(path) if path is not None else default_log_path()
    if _configured == target:
        return target

    root = logging.getLogger()
    root.setLevel(logging.DEBUG)
    for handler in list(root.handlers):
        root.removeHandler(handler)
        _close(handler)

    formatter = logging.Formatter(
        "%(asctime)s %(levelname)-7s %(name)s: %(message)s", "%Y-%m-%d %H:%M:%S"
    )

    written: Path | None = None
    try:
        target.parent.mkdir(parents=True, exist_ok=True)
        handler = logging.handlers.RotatingFileHandler(
            target, maxBytes=MAX_BYTES, backupCount=BACKUP_COUNT, encoding="utf-8"
        )
        handler.setLevel(FILE_LEVEL)
        handler.setFormatter(formatter)
        root.addHandler(handler)
        written = target
    except OSError as exc:  # pragma: no cover - depends on the filesystem
        logging.getLogger(PACKAGE_LOGGER).warning("无法写入日志文件 %s: %s", target, exc)

    if console and sys.stderr is not None:
        stream = logging.StreamHandler(sys.stderr)
        stream.setLevel(_level_of(level, CONSOLE_LEVEL))
        stream.setFormatter(formatter)
        root.addHandler(stream)

    # Third-party chatter at DEBUG is enormous and says nothing about the
    # gripper; the console's own modules keep their detail.
    logging.getLogger("PyQt5").setLevel(logging.WARNING)
    logging.getLogger(PACKAGE_LOGGER).setLevel(logging.DEBUG)

    _configured = target
    return written


def _level_of(name: str | int | None, fallback: int) -> int:
    """A level given either as its number or as its name, or ``fallback``."""
    if isinstance(name, int):
        return name
    if not name:
        return fallback
    found = getattr(logging, str(name).upper(), None)
    return found if isinstance(found, int) else fallback


def _close(handler: logging.Handler) -> None:
    try:
        handler.close()
    except Exception:  # noqa: BLE001 - closing a handler must never raise
        pass


def install_qt_message_handler() -> None:
    """Send Qt's own messages to :mod:`logging` rather than to stderr.

    Qt's writer is C++: it reaches stderr directly, outside logging's control,
    and on a frozen build stderr may not exist at all.  This routes it in — and
    keeps it out of the console, because Qt's informational output ("QStandardPaths:
    XDG_RUNTIME_DIR not set") is noise in a control panel.
    """
    from PyQt5.QtCore import QtMsgType, qInstallMessageHandler

    log = logging.getLogger(PACKAGE_LOGGER + ".qt")
    levels = {
        QtMsgType.QtDebugMsg: logging.DEBUG,
        QtMsgType.QtInfoMsg: logging.INFO,
        QtMsgType.QtWarningMsg: logging.WARNING,
        QtMsgType.QtCriticalMsg: logging.ERROR,
        QtMsgType.QtFatalMsg: logging.CRITICAL,
    }

    def handler(mode, context, message) -> None:  # pragma: no cover - needs Qt
        level = levels.get(mode, logging.WARNING)
        where = ""
        if context is not None and context.file:
            where = f" ({context.file}:{context.line})"
        log.log(level, "%s%s", message, where)

    qInstallMessageHandler(handler)


def install_excepthook(log: logging.Logger | None = None) -> None:
    """Log what would otherwise be printed to a stderr nobody is reading.

    Qt swallows exceptions raised inside slots on some platforms, and a frozen
    build has no terminal at all, so the traceback has to reach the file or it
    ceases to exist.  The previous hook is still called: this adds a copy, it
    does not take over.
    """
    log = log or logging.getLogger(PACKAGE_LOGGER)
    previous = sys.excepthook

    def hook(kind, value, traceback) -> None:
        if issubclass(kind, KeyboardInterrupt):
            previous(kind, value, traceback)
            return
        log.critical("未捕获的异常", exc_info=(kind, value, traceback))
        previous(kind, value, traceback)

    sys.excepthook = hook


def enable_faulthandler() -> None:
    """Record a C-level crash, which logging cannot see.

    A segfault inside the CAN driver takes the process down with no traceback at
    all; this writes the Python stack of every thread to the log file first.
    Best-effort, and never fatal — the console opening without it is better than
    the console not opening.
    """
    try:
        stream = None
        for handler in logging.getLogger().handlers:
            if isinstance(handler, logging.FileHandler):
                stream = handler.stream
                break
        faulthandler.enable(file=stream if stream is not None else sys.stderr)
    except Exception:  # noqa: BLE001 - pragma: no cover
        pass
