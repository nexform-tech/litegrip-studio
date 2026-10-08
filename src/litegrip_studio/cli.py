"""The command line: pick a backend, set up logging, open the window.

Qt is imported inside :func:`main` and not at module scope, because
``--selftest`` has to work on a machine where Qt does not — which is the machine
most likely to be running it.

One decision worth stating here, because it looks like an omission: the backend
is built with **no** calibration applied.  Loading one is a command the worker
runs when it connects, on purpose, so that the provenance check and the gate
always describe the file that was actually handed to the SDK rather than
whatever a command line said at startup.  The flags below only say *which file*
that will be, and how wide this gripper is.
"""

from __future__ import annotations

import argparse
import logging
import os
import signal
import sys
from pathlib import Path

from . import constants, logging_setup, version

BACKENDS = ("sim", "real")

log = logging.getLogger(__name__)


def build_parser(release: str | None = None) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="litegrip-studio",
        description="LiteGrip 夹爪控制台",
    )
    parser.add_argument(
        "--version", action="version",
        version=f"litegrip-studio {release or version.resolve_version()}",
        help="打印版本号后退出",
    )
    parser.add_argument(
        "--backend", choices=BACKENDS, default="real",
        help="sim = 内置仿真被控对象（不需要硬件）；real = 通过 SocketCAN 驱动真机",
    )
    parser.add_argument(
        "--can-channel", default=constants.CAN_CHANNEL, help="SocketCAN 接口名"
    )
    parser.add_argument(
        "--can-bitrate", type=int, default=constants.CAN_BITRATE,
        help="连接时把接口配成这个比特率（仅在连接前发现它不对时才动手）",
    )
    parser.add_argument(
        "--no-can-setup", action="store_true",
        help="不自动准备 CAN 口：接口由你自己管，连接时不再弹授权框",
    )
    parser.add_argument(
        "--can-id", type=int, default=None,
        help="电机 CAN ID；缺省用标定文件里的值",
    )
    parser.add_argument(
        "--mst-id", type=int, default=None,
        help="主机 CAN ID；缺省用标定文件里的值",
    )
    parser.add_argument(
        "--calibration", default=None,
        help="标定文件路径；缺省用上次记住的路径，再回退到 ~/.litegrip 下的默认位置",
    )
    parser.add_argument(
        "--log-level", default="INFO",
        help="控制台输出的日志级别；文件里始终记录 DEBUG",
    )
    parser.add_argument("--log-file", default=None, help="日志文件路径")
    parser.add_argument(
        "--selftest", action="store_true",
        help="只跑不需要 Qt、不需要硬件的自检，然后退出",
    )
    return parser


def ensure_sdk() -> bool:
    """Make ``import litegrip`` work, or say why it did not.

    The SDK is vendored under ``src/litegrip`` and travels with this package, so
    normally nothing has to happen here: it is importable already.  It is
    deliberately not a resolved dependency — its one declared dependency is
    never imported by its library code — so there is no distribution to install.

    ``LITEGRIP_SDK_PATH`` points at a different checkout instead, which is how a
    developer tests against SDK HEAD.  A path set on purpose and then found
    unusable is reported rather than quietly falling back to the vendored copy:
    the operator asked for that one.
    """
    override = os.environ.get("LITEGRIP_SDK_PATH")
    if override:
        path = _sdk_checkout(Path(override).expanduser())
        if path is None:
            print(
                f"LITEGRIP_SDK_PATH 指到的地方没有 litegrip 包：{override}\n"
                f"它要么是 SDK 检出的根目录，要么是那个检出里的 src/。",
                file=sys.stderr,
            )
            return False
        if str(path) not in sys.path:
            sys.path.insert(0, str(path))

    try:
        import litegrip  # noqa: F401
    except ImportError as exc:
        print(
            f"找不到 litegrip SDK：{exc}\n"
            f"SDK 随本仓库一起发在 src/litegrip，跑 run_litegrip_studio.sh 即可；"
            f"也可用 LITEGRIP_SDK_PATH 指到另一份检出，或用 --backend sim 跑仿真。",
            file=sys.stderr,
        )
        return False
    return True


def _sdk_checkout(root: Path) -> Path | None:
    """``root`` or ``root/src``, whichever holds the ``litegrip`` package.

    Two layouts exist and both are worth accepting without a second guess: a
    checkout is sometimes the package itself (``<repo>/litegrip/``), sometimes a
    source tree with it one level down (``<repo>/src/litegrip/``).  ``None`` when
    neither holds one.
    """
    for candidate in (root, root / "src"):
        if (candidate / "litegrip" / "__init__.py").is_file():
            return candidate
    return None


def make_backend(args):
    """Build the backend the arguments ask for.

    Imported here rather than at module scope so that ``--selftest`` never
    touches the SDK.
    """
    try:
        if args.backend == "sim":
            from .backend.sim import SimBackend

            backend = SimBackend(calibration_path=args.calibration, realtime=True)
        else:
            if not ensure_sdk():
                raise SystemExit(2)
            from .backend.real import RealBackend

            backend = RealBackend(
                channel=args.can_channel,
                can_id=args.can_id,
                mst_id=args.mst_id,
                calibration_path=args.calibration,
            )
    except ImportError as exc:
        print(f"无法载入 {args.backend} 后端：{exc}", file=sys.stderr)
        raise SystemExit(2) from exc

    return backend


def make_can_link(args):
    """The interface preparation the worker runs before it connects, if any.

    Wired here rather than inside the worker for the same reason the backend is:
    which bus this console is talking to is decided in exactly one place, and the
    worker stays a thing that talks to whatever backend it was handed.  The
    simulator gets ``None`` — there is no interface behind it, and asking for a
    password to configure a bus that does not exist would be theatre.
    """
    if args.backend != "real" or args.no_can_setup:
        return None
    from .can_link import CanLink

    return CanLink(args.can_channel, args.can_bitrate)


def resolve_calibration_path(args, settings) -> str | None:
    """Which file the worker should load when it connects, if any.

    The remembered path is the *bench gripper's* file, so it is only honoured for
    the real backend.  Applying it to the simulator would be harmless; saving one
    from a simulator run would not — the simulator keeps its calibration in a file
    of its own precisely so that it can never overwrite the one that cannot be
    regenerated.  Keeping the two separate at the entrance is what makes that
    guarantee hold from both directions.
    """
    if args.calibration is not None:
        return args.calibration
    if args.backend == "real":
        return settings.calibration_path
    return None


def quit_handler(app):
    """The signal handler, named so a test can call it without a real signal."""
    def handler(_signum, _frame) -> None:
        app.quit()

    return handler


def _keep_the_interpreter_awake(app) -> None:
    """Give Python a reason to run between signals.

    A signal handler only runs when the interpreter is executing bytecode, and a
    Qt event loop waiting for input is not; without a timer firing periodically
    a Ctrl+C would sit unheard until something else woke the process.
    """
    from PyQt5.QtCore import QTimer

    ticker = QTimer(app)
    ticker.start(int(constants.HEARTBEAT_INTERVAL_MS))
    ticker.timeout.connect(lambda: None)


def install_signal_handlers(app) -> None:
    """Make Ctrl+C and SIGTERM close the window rather than kill the process.

    Installed on the main thread, which is the only thread Python delivers
    signals to — the worker cannot receive one at all, which is why the console
    stops the motor itself instead of relying on KeyboardInterrupt.
    """
    handler = quit_handler(app)
    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)
    _keep_the_interpreter_awake(app)


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    if args.selftest:
        from . import selftest

        return selftest.run()

    log_path = logging_setup.setup(level=args.log_level, path=args.log_file)
    log.info("控制台启动：版本 %s，backend=%s", version.resolve_version(), args.backend)
    log.info("日志文件：%s", log_path if log_path is not None else "（无法写入，只有控制台）")

    from PyQt5.QtWidgets import QApplication

    from .core.worker import GripperWorker
    from .settings import Settings
    from .ui import theme
    from .ui.main_window import MainWindow

    app = QApplication(sys.argv[:1])
    app.setApplicationName("litegrip-studio")
    app.setOrganizationName("litearm")

    settings = Settings()
    # The palette is resolved before anything is built: a widget that painted
    # itself during construction would otherwise keep the theme the process
    # happened to start in.  Both calls, because they answer different
    # questions — ``set_theme`` says which palette is in force (and a console
    # opening in its own default theme changes nothing), while ``apply`` is
    # what hands it to the application, which a fresh process has never had.
    theme.set_theme(theme.resolve(settings.theme))
    theme.apply(app)

    logging_setup.install_qt_message_handler()
    logging_setup.install_excepthook()
    logging_setup.enable_faulthandler()

    args.calibration = resolve_calibration_path(args, settings)

    backend = make_backend(args)
    log.info("后端：%s", backend.describe())

    worker = GripperWorker(backend, can_link=make_can_link(args))

    window = MainWindow(worker, settings)
    window.start()

    # Two layers, and the order matters: the window's closeEvent stops the worker
    # while the event loop is still alive to report a failure, and this catches a
    # quit that did not come through the window (a signal, or the last window
    # closing).  The worker's own ``finally`` is the third.
    app.aboutToQuit.connect(worker.shutdown)
    install_signal_handlers(app)

    worker.start()
    window.show()

    exit_code = app.exec_()
    log.info("控制台退出，返回码 %s", exit_code)
    return int(exit_code)


if __name__ == "__main__":  # pragma: no cover - exercised through __main__
    raise SystemExit(main())
