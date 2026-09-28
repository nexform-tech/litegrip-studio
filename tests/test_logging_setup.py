"""The log file, which is read after something has gone wrong.

What is pinned here is the part that is easy to get wrong and impossible to
notice: that a record written by any module reaches the file, that the console's
own modules keep their detail while Qt's chatter is dropped, that the whole thing
survives a home directory it cannot write to, and that a traceback which Qt
would otherwise swallow ends up in the file.
"""

from __future__ import annotations

import logging
import sys

import pytest

from litegrip_studio import logging_setup


def reset() -> None:
    """Tear the global logging stack down the way the next test needs it.

    The console reconfigures itself once, so a test that wants to watch it
    configure has to clear the flag that says it already did — otherwise
    ``setup`` returns early and the test passes without having run anything.
    """
    root = logging.getLogger()
    for handler in list(root.handlers):
        root.removeHandler(handler)
        handler.close()
    logging_setup._configured = None


@pytest.fixture
def log_file(tmp_path, monkeypatch):
    """A configured logging stack writing into the test's own directory.

    Both handlers are torn down afterwards: the root logger is global, and a
    test that leaves a ``RotatingFileHandler`` open on a temporary directory
    makes every later test in the session write into a deleted file.
    """
    path = tmp_path / "logs" / "litegrip-studio.log"
    monkeypatch.setattr(logging_setup, "_configured", None)
    written = logging_setup.setup(path=path, console=False)
    try:
        yield written
    finally:
        reset()


def lines_in(path) -> str:
    return path.read_text(encoding="utf-8")


class TestTheFile:
    def test_it_creates_the_directory_it_needs(self, log_file) -> None:
        """``~/.litegrip/logs`` does not exist on a fresh machine."""
        assert log_file is not None
        assert log_file.parent.is_dir()

    def test_a_record_from_any_module_reaches_it(self, log_file) -> None:
        logging.getLogger("litegrip_studio.core.worker").info("控制线程已启动")

        assert "控制线程已启动" in lines_in(log_file)

    def test_debug_detail_is_kept_in_the_file(self, log_file) -> None:
        """The file is for forensics: the level that matters is the one that
        says what the loop was doing, which is DEBUG."""
        logging.getLogger("litegrip_studio.core.worker").debug("tick 12.4 ms")

        assert "tick 12.4 ms" in lines_in(log_file)

    def test_the_console_shows_info_and_stays_quiet_about_debug(self, log_file, capsys) -> None:
        reset()
        logging_setup.setup(path=log_file, console=True)
        log = logging.getLogger("litegrip_studio.core.worker")
        log.debug("noise")
        log.info("控制线程已启动")

        captured = capsys.readouterr()
        assert "控制线程已启动" in captured.err, "the console handler was never installed"
        assert "noise" not in captured.err

    def test_qt_is_kept_out_of_the_file_below_warning(self, log_file) -> None:
        """Qt logs at DEBUG constantly about things that are nobody's problem."""
        logging.getLogger("PyQt5.QtCore").debug("QStandardPaths: XDG_RUNTIME_DIR not set")

        assert "XDG_RUNTIME_DIR" not in lines_in(log_file)

    def test_it_rotates_rather_than_growing_without_bound(self, log_file) -> None:
        log = logging.getLogger("litegrip_studio")
        payload = "x" * 1024
        for _ in range(600):
            log.debug(payload)

        backups = list(log_file.parent.glob("litegrip-studio.log.*"))
        assert backups, "the file must rotate under a console left running for days"
        assert log_file.stat().st_size <= logging_setup.MAX_BYTES

    def test_configuring_twice_does_not_double_every_line(self, log_file) -> None:
        reset()
        logging_setup.setup(path=log_file, console=False)
        handlers = len(logging.getLogger().handlers)
        logging_setup.setup(path=log_file, console=False)

        assert len(logging.getLogger().handlers) == handlers, "handlers were added twice"
        logging.getLogger("litegrip_studio.core.worker").info("only once")
        assert lines_in(log_file).count("only once") == 1


class TestItIsNeverFatal:
    def test_an_unwritable_home_directory_does_not_stop_the_console(self, tmp_path) -> None:
        reset()
        target = tmp_path / "logs" / "x.log"
        target.parent.write_text("not a directory")
        try:
            assert logging_setup.setup(path=target, console=False) is None
            logging.getLogger("litegrip_studio").info("still running")  # must not raise
        finally:
            reset()


class TestLevels:
    @pytest.mark.parametrize(
        "given, expected",
        [("DEBUG", logging.DEBUG), ("warning", logging.WARNING), (logging.ERROR, logging.ERROR)],
    )
    def test_a_level_is_accepted_by_name_or_number(self, given, expected) -> None:
        assert logging_setup._level_of(given, logging.INFO) == expected

    @pytest.mark.parametrize("junk", ["", None, "LOUD", 1.5])
    def test_a_level_that_is_not_one_falls_back(self, junk) -> None:
        assert logging_setup._level_of(junk, logging.INFO) == logging.INFO


class TestTheExcepthook:
    def test_a_traceback_reaches_the_file(self, log_file) -> None:
        """A frozen build has no terminal, so an unlogged traceback is gone."""
        original = sys.excepthook
        try:
            logging_setup.install_excepthook()
            try:
                raise RuntimeError("控制线程崩了")
            except RuntimeError:
                kind, value, tb = sys.exc_info()
                sys.excepthook(kind, value, tb)
        finally:
            sys.excepthook = original

        text = lines_in(log_file)
        assert "未捕获的异常" in text
        assert "控制线程崩了" in text

    def test_it_still_calls_the_previous_hook(self, log_file) -> None:
        seen = []
        original = sys.excepthook
        sys.excepthook = lambda *args: seen.append(args)
        try:
            logging_setup.install_excepthook()
            sys.excepthook(RuntimeError, RuntimeError("x"), None)
        finally:
            sys.excepthook = original

        assert seen, "adding a copy must not take the hook over"

    def test_a_keyboard_interrupt_is_left_alone(self, log_file) -> None:
        """Ctrl-C is how the operator stops the console, not a crash; it goes
        to the previous hook and not to the log as a critical error."""
        original = sys.excepthook
        try:
            logging_setup.install_excepthook()
            sys.excepthook(KeyboardInterrupt, KeyboardInterrupt(), None)
        finally:
            sys.excepthook = original

        assert "未捕获的异常" not in lines_in(log_file)
