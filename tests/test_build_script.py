"""``build.sh`` has to leave the source tree exactly as it found it.

Both claims here were defects that actually happened, and both are about cleanup
that silently did not run:

* The version stamp stayed behind, so every later *source* run reported itself as
  the previous build — with a git hash that need not describe the working tree
  any more.  That is a version reading that lies, and it is the same invariant
  ``test_cli.TestTheVersion`` pins from the other side.
* The entry file's temp directory stayed behind.  Less serious, but it
  accumulates one directory per build.

The mechanism that broke them is worth naming, because it is not obvious: the
script used ``exec`` for the PyInstaller step, and ``exec`` *replaces* the shell
process, so the ``trap ... EXIT`` that does the cleanup never ran.  These are
behavioural tests rather than a scan of the script's text, so any rewrite that
still cleans up passes.
"""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

STUB_OK = "#!/bin/sh\nexit 0\n"


def make_tree(tmp_path: Path, packager: str = STUB_OK) -> Path:
    """A throwaway copy of the script, with a PyInstaller that does nothing.

    ``PYTHON_BIN`` is the script's own override hook, so pointing it at a stub
    exercises the real control flow — including the trap — in milliseconds,
    without touching the actual packaging or the real source tree.  The stamp is
    written into the copy because ``build.sh`` resolves its paths from its own
    location, not from the caller's directory.

    ``version.py`` has to be present even though the stub cannot read it: the
    script runs under ``pipefail``, so a missing file makes ``sed`` fail and the
    script abort before it has written anything — and a test that stops before
    the interesting line proves nothing about the line.
    """
    package = tmp_path / "src/litegrip_studio"
    package.mkdir(parents=True)
    (package / "version.py").write_text(
        'BASE_VERSION = "0.1.0"\n', encoding="utf-8"
    )
    shutil.copy(REPO_ROOT / "build.sh", tmp_path / "build.sh")

    stub = tmp_path / "python3"
    stub.write_text(packager, encoding="utf-8")
    stub.chmod(0o755)
    return stub


def run_build(tmp_path: Path, packager: str = STUB_OK) -> subprocess.CompletedProcess:
    stub = make_tree(tmp_path, packager)
    scratch = tmp_path / "tmp"
    scratch.mkdir()

    return subprocess.run(
        ["bash", "build.sh"],
        cwd=tmp_path,
        env={**os.environ, "PYTHON_BIN": str(stub), "TMPDIR": str(scratch)},
        capture_output=True,
        text=True,
    )


class TestItCleansUpAfterItself:
    def test_the_version_stamp_does_not_survive_the_build(self, tmp_path) -> None:
        result = run_build(tmp_path)

        assert result.returncode == 0, result.stderr
        assert not (tmp_path / "src/litegrip_studio/_version.py").exists()

    def test_the_entry_files_directory_does_not_survive_either(self, tmp_path) -> None:
        result = run_build(tmp_path)

        assert result.returncode == 0, result.stderr
        assert list((tmp_path / "tmp").iterdir()) == []

    def test_the_stamp_was_written_before_being_removed(self, tmp_path) -> None:
        """Otherwise the two tests above would pass on a script that never wrote
        one — the cleanup would be vacuously satisfied."""
        result = run_build(tmp_path)

        assert "版本号：" in result.stdout


class TestItFailsLoudly:
    def test_a_failing_packager_fails_the_build_and_still_cleans_up(
        self, tmp_path
    ) -> None:
        result = run_build(tmp_path, packager="#!/bin/sh\nexit 3\n")

        assert result.returncode == 3
        assert not (tmp_path / "src/litegrip_studio/_version.py").exists()
