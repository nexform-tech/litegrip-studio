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
import re
import shutil
import subprocess
from pathlib import Path

from litegrip_studio import version

REPO_ROOT = Path(__file__).resolve().parent.parent

STUB_OK = "#!/bin/sh\nexit 0\n"

#: Git reads these to make a commit in the throwaway tree, which has no config of
#: its own and must not depend on the machine's.
GIT_IDENTITY = {
    "GIT_AUTHOR_NAME": "build test",
    "GIT_AUTHOR_EMAIL": "build@example.invalid",
    "GIT_COMMITTER_NAME": "build test",
    "GIT_COMMITTER_EMAIL": "build@example.invalid",
}


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
    the interesting line proves nothing about the line.  The value is read from
    the real module rather than copied, so a test asserting on the fallback
    cannot pass against a fixture that has drifted from what ships.
    """
    package = tmp_path / "src/litegrip_studio"
    package.mkdir(parents=True)
    (package / "version.py").write_text(
        f'BASE_VERSION = "{version.BASE_VERSION}"\n', encoding="utf-8"
    )
    shutil.copy(REPO_ROOT / "build.sh", tmp_path / "build.sh")

    stub = tmp_path / "python3"
    stub.write_text(packager, encoding="utf-8")
    stub.chmod(0o755)
    return stub


def init_git_repo(tmp_path: Path, tag: str | None) -> None:
    """Make the throwaway tree a repository, optionally with one ``v*`` tag."""

    def git(*args: str) -> None:
        subprocess.run(
            ["git", *args],
            cwd=tmp_path,
            env={**os.environ, **GIT_IDENTITY},
            check=True,
            capture_output=True,
        )

    git("init", "-q")
    git("add", "-A")
    git("commit", "-q", "-m", "tree")
    if tag is not None:
        git("tag", tag)


def run_build(
    tmp_path: Path,
    packager: str = STUB_OK,
    *,
    git_repo: bool = False,
    tag: str | None = None,
) -> subprocess.CompletedProcess:
    stub = make_tree(tmp_path, packager)
    if git_repo:
        init_git_repo(tmp_path, tag)
    scratch = tmp_path / "tmp"
    scratch.mkdir()

    # The stamp is removed again before the script exits, so the only way to read
    # it is out of the line the script prints.
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


class TestTheNumberItStamps:
    """What the artifact calls itself, which is the first question asked of a
    binary that behaved oddly on somebody else's machine."""

    def test_the_base_is_the_nearest_release_tag(self, tmp_path) -> None:
        """An artifact built from a tagged tree says which release it came from.

        It used to say which *first* release it came from, always: the base was
        ``version.py``'s constant, so everything this script ever built claimed
        ``0.1.0`` no matter how far past it the tree was.
        """
        result = run_build(tmp_path, git_repo=True, tag="v0.9.3")

        assert result.returncode == 0, result.stderr
        assert "版本号：0.9.3." in result.stdout, result.stdout
        assert re.search(r"版本号：0\.9\.3\.\d+\+g[0-9a-f]+", result.stdout), result.stdout

    def test_the_tag_is_read_without_its_leading_v(self, tmp_path) -> None:
        """``v0.9.3`` is the tag; ``0.9.3`` is the version. Mixing them gives a
        string no version comparison has ever seen."""
        result = run_build(tmp_path, git_repo=True, tag="v0.9.3")

        assert "版本号：v" not in result.stdout

    def test_a_repository_with_no_tag_falls_back_to_version_py(self, tmp_path) -> None:
        """A shallow export or a source tarball has no tag to ask."""
        result = run_build(tmp_path, git_repo=True, tag=None)

        assert result.returncode == 0, result.stderr
        assert f"版本号：{version.BASE_VERSION}." in result.stdout, result.stdout

    def test_a_tree_with_no_git_at_all_still_builds(self, tmp_path) -> None:
        """``git describe`` fails outside a repository and the script runs under
        ``pipefail``, so without the guard the build would abort at the version
        line rather than fall back to it."""
        assert not (tmp_path / ".git").exists()

        result = run_build(tmp_path)

        assert result.returncode == 0, result.stderr
        assert f"版本号：{version.BASE_VERSION}." in result.stdout, result.stdout
