"""The LiteGrip SDK that ships inside this repository.

The console imports ``litegrip``.  It used to have to be checked out beside this
repository; it is now vendored under ``src/litegrip`` (provenance in
``src/litegrip/VENDORED.md``) so that a clone alone runs.

Two kinds of claim are pinned here, and they fail in different ways.  The first
is that a machine with this checkout and nothing else can import the SDK at all —
those tests run a child interpreter with the environment and ``site-packages``
stripped, because in this process the SDK is already imported and would answer
for itself.  The second is that the vendored tree is still the tree the note
describes: an edit in place is invisible until someone re-vendors over it, and
the hash is what makes it visible now.
"""

from __future__ import annotations

import hashlib
import os
import re
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"
VENDORED_SDK = SRC / "litegrip"
PROVENANCE = VENDORED_SDK / "VENDORED.md"

UPSTREAM = "https://github.com/nexform-tech/litegrip-python"


def clean_env(**overrides: str) -> dict[str, str]:
    """The environment of a machine that has this checkout and nothing else.

    Every ``LITEGRIP_*`` variable goes, not just the one this file is about: a
    developer who exported one in their shell is testing against their own SDK,
    and these tests are the ones that must not.
    """
    env = {k: v for k, v in os.environ.items() if not k.startswith("LITEGRIP_")}
    env.pop("PYTHONPATH", None)
    env.update(overrides)
    return env


def run_python(source: str, *, env: dict[str, str], flags: tuple[str, ...] = ()) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, *flags, "-c", source],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
    )


def tree_hash(root: Path) -> str:
    """The vendored tree's hash, by the recipe written in ``VENDORED.md``.

    Kept in step with that file deliberately: the point of the hash is that it
    can be recomputed by someone who has only the note, so the note documents the
    recipe rather than pointing at this function.
    """
    digest = hashlib.sha256()
    for path in sorted(root.rglob("*")):
        if path.is_file() and "__pycache__" not in path.parts and path.name != "VENDORED.md":
            digest.update((path.relative_to(root).as_posix() + "\n").encode())
            digest.update(path.read_bytes())
    return digest.hexdigest()


class TestAMachineWithNothingButThisCheckout:
    def test_it_imports_the_vendored_sdk(self) -> None:
        result = run_python(
            f"import sys; sys.path.insert(0, {str(SRC)!r});"
            " import litegrip; print(litegrip.__file__)",
            env=clean_env(),
            # -S as well as -I: with site-packages in play, an *installed*
            # litegrip can answer this import, and then the test says nothing
            # about a machine that has installed nothing.
            flags=("-I", "-S"),
        )

        assert result.returncode == 0, result.stderr
        assert Path(result.stdout.strip()).is_relative_to(VENDORED_SDK)

    def test_the_real_backend_imports_through_it(self) -> None:
        """The console's own import of the SDK, in a process that starts clean."""
        result = run_python(
            "from litegrip_studio import cli;"
            " assert cli.ensure_sdk(), 'no SDK';"
            " import litegrip_studio.backend.real as real;"
            " print(real.LiteGrip.__module__)",
            env=clean_env(PYTHONPATH=str(SRC)),
        )

        assert result.returncode == 0, result.stderr
        assert result.stdout.strip().startswith("litegrip")

    def test_the_launcher_runs_with_no_sdk_environment(self) -> None:
        """``run_litegrip_studio.sh`` is the documented way in, so it is the
        thing that has to work on a fresh clone.  ``selftest`` is the one
        subcommand that needs no hardware and no display."""
        result = subprocess.run(
            ["bash", "run_litegrip_studio.sh", "selftest"],
            cwd=REPO_ROOT,
            env=clean_env(PYTHON_BIN=sys.executable),
            capture_output=True,
            text=True,
        )

        assert result.returncode == 0, result.stdout + result.stderr
        assert "FAIL" not in result.stdout

    def test_an_explicit_sdk_path_still_wins(self, tmp_path) -> None:
        """The override is what a developer testing against SDK HEAD uses, so it
        has to beat the copy this repository carries — not merely be on the path."""
        elsewhere = tmp_path / "sdk-checkout"
        (elsewhere / "litegrip").mkdir(parents=True)
        (elsewhere / "litegrip" / "__init__.py").write_text(
            "MARKER = 'not the vendored one'\n", encoding="utf-8"
        )

        result = run_python(
            "from litegrip_studio import cli;"
            " assert cli.ensure_sdk(), 'no SDK';"
            " import litegrip; print(litegrip.__file__)",
            env=clean_env(PYTHONPATH=str(SRC), LITEGRIP_SDK_PATH=str(elsewhere)),
        )

        assert result.returncode == 0, result.stderr
        assert Path(result.stdout.strip()).is_relative_to(elsewhere)

    @pytest.mark.parametrize("inherited", ("nothing", "src-first"))
    def test_the_override_also_wins_over_the_bootstrap(
        self, tmp_path, inherited: str
    ) -> None:
        """The same claim for ``conftest.py``, which is the *other* thing that
        decides which ``litegrip`` this suite imports.

        It cannot be observed from in here: by the time a test body runs, both
        directories are on ``sys.path`` and the import has happened.  So the
        child imports ``conftest`` itself, from a process that starts in the same
        position a bare ``pytest`` is in.

        ``src-first`` is the case a "not already on the path" guard gets wrong:
        with an inherited ``PYTHONPATH`` naming the vendored tree before the
        checkout, skipping both leaves the vendored copy in front and the
        override silently idle.
        """
        elsewhere = tmp_path / "sdk-checkout"
        (elsewhere / "litegrip").mkdir(parents=True)
        (elsewhere / "litegrip" / "__init__.py").write_text(
            "MARKER = 'not the vendored one'\n", encoding="utf-8"
        )

        env = clean_env(LITEGRIP_SDK_PATH=str(elsewhere))
        if inherited == "src-first":
            env["PYTHONPATH"] = f"{SRC}{os.pathsep}{elsewhere}"

        result = run_python(
            "import conftest, litegrip; print(litegrip.__file__)",
            env=env,
        )

        assert result.returncode == 0, result.stderr
        assert Path(result.stdout.strip()).is_relative_to(elsewhere), inherited


class TestItStaysWhatItSaysItIs:
    def test_the_note_names_the_upstream_repository_and_commit(self) -> None:
        text = PROVENANCE.read_text(encoding="utf-8")

        assert UPSTREAM in text
        assert re.search(r"\b[0-9a-f]{40}\b", text), "no commit sha in the note"

    def test_the_tree_still_matches_the_recorded_hash(self) -> None:
        # The note records one hash, and covers the whole tree including the
        # calibration data.  A second one appearing means a local override was
        # declared; the next test is about that.
        text = PROVENANCE.read_text(encoding="utf-8")
        recorded = re.search(r"^([0-9a-f]{64})$", text, re.MULTILINE)

        assert recorded, "the note records no tree hash"
        assert tree_hash(VENDORED_SDK) == recorded.group(1), (
            "src/litegrip 已经被就地改动，与 VENDORED.md 记的哈希对不上了："
            "要么改回去，要么按 VENDORED.md 的步骤重新 vendor 并更新哈希"
        )

    def test_the_note_declares_no_local_override(self) -> None:
        """The tree is upstream's byte for byte, and this is what says so.

        The note used to record a second hash: the factory calibration was taken
        from litegrip-cpp instead of from the vendored commit, because the two
        upstream repositories disagreed about those numbers.  They agree now
        (upstream `b9caae8` re-measured the shipped files for this unit), so
        there is no override left and the tree hash is the whole guarantee.

        A second hash coming back means somebody took a divergence again.  That
        is allowed — it is what the note is for — but it needs a test of its own
        saying which file it is and why, not merely a number in the note.
        """
        hashes = re.findall(
            r"^([0-9a-f]{64})$", PROVENANCE.read_text(encoding="utf-8"), re.MULTILINE
        )

        assert len(hashes) == 1, (
            "VENDORED.md 记了不止一个哈希：要么本地又分叉了（那要补一条按它自己"
            "哈希校验的测试，并在「No local override」那节写清原因），要么这一节该更新"
        )

    def test_the_data_files_it_resolves_by_name_travel_with_it(self) -> None:
        """``load_calibration(template=...)`` builds these paths from
        ``dirname(__file__)``, so they are code, not samples: absent from the
        wheel or the bundle, the public call fails at the bench."""
        import litegrip

        for name, path in litegrip.CALIB_TEMPLATES.items():
            assert Path(path).is_file(), f"{name} → {path} is missing"
            assert Path(path).is_relative_to(VENDORED_SDK)

        factory = Path(litegrip.__file__).resolve().parent / "factory_calibration.json"
        assert factory.is_file(), "the SDK's factory calibration is missing"
