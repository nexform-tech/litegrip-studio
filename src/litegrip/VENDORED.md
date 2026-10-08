# The vendored LiteGrip SDK

This file records where the `litegrip` package in this directory came from and how to move it to a
new upstream commit. Read it before editing anything under `src/litegrip/`.

The console drives the gripper through this SDK. It used to require a second checkout beside this
repository; carrying the SDK in-tree is what makes `./run_litegrip_studio.sh gui` work on a machine
that has nothing but this repository.

## What is vendored, and from where

| | |
| --- | --- |
| Upstream | <https://github.com/nexform-tech/litegrip-python> |
| Commit | `85e619f81b182afeb341747d12051b9dcf2382cb` (2026-09-30) |
| License | Apache-2.0 — `LICENSE` is that commit's file, copied verbatim |

`src/litegrip/` holds the modules, the `can/` and `protocols/` subpackages, `py.typed`, and the
three JSON data files the code resolves through `dirname(__file__)`: `factory_calibration.json`,
`calibration_normal.json`, `calibration_reverse.json`. The last two are not decoration — the public
`load_calibration(template="normal" | "reverse")` resolves them by name (see `gripper.py`'s
`CALIB_TEMPLATES`), so they must travel with the code.

Upstream's `tests/`, `.github/`, `.releaserc.json`, `AGENTS.md`, `README*`, `examples/`,
`pyproject.toml`, and `__pycache__/` are deliberately **not** copied. Its 244 tests are upstream's
suite about upstream's internals; the console pins the part of the SDK surface it actually uses
(`tests/test_vendored_sdk.py`, `tests/test_real_backend_api.py`) and records the tree hash below so
drift cannot go unnoticed.

## Do not edit these files

An edit inside `src/litegrip/` is a fork of upstream that nothing records. It also survives only
until the next re-vendor, which copies each file wholesale and would silently drop it. Fix the SDK
upstream and re-vendor, or — when a local divergence is genuinely unavoidable — write it down in
this file with the reason, so the person re-vendoring sees it.

The tree hash below is what makes a silent edit visible: `tests/test_vendored_sdk.py` recomputes it
and fails when it no longer matches.

## Tree hash

```text
83b4704ea1b4c38e48ef428a69ffb65a8809264d8f5fe8345351d3c9f0461fd3
```

The recipe, so it can be recomputed without reading the test: take every file under `src/litegrip/`
except `VENDORED.md` (this file holds the hash, so it cannot be part of it) and `__pycache__`,
ordered by its POSIX path relative to `src/litegrip/`; feed each path, a newline, then the file's
bytes into one SHA-256 stream; read the hex digest.

```bash
python3 - <<'PY'
import hashlib, pathlib
root = pathlib.Path("src/litegrip")
h = hashlib.sha256()
for p in sorted(root.rglob("*")):
    if p.is_file() and "__pycache__" not in p.parts and p.name != "VENDORED.md":
        h.update((p.relative_to(root).as_posix() + "\n").encode())
        h.update(p.read_bytes())
print(h.hexdigest())
PY
```

## How to re-vendor

Run from this repository's root. Replace `NEW_SHA` with the upstream commit you are moving to, and
check out upstream first.

```bash
git -C ../litegrip-python fetch origin
git -C ../litegrip-python rev-parse origin/main          # the commit to record above
git -C ../litegrip-python archive --format=tar NEW_SHA src/litegrip \
    | tar -x -C src --strip-components=1
git -C ../litegrip-python show NEW_SHA:LICENSE > src/litegrip/LICENSE
```

Then, in one commit:

1. Update the commit and date in the table above.
2. Recompute the tree hash with the recipe above and replace it.
3. Run the suite: `env -u LITEGRIP_SDK_PATH python3 -m pytest -q`. The SDK-surface tests are the
   contract — if upstream changed a signature the console calls, they fail here rather than on the
   bench.
4. Read `git -C ../litegrip-python log --oneline OLD_SHA..NEW_SHA` before trusting the diff; a
   behaviour change the console does not test for will not announce itself.

## Known divergence: the reported version

`litegrip.__version__` reads the *installed* distribution's metadata through
`importlib.metadata.version("litegrip")`, so it reports the version of whichever `litegrip`
distribution is installed in the interpreter — which is not necessarily this code. A machine with
no such installation reports `0.0.0+source`; a machine where an unrelated `litegrip` is installed
reports that distribution's number.

This is upstream's design, and it is copied unchanged rather than patched, so that a re-vendor stays
a plain file copy. Nothing in the console reads `litegrip.__version__`; the version a running console
reports is its own (`litegrip_studio.version`).
