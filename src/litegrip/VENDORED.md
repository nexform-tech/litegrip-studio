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
| Commit | `4cec95da2fb12ce5eb2617477564e81b65ba117e` (2026-10-08) |
| License | Apache-2.0 — `LICENSE` is that commit's file, copied verbatim |
| Factory calibration | <https://github.com/nexform-tech/litegrip-cpp> at `a2354fec5179cd968e24b8e1c00b7787d7009cce` (2026-10-08) — the one file that is not upstream Python's; see Local override below |

`src/litegrip/` holds the modules, the `can/` and `protocols/` subpackages, `py.typed`, and the
three JSON data files the code resolves through `dirname(__file__)`: `factory_calibration.json`,
`calibration_normal.json`, `calibration_reverse.json`. The last two are not decoration — the public
`load_calibration(template="normal" | "reverse")` resolves them by name (see `gripper.py`'s
`CALIB_TEMPLATES`), so they must travel with the code.

Upstream's `tests/`, `.github/`, `.releaserc.json`, `AGENTS.md`, `README*`, `examples/`,
`pyproject.toml`, and `__pycache__/` are deliberately **not** copied. Its tests are upstream's
suite about upstream's internals; the console pins the part of the SDK surface it actually uses
(`tests/test_vendored_sdk.py`, `tests/test_real_backend_api.py`) and records the tree hash below so
drift cannot go unnoticed.

`factory_calibration.json` is the one file here that does **not** come from that commit. It is
copied from the C++ SDK, and the reason is recorded in **Local override: the factory calibration**
below.

## Do not edit these files

An edit inside `src/litegrip/` is a fork of upstream that nothing records. It also survives only
until the next re-vendor, which copies each file wholesale and would silently drop it. Fix the SDK
upstream and re-vendor, or — when a local divergence is genuinely unavoidable — write it down in
this file with the reason, so the person re-vendoring sees it.

The tree hash below is what makes a silent edit visible: `tests/test_vendored_sdk.py` recomputes it
and fails when it no longer matches. The factory calibration is the one divergence taken that way —
a deliberate one, copied from the other upstream repository and written down under
**Local override: the factory calibration**. Every other file here is expected to be upstream's
byte for byte.

## Tree hash

```text
0c7c22a7e764ed896ee93aadcc7bcf8f6354de46d10f59ae3a636b0a6ec34fbf
```

The recipe, so it can be recomputed without reading the test: take every file under `src/litegrip/`
except `VENDORED.md` (this file holds the hash, so it cannot be part of it) and `__pycache__`,
ordered by its POSIX path relative to `src/litegrip/`; feed each path, a newline, then the file's
bytes into one SHA-256 stream; read the hex digest.

This hash covers every file, the locally overridden factory calibration included, so an edit to it
is caught the same way as an edit to anything else. *Which* copy of that file is here is a second
question, and it has a second hash — the one below.

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

## Local override: the factory calibration

`factory_calibration.json` in this directory is copied verbatim from
[litegrip-cpp](https://github.com/nexform-tech/litegrip-cpp) at
`a2354fec5179cd968e24b8e1c00b7787d7009cce` (2026-10-08), byte for byte that commit's
`calibration/factory_calibration.json`. Its own hash:

```text
448f6a6881bd08f4a89754acab1b178dc68125e51f158b772eba00a046147833
```

The two upstream repositories disagree about this file, which is why it is written down here. The
Python SDK's copy carries `0.0 / -1.651026 / 1.651026 / 52.69` plus a `work_stroke_mm` the console
does not read; the C++ SDK's copy carries `0.052071 / -1.357481 / 1.409552 / 61.01229326764816` and
`kp 5.0`, and the C++ repository's own commit is titled "sync the packaged factory calibration with
the re-measured unit". These are the numbers the unit on this bench was measured at, and they are
the scale the console derives for itself from an 86 mm travel (`61.01229326764816 × 1.409552` is
86.0 mm), so the file is consistent with the console rather than merely a fallback it tolerates.

Copy it from a commit, never from a checkout:

```bash
git -C <a-litegrip-cpp-checkout> show NEW_CPP_SHA:calibration/factory_calibration.json \
    > src/litegrip/factory_calibration.json
```

Re-vendoring from `litegrip-python` overwrites this file with the other unit's numbers, and the
diff says nothing about which unit they belong to. Re-apply this override after every re-vendor.

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

**Do not** copy the checkout's working tree instead (`cp -r ../litegrip-python/src/litegrip src/`).
A checkout sits on whatever branch it was last switched to, and that branch decides the numbers in
its `factory_calibration.json` — this file's own history is the evidence: upstream changed that file
in `4cec95d`, from `0.052071 / -1.357481 / 1.409552 / 61.01229326764816` to
`0.0 / -1.651026 / 1.651026 / 52.69` plus a `work_stroke_mm` key the console ignores. The console
runs on whichever of the two is vendored here, and a checkout left on a branch from before that
change still carries the other one. A `cp` from there would have moved the numbers the console
measures with, and nothing in the diff would have said so — a key the console does not know is not
refused, so the file loads either way. To see what a commit carries before copying it, ask the
commit: `git -C ../litegrip-python show NEW_SHA:src/litegrip/factory_calibration.json`.

Then, in one commit:

1. Update the commit and date in the table above.
2. Recompute the tree hash with the recipe above and replace it.
3. Re-apply the factory-calibration override, and update its hash too if the C++ repository has
   moved since. Skipping this ships the Python repository's copy, which is the other unit's.
4. Run the suite: `env -u LITEGRIP_SDK_PATH python3 -m pytest -q`. The SDK-surface tests are the
   contract — if upstream changed a signature the console calls, they fail here rather than on the
   bench.
5. Read `git -C ../litegrip-python log --oneline OLD_SHA..NEW_SHA` before trusting the diff; a
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
