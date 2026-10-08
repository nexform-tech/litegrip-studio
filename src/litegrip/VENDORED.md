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
| Commit | `b9caae84360d9a32332c811f1af348181616a2fd` (2026-10-08) |
| License | Apache-2.0 — `LICENSE` is that commit's file, copied verbatim |

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

Every file here is that commit's byte for byte, with no local override. That was not always true —
see **No local override** for the divergence this repository used to carry and why it is gone.

## Do not edit these files

An edit inside `src/litegrip/` is a fork of upstream that nothing records. It also survives only
until the next re-vendor, which copies each file wholesale and would silently drop it. Fix the SDK
upstream and re-vendor, or — when a local divergence is genuinely unavoidable — write it down in
this file with the reason, so the person re-vendoring sees it.

The tree hash below is what makes a silent edit visible: `tests/test_vendored_sdk.py` recomputes it
and fails when it no longer matches. There is no divergence to declare today, so the tree hash is
the whole of the check, and any edit at all — the calibration data included — moves it.

## Tree hash

```text
9b8b57df1a3ed4b5bf66f1bb8526f0cbf04f81ae6667c4de73acf2f289be2bad
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

## No local override

The two upstream repositories used to disagree about `factory_calibration.json`, and this repository
carried the C++ one: [litegrip-cpp](https://github.com/nexform-tech/litegrip-cpp) at
`a2354fec5179cd968e24b8e1c00b7787d7009cce`. Upstream Python's copy held another unit's reading —
`0.0 / -1.651026 / 1.651026 / 52.69`, a span implying 86.99 mm where this bench travels 85 mm.

Upstream's `b9caae8` ("fix: ship the factory geometry in the calibration files") closed the gap. All
three shipped calibration files now carry `0.052071 / -1.357481 / 1.409552 / 61.01229326764816`,
which is the unit on this bench, and the same numbers the override was carrying. Taking upstream's
file back is therefore not a change of numbers, and the override is gone rather than moved.

One key still differs from the C++ copy: upstream's file carries `work_stroke_mm: 80.0` and the C++
one has no such key at all. Upstream's commit says why it kept it — dropping it "would let open() run
to the mechanical stop instead of the 6 mm-short work stroke that was chosen deliberately".

**That key never reaches this console, in any file.** `work_stroke_mm` is read in exactly one place
in the SDK, `Gripper.open`, through `work_limit_target` (`actions.py:402`), and the console does not
call `Gripper.open` — or `close`, or `grasp`, or any other SDK convenience method. It streams its own
MIT frames from `send_mit_frame` / `poll` / `get_state` / `stop`; the reasons are structural and
listed in `backend/real.py` and `backend/__init__.py`. The console's own `open` drives to
`Limits.max_stroke_mm`, which is the travel measured with calipers, and `load_calibration` never
writes that field (`backend/real.py`).

An earlier version of this note claimed the key changed a motion here. It did not, and the decision
that removed it from the vendored copy was made on the strength of that claim: nobody should read
the removal of the override as reversing a motion, because there was no motion to reverse. A
consumer that does call the SDK's `Gripper.open` — upstream's own users, or the C++ SDK — does get
80 mm rather than the stop, which is upstream's stated intent for this unit and not this
repository's decision to make.

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
its `factory_calibration.json` — and upstream has now changed those numbers in both directions:
`4cec95d` replaced this unit's with another unit's, and `b9caae8` put this unit's back. A checkout
left on a branch from before the second change carries the other one, and nothing in the diff would
say so: the file loads either way, and its own `rad_to_mm` is not what the console moves by. To see
what a commit carries before copying it, ask the commit:
`git -C ../litegrip-python show NEW_SHA:src/litegrip/factory_calibration.json`.

Then, in one commit:

1. Update the commit and date in the table above.
2. Recompute the tree hash with the recipe above and replace it.
3. Run the suite: `env -u LITEGRIP_SDK_PATH python3 -m pytest -q`. The SDK-surface tests are the
   contract — if upstream changed a signature the console calls, they fail here rather than on the
   bench.
4. Read `git -C ../litegrip-python log --oneline OLD_SHA..NEW_SHA` before trusting the diff; a
   behaviour change the console does not test for will not announce itself.
5. Move the calibration numbers this repository repeats by hand. `src/litegrip_studio/selftest.py`
   carries the shipped files' angles and scales as literals (`KNOWN_CALIBRATIONS`) and asserts how
   each stands to the console's own derivation; a re-vendor that changes them does not move that
   table, and the assertions about the difference are exactly what fails when the two drift apart.

## Known divergence: the reported version

`litegrip.__version__` reads the *installed* distribution's metadata through
`importlib.metadata.version("litegrip")`, so it reports the version of whichever `litegrip`
distribution is installed in the interpreter — which is not necessarily this code. A machine with
no such installation reports `0.0.0+source`; a machine where an unrelated `litegrip` is installed
reports that distribution's number.

This is upstream's design, and it is copied unchanged rather than patched, so that a re-vendor stays
a plain file copy. Nothing in the console reads `litegrip.__version__`; the version a running console
reports is its own (`litegrip_studio.version`).
