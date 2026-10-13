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
| Commit | `5b7da892e34b1fb8a405cc6db9b924c656daf802` (2026-10-13) |
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

Every file here is that commit's byte for byte, with one declared exception: this bench's
`factory_calibration.json`, re-recorded by hand. The tree hash below covers every *other* file, and
**Local override** records the exception with a hash of its own.

## What the `b9caae8` → `5b7da89` move changed

Twenty-three upstream commits, eleven files under `src/litegrip/`. Most of the range is the
trajectory and force-hold engine and the readme. Three parts of it reach this console:

- **`LiteGrip.write_zero()` is new** (`gripper.py`). It rewrites the encoder offset over CMD `0xFE`,
  disabling and re-enabling the motor around the frame. The console's 电机标零 action is built on
  it; nothing in `litegrip_studio/` rolls its own sequence.
- **`factory_calibration.json` now carries `kp: 100.0` where it carried `5.0`** ("fix: align the
  factory calibration with the default stiffness"). This changes no motion here: the console
  streams its own MIT frames and passes its own gains to `send_mit_frame`, and never calls an SDK
  convenience method that would read the config's. It does travel further than that, though — the
  SDK's `save_calibration` writes `config.kp` back out, so a calibration the console saves after
  this re-vendor carries `100.0` where one saved before carried `5.0`. The two shipped
  `calibration_*.json` templates were *not* touched upstream and still say `5.0`.
- **The range ends in `feat!: remove set_force from the public API`** — a breaking change for
  upstream's own callers, and inert here. The console's `SetForce` command and
  `MotionFSM.set_force` are its own, and no module under `src/litegrip_studio/` calls the SDK's.
  It is not on the `FORBIDDEN` list in `tests/test_real_backend_api.py` for the same reason: that
  list names methods the console refuses to use, and this one is no longer there to refuse.

## Do not edit these files

An edit inside `src/litegrip/` is a fork of upstream that nothing records. It also survives only
until the next re-vendor, which copies each file wholesale and would silently drop it. Fix the SDK
upstream and re-vendor, or — when a local divergence is genuinely unavoidable — write it down in
this file with the reason, so the person re-vendoring sees it.

The tree hash below is what makes a silent edit visible: `tests/test_vendored_sdk.py` recomputes it
and fails when it no longer matches. It covers every file except the one **Local override** declares,
so an edit to any other file moves it, and an edit to the overridden file is caught by that file's
own hash instead.

## Tree hash

```text
696c22bd33727a5732e643e7dfcfffe46f93b9a6aa2f96c79af2a1deb97ef0b3
```

The recipe, so it can be recomputed without reading the test: take every file under `src/litegrip/`
except `VENDORED.md` (this file holds the hash, so it cannot be part of it), `__pycache__`, and the
overridden `factory_calibration.json` (whose content is pinned by its own hash in **Local override**
instead), ordered by its POSIX path relative to `src/litegrip/`; feed each path, a newline, then the
file's bytes into one SHA-256 stream; read the hex digest.

```bash
python3 - <<'PY'
import hashlib, pathlib
root = pathlib.Path("src/litegrip")
overridden = "factory_calibration.json"
h = hashlib.sha256()
for p in sorted(root.rglob("*")):
    if not p.is_file() or "__pycache__" in p.parts or p.name == "VENDORED.md":
        continue
    if p.relative_to(root).as_posix() == overridden:
        continue
    h.update((p.relative_to(root).as_posix() + "\n").encode())
    h.update(p.read_bytes())
print(h.hexdigest())
PY
```

## Local override

One file here is not upstream's, and it is the only one. `tests/test_vendored_sdk.py` hashes it on
its own and fails when it stops matching what this section records, so a re-vendor that copies
upstream's file back over it is a loud failure rather than a quiet change of millimetres.

### The factory calibration, re-recorded by hand

Upstream's `factory_calibration.json` carries the geometry the SDK's motor-driven probe measured for
the reference unit. This bench is calibrated by hand instead: the operator pushes the jaws onto each
stop and records the encoder at both, and on this unit that recording is the one in force. The pair
recorded on 2026-10-13 is `0.029564 / -1.141184`, a 1.170748 rad span, with `rad_to_mm` at
`73.45731105242119` — which is what the console derives for that span over `DEFAULT_TRAVEL_MM` plus
`SPAN_INSET_MM` (`86 / 1.170748`), so the fallback carries the scale the console moves by and not
one that merely looks plausible.

The override is data only. No file of code under `src/litegrip/` is edited: the one place in the SDK
that illustrates the span-versus-travel derivation with the shipped numbers — `GripperGeometry` in
`constants.py` — still shows upstream's.

Its hash, so a re-vendor can tell this file from upstream's:

```text
8f891b1022d35d13f27eb029ada0e6d8f94d45ad70ee69ee84eeac4154174b15
```

### The override this repository used to carry

Before the above, this repository carried the C++ SDK's copy of the same file. The two upstream
repositories disagreed about those numbers, and this repository took the C++ one:
[litegrip-cpp](https://github.com/nexform-tech/litegrip-cpp) at
`a2354fec5179cd968e24b8e1c00b7787d7009cce`. Upstream Python's copy held another unit's reading —
`0.0 / -1.651026 / 1.651026 / 52.69`, a span implying 86.99 mm where this bench travels 85 mm.

Upstream's `b9caae8` ("fix: ship the factory geometry in the calibration files") closed the gap. All
three of upstream's shipped calibration files carry `0.052071 / -1.357481 / 1.409552 /
61.01229326764816` since then, which is the unit on this bench, and the same numbers that override
was carrying. Taking upstream's file back was therefore not a change of numbers, and that override
is gone rather than moved.

The override above is a different claim about the same file, not a revival of this one: it is this
bench's hand-recorded pair against upstream's probe-measured one, rather than a choice between the
two upstream repositories.

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
2. Settle the local override, then recompute the tree hash with the recipe above and replace it. The
   copy above brings upstream's `factory_calibration.json` back with it, which silently reverts the
   fallback — the override hash test is what catches that. If upstream now carries this bench's
   hand-recorded pair, delete the override from **Local override** and take the file as it comes;
   if it does not, restore the file (`git show HEAD:src/litegrip/factory_calibration.json >
   src/litegrip/factory_calibration.json`) and leave that section as it is.
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
