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
| Commit | `78b8766c920cb0b7f0b35584c70b8ff33186272a` (2026-10-29, head of upstream's `feat/release-api` branch) |
| License | Apache-2.0 — `LICENSE` is that commit's file, copied verbatim |

That commit is on upstream's release-API branch, not on `main`: the tree is PR
<https://github.com/nexform-tech/litegrip-python/pull/77>. Re-vendor from `main` once it is merged —
the squash merge keeps this tree, so the hash below does not move.

`src/litegrip/` holds the modules, the `can/` and `protocols/` subpackages, `py.typed`, and the
three JSON data files the code resolves through `dirname(__file__)`: `factory_calibration.json`,
`calibration_normal.json`, `calibration_reverse.json`. The last two are not decoration — the public
`load_calibration(template="normal" | "reverse")` resolves them by name (see `gripper.py`'s
`CALIB_TEMPLATES`; upstream deprecates the template path but keeps shipping the files), so they must
travel with the code.

Upstream's `tests/`, `.github/`, `.releaserc.json`, `AGENTS.md`, `README*`, `examples/`,
`pyproject.toml`, and `__pycache__/` are deliberately **not** copied. Its tests are upstream's
suite about upstream's internals; the console pins the part of the SDK surface it actually uses
(`tests/test_vendored_sdk.py`, `tests/test_real_backend_api.py`) and records the tree hash below so
drift cannot go unnoticed.

Every file here is that commit's byte for byte, with no exception left: the factory calibration this
repository used to override went upstream, so the tree hash below covers the whole tree. **Local
override** records that file and where it went.

## What the `cacf392` → `78b8766` move changed

Ten upstream commits. The one that matters here is the release API, which this console now calls;
the rest is packaging, CI and deprecation and changes no motion.

- **`d8cf393` is where the console stopped being a pure `send_mit_frame` client.**
  `d8cf393` ("feat: add thread-safe release api for ending a grasp") adds
  `LiteGrip.release(clearance_mm=10.0, *, abort=False, timeout_s=None)` and
  `LiteGrip.request_release(clearance_mm=10.0, *, abort=False)`, a `ReleaseResult`, and
  `GraspResult.released`. `grasp(hold_s=0)` still holds indefinitely; the hold now also ends on a
  release request, which the grasp thread checks before every frame (5 ms). Because only the grasp
  thread may send frames, `release()` from another thread queues the request and waits for that
  thread to do the opening; `request_release()` is the non-blocking form and never touches the bus —
  that is the abort hook the console's action thread needs, and the reason `grasp` and `release` left
  `FORBIDDEN` in `tests/test_real_backend_api.py`. The opening pose is the one the grip was
  *declared* at, `max(current, grip) + clearance_mm`, clamped to the calibrated open stop — the same
  rule this console's BackOff applies, since the console's BackOff now calls it. `abort=True` drops
  the torque without an opening move, for E-stops. **`MotionConfig.release_wait_s` (10 s) is new**:
  how long `release()` waits for the grasp thread before returning `ok=False`.
- **The distribution is named `litegrip-python`, not `litegrip`** (`7f98ef5`). This reaches every
  `importlib.metadata.version()` call and every `pip install litegrip[zenoh]` hint in the tree, and it
  moves what `litegrip.__version__` answers — see **Known divergence: the reported version** below.
  No console code reads that, and the console never pip-installs the SDK. `5b16220` then requires
  Python 3.10 and completes the PyPI metadata (`pyproject.toml` and both READMEs, no code).
- **`21e8981` retires the reverse-mount guidance, comments and docstrings only.** `LiteGrip(mount=...)`,
  `GripperConfig.mount`, `load_template`, `list_templates` and `load_calibration(template=...)` are
  now marked deprecated in favour of the one shipped mounting and a named calibration file; every one
  of them still works, and no runtime behaviour changes. The console never called any of them; its
  only contact with `CALIB_TEMPLATES` is reading the two shipped template files by path
  (`tests/test_units.py`, `src/litegrip_studio/selftest.py`), and those files did not move.
- **`a2902fa` (repository standards) removes upstream's `AGENTS.md`** and its CI check for it;
  `25ecacd`, `0d46d6d` and `35e0bc8` are CI and README changes (PyPI publishing, the
  `README.zh-CN.md` rename, the PyPI link targets). None of these files is vendored.
- **`78b8766` deletes two internal bench scripts from upstream's `tests/`** (`torque_guard_check.py`,
  `trajectory_check.py`). `tests/` is not vendored, so it cannot reach this console.

Step 5 of the recipe below is a no-op for this move: no shipped calibration file changed, and apart
from the release API no motion did either.

## What the `9180117` → `cacf392` move changed

Two upstream commits, the fixes for the two defects found on this bench on 2026-10-14. Both reach
this console, and neither touches a calibration file.

- **`ffc1ee5` ("fix: report a failed send with its cause and its type") changes what a failed send
  raises.** `GripperActions._emit` used to raise `CommandError` for every failure — a type the
  README's error table does not name, carrying none of the reason. It now raises what fits:
  `NotInitializedError` when the bus was never connected or the motor never registered, `CommError`
  otherwise, with the transport's own exception on `__cause__` and the reason on `last_send_error`.
  The console is unaffected, and both halves of that were checked rather than assumed: every send it
  makes goes through `LiteGrip.send_mit_frame`, which still reports with a `bool` (it gained only the
  recorded `last_send_error`), and every console clause that names an SDK exception names
  `LiteGripError`, the base class both new types share. **Do not** write `except CommandError` around
  a console motion call: that is no longer what a failed send raises.
- **`cacf392` ("feat: report a stale status read, and let the caller refresh it") tells a cached
  reading from a measured one.** `GripperState` gained `fresh`, `data_age_s`, `is_stale()` and the
  module-level `STALE_AFTER_S = 0.5`; `LiteGrip.refresh_status()` sends the `0xCC` request that makes
  an idle motor speak; `LiteGripCAN` gained `last_update` (monotonic) and `last_frame_time` (wall
  clock). The console reads none of that — it takes `get_state(wait=False)` for the readings and
  never asks whether a frame arrived — so no file under `src/litegrip_studio/` changes for this
  commit. `GripperState.timestamp` did change meaning, from the time of the call that returned the
  snapshot to the arrival time of the frame behind it, `0.0` when no frame has ever arrived. **Do
  not** time anything with it or read it as a heartbeat; it now describes the frame, and the console
  keeps its own clock.

Step 5 of the recipe below is a no-op for this move: neither commit changes a shipped calibration
file, so the angles and scales this repository repeats by hand did not move.

## What the `5b7da89` → `9180117` move changed

Three upstream commits. `351f80d` ("docs: note the empty-grasp runaway trip and how to get out of
it") touches no file under `src/litegrip/`; the other two do, and both reach this console.

- **`_iface_mtu()` hands the kernel a whole `struct ifreq`** (`9180117`, "size the SIOCGIFMTU buffer
  for a whole struct ifreq"). `SIOCGIFMTU` writes back all 40 bytes of the struct, and since Python
  3.14 `fcntl.ioctl()` raises `SystemError: buffer overflow` when anything lands past the end of its
  argument (`cpython c2eaeee3`) — a `SystemError`, not the `OSError` that helper catches. The
  18-byte buffer it was passing therefore killed `connect()` on 3.14 against every real gripper,
  while the simulated backends, which never call `_iface_mtu()`, and every interpreter up to 3.13
  stayed green. **This is what this re-vendor is for**: the console carries this code rather than
  depending on an installed SDK, so the fix reaches it only by copying the tree.
- **`factory_calibration.json` carries this bench's geometry** (`c187b78`, "replace the factory
  calibration with the re-measured geometry"): `0.029564 / -1.141184 / 1.170748 /
  73.45731105242119` where it carried `0.052071 / -1.357481 / 1.409552 / 61.01229326764816`. The
  file also gained `"calibrated": true` and moved `mst_id` to the end. These are the numbers this
  repository was carrying as its **local override**, and upstream adopted them, so the override is
  gone and the file is upstream's again — see **Local override**.
- The two `calibration_*.json` templates moved to that same geometry, so all three shipped files
  still describe one unit, and `GripperGeometry`'s docstring in `constants.py` now derives its
  span-versus-travel example from the new numbers. Nothing in the console repeats that docstring, but
  the numbers it *does* repeat by hand — `KNOWN_CALIBRATIONS` in `src/litegrip_studio/selftest.py`
  and `TEMPLATE` in `tests/test_units.py` — moved with them (re-vendor step 5).

`rad_to_mm` rises by a factor of 1.204, so the two factory calibrations are not interchangeable: each
spans the same 86 mm and each is internally consistent, and only the absolute angles separate them.
Anything pinned to the old numbers had to move, and did.

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
  `calibration_*.json` templates were *not* touched by that commit and carry no `kp` key at all.
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
and fails when it no longer matches. It covers every file in the tree except this note, so an edit to
any file the SDK ships moves it.

## Tree hash

```text
56f014cee55197fd6673271b802e4f1eecb52f7f313363e5f6cfde138e120276
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
    if not p.is_file() or "__pycache__" in p.parts or p.name == "VENDORED.md":
        continue
    h.update((p.relative_to(root).as_posix() + "\n").encode())
    h.update(p.read_bytes())
print(h.hexdigest())
PY
```

If a re-vendor ever has to leave a local divergence in place, declare it here by name and exclude it
from the recipe above, pinning it with a hash of its own — the recipe and the exclusions have to be
written where a reader can recompute the hash from this file alone.

## Local override

There is none today: every file under `src/litegrip/` is upstream's at the commit above, byte for
byte, and the tree hash covers all of them. This section records the one that was here and where it
went, so the next re-vendor does not have to rediscover it.

### The factory calibration, re-recorded by hand

Upstream's `factory_calibration.json` carries the geometry the SDK's motor-driven probe measured for
the reference unit. This bench is calibrated by hand instead: the operator pushes the jaws onto each
stop and records the encoder at both, and on this unit that recording is the one in force. The pair
recorded on 2026-10-13 is `0.029564 / -1.141184`, a 1.170748 rad span, with `rad_to_mm` at
`73.45731105242119` — which is what the console derives for that span over `DEFAULT_TRAVEL_MM` plus
`SPAN_INSET_MM` (`86 / 1.170748`), so the fallback carries the scale the console moves by and not
one that merely looks plausible.

The override was data only. No file of code under `src/litegrip/` was edited: the one place in the SDK
that illustrates the span-versus-travel derivation with the shipped numbers — `GripperGeometry` in
`constants.py` — showed upstream's until this re-vendor, which moved both to the same re-measured
pair.

Upstream `c187b78` ("fix: replace the factory calibration with the re-measured geometry") then wrote
exactly those numbers into upstream's own `factory_calibration.json`. So the override was dropped
rather than kept: it no longer said anything upstream disagreed with, it *was* upstream's file. Its
hash, which is what upstream's file hashes to since `c187b78`, and which `tests/test_vendored_sdk.py`
pinned while the override existed so that a re-vendor could not revert it quietly:

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
three of upstream's shipped calibration files carried `0.052071 / -1.357481 / 1.409552 /
61.01229326764816` from then on, which is the unit on this bench, and the same numbers that override
was carrying. Taking upstream's file back was therefore not a change of numbers, and that override
is gone rather than moved. (`c187b78` moved all three files once more, to the pair recorded above.)

The override that replaced it was a different claim about the same file, not a revival of this one:
this bench's hand-recorded pair against upstream's probe-measured one, rather than a choice between
the two upstream repositories. Upstream then measured this unit again and got the same answer, so
that claim is upstream's too now.

One key still differs from the C++ copy: upstream's file carries `work_stroke_mm: 80.0` and the C++
one has no such key at all. Upstream's commit says why it kept it — dropping it "would let open() run
to the mechanical stop instead of the 6 mm-short work stroke that was chosen deliberately".

**That key never reaches this console, in any file.** `work_stroke_mm` changes a motion in exactly
one place in the SDK, `Gripper.open`, through `work_limit_target`, and the console does not
call `Gripper.open` — or `close`, or any other SDK motion method. It streams its own MIT frames from
`send_mit_frame` / `poll` / `get_state` / `stop`, with `grasp` / `release` / `request_release` as the
one exception: those run on the worker's dedicated action thread, which is what makes them
interruptible, and neither reads `work_limit_target`. The reasons are structural and listed in
`backend/real.py` and `backend/__init__.py`. The console's own `open` drives to
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
   copy above brings upstream's `factory_calibration.json` with it, and if this repository has
   diverged from that file, the copy silently reverts the divergence — the hash tests are what catch
   that. There is no override in place today: take the file as it comes, unless this bench's hand
   recording has moved away from upstream's again. In that case save upstream's copy first
   (`git show HEAD:src/litegrip/factory_calibration.json > /tmp/upstream-factory.json`), put the hand
   recording back, and redeclare it in **Local override** — its hash, and its exclusion from the
   recipe above — so the next re-vendor sees the divergence rather than overwriting it.
3. Run the suite: `env -u LITEGRIP_SDK_PATH python3 -m pytest -q`. The SDK-surface tests are the
   contract — if upstream changed a signature the console calls, they fail here rather than on the
   bench.
4. Read `git -C ../litegrip-python log --oneline OLD_SHA..NEW_SHA` before trusting the diff; a
   behaviour change the console does not test for will not announce itself.
5. Move the calibration numbers this repository repeats by hand. `src/litegrip_studio/selftest.py`
   carries the shipped files' angles and scales as literals (`KNOWN_CALIBRATIONS`) and asserts how
   each stands to the console's own derivation; `tests/test_units.py` carries the `normal` template's
   three numbers as `TEMPLATE`, and `tests/test_calibration.py` and
   `tests/test_ui_calibration_page.py` carry the factory file's. A re-vendor that changes them does
   not move any of those, and the assertions about the difference are exactly what fails when the two
   drift apart.

## Known divergence: the reported version

`litegrip.__version__` reads the *installed* distribution's metadata through
`importlib.metadata.version("litegrip-python")`, so it reports the version of whichever
`litegrip-python` distribution is installed in the interpreter — which is not necessarily this code.
A machine with no such installation reports `0.0.0+source`; a machine where an unrelated
`litegrip-python` is installed reports that distribution's number.

This is upstream's design, and it is copied unchanged rather than patched, so that a re-vendor stays
a plain file copy. Nothing in the console reads `litegrip.__version__`; the version a running console
reports is its own (`litegrip_studio.version`).
