# LiteGrip Studio

**English** | [简体中文](README_ZH.md)

**LiteGrip Studio** is the PyQt5 operator console for the LiteGrip adaptive two-finger gripper:
opening and closing, positioning by dragging, force-limited grasping, speed, live state and fault
monitoring, plots, and a full calibration page.

It drives the gripper over SocketCAN by importing `litegrip` in the same process — there is no
cross-language bridge. With no hardware attached the backend switches to a built-in simulated
plant, and **every interaction still works**: the simulation is not a stand-in for the interface,
it runs the same motion state machine the real gripper does.

---

## Related repositories

| Repository | Role |
| --- | --- |
| [litegrip-python](https://github.com/nexform-tech/litegrip-python) | Python SDK |
| [litegrip-cpp](https://github.com/nexform-tech/litegrip-cpp) | C++ SDK |
| [litegrip-docs](https://github.com/nexform-tech/litegrip-docs) | Product documentation |
| [litegrip-ros2](https://github.com/nexform-tech/litegrip-ros2) | ROS 2 driver |

---

## 🚀 Quick start

```bash
./run_litegrip_studio.sh sim        # simulation, no hardware needed
./run_litegrip_studio.sh selftest   # self-check: no Qt, no hardware
./run_litegrip_studio.sh test       # test suite
./run_litegrip_studio.sh gui        # real gripper over SocketCAN
./run_litegrip_studio.sh build      # one-file executable
```

Every path in the scripts is quoted: this repository's directory name contains a space. The
interpreter and the SDK location can both be overridden from the environment, with `PYTHON_BIN`
and `LITEGRIP_SDK_PATH`. The launcher otherwise uses `python3`, so where Qt lives in a
virtualenv, point `PYTHON_BIN` at it:

```bash
PYTHON_BIN=.venv/bin/python3 ./run_litegrip_studio.sh sim
```

Remaining arguments are passed to the program unchanged, for example
`./run_litegrip_studio.sh sim --log-level DEBUG`. `-h` lists every subcommand.

### Connecting to real hardware

**You do not have to bring the interface up yourself.** When you press Connect, the console looks
at `can0` first, and only intervenes when the interface really is wrong — down, or at the wrong
bitrate — and only in that one place:

- Privilege is escalated through `pkexec`, the desktop's own authorization dialog. **This process
  never touches the password**, so it cannot write it into a log either.
- An interface that is already configured: no privileged command runs and **no dialog appears**.
  That is deliberate, so that bringing the interface up by hand does not make every connection
  prompt you. "Configured" is judged on more than the up flag and the bitrate: the controller's
  `can state` is checked too, because a **bus-off interface reports all of those fields as
  perfectly normal** while being unable to send a single frame.
- Configuration adds `restart-ms 100`: the controller reopens itself 100 ms after entering
  bus-off. Without it — the kernel default is 0 — it stays down **forever**, every subsequent
  frame send returns ENETDOWN, and `connect()` only opens a socket, so nothing is reported until
  the first action that really sends a frame (enable). What you then see is a kernel message,
  "Network is down, errno 100".
- A CAN FD interface is reported, never changed. The SDK detects FD from the interface MTU and
  works with it, whereas changing the interface back to classic CAN would change every other node
  on that bus too.
- When ENETDOWN does happen, **it is treated as a link problem rather than a motor problem**: the
  message names the interface and `ENETDOWN`, and asks you to check the adapter and wiring before
  reconnecting. The kernel's errno and a driver fault code are entirely different things — the
  driver's codes are only 0/1/9/0xA/0xB/0xC — and repeating one as the other only sends people
  looking at the 24 V supply.
- An authorization prompt that is dismissed, a `can0` that does not exist, an `ip` that fails:
  **the connection attempt continues anyway**, and the log and the banner say what happened. This
  step must never be the reason a connection fails, because something else may well own the
  interface — only the connection itself can say whether the link works.

If a unit file or a script owns the interface and you do not want the prompt, turn the step off
with `--no-can-setup`; for a bitrate other than 1 Mbit, use `--can-bitrate`.

To do it by hand, the equivalent three commands are still these:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 restart-ms 100 fd off
sudo ip link set can0 up
./run_litegrip_studio.sh gui
```

**On a real gripper, run 自动标定 from the calibration page before touching the
slider.** The result is written out as soon as the probe finishes. If the calibration page shows
`FACTORY` or `BLOCKED`, calibrate first rather than overriding the gate.

---

## 🖥️ The window

Four tabs, plus the connection bar that is always visible (connect / disconnect / enable /
disable / clear fault / reset emergency stop), the emergency-stop button, and the log dock.

Above the tabs sits one **alert banner**, and it carries the alert in force rather than a
history. The worker takes it down again when the condition behind it stops holding — a fault
that cleared, an emergency stop that was reset, a refusal whose reason no longer applies — so
a console that has recovered stops reading as a broken one. What it does not retract is the
report of a single event (a save that failed, a probe that ended): those stay until the next
alert replaces them. Successes do not reach the banner at all. The log has all of it either
way.

| Page | Contents |
| --- | --- |
| Control | Position slider, open all / close all, stop (hold position), zero gravity (back-drivable), speed, grasp force, grasp / let go |
| Status | Telemetry table, temperature, faults and clearing them, link health |
| Plots | pyqtgraph position and force plots (the force axis autoscales) |
| Calibration | Provenance banner, the 自动标定 and manual two-point wizards, the file actions, the gate |

**Esc is an application-level shortcut**: it works with a spinbox focused and with a modal dialog
open.

### Light and dark

The console opens dark, which is what a bench under a machine tends to be, and is what it opened
in before there was a choice. The button beside the emergency stop switches it to light and back,
and the choice is written to `~/.litegrip/litegrip_studio.ini` under `view/theme`.

Nothing is written until you switch, so "nobody has chosen" stays distinguishable from a choice
that happens to match the default.

The palette is litearm-studio's in both themes: the same ink ramp, the same semantic fill/edge
pairs, and the same type scale. What is this console's own is the meaning layer — blue for the
position that was *commanded*, green for the one the jaws *measured* — and those two are the same
colours on the slider, in the readout and in the plots.

### The slider

The core of the requirement. Three visual elements, two independent values:

| Element | Source | Meaning |
| --- | --- | --- |
| Handle | The user's drag; the measurement when the slider is not driving | The commanded target |
| Target hairline (cyan) | The last command | Where the jaws are heading |
| Measured triangle (thick green) | Telemetry | Where the jaws actually are |

The left end is the closed position and the right end the widest opening, with a fixed-width read-
out to the right showing actual / target / error. **The bar follows the gripper as it moves**: when
the open and close buttons drive it the handle follows the measurement, and while you drag, the
handle follows your finger while the measured marker moves on its own — otherwise the slider under
your finger gets pulled back by telemetry. A command is sent on release by default, matching
litearm-studio's `onValueChange` / `onValueCommit` split; "follow while dragging" can be switched
on.

Following live is safe only because every motion command goes through the rate-limited reference
generator: drag out a 120 mm jump and the gripper still crosses it at the configured speed.

While the jaws are holding something (`frame.grasped`, which is `HOLD_FORCE`) the read-out shows
**the actual position only**. A grasp drives to 0 mm under a force cap, so its target is the closed
end and its error is the width of the object — both true, and side by side they read as a move that
has gone badly wrong ("28 mm short of the target, and not closing"). During the approach, before
the jaws meet anything, the target and the error are shown as usual: they are the real trajectory
and the real following error, which is what an operator watching for contact is reading them for.

Each end of the slider has a **one-press** button, placed next to the end it drives:

| Button | Command | Where it goes |
| --- | --- | --- |
| Close all (left) | `Close()` | Travel 0.0 mm, the closed position the calibration recorded |
| Open all (right) | `Open()` | `limits.max_stroke_mm`, the top of the range — 1 mm inside the recorded open limit |

Both commands go through the rate-limited reference generator rather than jumping, and Stop or the
emergency stop can take over at any moment while the button is held. The tooltip names the exact
millimetre figure and refreshes as soon as a calibration arrives — before one is read the tooltip
**quotes no number**, because at that point no millimetre figure belongs to this gripper. The
tooltip quotes the **range** (`max_stroke_mm`), not the span the calibration file records: once
mm/rad has been derived from the measured travel the two differ, and quoting the span sends people
looking for travel that cannot be reached.

Directly under **Grasp** in the force box is a **Let go** button: it opens 10 mm further than where
the jaws **are** (`RELEASE_OPEN_MM`), to release whatever they are holding. It measures from the
**measurement**, not from the last command's target — a grasp drives to 0 mm under a force cap, so
its target is always the closed end and "target + 10 mm" is a command straight back into the
object. At the top of the range it is a move of zero length (`move_to_mm` clamps) rather than an
error. It is a millimetre command, so the gate refuses it like any other move; with an unusable
calibration the two ways to let go are **zero gravity (back-drivable)** and **stop**.

### The force plot autoscales

The force plot used to be pinned to the 40 N rating, which states what the mechanism is **allowed**
to do rather than what it **is** doing: a 12 N grasp was drawn in the bottom third of the axis, and
the ripple inside the grasp — the thing that tells you whether it is settling or still climbing —
was a few pixels tall. The axis is now fitted to the data in the window on every redraw:

- rounded up to 1 / 2 / 5 × 10ⁿ (10 N, 20 N, 50 N), so the reading has a reference point;
- **a peak is never outside the plot**: it opens the axis immediately;
- it only closes again once both peaks are within three quarters of the axis, which stops 8.6 N
  (→ 10 N) and 8.7 N (→ 20 N) from flipping back and forth at 25 Hz across a step boundary;
- the floor is 2 N (1 N each way): unloaded, the braking torque is a fraction of a newton, and an
  axis fitted to that magnifies noise into a screen-filling waveform that looks like a grasp.

The sign of the force is kept — the SDK makes no promise about the sign of its torque — so the
axis **crosses zero** and a negative braking torque is not clipped. The position plot does not do
this: its axis is scaled to the travel, so the same motion looks the same on any gripper.

---

## 📐 Calibration: where the file comes from

This was confirmed against the source while designing, and the answer is not either/or. The
console resolves the calibration itself, best source first:

1. **The user calibration file** (`$LITEGRIP_CALIB` or `~/.litegrip/litegrip_calibration.json`),
   written by `calibrate_*()` → `save_calibration()`. **This is the file that describes this
   gripper.**
2. **The SDK's factory calibration file** (`litegrip/factory_calibration.json` inside the SDK
   package, read-only).
3. **The console's own copy of the same fallback** (`factory_calibration.json`, package data of
   `litegrip_studio`, read-only) — what a machine with no SDK checkout still has. It sits last on
   purpose: adding it can only give a fallback to a machine that had none, never change which
   numbers a machine already works on.

`LITEGRIP_FACTORY_CALIB` replaces 2 and 3 outright, for a bench that keeps its fallback elsewhere.
Neither of the two is ever a save target.

So: **the console uses the calibration result, and the factory file is only a fallback.** The
process has several traps, which is why the console reads the files and decides the provenance
itself rather than trusting the SDK's return value:

- **The fallback is silent.** `load_calibration()` returns `True` in both cases; only one
  `log.info` line distinguishes them. On real hardware that means millimetre readings that are
  systematically wrong with nothing on screen to show it.
- **The SDK does not require a calibration.** Neither `connect()` nor `enable()` loads one. Without
  it you get `GripperConfig`'s defaults, `pos_closed_rad=0.0 / pos_open_rad=1.14`, whereas on real
  hardware the closed value is the **larger** one (the fallback the console ships has
  `0.052 / -1.357`). The mapping `mm = (pos_closed_rad - position_rad) * rad_to_mm` then runs
  **exactly backwards**, computing negative millimetres and driving into the hard stop.
- **`load_calibration()` overwrites `kp` / `kd` / `grasp_torque_threshold`**, and reads its fields
  without protection: a bad file raises `KeyError`.

The ordering `zero_rad <= open_rad` is what the SDK's defaults look like — `(0.0, 1.14)` — and it
is still the signature the console recognises them by. But an ordering is not a defect: the same
two numbers, read in the same order, are a file whose angles were recorded with the encoder reading
*growing* as the jaws open, and every number derived from that file stays self-consistent. So the
console *reads* the direction out of the two angles (`Limits.direction`) and every conversion,
clamp and velocity feed-forward uses it, which makes either ordering work end to end. What refuses
a file is `frame_mismatch`, below, which never looks at the ordering.

All of the above is decided by **reading files**, and a file can be perfectly self-consistent while
describing a different gripper — as long as it was calibrated against another encoder zero (a
different unit, or the same one reassembled and recalibrated). Travel, coefficient and direction
all check out, the gate calls it a user calibration, and every millimetre the operator sees is off
by a constant. Nothing in the file can reveal this; only the measured angle can. **When the
measured angle falls outside the calibrated travel by more than `CALIB_MISMATCH_RAD` (0.15 rad) the
gate goes `BLOCKED`**, reporting the measured value and the travel interval together.

The 0.15 rad tolerance is there because a calibration that stops at the soft limit rather than at
the hard stop — the recommended kind — leaves a gripper resting on its hard stop **legitimately**
outside its commandable travel; on this machine that margin is at most 0.0866 rad. A zero that
belongs to another frame is off by a radian (1.885 rad was measured once). 0.15 rad sits between
the two: it neither closes the gate by mistake nor lets a mismatch through.

This check runs **after the measured angle is known and before any frame is sent.** Before the
frame is not a preference but a requirement: once a position has been clamped into the travel by
`clamp_mm`, a measurement outside the travel and one resting on the travel limit become
indistinguishable, and the clamped number still looks entirely reasonable. On real hardware this is
what "it clamps the moment you enable it" is made of.

The gate has three states: `READY` (user calibration, no problems), `FACTORY` (needs "I understand
the risk" ticked), and `BLOCKED` (missing, or a zero that belongs to another machine —
**never overridable**). **The gate logic lives in the worker, not the UI**: the UI can disable
widgets, but the refusal has to happen before a frame is sent.

One more thing: the calibration file is loaded by the worker **on connect**, not at start-up, so
the provenance and the gate always describe the file that was actually handed to the SDK rather
than the one a command line mentioned at the time.

Changing the calibration invalidates every millimetre held in the loop. The position is **read
again** from a new frame — the reading was originally converted by the backend using the limits in
force at the time — and a target being held or moved is **re-anchored** in place, because an
intention expressed in the old frame's millimetres has no honest conversion. Without those two
steps, loading a *correct* calibration sends the axis straight for the other end of the new travel.

A probe result lives only in memory at first (`in_memory_unsaved`), and **the gate stays shut
while it does**: you look at what was measured before deciding it should govern motion. A probe
that runs to the end therefore **writes itself out** rather than waiting to be told — the two
presses of 记录 are already the request for a calibration, and leaving the axis locked until the
operator works out that a third press is needed is how a finished calibration looks like a broken
console. Saving is followed by the backend reading the file back, and only then does the provenance
become `user_file` and the gate `READY`. Both the real and the simulated backend do this: a
simulation that wrote without reading back would leave the operator with the gate shut after a
successful calibration.

Two things the automatic write deliberately will not do. It will not save a result that failed
validation, because the file it would replace is a working calibration — the operator keeps a
console that refuses to move over one that moves on numbers it has just called unusable. And it
will not save over the SDK's own factory file, which ships with the package and describes whichever
unit it was taken on. 重新保存标定… stays on the page for the one case that is left: the write
itself failing, on a read-only directory or a disk that has filled up.

### Enabling: read a position first, then hold it

`get_state().position_rad` is `0.0` until the first status frame arrives, and `to_mm(0.0)` lands
somewhere in the **middle** of the travel, at a position that looks entirely reasonable. So "hold
in place after enabling", if it happens before that first frame, holds an angle the motor never
reported: on real hardware this one sends the gripper straight into the closed limit.

The position reading therefore has two states, and `TelemetryFrame.position_mm` uses `None` for
**no measurement yet** — not 0. The worker only produces a number once it has counted status
frames itself (`_rx_frames > 0`, the same signal the gate and the link liveness check use). Before
that:

- **Enabling** does not hold, it goes to zero gravity (`RELEASE`, `kp=kd=tau=0`). Zero gain needs
  no position and is the only honest command in that window; the first frame turns it into a hold at
  the measured position automatically, with a log line.
- Stop / leaving zero-gravity / clearing a fault / reopening the gate all need to "hold in place",
  and the rule is the same: with no measurement, zero gravity.
- A motion request is refused with the reason spelled out: "no position read yet; the motor has
  never reported a status frame".
- No number appears anywhere in the interface: the slider draws no measured triangle and does not
  move its handle, and the read-out, status page, status bar, calibration page and plots all show
  `—` (the plot records NaN and the line breaks). 0.00 mm is the closed position: a position, and
  the wrong one.

The liveness counter `_rx_frames` is deliberately conservative — the SDK's own `enable()` polls
internally, and the frame it receives is one we never see — so the window may last a few ticks
longer than strictly necessary. Erring towards refusing motion is the only correct way to err.

### Two rules for the automatic probe

The probe is the only action in the application allowed to do two things no other action may, and
both follow from what it is for:

- **It may move before a calibration exists** — it is the step that produces one.
- **It may travel outside the calibrated travel once one does** — it is looking for the hard stops,
  and the hard stops are by definition beyond the soft limits.

The second rule was once missing: `stream_frame`'s travel check only asked whether a usable
calibration existed, not whether this frame carried a target the check could judge, so any valid
calibration shut the probe inside the very travel it was there to measure, and it recorded "the
last step I was allowed to command" as the open limit — a wrong answer that reads like a successful
measurement. Such frames now say so explicitly (`ungated=True`): the probe steps, which are looking
for stops beyond the red lines, and the frames that carry no target at all — 零重力 (`RELEASE`),
the wizard's zero gravity (`ZERO_G`), and the hold a probe is left in, whose pose is the angle the
encoder has just reported. An ungated frame still refuses non-finite values and negative gains,
which are wrong whatever the calibration says.

The flag is named for what the frame *is* rather than for who sent it, and that includes the
long-lived states: 零重力 can be held open for an hour without widening what it permits, because what
it permits is a frame with no stiffness in it. Before it existed there was one flag named after its
first caller, and 零重力 behind a shut gate — the state an operator needs precisely when the file is
bad — was refused by the gate it was there to work around.

The probe also has to know **which way the jaws open**, and it cannot read that from a file: it is
the thing producing the file. It walks the jaws into the open stop and then into the closed one,
and the units this console drives have the open stop at the *smaller* angle, so the direction is a
constant of the machine rather than an answer to collect — one direction, and therefore nothing an
operator can get wrong before the probe starts. The page shows the raw encoder angle beside the
millimetres, because the millimetres are computed through the calibration under suspicion.

The probe decides it has reached a stop when the angle it reads stops changing, and that same
signal also means "no frame reached the motor" and "the feedback died" — in which case it records
both limits at the same position and reports a travel error (real hardware has produced
`closed -1.370650 rad is not greater than open -1.370650 rad`; two identical figures are exactly
this). The state machine cannot tell those three apart from the inside, so the worker watches from
the outside:

- **Frames not going out**: `stream_frame` returning False for `PROBE_REFUSED_FAIL_S` (0.5 s,
  a small part of the 1.8 s stall window) in a row → abort with "no position frame could be sent
  for N s; the motor received no command at all".
- **Connection / enable / link / fault**: all four are checked every tick, and any of them wrong
  aborts immediately, saying which one it was.

An abort **writes every step the probe judged into the log before stating its conclusion** — in a
failed probe those lines are the whole truth.

The probe measures the distance between the hard stops (the two hard stops, not the distance
between the soft limits); the angle it measures is the denominator of the mm conversion, and the
numerator is the travel the operator measured — see "Known limitations".

### The two-point manual probe

The automatic probe needs the stops to be reachable and detectable. When they are not — a stiff
linkage, a travel that is not where the SDK expects it, a drive that will not take a probe step at
all — the manual wizard takes over: the axis is held limp and the operator works the jaws to each
extreme by hand. **Starting it puts the axis in zero gravity and says so**, and finishing it takes
the axis back — on every way out, including a cancel, because the operator's hands are on the jaws
for all of it. The automatic probe is left alone: it drives the jaws into the stops itself and needs
the axis to itself to do it.

- **Two labelled buttons, not one.** The open extreme is recorded first, then the closed one, which
  is 0 mm. The label on the button is the operator's whole answer to "which end is zero", so a
  press that arrives out of step is refused out loud rather than taken as whichever point is due:
  the two angles swapped pass every check the console makes and drive the gripper backwards.
- **Nothing is sampled in the background.** The angle recorded is the one the encoder reports on the
  tick after the press — the first reading after the hand stopped moving. Sweeping the axis and
  keeping the extremes, which is what the SDK's zero-gravity mode does, measures the travel the hand
  happened to sweep: a gripper released halfway still looks like a calibration, and neither end is
  named, so the file cannot say which one is 0 mm.
- **It needs no declaration of any kind.** The two labelled presses say which end is which, so the
  direction is whatever the two recorded angles turn out to be rather than anything agreed
  beforehand. Both points recorded in the same place is reported, not adopted — a file built from
  it would put every millimetre of the travel at one angle.
- **Readings are checked before they are believed.** A value outside the SDK's own plausibility
  bound, or one that moved further in one tick than the mechanism can, is dropped rather than
  recorded: a lost frame leaves the SDK's cached position at `0.0`, which is inside that bound and
  is exactly where a misread would put the *open* limit — the one reading that would quietly invert
  the whole travel. A drop also cancels a press it lands on, since the angle the operator pressed
  for is the one that could not be read.
- **Every wait is bounded** at `TWO_POINT_TIMEOUT_S` (5 minutes per point). The axis is limp for the
  whole procedure, and a console left holding an enabled, limp motor is a gripper that falls open on
  whatever is under it. A probe that fails or is cancelled stays limp; only one that ran to the end
  hands the axis back under position control, at the SDK's exit gains — and with the write broken it
  is handed back at the *angle the encoder reports*, because a hold in millimetres would have to
  come from the very limits that are still in doubt.

---

## 🏗️ Architecture

Three SDK constraints decided the structure, all of them confirmed by reading the source:

1. **The SDK is not thread-safe at all** — the whole package contains no `threading` and no locks,
   and every motion method runs a 200 Hz blocking loop in the caller's thread, sharing one socket
   and one `MotorState`. ⇒ A single `GripperWorker` (`QThread`) owns the SDK; the GUI thread never
   touches it, and a cross-thread call raises `RuntimeError`.
2. **`control_mit_stream()` has no interruption hook** (`protocols/can_bus.py:342` is a bare
   `while`), and every convenience motion method ends up there. ⇒ **Those methods are not used**;
   the console sends `send_mit_frame` and calls `poll` on its own 5 ms tick, which is what puts the
   emergency-stop check inside the loop.
3. **`move_at_speed()` always appends 20 hold frames** (`gripper.py:1117`) per call. ⇒ "Move while
   echoing live" cannot be built by calling it in small slices; each slice would stall for 100 ms.

```text
src/litegrip_studio/
├── constants.py        # every policy number lives here, testable without Qt or the SDK
├── units.py            # mm↔rad, travel, limits, clamping (pure functions)
├── calibration.py      # calibration file parsing/validation/provenance/saving (pure + fs)
├── settings.py  telemetry.py  logging_setup.py  version.py  selftest.py
├── backend/
│   ├── __init__.py     # GripperBackend ABC + _claim() thread-ownership assertion
│   ├── real.py         # public SDK primitives only
│   ├── plant.py        # pure simulated plant: step(dt), no threads, no sleep
│   └── sim.py          # plant + real-time pacing + fault injection
├── core/
│   ├── commands.py     # frozen dataclass command set
│   ├── profile.py      # anti-windup rate-limited reference generator (pure)
│   ├── motion.py       # MotionFSM: IDLE/HOLD/SERVO/GRASP/HOLD_FORCE/FAULT
│   ├── calibration_fsm.py
│   └── worker.py       # GripperWorker(QThread): owns the backend, 200 Hz tick
└── ui/
```

The layering rule: **policy numbers** all live in `constants.py`; **pure logic** (`units`,
`profile`, `calibration`, `plant`, the state machines) depends on neither Qt nor the SDK nor
threads and is tested with plain pytest; only `backend/` touches the SDK; the Qt layer is thin.

The backend interface exposes only the primitives the state machine needs and **never an SDK
convenience method** — that is what makes the same state machine run on the real gripper and on
the simulation, so the simulation actually exercises the logic instead of standing next to it.

### The reference generator

Every tick anchors the reference to the **measured position**, which is the key to the anti-windup
behaviour. Recursing through an internal integrator instead would let the reference run to the
target while blocked, with the error and `kp·err` growing without bound — an infinite clamp:

```python
v_allow = min(speed_mm_s, sqrt(2*acc*max(rem - TOL_MM, 0)))   # stopping-distance limit
v_ref   = slew(v_ref, v_allow, acc*dt)                        # trapezoid
q_cmd   = measured + sign*min(v_ref*dt, rem)                  # anchored, never overshoots
```

The trapezoid costs three lines and buys a start and stop with no velocity step (no current spike),
a stopping distance (it never brakes at full speed on the servo alone), and a reference that cannot
overshoot.

### When a gap counts as an obstruction

Hit something while moving and the move must stop, and the evidence is **lost travel**: the
reference generator also integrates an ideal path ("where it should have got to by now",
`virtual_mm`, used only for comparison and never commanded, so it cannot wind up), and the
measurement failing to keep up with it means something is in the way. The window is 1 mm.

But that test **is a model**: it assumes the mechanism keeps up with the reference while cruising.
A mechanism that cannot — high friction, tight linkage, a load, or too small a `kp` for the speed
asked for — accumulates the same 1 mm while **moving perfectly normally**, and then stopping is a
fault rather than a safety measure: what the operator sees is "it moves twice and stops", not the
position they dragged to.

So for a plain position move (no force setpoint) the 1 mm also needs **corroboration** before it
counts as contact: the measured speed below 25 % of the reference speed (`CONTACT_STILL_RATIO`).
Because the test is a **ratio** and not an absolute speed it holds at 5 mm/s and at 150 mm/s alike;
a real obstruction collapses the speed and is still caught, and a mechanism that genuinely does not
move is caught by the stall counter and the **arrival timeout**, neither of which changed.

The number has a measured origin: in the simulation, a mechanism with 0.3 Nm of Coulomb friction
moving at 20 mm/s was declared "position not changing over time" after 2.7 mm while travelling at
**87 % of the commanded speed**. With the corroboration, the same move completes.
`tests/test_motion_fsm.py::TestWhetherAGapIsAnObstruction` pins both cases.

### Shutdown and safety

Shutdown has three layers, so the motor never stays enabled: `run()`'s `finally` →
`zero_torque→disable→disconnect`; `app.aboutToQuit` calling the same (idempotent); and
`SIGINT`/`SIGTERM`. The `terminate()` fallback is **inherently unsafe** — it can stop halfway
through a frame — and the log says so instead of pretending otherwise.

Three stop semantics, because the SDK's single `stop()` is not enough:

| Button | Behaviour | Motor |
| --- | --- | --- |
| Stop | FSM→HOLD at the current rad, reference reset | Stays enabled |
| Release | Sends `kp=0,kd=0,tau=0` continuously, back-drivable by hand | Stays enabled, back-drivable |
| **Emergency stop** (red / Esc) | `zero_torque()→disable()` and **latches**: every motion is refused until it is reset | Disabled |

The emergency stop travels on a `threading.Event` rather than the command queue — it must not be
queueable — and the tick checks it before the state machine. There is also a GUI watchdog: the GUI
sends a heartbeat every 500 ms, and a worker that does not see one for 3.0 s treats the console as
dead, zeroes the torque and disables the motor.

---

## 🛠️ Development

The tests need neither hardware nor an installed SDK:

```bash
./run_litegrip_studio.sh test
# equivalent to
QT_QPA_PLATFORM=offscreen python3 -m pytest tests/ -q
```

`pytest-qt` is not installed; the suite uses plain pytest with a session-scoped `QApplication`
fixture and `PyQt5.QtTest`, under `QT_QPA_PLATFORM=offscreen`. `conftest.py` puts both the SDK and
`src/` on `sys.path`, so the SDK does not have to be installed for the suite to run.

The `--selftest` mode runs pure logic only (unit conversions, calibration direction checks,
wrong-zero rejection, plant convergence, arrival, torque limits, emergency-stop disable) and
**imports no Qt at all** — the machine that needs it most is the one where Qt is broken. A
sub-interpreter test asserts exactly that.

### About the SDK dependency

The SDK is **deliberately not** a resolved dependency. It is a checkout beside this repository, its
one declared dependency `eclipse-zenoh` is never imported by library code, and the main path is
therefore to put the checkout on `PYTHONPATH` — which is what the launcher does. Installing it is
also possible:

```bash
python -m pip install -e ../lite-grip --no-deps
```

The tests that are about the SDK's surface skip themselves when no checkout is present, which is
why a CI run reports a non-zero skip count.

### Packaging

```bash
./run_litegrip_studio.sh build      # → dist/litegrip-studio
```

`--collect-data litegrip` is required: the SDK finds its factory calibration through
`dirname(litegrip.__file__)`, which still holds under `sys._MEIPASS`, but without that flag the
file never enters the artifact and the factory fallback dies with it.

`--collect-data litegrip_studio` is required for the same reason: the console's own copy of the
fallback is package data, which only puts it into a wheel, not into a PyInstaller artifact — and
without it a machine with no SDK checkout is left with no fallback at all. The console's selftest
checks that this file is where the code reads it from.

The version stamp is written into `_version.py` at build time and **removed from the source tree
when the build ends** (it is git-ignored as well). Leaving it behind would make every later
source run claim the number from the previous build, whose git hash may long since disagree with
the working tree. The stamp only lives inside the artifact, so a source run always reports
`+source`.

The number inside `_version.py` is **not the released version**. Releases are automated by
semantic-release from the commit history, the git tag is the only source of truth, and
`0.0.0-semantic-release` in `pyproject.toml` is a placeholder that must not be edited by hand.

---

## ⚠️ Known limitations

- **The SDK's own `calibrate()` is not offered in this version.** It has no stdin
  dependency, but it cannot be interrupted and drives the motors for about 24 seconds — better to
  offer only the two wizards on the calibration page (自动标定 and the two-point manual one) than a
  24-second window in which the emergency stop does nothing.
- The factory calibration is a **fallback, not a substitute**: used on another gripper it does not
  crash, it silently makes every millimetre wrong. The console carries its own copy of that file so
  a machine with no SDK checkout still has one, and on either copy the gate stops at `FACTORY`:
  the machine does not move until the operator ticks the acknowledgement on the calibration page.
  The page looks at the travel implied by the file's own scale, but only warns when it is wrong by
  orders of magnitude (outside
  `STROKE_MIN_MM` / `STROKE_MAX_MM`). That test **cannot** be tightened into a comparison with the
  measured travel: files written by the SDK and by earlier versions of the console all carry
  "nominal travel ÷ span", and that nominal is the SDK's own default rather than a measurement of
  the unit it was recorded on, so a 60 mm
  gripper and a 120 mm gripper write the same number — tightening it would make every file,
  including this machine's own, warn, and an alarm that is always on is no alarm at all.
  Telling which gripper a file belongs to is the job of the measured-angle frame check
  (`frame_mismatch`) and of a plausible range for the derived coefficient.
- The force limit is the 40 N mechanical rating; the theoretical 100 N (10 Nm × 10) appears only as
  a greyed-out, unselectable comment. Changing it means editing `constants.FORCE_MAX_N`, one place.
- **The mm conversion is derived from the two angles and the measured travel, not read from the
  file's `rad_to_mm`.** The SDK writes that field as "nominal travel ÷ span", so every calibration
  file on this machine claims a 120 mm coefficient no matter which gripper the two angles were
  recorded on. Loading such a file makes a gripper whose real travel is 85 mm read 120 mm, while
  the slider covers only part of it and neither end is reachable — which is where "a slice taken
  out of the middle of 0–120" came from. The two angles in the file are measurements and the
  coefficient is derived, so `units.derive_scale()` computes it:
  `(measured travel + SPAN_INSET_MM) / span`. On this machine that is
  `(85 + 1) / 1.681925 = 51.13 mm/rad`, so the two recorded limits span 86.00 mm, the slider's
  0…85 covers the whole travel, and the top of the range sits 1 mm inside the recorded open limit.
- The travel (`constants.DEFAULT_TRAVEL_MM`, 85.0 mm) is the **measured value for this gripper** and
  is a constant, not a setting. It decides two things: the mm coefficient and the slider's range, so
  a console told the wrong one reports every millimetre wrong while looking healthy. It was editable
  on the calibration page until 2026-09-28, when 10 mm was typed into it: that derives 8.0 mm/rad,
  below `RAD_TO_MM_MIN`, so the plausibility check turned a good calibration into a problem and the
  gate refused motion — a console that could not move and could not say why. There is no flag, no
  setting and no widget for it now (`--travel-mm` is gone; the retired `calibration/travel_mm` is
  deleted from an existing ini when the console opens), and moving this console to another unit means
  editing the constant and re-probing. Saving a calibration writes **this derived coefficient** into the file (the
  SDK writes the file from its own config, and the console pushes the limits into that config
  before saving), which is the number the console actually moves with: reading it back is
  self-consistent, and the cross-check (the coefficient the SDK applied equals the one in the file)
  means something. The coefficient the probe computed itself stays in `raw` for display and
  comparison; it describes "the nominal travel that was set at the time ÷ the span", not how wide
  this gripper is.
- **Why the top of the range is inset by 1 mm**: the probe pushes the open limit out under 40 N of
  pressure and the linkage gives about 1 mm under that force, so the recorded open limit is slightly
  past the mechanical end. Commanding the recorded limit directly presses the stop on every full
  opening; 1 mm inside reaches the same physical position without the press. The closed end is not
  inset: what the probe presses against there is the two fingers touching, which has no margin to
  begin with, and 0 is the calibrated zero.

---

## 📋 Repository standards

This repository follows the shared NEXFORM ROBOTICS repository standards: the agent operating rules
in [AGENTS.md](AGENTS.md), Conventional Commits, and automated semantic-release versioning on every
merge to `main`.

---

## 📄 License

Licensed under the **Apache License 2.0** — see [LICENSE](LICENSE).

Copyright © 2026 NEXFORM ROBOTICS.
