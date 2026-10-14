# LiteGrip Studio design notes

Why the console is built the way it is: the SDK constraints that fixed its structure, the control
laws behind the slider and the plots, the safety behaviour, and the reasoning behind the calibration
rules. Read this if you are changing the console — [README.md](README.md) is the manual for
installing and running it.

---

## Architecture

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
the simulation, so the simulation actually exercises the logic instead of standing next to it. The
one named exception is `write_zero`, which is 标零: it never runs on the control path, so the
reason the other convenience methods are banned (they block inside a loop with no abort hook) does
not apply to it.

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

So the 1 mm also needs **corroboration** before it counts as contact, on any move: the measured
speed below 25 % of the reference speed (`CONTACT_STILL_RATIO`). Because the test is a **ratio**
and not an absolute speed it holds at 5 mm/s and at 150 mm/s alike; a real obstruction collapses
the speed and is still caught, and a mechanism that genuinely does not move is caught by the stall
counter and the **arrival timeout**, neither of which changed. A move that carries a force setpoint
leans on that corroboration hardest: it is given no `lead`, so the push behind its approach is the
tick's own `kp·v·dt` and whatever friction eats into that shows up as tracking lag, for the gap to
integrate.

The number has a measured origin: in the simulation, a mechanism with 0.3 Nm of Coulomb friction
moving at 20 mm/s was declared "position not changing over time" after 2.7 mm while travelling at
**87 % of the commanded speed**. With the corroboration, the same move completes. The same defect
reaches the operator as "at low speed the jaws race to the grasp position, and 放开 does not open
1 cm": 0.05 Nm of Coulomb friction — five times what the simulated unit's own free travel draws —
at 10 mm/s hands the grip over **35.4 mm short of the object**, force mode then drives the jaws at
up to **115 mm/s** against that 10 mm/s slider, and 放开, which opens from the pose the grip was
made at, runs to the open stop, **45 mm clear** of the object. With the corroboration the grip is
made at the object and 放开 opens `RELEASE_OPEN_MM`.
`tests/test_motion_fsm.py::TestWhetherAGapIsAnObstruction` and
`TestAGraspAtTheSlidersSlowEnd` pin both cases.

### A grip is made where the gap detector is blind

The gap is only integrated while the trajectory is at **cruise**, because during the acceleration
ramp the plant legitimately lags its moving reference by a few millimetres — an artefact of
accelerating, not of obstruction. A close leaves cruise `speed²/(2·acc) + TOL_MM` before its
target: 1.2 mm at 25 mm/s, 3.5 mm at 50, because that is where it starts braking. Meet the object
inside that last stretch and the primary detector has already stopped looking, which is where a
grip on a stiff or a thin object is made.

So a move that **carries** a force setpoint has **three** contact channels, and they cover each
other: the gap, the measured torque (`CONTACT_TAU_NM`, and the only channel that works off an old
reading — a torque is what the drive applied, whenever it was read), and **stillness** —
`ProfileOutput.stalled`, the trajectory asking for motion for 0.1 s while the jaws deliver less
than `STALL_RAD` of it. Stillness takes no account of speed, so it reaches the slow end where the
torque channel cannot, and it costs one thing: the approach gain's press for those 0.1 s.

The numbers come from the simulated plant. A 20 N grasp at 25 mm/s against an object 1 mm short of
the closed stop pressed **9.2 N for 5.3 s** on the gap and torque channels alone — the gap blind
for the reason above, and 9.2 N well under the 15 N the torque channel needs — and then jumped to
the setpoint on the move deadline. With stillness it hands over in **0.25 s**. That press is
`kd·v_ref`-dominated, so it is the speed slider that sets it, not the setpoint: 9.2 N at 20 mm/s,
11.4 at 25, 18.8 at the 50 mm/s default.
`tests/test_motion_fsm.py::TestAGraspThatMeetsItsObjectInTheLastMillimetre` pins the hand-over, the
requirement that the reading be current (stillness in a cached position is not evidence), and that
a free approach is not mistaken for contact.

A move that **carries** a force setpoint is also the one exception to what the arrival timeout does.
It exists to stop a position that is not converging, and a grip that is not converging has already
reached the state it was aiming for: the feed-forward alone. Taking the position hold instead
leaves the jaws with nothing but whatever the object's own stiffness offers at the pose they froze
at, which on the bench is a grip that sags away from its setpoint a few seconds after it took hold.
`tests/test_motion_fsm.py::TestTimeout` pins both endings.

## How a grasp is commanded

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

### The two frames of a grasp

The frame a held grip sends carries **no gains at all** — `kp=0` and `kd=0` — so the torque in it is
the setpoint and nothing else. Any gain adds a term that depends on something other than the
setpoint, and the grip stops being the number on the slider: a position term makes it depend on how
far the jaws sank into the object, a velocity term on how fast they were still moving when they
arrived. One consequence is worth knowing before **Let go**: with nothing holding the pose the grip
was made at, the setpoint is reached by driving the jaws a little further *into* the object, so they
hold it from a position that is not quite where they met it.

Nor is the setpoint switched on. The torque climbs to it at `FORCE_RAMP_N_S`, 20 newtons per second,
starting from the torque already in flight: a grasp that met its object and handed over at the ~10 N
the approach had pressed with reaches 20 N half a second later, as a rise the operator can watch
rather than a jump. The rate is the constant on purpose — a ramp specified as a *time* is at its
steepest in its first tick, which is a step with a slow tail, and the mechanism answers a step with
a lurch.

A grasp is two frames, not one, and the force plot draws both. Until the jaws meet something the
approach is a **position** frame carrying the slider's speed and `tau_ff = 0`: the console commands
no force at all, and the drive computes `kp·(q_cmd−q) + kd·(dq_cmd−dq)` itself. That torque is the
first rise on the plot, and it is bounded by the setpoint's own torque. The press a frame applies
when it meets something is `kp·(v·dt + lead) + kd·v`, and the two terms the console supplies — the
damping the approach travels with (`MotionFSM._approach_kd`) and the lead it is given
(`MotionFSM._approach_lead_rad`) — are sized so that whole expression fits inside the budget the
setpoint names, at any speed. The slider keeps deciding how fast the approach travels; above the
speed whose damping alone would spend the budget it is the damping that yields, not the speed. A
*plain* move carries no setpoint and keeps the slider's speed and the operator's damping both. The
second rise is the ramp above, from that press to the setpoint. Two rises on the plot are expected,
then; only the first was ever unbounded, and it no longer is.

The hand-over is logged: the pose the grip was made at, how far the jaws had travelled to reach it,
and which of the three contact channels declared it. The pose alone does not say what happened — a
grip made at the object is a grasp, and the same reading taken a millimetre into a close is a
mechanism that had not broken away from rest yet being read as an obstruction, after which force
mode drives the jaws the rest of the way at the ramp rather than the slider. The channel is what
tells those apart after the run, and the log is the only place that survives it.

Do not lower `FORCE_MIN_N` or `SPEED_MIN_MM_S` to chase a very small setpoint. `FORCE_MIN_N` is the
one that decides what the console serves: `clamp_force_setpoint` turns a setpoint under it into the
floor rather than serving it as asked, and the spinbox does not offer one at all, so the number on
the screen is the number pressed either way. The floor is there because the approach's entire press
budget *is* its own setpoint's torque, this mechanism spends about 0.6 N·m of that getting its jaws
moving, and a budget under that presses with less than the friction it has to overcome. The jaws then
never leave the open stop and the stillness channel reads that rest as contact. Measured on the plant
at the maximum slider speed, with no floor: a 0.1 N grasp handed over at 85.01 mm — the open stop,
without having travelled — and a 1 N one at 84.90 mm, 44.9 mm short of an object at 40 mm, after
which force mode drew the jaws the rest of the way and **Let go**, which measures from the pose the
grip was made at, opened 45 mm further than it promises.

`SPEED_MIN_MM_S` is the other half of the same rule: the slowest speed that breaks the jaws away from
rest, and lowering it trades the one case it covers for a stall that every contact detector reads as
an obstruction. The speed a setpoint still decides is the one whose tick's own step `kp·v·dt` would
spend the whole budget — one term of the press is not the console's to withhold, since the command
leads the measurement by the tick's own travel whatever damping or lead it carries — so a setpoint
whose budget cannot cover that step at the speed asked for has the approach slowed until it can
(`MotionFSM._tick_press_cap_mm_s`). With the floor in place that lever no longer bites anywhere the
slider reaches: the cap at 6 N is 317 mm/s at the bench file's 73.46 mm/rad, and 202 mm/s on the
simulated unit's 46.73, against a slider that stops at 150. A gripper with more millimetres per rad,
or a lowered floor, puts it back in play.

### The ends of the range and Let go

Both the close and the open command go through the rate-limited reference generator rather than
jumping, and Stop or the emergency stop can take over at any moment while the button is held. The
tooltip names the exact millimetre figure and refreshes as soon as a calibration arrives — before
one is read the tooltip **quotes no number**, because at that point no millimetre figure belongs to
this gripper. The tooltip quotes the **range** (`max_stroke_mm`), not the span the calibration file
records: once mm/rad has been derived from the measured travel the two differ, and quoting the span
sends people looking for travel that cannot be reached.

**Let go** measures from the **measurement**, not from the last command's target — a grasp drives to
0 mm under a force cap, so its target is always the closed end and "target + 10 mm" is a command
straight back into the object. There is one exception, and it is the case the button exists for:
while a grip is held the reference is the pose the grip was **made** at. A held force carries no
position term, so the setpoint is reached by driving the jaws *into* the object until its own
stiffness balances the torque — millimetres of travel against a compliant object, a fraction of one
against a stiff object. Measuring from where the jaws are now would spend part of the 10 mm on that
draw-in and leave them resting on the object, and the button would open less than it says; measured
from where the grip was made, the 10 mm is the clearance the object gets. The draw-in is logged, so
the number is visible. At the top of the range it is a move of zero length (`move_to_mm` clamps)
rather than an error.

## The force plot's autoscale

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

## Calibration

### What the console decides for itself

The process has several traps, which is why the console reads the files and decides the provenance
itself rather than trusting the SDK's return value:

- **It does not say which file it read.** `load_calibration()` returns `True` for the named file
  and for the factory default alike; only one `log.info` line distinguishes them. On real hardware
  that means millimetre readings that are systematically wrong with nothing on screen to show it.
- **The SDK does not require a calibration.** Neither `connect()` nor `enable()` loads one. Without
  it you get `GripperConfig`'s defaults, `pos_closed_rad=0.0 / pos_open_rad=1.14`, whereas on real
  hardware the closed value is the **larger** one (the fallback vendored with the SDK has
  `0.029564 / -1.141`). The mapping `mm = (pos_closed_rad - position_rad) * rad_to_mm` then runs
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

The gate has two states: `READY` (a calibration is in effect and it validated) and `BLOCKED`
(no file at all, a file that failed validation, a named file that is not there, a zero that
belongs to another machine, or a probe result that has not been written out yet). There is
nothing to tick and no override: a state whose answer is always the same is not a state, and the
factory file is what the console runs on by default rather than something it has to be talked
into. **The gate logic lives in the worker, not the UI**: the UI can disable widgets, but the
refusal has to happen before a frame is sent.

One more thing: the calibration file is loaded by the worker **on connect**, not at start-up, so
the provenance and the gate always describe the file that was actually handed to the SDK rather
than the one a command line mentioned at the time.

Changing the calibration invalidates every millimetre held in the loop. The position is **read
again** from a new frame — the reading was originally converted by the backend using the limits in
force at the time — and a target being held or moved is **re-anchored** in place, because an
intention expressed in the old frame's millimetres has no honest conversion. Without those two
steps, loading a *correct* calibration sends the axis straight for the other end of the new travel.

The simulator does not remember the path, and the backend decides that rather than the window: a
remembered path is read on every later launch, including a real one, so remembering
`litegrip_calibration.sim.json` would have the bench gripper measuring on simulated geometry.
`SimBackend.remember_calibration_path` is `False` for that reason.

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
numerator is the travel the operator measured — see "Known limitations" in [README.md](README.md).

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

## Motor zero and the fallback file

**电机标零 re-measures nothing.** The span on file keeps its length; both of its angles, and
therefore every millimetre computed from them, are now offset by the angle that was zeroed.
Afterwards the same jaws are read as a different place.

**The gate will not catch it.** A zero taken at a stop moves the frame by well under
`CALIB_MISMATCH_RAD` (0.15 rad), so the console goes on moving by millimetres that are now wrong.
That is why the warning is stated in so many words instead of the motion being blocked.

The warning is dismissed by the operator and by nothing else, through the 已了解 button in the
banner. The worker does not retract it, because nothing in the loop can: a calibration measured
against the old offset does not become correct again on its own. Immediately after the write the
axis is re-anchored to a fresh measurement, so the FSM is never left executing a target expressed in
the frame that just stopped existing.

标零 needs the console connected and the motor enabled — a DM drive ignores `0xFE` precisely *because*
it is disabled, so the refusal for a disabled motor is the one an operator meets. It is also refused
while a probe is running, which owns the axis and would have its next frame aimed at a frame that
moved underneath it.

The simulated backend refuses 标零 rather than faking the frame shift. The refusal is deliberate,
not an omission: the plant's `q` is at once the mechanism's physics *and* the angle it reports, so a
simulator that shifted only the report would integrate against a state that no longer matches what
it says — disagreeing with the hardware in exactly the way this feature is about.

### The page's travel sanity check

The page looks at the travel implied by the file's own scale, but only warns when it is wrong by
orders of magnitude (outside `STROKE_MIN_MM` / `STROKE_MAX_MM`). That test **cannot** be tightened
into a comparison with the measured travel: files written by the SDK and by earlier versions of the
console all carry "configured stroke ÷ span" — the SDK's 120 mm nominal in files written before
2026-10-10, its measured 85 mm default since, whatever the caller had set — and that is not a
measurement of the unit the file was recorded on, so a 60 mm gripper and an 85 mm gripper can write
the same number — tightening it would make every file, including this machine's own, warn, and an
alarm that is always on is no alarm at all. Telling which gripper a file belongs to is the job of
the measured-angle frame check (`frame_mismatch`) and of a plausible range for the derived
coefficient.

### Why the mm conversion is derived, not read

The mm conversion is derived from the two angles and the measured travel, not read from the file's
`rad_to_mm`. The SDK writes that field as "configured stroke ÷ span" — the 120 mm nominal in files
written before 2026-10-10, its measured 85 mm default since — so it reports the stroke that was set
rather than the unit the two angles came from. Loading such a file makes a gripper read that stroke
across its jaws, while the slider covers only part of it and neither end is reachable, which is
where "a slice taken out of the middle of 0–120" came from. The two angles in the file are
measurements and the coefficient is derived, so `units.derive_scale()` computes it:
`(measured travel + SPAN_INSET_MM) / span`. On this machine that is
`(85 + 1) / 1.681925 = 51.13 mm/rad`, so the two recorded limits span 86.00 mm, the slider's
0…85 covers the whole travel, and the top of the range sits 1 mm inside the recorded open limit.

The travel (`constants.DEFAULT_TRAVEL_MM`, 85.0 mm) is the **measured value for this gripper** and
is a constant, not a setting. It decides two things: the mm coefficient and the slider's range, so
a console told the wrong one reports every millimetre wrong while looking healthy. It was editable
on the calibration page until 2026-09-28, when 10 mm was typed into it: that derives 8.0 mm/rad,
below `RAD_TO_MM_MIN`, so the plausibility check turned a good calibration into a problem and the
gate refused motion — a console that could not move and could not say why. There is no flag, no
setting and no widget for it now (`--travel-mm` is gone; the retired `calibration/travel_mm` is
deleted from an existing ini when the console opens), and moving this console to another unit means
editing the constant and re-probing. Saving a calibration writes **this derived coefficient** into
the file (the SDK writes the file from its own config, and the console pushes the limits into that
config before saving), which is the number the console actually moves with: reading it back is
self-consistent, and the cross-check (the coefficient the SDK applied equals the one in the file)
means something. The coefficient the probe computed itself stays in `raw` for display and
comparison; it describes "the stroke that was set at the time ÷ the span", not how wide this
gripper is.

**Why the top of the range is inset by 1 mm**: the probe pushes the open limit out under 40 N of
pressure and the linkage gives about 1 mm under that force, so the recorded open limit is slightly
past the mechanical end. Commanding the recorded limit directly presses the stop on every full
opening; 1 mm inside reaches the same physical position without the press. The closed end is not
inset: what the probe presses against there is the two fingers touching, which has no margin to
begin with, and 0 is the calibrated zero.

## Shutdown and safety

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

## The CAN link

Pressing 连接 looks at `can0` first and intervenes only when the interface really is wrong — down, or
at the wrong bitrate — and only in that one place. Each of the following is a decision, not an
implementation detail:

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
- **Not every adapter implements `restart-ms`**, and the ones that do not say so: the gs_usb
  clone on this bench (`1d50:606f`) answers "Device doesn't support restart from Bus Off." When
  that specific refusal comes back, the console drops the option, configures without it, and
  brings the interface up anyway — one authorization, and a log line saying the difference. On
  such an adapter a bus-off controller does **not** recover by itself; the next Connect
  reconfigures it.
- **A USB-CAN adapter can lose its USB endpoint table**, and then no configuration command
  raises it: `sudo ip link set can0 up` answers "RTNETLINK answers: No such file or directory"
  (ENOENT) while `ip -details link show can0` reads perfectly normal. This is the kernel refusing
  a transfer whose endpoint is no longer in the device's table; it is not about the bitrate, and
  nothing userspace can set on the interface repairs it. Only a fresh probe of the device does,
  which is why **unplugging the adapter and plugging it back in repairs it** — and so does
  `sudo modprobe -r gs_usb && sudo modprobe gs_usb`, without touching the cable. The console does
  the second one by itself when Connect hits this, then configures and raises the interface again.
  That reload resets every `gs_usb` interface on the machine, including adapters this console is
  not using — and it is attempted for `gs_usb` only, because unbinding an unknown driver is not
  something to do on a guess. When it still does not come up, the message tells you to replug
  rather than quoting an `ip link` command that cannot work.
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

## Themes

The console opens dark, which is what a bench under a machine tends to be, and is what it opened
in before there was a choice. The button beside the emergency stop switches it to light and back,
and the choice is written to `~/.litegrip/litegrip_studio.ini` under `view/theme`.

Nothing is written until you switch, so "nobody has chosen" stays distinguishable from a choice
that happens to match the default.

The palette is litearm-studio's in both themes: the same ink ramp, the same semantic fill/edge
pairs, and the same type scale. What is this console's own is the meaning layer — blue for the
position that was *commanded*, green for the one the jaws *measured* — and those two are the same
colours on the slider, in the readout and in the plots.

## The SDK: vendored, not a dependency

The SDK is **vendored, not a dependency**: `src/litegrip` holds litegrip-python at a pinned commit,
so a clone runs with nothing installed beside it. Provenance, the file list, the tree hash and the
re-vendor commands are in [`src/litegrip/VENDORED.md`](src/litegrip/VENDORED.md).

Vendoring rather than installing is deliberate. The SDK's one declared dependency, `eclipse-zenoh`,
is never imported by its library code, so an install step would buy nothing — and a distribution of
the same name would be a *different* SDK: the numbers the console falls back to are read from the
file beside whichever `litegrip` was imported, so the wrong one loads a different set of numbers
without saying so.

Do not edit anything under `src/litegrip` other than the factory calibration this repository
deliberately overrides. A local edit survives only until the next re-vendor, which copies each file
wholesale; a change that belongs there goes upstream. `VENDORED.md` records a hash of the whole
tree and `tests/test_vendored_sdk.py` recomputes it, so an edit in place fails the suite.

To run the console or the suite against SDK HEAD instead, set `LITEGRIP_SDK_PATH` to that checkout.
It goes in front of the vendored copy.

## Packaging internals

`--collect-data litegrip` is required: the SDK finds its factory calibration and its two direction
templates through `dirname(litegrip.__file__)`, which still holds under `sys._MEIPASS`, but without
that flag the files never enter the artifact and the factory fallback dies with them. `--selftest`
inside the built binary checks that the file is where the code reads it from.

The version stamp is written into `_version.py` at build time and **removed from the source tree
when the build ends** (it is git-ignored as well). Leaving it behind would make every later
source run claim the number from the previous build, whose git hash may long since disagree with
the working tree. The stamp only lives inside the artifact, so a source run always reports
`+source`.

The stamp starts from the nearest `v*` git tag, so an artifact built from `v0.8.2` calls itself
`0.8.2.<commit count>+g<sha>.<date>` and can be matched to a release at a glance. A tree with no tag
to ask — a shallow export, a source tarball — falls back to `BASE_VERSION` in `version.py`, which is
`0.0.0`, so a source run reports `0.0.0+source`.

Neither of those numbers is **the released version**. Releases are automated by semantic-release
from the commit history, the git tag is the only source of truth, and `0.0.0-semantic-release` in
`pyproject.toml` is a placeholder that must not be edited by hand.
