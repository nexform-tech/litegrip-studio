# LiteGrip Studio

**English** | [简体中文](README.zh-CN.md)

**LiteGrip Studio** is the operator console for the LiteGrip adaptive two-finger gripper. You drag
a slider to move the jaws, press a button to open or close them, grasp with a force limit you set,
and watch position, force, temperature and faults live. It talks to the gripper over SocketCAN by
importing the LiteGrip Python SDK in the same process — there is no cross-language bridge.

With no gripper attached it runs a built-in simulated one, and **every control still works**: the
simulation is not a stand-in for the interface, it runs the same motion state machine the hardware
does. You can install it, try every page and run the test suite with no machine.

The window is four tabs — 控制, 状态, 曲线, 标定 — in Simplified Chinese, and this manual quotes the
labels as they appear on screen.

## Related repositories

| Repository | Role |
| --- | --- |
| [litegrip-python](https://github.com/nexform-tech/litegrip-python) | Python SDK — vendored under `src/litegrip` |
| [litegrip-cpp](https://github.com/nexform-tech/litegrip-cpp) | C++ SDK |
| [litegrip-docs](https://github.com/nexform-tech/litegrip-docs) | Product documentation |
| [litegrip-ros2](https://github.com/nexform-tech/litegrip-ros2) | ROS 2 driver |

---

## 1. Install

Python 3.10 or newer, and three libraries, into the interpreter your `python3` points at:

```bash
python3 -m pip install PyQt5 pyqtgraph numpy
```

Add `python3 -m pip install pytest` to run the test suite, and
`python3 -m pip install pyinstaller` to package. The SDK needs no install — it is vendored under
`src/litegrip`.

Two things not to do:

- **Do not run `pip install -e .`.** The version in `pyproject.toml` is semantic-release's
  placeholder, which is not a valid PEP 440 version, so setuptools refuses to build.
- **Do not install a `litegrip` distribution from PyPI.** The console reads its fallback calibration
  from the `litegrip` it imported, so a package of the same name would load a different set of
  numbers without saying so.

## 2. Run it

```bash
./run_litegrip_studio.sh sim        # the console, against the built-in simulated gripper
./run_litegrip_studio.sh gui        # the console, against a real gripper on can0
./run_litegrip_studio.sh selftest   # self-check: no Qt, no hardware
./run_litegrip_studio.sh test       # the test suite
./run_litegrip_studio.sh build      # one-file executable in dist/
```

Start with `sim`: it needs no hardware and exercises the same code.

`selftest` runs pure logic only — unit conversions, calibration direction, rejection of a zero that
belongs to another machine, plant convergence, arrival, torque limits, the emergency stop — and
imports no Qt at all, so it still works on a machine where PyQt5 is broken. It prints one `PASS` or
`FAIL` per check and a final count.

Nothing has to be installed or checked out beside this repository: the SDK is vendored under
`src/litegrip`, and the launcher puts `src/` on `PYTHONPATH`.

The launcher uses `python3`. Where the dependencies live in a virtualenv, point `PYTHON_BIN` at it:

```bash
python3 -m venv .venv
.venv/bin/python3 -m pip install PyQt5 pyqtgraph numpy
PYTHON_BIN=.venv/bin/python3 ./run_litegrip_studio.sh sim
```

**`ModuleNotFoundError: No module named 'PyQt5'` means the dependencies went to a different
interpreter.** `./run_litegrip_studio.sh -h` prints the one the launcher uses (`当前解释器：…`);
install into that one, or point `PYTHON_BIN` at the one you installed into.

Extra arguments are passed to the program unchanged, for example
`./run_litegrip_studio.sh sim --log-level DEBUG`; `-h` lists every subcommand.

### Options

| Option | Default | Effect |
| --- | --- | --- |
| `--backend {sim,real}` | `real` | Drive the simulated plant instead of hardware |
| `--can-channel NAME` | `can0` | SocketCAN interface to use |
| `--can-bitrate INT` | `1000000` | Bitrate to configure the interface to on connect |
| `--no-can-setup` | off | Do not prepare the interface on connect, and raise no authorization prompt |
| `--can-id INT` | from the calibration file | Motor CAN ID |
| `--mst-id INT` | from the calibration file | Host CAN ID |
| `--calibration PATH` | the remembered path, then the bundled file | Calibration file to load |
| `--log-level LEVEL` | `INFO` | Console log level; the log file always records `DEBUG` |
| `--log-file PATH` | `~/.litegrip/logs/litegrip-studio.log` | Where the log is written |
| `--selftest` | off | Run the Qt-free, hardware-free self-check and exit |
| `--version` | | Print the version and exit |

## 3. Connect to a real gripper

**You do not have to bring the interface up yourself.** Press 连接 and the console looks at `can0`,
intervening only when the interface really is wrong:

- Privilege is escalated through `pkexec`, the desktop's own authorization dialog. The console never
  sees the password, so it cannot write it into a log either.
- An interface that is already configured raises no dialog and runs no privileged command, so
  bringing `can0` up by hand does not make every connection prompt you.
- Configuration adds `restart-ms 100`: the controller reopens itself 100 ms after going bus-off
  instead of staying down forever. **Not every adapter implements it.** When one refuses that
  option, the console drops it, configures without it, and says so in the log — and on such an
  adapter a bus-off controller does not recover by itself, so the next 连接 reconfigures it.
- A CAN FD interface is reported, never changed: the SDK detects FD and works with it, whereas
  switching the interface back to classic CAN would change every other node on that bus.
- A dismissed authorization prompt, a `can0` that does not exist, an `ip` that fails: **the
  connection attempt continues anyway**, and the log and the banner say what happened.

If a unit file or a script owns the interface and you do not want the prompt, pass `--no-can-setup`.
For a bitrate other than 1 Mbit, pass `--can-bitrate`.

To do it by hand, the equivalent commands are:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 restart-ms 100 fd off
sudo ip link set can0 up
./run_litegrip_studio.sh gui
```

If the middle command fails with "Device doesn't support restart from Bus Off.", your adapter does
not implement `restart-ms` — drop that one option and run the other two.

If the last command fails with "RTNETLINK answers: No such file or directory" while the first two
worked, the adapter has lost its USB endpoint table and no `ip link` command will bring it back.
Unplug the adapter and plug it back in, or run `sudo modprobe -r gs_usb && sudo modprobe gs_usb`,
which does the same thing without touching the cable — and which takes every `gs_usb` interface on
the machine down with it. The console does this by itself when 连接 hits it.

## 4. Calibrate before you move anything

The console starts on the **factory calibration file bundled with the SDK**, which was recorded on
the unit it ships with. Another gripper of the same design is not a different machine: the readings
are the right order of magnitude and the ends of the travel are roughly where you expect them. What
differs between two units is their manufacturing tolerance — the jaws, the linkage, the encoder
zero — so a reading can be a little off, and removing that is what a calibration is for.

So on real hardware, run 自动标定 from the 标定 tab before you rely on the numbers. Until a
calibration belongs to *your* gripper, treat what is on screen as close rather than exact.

If the calibration page shows 运动闸门：已阻断 (`BLOCKED`), nothing will move until you deal with it.
That is the design: the console refuses motion rather than driving by numbers it cannot vouch for.
See [Calibration](#calibration).

---

## The window

Four tabs, plus the connection bar that is always visible, the emergency-stop button, and the log
dock.

| Tab | What is on it |
| --- | --- |
| 控制 | The position slider, the motion buttons, speed and grasp force |
| 状态 | Live readings, temperatures, faults, link health, and the motor maintenance action |
| 曲线 | Position and force plots |
| 标定 | Calibration: the two probes, the file actions, the gate |

### The bar that is always there

**连接与电源** holds six buttons and two status lights:

| Button | What it does |
| --- | --- |
| 连接 | Connect to the gripper (becomes 连接中… while it works) |
| 断开 | Disconnect |
| 使能 | Enable the motor — nothing moves before this |
| 失能 | Disable the motor |
| 清除故障 | Clear a latched fault |
| 急停复位 | Reset the latched emergency stop |

Each one is enabled only when its command could actually be accepted, so a greyed-out 使能 means
the console is not connected yet, not that the button is broken.

**急停 (Esc)** zeroes the torque and disables the motor immediately, and **latches**: every motion
is refused until you press 急停复位. It is on the **Esc** key, and that works with a spinbox focused
and with a dialog open — the one shortcut that is deliberately allowed to interrupt anything.

### 控制 — moving the jaws

**位置（拖动即控制，松手执行）** is the slider. Drag it to set a target; the command is sent when
you release. Two markers move with it:

- the **cyan hairline** is where the jaws are heading — the last command;
- the **green triangle** is where they actually are, from telemetry.

The read-out beside it shows 实际位置, 目标 and 误差. **While the jaws are holding something the
read-out shows the actual position only.** A grasp drives to 0 mm under a force cap, so its target
is always the closed end and its error is the width of the object — both true, and side by side they
read as a move that has gone badly wrong.

| Control | What it does |
| --- | --- |
| 全部闭合 | Travel to the closed position in one press |
| 全部张开 | Travel to the widest opening in one press |
| 拖动时实时跟随 | Send the target while you drag, instead of on release |
| 停止（保持位置） | Stop, and hold the angle the jaws are at |
| 零重力（可手掰） | Zero stiffness and zero torque: the jaws can be pushed by hand with the motor enabled |

**速度** is the travel speed, 5–150 mm/s and 50 mm/s by default.

**夹持力** is the force limit for a grasp, 3–40 N. The box turns orange above 35 N and red at 40 N,
which is the mechanism's rating rather than a target to aim at. The lower end is the smallest
setpoint a grip actually works at. A grip is the SDK's force-budgeted `grasp()`, whose approach
spends only 0.9 × the setpoint, and that budget goes first on breaking the jaws away from static
friction: on this bench a 2 N grasp (approach 1.8 N) never gets them moving, while a 3 N one
(approach 2.7 N) reaches the object. Ask for less and the box stops you at 3 N. Two buttons sit
with it:

| Button | What it does |
| --- | --- |
| 夹取 N N | Close the jaws onto an object and hold it at the set force |
| 放开 | Open 10 mm clear of the object being held — measured from the pose the grip was made at, so the 10 mm is the clearance the object actually gets |

Both 停止（保持位置） and 零重力（可手掰） stay available when the gate is `BLOCKED`. Closing,
opening, grasping and releasing are refused, so with a bad calibration the ways to move the jaws are
零重力 and your hands.

### 状态 — readings and faults

**实时读数** shows 位置, 速度, 夹持力, 力矩, 速度参考, 误差, MOS 温度 and 线圈温度.
Temperature rows turn amber 10 °C below the limit and red at it (70 °C MOS, 80 °C coil).

**链路与回路** shows 运动状态, 已抓取, 状态帧, 帧速率, 链路静默, tick 耗时, tick 超时 and 错误码.

**连接与故障** shows a link light and a fault light. When a fault is latched, the hint under it says
what to check: undervoltage → the 24 V supply; overcurrent → a mechanical jam; over-temperature →
wait for the MOS or the coil to cool.

**维护** holds the one maintenance action, **电机标零…**. See [Calibration](#calibration).

### 曲线 — plots

Two plots share a time axis, over the last 10 seconds:

- **位置 (mm)** — 实测 (solid) against 目标 (dashed). The axis is scaled to the calibrated travel.
- **夹持力 (N)** — 力. The axis fits itself to the data on screen, so a 12 N grasp fills the plot
  instead of hiding in the bottom third of a 40 N axis.

**暂停** freezes them and 清空 empties them; resuming clears the buffer and starts drawing again,
which the note under the plots says.

### The rest of the window

- **The alert banner** above the tabs carries one alert at a time. Alerts that describe a condition
  — a fault, an emergency stop, a refusal — are taken down again when the condition stops holding,
  so a console that has recovered stops looking broken. Alerts that report a single event — a save
  that failed, a probe that ended — stay until the next one replaces them. Everything, including
  successes, is in the log.
- **日志** at the bottom is the log dock. **显示调试信息** adds debug lines to what the console
  shows; the file always records them either way.
- **深色 / 浅色** in the status bar switches theme. The console opens dark. Nothing is written until
  you choose, so "nobody has chosen" stays distinguishable from a choice that matches the default.
- The status bar also shows **位置** and **闸门** — the gate word, `READY` or `BLOCKED`.

---

## Calibration

### What the console runs on

There are two sources, and the console resolves which one applies itself:

1. **A file you named** — `--calibration`, the path remembered in the settings file, or
   `$LITEGRIP_CALIB`. **This is the file that describes your gripper**, and naming one is the only
   way off the default.
2. **The calibration file bundled inside the SDK** (`litegrip/factory_calibration.json`), which is
   what the console runs on when you have named nothing.

That bundled file carries **this bench's hand-recorded values**, which upstream adopted into its own
copy; this repository overrides nothing, and `src/litegrip/VENDORED.md` records where the file comes
from.

A named file that is missing, or that exists but cannot be used, is reported as such and **stops
there** — the console does not quietly fall back to the factory numbers, because moving on numbers
from a file you did not choose, with nothing on screen to say so, is the worst available outcome.

### The gate

**运动闸门** on the calibration page, and **闸门** in the status bar, read `READY` or `BLOCKED`.

`BLOCKED` means the console will not send a motion: there is no calibration at all, the file failed
validation, a named file is not there, the calibration belongs to a different machine, or a probe
result has not been written out yet. The page says which, and it is checked before any frame is
sent rather than in the widgets.

With the gate shut, 停止 and 零重力（可手掰） still work — 零重力 is how you move the jaws when the
file is the thing that is wrong.

### 自动标定 — measure the travel

Needs the motor enabled. Press **开始自动标定** and the console drives the jaws into the open stop
and then into the closed one, asking you to confirm each with **已到张开极限，确认** and
**已到闭合极限，确认**. The progress strip under the page shows the phase (正在顶向张开极限,
正在退回, 正在顶向闭合极限) and the live 实测位置 in both millimetres and radians. **取消标定**
stops it.

The probe presses against the hard stops, which it has to in order to find them, so the jaws will
push into both ends of the travel. It measures the distance between the hard stops, and it is the
one action allowed to move before a calibration exists and to travel outside a calibrated range —
it is the step that produces one.

### 手动两点标定 — record two points by hand

For when the stops are not reachable or not detectable — a stiff linkage, a drive that will not take
a probe step. **开始手动标定** puts the axis in zero gravity and says so, and the console takes it
back on every way out, including a cancel, because your hands are on the jaws for all of it.

Then:

1. Work the jaws to the **open** extreme and press **记录张开极限**.
2. Work them to the **closed** extreme and press **记录闭合极限**. That one is 0 mm.

The button you press is the whole answer to "which end is zero", so a press that arrives out of step
is refused out loud rather than taken as whichever point is due. Each wait is bounded at five
minutes: the axis is limp for the whole procedure, and a console left holding an enabled, limp motor
is a gripper that falls open on whatever is under it.

### Choosing a file, and keeping it

| Button | What it does |
| --- | --- |
| 载入文件… | Pick a calibration file and read it. The page then shows which path the numbers came from. |
| 刷新显示 | Re-read **the file in force**, and only that file |

**The file you load is the file the next launch loads.** A load is not undone by a restart. So if
the remembered file has gone, 载入文件… is the way out: pick another one, or pick the SDK's own
`litegrip/factory_calibration.json` to go back to the default — the console then forgets the
remembered path rather than writing the bundled file down as one you chose. 刷新显示 will tell you
the same thing if the file in force has been deleted, naming the factory file to load.

When a probe runs to the end it **asks** — a dialog with **应用标定** and **暂不**, naming the file
the result would be written to. Nothing is written until you answer, because pressing 记录 is the
request to *measure*, not the request to move by what was measured.

- **应用标定** writes the result and reads the file back; the source becomes 用户标定 and the gate
  goes `READY`. From then on the console loads that file on every launch.
- **暂不** leaves the result in memory. The gate stays shut and the jaws hold the angle the probe
  measured rather than snapping back.

Two things the write refuses, and says why: a result that failed validation, because the file it
would replace is a working calibration, and the SDK's own bundled file, because it describes
whichever unit it was taken on. Either refusal asks again, so 应用标定 is its own retry.

### What the 来源 row means

| Label | Meaning |
| --- | --- |
| 用户标定 | A calibration file that belongs to your gripper |
| 出厂标定 | The file bundled with the SDK — the default, and not a measurement of your unit |
| 标定无效 | A user file that failed validation |
| 未标定 | No file at all |
| 内存标定（未保存） | A probe result still in memory, not written out; the gate stays shut until it is |

**显示技术详情** on the same box opens the raw numbers the console is working from.

### 电机标零 — write the encoder zero

On the **状态** tab, under **维护**. **电机标零…** tells the drive that wherever the jaws are at
that moment is angle zero. The offset goes into the motor's own flash, and **there is no undo** —
the console cannot read it back and cannot put it back.

It is a maintenance action, not a step in calibration: it re-measures nothing, and it is what to
reach for when the jaws and the numbers have drifted far enough apart that measuring the ends again
is the point.

The confirmation names three consequences before the write, and the banner repeats them after:

- **Re-calibration is required.** The span on file keeps its length, but both of its angles, and
  every millimetre computed from them, are now offset. Until you re-calibrate with 自动标定 or the
  manual probe, all of those are wrong together.
- **The gate will not catch it.** A zero taken at a stop moves the frame by well under the tolerance
  the gate checks, so motion is not blocked. This is why the warning is stated in so many words
  instead of the console refusing.
- **The motor is briefly limp.** The SDK disables the drive, writes the offset and re-enables it, so
  for about a second the jaws can be pushed by hand and Esc does not take effect until the write
  ends. Keep hands and workpieces clear before confirming.

Confirm with **写入零点**. The warning is dismissed by **已了解** and by nothing else — nothing in
the console can retract it, because a calibration measured against the old offset does not become
correct again on its own.

标零 needs the console connected and the motor enabled, and it is refused while a probe is running.
The simulated backend refuses it outright, reporting 本后端不支持标零.

---

## Troubleshooting

| What you see | What it means, and what to do |
| --- | --- |
| `ModuleNotFoundError: No module named 'PyQt5'` | The libraries went to another interpreter. `run_litegrip_studio.sh -h` prints the one in use — install into that one, or set `PYTHON_BIN` |
| "Network is down, errno 100" after 连接 | The interface is down or bus-off. This is a link problem, not a motor one: check the adapter and the wiring, then reconnect |
| "RTNETLINK answers: No such file or directory" | The USB-CAN adapter lost its endpoint table. Unplug and replug it, or `sudo modprobe -r gs_usb && sudo modprobe gs_usb` |
| 连接 does nothing and no dialog appears | Your user dismissed the authorization prompt, or a unit file owns `can0`. Bring the interface up by hand with the three commands above |
| 运动闸门：已阻断 | The console will not move. The page says which reason applies — no file, an invalid file, a missing file, or a zero that belongs to another machine. 零重力（可手掰） still works |
| 标定文件已不存在：… | The remembered file has been deleted. 载入文件… picks another, or picks the SDK's `litegrip/factory_calibration.json` to go back to the default |
| The jaws will not move at all | Check the gate first, then that the motor is enabled (使能), then the emergency-stop banner: a latched 急停 refuses every motion until 急停复位 |
| Movement refused: "no position read yet" | The motor has never reported a status frame. Enable, and wait for 状态帧 to start counting |
| 电机标零 reports 本后端不支持标零 | You are on the simulated backend, which refuses 标零 on purpose |
| The force plot looks like noise | Unloaded, the force is a fraction of a newton. Grasp something, or ignore it |

---

## Known limitations

- **电机标零 is refused by the simulated backend.** Deliberate, not an omission: the simulation's
  position is at once the mechanism's physics and the angle it reports, so shifting only the report
  would make it integrate against a state that disagrees with what it says.
- **The SDK's own `calibrate()` is not offered.** It cannot be interrupted and drives the motors for
  about 24 seconds — better to offer the two probes on the calibration page than a 24-second window
  in which the emergency stop does nothing.
- **The factory calibration is the default, not a measurement of *your* gripper.** It was recorded
  on the unit it ships with, so on another gripper of the same design the readings are close rather
  than exact, off by that unit's manufacturing tolerance. The console does not try to judge that —
  but it does catch a file recorded against a different encoder zero, a reassembled unit or a
  different design, because such a file's travel does not contain the angle the jaws actually
  report, and the gate goes `BLOCKED` before a frame is sent.
- **The travel is a constant for this bench**, 85.0 mm in `constants.DEFAULT_TRAVEL_MM`. It decides
  both the millimetre conversion and the slider's range, so a console told the wrong one reports
  every millimetre wrong while looking healthy. Moving this console to another gripper means editing
  that constant and re-probing — there is no flag, setting or widget for it.
- **The force limit is the 40 N mechanical rating**, in `constants.FORCE_MAX_N`. The theoretical
  100 N appears only as a greyed-out comment.
- **The smallest force you can ask for is 3 N**, in `constants.FORCE_MIN_N`. A grip is the SDK's
  force-budgeted `grasp()`, and its approach spends only 0.9 × the setpoint, all of which goes
  first on breaking the jaws away from static friction before any of it moves them. The breakaway
  on this bench is between 1.8 and 2.0 N, read off the gripper's own log on 2026-10-29: the 2 N
  approach presses 0.18 N·m and moved 0.0004 rad in its whole 0.25 s stall window, while the 2 N
  hold that then took over presses 0.20 N·m steady and walked the jaws 19 mm. So a 2 N grasp
  stands still, its approach's stall detector reads the standstill as contact, and the pure-torque
  hold closes the rest of the travel under the full setpoint — measured as a grip declared 19–39 mm
  short of the object followed by a fast close. 3 N approaches at 2.7 N, a third clear of the
  breakaway, and reaches the object, so 3 N is the floor. The console's floor only keeps zero
  from meaning "a very light grip" — zero is the mode that says "no force limit". The simulator
  still runs the console's own FSM, and its contact detector needs about 6 N, so a simulated 3 N
  grip may declare contact late. That limitation does not apply to the real gripper.
- **The top of the range stops 1 mm inside the recorded open limit.** The probe presses the open
  stop out under 40 N and the linkage gives about 1 mm under that force, so commanding the recorded
  limit directly would press the stop on every full opening. The closed end is not inset: 0 is the
  calibrated zero.

---

## Files and settings

The console keeps everything for one user under `~/.litegrip/`:

| Path | What it is |
| --- | --- |
| `litegrip_studio.ini` | Settings: the last speed and force, whether the slider follows while dragging, the remembered calibration path, the theme, the window geometry and the log's debug toggle |
| `litegrip_calibration.json` | The default place a calibration is saved — but sitting here is not what makes it load. The console reads the path it remembers |
| `logs/litegrip-studio.log` | The log, rotated at 512 KiB with three backups, always at `DEBUG` |

Environment variables:

| Variable | Effect |
| --- | --- |
| `PYTHON_BIN` | The interpreter the launcher and the build script use |
| `LITEGRIP_SDK_PATH` | Use an SDK checkout instead of the vendored one; the checkout root or its `src/` |
| `LITEGRIP_CALIB` | Override the default user calibration path |
| `LITEGRIP_FACTORY_CALIB` | Replace the factory calibration file with one path of your own |

---

## Development

The tests need neither hardware nor an installed SDK:

```bash
./run_litegrip_studio.sh test
# equivalent to
QT_QPA_PLATFORM=offscreen python3 -m pytest tests/ -q
```

`QT_QPA_PLATFORM=offscreen` is what lets the UI tests build real widgets without a display.
`tests/conftest.py` points `HOME` at a temporary directory of the session's own, so no test reads or
writes your `~/.litegrip/`, and the repository-root `conftest.py` puts `src/` on `sys.path`, which
is all the vendored SDK needs.

The SDK is **vendored, not a dependency**: `src/litegrip` holds litegrip-python at a pinned commit,
so a clone runs with nothing installed beside it. **Do not edit anything under it** — a local edit
survives only until the next re-vendor, which copies each file wholesale. When a divergence is
genuinely unavoidable, record it by name in `src/litegrip/VENDORED.md`, which also holds the tree
hash and the re-vendor steps; `tests/test_vendored_sdk.py` recomputes the hash, so an edit in place
fails the suite. There is no local override today: every file there is upstream's.

`./run_litegrip_studio.sh build` produces a one-file executable in `dist/`. Releases are automated
by semantic-release on every merge to `main`; the git tag is the only source of truth for the
version, and `0.0.0-semantic-release` in `pyproject.toml` is a placeholder that must not be edited
by hand.

**The reasons behind all of this — why the architecture is shaped this way, what the slider is
actually doing, why the calibration gate sits where it does — are in [DESIGN.md](DESIGN.md).** This
file is the manual.

This repository follows the shared NEXFORM ROBOTICS standards: the agent operating rules in
[`.nexform-standards.json`](.nexform-standards.json), Conventional Commits, and automated semantic-release versioning.

---

## License

Licensed under the **Apache License 2.0** — see [LICENSE](LICENSE).

Copyright © 2026 NEXFORM ROBOTICS.
