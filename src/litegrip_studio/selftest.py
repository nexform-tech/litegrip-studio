"""A smoke test that runs without Qt, for machines where the GUI will not start.

The point is to answer one question on a target machine — *is the control law
and the arithmetic behind it working here?* — before anyone trusts it with a
motor, and to do it with no display, no CAN interface and no SDK installed.

Nothing in here imports PyQt5.  That is deliberate and it is also why this file
re-implements the 200 Hz loop instead of borrowing the worker: the worker is a
``QThread``, and a self-test that needs a working Qt is no use on the machine
that has a broken one.
"""

from __future__ import annotations

import math
import sys
import traceback
from collections.abc import Callable

from . import calibration, constants
from .backend.plant import Plant, PlantConfig
from .backend.sim import SimBackend
from .can_link import parse_link
from .core.motion import MotionFSM, MotionParams, MotionState
from .units import Limits, frame_mismatch

#: The three calibrations this console has to get right, as (closed, open,
#: file_scale, agrees, description).  The first is the SDK's uncalibrated default
#: pair, whose two angles are ordered the other way round and whose range this
#: bench's encoder readings fall outside of — the two facts the checks below turn
#: on.
#:
#: ``file_scale`` is the millimetres per rad the *file* carries, and ``agrees``
#: says whether that number is the scale this console derives for the bench's own
#: travel.  The console never moves by the file's number — it derives the scale
#: from the recorded angles and the measured travel (``units.derive_scale``), and
#: that is what keeps a slider from covering the middle of a travel nobody has
#: seen the ends of.  On the first and third datasets the two differ, and those
#: are the witnesses of it: put the file's number back and the travel check
#: fails.  The second is the SDK's shipped template
#: (``calibration_normal.json``), and there the two are the *same* number, because
#: upstream re-measured the shipped files for this unit in ``b9caae8``: 61.012293
#: is what the derivation gives over its span.  Equality is the fact to pin
#: there, not the defect to catch, and ``agrees`` records which of the two a
#: dataset is.
#:
#: The template is *not* the factory file, though the two now carry the same
#: angles: the factory file is a different schema and is checked as a file, by
#: :func:`check_the_fallback_calibration_is_usable`.  The reverse template carries
#: these same numbers mirrored, so it is the same case again.
KNOWN_CALIBRATIONS = (
    (0.0, 1.14, 120.0 / 1.14, False, "SDK 默认值（两个角次序相反，且在实测角度之外）"),
    (
        0.052071,
        -1.357481,
        61.01229326764816,
        True,
        "SDK 模板（calibration_normal.json，上游已按本机重新实测）",
    ),
    (1.775959, -0.064279, 65.21, False, "用户标定（示例）"),
)

#: The two angles the bench unit this console was debugged against stands at, all
#: the way open and all the way shut.  That unit is the one the direction work
#: came from, and the point of it is the *ordering*: the angle is larger with the
#: jaws apart, which is not what the SDK's formulas describe.  The file that was
#: in force there had the two the other way round, which is why commanding 闭合
#: opened the gripper.
MEASURED_OPEN_RAD = 1.421569
MEASURED_CLOSED_RAD = -0.300793


def _limits(closed: float, open_: float) -> Limits:
    """The limits the console actually runs on for a pair of recorded angles.

    Built through :func:`~litegrip_studio.calibration.limits_from_raw`, which is
    the function every calibration the console loads goes through, so these
    checks exercise the path in production rather than a copy of it.  The scale
    in the result is not the file's — see
    :func:`~litegrip_studio.units.derive_scale` — which is why the checks below
    that care about millimetres have to be built this way at all.
    """
    raw = {"zero_position_rad": closed, "max_position_rad": open_}
    return calibration.limits_from_raw(raw, constants.DEFAULT_TRAVEL_MM)


#: What ``ip -details link show`` prints for the states that decide whether 连接
#: has to ask for a password.  Captured from the kernel rather than invented —
#: the tab indentation and the wording are ip's, not this console's.
IP_RAISED_CLASSIC = (
    "2: can0: <NOARP,UP,LOWER_UP> mtu 16 qdisc pfifo_fast state UP "
    "mode DEFAULT group default qlen 10\n"
    "    link/can  promiscuity 0 minmtu 0 maxmtu 0 \n"
    "\t  bitrate 1000000 sample-point 0.750 \n"
)
IP_RAISED_FD = (
    "2: can0: <NOARP,UP,LOWER_UP> mtu 72 qdisc pfifo_fast state UP "
    "mode DEFAULT group default qlen 10\n"
    "    link/can  promiscuity 0 minmtu 0 maxmtu 0 \n"
    "\t  bitrate 1000000 sample-point 0.750 \n"
    "\t  dbitrate 2000000 dsample-point 0.800 \n"
    "\t  fd on fd-non-iso off\n"
)
IP_UNPLUGGED = 'Device "can0" does not exist.\n'
#: ``lo``, verbatim: administratively up while its operstate reads UNKNOWN.
IP_UP_BUT_UNKNOWN_OPERSTATE = (
    "1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN "
    "mode DEFAULT group default qlen 1000\n"
)


class Check:
    """One named check, run in isolation so a crash is a failure and not the end."""

    def __init__(self, name: str, run: Callable[[], None]) -> None:
        self.name = name
        self.run = run


def _tick_loop(
    fsm: MotionFSM, sim: SimBackend, clock, seconds: float
) -> None:
    """Drive the FSM against the simulator at the control rate.

    The same loop the worker runs, minus the thread, the queue and the
    publishing: read, tick, advance the clock, poll.
    """
    for _ in range(int(round(seconds / constants.CTRL_DT))):
        fsm.tick(sim, sim.read(), constants.CTRL_DT, allow_motion=True)
        clock.advance(constants.CTRL_DT)
        sim.poll()


class _Clock:
    def __init__(self) -> None:
        self.t = 1000.0

    def __call__(self) -> float:
        return self.t

    def advance(self, seconds: float) -> None:
        self.t += seconds


# ── the checks ──────────────────────────────────────────────────────────────
def check_units_round_trip() -> None:
    for closed, open_, _file_scale, _agrees, _label in KNOWN_CALIBRATIONS:
        limits = _limits(closed, open_)
        for mm in (0.0, 1.0, 37.5, limits.stroke_mm):
            rad = limits.to_rad(mm)
            back = limits.to_mm(rad)
            assert abs(back - mm) < 1e-6, f"{mm} mm 往返后成了 {back} mm"


def check_the_direction_is_read_from_the_two_angles() -> None:
    """Which way the jaws open is a fact about the assembly, and it is read from
    the two recorded angles rather than declared or assumed.

    The SDK's formulas only describe a unit whose angle shrinks as the jaws open,
    which is how the units here are assembled — but the console does not take
    even that on trust: the sign comes out of the recorded pair, so a file whose
    two angles were recorded the other way round is converted by what it says.
    That ordering is also what the SDK's uncalibrated default pair looks like, so
    it cannot decide whether a calibration is good either.  Two things have to
    hold instead, and both are checked here on all three datasets: every
    conversion agrees with the ordering the angles have, and the calibration's
    range contains the angles its own gripper is standing at.
    """
    for closed, open_, _file_scale, _agrees, label in KNOWN_CALIBRATIONS:
        limits = _limits(closed, open_)
        assert limits.to_rad(0.0) == closed, f"{label}：0 mm 不是记录的闭合角"
        assert limits.to_rad(limits.max_stroke_mm) != closed, (
            f"{label}：量程顶端和 0 mm 落在同一个角度，换算没有方向"
        )
        # The sign of the slope has to be the ordering the two recorded angles
        # have — the one thing a wrong `direction` cannot fake, since it moves
        # the conversions and leaves the angles where they were.
        grew = limits.to_rad(limits.max_stroke_mm) > closed
        assert grew is (open_ > closed), f"{label}：换算是朝反方向的"

    uncalibrated = _limits(*KNOWN_CALIBRATIONS[0][:2])
    template = _limits(*KNOWN_CALIBRATIONS[1][:2])
    user = _limits(*KNOWN_CALIBRATIONS[2][:2])
    assert uncalibrated.direction == 1.0, "SDK 默认值的两个角次序被读成了别的方向"
    assert template.direction == -1.0, "SDK 出厂模板的方向读错了"
    assert user.direction == -1.0, "用户标定的方向读错了"

    # And the answer has to come from something other than the ordering: both
    # orderings of a pair describe a gripper, so what refuses a file that
    # belongs to another one is the encoder reading.
    bench = _limits(MEASURED_CLOSED_RAD, MEASURED_OPEN_RAD)
    assert bench.direction == 1.0, "本机两个极限的次序被读成了别的方向"
    for measured in (MEASURED_CLOSED_RAD, MEASURED_OPEN_RAD):
        assert frame_mismatch(uncalibrated, measured), (
            f"实测角度 {measured:.4f} rad 落在 SDK 默认值的行程里，这份零点不属于本机"
        )
        assert frame_mismatch(bench, measured) == "", "本机标定在自己的读数上被拦下"


def check_the_travel_is_derived_from_the_recorded_angles() -> None:
    """Every dataset records a pair of angles, and the millimetres per rad the
    console moves by comes out of them and the measured travel.

    Three things have to hold on each, and the third is the one the operator
    sees: 0 mm is the recorded closed angle, the recorded extremes span the
    travel plus the probe's inset rather than the file's nominal stroke, and the
    top of the commanded range is that inset short of the recorded open one.
    Move by the file's own scale instead and the recorded span comes out as that
    file's nominal — 120 mm for the defaults — a slider covering the middle of a
    travel nobody has seen the ends of, which is what this check exists to catch.

    All three are stated without a sign, and the first dataset is the one that
    needs that: its two angles are the other way up, and every one of them holds
    anyway.
    """
    travel = constants.DEFAULT_TRAVEL_MM
    inset = constants.SPAN_INSET_MM
    for closed, open_, file_scale, agrees, label in KNOWN_CALIBRATIONS:
        limits = _limits(closed, open_)
        assert abs(limits.to_mm(closed)) < 1e-9, f"{label}：记录的闭合角不是 0 mm"

        span = limits.stroke_mm
        assert abs(span - (travel + inset)) < 1e-6, (
            f"{label} 的记录跨度是 {span:.2f} mm，应为 {travel + inset:.2f} mm"
            f"（行程 {travel:.0f} + 内缩 {inset:.0f}）"
        )

        # The top of the commanded range, back in angles: it sits the inset away
        # from the recorded open extreme.  This is the one place the two
        # conversions are checked against each other, so it is also the place a
        # sign error between them shows up — a to_rad that ran the other way
        # would leave the whole travel between them here, not the inset.
        gap_mm = abs(limits.to_rad(travel) - open_) * limits.rad_to_mm
        assert abs(gap_mm - inset) < 1e-6, (
            f"{label} 的量程顶端离记录的张开角 {gap_mm:.2f} mm，应为 {inset:.0f} mm"
        )

        # The file's own number is not what the console moves by.  On the first
        # and third datasets it differs from the derivation by far more than
        # rounding — 52.09 against 52.69, 46.73 against 65.21 — and that
        # difference is the witness.  On the template the two are the same
        # number, because the SDK's shipped files are this unit's again; the
        # equality is the fact being pinned there, and the tolerance keeps it
        # from turning into a claim about the last decimal of a stored float.
        same = math.isclose(limits.rad_to_mm, file_scale, rel_tol=1e-9)
        assert same is agrees, (
            f"{label}：文件自带的 mm/rad（{file_scale}）与推导值（{limits.rad_to_mm}）"
            f"{'应当' if agrees else '不应'}相同"
        )


def check_plant_settles_on_a_step() -> None:
    """A step into free space must arrive, and must not overshoot."""
    clock = _Clock()
    plant = Plant(PlantConfig())
    limits = plant.limits
    for _ in range(400):
        plant.stream(limits.to_rad(60.0), constants.KP_MOVE, constants.KD_DEFAULT)
        plant.step(constants.CTRL_DT)
        clock.advance(constants.CTRL_DT)

    position = plant.snapshot().position_mm
    assert abs(position - 60.0) < 1.0, f"稳态位置 {position:.2f} mm，目标 60.00 mm"


def check_a_move_arrives_and_stops() -> None:
    """The whole point of the console, driven end to end with no GUI."""
    clock = _Clock()
    sim = SimBackend(clock=clock)
    sim.connect()
    sim.enable()
    limits = sim.limits()
    fsm = MotionFSM(limits, MotionParams())

    # The far end is the travel, not a number: it is what the console is set to
    # and it is what the slider tops out at, so a literals here would be a check
    # on the literal.
    top = limits.max_stroke_mm
    fsm.open()
    _tick_loop(fsm, sim, clock, 8.0)
    opened = sim.read().position_mm
    assert fsm.state is MotionState.HOLD, f"张开后停在 {fsm.state.value}"
    assert abs(opened - top) < 1.0, f"张开后停在 {opened:.1f} mm，量程顶端是 {top:.1f} mm"

    fsm.move_to_mm(45.0, source="selftest")
    _tick_loop(fsm, sim, clock, 8.0)
    arrived = sim.read().position_mm
    assert abs(arrived - 45.0) < constants.TOL_MM, f"停在 {arrived:.2f} mm，目标 45.00 mm"

    sim.disconnect()


def check_a_stall_is_detected_rather_than_fought() -> None:
    """Closing on something solid has to end in a bounded force, not in a
    torque that keeps climbing."""
    clock = _Clock()
    sim = SimBackend(clock=clock)
    sim.connect()
    sim.enable()
    sim.plant.object_mm = 20.0
    fsm = MotionFSM(sim.limits(), MotionParams())

    fsm.close()
    _tick_loop(fsm, sim, clock, 8.0)
    force = abs(sim.read().force_n)

    assert fsm.state is not MotionState.SERVO, "顶住不动却还在伺服"
    assert force <= constants.FORCE_MAX_N * 1.1, f"顶住时力到了 {force:.1f} N"
    sim.disconnect()


def check_the_estop_zeroes_the_torque() -> None:
    clock = _Clock()
    sim = SimBackend(clock=clock)
    sim.connect()
    sim.enable()
    sim.stream_frame(sim.limits().to_rad(10.0), constants.KP_MOVE, constants.KD_DEFAULT)
    sim.zero_torque()
    sim.disable()
    _ = clock

    assert not sim.read().is_enabled, "急停后电机仍是使能状态"
    sim.disconnect()


def check_the_can_probe_is_read_correctly() -> None:
    """The one step before 连接 that reads text instead of hardware.

    It is what decides whether a password dialog appears: reading an interface
    that is already right as "wrong" would make every connect ask for one, and
    reading CAN FD as classic CAN would invite the console to rewrite a bus that
    other nodes are on.  Both are string handling, so they can be pinned here, on
    a machine with no adapter plugged in.
    """
    assert parse_link(IP_RAISED_CLASSIC, 0).matches(constants.CAN_BITRATE), (
        "已 up 且比特率正确的接口被判成需要改动，每次连接都会弹授权框"
    )

    fd = parse_link(IP_RAISED_FD, 0)
    assert not fd.matches(constants.CAN_BITRATE), (
        "CAN FD 被判成经典 CAN，控制台会去改这个总线"
    )
    assert fd.bitrate == constants.CAN_BITRATE, (
        f"FD 接口的标称比特率读成了 {fd.bitrate}：dbitrate 被当成了 bitrate"
    )

    assert not parse_link(IP_UNPLUGGED, 1).exists, "不存在的接口被判成存在"

    assert parse_link(IP_UP_BUT_UNKNOWN_OPERSTATE, 0).up, (
        "从 operstate 读管理状态，把 up 的接口读成了 down"
    )


def check_a_calibration_from_another_frame_is_refused() -> None:
    """The one calibration check that reads the hardware instead of the file.

    Every other check in this file asks whether the numbers are self-consistent,
    and a file can pass all of them while its zero belongs to a different
    encoder frame — a different unit, or the same one before it was taken apart
    and re-zeroed.  Nothing about such a file looks wrong; only the live angle
    can tell, and by then it is the clamp that decides what to do with it, which
    turns a wrong reading into a confident command.  The two ranges below are
    the ones from the incident, and they do not overlap at all.
    """
    other_frame = Limits(1.775959, -0.064279, 65.21, constants.DEFAULT_TRAVEL_MM)
    assert frame_mismatch(other_frame, -1.0), (
        "零点在别的编码器帧上的标定没有被拦下，实测位置会被夹到行程端点再发出去"
    )
    assert frame_mismatch(other_frame, 0.5) == "", "行程内的实测位置被判成了不匹配"

    # A calibration that stops short of the hard stops is the safer kind, and it
    # leaves the axis legitimately parked outside its own commanded range — so
    # the check has to have a tolerance, and this is what it is for.
    red_lines = Limits(-0.170982, -1.403653, 65.0229, 80.16)
    assert frame_mismatch(red_lines, -0.109674) == "", (
        "停在闭合硬止挡上的夹爪被判成标定不匹配，闸门会被误关"
    )


def check_the_fallback_calibration_is_usable() -> None:
    """The one check that reads a file instead of doing arithmetic.

    The fallback is the SDK's own ``factory_calibration.json``, which is data
    beside the code, and that is exactly why it can be absent from an artifact
    that is otherwise complete — a wheel built without its package data, a
    bundle built without ``--collect-data litegrip`` — and the symptom on the
    target machine is a console that refuses to move.  Nothing else in this file
    would notice, and the machine that needs this answer is the one where the
    GUI was not the first thing to be tried.
    """
    path = calibration.factory_path()
    assert path.is_file(), f"出厂标定文件不在产物里：{path}（打包时漏了数据文件）"

    raw, problems = calibration.parse_calibration_json(path.read_text(encoding="utf-8"))
    assert not problems, f"出厂标定读不出来：{problems}"

    limits = calibration.limits_from_raw(raw, constants.DEFAULT_TRAVEL_MM)
    hard, _soft = calibration.validate_limits(
        limits, constants.DEFAULT_TRAVEL_MM, calibration.file_scale(raw)
    )
    assert not hard, f"出厂标定没通过校验：{hard}"


CHECKS = (
    Check("单位换算往返一致", check_units_round_trip),
    Check("方向取自记录的两个角度", check_the_direction_is_read_from_the_two_angles),
    Check("标定零点不属于本机时被拦下", check_a_calibration_from_another_frame_is_refused),
    Check("三份标定的 0 点与量程顶端正确", check_the_travel_is_derived_from_the_recorded_angles),
    Check("被控对象在阶跃下收敛且不过冲", check_plant_settles_on_a_step),
    Check("CAN 口探测结果读取正确", check_the_can_probe_is_read_correctly),
    Check("一次运动能到位并停住", check_a_move_arrives_and_stops),
    Check("顶住硬物时力矩有上限", check_a_stall_is_detected_rather_than_fought),
    Check("急停后电机失能", check_the_estop_zeroes_the_torque),
    Check("出厂标定文件可用", check_the_fallback_calibration_is_usable),
)


def run(stream=None) -> int:
    """Run every check.  Returns a process exit code.

    ``stream`` defaults to whatever ``sys.stdout`` is *when it is called*.  Bound
    as a default argument it would be resolved at import, which freezes in the
    stream that happened to be current then — and then a caller who redirects
    output, or a test that captures it, silently gets the old one.
    """
    if stream is None:
        stream = sys.stdout
    print("LiteGrip 控制台自检（不需要 Qt，不需要硬件）", file=stream)
    failures = 0
    for check in CHECKS:
        try:
            check.run()
        except Exception:  # noqa: BLE001 - a self-test reports, it does not raise
            failures += 1
            print(f"  FAIL  {check.name}", file=stream)
            traceback.print_exc(file=stream)
        else:
            print(f"  PASS  {check.name}", file=stream)
    total = len(CHECKS)
    print(f"\n{total - failures}/{total} 项通过", file=stream)
    return 1 if failures else 0


def main() -> int:
    return run()
