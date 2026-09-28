"""Raising the CAN interface: what it reads, decides, and refuses.

Two halves, and the second is the one that matters.  The first is reading
``ip``'s answer, which is pure and is pinned here against real transcripts.  The
second is everything about the *privileged* half: that it runs no command when
the interface is already right, that the interface name is never spliced into
the script a shell will read, that a refusal leaves the interface alone, and that
none of it can raise into the connect path.

The subprocess is injected, so the suite never runs ``ip``, never asks for a
password, and never needs an interface to exist.
"""

from __future__ import annotations

import subprocess

import pytest

from litegrip import LiteGripError

from litegrip_studio import constants
from litegrip_studio.can_link import (
    LINK_CONFIGURED,
    LINK_DENIED,
    LINK_FAILED,
    LINK_FD,
    LINK_MISSING,
    LINK_OK,
    PKEXEC_DISMISSED,
    PKEXEC_NOT_AUTHORIZED,
    CanLink,
    link_failure,
    parse_link,
)

IP_PATH = "/usr/sbin/ip"
PKEXEC_PATH = "/usr/bin/pkexec"

#: A raised classic-CAN interface at 1 Mbit, in the shape ``ip -details`` prints
#: it (kernel and iproute2 wording, including the tab indentation).
CAN_UP_1M = """\
2: can0: <NOARP,UP,LOWER_UP> mtu 16 qdisc pfifo_fast state UP mode DEFAULT group default qlen 10
    link/can  promiscuity 0 minmtu 0 maxmtu 0 \x20
    can state ERROR-ACTIVE (berr-counter tx 0 rx 0) restart-ms 0 \x20
\t  bitrate 1000000 sample-point 0.750 \x20
\t  tq 62 prop-seg 7 phase-seg1 6 phase-seg2 3 sjw 1
\t  bittiming-const 0 \x20
\t  clock 80000000 \x20
\t  re-started bus-errors arbit-lost error-warn error-pass bus-off
\t  numtxqueues 1 numrxqueues 1 gso_max_size 65536 gso_max_segs 65535 \x20
"""

#: Plugged in and never configured: no bitrate line at all.
CAN_DOWN_UNCONFIGURED = """\
2: can0: <NOARP> mtu 16 qdisc noop state DOWN mode DEFAULT group default qlen 10
    link/can  promiscuity 0 minmtu 0 maxmtu 0 \x20
\t  re-started bus-errors arbit-lost error-warn error-pass bus-off
"""

#: Configured, then taken down.  The bitrate survives a down/up cycle, so this is
#: exactly what the README's first two commands leave behind if the third is
#: never run — and the state most likely to be mistaken for ready, because
#: everything about it reads right except the one flag that decides.
CAN_DOWN_CONFIGURED = """\
2: can0: <NOARP> mtu 16 qdisc noop state DOWN mode DEFAULT group default qlen 10
    link/can  promiscuity 0 minmtu 0 maxmtu 0 \x20
\t  bitrate 1000000 sample-point 0.750 \x20
"""

#: Raised, but at 500 kbit — someone else's idea of the bus.
CAN_UP_500K = """\
2: can0: <NOARP,UP,LOWER_UP> mtu 16 qdisc pfifo_fast state UP mode DEFAULT group default qlen 10
    link/can  promiscuity 0 minmtu 0 maxmtu 0 \x20
\t  bitrate 500000 sample-point 0.875 \x20
"""

#: CAN FD: a nominal bitrate *and* a data bitrate, which is the pair that makes
#: a naive "find the first bitrate" read the wrong number.
CAN_UP_FD = """\
2: can0: <NOARP,UP,LOWER_UP> mtu 72 qdisc pfifo_fast state UP mode DEFAULT group default qlen 10
    link/can  promiscuity 0 minmtu 0 maxmtu 0 \x20
\t  bitrate 1000000 sample-point 0.750 \x20
\t  dbitrate 2000000 dsample-point 0.800 \x20
\t  fd on fd-non-iso off
"""

#: Plugged in, raised, at the right bitrate — and off the bus.  Every field this
#: module used to read is correct here, which is the whole point: ``UP,LOWER_UP``
#: is still set, the bitrate survived, ``fd`` is off.  The only difference from
#: ``CAN_UP_1M`` above is the ``can state`` token, and it is the difference
#: between an interface that works and one that returns ENETDOWN on every send.
#:
#: Shaped like ``ip -details`` prints it (same wording, same tab indentation) but
#: not captured: this machine has no CAN adapter, and a ``vcan`` device — the
#: only bus that could be made here without hardware — never goes bus-off.
CAN_UP_BUS_OFF = """\
2: can0: <NOARP,UP,LOWER_UP> mtu 16 qdisc pfifo_fast state UP mode DEFAULT group default qlen 10
    link/can  promiscuity 0 minmtu 0 maxmtu 0 \x20
    can state BUS-OFF (berr-counter tx 0 rx 0) restart-ms 0 \x20
\t  bitrate 1000000 sample-point 0.750 \x20
\t  tq 62 prop-seg 7 phase-seg1 6 phase-seg2 3 sjw 1
\t  bittiming-const 0 \x20
\t  clock 80000000 \x20
\t  re-started bus-errors arbit-lost error-warn error-pass bus-off
\t  numtxqueues 1 numrxqueues 1 gso_max_size 65536 gso_max_segs 65535 \x20
"""

#: One step short of the above: it can still transmit and it recovers on its own,
#: so nothing is reconfigured — but a message that omitted it would describe a
#: marginal bus as if it were a healthy one.
CAN_UP_ERROR_PASSIVE = """\
2: can0: <NOARP,UP,LOWER_UP> mtu 16 qdisc pfifo_fast state UP mode DEFAULT group default qlen 10
    link/can  promiscuity 0 minmtu 0 maxmtu 0 \x20
    can state ERROR-PASSIVE (berr-counter tx 200 rx 0) restart-ms 0 \x20
\t  bitrate 1000000 sample-point 0.750 \x20
"""

#: The real thing, captured on this machine (``ip -details link show lo``): an
#: interface that is administratively up while its operstate reads something
#: else entirely.  This is why the state is read from the flags.
REAL_LO = (
    "1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN "
    "mode DEFAULT group default qlen 1000\n"
    "    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00 promiscuity 0 "
    "minmtu 0 maxmtu 0 addrgenmode eui64 numtxqueues 1 numrxqueues 1 "
    "gso_max_size 65536 gso_max_segs 65535 \n"
)

#: Also real (``ip -details link show can0`` with nothing plugged in).
REAL_MISSING = 'Device "can0" does not exist.\n'


class FakeRun:
    """The subprocess seam: answers from a queue, records every argv."""

    def __init__(self, *results: tuple[int, str, str]) -> None:
        self.results = list(results)
        self.calls: list[list[str]] = []

    def __call__(self, argv, timeout):
        self.calls.append(list(argv))
        rc, out, err = self.results.pop(0) if self.results else (0, "", "")
        return subprocess.CompletedProcess(list(argv), rc, out, err)

    @property
    def pkexec_calls(self) -> list[list[str]]:
        return [call for call in self.calls if "pkexec" in call[0]]


def link(*results, channel: str = "can0", bitrate: int = constants.CAN_BITRATE, ip=IP_PATH):
    """A :class:`CanLink` whose ``ip`` and ``pkexec`` are fakes."""
    run = FakeRun(*results)
    tool = CanLink(
        channel,
        bitrate,
        run=run,
        which=lambda name: {"ip": ip, "pkexec": PKEXEC_PATH}.get(name),
    )
    return tool, run


def ok(text: str) -> tuple[int, str, str]:
    return 0, text, ""


class TestReadingWhatIpSays:
    def test_a_raised_classic_can_interface(self) -> None:
        state = parse_link(CAN_UP_1M, 0)

        assert state.exists and state.up
        assert state.bitrate == 1_000_000
        assert state.fd is False

    def test_an_interface_that_has_never_been_configured(self) -> None:
        """No bitrate line at all: plugged in, and nothing else."""
        state = parse_link(CAN_DOWN_UNCONFIGURED, 0)

        assert state.exists
        assert not state.up
        assert state.bitrate is None

    def test_a_missing_interface(self) -> None:
        """Verbatim from this machine, with the exit code that goes with it."""
        state = parse_link(REAL_MISSING, 1)

        assert not state.exists

    def test_the_state_comes_from_the_flags_not_the_operstate(self) -> None:
        """A real transcript: ``lo`` is UP while its operstate reads UNKNOWN.

        Reading ``state`` would call this interface down, and for CAN, whose
        operstate depends on whether anything is on the bus, that mistake would
        be the normal case rather than the exception.
        """
        state = parse_link(REAL_LO, 0)

        assert state.exists and state.up

    def test_an_fd_interface_is_read_as_fd(self) -> None:
        state = parse_link(CAN_UP_FD, 0)

        assert state.fd is True

    def test_the_nominal_bitrate_is_not_confused_with_the_data_bitrate(self) -> None:
        """``dbitrate`` ends in ``bitrate`` and the two are different numbers on
        the same bus.

        The real transcript alone does not pin this: ip prints the nominal one
        first, so a plain "first bitrate wins" search reads it correctly *by
        ordering*.  That is not something to rely on for the number every
        millimetre reading is derived from, so the same two lines are also read
        in the other order — the transcript that would defeat it.
        """
        assert parse_link(CAN_UP_FD, 0).bitrate == 1_000_000  # the real one

        swapped = "\t  dbitrate 2000000 dsample-point 0.800 \n\t  bitrate 1000000 \n"
        assert parse_link(swapped, 0).bitrate == 1_000_000

    def test_a_bus_off_controller_is_not_a_working_interface(self) -> None:
        """The state that made the console lie about the link.

        Everything the probe used to look at reads correct — raised, right
        bitrate, classic CAN — so the interface was left exactly as it was and
        the first frame that tried to leave came back as ENETDOWN.  The
        controller's own state is the only field that tells them apart.
        """
        state = parse_link(CAN_UP_BUS_OFF, 0)

        assert state.up and state.bitrate == 1_000_000 and not state.fd
        assert state.can_state == "BUS-OFF"
        assert state.deaf, "发不出帧的接口不能算就绪"
        assert not state.matches(constants.CAN_BITRATE)
        assert "BUS-OFF" in state.describe()

    def test_an_error_passive_controller_matches_but_is_reported(self) -> None:
        """It can still transmit and it recovers by itself, so raising it would
        be a password dialog for nothing — but the operator reading the log has
        to be able to see why the bus is flaky."""
        state = parse_link(CAN_UP_ERROR_PASSIVE, 0)

        assert state.matches(constants.CAN_BITRATE)
        assert not state.deaf
        assert "ERROR-PASSIVE" in state.describe()

    def test_an_interface_that_says_nothing_about_its_controller_is_not_assumed_broken(
        self,
    ) -> None:
        """Silence is not evidence.  A device that is not a CAN interface, or a
        version of ``ip`` whose state line this does not know, must not be
        reconfigured on a guess — and a healthy one still reads as healthy."""
        assert parse_link(CAN_UP_1M, 0).can_state == "ERROR-ACTIVE"
        assert parse_link(REAL_LO, 0).can_state == ""
        assert not parse_link(REAL_LO, 0).deaf

    def test_anything_that_is_not_the_state_we_need_does_not_match(self) -> None:
        wanted = constants.CAN_BITRATE

        assert not parse_link(CAN_DOWN_UNCONFIGURED, 0).matches(wanted)
        assert not parse_link(CAN_DOWN_CONFIGURED, 0).matches(wanted), (
            "「bitrate 对了但没 up」是这里唯一只靠 UP 才能判出的情形"
        )
        assert not parse_link(CAN_UP_500K, 0).matches(wanted)
        assert not parse_link(REAL_MISSING, 1).matches(wanted)
        assert not parse_link(CAN_UP_FD, 0).matches(wanted)
        assert not parse_link(CAN_UP_BUS_OFF, 0).matches(wanted), (
            "总线关闭的接口每个字段都对，只有控制器状态不对"
        )
        assert parse_link(CAN_UP_1M, 0).matches(wanted)


class TestWhatItRuns:
    def test_an_interface_already_right_is_left_alone(self) -> None:
        """The whole reason this is tolerable as an automatic step: the common
        case — the interface the operator already raised — must not produce a
        password dialog."""
        tool, run = link(ok(CAN_UP_1M))

        outcome = tool.ensure()

        assert outcome.state == LINK_OK
        assert len(run.calls) == 1, "only the probe"
        assert run.pkexec_calls == []

    def test_a_wrong_bitrate_is_reconfigured(self) -> None:
        tool, run = link(ok(CAN_UP_500K), (0, "", ""), ok(CAN_UP_1M))

        outcome = tool.ensure()

        assert outcome.state == LINK_CONFIGURED
        assert "500000" in outcome.detail and "已配置" in outcome.detail

    def test_a_downed_interface_at_the_right_bitrate_is_raised(self) -> None:
        """Everything about this state reads correct except ``UP``, which is what
        makes it the one that gets mistaken for ready — and a down interface is
        precisely what the SDK cannot open, so mistaking it means offering no
        help at the one moment the operator needs it."""
        tool, run = link(ok(CAN_DOWN_CONFIGURED), (0, "", ""), ok(CAN_UP_1M))

        outcome = tool.ensure()

        assert outcome.state == LINK_CONFIGURED
        assert len(run.pkexec_calls) == 1, "it has to be raised, not left alone"

    def test_a_down_interface_is_raised(self) -> None:
        tool, run = link(ok(CAN_DOWN_UNCONFIGURED), (0, "", ""), ok(CAN_UP_1M))

        assert tool.ensure().state == LINK_CONFIGURED

    def test_privilege_is_asked_for_once(self) -> None:
        """Three ``pkexec`` calls would be three password dialogs, which is why
        the three commands live in one script."""
        tool, run = link(ok(CAN_UP_500K), (0, "", ""), ok(CAN_UP_1M))

        tool.ensure()

        assert len(run.pkexec_calls) == 1

    def test_a_bus_off_interface_is_reconfigured_rather_than_left_alone(self) -> None:
        """The case that reached the operator as ``errno 100 网络已断开``: the
        interface was left alone on the strength of fields that all read right,
        and the failure only surfaced on the first send — 使能, because connect
        merely opens a socket."""
        tool, run = link(ok(CAN_UP_BUS_OFF), (0, "", ""), ok(CAN_UP_1M))

        outcome = tool.ensure()

        assert outcome.state == LINK_CONFIGURED
        assert len(run.pkexec_calls) == 1, "it has to be re-raised, not left alone"
        assert "BUS-OFF" in outcome.detail, "先说清它原来是什么状态"

    def test_a_bus_off_that_survives_configuration_names_the_likely_causes(self) -> None:
        """Raised and configured and still off the bus: a controller only gets
        there by sending frames nobody acknowledged, so the fault is on the far
        side of the wire.  Three causes, and the console cannot tell which — but
        it can say all three rather than reporting a generic wrong state."""
        tool, run = link(ok(CAN_UP_BUS_OFF), (0, "", ""), ok(CAN_UP_BUS_OFF))

        outcome = tool.ensure()

        assert outcome.state == LINK_FAILED
        assert outcome.needs_attention
        assert "BUS-OFF" in outcome.detail and "应答" in outcome.detail
        assert "驱动器没上电" in outcome.detail

    def test_the_interface_is_configured_with_auto_restart(self) -> None:
        """Without ``restart-ms`` the kernel default is 0 — a bus-off controller
        stays off until someone runs down/up by hand, which is what made this a
        permanent state the operator had to recognise on their own."""
        tool, run = link(ok(CAN_UP_500K), (0, "", ""), ok(CAN_UP_1M))

        tool.ensure()

        script = run.pkexec_calls[0][3]
        assert f'restart-ms "$3"' in script
        assert "$3" in script and "restart-ms" in script
        assert str(constants.CAN_LINK_RESTART_MS) in run.pkexec_calls[0]

    def test_the_device_reaches_the_script_as_an_argument(self) -> None:
        """Not spliced into the text a shell will read: ``$1`` is the interface,
        ``$2`` the bitrate and ``$3`` the restart delay, and none of them is ever
        part of the script."""
        tool, run = link(ok(CAN_UP_500K), (0, "", ""), ok(CAN_UP_1M))

        tool.ensure()

        argv = run.pkexec_calls[0]
        script = argv[3]
        assert "can0" not in script, "the interface name must not be in the script"
        assert argv[4:] == [
            "litegrip-can",
            "can0",
            "1000000",
            str(constants.CAN_LINK_RESTART_MS),
        ]
        assert "$1" in script and "$2" in script and "$3" in script

    def test_the_privileged_command_is_not_a_shell_string(self) -> None:
        """``sh -c`` is handed a fixed script; nothing else about the call is a
        string, so there is no second place for the name to be reinterpreted."""
        tool, run = link(ok(CAN_UP_500K), (0, "", ""), ok(CAN_UP_1M))

        tool.ensure()

        argv = run.pkexec_calls[0]
        assert argv[:3] == [PKEXEC_PATH, "/bin/sh", "-c"]
        assert isinstance(argv, list)

    def test_an_fd_interface_is_reported_and_left_exactly_as_it_is(self) -> None:
        """``fd off`` would rewrite a bus that other nodes are on, and the SDK
        reads the MTU and speaks FD by itself — so this is a bus that may be
        working perfectly.  Certainty is the bar for changing anything, and here
        there is none."""
        tool, run = link(ok(CAN_UP_FD))

        outcome = tool.ensure()

        assert outcome.state == LINK_FD
        assert run.pkexec_calls == [], "an FD bus is not ours to rewrite"
        assert "FD" in outcome.detail and "未改动" in outcome.detail
        assert outcome.after == outcome.before, "not even a re-probe"

    def test_an_fd_interface_is_not_an_alarm(self) -> None:
        """It may connect fine.  Raising a banner for a bus that then works
        teaches the operator to ignore banners."""
        tool, _run = link(ok(CAN_UP_FD))

        assert not tool.ensure().needs_attention

    def test_a_missing_interface_does_not_ask_for_privilege(self) -> None:
        """Nothing to raise: the adapter is unplugged, and no password can fix
        that.  The connect attempt still goes ahead and will say so."""
        tool, run = link((1, "", REAL_MISSING), channel="can0")

        outcome = tool.ensure()

        assert outcome.state == LINK_MISSING
        assert run.pkexec_calls == []
        assert "USB-CAN" in outcome.detail

    def test_an_implausible_interface_name_is_refused_before_anything_runs(self) -> None:
        """The one value that comes from outside.  It is passed as an argument
        rather than spliced in, and it is also refused outright — two
        independent reasons it cannot reach a shell."""
        tool, run = link(ok(CAN_UP_500K), channel="can0; rm -rf /")

        outcome = tool.ensure()

        assert outcome.state == LINK_FAILED
        assert run.calls == [], "nothing may run for a name like that"
        assert outcome.needs_attention

    def test_a_bitrate_ip_would_reject_is_refused_too(self) -> None:
        tool, run = link(ok(CAN_UP_500K), bitrate=8_000_000)

        outcome = tool.ensure()

        assert outcome.state == LINK_FAILED
        assert run.calls == []

    def test_without_ip_or_pkexec_it_says_so_instead_of_failing(self) -> None:
        run = FakeRun()
        tool = CanLink("can0", run=run, which=lambda name: None)

        outcome = tool.ensure()

        assert outcome.state == LINK_FAILED
        assert run.calls == []
        assert "手动执行" in outcome.detail

    def test_the_interface_is_asked_again_before_success_is_claimed(self) -> None:
        """A privileged command that reported success is not evidence that the
        state changed; the bus is what says so."""
        rerun = (0, CAN_UP_500K, "")
        tool, run = link(ok(CAN_UP_500K), (0, "", ""), *[rerun] * 1)

        outcome = tool.ensure()

        assert outcome.state == LINK_FAILED
        assert len(run.calls) == 3, "probe, escalate, probe again"

    def test_every_message_that_offers_a_way_out_carries_the_manual_command(self) -> None:
        """For an interface that exists, the manual route is real: the operator
        who cannot answer the dialog — or does not want to — can paste the
        commands the README documents and get the same result."""
        problems = [
            (ok(CAN_UP_500K), (PKEXEC_DISMISSED, "", "")),
            (ok(CAN_UP_500K), (1, "", "RTNETLINK answers: Operation not permitted")),
            (ok(CAN_UP_500K), (0, "", ""), ok(CAN_UP_500K)),  # ran, still wrong
        ]

        for results in problems:
            tool, _run = link(*results)
            detail = tool.ensure().detail
            assert "ip link set can0" in detail, detail

    def test_a_missing_interface_is_not_offered_a_command_that_cannot_work(self) -> None:
        """The same commands fail on a device that does not exist, so quoting
        them would send the operator after the wrong thing entirely.  What they
        need to hear is the hardware, and how to tell a wrong name from an
        unplugged adapter."""
        tool, _run = link((1, "", REAL_MISSING))

        detail = tool.ensure().detail

        assert "ip link set can0" not in detail
        assert "USB-CAN" in detail
        assert "ip link" in detail, "the wrong-name case looks identical from here"


class TestWhenPrivilegeIsRefused:
    def test_a_dismissed_dialog(self) -> None:
        """The operator's own decision, and it leaves the interface alone."""
        tool, _run = link(ok(CAN_UP_500K), (PKEXEC_DISMISSED, "", ""))

        outcome = tool.ensure()

        assert outcome.state == LINK_DENIED
        assert "取消" in outcome.detail

    def test_an_authorization_that_did_not_come_through(self) -> None:
        tool, _run = link(ok(CAN_UP_500K), (PKEXEC_NOT_AUTHORIZED, "", "Not authorized"))

        outcome = tool.ensure()

        assert outcome.state == LINK_DENIED
        assert "Not authorized" in outcome.detail

    def test_a_script_that_ran_and_failed_is_not_a_denial(self) -> None:
        """Different problem, different message: nobody was refused here, the
        command itself did not work."""
        tool, _run = link(ok(CAN_UP_500K), (1, "", "RTNETLINK answers: Device or resource busy"))

        outcome = tool.ensure()

        assert outcome.state == LINK_FAILED
        assert outcome.needs_attention
        assert "resource busy" in outcome.detail

    def test_a_refusal_is_still_not_an_error_the_operator_must_clear(self) -> None:
        """It is reported, not latched: the connect attempt that follows is what
        decides whether anything is wrong."""
        tool, _run = link(ok(CAN_UP_500K), (PKEXEC_DISMISSED, "", ""))

        assert tool.ensure().needs_attention, "dismissing the dialog is worth saying"

    @pytest.mark.parametrize("state_name", [LINK_OK, LINK_CONFIGURED])
    def test_nothing_that_worked_raises_a_banner(self, state_name: str) -> None:
        results = [ok(CAN_UP_1M)] if state_name == LINK_OK else [
            ok(CAN_UP_500K), (0, "", ""), ok(CAN_UP_1M)
        ]
        tool, _run = link(*results)

        assert not tool.ensure().needs_attention


class TestTheProcessRunner:
    """The un-faked half: what actually goes to the operating system.

    Everything above injects this away, which is the right way to test the
    decisions but the wrong way to test the call.  These pin the two properties
    that must hold however the caller is written.
    """

    def test_no_shell_is_ever_involved(self, monkeypatch) -> None:
        seen: list[dict] = []

        def fake_run(argv, **kwargs):
            seen.append({"argv": argv, **kwargs})
            return subprocess.CompletedProcess(argv, 0, "", "")

        monkeypatch.setattr(subprocess, "run", fake_run)
        tool = CanLink("can0", run=None, which=lambda name: name)

        tool._run_process(["/usr/sbin/ip", "link", "show", "can0"], 5.0)

        assert seen[0]["argv"] == ["/usr/sbin/ip", "link", "show", "can0"]
        assert "shell" not in seen[0], "no shell, in any spelling"

    def test_a_timeout_is_a_denial_rather_than_an_exception(self, monkeypatch) -> None:
        """The realistic cause is a dialog nobody answered, and it must not
        travel up into the connect path as a traceback."""
        def fake_run(argv, **kwargs):
            raise subprocess.TimeoutExpired(argv, kwargs.get("timeout", 0))

        monkeypatch.setattr(subprocess, "run", fake_run)

        result = CanLink("can0")._run_process(["/usr/bin/pkexec", "true"], 120.0)

        assert result.returncode == PKEXEC_NOT_AUTHORIZED
        assert "未完成" in result.stderr

    def test_a_missing_binary_is_a_failure_rather_than_an_exception(self, monkeypatch) -> None:
        def fake_run(argv, **kwargs):
            raise FileNotFoundError(2, "No such file or directory")

        monkeypatch.setattr(subprocess, "run", fake_run)

        result = CanLink("can0")._run_process(["/usr/bin/pkexec", "true"], 5.0)

        assert result.returncode == 127


class TestTellingATransportFailureApart:
    """Which failures are the interface's, and which belong to the motor.

    ``errno 100`` is the kernel's answer to "this interface cannot carry a
    frame" — a down interface, an unplugged adapter, or a controller that has
    gone bus-off — and it reaches the operator as "使能失败: [Errno 100]
    网络已断开", which names neither the interface nor anything to do with it.
    Recognising it is what lets the GUI report a link problem as a link
    problem.

    The wrapped case is the one that actually happens: the SDK stringifies the
    exception (``gripper.py:221``, ``raise HardwareError(f"使能失败: {e}")``),
    so the errno survives only in the implicit ``__context__`` chain.
    """

    def test_a_bare_transport_errno_is_recognised(self) -> None:
        reason = link_failure(OSError(100, "Network is down"), "can0")

        assert reason is not None
        assert "can0" in reason
        assert "ENETDOWN" in reason

    def test_the_sdks_own_wrapping_is_looked_through(self) -> None:
        """The shape ``gripper.enable`` really produces: ``except Exception as e:
        raise HardwareError(f"使能失败: {e}")``, so the number the operator sees
        is inside a sentence."""
        with pytest.raises(LiteGripError) as excinfo:
            try:
                raise OSError(100, "Network is down")
            except OSError as cause:
                raise LiteGripError(f"使能失败: {cause}")

        assert "Errno 100" in str(excinfo.value), "the text is what the SDK keeps"
        assert "ENETDOWN" in (link_failure(excinfo.value, "can0") or "")

    def test_the_chain_is_read_even_when_the_text_says_nothing(self) -> None:
        """Isolates the chain walk from the text fallback: this wrapper throws
        the errno away and only ``__context__`` still has it."""
        with pytest.raises(LiteGripError) as excinfo:
            try:
                raise OSError(100, "Network is down")
            except OSError:
                raise LiteGripError("使能失败: 驱动器无响应")

        assert "Errno" not in str(excinfo.value), "nothing for the text path to find"
        assert "ENETDOWN" in (link_failure(excinfo.value, "can0") or "")

    def test_an_explicit_cause_is_followed_too(self) -> None:
        with pytest.raises(LiteGripError) as excinfo:
            try:
                raise OSError(6, "No such device or address")
            except OSError as cause:
                raise LiteGripError("使能失败: 适配器没了") from cause

        assert "ENXIO" in (link_failure(excinfo.value, "can0") or "")

    def test_a_message_that_only_stringifies_the_errno_still_counts(self) -> None:
        """Belt and braces for a chain that was not kept at all — the number in
        the text is the same number, in any locale."""
        reason = link_failure(LiteGripError("使能失败: [Errno 100] 网络已断开"), "can0")

        assert reason is not None and "ENETDOWN" in reason

    def test_the_advice_names_the_problem_and_the_way_out(self) -> None:
        reason = link_failure(OSError(100, "Network is down"), "can0") or ""

        assert "总线关闭" in reason
        assert "重新连接" in reason

    def test_the_channel_is_named_when_there_is_one(self) -> None:
        assert (link_failure(OSError(100, "x")) or "").startswith("CAN 接口")
        assert (link_failure(OSError(100, "x"), "can1") or "").startswith("can1")

    @pytest.mark.parametrize("number", [100, 6, 19])
    def test_the_interfaces_own_numbers_are_the_ones_recognised(self, number: int) -> None:
        assert link_failure(OSError(number, "x"), "can0") is not None

    @pytest.mark.parametrize("number", [16, 110, 32, 11])
    def test_other_errnos_are_left_to_the_motor_to_explain(self, number: int) -> None:
        """EBUSY, ETIMEDOUT, EPIPE, EAGAIN: not this module's to interpret.
        Claiming them would send the operator to the adapter for a problem that
        is not there."""
        assert link_failure(OSError(number, "x"), "can0") is None

    def test_a_motor_error_is_not_a_link_error(self) -> None:
        assert link_failure(LiteGripError("初始化失败：所有重试耗尽"), "can0") is None
        assert link_failure(LiteGripError("电机欠压故障", error_code=0x9), "can0") is None

    def test_a_self_referential_chain_terminates(self) -> None:
        """``__context__`` cycles are legal to construct, and this runs on the
        enable path — hanging there would hang the worker thread."""
        exc = LiteGripError("使能失败")
        exc.__context__ = exc

        assert link_failure(exc, "can0") is None
