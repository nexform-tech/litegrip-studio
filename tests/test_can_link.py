"""Raising the CAN interface: what it reads, decides, and refuses.

Two halves, and the second is the one that matters.  The first is reading
``ip``'s answer, which is pure and is pinned here against real transcripts.  The
second is everything about the *privileged* half: that it runs no command when
the interface is already right, that the interface name is never spliced into
the script a shell will read, that a refusal leaves the interface alone, and that
none of it can raise into the connect path.

The subprocess is injected, so the suite never runs the real ``ip``, never asks
for a password, and never needs an interface to exist.  The script itself is the
one part that cannot be tested that way — its fallback is shell control flow —
so the last class runs it under a real ``sh`` against a stand-in ``ip``.
"""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

import pytest

import litegrip

#: What these tests are about is how the SDK's own failure messages are read, so
#: a stand-in would test the stand-in.  The SDK is vendored under
#: ``src/litegrip``, so it is always importable.
LiteGripError = litegrip.LiteGripError

from litegrip_studio import constants
from litegrip_studio.can_link import (
    DRIVER_RELOADED,
    LINK_CONFIGURED,
    LINK_DENIED,
    LINK_FAILED,
    LINK_FD,
    LINK_MISSING,
    LINK_OK,
    PKEXEC_DISMISSED,
    PKEXEC_NOT_AUTHORIZED,
    RELOADABLE_DRIVER,
    RESTART_MS_FALLBACK,
    SCRIPT,
    SCRIPT_NAME,
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

#: The kernel's refusal, captured on this bench's adapter (a gs_usb/candleLight
#: clone at ``1d50:606f``, kernel 6.8.0-138).  It arrives as an extack — ``strings
#: /usr/sbin/ip`` does not carry it — and it says "restart" while never saying
#: "restart-ms", which is what the fallback matches on.
ADAPTER_REJECTS_RESTART_MS = "Error: Device doesn't support restart from Bus Off.\n"

#: What that adapter's stderr looks like on a run the fallback rescued: the
#: marker, and nothing else.  The kernel's refusal is not repeated once it has
#: been acted on — the script says what it did, not what it was told.
FALLBACK_STDERR = RESTART_MS_FALLBACK + "\n"

#: ``ip``'s answer when the interface cannot be raised at all, seen on this bench
#: after the fallback had configured it (``sudo ip link set can0 up`` says the
#: same thing by hand).  The driver refuses to start the controller, and the
#: kernel reports it as ENOENT.
UP_REFUSED = "RTNETLINK answers: No such file or directory"

#: What the script's stderr looks like when it had to reload the driver: the
#: marker alone, printed after the interface is configured again.
RELOADED_STDERR = DRIVER_RELOADED + "\n"


def as_the_script_says(step: str, said: str) -> str:
    """The line the privileged script writes when *step* fails."""
    return f"litegrip: {step} failed: {said}\n"


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
        assert 'restart_ms="$3"' in script
        assert 'restart-ms "$restart_ms"' in script
        assert str(constants.CAN_LINK_RESTART_MS) in run.pkexec_calls[0]
        # And the one retry, without the option, for the adapters that refuse it.
        # The count rather than a substring: a second ``type can bitrate`` is
        # what makes the retry possible at all.
        assert script.count("type can bitrate") == 2, script
        assert RESTART_MS_FALLBACK in script, "the retry says that it took place"
        assert 'ip link set "$dev" up' in script
        # The locale is pinned before anything can fail, because the one failure
        # the script reads back — ENOENT, the trigger for the reload — is matched
        # by its English wording rather than by an exit code that cannot tell it
        # from any other refusal.
        assert script.index("export LC_ALL") < script.index("ip link set")

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
            (
                ok(CAN_UP_500K),
                (1, "", as_the_script_says("configure", "RTNETLINK answers: Operation not permitted")),
                ok(CAN_DOWN_CONFIGURED),
            ),
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


class TestAnAdapterThatRefusesRestartMs:
    """The bench's gs_usb clone, and the bug it produced.

    ``restart-ms`` is not implemented by every adapter, and this one says so
    loudly: the configure command fails, and under ``set -e`` the script stopped
    right there — before ``up``.  So the interface was left *down*, the console
    reported 未被改动, and the manual command it offered carried the same option
    and failed the same way.  Three defects, one cause, and it reached the
    operator more than once.
    """

    def test_the_interface_comes_up_anyway(self) -> None:
        """The fallback, end to end: the adapter refuses the option, the script
        retries without it, and the probe that follows says the link is up."""
        tool, run = link(
            ok(CAN_DOWN_CONFIGURED),
            (0, "", FALLBACK_STDERR),
            ok(CAN_UP_1M),
        )

        outcome = tool.ensure()

        assert outcome.state == LINK_CONFIGURED
        assert len(run.pkexec_calls) == 1, "one retry inside the script, still one dialog"
        assert "比特率 1000000" in outcome.detail
        assert "restart-ms" in outcome.detail, "the difference is said out loud"

    def test_it_is_not_an_alarm(self) -> None:
        """The interface is up and the SDK can drive it.  A banner for that would
        teach the operator to ignore banners."""
        tool, _run = link(ok(CAN_DOWN_CONFIGURED), (0, "", FALLBACK_STDERR), ok(CAN_UP_1M))

        assert not tool.ensure().needs_attention

    def test_a_plain_success_says_nothing_about_restart_ms(self) -> None:
        """The other half of the same rule: an adapter that takes the option must
        not be told about a fallback that never ran."""
        tool, _run = link(ok(CAN_DOWN_CONFIGURED), (0, "", ""), ok(CAN_UP_1M))

        assert "restart-ms" not in tool.ensure().detail

    def test_a_failure_still_reports_what_is_true_and_what_to_type(self) -> None:
        """The command failed, so the interface is wherever the script left it —
        and the operator is handed a command their adapter can actually run."""
        tool, run = link(
            ok(CAN_UP_500K),
            (1, "", as_the_script_says("configure", ADAPTER_REJECTS_RESTART_MS.strip())),
            ok(CAN_DOWN_CONFIGURED),
        )

        outcome = tool.ensure()

        assert outcome.state == LINK_FAILED
        assert len(run.calls) == 3, "probe, escalate, probe again"
        assert "未被改动" not in outcome.detail
        assert "配置接口失败" in outcome.detail, "which step failed, not a blob of stderr"
        assert "比特率 1000000" in outcome.detail, "what the kernel says now"
        assert "ip link set can0" in outcome.detail
        assert "restart-ms" not in outcome.detail, "the hint drops what it refused"

    def test_a_failure_that_came_from_somewhere_else_is_still_quoted(self) -> None:
        """Not everything on stderr is the script's.  A shell that died before it
        could label anything, or a stale script that never learned to, still has
        to reach the operator."""
        tool, _run = link(ok(CAN_UP_500K), (2, "", "Error executing /bin/sh: No such file"))

        outcome = tool.ensure()

        assert outcome.state == LINK_FAILED
        assert "No such file" in outcome.detail

    def test_the_step_that_failed_is_the_last_one_the_script_named(self) -> None:
        """The bench's own run, in one message: the fallback configured the
        interface, and the step that stopped the script was ``up``.  Saying
        "configure failed" there would send the operator after the wrong thing —
        the option had already been dropped and the bitrate was applied."""
        tool, _run = link(
            ok(CAN_DOWN_CONFIGURED),
            (1, "", FALLBACK_STDERR + as_the_script_says("up", UP_REFUSED)),
            ok(CAN_DOWN_CONFIGURED),
        )

        outcome = tool.ensure()

        assert outcome.state == LINK_FAILED
        assert "把接口 up 失败" in outcome.detail
        assert UP_REFUSED in outcome.detail
        assert "restart-ms unsupported" not in outcome.detail, "the marker is ours, not his"
        assert "已去掉适配器不支持的 restart-ms 重试" in outcome.detail
        assert "未 up，经典 CAN，比特率 1000000" in outcome.detail
        assert "ip link set can0" not in outcome.detail, "see the class below"

    def test_the_fallback_is_also_answered_when_the_controller_stays_off(self) -> None:
        """Up, configured and still BUS-OFF: the fallback ran, and the advice
        that follows is about the bus rather than about the option."""
        tool, _run = link(ok(CAN_DOWN_CONFIGURED), (0, "", FALLBACK_STDERR), ok(CAN_UP_BUS_OFF))

        detail = tool.ensure().detail

        assert "BUS-OFF" in detail
        assert "restart-ms" not in detail

    def test_a_dismissed_dialog_is_not_probed_again(self) -> None:
        """The other side of the re-probe: nothing ran, so ``before`` is the
        truth, 未被改动 is accurate, and a second probe would cost the operator a
        subprocess while a dialog they never answered is still on screen."""
        tool, run = link(ok(CAN_UP_500K), (PKEXEC_DISMISSED, "", ""))

        outcome = tool.ensure()

        assert outcome.state == LINK_DENIED
        assert len(run.calls) == 2
        assert "未被改动" in outcome.detail


#: A stand-in for ``ip``.  It records the arguments it was handed, because the
#: order and the identity of the commands *is* what is under test here, and it
#: behaves as ``IP_MODE`` asks.
#:
#: ``uphealed`` is the bench's own sequence: ``up`` is refused until the driver
#: has been rebound, exactly as it was refused until ``modprobe -r gs_usb &&
#: modprobe gs_usb`` was run by hand.  It reads the file the modprobe stand-in
#: touches, which is how the two stubs model one machine.
STUB_IP = """\
#!/bin/sh
echo "$*" >> "$IP_LOG"
refuse() {
    case "$1" in
        restart) echo "Error: Device doesn't support restart from Bus Off." >&2 ;;
        busy)    echo "RTNETLINK answers: Device or resource busy" >&2 ;;
        up)      echo "RTNETLINK answers: No such file or directory" >&2 ;;
        missing) echo 'Cannot find device "can0"' >&2 ;;
    esac
    exit 1
}
case "$IP_MODE" in
    rejects)  case "$*" in *restart-ms*) refuse restart ;; esac ;;
    busy)     case "$*" in *restart-ms*) refuse busy ;; esac ;;
    always)   case "$*" in *type*) refuse restart ;; esac ;;
    upfails)  case "$*" in
                  *restart-ms*) refuse restart ;;
                  "link set can0 up") refuse up ;;
              esac ;;
    uphealed) case "$*" in
                  *restart-ms*) refuse restart ;;
                  "link set can0 up") [ -e "$IP_REBOUND" ] || refuse up ;;
              esac ;;
    noreturn) case "$*" in
                  *restart-ms*) refuse restart ;;
                  "link set can0 up") refuse up ;;
                  "link show can0") exit 1 ;;
              esac ;;
    downfails) case "$*" in "link set can0 down") refuse missing ;; esac ;;
esac
exit 0
"""

#: A stand-in for ``modprobe``: it records what it was asked to unload in the
#: same log as the ``ip`` stand-in — one file, so the order across both tools is
#: what the assertions read — and touches another file so that ``ip`` can tell
#: whether the rebind has happened yet.  ``MODPROBE_MODE`` decides whether it
#: works at all.
STUB_MODPROBE = """\
#!/bin/sh
echo "modprobe $*" >> "$IP_LOG"
case "$MODPROBE_MODE" in
    fails) echo "modprobe: FATAL: Module gs_usb is in use." >&2; exit 1 ;;
esac
[ "$1" = "-r" ] || : > "$IP_REBOUND"
exit 0
"""

#: A stand-in for ``readlink``, answering for the driver symlink and nothing
#: else.  The real ``readlink -f`` on a path that is not there still prints it,
#: which is what a virtual interface looks like from the script.
STUB_READLINK = """\
#!/bin/sh
case "$2" in
    */device/driver) [ -n "$DRIVER_NAME" ] && echo "/sys/bus/usb/drivers/$DRIVER_NAME" ;;
    *) echo "$2" ;;
esac
exit 0
"""


def run_the_script(
    tmp_path: Path,
    mode: str,
    *,
    driver: str = RELOADABLE_DRIVER,
    modprobe: str = "ok",
) -> tuple[subprocess.CompletedProcess, list[str]]:
    """Run the real script under a real ``sh``, with stand-ins first on ``PATH``.

    No privilege, no interface, no kernel.  ``ip``, ``modprobe`` and ``readlink``
    are all stubbed, and they are stubbed for every test rather than only for the
    ones that expect them to be called: a test that reaches the reload branch by
    accident must not be able to unload a module on the machine running the
    suite.  The log the stubs share is the record of what the script asked for,
    in order.
    """
    stubs = {
        "ip": STUB_IP,
        "modprobe": STUB_MODPROBE,
        "readlink": STUB_READLINK,
    }
    for name, text in stubs.items():
        path = tmp_path / name
        path.write_text(text, encoding="utf-8")
        path.chmod(0o755)
    log = tmp_path / "ip-arguments"
    env = {
        **os.environ,
        "PATH": os.pathsep.join([str(tmp_path), os.environ.get("PATH", "")]),
        "IP_LOG": str(log),
        "IP_MODE": mode,
        "IP_REBOUND": str(tmp_path / "rebound"),
        "MODPROBE_MODE": modprobe,
        "DRIVER_NAME": driver,
    }

    result = subprocess.run(
        [
            "/bin/sh", "-c", SCRIPT, SCRIPT_NAME,
            "can0", "1000000", str(constants.CAN_LINK_RESTART_MS),
        ],
        capture_output=True,
        text=True,
        env=env,
    )
    calls = log.read_text(encoding="utf-8").splitlines() if log.exists() else []
    return result, calls


class TestTheScriptItself:
    """The script's control flow, which the injected subprocess cannot reach.

    ``set -e`` is what makes the fallback delicate: the retry has to happen
    *after* the failure it answers and *before* ``up``, and a retry that fails
    must not fall through to raising an interface with no bitrate.  Run under a
    real shell against a stub, because that is what the privileged half is.
    """

    DOWN = "link set can0 down"
    WITH_RESTART = "link set can0 type can bitrate 1000000 restart-ms 100 fd off"
    WITHOUT_RESTART = "link set can0 type can bitrate 1000000 fd off"
    UP = "link set can0 up"

    def test_an_adapter_that_refuses_the_option_is_retried_without_it(
        self, tmp_path: Path
    ) -> None:
        result, calls = run_the_script(tmp_path, "rejects")

        assert result.returncode == 0, result.stderr
        assert calls == [self.DOWN, self.WITH_RESTART, self.WITHOUT_RESTART, self.UP]
        assert result.stderr == RESTART_MS_FALLBACK + "\n", "one line, and only that"

    def test_an_adapter_that_takes_the_option_is_not_retried(self, tmp_path: Path) -> None:
        result, calls = run_the_script(tmp_path, "ok")

        assert result.returncode == 0, result.stderr
        assert calls == [self.DOWN, self.WITH_RESTART, self.UP]
        assert result.stderr == ""

    def test_another_failure_is_not_quietly_downgraded(self, tmp_path: Path) -> None:
        """Only the kernel's own wording for this option buys a retry.  Retrying
        any other failure would trade an intermittent error for an interface that
        comes up and silently never auto-recovers."""
        result, calls = run_the_script(tmp_path, "busy")

        assert result.returncode != 0
        assert calls == [self.DOWN, self.WITH_RESTART], "no retry, and no up"
        assert result.stderr == as_the_script_says("configure", "RTNETLINK answers: Device or resource busy")

    def test_a_retry_that_fails_does_not_raise_an_unconfigured_interface(
        self, tmp_path: Path
    ) -> None:
        """``set -e`` still guards the tail: the retry ran, failed, and the script
        stopped before ``up`` — it must not leave a bitrate-less interface up."""
        result, calls = run_the_script(tmp_path, "always")

        assert result.returncode != 0
        assert self.WITHOUT_RESTART in calls, "the retry did run"
        assert self.UP not in calls

    def test_the_bench_adapter_configures_and_then_cannot_be_raised(
        self, tmp_path: Path
    ) -> None:
        """The run that produced the message on the bench, pinned as a script.

        The option is refused, the retry configures the interface, the marker is
        printed — and then ``up`` answers ENOENT, which is what actually stopped
        it.  That answer is now the trigger for one more attempt: the driver is
        rebound, the fresh interface is configured again, and ``up`` is tried
        once more.  Two different faults in one run, and the order of these lines
        is the whole reason a reader can tell them apart.
        """
        result, calls = run_the_script(tmp_path, "upfails")

        assert result.returncode != 0
        assert calls == [
            self.DOWN,
            self.WITH_RESTART,
            self.WITHOUT_RESTART,
            self.UP,
            "modprobe -r gs_usb",
            "modprobe gs_usb",
            "link show can0",
            self.WITH_RESTART,
            self.WITHOUT_RESTART,
            self.UP,
        ], "the reload happened between the two raises, and only once"
        assert result.stderr == (
            RESTART_MS_FALLBACK + "\n"
            + RELOADED_STDERR
            + as_the_script_says("up", UP_REFUSED)
        ), "the markers say what the script did; the last line is what failed"

    def test_a_rebound_driver_is_what_raises_the_interface(self, tmp_path: Path) -> None:
        """The bench's own repair, in the script: ``up`` is refused until the
        driver has been rebound — run by hand there, and run here by the console.
        The interface that comes back from the rebind has no bitrate, so it is
        configured again before it is raised."""
        result, calls = run_the_script(tmp_path, "uphealed")

        assert result.returncode == 0, result.stderr
        assert calls == [
            self.DOWN,
            self.WITH_RESTART,
            self.WITHOUT_RESTART,
            self.UP,
            "modprobe -r gs_usb",
            "modprobe gs_usb",
            "link show can0",
            self.WITH_RESTART,
            self.WITHOUT_RESTART,
            self.UP,
        ]
        assert result.stderr == RESTART_MS_FALLBACK + "\n" + RELOADED_STDERR

    def test_another_drivers_interface_is_never_unbound(self, tmp_path: Path) -> None:
        """Unbinding a driver resets every interface it serves, so the console
        does it for one driver only — the one this was diagnosed on.  Any other
        adapter gets the failure reported and its driver left alone."""
        result, calls = run_the_script(tmp_path, "uphealed", driver="kvaser_pciefd")

        assert result.returncode != 0
        assert not [call for call in calls if call.startswith("modprobe")], calls
        assert calls == [self.DOWN, self.WITH_RESTART, self.WITHOUT_RESTART, self.UP]
        assert result.stderr == (
            RESTART_MS_FALLBACK + "\n" + as_the_script_says("up", UP_REFUSED)
        )

    def test_a_driver_that_cannot_be_unloaded_says_so(self, tmp_path: Path) -> None:
        """``modprobe -r`` fails when the module is in use, and then the fault is
        in the reload rather than in the interface: it is named as that step, and
        the interface is not raised again behind its back."""
        result, calls = run_the_script(tmp_path, "uphealed", modprobe="fails")

        assert result.returncode != 0
        assert result.stderr == (
            RESTART_MS_FALLBACK + "\n"
            + as_the_script_says(
                "reload", "modprobe: FATAL: Module gs_usb is in use."
            )
        )
        assert calls.count(self.UP) == 1, "no second attempt after a failed reload"

    def test_an_interface_that_never_comes_back_says_so(self, tmp_path: Path) -> None:
        """The rebind is asynchronous, so the script waits — up to five seconds
        here, which is what this test costs.  When the interface does not return
        the script must not go on to configure and raise something that is not
        there."""
        result, calls = run_the_script(tmp_path, "noreturn")

        assert result.returncode != 0
        assert result.stderr == (
            RESTART_MS_FALLBACK + "\n"
            + as_the_script_says(
                "reload", "can0 did not come back after reloading /sys/bus/usb/drivers/gs_usb"
            )
        )
        assert calls.count(self.UP) == 1, "the interface was never raised again"

    def test_a_step_that_fails_first_says_so(self, tmp_path: Path) -> None:
        """The interface is only touched by ``down``, so a failure there is the
        one case where it really was left alone — worth being able to tell."""
        result, calls = run_the_script(tmp_path, "downfails")

        assert result.returncode != 0
        assert calls == [self.DOWN], "nothing else ran"
        assert result.stderr == as_the_script_says("down", "Cannot find device \"can0\"")


class TestAnAdapterThatLostItsUsbEndpoints:
    """The second state this bench reaches, and the one no command can leave.

    ``up`` answers ENOENT while the interface is configured exactly right: the
    driver has lost the USB endpoint table and only a fresh probe of the device
    puts it back.  Run by hand, ``modprobe -r gs_usb && modprobe gs_usb`` is what
    repaired it; the script now does that itself, and these are the messages for
    the two ways it can end.  The one thing no message here may do is hand the
    operator another ``ip link`` command, which is the hint that has already been
    proved not to work.
    """

    def test_a_reload_that_worked_is_reported_without_a_banner(self) -> None:
        """The interface is up and the SDK can drive it, so this is a log line —
        but the reload took the machine's other gs_usb interfaces with it, and
        that consequence is worth the words."""
        tool, run = link(
            ok(CAN_DOWN_CONFIGURED),
            (0, "", FALLBACK_STDERR + RELOADED_STDERR),
            ok(CAN_UP_1M),
        )

        outcome = tool.ensure()

        assert outcome.state == LINK_CONFIGURED
        assert not outcome.needs_attention
        assert len(run.pkexec_calls) == 1, "one script, one dialog, reload included"
        assert "已重载 gs_usb 驱动修好" in outcome.detail
        assert "其它同驱动接口也被一起重置" in outcome.detail

    def test_a_plain_success_does_not_mention_a_reload(self) -> None:
        """The other half of the same rule, and the one the bench usually sees."""
        tool, _run = link(ok(CAN_DOWN_CONFIGURED), (0, "", ""), ok(CAN_UP_1M))

        detail = tool.ensure().detail

        assert "重载" not in detail and "gs_usb" not in detail

    def test_a_reload_that_did_not_help_sends_the_operator_to_the_adapter(
        self,
    ) -> None:
        """The reload ran and ``up`` still refuses: there is nothing left to
        configure, so the advice is the device rather than the interface, and the
        reload that was already tried is not offered again as if it were new."""
        tool, _run = link(
            ok(CAN_DOWN_CONFIGURED),
            (
                1,
                "",
                FALLBACK_STDERR + RELOADED_STDERR + as_the_script_says("up", UP_REFUSED),
            ),
            ok(CAN_DOWN_CONFIGURED),
        )

        detail = tool.ensure().detail

        assert "把接口 up 失败" in detail
        assert UP_REFUSED in detail
        assert "已自动重载 gs_usb 驱动重试，仍未起来" in detail
        assert "USB 层没有就绪" in detail
        assert "处理：拔插一次适配器" in detail
        assert "ip link set can0" not in detail
        assert "手动执行" not in detail, "this is not a command to paste"

    def test_a_driver_this_console_will_not_unbind_is_offered_by_hand(self) -> None:
        """The script left another driver alone on purpose, so the manual command
        names gs_usb — and only as something the operator may try, since the
        console cannot know that this adapter is one."""
        tool, _run = link(
            ok(CAN_DOWN_CONFIGURED),
            (1, "", as_the_script_says("up", UP_REFUSED)),
            ok(CAN_DOWN_CONFIGURED),
        )

        detail = tool.ensure().detail

        assert "拔插一次适配器" in detail
        assert "sudo modprobe -r gs_usb && sudo modprobe gs_usb" in detail
        assert "ip link set can0" not in detail

    def test_a_reload_that_failed_quotes_what_modprobe_said(self) -> None:
        """``Module gs_usb is in use`` is a fact about this machine that no
        generic sentence in this module could have guessed, so it is passed
        through — and the step it is attributed to is the reload, not ``up``."""
        tool, _run = link(
            ok(CAN_DOWN_CONFIGURED),
            (
                1,
                "",
                FALLBACK_STDERR
                + as_the_script_says("reload", "modprobe: FATAL: Module gs_usb is in use."),
            ),
            ok(CAN_DOWN_CONFIGURED),
        )

        detail = tool.ensure().detail

        assert "重载适配器驱动失败" in detail
        assert "Module gs_usb is in use" in detail
        assert "拔插一次适配器" in detail
        assert "ip link set can0" not in detail

    def test_an_up_failure_that_is_not_this_one_keeps_the_usual_advice(self) -> None:
        """The guard on the whole class: ENOENT is the trigger, nothing wider.
        Every other way ``up`` can fail is still answered with the commands the
        README documents, because for those the interface really is the thing to
        fix."""
        tool, _run = link(
            ok(CAN_DOWN_CONFIGURED),
            (
                1,
                "",
                as_the_script_says("up", "RTNETLINK answers: Operation not permitted"),
            ),
            ok(CAN_DOWN_CONFIGURED),
        )

        detail = tool.ensure().detail

        assert "手动执行：sudo ip link set can0 down" in detail
        assert "拔插" not in detail


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
        command itself did not work.

        The state reported is the one the script *left behind*, not the one it
        found.  Its first command takes the interface down, so "未被改动" was a
        claim the operator could check and catch — on this bench, the interface
        really was down while the console said it had not been touched.
        """
        tool, run = link(
            ok(CAN_UP_500K),
            (1, "", as_the_script_says("configure", "RTNETLINK answers: Device or resource busy")),
            ok(CAN_DOWN_CONFIGURED),
        )

        outcome = tool.ensure()

        assert outcome.state == LINK_FAILED
        assert outcome.needs_attention
        assert "resource busy" in outcome.detail
        assert len(run.calls) == 3, "the failure is re-probed before it is described"
        assert "未被改动" not in outcome.detail
        assert "比特率 1000000" in outcome.detail, "and the probe's answer is quoted"

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
