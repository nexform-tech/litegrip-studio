"""The simulated gripper's dynamics.

These tests exist because the plant's coefficients are tuning choices, not
datasheet values: they have to be justified by the behaviour they produce, and
three of the behaviours below are regressions for modelling errors that were
found the hard way.

* ``TestContactIsASpring`` pins the property that made the contact model a
  spring instead of a position clamp.  As a clamp the reaction is an impulsive
  constraint force with no relation to the motor's torque — it produced 4600 N
  transients, and a reported force that was identical (112.50 N) at every
  setpoint from 5 N to 999 N because the constraint force, not the motor, was
  being reported.
* ``TestHardStops`` pins the sign of the stop damper.  Written ``+ c·dq`` on the
  lower stop it *accelerated* the jaws through the stop instead of absorbing
  them: a −22 Nm kick that threw the gripper from 117 mm back to 10 mm.
* The stop stiffness is chosen so a full-torque push penetrates a bounded
  fraction of a millimetre; that bound is asserted rather than assumed.
"""

from __future__ import annotations

import math

import pytest

from litegrip_studio import constants
from litegrip_studio.backend.plant import Plant, PlantConfig
from litegrip_studio.units import force_from_torque, torque_from_force

DT = constants.CTRL_DT
OBJECT_MM = 60.0


def run(plant: Plant, seconds: float, dt: float = DT) -> None:
    """Advance the plant in place for ``seconds`` of simulated time."""
    for _ in range(int(round(seconds / dt))):
        plant.step(dt)


def hold(plant: Plant, *, kp: float, kd: float = 0.0, tau: float = 0.0, q: float | None = None) -> None:
    """Stream a constant frame, the way the FSM does once per tick."""
    target = plant.q if q is None else q
    plant.stream(target, kp, kd, 0.0, tau)


class TestContactIsASpring:
    """A blocked gripper reports the force it is actually applying."""

    def test_the_jaws_penetrate_a_fraction_of_a_millimetre(self, plant: Plant) -> None:
        """A spring compresses; a position clamp would sit exactly on the object."""
        plant.object_mm = OBJECT_MM
        q_obj = plant.limits.to_rad(OBJECT_MM)
        plant.q = q_obj + 0.02  # already 1.3 mm into the object
        hold(plant, kp=0.0, tau=torque_from_force(40.0))
        run(plant, 2.0)

        penetration = plant.q - q_obj
        assert 0.0 < penetration < 0.02, "should rest just inside the object"
        assert plant.limits.to_mm(q_obj) - plant.limits.to_mm(plant.q) < 1.5

    @pytest.mark.parametrize("force_n", [5.0, 12.0, 20.0, 30.0, 40.0])
    def test_the_force_equals_the_setpoint(self, plant: Plant, force_n: float) -> None:
        """The regression: every setpoint used to report the same 112.50 N.

        That happened because the contact reaction was being *added* to the
        reported torque, so what came back was the constraint force.  The motor's
        own torque estimate is what the SDK reads, and at equilibrium that
        already equals the reaction — so it is the only thing to report.
        """
        plant.object_mm = OBJECT_MM
        q_obj = plant.limits.to_rad(OBJECT_MM)
        plant.q = q_obj + 0.05  # start well inside, then settle back
        hold(plant, kp=0.0, tau=torque_from_force(force_n))
        run(plant, 3.0)

        assert force_from_torque(plant.tau) == pytest.approx(force_n, abs=0.5)

    def test_the_equilibrium_is_consistent_with_the_spring(self, plant: Plant) -> None:
        """The resting penetration must equal ``tau / contact_k``.

        This is the self-consistency that a clamp cannot have: the same torque
        both deflects the spring and is reported, so the two must agree.
        """
        plant.object_mm = OBJECT_MM
        plant.q = plant.limits.to_rad(OBJECT_MM) + 0.01
        tau_ff = torque_from_force(20.0)
        hold(plant, kp=0.0, tau=tau_ff)
        run(plant, 3.0)

        penetration = plant.q - plant.limits.to_rad(OBJECT_MM)
        # Friction is a small correction; the spring dominates.
        assert penetration == pytest.approx(tau_ff / plant.config.contact_k, rel=0.1)

    def test_an_object_pushed_hard_does_not_report_a_wild_force(self, plant: Plant) -> None:
        """With kp=0 the torque is bounded by the feed-forward, whatever the depth."""
        plant.object_mm = OBJECT_MM
        plant.q = plant.limits.to_rad(OBJECT_MM) + 0.30  # deep inside
        hold(plant, kp=0.0, tau=torque_from_force(40.0))
        run(plant, 2.0)
        assert abs(force_from_torque(plant.tau)) <= constants.FORCE_MAX_N + 1.0

    def test_a_free_gripper_reports_no_force(self, plant: Plant) -> None:
        """Nothing between the jaws and nothing commanded: no torque at all."""
        plant.q = plant.limits.to_rad(60.0)
        hold(plant, kp=constants.KP_MOVE, kd=constants.KD_DEFAULT)
        run(plant, 1.0)
        assert abs(plant.tau) < 0.05


class TestHardStops:
    """The jaws may press into a stop; they may not be thrown through it."""

    def test_the_damper_opposes_motion_into_the_lower_stop(self, plant: Plant) -> None:
        """The regression: ``+ stop_c·dq`` here accelerated the jaws inward.

        Travelling into the open stop at speed with the motor idle, the stop has
        to take energy out.  With the sign inverted it added energy — a −22 Nm
        kick that drove the gripper from 117 mm back to 10 mm.
        """
        plant.q = plant.limits.rad_low + 0.005
        plant.dq = -2.0  # opening, i.e. toward rad_low
        plant.stream(plant.q, 0.0, 0.0, 0.0, 0.0)
        before = plant.dq
        plant.step(DT)
        assert plant.dq > before, "the stop must decelerate, not accelerate"

    def test_the_jaws_are_never_thrown_back_out_of_the_travel(self, plant: Plant) -> None:
        plant.q = plant.limits.rad_low + 0.005
        plant.dq = -2.0
        plant.stream(plant.limits.rad_low, 0.0, 0.0, 0.0, 0.0)
        worst = plant.q
        for _ in range(400):
            plant.step(DT)
            worst = min(worst, plant.q)
        assert worst >= plant.limits.rad_low - 0.05

    @pytest.mark.parametrize("direction", [-1.0, 1.0])
    def test_a_full_torque_push_penetrates_a_bounded_fraction(
        self, plant: Plant, direction: float
    ) -> None:
        """The stop stiffness sets how deep a hard push sinks — 10 Nm / 800."""
        stop = plant.limits.rad_low if direction < 0 else plant.limits.rad_high
        plant.q = stop
        hold(plant, kp=0.0, tau=direction * plant.config.tau_limit)
        run(plant, 2.0)

        excursion = abs(plant.q - stop)
        assert excursion < 0.03, "a stop is a stop, not a spring you can drive through"
        assert math.isfinite(plant.q) and math.isfinite(plant.dq)

    def test_pressing_into_a_stop_settles_rather_than_chattering(self, plant: Plant) -> None:
        plant.q = plant.limits.rad_low + 0.01
        hold(plant, kp=0.0, tau=-3.0)
        run(plant, 2.0)
        assert abs(plant.dq) < 0.05

    def test_a_full_speed_arrival_stays_inside_the_travel(self, plant: Plant) -> None:
        """No excursion, no NaN: the spring absorbs the arrival."""
        plant.q = plant.limits.to_rad(60.0)
        target = plant.limits.rad_low
        hold(plant, kp=constants.KP_MOVE, kd=constants.KD_DEFAULT, q=target)
        worst = plant.q
        for _ in range(600):
            plant.step(DT)
            worst = min(worst, plant.q)
        assert worst >= plant.limits.rad_low - 0.05
        assert math.isfinite(plant.q)


class TestTracking:
    """The coefficients have to produce a servo that visibly lags but tracks.

    Tracking lag is only meaningful against a *moving* reference: a fixed target
    at a hard stop measures stop penetration instead, and says nothing about the
    servo.  Both tests below therefore ramp the command at cruise speed, which is
    also the only condition in which the steady state exists at all — while the
    reference is still accelerating there is no constant lag to compare.
    """

    @staticmethod
    def cruise(plant: Plant, start_mm: float, speed_mm_s: float, seconds: float, kp: float):
        """Ramp the position reference at ``speed_mm_s``; return (cmd, measured).

        The measured position is sampled *before* the step that consumes the
        command, because the plant integrates within the tick.  Reading it after
        the step instead adds a full tick of travel to the apparent error — at
        50 mm/s that is 0.25 mm, fifteen times the real lag, and it would be
        blamed on the servo.
        """
        dq = -speed_mm_s / plant.limits.rad_to_mm
        cmd_mm = start_mm
        for _ in range(int(round(seconds / DT))):
            cmd_mm += speed_mm_s * DT
            measured_mm = plant.limits.to_mm(plant.q)
            plant.stream(plant.limits.to_rad(cmd_mm), kp, constants.KD_DEFAULT, dq)
            plant.step(DT)
        return cmd_mm, measured_mm

    def test_a_cruising_axis_lags_by_friction_over_kp(self, plant: Plant) -> None:
        """At equilibrium the position term alone carries the friction, since the
        velocity term vanishes once the jaws match the reference speed — so the
        steady-state lag is ``(coulomb + viscous·ω) / kp``, about 0.017 mm."""
        cmd, measured = self.cruise(
            plant, 10.0, constants.SPEED_DEFAULT_MM_S, 1.0, constants.KP_MOVE
        )
        assert cmd - measured == pytest.approx(0.017, abs=0.01)
        assert -plant.dq * plant.limits.rad_to_mm == pytest.approx(
            constants.SPEED_DEFAULT_MM_S, rel=0.05
        )

    def test_a_softer_gain_lags_more(self, plant: Plant) -> None:
        """Pins that kp is load-bearing rather than decorative: the lag is
        inversely proportional to it, so a fifth of the gain is five times the lag."""

        def lag(kp: float) -> float:
            p = Plant()
            cmd, measured = self.cruise(
                p, 10.0, constants.SPEED_DEFAULT_MM_S, 1.0, kp
            )
            return cmd - measured

        assert lag(constants.KP_MOVE) < lag(20.0) / 3.0

    def test_no_overshoot_on_a_step(self, plant: Plant) -> None:
        """kd has to be enough to stop the jaws without ringing past the target."""
        target = plant.limits.to_rad(40.0)
        hold(plant, kp=constants.KP_MOVE, kd=constants.KD_DEFAULT, q=target)
        run(plant, 2.0)
        assert plant.limits.to_mm(plant.q) == pytest.approx(40.0, abs=0.5)
        assert abs(plant.dq) < 0.05


class TestThermal:
    def test_sustained_effort_warms_the_drive(self, plant: Plant) -> None:
        p = plant
        p.q = p.limits.to_rad(60.0)
        hold(p, kp=0.0, tau=torque_from_force(40.0))
        run(p, 120.0)
        assert p.temp_mos > p.config.amb_temp + 3.0
        assert p.temp_coil > p.config.amb_temp + 6.0

    def test_it_cools_once_the_load_is_removed(self, plant: Plant) -> None:
        p = plant
        p.q = p.limits.to_rad(60.0)
        hold(p, kp=0.0, tau=torque_from_force(40.0))
        run(p, 120.0)
        hot = p.temp_mos
        hold(p, kp=constants.KP_MOVE, kd=constants.KD_DEFAULT, q=p.q)
        run(p, 300.0)
        assert p.temp_mos < hot

    def test_it_never_falls_below_ambient(self, plant: Plant) -> None:
        run(plant, 600.0)
        assert plant.temp_mos >= plant.config.amb_temp
        assert plant.temp_coil >= plant.config.amb_temp


class TestStatusFrames:
    def test_the_motor_emits_frames_on_its_own_schedule(self, plant: Plant) -> None:
        """500 Hz status, regardless of whether anything is transmitted."""
        plant.step(0.002)
        assert plant.poll() is True
        assert plant.poll() is False

    def test_a_long_stall_does_not_bank_a_burst(self, plant: Plant) -> None:
        """A backlog would arrive as one spike and read as a phantom burst."""
        run(plant, 60.0)
        delivered = 0
        while plant.poll():
            delivered += 1
        assert delivered <= 2

    def test_the_snapshot_is_internally_consistent(self, plant: Plant) -> None:
        plant.q = plant.limits.to_rad(37.5)
        plant.dq = 0.0
        tele = plant.snapshot()
        assert tele.position_mm == pytest.approx(37.5)
        assert tele.position_rad == pytest.approx(plant.q)
        assert tele.force_n == pytest.approx(force_from_torque(plant.tau))


class TestDegenerateInputs:
    def test_it_starts_closed_and_at_rest(self, plant: Plant) -> None:
        assert plant.q == pytest.approx(plant.config.closed_rad)
        assert plant.dq == 0.0
        assert plant.limits.to_mm(plant.q) == pytest.approx(0.0)

    def test_a_zero_or_negative_step_is_inert(self, plant: Plant) -> None:
        q, dq = plant.q, plant.dq
        plant.step(0.0)
        plant.step(-1.0)
        assert (plant.q, plant.dq) == (q, dq)

    def test_reset_returns_it_to_the_powered_off_pose(self, plant: Plant) -> None:
        plant.object_mm = OBJECT_MM
        hold(plant, kp=0.0, tau=torque_from_force(40.0))
        run(plant, 1.0)
        plant.reset()
        assert plant.q == pytest.approx(plant.config.closed_rad)
        assert plant.dq == 0.0
        assert plant.tau == 0.0
        assert plant.object_mm is None
        assert plant.temp_mos == plant.config.amb_temp

    def test_it_is_deterministic(self) -> None:
        """Bit-identical replay is what makes the simulator usable in tests."""
        def replay() -> tuple[float, float]:
            p = Plant()
            p.object_mm = OBJECT_MM
            p.stream(p.limits.to_rad(10.0), constants.KP_MOVE, constants.KD_DEFAULT)
            run(p, 1.0)
            return p.q, p.dq

        assert replay() == replay()

    def test_the_plant_does_not_sanitise_what_reaches_it(self, plant: Plant) -> None:
        """Stated so the boundary is explicit: this is a *simulator*, not a guard.

        It has no more obligation to reject a NaN than a real motor does — and a
        real motor would happily act on one.  The defence belongs upstream, in
        ``units`` (which clamps and sanitises) and in the FSM's ``_send``; this
        test exists so nobody mistakes the plant for a safety layer and removes
        those.
        """
        plant.stream(math.nan, constants.KP_MOVE, constants.KD_DEFAULT)
        run(plant, 0.1)
        assert not math.isfinite(plant.q), "if this ever fails, the guard moved"

    def test_the_default_config_is_a_usable_calibration(self) -> None:
        """The simulator's own travel is the ordering this bench has, with travel
        in it.

        Not the SDK's untouched defaults: those are a pair of angles recorded the
        other way round, which is a valid calibration but not this plant's, and a
        simulator whose limits are a different gripper's would exercise the
        console against numbers no bench produces.
        """
        cfg = PlantConfig()
        limits = Plant(cfg).limits
        assert limits.direction == -1.0
        assert limits.travel_rad > 0.0
        assert limits.to_mm(limits.open_rad) == pytest.approx(limits.stroke_mm)
        assert Plant(cfg).limits.stroke_mm == pytest.approx(
            constants.DEFAULT_TRAVEL_MM + constants.SPAN_INSET_MM, abs=0.5
        )
