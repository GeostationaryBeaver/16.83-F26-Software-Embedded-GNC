"""
Cold-gas N2 nozzle thrust model with upstream valve dynamics.

Geometry, gas properties and loss coefficients are taken from
"6.06N Nozzle Calculations" (CR=4, matched expansion at 200 psig, sea level).

Model chain:
    valve command u(t)  ->  valve position x(t)   (dead time + first-order lag)
                        ->  plenum pressure p0(t) (mass balance: valve in, throat out)
                        ->  thrust F(t)           (quasi-steady C-D nozzle at p0)

The plenum is what makes pulsed thrust interesting: it cannot charge or
discharge instantaneously, so at high pulse frequency the delivered impulse is
less than duty_cycle * steady_thrust.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from functools import lru_cache

import numpy as np
from scipy.optimize import brentq

PSI_TO_PA = 6894.757293168361
G0 = 9.80665  # standard gravity, conversion constant only


# ----------------------------------------------------------------------------
# Parameters
# ----------------------------------------------------------------------------

@dataclass(frozen=True)
class Nozzle:
    """Converging-diverging nozzle geometry and loss coefficients."""

    d_throat: float = 2.010e-3          # m
    d_exit: float = 3.115e-3            # m
    d_inlet: float = 4.020e-3           # m
    divergent_half_angle: float = 15.0  # deg
    cd: float = 0.98                    # throat discharge coefficient
    eta_friction: float = 0.99          # wall friction / heat-loss efficiency

    @property
    def a_throat(self) -> float:
        return 0.25 * math.pi * self.d_throat ** 2

    @property
    def a_exit(self) -> float:
        return 0.25 * math.pi * self.d_exit ** 2

    @property
    def a_inlet(self) -> float:
        return 0.25 * math.pi * self.d_inlet ** 2

    @property
    def area_ratio(self) -> float:
        """epsilon = Ae / At."""
        return self.a_exit / self.a_throat

    @property
    def contraction_ratio(self) -> float:
        return self.a_inlet / self.a_throat

    @property
    def lambda_divergence(self) -> float:
        """Conical-nozzle divergence efficiency."""
        return 0.5 * (1.0 + math.cos(math.radians(self.divergent_half_angle)))


@dataclass(frozen=True)
class Gas:
    """Ideal-gas working fluid."""

    gamma: float = 1.4
    r_specific: float = 296.8    # J/(kg*K), nitrogen
    t_stagnation: float = 293.0  # K

    @property
    def c_star(self) -> float:
        """Characteristic velocity, a property of the gas alone."""
        g = self.gamma
        return math.sqrt(self.r_specific * self.t_stagnation / g) * (
            (g + 1.0) / 2.0
        ) ** ((g + 1.0) / (2.0 * (g - 1.0)))

    @property
    def critical_pressure_ratio(self) -> float:
        """p_downstream / p_upstream below which an orifice chokes."""
        g = self.gamma
        return (2.0 / (g + 1.0)) ** (g / (g - 1.0))


@dataclass(frozen=True)
class Valve:
    """Solenoid valve upstream of the nozzle.

    Default flow area is the 4.020 mm feed bore. Its effective area
    (Cd * A = 8.9 mm^2) is only ~2.9x the throat's, so even fully open the
    valve costs about 2.8% of plenum pressure and the steady thrust settles
    near 5.87 N rather than the 6.06 N of the hand analysis -- that analysis
    assumes the full 200 psig reaches the converging section. Raise
    ``flow_area`` (or model a regulator that holds plenum pressure) to close
    that gap; see ``steady_thrust_reference``.
    """

    flow_area: float = 0.25 * math.pi * 4.020e-3 ** 2  # m^2, fully open
    cd: float = 0.70                                   # orifice discharge coeff
    dead_time_open: float = 2.0e-3                     # s, command -> motion
    dead_time_close: float = 1.5e-3                    # s
    tau_open: float = 1.5e-3                           # s, first-order lag
    tau_close: float = 1.0e-3                          # s


@dataclass(frozen=True)
class Plenum:
    """Volume between valve seat and nozzle throat.

    Default is ~50 mm of 4.020 mm bore tubing. Treated as isothermal at the
    supply temperature: the volume is small and refilled from a 293 K source on
    a ~1 ms timescale, so stagnation-temperature swing is neglected.
    """

    volume: float = 0.25 * math.pi * 4.020e-3 ** 2 * 50.0e-3  # m^3


@dataclass(frozen=True)
class Conditions:
    """Operating environment."""

    p_supply_gauge_psi: float = 200.0
    p_ambient: float = 101_325.0  # Pa, sea level standard

    @property
    def p_supply(self) -> float:
        """Absolute supply (regulator outlet) pressure, Pa."""
        return self.p_supply_gauge_psi * PSI_TO_PA + self.p_ambient


@dataclass(frozen=True)
class Model:
    """Full model assembly plus numerical and fidelity switches."""

    nozzle: Nozzle = field(default_factory=Nozzle)
    gas: Gas = field(default_factory=Gas)
    valve: Valve = field(default_factory=Valve)
    plenum: Plenum = field(default_factory=Plenum)
    conditions: Conditions = field(default_factory=Conditions)

    # Apply Cd to the momentum thrust term.
    #
    # The source analysis computes F_ideal = CF * p0 * At with CF = Ve/c*,
    # which omits Cd from the momentum term, while separately reporting mdot
    # WITH Cd. False reproduces that convention (6.06 N steady). True is the
    # self-consistent F = mdot * Ve (about 2% lower).
    apply_cd_to_momentum: bool = False

    # Truncate the over-expansion pressure deficit once the exit flow
    # separates (Summerfield: separation near pe < 0.4 * pa). Without this a
    # naive model predicts unphysically large negative thrust during decay.
    use_separation_model: bool = True
    separation_ratio: float = 0.4


# ----------------------------------------------------------------------------
# Steady nozzle aerodynamics
# ----------------------------------------------------------------------------

def _area_ratio_from_mach(mach: float, gamma: float) -> float:
    g = gamma
    return (1.0 / mach) * (
        (1.0 + 0.5 * (g - 1.0) * mach ** 2) / ((g + 1.0) / 2.0)
    ) ** ((g + 1.0) / (2.0 * (g - 1.0)))


@lru_cache(maxsize=256)
def _exit_mach_cached(area_ratio: float, gamma: float) -> float:
    return brentq(
        lambda m: _area_ratio_from_mach(m, gamma) - area_ratio, 1.0 + 1e-9, 50.0
    )


def exit_mach(model: Model) -> float:
    """Supersonic-branch exit Mach set purely by the area ratio.

    Cached: this sits inside the RK4 inner loop, and the root-find result
    depends only on (area ratio, gamma), both fixed for a given model.
    """
    return _exit_mach_cached(model.nozzle.area_ratio, model.gas.gamma)


@lru_cache(maxsize=256)
def _subsonic_exit_mach_cached(area_ratio: float, gamma: float) -> float:
    return brentq(
        lambda m: _area_ratio_from_mach(m, gamma) - area_ratio, 1e-9, 1.0
    )


def first_choke_pressure(model: Model) -> float:
    """Plenum pressure (absolute) at which the throat first reaches Mach 1.

    NOT the simple-orifice critical ratio: in a C-D nozzle the throat is
    upstream of a diverging section, so the throat goes sonic at a much lower
    plenum pressure than a plain orifice would. The condition is the subsonic
    root of the area-Mach relation at this area ratio, which for eps = 2.4
    puts first choke near p0 = 1.04 * pa rather than 1.89 * pa.
    """
    gas, noz = model.gas, model.nozzle
    me = _subsonic_exit_mach_cached(noz.area_ratio, gas.gamma)
    g = gas.gamma
    return model.conditions.p_ambient * (
        1.0 + 0.5 * (g - 1.0) * me ** 2
    ) ** (g / (g - 1.0))


@dataclass(frozen=True)
class SteadyPoint:
    """Quasi-steady nozzle solution at one plenum pressure."""

    p0: float
    mdot: float
    thrust: float
    momentum_thrust: float
    pressure_thrust: float
    exit_mach: float
    exit_pressure: float
    exit_velocity: float
    choked: bool
    separated: bool


def steady_state(model: Model, p0: float) -> SteadyPoint:
    """Thrust and mass flow for a plenum held at ``p0`` (absolute, Pa)."""
    gas, noz, cond = model.gas, model.nozzle, model.conditions
    g, pa = gas.gamma, cond.p_ambient

    p_choke = first_choke_pressure(model)

    if p0 <= pa * (1.0 + 1e-12):
        return SteadyPoint(p0, 0.0, 0.0, 0.0, 0.0, 0.0, pa, 0.0, False, False)

    if p0 >= p_choke:
        # Throat sonic: mass flow set by p0, At, c*. Exit state fixed by area ratio.
        choked = True
        mdot = noz.cd * p0 * noz.a_throat / gas.c_star
        me = exit_mach(model)
        t_exit = gas.t_stagnation / (1.0 + 0.5 * (g - 1.0) * me ** 2)
        p_exit = p0 * (1.0 + 0.5 * (g - 1.0) * me ** 2) ** (-g / (g - 1.0))
        v_exit = me * math.sqrt(g * gas.r_specific * t_exit)
        # Momentum term; mdot without Cd matches the source analysis convention.
        mdot_momentum = mdot if model.apply_cd_to_momentum else p0 * noz.a_throat / gas.c_star
    else:
        # Fully subsonic: the nozzle acts as a venturi. The THROAT is the
        # restriction that sets mass flow, not the exit; the exit then
        # discharges at ambient.
        choked = False
        me = math.sqrt(2.0 / (g - 1.0) * ((p0 / pa) ** ((g - 1.0) / g) - 1.0))
        t_exit = gas.t_stagnation / (1.0 + 0.5 * (g - 1.0) * me ** 2)
        p_exit = pa
        v_exit = me * math.sqrt(g * gas.r_specific * t_exit)
        mdot = noz.cd * pa / (gas.r_specific * t_exit) * v_exit * noz.a_exit
        mdot_momentum = mdot

    momentum = mdot_momentum * v_exit * noz.lambda_divergence * noz.eta_friction

    # Pressure term. A badly over-expanded fixed-geometry nozzle does not pull
    # the full (pe - pa) * Ae deficit: the flow separates from the wall partway
    # down the diverging section, and downstream of that the wall sees roughly
    # ambient. So the nozzle effectively truncates at the separation plane --
    # both the momentum and the pressure term are evaluated there.
    #
    # Holding the full exit area while clamping only the pressure would make
    # thrust fall as plenum pressure rises just above choke, which is wrong.
    separated = False
    p_eff, a_eff = p_exit, noz.a_exit
    if choked and model.use_separation_model and p_exit < model.separation_ratio * pa:
        p_sep = model.separation_ratio * pa
        if p0 > p_sep:
            separated = True
            p_eff = p_sep
            m_sep = math.sqrt(
                ((p0 / p_sep) ** ((g - 1.0) / g) - 1.0) * 2.0 / (g - 1.0)
            )
            a_eff = min(
                noz.a_exit, noz.a_throat * _area_ratio_from_mach(m_sep, g)
            )
            t_sep = gas.t_stagnation / (1.0 + 0.5 * (g - 1.0) * m_sep ** 2)
            v_sep = m_sep * math.sqrt(g * gas.r_specific * t_sep)
            momentum = mdot_momentum * v_sep * noz.lambda_divergence * noz.eta_friction

    pressure = (p_eff - pa) * a_eff

    thrust = max(0.0, momentum + pressure)
    return SteadyPoint(
        p0, mdot, thrust, momentum, pressure, me, p_exit, v_exit, choked, separated
    )


def steady_thrust_reference(model: Model | None = None) -> SteadyPoint:
    """Thrust with the full supply pressure reaching the nozzle.

    This is the hand-analysis operating point (no valve pressure drop), useful
    as the ceiling the transient model approaches.
    """
    model = model or Model()
    return steady_state(model, model.conditions.p_supply)


# ----------------------------------------------------------------------------
# Valve command schedule
# ----------------------------------------------------------------------------

@dataclass(frozen=True)
class Phase:
    """One segment of the valve command schedule.

    mode: "closed" | "open" | "pulse"
    """

    mode: str
    duration: float
    frequency: float = 0.0   # Hz, pulse mode only
    duty_cycle: float = 0.0  # 0..1, pulse mode only
    label: str = ""

    def command(self, t_local: float) -> float:
        if self.mode == "closed":
            return 0.0
        if self.mode == "open":
            return 1.0
        if self.mode == "pulse":
            if self.frequency <= 0.0 or self.duty_cycle <= 0.0:
                return 0.0
            if self.duty_cycle >= 1.0:
                return 1.0
            period = 1.0 / self.frequency
            return 1.0 if (t_local % period) < self.duty_cycle * period else 0.0
        raise ValueError("unknown phase mode: " + repr(self.mode))


class Schedule:
    """Ordered valve command phases, evaluable at any time (including t<0)."""

    def __init__(self, phases: list[Phase]):
        if not phases:
            raise ValueError("schedule needs at least one phase")
        self.phases = list(phases)
        self.edges = np.cumsum([0.0] + [p.duration for p in self.phases])

    @property
    def duration(self) -> float:
        return float(self.edges[-1])

    def command(self, t: float) -> float:
        """Commanded valve state at time ``t``. Closed outside the schedule."""
        if t < 0.0 or t >= self.duration:
            return 0.0
        idx = int(np.searchsorted(self.edges, t, side="right") - 1)
        idx = min(max(idx, 0), len(self.phases) - 1)
        return self.phases[idx].command(t - self.edges[idx])

    def phase_spans(self) -> list[tuple[Phase, float, float]]:
        return [
            (p, float(self.edges[i]), float(self.edges[i + 1]))
            for i, p in enumerate(self.phases)
        ]


def default_schedule() -> Schedule:
    """Closed -> 20 Hz / 50% pulse -> fully open -> closed, 5 s each."""
    return Schedule([
        Phase("closed", 5.0, label="Closed"),
        Phase("pulse", 5.0, frequency=20.0, duty_cycle=0.50, label="Pulsed"),
        Phase("open", 5.0, label="Fully open"),
        Phase("closed", 5.0, label="Closed"),
    ])


# ----------------------------------------------------------------------------
# Transient simulation
# ----------------------------------------------------------------------------

def _valve_mass_flow(model: Model, x: float, p0: float) -> float:
    """Mass flow through the valve at opening fraction ``x``."""
    if x <= 0.0:
        return 0.0
    gas, cond, valve = model.gas, model.conditions, model.valve
    g, p_sup = gas.gamma, cond.p_supply
    if p0 >= p_sup:
        return 0.0

    area = valve.cd * valve.flow_area * x
    ratio = p0 / p_sup
    if ratio <= gas.critical_pressure_ratio:
        return area * p_sup / gas.c_star
    return (
        area
        * p_sup
        * math.sqrt(
            2.0 * g / ((g - 1.0) * gas.r_specific * gas.t_stagnation)
            * (ratio ** (2.0 / g) - ratio ** ((g + 1.0) / g))
        )
    )


def _throat_mass_flow(model: Model, p0: float) -> float:
    """Mass flow out through the nozzle throat.

    A lightweight path for the RK4 inner loop: same physics as
    ``steady_state(...).mdot`` without building the full result record.
    """
    gas, noz, pa = model.gas, model.nozzle, model.conditions.p_ambient
    if p0 <= pa * (1.0 + 1e-12):
        return 0.0
    if p0 >= first_choke_pressure(model):
        return noz.cd * p0 * noz.a_throat / gas.c_star
    g = gas.gamma
    me = math.sqrt(2.0 / (g - 1.0) * ((p0 / pa) ** ((g - 1.0) / g) - 1.0))
    t_exit = gas.t_stagnation / (1.0 + 0.5 * (g - 1.0) * me ** 2)
    v_exit = me * math.sqrt(g * gas.r_specific * t_exit)
    return noz.cd * pa / (gas.r_specific * t_exit) * v_exit * noz.a_exit


def _derivatives(model: Model, t: float, x: float, p0: float, schedule: Schedule):
    """[dx/dt, dp0/dt] with the valve's dead time applied to the command."""
    valve, gas, plenum = model.valve, model.gas, model.plenum

    # Dead time differs opening vs closing; pick by what the valve is doing.
    cmd_open = schedule.command(t - valve.dead_time_open)
    cmd_close = schedule.command(t - valve.dead_time_close)
    cmd = cmd_open if cmd_open >= x else cmd_close

    tau = valve.tau_open if cmd >= x else valve.tau_close
    dx = (cmd - x) / tau

    mdot_in = _valve_mass_flow(model, x, p0)
    mdot_out = _throat_mass_flow(model, p0)
    dp0 = gas.r_specific * gas.t_stagnation / plenum.volume * (mdot_in - mdot_out)
    return dx, dp0


@dataclass
class Result:
    """Simulation output plus per-phase summary statistics."""

    time: np.ndarray
    command: np.ndarray
    valve_position: np.ndarray
    plenum_pressure: np.ndarray
    thrust: np.ndarray
    mass_flow: np.ndarray
    model: Model
    schedule: Schedule

    @property
    def total_impulse(self) -> float:
        return float(np.trapezoid(self.thrust, self.time))

    @property
    def propellant_used(self) -> float:
        return float(np.trapezoid(self.mass_flow, self.time))

    def phase_stats(self) -> list[dict]:
        out = []
        last = len(self.schedule.phases) - 1
        for i, (phase, t0, t1) in enumerate(self.schedule.phase_spans()):
            # Half-open window [t0, t1) so a sample on a boundary belongs to
            # one phase only; the final phase keeps its closing endpoint.
            m = (self.time >= t0) & (self.time < t1)
            if i == last:
                m |= self.time == t1
            t, f, md = self.time[m], self.thrust[m], self.mass_flow[m]
            impulse = float(np.trapezoid(f, t)) if t.size > 1 else 0.0
            mass = float(np.trapezoid(md, t)) if t.size > 1 else 0.0
            out.append({
                "phase": phase,
                "t_start": t0,
                "t_end": t1,
                "mean_thrust": impulse / (t1 - t0) if t1 > t0 else 0.0,
                "peak_thrust": float(f.max()) if f.size else 0.0,
                "impulse": impulse,
                "propellant": mass,
                "isp": impulse / (mass * G0) if mass > 1e-12 else float("nan"),
            })
        return out


def simulate(
    model: Model | None = None,
    schedule: Schedule | None = None,
    dt: float = 50e-6,
    sample_dt: float | None = None,
) -> Result:
    """Integrate the valve + plenum + nozzle model with fixed-step RK4.

    dt defaults to 50 us: the plenum time constant is ~1 ms and the valve lag
    ~1.5 ms, so this resolves both with wide margin. The command signal is
    discontinuous, which is why a fixed step beats an adaptive solver here.
    """
    model = model or Model()
    schedule = schedule or default_schedule()

    n = int(round(schedule.duration / dt))
    stride = max(1, int(round((sample_dt or dt) / dt)))

    # Start settled: valve closed, plenum at ambient.
    x, p0 = 0.0, model.conditions.p_ambient

    times, xs, ps = [], [], []
    for i in range(n + 1):
        t = i * dt
        if i % stride == 0:
            times.append(t)
            xs.append(x)
            ps.append(p0)
        if i == n:
            break

        k1x, k1p = _derivatives(model, t, x, p0, schedule)
        k2x, k2p = _derivatives(
            model, t + 0.5 * dt, x + 0.5 * dt * k1x, p0 + 0.5 * dt * k1p, schedule
        )
        k3x, k3p = _derivatives(
            model, t + 0.5 * dt, x + 0.5 * dt * k2x, p0 + 0.5 * dt * k2p, schedule
        )
        k4x, k4p = _derivatives(
            model, t + dt, x + dt * k3x, p0 + dt * k3p, schedule
        )

        x += dt / 6.0 * (k1x + 2.0 * k2x + 2.0 * k3x + k4x)
        p0 += dt / 6.0 * (k1p + 2.0 * k2p + 2.0 * k3p + k4p)
        x = min(1.0, max(0.0, x))
        p0 = min(model.conditions.p_supply, max(model.conditions.p_ambient, p0))

    time = np.asarray(times)
    valve_position = np.asarray(xs)
    plenum_pressure = np.asarray(ps)
    command = np.asarray([schedule.command(t) for t in time])

    pts = [steady_state(model, p) for p in plenum_pressure]
    thrust = np.asarray([pt.thrust for pt in pts])
    mass_flow = np.asarray([pt.mdot for pt in pts])

    return Result(
        time=time,
        command=command,
        valve_position=valve_position,
        plenum_pressure=plenum_pressure,
        thrust=thrust,
        mass_flow=mass_flow,
        model=model,
        schedule=schedule,
    )
