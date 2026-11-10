#!/usr/bin/env python3
"""The valve between a control law's thrust demand and the force it gets.

A real thruster is not commanded in Newtons. You command a VALVE, and the
relation between valve opening and thrust is nonlinear, hardware-specific,
and drifts as the tank empties. This module is that valve - the plant a
controller has to work through without being told what it looks like.

    from valve import ValveActuator
    valve = ValveActuator(model, body_id, max_thrust_n=60.0)
    ...
    valve.apply(data, {"main": (0.58, direction)})   # 0..1 in, Newtons out
    valve.update(data, dt)                           # blowdown, once a tick

TO CHANGE THE VALVE CURVE, edit curve() below. It is the only place the
valve's shape lives; everything else here is bookkeeping and the feed model.

*** WHY A CONTROLLER CAN TRACK THROUGH THIS WITHOUT KNOWING IT. ***  Close
an integral loop on measured thrust:

    e = F_cmd - F_meas ,     u += K * e * dt

Linearised about g(u*) = F_cmd with gamma = g'(u*), that is a first-order
system with eigenvalue -K*gamma, so

    Y/R = K*gamma / (s + K*gamma)      DC gain EXACTLY 1 for any gamma > 0
    E/R = s / (s + K*gamma)            a zero at DC

The accuracy does not depend on the curve. Only the SPEED does. That is the
whole reason this works: the integrator's state converges to g^-1(F_cmd)
without anyone ever computing an inverse, and it re-converges for free when
the curve drifts. Measured on the ball curve: steady error 0.00000 N at
every gain from K=0.004 to K=0.5, and 0.00000 N against unmodelled constant
offsets of -3 N and +5 N (the valve just parks somewhere else).

Discrete, the stability bound is 0 < K*gamma*T < 2. At T = 2 ms and the
measured worst-case gamma = 391 N/unit that is K < 2.56, so the sample rate
is not the binding constraint. The ACTUATOR LAG is: with a first-order lag
tau the loop is second order, zeta = 1/(2*sqrt(K*gamma*tau)), so damping
FALLS as K rises.

*** WHERE THAT MODEL STOPS BEING TRUE. ***  Four places, all of them real
on this hardware, and each one is a test in scenario.py's selftest:

    gamma = 0 (dead zone)    eigenvalue 0, no dynamics at all. The loop is
                             not converging, it is drifting open-loop at
                             K*F_cmd until thrust appears. On the ball
                             curve that transit DOMINATES the settle time.
    gamma varies 32x         bandwidth is K*gamma, so one fixed K gives a
                             32x spread in response speed across the range.
    u clamped to [0, 1]      the state equation does not hold outside it;
                             integrating while pinned is windup.
    grounded                 F_meas saturates at vehicle WEIGHT, because
                             the floor supplies the balance. The loop is
                             then closing on a false measurement, which is
                             a broken observation, not a plant property.
"""
import numpy as np

# --- VALVE CURVE ----------------------------------------------------------
# Which shape sits between command and thrust. "ball" is the real hardware,
# from sprint 2/thrust_sim; the rest exist so the controller can be tested
# against a family and the result attributed to the curve rather than luck.
VALVE_CURVE = "ball"

# --- FEED PRESSURE, i.e. the time dependence ------------------------------
# The curve's SHAPE is valve geometry; its AMPLITUDE is feed pressure. They
# are kept apart so each can be studied alone - and because, choked, thrust
# is roughly proportional to chamber pressure, so a falling feed scales the
# whole curve without reshaping it.
#
#   constant   feed never changes. Isolates the curve nonlinearity.
#   regulated  ideal regulator with dropout: flat until tank < setpoint,
#              then tracks the tank down. THE REAL BEHAVIOUR, so the default.
#   from_tank  feed fraction follows PropellantTank.fraction_remaining, so
#              blowdown and vehicle mass loss cannot disagree.
VALVE_FEED_MODE = "regulated"
REG_SETPOINT_PSIG = 200.0   # psig, regulator setpoint
TANK_INITIAL_PSIG = 1000.0  # psig, bottle pressure at t=0
TANK_VOLUME_L = 2.0         # L, tank volume ON THE REAL 5.7 N THRUSTER.
                            #   Measured there: a representative 4.33 g/s
                            #   hold drains 27.3 psi/s, so the regulator
                            #   holds ~29 s and a standard ~18 s profile runs
                            #   entirely inside the flat window. That is the
                            #   quantitative version of "steady long enough
                            #   to be useful".
                            #
                            #   *** AND IT MUST SCALE WITH THE ENGINE. ***
                            #   The curve is scaled to each scene's ctrlrange
                            #   so the controller meets the same shape on a
                            #   60 N craft as on a 5.7 N one - but gas is not
                            #   free. A 60 N engine at Isp 57.8 s burns
                            #   105.9 g/s against the real thruster's
                            #   10.1 g/s, so the SAME 2 L bottle would empty
                            #   10.4x faster and the window would collapse
                            #   from 29 s to 1.2 s. Holding the bottle fixed
                            #   while scaling the thrust is not a
                            #   conservative choice, it is an incoherent
                            #   one: it models a vehicle whose tank could
                            #   never have flown the profile.
                            #
                            #   So the tank is sized FROM the engine, below,
                            #   unless a volume is passed explicitly.
VALVE_HOLD_POSITION = 0.425 # fraction of full-open FLOW at a representative
                            #   hover hold point. The regulated window is
                            #   quoted two ways because they differ by 2.4x:
                            #   12.5 s wide open, 29.5 s here. A hover
                            #   profile mostly sits near the hold point, so
                            #   that is the figure that matters for "long
                            #   enough to be useful"; full open is the
                            #   worst case. Quoting only one of them is how
                            #   this was first got wrong.
VALVE_REF_THRUST_N = 5.738  # N, full-open thrust of the real thruster the
                            #   2 L figure was measured on. The scaling
                            #   reference, not a limit.
GAS_R_J_KGK = 296.807       # J/(kg K), nitrogen
GAS_TEMP_K = 293.0          # K, isothermal. Real blowdown cools the tank and
                            #   would decay thrust faster; that would attach
                            #   here as a temperature term.
PSI_TO_PA = 6894.757
AMBIENT_PA = 101325.0

# The ball valve's dead zone, as a fraction of full travel. Below this the
# two bores do not overlap at all and the valve passes NOTHING. Derived:
# area is zero until cos(theta) < 2*bore_radius/ball_radius = 0.804, i.e.
# theta < 36.49 deg, and thrust stays zero out to ~44.4 deg because the
# nozzle is unchoked below that.
BALL_DEAD_FRACTION = 36.49 / 90.0

_DEADZONE_U0 = 0.30         # where the "deadzone" test curve starts passing


def _ball_table():
    """The real ball-valve characteristic, as (u, thrust_fraction) arrays.

    Imported from sprint 2/thrust_sim/thrust_sim.py so there is ONE source
    of truth for the hardware's shape. That module imports pyplot at module
    scope, so MPLBACKEND is forced to Agg first - without it an import here
    can try to pick a GUI backend in a headless run.

    Falls back to a built-in 19-point sample of the same curve if the import
    fails, so this module never hard-depends on a sibling sprint directory.
    """
    import os
    import pathlib
    import sys

    os.environ.setdefault("MPLBACKEND", "Agg")
    here = pathlib.Path(__file__).resolve()
    sibling = here.parent.parent.parent / "thrust_sim"
    try:
        if str(sibling) not in sys.path:
            sys.path.insert(0, str(sibling))
        import thrust_sim as ts
        ang, thr, _ = ts.steady_thrust_vs_angle(n_points=181)
        u = np.asarray(ang, dtype=float) / 90.0
        f = np.asarray(thr, dtype=float)
        if f[-1] <= 0:
            raise ValueError("thrust_sim returned a zero full-open thrust")
        return u, f / f[-1]
    except Exception:
        # Sampled from the same function, normalized. Keeps the dead zone,
        # the knee and the saturating top, which is what matters.
        u = np.linspace(0.0, 1.0, 19)
        f = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0363, 0.2119, 0.4356, 0.6686, 0.8215,
                      0.9120, 0.9594, 0.9834, 0.9951, 1.0])
        return u, f


_BALL_U, _BALL_F = None, None


def ball_fraction(position):
    """Thrust fraction (0..1) for the real ball valve at this opening."""
    global _BALL_U, _BALL_F
    if _BALL_U is None:
        _BALL_U, _BALL_F = _ball_table()
    return float(np.interp(float(np.clip(position, 0.0, 1.0)),
                           _BALL_U, _BALL_F))


def curve(position, max_thrust_n, name=None):
    """*** THIS IS THE FUNCTION TO REPLACE. ***  0..1 -> Newtons.

    Every shape is normalized so curve(1.0) == max_thrust_n, which is what
    lets the SAME curve be flown on the 60 N and the 10 N scene: the
    controller then faces identical difficulty on both, rather than the
    difficulty being an artefact of the engine size.

    The family:

        ball       the real hardware. 36.5% dead zone, steep knee,
                   saturating top; slope spans 32x across the live band.
        linear     the easy case, and the baseline everything is read
                   against.
        quadratic  gain RISES with opening.
        sqrt       gain FALLS with opening - the opposite skew, so a gain
                   tuned on `quadratic` is wrong here in the other
                   direction.
        deadzone   zero below _DEADZONE_U0, linear above. Isolates the
                   dead zone from the ball curve's other features.
        reversed   NON-MONOTONIC on purpose. g' < 0 over part of the range,
                   which flips the feedback sign. The integral tracker
                   MUST fail here, and that is the point: monotonicity is
                   the one condition the architecture actually requires,
                   so the test suite has to contain a case that violates
                   it. A suite where everything passes has not found the
                   boundary.
    """
    name = VALVE_CURVE if name is None else name
    u = float(np.clip(position, 0.0, 1.0))
    if name == "ball":
        frac = ball_fraction(u)
    elif name == "linear":
        frac = u
    elif name == "quadratic":
        frac = u * u
    elif name == "sqrt":
        frac = float(np.sqrt(u))
    elif name == "deadzone":
        frac = 0.0 if u <= _DEADZONE_U0 else (u - _DEADZONE_U0) / (1.0 - _DEADZONE_U0)
    elif name == "reversed":
        # A valve that CHOKES as it opens past halfway: rises to full
        # thrust at u=0.5, then falls away to 40% at u=1.0. Physically this
        # is a flow that stalls or a seat that re-seals; here it exists only
        # to break monotonicity.
        #
        # The falling branch has to descend BELOW any demand the loop will
        # be asked for, or the test is vacuous. Two failed attempts:
        # peaking at 0.6 and bottoming at 0.4 of full left a 32% hover
        # demand satisfied on the RISING branch at u=0.324, so the loop
        # converged happily and the check proved nothing; bottoming at
        # 0.4 of full (24 N on a 60 N engine) still sat above a 19.44 N
        # demand. Now it falls all the way to zero at full open, so EVERY
        # reachable demand has a second equilibrium above the peak where
        # g' < 0 flips the feedback sign.
        frac = (u / 0.5) if u <= 0.5 else (1.0 - (u - 0.5) / 0.5)
    else:
        raise ValueError(f"unknown valve curve {name!r}; "
                         f"choose from {', '.join(CURVE_NAMES)}")
    return float(max_thrust_n) * float(np.clip(frac, 0.0, 1.0))


CURVE_NAMES = ("ball", "linear", "quadratic", "sqrt", "deadzone", "reversed")
MONOTONIC_CURVES = ("ball", "linear", "quadratic", "sqrt", "deadzone")


def slope(position, max_thrust_n, name=None, h=1e-3):
    """dF/du at this opening, in N per unit of travel - the `gamma` above.

    Not used by the plant: it is what the ANALYSIS needs, and what a gain
    scheduler would need if one is added later. Exposed here rather than
    recomputed in the selftest so there is one definition.
    """
    u = float(np.clip(position, 0.0, 1.0))
    lo, hi = max(0.0, u - h), min(1.0, u + h)
    if hi <= lo:
        return 0.0
    return (curve(hi, max_thrust_n, name) - curve(lo, max_thrust_n, name)) / (hi - lo)


class ValveActuator:
    """Converts a 0..1 valve command into Newtons through an unknown curve.

    Sits beside the sim the way PropellantTank does: constructed before the
    loop, consulted once per tick, and the control law is never told what it
    is working through.

    Holds two states. `position` is fast and is whatever the controller last
    commanded (optionally through a first-order actuator lag). `p_tank_pa`
    is slow - it is the blowdown, and NOTHING in the control loop measures
    or models it. The integral tracker absorbs it anyway, which is the
    strongest argument for closing the loop at all: measured over a 60 s
    blowdown, a frozen valve loses 47% of its thrust where the closed loop
    loses 0.6%.
    """

    def __init__(self, model, body_id, max_thrust_n,
                 curve_name=None, feed_mode=None,
                 reg_setpoint_psig=REG_SETPOINT_PSIG,
                 tank_initial_psig=TANK_INITIAL_PSIG,
                 tank_volume_l=None,
                 tau_s=0.0, isp_s=None):
        self.curve_name = VALVE_CURVE if curve_name is None else str(curve_name)
        if self.curve_name not in CURVE_NAMES:
            raise ValueError(f"unknown valve curve {self.curve_name!r}; "
                             f"choose from {', '.join(CURVE_NAMES)}")
        self.feed_mode = VALVE_FEED_MODE if feed_mode is None else str(feed_mode)
        if self.feed_mode not in ("constant", "regulated", "from_tank"):
            raise ValueError(f"unknown feed mode {self.feed_mode!r}")
        if max_thrust_n <= 0:
            raise ValueError(f"max_thrust_n must be positive, got {max_thrust_n}")

        # Size the bottle from the engine, so the regulated window is the
        # same number of SECONDS whatever scene is flown. See TANK_VOLUME_L:
        # keeping 2 L on a 60 N engine would give 1.2 s instead of 29 s and
        # model a vehicle that could not have flown the profile at all.
        self.tank_scaled = tank_volume_l is None
        if self.tank_scaled:
            tank_volume_l = TANK_VOLUME_L * float(max_thrust_n) / VALVE_REF_THRUST_N
        if tank_volume_l <= 0:
            raise ValueError(f"tank_volume_l must be positive, got {tank_volume_l}")


        self._model = model
        self.body_id = body_id
        self.max_thrust_n = float(max_thrust_n)
        self.tau_s = max(0.0, float(tau_s))
        self.isp_s = isp_s

        self.p_set_pa = float(reg_setpoint_psig) * PSI_TO_PA + AMBIENT_PA
        self.p_tank_initial_pa = float(tank_initial_psig) * PSI_TO_PA + AMBIENT_PA
        self.tank_volume_m3 = float(tank_volume_l) * 1e-3
        self.p_tank_pa = self.p_tank_initial_pa

        self.position = 0.0        # actual opening, after any lag
        self.commanded = 0.0       # what the controller last asked for
        self.last_thrust_n = 0.0
        self.travel_total = 0.0    # integrated |du|, a wear/effort proxy
        self.starved_at_s = None   # when the valve pinned open and still
                                   #   could not meet demand. Propellant
                                   #   exhaustion, NOT a control failure, so
                                   #   it is reported rather than tuned away.
        self.pinned_s = 0.0
        self._tank_frac = 1.0      # set by from_tank mode

    # ---- the feed ------------------------------------------------------
    def feed_fraction(self):
        """How much of the nominal curve amplitude is available right now.

        1.0 while the regulator holds, then p_tank/p_set once it drops out.
        Choked, thrust is ~proportional to chamber pressure, so this scales
        the curve without reshaping it - verified against thrust_sim.
        """
        if self.feed_mode == "constant":
            return 1.0
        if self.feed_mode == "from_tank":
            return float(np.clip(self._tank_frac, 0.0, 1.0))
        return float(np.clip(min(self.p_set_pa, self.p_tank_pa) / self.p_set_pa,
                             0.0, 1.0))

    @property
    def p_tank_psig(self):
        return (self.p_tank_pa - AMBIENT_PA) / PSI_TO_PA

    def regulated_window_s(self, position=1.0):
        """How long the feed stays flat at this opening, in seconds.

        Analytic rather than simulated: while regulated, dP/dt = (RT/V)*mdot
        is constant for a fixed opening, so the window is the pressure above
        the setpoint divided by that rate. Reported by describe() and
        asserted by the selftest, because "steady long enough to be useful"
        is a claim that should fail loudly if a parameter changes.
        """
        thrust = curve(position, self.max_thrust_n, self.curve_name)
        if thrust <= 0.0 or not self.isp_s:
            return float("inf")
        mdot = thrust / (self.isp_s * 9.80665)
        rate = (GAS_R_J_KGK * GAS_TEMP_K / self.tank_volume_m3) * mdot
        if rate <= 0.0:
            return float("inf")
        return max(0.0, (self.p_tank_initial_pa - self.p_set_pa) / rate)

    @property
    def regulated(self):
        """True while the feed is still flat - the useful window."""
        return self.feed_fraction() >= 1.0 - 1e-9

    def set_tank_fraction(self, fraction):
        """For feed_mode='from_tank': couple to the sim's own tank.

        Keeps one source of truth for how much gas is left, instead of two
        tank models that can disagree.
        """
        self._tank_frac = float(np.clip(fraction, 0.0, 1.0))

    # ---- the curve -----------------------------------------------------
    def thrust_for(self, position):
        """Newtons this valve would produce at this opening, right now."""
        return curve(position, self.max_thrust_n, self.curve_name) * self.feed_fraction()

    def slope_at(self, position):
        """dF/du including the current feed scaling."""
        return slope(position, self.max_thrust_n, self.curve_name) * self.feed_fraction()

    # ---- per-tick ------------------------------------------------------
    def apply(self, data, commands, thrusters):
        """Write Newtons into data.ctrl from 0..1 valve commands.

        `commands` is in the shape a control law returns - either a bare
        number or a (magnitude, direction) pair - except the magnitude is a
        valve opening, not a force. Returns the applied Newtons per thruster
        so a caller can log demand against delivery.

        Only the thruster named in VALVE_THRUSTER goes through the valve.
        Anything else (the RCS nozzles) is passed through as Newtons, since
        those are not throttled by this valve.
        """
        out = {}
        converted = {}
        for name, cmd in commands.items():
            direction = None
            value = cmd
            if isinstance(cmd, (tuple, list)) and len(cmd) == 2:
                value, direction = cmd
            if name == VALVE_THRUSTER:
                self.commanded = float(np.clip(float(value), 0.0, 1.0))
                newtons = self.thrust_for(self.position)
                out[name] = newtons
                converted[name] = (newtons, direction) if direction is not None else newtons
            else:
                converted[name] = cmd
        applied = thrusters.apply(data, converted)
        self.last_thrust_n = float(out.get(VALVE_THRUSTER, 0.0))
        return applied

    def update(self, data, dt):
        """Advance the actuator lag and the blowdown. Once per physics tick.

        Called AFTER the control law has written its command, like
        PropellantTank.enforce_cutoff - so the law goes on asking and simply
        gets whatever the valve and the tank can actually deliver.
        """
        dt = float(dt)
        if dt <= 0.0:
            return self.position

        # Actuator lag, first order. tau=0 means the valve tracks instantly,
        # which is the right default here: the real 1 ms lag is below the
        # 2 ms physics step, so modelling it would add nothing.
        prev = self.position
        if self.tau_s > 0.0:
            self.position += (self.commanded - self.position) * min(dt / self.tau_s, 1.0)
        else:
            self.position = self.commanded
        self.position = float(np.clip(self.position, 0.0, 1.0))
        self.travel_total += abs(self.position - prev)

        if self.position >= 1.0 - 1e-9:
            self.pinned_s += dt

        # Blowdown. mdot from the thrust actually produced, via Isp, so the
        # gas spent matches the impulse delivered.
        if self.feed_mode == "regulated":
            thrust = self.thrust_for(self.position)
            if thrust > 0.0 and self.isp_s:
                mdot = thrust / (self.isp_s * 9.80665)
                self.p_tank_pa -= dt * (GAS_R_J_KGK * GAS_TEMP_K
                                        / self.tank_volume_m3) * mdot
                self.p_tank_pa = max(AMBIENT_PA, self.p_tank_pa)
        return self.position

    def note_starved(self, t, demand_n, tol_n=0.5):
        """Record the moment the valve ran out of authority.

        Pinned wide open and still short of demand means the engine cannot
        make the commanded thrust at any valve position. That is propellant
        exhaustion; the tracker is behaving correctly by sitting saturated,
        and the run should SAY so rather than leave a 15 N tracking error
        looking like a tuning problem.
        """
        if (self.starved_at_s is None
                and self.position >= 1.0 - 1e-6
                and demand_n - self.thrust_for(1.0) > tol_n):
            self.starved_at_s = float(t)
        return self.starved_at_s

    def reset(self):
        """Back to a full tank and a shut valve, for a sim reset."""
        self.p_tank_pa = self.p_tank_initial_pa
        self.position = 0.0
        self.commanded = 0.0
        self.last_thrust_n = 0.0
        self.travel_total = 0.0
        self.pinned_s = 0.0
        self.starved_at_s = None
        self._tank_frac = 1.0

    def describe(self):
        """Two lines for the drivers to print before a flight."""
        dead = next((u for u in np.linspace(0, 1, 201)
                     if curve(u, self.max_thrust_n, self.curve_name) > 1e-9), 1.0)
        g = [slope(u, self.max_thrust_n, self.curve_name)
             for u in np.linspace(0, 1, 201)]
        live = [x for x in g if x > 1e-6]
        spread = (max(live) / min(live)) if live else float("nan")
        if self.feed_mode == "regulated":
            feed = (f"regulated {REG_SETPOINT_PSIG:g} psig, tank "
                    f"{self.p_tank_psig:.0f} psig / "
                    f"{self.tank_volume_m3 * 1e3:.2f} L"
                    + ("  (sized from the engine)" if self.tank_scaled else "")
                    + f"   flat ~{self.regulated_window_s():.0f} s at full open")
        else:
            feed = self.feed_mode
        return (f"valve       {self.curve_name!r}, 0..1 -> 0..{self.max_thrust_n:.1f} N"
                f"   dead zone to u={dead:.3f}   slope spread {spread:.0f}x\n"
                f"feed        {feed}")


# Which thruster the valve throttles. The RCS nozzles are not fed through
# it, so they stay commanded in Newtons.
VALVE_THRUSTER = "main"


def run_selftest() -> int:
    """Checks on the plant alone, with no sim.

        uv run python scripts/valve.py --selftest
    """
    bar = "=" * 74
    print(bar); print("VALVE SELF-TEST"); print(bar)
    passed, failed = 0, []

    def truth(label, cond, detail=""):
        nonlocal passed
        print(f"  [{'ok  ' if cond else 'FAIL'}] {label:<48} {detail}")
        if cond:
            passed += 1
        else:
            failed.append(label)

    print("\n[1] the curve family")
    for name in MONOTONIC_CURVES:
        truth(f"{name}: full open gives max thrust",
              abs(curve(1.0, 60.0, name) - 60.0) < 1e-9,
              f"{curve(1.0, 60.0, name):.4f} N")
    # reversed deliberately does NOT: it peaks mid-travel and falls away,
    # so full open is below full thrust. That is the violation it exists for.
    truth("reversed peaks mid-travel and falls to zero at full open",
          curve(0.5, 60.0, "reversed") > 59.9
          and curve(1.0, 60.0, "reversed") < 0.1,
          f"peak {curve(0.5, 60.0, 'reversed'):.1f} N vs "
          f"{curve(1.0, 60.0, 'reversed'):.1f} N open")
    for name in CURVE_NAMES:
        truth(f"{name}: shut gives zero", curve(0.0, 60.0, name) == 0.0)
    truth("an unknown curve name raises",
          _raises(lambda: curve(0.5, 60.0, "nope")))

    print("\n[2] the ball valve's dead zone is real")
    truth("ball passes nothing at u=0.30",
          curve(0.30, 60.0, "ball") == 0.0)
    truth("ball passes nothing at u=0.40",
          curve(0.40, 60.0, "ball") == 0.0)
    truth("ball IS live by u=0.60",
          curve(0.60, 60.0, "ball") > 1.0,
          f"{curve(0.60, 60.0, 'ball'):.3f} N")
    truth("linear has no dead zone",
          curve(0.05, 60.0, "linear") > 0.0)

    print("\n[3] slope: zero in the dead zone, and a wide spread")
    gs = [slope(u, 60.0, "ball") for u in np.linspace(0, 1, 201)]
    live = [x for x in gs if x > 1e-6]
    truth("slope is zero inside the dead zone",
          max(slope(u, 60.0, "ball") for u in (0.0, 0.1, 0.2, 0.3)) < 1e-6)
    truth("slope spread over the live band is > 10x",
          max(live) / min(live) > 10.0,
          f"{max(live) / min(live):.1f}x  ({min(live):.1f} to {max(live):.1f} N/unit)")
    truth("linear's slope is flat",
          abs(slope(0.2, 60.0, "linear") - slope(0.9, 60.0, "linear")) < 1e-6)

    print("\n[4] monotonicity, which is the one thing the loop requires")
    for name in MONOTONIC_CURVES:
        vals = [curve(u, 60.0, name) for u in np.linspace(0, 1, 101)]
        truth(f"{name} is monotonic",
              all(b >= a - 1e-9 for a, b in zip(vals, vals[1:])))
    vals = [curve(u, 60.0, "reversed") for u in np.linspace(0, 1, 101)]
    truth("reversed is NOT monotonic (the negative control)",
          any(b < a - 1e-6 for a, b in zip(vals, vals[1:])))
    # *** AND THE FALLING BRANCH MUST BE REACHABLE. ***  A curve that is
    # non-monotonic only where the operating point never goes is not a test.
    # My first version peaked at 0.6 and bottomed at 0.4 of full, so a 32%
    # hover demand was met on the rising branch and the loop converged -
    # the check passed while proving nothing. A demand below full thrust
    # must now have a SECOND equilibrium above the peak.
    demand = 19.44
    above = [u for u in np.linspace(0.5, 1.0, 101)
             if abs(curve(u, 60.0, "reversed") - demand) < 1.0]
    truth("  a hover-sized demand has an equilibrium PAST the peak",
          bool(above), f"u ~ {above[0]:.3f}" if above else "none - vacuous!")
    truth("  and the slope is negative there",
          bool(above) and slope(above[0], 60.0, "reversed") < 0.0,
          f"{slope(above[0], 60.0, 'reversed'):.1f} N/unit" if above else "")
    truth("  no discontinuous jump at full open",
          abs(curve(1.0, 60.0, "reversed")
              - curve(0.99, 60.0, "reversed")) < 2.0,
          f"{curve(0.99, 60.0, 'reversed'):.2f} -> "
          f"{curve(1.0, 60.0, 'reversed'):.2f} N")

    print("\n[5] the curve scales, so difficulty is not engine-size")
    for m in (10.0, 60.0):
        f60 = [curve(u, 60.0, "ball") / 60.0 for u in np.linspace(0, 1, 51)]
        fm = [curve(u, m, "ball") / m for u in np.linspace(0, 1, 51)]
        truth(f"ball shape identical at {m:g} N",
              max(abs(a - b) for a, b in zip(f60, fm)) < 1e-12)

    print("\n[6] feed pressure and the regulated window")
    v = ValveActuator(None, 1, 60.0, curve_name="linear", isp_s=57.8)
    truth("starts regulated", v.regulated and v.feed_fraction() == 1.0,
          f"tank {v.p_tank_psig:.0f} psig")
    truth("tank starts at the configured pressure",
          abs(v.p_tank_psig - TANK_INITIAL_PSIG) < 1.0)
    # Burn it down and find the dropout.
    v.commanded = 1.0
    t, dt, dropout = 0.0, 0.002, None
    while t < 200.0:
        v.update(None, dt)
        t += dt
        if dropout is None and not v.regulated:
            dropout = t
            break
    truth("the regulator eventually drops out", dropout is not None,
          f"at t={dropout:.1f}s wide open" if dropout else "never")
    truth("the analytic window agrees with the simulated dropout",
          dropout is not None and abs(v.regulated_window_s() - dropout) < 0.5,
          f"analytic {v.regulated_window_s(1.0):.2f}s vs sim {dropout:.2f}s")
    # TWO operating points, and they differ by 2.4x - worth being explicit
    # about, because quoting one number for "the window" is how I first got
    # this wrong. Full open is the worst case; a hover profile actually sits
    # near the part-open hold point, which is where the ~29 s figure came
    # from (the hold point is 42.5% of full-open flow).
    w_open = v.regulated_window_s(1.0)
    w_hold = v.regulated_window_s(VALVE_HOLD_POSITION)
    truth("the full-open window is the worst case", w_open < w_hold,
          f"{w_open:.1f}s open vs {w_hold:.1f}s at the hold point")
    truth("and the HOLD-point window covers a standard ~18 s profile",
          w_hold > 18.0, f"{w_hold:.1f}s > 18s")
    # *** The bottle must scale with the engine. *** The 2 L / ~29 s figure
    # was measured on the real 5.7 N thruster. A 60 N engine at Isp 57.8 s
    # burns 105.9 g/s against that thruster's 10.1 g/s, so holding 2 L fixed
    # gives a 1.2 s window - a vehicle whose tank could never have flown the
    # profile. Found by this check failing, which is what it is for.
    windows = []
    for fmax in (5.738, 10.0, 60.0):
        vv = ValveActuator(None, 1, fmax, curve_name="linear", isp_s=57.8)
        w = vv.regulated_window_s(VALVE_HOLD_POSITION)
        windows.append(w)
        truth(f"  a {fmax:g} N engine keeps the window", w > 18.0, f"{w:.1f}s")
    truth("  and every engine gets the SAME window, not a scaled one",
          max(windows) - min(windows) < 0.01,
          f"{min(windows):.2f}s to {max(windows):.2f}s")
    fixed = ValveActuator(None, 1, 60.0, curve_name="linear",
                          isp_s=57.8, tank_volume_l=2.0)
    truth("  an explicit 2 L bottle on 60 N is SHORT (why it scales)",
          fixed.regulated_window_s() < 3.0,
          f"{fixed.regulated_window_s():.1f}s")
    truth("  an explicit volume is honoured, not overridden",
          not fixed.tank_scaled and abs(fixed.tank_volume_m3 * 1e3 - 2.0) < 1e-9)
    truth("feed fraction falls after dropout",
          v.feed_fraction() < 1.0, f"{v.feed_fraction():.4f}")
    truth("thrust scales with the feed, shape preserved",
          abs(v.thrust_for(1.0) - 60.0 * v.feed_fraction()) < 1e-9)

    print("\n[7] 'constant' feed isolates the curve from the drift")
    c = ValveActuator(None, 1, 60.0, curve_name="linear",
                      feed_mode="constant", isp_s=57.8)
    c.commanded = 1.0
    for _ in range(20000):
        c.update(None, 0.002)
    truth("constant feed never decays",
          c.feed_fraction() == 1.0 and c.thrust_for(1.0) == 60.0)

    print("\n[8] from_tank couples to the sim's own tank")
    ft = ValveActuator(None, 1, 60.0, curve_name="linear", feed_mode="from_tank")
    ft.set_tank_fraction(0.5)
    truth("feed follows the tank fraction", ft.feed_fraction() == 0.5)
    truth("and scales thrust with it", abs(ft.thrust_for(1.0) - 30.0) < 1e-9)

    print("\n[9] bookkeeping a run needs afterwards")
    v2 = ValveActuator(None, 1, 60.0, curve_name="linear", isp_s=57.8)
    v2.commanded = 0.5; v2.update(None, 0.002)
    v2.commanded = 0.8; v2.update(None, 0.002)
    truth("travel is integrated", abs(v2.travel_total - 0.8) < 1e-9,
          f"{v2.travel_total:.4f}")
    v2.commanded = 1.0; v2.update(None, 0.002)
    truth("pinned time is counted", v2.pinned_s > 0.0, f"{v2.pinned_s:.4f}s")
    truth("not starved when demand is reachable",
          v2.note_starved(1.0, 30.0) is None)
    truth("starved when demand exceeds a wide-open valve",
          v2.note_starved(2.0, 999.0) == 2.0)
    truth("and the moment is recorded once", v2.note_starved(3.0, 999.0) == 2.0)
    v2.reset()
    truth("reset restores a full tank and a shut valve",
          v2.position == 0.0 and v2.starved_at_s is None
          and abs(v2.p_tank_psig - TANK_INITIAL_PSIG) < 1.0)

    print("\n[10] the actuator lag, when enabled")
    lag = ValveActuator(None, 1, 60.0, curve_name="linear", tau_s=0.010)
    lag.commanded = 1.0
    lag.update(None, 0.002)
    truth("a lagged valve does not jump", 0.0 < lag.position < 1.0,
          f"{lag.position:.4f} after one 2 ms tick")
    for _ in range(100):
        lag.update(None, 0.002)
    truth("but it gets there", lag.position > 0.99, f"{lag.position:.4f}")
    instant = ValveActuator(None, 1, 60.0, curve_name="linear", tau_s=0.0)
    instant.commanded = 1.0
    instant.update(None, 0.002)
    truth("tau=0 tracks immediately", instant.position == 1.0)

    print("\n[11] bad configuration is refused, not absorbed")
    truth("an unknown feed mode raises",
          _raises(lambda: ValveActuator(None, 1, 60.0, feed_mode="nope")))
    truth("a non-positive max thrust raises",
          _raises(lambda: ValveActuator(None, 1, 0.0)))
    truth("a non-positive tank volume raises",
          _raises(lambda: ValveActuator(None, 1, 60.0, tank_volume_l=0.0)))

    print("\n[12] describe() states what is flying")
    d = ValveActuator(None, 1, 60.0, curve_name="ball", isp_s=57.8).describe()
    truth("names the curve and the dead zone",
          "ball" in d and "dead zone" in d and "slope spread" in d)
    print("        " + d.replace("\n", "\n        "))

    print()
    print(bar)
    total = passed + len(failed)
    if failed:
        print(f"FAILED  {len(failed)}/{total}")
        for f in failed:
            print(f"  - {f}")
        print(bar)
        return 1
    print(f"ALL {total} CHECKS PASSED")
    print(bar)
    return 0


def _raises(fn):
    try:
        fn()
    except Exception:
        return True
    return False


def main() -> int:
    import argparse
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--selftest", action="store_true",
                   help="verify the curve family and the feed model, then exit")
    p.add_argument("--table", metavar="CURVE", default=None,
                   help="print a curve's characteristic and slope")
    args = p.parse_args()
    if args.selftest:
        return run_selftest()
    if args.table:
        print(f"{'u':>6} {'F [N]':>10} {'dF/du [N/unit]':>16}")
        for u in np.linspace(0, 1, 21):
            print(f"{u:6.3f} {curve(u, 60.0, args.table):10.3f} "
                  f"{slope(u, 60.0, args.table):16.2f}")
        return 0
    p.print_help()
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
