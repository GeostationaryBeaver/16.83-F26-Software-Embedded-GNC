#!/usr/bin/env python3
"""An example control law, kept deliberately minimal.

This exists to prove the harness works end to end - that state really
reaches your law and your commands really reach the thrusters. It is NOT
a controls lesson, and it is not meant to be good. Delete it, or copy it
as a starting point for your own law.

A control law is just a function:

    def my_law(state) -> dict[str, float | tuple[float, ndarray]]

`state` is a CraftState (see thrusters.py): pos, vel, quat, angvel,
euler_deg, up, mass, t. Return a dict mapping thruster name to either a
magnitude in Newtons, or a (magnitude, direction) pair where direction is
a 3-vector in the body frame.

Run it:
    uv run python scripts/fly.py scenes/06_thrust_craft.xml \
        --controller example_hover:hover --duration 10
"""
import math
import sys
import threading

import numpy as np

# --- PARAMETERS -----------------------------------------------------------
TARGET_ALT_M = 2.0      # m, commanded altitude
KP_ALT = 18.0           # N per m of altitude error
KD_ALT = 22.0           # N per m/s of vertical speed

# --- GAINS, overridable at runtime ----------------------------------------
# Change these from the command line or the GUI rather than by editing:
#     uv run python scripts/scenario.py --controller example_hover:hover_pid \
#         --gains "KI_ALT=4,KD_ALT=25"
# set_gains() below is what applies them. Every name here is settable.
KI_ALT = 6.0            # N per m*s of accumulated error      (hover_pid)
I_LIMIT_N = 15.0        # N, cap on the integral's contribution (hover_pid)
ANTI_WINDUP = True      # stop integrating while the engine is saturated
KP_POS_V = 0.6          # (m/s) per m of error, outer loop  (hover_cascade)
KP_V = 15.0             # N per (m/s) of rate error, inner loop (hover_cascade)
V_MAX_MPS = 0.6         # m/s, limit on the commanded climb rate
SP_RATE_LPF = 0.05      # 0..1, smoothing on the differenced setpoint rate.
                        #   Measured on the standard profile: 0.002 -> lag
                        #   -0.0180 m, 0.01 -> -0.0089, 0.05 -> -0.0073, with
                        #   overshoot unchanged at 0.0518 m throughout. The
                        #   V_MAX_MPS clamp contains the step spike on its
                        #   own (peak vz pinned at exactly 0.600 m/s even on
                        #   a 7.6 m step), so the filter only has to smooth.

# hover_pid needs two things a control law cannot read off CraftState:
THRUST_CEILING_N = 60.0 # N, where the command saturates. The real ceiling
                        #   is the actuator's ctrlrange in the scene XML and
                        #   a law never sees it, so this is the figure the
                        #   anti-windup test uses. Set it to match the scene
                        #   you are flying (10.0 for 08_thrust_1D_smol), or
                        #   the integral will keep winding while the engine
                        #   is already pinned.
DT_NOMINAL_S = 0.002    # s, the integration step for the accumulator. Equal
                        #   to every scene's <option timestep>; with
                        #   --control-hz the law runs less often than this,
                        #   which scales the integral's effective gain.

# The names set_gains() will accept. A whitelist, so a typo in a sweep is an
# error rather than a silent no-op.
_GAIN_NAMES = ("KP_ALT", "KD_ALT", "KI_ALT", "I_LIMIT_N", "ANTI_WINDUP",
               "KP_POS_V", "KP_V", "V_MAX_MPS", "SP_RATE_LPF",
               "THRUST_CEILING_N", "OUTER_HZ",
               "KI_POS_V", "KD_POS_V", "KI_V", "KD_V",
               "I_LIMIT_V_MPS", "I_LIMIT_V_N", "D_LPF",
               "K_VALVE", "ACCEL_NOISE_SIGMA", "ACCEL_LPF", "GROUND_FREEZE",
               "GROUND_VZ_MPS",
               "DT_NOMINAL_S", "KP_POS", "KD_POS", "KP_TILT", "KD_TILT",
               "MAX_CANT")

# The gains that are FLAGS, not numbers. set_gains() sends these through
# _as_bool and everything else through float(). Listed rather than
# special-cased one at a time: GROUND_FREEZE was added to _GAIN_NAMES but
# not to the coercion, so `--gain GROUND_FREEZE=False` died with
# "could not convert string to float: 'False'" - the gain existed, was
# documented and was settable in name only.
_BOOL_GAINS = ("ANTI_WINDUP", "GROUND_FREEZE")

# --- FULL PID CASCADE (hover_cascade_pid) --------------------------------
# hover_cascade is P on position and P on rate. These add I and D to BOTH
# loops, so each is a full PID. Every one defaults to 0.0 except the filter,
# which means *** hover_cascade_pid REDUCES TO hover_cascade AT THE DEFAULTS
# *** - the selftest asserts that byte-for-byte. Turning a gain on is then
# an explicit act whose effect can be measured against that baseline.
KI_POS_V = 0.0          # (m/s) per (m*s). Integrates POSITION error into the
                        #   rate command. The cascade already tracks a ramp
                        #   with no standing error (the sp_rate feedforward
                        #   does that), so this is for a steady POSITION
                        #   offset the feedforward cannot see - a persistent
                        #   disturbance rather than a reference it is
                        #   following.
KD_POS_V = 0.0          # (m/s) per (m/s). Derivative on position error.
                        #   NOTE this is nearly redundant: d/dt(sp - z) is
                        #   -vz when the setpoint is constant, and the inner
                        #   loop already closes on vz through KP_V. It is
                        #   here for completeness and because it is NOT
                        #   identical during a ramp, when d/dt(sp) != 0.
KI_V = 0.0              # N per (m/s * s). Integrates RATE error into thrust.
                        #   This is the useful one for hardware: it rejects a
                        #   steady THRUST bias - a mis-estimated mass, a
                        #   thrust offset, a stuck trim - which no amount of
                        #   proportional gain removes, because holding a
                        #   constant rate against a bias needs a standing
                        #   rate error of bias/KP_V.
KD_V = 0.0              # N per (m/s^2). Derivative on rate error, i.e. on
                        #   ACCELERATION. Differencing a differenced signal,
                        #   so it is the noisiest term here by far and the
                        #   one most likely to be left at zero on real
                        #   hardware. D_LPF exists because of it.
I_LIMIT_V_MPS = 0.3     # m/s, cap on the OUTER integral's contribution.
                        #   Anti-windup, same lesson hover_pid teaches: an
                        #   unguarded integral made that law five times
                        #   WORSE than the PD it was meant to improve.
I_LIMIT_V_N = 20.0      # N, cap on the INNER integral's contribution.
D_LPF = 0.1             # 0..1 smoothing on both derivative terms; 1.0 = no
                        #   filter. Needed because KD_V differentiates a
                        #   velocity that is itself differenced from
                        #   position by the physics - see ACCEL_LPF for the
                        #   same argument in the thrust tracker.

OUTER_HZ = 25.0         # Hz, how often hover_cascade_2rate updates its
                        #   OUTER (position) loop. The inner rate loop runs
                        #   on every call, so this sets the rate RATIO that
                        #   makes it a genuine two-rate cascade. 0 means
                        #   "every call", i.e. identical to hover_cascade.

_integral = 0.0         # hover_pid's accumulator, in m*s
_sp_prev = None         # hover_cascade: previous setpoint, for differencing
_sp_rate = 0.0          # hover_cascade: filtered setpoint rate, m/s
# --- VALVE TRACKER (hover_valve) -----------------------------------------
# The inner loop that drives measured thrust to the demanded thrust by
# moving a valve, without knowing the valve's curve. See scripts/valve.py
# for the plant and the state-space argument.
K_VALVE = 0.05          # valve travel per second per Newton of error. The
                        #   loop eigenvalue is -K*gamma, so with the ball
                        #   curve's gamma spanning 9.1..292.8 N/unit this
                        #   one number gives a 32x spread in bandwidth -
                        #   which is the point of the sweep, not a flaw to
                        #   tune away. Discrete bound: K*gamma*T < 2, i.e.
                        #   K < 2.56 at T=2ms and gamma=391.
ACCEL_NOISE_SIGMA = 0.0 # m/s^2, 1-sigma noise on the measured acceleration.
                        #   0 = a perfect sensor. Nonzero is how the
                        #   does-this-survive-real-hardware question gets
                        #   answered, since the estimate is differentiated.
ACCEL_LPF = 1.0         # 0..1 smoothing on a_meas; 1.0 = no filter. Needed
                        #   once ACCEL_NOISE_SIGMA is nonzero, because
                        #   differencing velocity amplifies noise.
GROUND_FREEZE = True    # stop integrating while the floor is holding us up.
                        #   *** NOT OPTIONAL ON THE PAD. *** Measured on
                        #   07_thrust_1D: resting on the floor, F = m(a+g)
                        #   reads vehicle WEIGHT no matter what is commanded
                        #   (0 N commanded -> 19.440 N measured, error
                        #   +19.440). Airborne the same estimator is exact
                        #   (30 N -> 30.000, 60 N -> 60.000). So below-weight
                        #   demands on the ground give a permanently negative
                        #   error and wind the valve shut.
GROUND_VZ_MPS = 0.02    # how still counts as "on the pad". The freeze test
                        #   needs this: a ~= 0 with F_meas ~= weight is ALSO
                        #   true in steady cruise, and freezing there froze
                        #   the valve at u=0.5812 and flew a 2 m profile to
                        #   61 m. The pad is distinguished by being
                        #   stationary, not by being in balance.

_t_prev = None          # last tick's state.t, for measuring the real interval
_valve_t_prev = None    # the tracker's OWN clock - see _valve_tick_dt
_valve_u = 0.0          # the tracker's integrator: the valve position, and
                        #   at convergence an estimate of g^-1(F_cmd)
_a_meas = 0.0           # filtered acceleration estimate, m/s^2
_f_demand = 0.0         # the Newtons the altitude law last asked for. The
                        #   tracker does not need this - it is published so a
                        #   driver can log DEMAND against DELIVERY, which is
                        #   the only way to see the tracker working: the two
                        #   curves should sit on top of each other, and where
                        #   they part the valve has run out of authority.
_f_meas = 0.0           # the thrust the tracker last ESTIMATED from
                        #   acceleration - what it is actually closing on,
                        #   as distinct from what the valve delivered.
_vz_prev = None         # previous vertical speed, for differencing a_meas
_grounded_hint = None   # what the DRIVER knows about contact, if anything.
                        #   None means "nobody told me", and the tracker then
                        #   falls back to inferring it from the measurement.
_i_pos = 0.0            # outer integral: accumulated POSITION error, m*s
_i_vel = 0.0            # inner integral: accumulated RATE error, (m/s)*s
_d_pos = 0.0            # filtered d/dt of the position error, m/s
_d_vel = 0.0            # filtered d/dt of the rate error, m/s^2
_e_pos_prev = None      # previous position error, for differencing
_e_vel_prev = None      # previous rate error, for differencing
_outer_t = None         # when the outer loop last ran (hover_cascade_2rate)
_v_cmd_held = 0.0       # the outer loop's output, HELD between its updates


def _tick_dt(t):
    """Seconds since this law last ran, measured rather than assumed.

    DT_NOMINAL_S is only correct when the law runs every physics step. With
    --control-hz it runs less often, and a law that divides by the nominal
    step then overestimates every rate by the decimation factor - measured,
    the cascade's setpoint-rate estimate was 10x high at 50 Hz, which
    pinned v_cmd at the V_MAX_MPS clamp and drove a 0.1 m/s commanded
    descent at 0.42 m/s. Timing itself off state.t fixes that for any rate.

    Returns DT_NOMINAL_S on the first call (no previous timestamp) and after
    a reset, and refuses a non-positive interval, so a repeated or rewound
    timestamp cannot divide by zero.
    """
    global _t_prev
    dt = DT_NOMINAL_S if _t_prev is None else t - _t_prev
    _t_prev = t
    return dt if dt > 0.0 else DT_NOMINAL_S


def _valve_tick_dt(t):
    """Seconds since the VALVE TRACKER last ran. Its own clock, deliberately.

    It cannot share _tick_dt, and the reason is subtle enough to be worth
    stating. _tick_dt owns a single module-level _t_prev, and the second
    caller in any given tick sees dt <= 0 and gets the nominal fallback. A
    law and the tracker both calling it would therefore have ONE of them
    silently timed wrong - and at full rate the fallback happens to equal
    the true step, so it would look correct and only diverge under
    --control-hz (the law gets 0.020 while the tracker gets 0.002).

    Same fallback contract as _tick_dt: nominal on the first call and after a
    reset, and a non-positive interval refused.
    """
    global _valve_t_prev
    dt = DT_NOMINAL_S if _valve_t_prev is None else t - _valve_t_prev
    _valve_t_prev = t
    return dt if dt > 0.0 else DT_NOMINAL_S


def _as_bool(value):
    """Parse a flag that may arrive as a string from the command line.

    bool() is the wrong tool: bool("0") and bool("False") are both True, so
    --gains "ANTI_WINDUP=0" silently left the guard ON and the flag could
    never be turned off - which also made the anti-windup regression test
    pass vacuously, since both arms ran with it enabled.
    """
    if isinstance(value, str):
        v = value.strip().lower()
        if v in ("0", "false", "no", "off", ""):
            return False
        if v in ("1", "true", "yes", "on"):
            return True
        raise ValueError(f"cannot read {value!r} as a true/false flag")
    return bool(value)


def reset_state():
    """Clear any controller state that must not survive into the next run.

    Right now that is hover_pid's integrator. It matters for gain sweeps:
    two runs in one process would otherwise start with whatever the previous
    run left accumulated, and only the first result would be trustworthy.
    scenario.py calls this before every run.
    """
    global _integral, _sp_prev, _sp_rate, _t_prev, _outer_t, _v_cmd_held
    global _valve_t_prev, _valve_u, _a_meas, _vz_prev, _f_demand, _f_meas
    global _grounded_hint, _i_pos, _i_vel, _d_pos, _d_vel
    global _e_pos_prev, _e_vel_prev
    _integral = 0.0
    _sp_prev = None
    _sp_rate = 0.0
    _t_prev = None
    _outer_t = None
    _v_cmd_held = 0.0
    _valve_t_prev = None
    _valve_u = 0.0
    _i_pos = 0.0
    _i_vel = 0.0
    _d_pos = 0.0
    _d_vel = 0.0
    _e_pos_prev = None
    _e_vel_prev = None
    _f_demand = 0.0
    _f_meas = 0.0
    _a_meas = 0.0
    _vz_prev = None
    _grounded_hint = None   # back to inferring, so a run that does not set
                            #   it is not silently using the last run's value


def set_gains(**kw):
    """Override gains by name: set_gains(KP_ALT=20.0, KI_ALT=0.0).

    The companion to set_target() - same shape, same reason. A driver
    outside this module cannot know what a law called its gains, so this is
    the one agreed way in, and scenario.py / sim_worker.py both use it.

    An unknown name RAISES rather than being ignored. A silent no-op here
    would make a mistyped gain in a sweep look like a legitimate result,
    which is the worst possible failure for something whose whole purpose is
    comparing tunings.

    Setting any gain also clears the integrator, since an accumulator built
    up under the old tuning means nothing under the new one.
    """
    for name, value in kw.items():
        if name not in _GAIN_NAMES:
            raise KeyError(
                f"unknown gain {name!r}; settable gains are "
                f"{', '.join(_GAIN_NAMES)}")
        globals()[name] = (_as_bool(value) if name in _BOOL_GAINS
                           else float(value))
    reset_state()


def current_gains():
    """Every settable gain and its current value, for logging a run."""
    return {n: globals()[n] for n in _GAIN_NAMES}


def hover(state):
    """Hold TARGET_ALT_M with the main engine. Vertical axis only.

    Three terms, which is the shape of most real thrust commands:

      feedforward   m*g, the thrust needed just to not fall. Get this
                    term right and the feedback terms only have to
                    correct small errors instead of fighting gravity.
      proportional  KP_ALT * (altitude error) - pushes back toward the
                    target. Alone, it oscillates.
      derivative    -KD_ALT * (vertical speed) - damps that oscillation.

    Note what is NOT here: no attitude control at all. This law assumes
    the craft stays upright, which it does only because the main engine
    is on the thrust axis (X=Y=0) and so produces no torque. Steer it, or
    fire an offset RCS thruster, and the craft tumbles - that is the
    attitude problem, and it is yours to solve.
    """
    err_alt = TARGET_ALT_M - state.pos[2]
    err_vel = -state.vel[2]

    thrust = state.weight_n() + KP_ALT * err_alt + KD_ALT * err_vel

    # Returning a direction too, to show the (magnitude, direction) form.
    # [0,0,1] is straight down the craft's own axis. Thruster.set clamps
    # the magnitude to the actuator's ctrlrange, so no clipping needed.
    return {"main": (thrust, np.array([0.0, 0.0, 1.0]))}


# --- live-retargetable hover ----------------------------------------------
# Same law as hover() above; the only difference is where the setpoint comes
# from. hover() reads the TARGET_ALT_M constant, so changing it means editing
# this file and restarting. hover_interactive() reads a module-level variable
# that a background thread updates from whatever you type in the terminal.
ALT_MIN_M = 0.3         # m, floor clearance - refuse targets below this
ALT_MAX_M = 15.0        # m, keeps the craft in frame and inside thrust authority
ALT_STEP_M = 0.5        # m, nudge applied by a bare "+" or "-"

_target_alt = TARGET_ALT_M   # the live setpoint, written by the reader thread
_listener = None             # stdin thread; started on the first control tick


def _parse_target(line, current):
    """Turn one typed line into a new target altitude, or None for no change.

    Pure function - no thread, no I/O beyond printing - so it can be tested
    directly. Accepts an absolute value ("3.5"), a relative step ("+0.5",
    "-1"), a bare nudge ("+", "-"), or "?" to report the current target.

    Out-of-range requests are clamped AND announced. Silently clamping would
    leave you typing 20 and wondering why the craft stops at 15.
    """
    s = line.strip()
    if not s:
        return None
    if s in ("?", "h", "help"):
        print(f"  [alt] target is {current:.2f} m   "
              f"(type a number, or +/-, range {ALT_MIN_M}-{ALT_MAX_M} m)")
        return None
    if s == "+":
        want = current + ALT_STEP_M
    elif s == "-":
        want = current - ALT_STEP_M
    else:
        try:
            want = current + float(s) if s[0] in "+-" else float(s)
        except ValueError:
            print(f"  [alt] not a number: {s!r}  (try 3.5, +0.5, -1, or ?)")
            return None
    clamped = min(max(want, ALT_MIN_M), ALT_MAX_M)
    if clamped != want:
        print(f"  [alt] {want:.2f} m is outside "
              f"{ALT_MIN_M}-{ALT_MAX_M} m; using {clamped:.2f} m")
    print(f"  [alt] target -> {clamped:.2f} m")
    return clamped


def _start_listener():
    """Start the stdin reader, or report why we did not.

    Deliberately lazy: selftest_thrust.py imports this module, so starting a
    thread at import time would have --selftest competing for stdin. It is
    started on the first control tick instead.

    With no TTY (--headless, --video, piped input) there is nothing to read,
    so no thread is started and the target simply stays at its default. A
    thread looping on a closed stdin would spin on an immediate empty return.
    """
    try:
        interactive = sys.stdin is not None and sys.stdin.isatty()
    except (ValueError, AttributeError):
        interactive = False

    if not interactive:
        print(f"  [alt] no terminal attached; target fixed at "
              f"{_target_alt:.2f} m")
        return False

    def reader():
        global _target_alt
        for line in sys.stdin:                 # ends when stdin closes
            new = _parse_target(line, _target_alt)
            if new is not None:
                _target_alt = new              # atomic under the GIL

    t = threading.Thread(target=reader, daemon=True, name="alt-input")
    t.start()
    print(f"  [alt] target {_target_alt:.2f} m - type a new altitude "
          f"(or +/-, ?) and press Enter")
    return t


def set_target(alt_m):
    """Command a new target altitude. The scenario-runner convention.

    A control law keeps its setpoint wherever that law chose to keep it, so
    an outside driver has no way to guess the variable's name. A module that
    defines set_target() can be driven by scripts/scenario.py; one that does
    not has to be poked by hand.

    Both names are assigned because the laws in this module disagree about
    where the setpoint lives: hover() and vectored_translate() read the
    TARGET_ALT_M constant, while hover_interactive() reads the _target_alt
    global that its stdin thread writes. Setting both makes all three
    drivable through one call.
    """
    global _target_alt, TARGET_ALT_M
    TARGET_ALT_M = float(alt_m)
    _target_alt = float(alt_m)


def hover_interactive(state):
    """hover(), but the target altitude can be changed while the sim runs.

    Type a number in the terminal and press Enter. Everything else - the
    feedforward/P/D structure, the gains, the vertical-only assumption - is
    exactly hover(), so this is purely a demonstration of a live setpoint.

        uv run python scripts/fly.py scenes/06_thrust_craft.xml \
            --controller example_hover:hover_interactive --duration 60

    WORTH WATCHING: KP_ALT and KD_ALT were tuned for the small errors of
    holding an altitude, not for large setpoint jumps. Measured on
    06_thrust_craft, stepping the target from a settled hover:

        2 -> 3 m    overshoots 0.029 m, actuator never saturates
        3 -> 12 m   overshoots 1.193 m, actuator pinned at 60 N for 1.12 s

    A 9 m error asks KP_ALT for 162 N from a 60 N engine, so the command
    saturates: the loop is open until the error shrinks, and the D term only
    starts arresting the climb once it is too late. That is a real property
    of a PD loop against a saturating actuator, not a bug - and being able
    to retarget live is what makes it visible. A real vehicle fixes it by
    ramping the setpoint (flying a trajectory) rather than stepping it.
    """
    global _listener
    if _listener is None:
        _listener = _start_listener()

    err_alt = _target_alt - state.pos[2]
    err_vel = -state.vel[2]

    thrust = state.weight_n() + KP_ALT * err_alt + KD_ALT * err_vel

    return {"main": (thrust, np.array([0.0, 0.0, 1.0]))}


def hover_pid(state):
    """hover() plus an integral term. Kills steady-state error.

    THE MATH. Three feedback terms on top of the m*g feedforward:

        e(t)   = target - z                       altitude error, m
        u(t)   = m*g + KP*e + KD*(-vz) + KI*INT   thrust command, N
        INT    = integral of e dt                 accumulated error, m*s

    WHAT THE I TERM IS FOR. A P+D loop settles wherever its own output
    happens to balance the disturbance, so any constant error the P term
    must fight shows up as permanent offset. Measured, holding 2.0 m:

        feedforward exact          PD -0.00000 m   PID +0.00000 m
        feedforward 10% low        PD +0.10800 m   PID +0.00000 m
        feedforward 25% low        PD +0.27000 m   PID +0.00000 m

    And against a constant-rate descent, where a P+D law has a structural
    lag of -KD*rate/KP (-0.1222 m predicted at these gains):

        descent ramp 0.1 m/s       PD -0.12202 m   PID +0.00013 m

    The ramp case is the one that bites in normal use: it happens on every
    descent even when the feedforward is perfect.

    *** AND WHY ANTI-WINDUP IS NOT OPTIONAL. *** While the engine is
    saturated the loop is open - more command produces no more thrust - but
    a naive integrator keeps accumulating anyway, so by the time the error
    finally closes it has built up a huge spurious term. Measured, stepping
    0.4 -> 12 m against the 60 N ceiling:

        PD only                     peak 14.877 m   overshoot  +2.877 m
        PID, no anti-windup         peak 27.211 m   overshoot +15.211 m
        PID, conditional integral   peak 16.738 m   overshoot  +4.738 m

    Unguarded, the integral makes the law five times WORSE than the PD it
    was meant to improve. Two defences, both on by default:

      ANTI_WINDUP   only accumulate when the command is not saturated.
      I_LIMIT_N     cap the integral's contribution in Newtons regardless.
    """
    global _integral

    # The real interval since this law last ran. Taken UNCONDITIONALLY, even
    # on a tick where anti-windup then skips the accumulation: _tick_dt
    # advances the timestamp, so timing it inside the `if` would make the
    # first tick after a saturated stretch measure the whole stretch and
    # dump it into the integral in one go.
    dt = _tick_dt(state.t)

    err_alt = _target_alt - state.pos[2]
    err_vel = -state.vel[2]

    # The integral's contribution, clamped. Computed before the update so
    # the clamp applies to what is actually commanded this tick.
    i_term = max(-I_LIMIT_N, min(I_LIMIT_N, KI_ALT * _integral))

    unsaturated = state.weight_n() + KP_ALT * err_alt + KD_ALT * err_vel + i_term

    # Would this command be clipped? Thruster.set clamps to the actuator's
    # ctrlrange, and the lower bound is 0 because a rocket cannot pull.
    # THRUST_CEILING_N is a conservative stand-in: the real ceiling lives in
    # the XML, and a law cannot see it, so this uses the configured value.
    hit_limit = unsaturated < 0.0 or unsaturated > THRUST_CEILING_N

    if not (ANTI_WINDUP and hit_limit):
        # Accumulate over the MEASURED interval, not DT_NOMINAL_S: under
        # --control-hz the law runs less often, and adding a nominal 2 ms
        # per call would under-integrate by the decimation factor, silently
        # scaling KI_ALT down (10x at 50 Hz).
        _integral += err_alt * dt

    return {"main": (unsaturated, np.array([0.0, 0.0, 1.0]))}


def hover_cascade(state):
    """Two nested loops: position commands a climb RATE, rate commands thrust.

    THE IDEA. A single-loop PD on position has to produce a steady error to
    hold a steady velocity - that is what -KD*rate/KP means. Split the
    problem instead:

        OUTER (position)   v_cmd = KP_POS_V * (target - z)      [m/s]
                           clamped to +-V_MAX_MPS
        INNER (rate)       u     = m*g + KP_V * (v_cmd - vz)    [N]

    Now the thing being tracked IS a velocity, so following a constant-rate
    reference needs no accumulated error and no integrator at all. Measured
    mean lag on a 0.1 m/s descent ramp:

        PD         -0.12265 m
        PID (Ki=6) -0.01188 m
        cascade    +0.00119 m

    The cascade wins outright, and it wins structurally rather than by
    tuning: there is nothing left to accumulate.

    WHY THE CLAMP MATTERS. V_MAX_MPS is not cosmetic - it is the whole
    reason this law behaves on large steps. A 10 m error asks the outer loop
    for 6 m/s, which the inner loop would chase at full throttle; clamping
    the commanded rate turns an aggressive step into a controlled climb at a
    known speed, and the engine never saturates. That is also why the
    cascade overshoots 0.05 m where the PD overshoots 2.9 m on a big step:
    the rate limit is doing the work a trajectory generator normally would.

    GAIN CHOICE, from a sweep at 12 kg / 60 N stepping 0.4 -> 2.0 m:

        KP_POS_V  KP_V   overshoot   final
          0.6      6.0    +0.2846    1.9996
          0.6     15.0    +0.0518    2.0000   <- the default
          0.9      9.0    +0.2245    2.0000
          1.5     15.0    +0.1488    2.0000

    A stiff inner loop and a gentle outer one is the usual answer, and the
    numbers agree: raising KP_V helps, raising KP_POS_V hurts. The same
    gains converge on 08_thrust_1D_smol (1 kg, 10 N) with no overshoot, so
    they are not tuned to one vehicle.

    *** RATES. BOTH LOOPS RUN AT THE SAME RATE HERE. *** A textbook cascade
    runs its inner loop several times per outer update - that is how the
    outer loop gets to treat the inner one as instantaneous. This law does
    not: both lines below execute once per call, so both run at whatever
    rate the driver calls the law (every physics step by default, or
    --control-hz). The separation that makes the cascade work is in the
    GAINS, not the timing: KP_V = 15 against KP_POS_V = 0.6 is a 25x
    stiffness ratio, so the rate loop settles far faster than the position
    loop commands, which buys the same separation-of-timescales argument
    without a second timer. Measured, it holds: overshoot moves 0.0518 ->
    0.0581 m between 500 Hz and 10 Hz.

    What the law DOES need is the real interval between its own calls, which
    is why dt comes from _tick_dt(state.t) and not DT_NOMINAL_S. Dividing
    the setpoint difference by a nominal 2 ms while actually being called
    every 20 ms overestimates the rate 10x; that pinned v_cmd at the
    V_MAX_MPS clamp and drove a commanded 0.1 m/s descent at 0.42 m/s. With
    the measured interval, peak descent is -0.13 m/s at every rate from
    500 Hz to 10 Hz.
    """
    return {"main": (_cascade_demand(state), np.array([0.0, 0.0, 1.0]))}


def _cascade_demand(state):
    """The cascade's thrust demand in Newtons. Shared by two laws.

    Factored out of hover_cascade so hover_cascade_valve can reuse the exact
    same outer/inner arithmetic rather than keep a second copy of it - two
    copies of a control law is how the copies stop agreeing. Everything the
    docstring above says about rates, the setpoint-rate feedforward and the
    V_MAX_MPS clamp applies here, because this IS that code.
    """
    global _sp_prev, _sp_rate

    # The REAL interval since this law last ran, not the nominal timestep.
    # Both loops run in this one call, so they share it - see RATES below.
    dt = _tick_dt(state.t)

    sp = _target_alt

    # SETPOINT RATE, estimated by differencing. This is the term that makes
    # the cascade actually beat the PID on a ramp, and leaving it out is a
    # trap I walked into: without it the outer loop can only ask for a
    # descent by holding a standing position error of rate/KP_POS_V, which
    # at 0.1 m/s and KP_POS_V=0.6 is 0.167 m - measured -0.165 m, WORSE than
    # the PD law it was supposed to improve on. A law cannot be told the
    # ramp rate (CraftState carries no setpoint history), so it differences
    # the setpoint itself and feeds that forward.
    #
    # Low-pass filtered because a stepped setpoint differences to a spike:
    # a 1.6 m step over one 2 ms tick looks like 800 m/s.
    if _sp_prev is None:
        _sp_prev = sp
    raw_rate = (sp - _sp_prev) / dt
    raw_rate = max(-V_MAX_MPS, min(V_MAX_MPS, raw_rate))   # ignore step spikes
    _sp_rate += SP_RATE_LPF * (raw_rate - _sp_rate)
    _sp_prev = sp

    # OUTER LOOP: how fast should we be moving right now?
    #   feedforward  the setpoint's own rate, so tracking a ramp needs no
    #                standing position error at all
    #   proportional close whatever position error remains
    v_cmd = _sp_rate + KP_POS_V * (sp - state.pos[2])
    v_cmd = max(-V_MAX_MPS, min(V_MAX_MPS, v_cmd))

    # INNER LOOP: what thrust holds that rate?
    return state.weight_n() + KP_V * (v_cmd - state.vel[2])


def hover_cascade_pid(state):
    """hover_cascade with a FULL PID in each loop instead of P in each.

        OUTER   v_cmd  = sp_rate + KP_POS_V*e_z  + KI_POS_V*Ie_z  + KD_POS_V*De_z
        INNER   thrust = m*g     + KP_V*e_v      + KI_V*Ie_v      + KD_V*De_v

    where e_z = sp - z and e_v = v_cmd - vz. Same architecture as
    hover_cascade - position commands a rate, rate commands thrust, the rate
    command clamped by V_MAX_MPS - with two more terms per loop.

    *** AT THE DEFAULTS THIS IS hover_cascade, EXACTLY. *** All four new
    gains default to 0.0, and the selftest asserts the CSV is byte-identical
    to hover_cascade's. That is deliberate: it makes this law a strict
    superset, so any difference you measure is attributable to the gain you
    turned on rather than to having switched laws.

    WHAT EACH TERM IS ACTUALLY FOR, which is not symmetric:

      KI_V   (inner integral)  THE USEFUL ONE. It rejects a steady THRUST
            bias - a mis-estimated mass, a thrust offset, a stuck trim.
            Proportional gain cannot: holding a constant rate against a
            bias b needs a standing rate error of b/KP_V, forever. This is
            the term to reach for on real hardware, where the vehicle never
            weighs exactly what the controller thinks.
      KI_POS_V (outer integral)  For a steady POSITION offset. Note the
            cascade already tracks a RAMP with no standing error - the
            sp_rate feedforward does that - so this is not what removes
            descent lag, and adding it expecting that is a misreading of
            why the cascade wins.
      KD_V   (inner derivative)  Acts on ACCELERATION, since e_v is a
            velocity error. Differencing a differenced signal: the noisiest
            term here by a wide margin, hence D_LPF.
      KD_POS_V (outer derivative)  NEARLY REDUNDANT, and worth saying so.
            d/dt(sp - z) = -vz when the setpoint is constant, and the inner
            loop already closes on vz through KP_V. It differs only while
            the setpoint is moving, when d/dt(sp) != 0.

    ANTI-WINDUP, on both loops, because hover_pid's lesson applies twice
    over. Each integral's CONTRIBUTION is clamped (I_LIMIT_V_MPS,
    I_LIMIT_V_N) and neither accumulates while its own output is saturated -
    the outer against V_MAX_MPS, the inner against the thrust ceiling. An
    unguarded integral turned hover_pid into a law five times worse than the
    PD it was meant to improve; a cascade gives it two places to do that.
    """
    global _i_pos, _i_vel, _d_pos, _d_vel, _e_pos_prev, _e_vel_prev
    global _sp_prev, _sp_rate

    # One clock for both loops, measured not assumed - see _tick_dt. Taken
    # unconditionally, before any anti-windup branch: timing it inside the
    # branch would make the first tick after a saturated stretch measure the
    # whole stretch and dump it into the integral at once.
    dt = _tick_dt(state.t)
    sp = _target_alt

    # Setpoint rate feedforward, identical to hover_cascade's. This is what
    # tracks a ramp without a standing error, and it is NOT the integral.
    if _sp_prev is None:
        _sp_prev = sp
    raw_rate = max(-V_MAX_MPS, min(V_MAX_MPS, (sp - _sp_prev) / dt))
    _sp_rate += SP_RATE_LPF * (raw_rate - _sp_rate)
    _sp_prev = sp

    # --- OUTER LOOP: position -> rate command ----------------------------
    e_pos = sp - state.pos[2]
    if _e_pos_prev is None:
        _e_pos_prev = e_pos
    _d_pos += D_LPF * ((e_pos - _e_pos_prev) / dt - _d_pos)
    _e_pos_prev = e_pos

    i_pos_term = max(-I_LIMIT_V_MPS, min(I_LIMIT_V_MPS, KI_POS_V * _i_pos))
    v_raw = (_sp_rate + KP_POS_V * e_pos + i_pos_term + KD_POS_V * _d_pos)
    v_cmd = max(-V_MAX_MPS, min(V_MAX_MPS, v_raw))
    # Accumulate only if the rate command is not pinned at its clamp: the
    # clamp is the outer loop's actuator, and integrating against a pinned
    # actuator is exactly the windup hover_pid taught.
    if not (abs(v_raw) > V_MAX_MPS and v_raw * e_pos > 0.0):
        _i_pos += e_pos * dt

    # --- INNER LOOP: rate -> thrust --------------------------------------
    e_vel = v_cmd - state.vel[2]
    if _e_vel_prev is None:
        _e_vel_prev = e_vel
    _d_vel += D_LPF * ((e_vel - _e_vel_prev) / dt - _d_vel)
    _e_vel_prev = e_vel

    i_vel_term = max(-I_LIMIT_V_N, min(I_LIMIT_V_N, KI_V * _i_vel))
    thrust = (state.weight_n() + KP_V * e_vel + i_vel_term + KD_V * _d_vel)
    # Same guard against the thrust ceiling. THRUST_CEILING_N is a
    # configured stand-in because a law cannot see the actuator's ctrlrange.
    if not (thrust < 0.0 or thrust > THRUST_CEILING_N):
        _i_vel += e_vel * dt

    return {"main": (thrust, np.array([0.0, 0.0, 1.0]))}


def hover_cascade_2rate(state):
    """hover_cascade with the two loops run at genuinely DIFFERENT rates.

    hover_cascade computes both loops in one call, so both run at whatever
    rate the driver uses. This is the textbook structure instead: the outer
    position loop runs at OUTER_HZ, its commanded rate is HELD between those
    updates, and the inner rate loop closes on every call.

        every call      u      = m*g + KP_V * (v_cmd_held - vz)
        at OUTER_HZ     v_cmd_held = sp_rate + KP_POS_V * (sp - z)

    No harness change is needed for this: the driver already calls the law
    every physics step and CraftState carries t, so the law decimates its
    own outer loop off state.t. That is exactly how flight software is laid
    out - one scheduler, several tasks at different periods - and it is the
    reason a law, not the runner, is the right place to put the rate.

    WHAT IT IS FOR. Two things the single-rate version cannot show:

      1. It is what real hardware does. An estimator feeding a position
         loop at 25 Hz with a rate loop at 500 Hz is an ordinary design,
         and a sim that cannot represent it cannot predict it.
      2. It makes the rate RATIO a variable you can sweep. The whole
         justification for the single-rate law is that KP_V / KP_POS_V = 25
         gives enough timescale separation to stand in for a real one; this
         law is how that claim gets tested rather than asserted.

    MEASURED, standard profile on 07_thrust_1D at 12 kg / 60 N, against the
    single-rate law's 0.05176 m overshoot and -0.00731 m ramp lag - see the
    README table for the full sweep. Down to about 25 Hz the two are
    indistinguishable, which is the real finding: it VALIDATES the cheaper
    single-rate law at the shipped gains rather than replacing it. Below
    that the held v_cmd goes stale between updates and the overshoot grows.

    The zero-order hold is the point, and it is why _v_cmd_held is a module
    global rather than recomputed: between outer updates the inner loop
    chases a rate command that is deliberately out of date, which is the
    error a real two-rate system has and a single-rate one does not.
    """
    global _sp_prev, _sp_rate, _outer_t, _v_cmd_held

    dt = _tick_dt(state.t)
    sp = _target_alt

    # Setpoint rate, exactly as in hover_cascade. Estimated on the INNER
    # tick because that is when a real system would sample its setpoint
    # input; only the position loop below is decimated.
    if _sp_prev is None:
        _sp_prev = sp
    raw_rate = (sp - _sp_prev) / dt
    raw_rate = max(-V_MAX_MPS, min(V_MAX_MPS, raw_rate))
    _sp_rate += SP_RATE_LPF * (raw_rate - _sp_rate)
    _sp_prev = sp

    # --- OUTER LOOP, at OUTER_HZ ------------------------------------------
    # Scheduled on sim time, not a call counter, so the period is honoured
    # whatever rate the driver runs at - including under --control-hz, where
    # a counter would silently mean a different frequency.
    period = (1.0 / OUTER_HZ) if OUTER_HZ else 0.0
    due = (_outer_t is None                      # first call: run it
           or period <= 0.0                      # OUTER_HZ=0: every call
           or state.t - _outer_t >= period
           or state.t < _outer_t)                # sim reset: re-baseline
    if due:
        _v_cmd_held = max(-V_MAX_MPS,
                          min(V_MAX_MPS,
                              _sp_rate + KP_POS_V * (sp - state.pos[2])))
        _outer_t = state.t

    # --- INNER LOOP, every call -------------------------------------------
    # Note it closes on the HELD command, not a fresh one. That lag is the
    # whole difference between this law and hover_cascade.
    thrust = state.weight_n() + KP_V * (_v_cmd_held - state.vel[2])

    return {"main": (thrust, np.array([0.0, 0.0, 1.0]))}


def measured_thrust(state, dt):
    """Estimate the thrust actually being produced, from acceleration.

        F = m * (a + g)

    This is the accelerometer an airframe really carries, so it is estimated
    rather than read off the simulator. a is differenced from vertical speed
    because CraftState carries no acceleration field, then optionally
    corrupted with Gaussian noise and low-passed - differencing velocity
    amplifies noise, so the filter is needed as soon as the noise is on.

    Returns (F_meas, a_meas).
    """
    global _a_meas, _vz_prev
    vz = float(state.vel[2])
    a_raw = 0.0 if _vz_prev is None else (vz - _vz_prev) / dt
    _vz_prev = vz
    if ACCEL_NOISE_SIGMA > 0.0:
        a_raw += float(np.random.normal(0.0, ACCEL_NOISE_SIGMA))
    lpf = min(1.0, max(0.0, ACCEL_LPF))
    _a_meas += lpf * (a_raw - _a_meas)
    g = float(np.linalg.norm(state.gravity))
    return state.mass * (_a_meas + g), _a_meas


def thrust_tracker(f_cmd, state, grounded=None):
    """Drive measured thrust to f_cmd by moving a valve. Returns u in 0..1.

    *** THIS FUNCTION DOES NOT KNOW IT IS A THRUST TRACKER. ***  It is a
    generic servo: drive a measured scalar to a reference by integrating the
    error into an actuator position. Swap the accelerometer for a load cell
    or a chamber-pressure transducer and nothing here changes. That is why
    it needs no model of the valve.

    THE MATH, with g() the unknown valve curve and gamma = g'(u):

        u̇ = K ( r - g(u) )                 the closed loop, one state
        δ̇ = -K gamma δ                     linearised about g(u*) = r
        Y/R = K gamma / (s + K gamma)       DC gain EXACTLY 1 for any gamma>0
        E/R = s / (s + K gamma)             a zero at DC

    So the ACCURACY is independent of the curve and only the SPEED depends on
    it. The integrator's state converges to g^-1(r) without anyone computing
    an inverse, which is also why it re-converges for free when the curve
    drifts as the tank empties. A proportional law u = K*e has no state and
    would need a standing error to hold any thrust at all.

    TWO GUARDS, both measured to be load-bearing:

      clamp + conditional integration   u lives in [0, 1]; integrating while
            pinned is windup, the same lesson hover_pid teaches.
      GROUND_FREEZE                     on the pad the floor supplies the
            balance, so F = m(a+g) reads vehicle WEIGHT however little is
            commanded. The error is then permanently negative and the valve
            winds shut. Freezing while that is true is the fix; it is a
            broken OBSERVATION, not a plant property, so no gain change
            helps. It needs BOTH halves - resting AND a demand below weight
            - and getting that wrong failed in both directions on a real
            flight; see the comment on the guard itself.

    THE ONE REQUIREMENT, stated precisely. g must be monotonic OVER THE
    REGION THE LOOP VISITS - not everywhere. Measured against the
    deliberately non-monotonic "reversed" curve at a 19.44 N demand:

        started shut          converged, u=0.1620, 19.440 N   (g' > 0 here)
        started past the peak ran away to u=1.0000, 0.000 N

    Same curve, same gain, same demand: only the initial condition differs.
    Past the peak g' < 0, the feedback sign is inverted, and the integrator
    drives the valve exactly the wrong way until it pins. The ball curve
    from that same start converges normally, so it is the sign of the slope
    that matters and nothing else.
    """
    global _valve_u, _f_demand, _f_meas

    dt = _valve_tick_dt(state.t)
    f_meas, a_meas = measured_thrust(state, dt)
    _f_demand, _f_meas = float(f_cmd), float(f_meas)

    # WHEN THE MEASUREMENT CARRIES NO INFORMATION, DO NOT INTEGRATE ON IT.
    #
    # Resting on the pad, the floor supplies whatever force the engine does
    # not, so F = m(a+g) reads vehicle WEIGHT however little is commanded
    # (measured: 0 N commanded -> 19.440 N read). The error is then
    # permanently negative and the valve winds shut.
    #
    # But only for demands BELOW weight. Above it the engine really can
    # accelerate the vehicle, so the measurement becomes informative again
    # and freezing there means never launching - measured, a 48.2 N demand
    # against a 19.44 N weight sat at u=0.0000 for the whole 30 s ASCEND
    # phase and the craft never moved. So "resting" gates the freeze and
    # the demand decides it; contact alone is not enough.
    #
    # `grounded` is taken from the driver when it knows (scenario.py reads
    # data.ncon; an airframe has a leg switch), because the inference below
    # CANNOT tell the pad from steady cruise: holding altitude at
    # weight-thrust also gives a ~= 0 with F_meas ~= weight. Measured with
    # only the inference, the tracker froze at u=0.5812 against a 0 N demand
    # and flew a 2 m profile to 61 m. The velocity gate is what separates
    # the two cases, since the pad is STATIONARY rather than merely in
    # balance.
    resting = grounded
    if resting is None:
        resting = _grounded_hint
    if resting is None:
        resting = (abs(float(state.vel[2])) < GROUND_VZ_MPS
                   and abs(a_meas) < 0.05
                   and abs(f_meas - state.weight_n()) < 0.5)
    uninformative = (GROUND_FREEZE and bool(resting)
                     and f_cmd < state.weight_n() - 0.1)

    err = f_cmd - f_meas
    step = K_VALVE * err * dt
    pinned_low = _valve_u <= 0.0 and step < 0.0
    pinned_high = _valve_u >= 1.0 and step > 0.0
    if not (uninformative or pinned_low or pinned_high):
        _valve_u = min(1.0, max(0.0, _valve_u + step))
    return _valve_u


# Laws that return a 0..1 VALVE OPENING rather than Newtons. A driver reads
# this to refuse --valve on a law that returns force: 19.44 N read as an
# opening clamps to 1.0 and flies a wide-open engine, which looks like a wild
# controller rather than the units mismatch it is.
VALVE_LAWS = ("hover_valve", "hover_cascade_valve")


def set_grounded(flag):
    """Tell the tracker whether the vehicle is resting on something.

    A real airframe has a contact switch or a load cell on the legs; a sim
    driver has data.ncon. Either way the information is AVAILABLE, and a
    guess reconstructed from acceleration is strictly worse than being told:
    the inference cannot distinguish the pad from steady cruise, because
    both show a ~= 0 with measured thrust at vehicle weight.

    Pass None to go back to inferring. Called by scenario.py every tick.
    """
    global _grounded_hint
    _grounded_hint = None if flag is None else bool(flag)


def valve_state():
    """What the tracker is doing, for a driver that wants to log it.

    (u, a_meas, f_demand, f_meas). Reading these is NOT how the loop works -
    the tracker needs none of them - it is how a flight becomes legible:
    f_demand against f_meas shows the tracker converging, and u against
    either shows the curve it inverted without being told.
    """
    return _valve_u, _a_meas, _f_demand, _f_meas


def hover_valve(state):
    """hover(), but commanding a VALVE instead of Newtons.

    The outer law is unchanged and is literally the same arithmetic hover()
    uses - a PD on altitude producing a thrust DEMAND in Newtons. The
    difference is that the demand is then handed to thrust_tracker(), which
    returns a valve opening, and the valve's unknown curve turns that back
    into force. The altitude law never learns a valve exists.

    Returns a valve position in 0..1, not Newtons. ValveActuator.apply()
    interprets it, so this law only works with the valve layer enabled
    (--valve); flown without it, 0..1 would be read as 0..1 NEWTONS and the
    craft would never leave the ground.

    The timescale rule from hover_cascade applies again: the thrust loop has
    to be much faster than the altitude loop, or the altitude law sees a
    lagging actuator. Here that is satisfied by the tracker running every
    tick while the altitude dynamics are seconds long.
    """
    err_alt = _target_alt - state.pos[2]
    err_vel = -state.vel[2]
    f_cmd = state.weight_n() + KP_ALT * err_alt + KD_ALT * err_vel
    f_cmd = max(0.0, f_cmd)          # a valve cannot pull
    u = thrust_tracker(f_cmd, state)
    return {"main": (u, np.array([0.0, 0.0, 1.0]))}


def hover_cascade_valve(state):
    """hover_cascade, but commanding a VALVE instead of Newtons.

    *** THIS IS A THREE-LOOP CASCADE. *** hover_valve puts the tracker under
    hover's single-loop PD; this puts it under the cascade, so there are now
    three nested loops, each closing on a different measurement:

        OUTER   position -> rate       v_cmd = sp_rate + KP_POS_V*(sp - z)
        MIDDLE  rate -> thrust         F_cmd = m*g + KP_V*(v_cmd - vz)
        INNER   thrust -> valve        u̇    = K_VALVE*(F_cmd - F_meas)

    Each loop treats the one inside it as fast enough to be instantaneous,
    and that is a real requirement, not a formality - see the timescales
    note below. The demand comes from _cascade_demand(), which IS
    hover_cascade's own arithmetic rather than a copy of it, so the two laws
    cannot drift apart.

    WHY BOTHER, given hover_valve already flies. The cascade's advantage is
    structural: tracking a constant-rate descent needs no accumulated error,
    because the thing being tracked IS a velocity. Measured descent-ramp lag
    with direct thrust commands - PD -0.12177 m, cascade -0.00731 m, a 17x
    improvement - and that argument is about the OUTER loops, so it should
    survive putting a valve underneath. The point of this law is to check
    that it does, rather than assume it.

    TIMESCALES, which is the one thing that could break. Three loops need
    two separations, and both are satisfied here by gain stiffness rather
    than by timers:

        middle vs outer   KP_V=15 against KP_POS_V=0.6, a 25x ratio
        inner vs middle   the tracker's 1/(K*gamma) is 0.068..2.19 s on the
                          ball curve, against the rate loop's ~1/KP_V

    That second one is the weak link, and it is curve-dependent: where gamma
    is small the thrust loop is SLOWER than the rate loop above it, which
    inverts the cascade's assumption. If this law is worse than hover_valve
    on some curve, that is where to look - raising K_VALVE is the fix, not
    retuning the outer loops.

    Returns a valve position in 0..1, not Newtons, so it needs --valve.
    """
    f_cmd = max(0.0, _cascade_demand(state))   # a valve cannot pull
    u = thrust_tracker(f_cmd, state)
    return {"main": (u, np.array([0.0, 0.0, 1.0]))}


def world_to_body(state, v_world):
    """Rotate a world-frame vector into the craft's body frame.

    You need this whenever a control law reasons in world coordinates
    (where gravity and your waypoint live) but has to write a body-frame
    direction (which is what `gear` is). Skipping it works only while the
    craft is exactly upright - and then fails the moment it tilts, with
    the feedback sign effectively inverting.
    """
    import mujoco
    q_inv = np.zeros(4)
    mujoco.mju_negQuat(q_inv, np.asarray(state.quat, dtype=float))
    out = np.zeros(3)
    mujoco.mju_rotVecQuat(out, np.asarray(v_world, dtype=float), q_inv)
    return out


# --- a second, harder example --------------------------------------------
TARGET_X_M = 1.0        # m, where vectored_translate flies to
KP_POS = 1.2            # 1/s^2 per m of horizontal error
KD_POS = 2.2            # 1/s per m/s of horizontal speed
RCS_ARM_M = 0.25        # m, RCS nozzle height above the CoM = torque arm
MAX_CANT = 0.22         # max |horizontal|/|vertical| of the thrust vector,
                        #   i.e. ~12.4 deg. This is a SIZING result, not a
                        #   guess. The RCS quad can make 20 N * 0.25 m =
                        #   5.0 N*m. A canted main engine 0.25 m below the
                        #   CoM makes F*sin(cant)*0.25 N*m against it, so
                        #   at the 60 N ceiling a 12.4 deg cant costs
                        #   3.2 N*m - a ~1.5x authority margin. Raise this
                        #   and the attitude loop eventually loses.
KP_TILT = 7.0           # N*m per unit of tilt (state.up x/y component)
KD_TILT = 2.6           # N*m per rad/s of body rate


def vectored_translate(state):
    """Fly to (TARGET_X_M, 0, TARGET_ALT_M) by AIMING the main engine.

    This is the point of a steerable thrust vector: the sideways force
    comes from tilting the engine, not from a separate sideways thruster.

    It is also the example that shows why thrust vectoring is hard. Two
    things have to be right, and both are easy to get wrong:

    1. FRAMES. The desired force is computed in world coordinates, but
       `gear` is a direction in the BODY frame. They coincide only while
       the craft is upright. Without the world_to_body() call below, this
       law tilts the craft, the two frames diverge, the feedback inverts,
       and the craft departs - measured: 23 deg of tilt within 0.3 s and
       164 deg by 0.9 s, ending up 87 m in the WRONG direction.

    2. ATTITUDE. The main engine sits at X=Y=0 but 0.25 m BELOW the
       centre of mass. Aiming it off-axis therefore produces a torque as
       well as a side force - that is real thrust-vector control, and it
       means you cannot steer without also controlling attitude. The RCS
       quad does that here, with a small PD loop on tilt.

    None of this is tuned well. It is here to show the shape of the
    problem, and the shape of the fix.
    """
    # ---- 1. translation: what force do we want, in WORLD coordinates? --
    target = np.array([TARGET_X_M, 0.0, TARGET_ALT_M])
    err = target - state.pos
    a_des = np.array([
        KP_POS * err[0] - KD_POS * state.vel[0],
        KP_POS * err[1] - KD_POS * state.vel[1],
        (KP_ALT * err[2] + KD_ALT * (-state.vel[2])) / state.mass,
    ])
    # Subtract gravity so the engine also carries the weight.
    f_world = state.mass * (a_des - state.gravity)

    # Cap the cant. A steeply canted engine 0.25 m below the CoM makes a
    # large disturbance torque, and if it exceeds what the RCS can
    # produce the attitude loop simply loses. Clamping the horizontal
    # component keeps the problem inside the vehicle's authority.
    horiz = math.hypot(f_world[0], f_world[1])
    vert = max(f_world[2], 1e-6)
    if horiz > MAX_CANT * vert:
        f_world = f_world.copy()
        f_world[0] *= MAX_CANT * vert / horiz
        f_world[1] *= MAX_CANT * vert / horiz

    mag = float(np.linalg.norm(f_world))
    if mag < 1e-9:
        return {"main": 0.0}

    # ---- 2. THE FRAME CONVERSION. world -> body, because gear is body. -
    dir_body = world_to_body(state, f_world / mag)

    # ---- 3. attitude: keep the craft upright with the RCS quad --------
    # state.up is the craft's thrust axis in world coordinates. Its x and
    # y components say how far it has tipped toward world +X / +Y, and
    # they go to zero when upright - so they can serve as the tilt error
    # with no Euler angles or quaternion algebra.
    #
    # *** BUT THEY ALSO GO TO ZERO WHEN INVERTED. ***
    # up = [0,0,+1] (upright) and up = [0,0,-1] (upside down) both give
    # tip_x = tip_y = 0, so a naive law sees zero error either way. 180
    # deg is then a stable equilibrium, and measured, that is exactly
    # where this craft ended up: position converged beautifully to
    # x=1.003, z=2.000 while hanging inverted at tilt=179.9 deg.
    #
    # The fix: when up[2] < 0 the craft is inverted, and we need a
    # non-zero error that drives it back over. Using the horizontal part
    # of `up` normalized, with a bias, gives a signal that is largest
    # when inverted instead of smallest.
    tip_x, tip_y = float(state.up[0]), float(state.up[1])
    if state.up[2] < 0.0:
        horiz = math.hypot(tip_x, tip_y)
        if horiz < 1e-6:
            # Exactly inverted: pick an arbitrary axis to tip over.
            tip_x, tip_y = 1.0, 0.0
        else:
            # Push outward past 90 deg, where the normal law takes over.
            tip_x, tip_y = tip_x / horiz, tip_y / horiz

    # Desired restoring TORQUE, in N*m. Oppose the tip (P) and damp the
    # rotation (D). Watch the index pairing: tipping toward +X is a
    # rotation about the Y axis, so tip_x pairs with angvel[1].
    torque_y = -KP_TILT * tip_x - KD_TILT * float(state.angvel[1])
    torque_x = -KP_TILT * tip_y + KD_TILT * float(state.angvel[0])

    # Torque -> force, via the moment arm. This is the units step that is
    # easy to skip: the gains above are N*m, but a thruster takes Newtons.
    force_x = torque_y / RCS_ARM_M
    force_y = -torque_x / RCS_ARM_M

    # Differential throttling. A rocket can push but never pull, so a
    # signed command is split across the opposing pair: whichever sign is
    # needed fires, the other stays shut.
    return {
        "main": (mag, dir_body),
        "rcs_px": max(0.0, +force_x),
        "rcs_nx": max(0.0, -force_x),
        "rcs_py": max(0.0, +force_y),
        "rcs_ny": max(0.0, -force_y),
    }
