#!/usr/bin/env python3
"""Run any control law through one fixed, repeatable altitude profile.

    uv run python scripts/scenario.py --controller example_hover:hover
    uv run python scripts/scenario.py --controller example_hover:hover --target 3.5
    uv run python scripts/scenario.py --controller mine:my_law --scene scenes/06_thrust_craft.xml

The point is comparability. Two controllers flown through the same profile
produce two directories of plots and metrics you can put side by side, which
is not something you can do by watching numbers scroll past.

THE PROFILE
    GROUND_SIT    rest on the floor, setpoint = ground      2 s
    ASCEND        setpoint STEPS to the target              until arrived
    HOLD_TARGET   setpoint constant at the target           5 s
    DESCEND       setpoint RAMPS down at DESCENT_RATE_MPS   until landed
    GROUND_HOLD   rest on the floor again                   3 s

The hold clocks start when the craft ARRIVES and has settled, not when the
setpoint changes, so a slow controller still gets a full 5 s genuinely at
altitude rather than spending it climbing.

HOW THE DESCENT RATE WORKS, given the laws here are position-only. The
scenario does not command a velocity - it ramps the position SETPOINT at
DESCENT_RATE_MPS and lets the controller chase it. Measured on 07_thrust_1D
descending from 2.0 m: a stepped setpoint falls at 0.863 m/s peak, a 0.1 m/s
ramp at 0.103 m/s, a 0.5 m/s ramp at 0.508 m/s. So the rate is real, and no
control law had to change to honour it.

Outputs land in outputs/<scene>_<controller>_<stamp>/ as flight.csv,
flight.png, run.json and summary.txt.

SEEING IT. Two flags, because "render the run" means two different things:

    --video            write flight.mp4 into the run directory
    --view             watch it live in the interactive viewer

Both fly the IDENTICAL profile through the identical loop, so what you watch
is the run that is logged - not a re-flight that might diverge. --video is
headless and safe for a batch; --view needs a display and paces to real time.
"""
import argparse
import datetime as _dt
import json
import os
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

# --- THE PROFILE ----------------------------------------------------------
# One labeled block, every value with its unit, matching the house style in
# assignment1/lunar-hopper/02-cad/base_frame.py.
TARGET_ALT_M     = 2.0    # m, commanded hover altitude
DESCENT_RATE_MPS = 0.1    # m/s, rate the setpoint ramps down on the way home
GROUND_SIT_S     = 2.0    # s, sitting on the floor before launch
HOLD_AT_TARGET_S = 5.0    # s of SETTLED hold at the target altitude
HOLD_AT_GROUND_S = 3.0    # s, sitting on the floor again before the run ends
ARRIVE_TOL_M     = 0.05   # m, half-width of the "close enough" band
SETTLE_TOL_MPS   = 0.05   # m/s, must also be this slow to count as arrived
PHASE_TIMEOUT_S  = 30.0   # s, give up on a phase that never arrives
GROUND_VZ_MPS    = 0.02   # m/s, how still counts as resting on the pad when
                          #   telling a control law about contact. Paired
                          #   with data.ncon > 0: touching alone is true in a
                          #   bounce, slow alone is true at the top of a
                          #   climb, and only both together mean resting.

PHASES = ("GROUND_SIT", "ASCEND", "HOLD_TARGET", "DESCEND", "GROUND_HOLD")
STAMP_FMT = "%Y-%m-%d_%H%M%S"   # matches the 03 - Checks house convention

# --- PLOTTING -------------------------------------------------------------
THRUST_PLOT_TOL_N = 1e-9  # N. A thruster whose applied force and command
                          #   both stay under this for the whole run counts
                          #   as never fired and is left off the two thrust
                          #   panels. Measured on a vertical law, the four
                          #   RCS nozzles are exactly 0.0, so this only has
                          #   to reject float dust - it is NOT a "small
                          #   thrust" threshold, which would hide a real
                          #   trace. --all-thrusters plots everything.

# --- RENDERING (--video / --view) -----------------------------------------
VIDEO_FPS      = 30      # frames per second in the written mp4
VIDEO_WIDTH    = 1280    # px
VIDEO_HEIGHT   = 720     # px
VIDEO_CAMERA   = "pad"   # scene camera to film from. Tried in order:
                         #   this name, then "chase", then the free camera.
                         #
                         #   "pad" is a FIXED world camera, and that choice
                         #   matters more than it sounds. "chase" rides the
                         #   craft (mode="trackcom"), which pins the vehicle
                         #   to the same screen position - rendered, a 2 m
                         #   climb looks like a stationary capsule with a
                         #   sliding floor, and the altitude the whole
                         #   profile is about is the one thing you cannot
                         #   see. Verified by looking at both. Use
                         #   --camera chase when the thrust plume or the
                         #   attitude is the subject instead.
# Measured offscreen on osmesa (software GL): 18.5 ms/frame at 1280x720,
# 16.4 at 960x540, 13.3 at 640x360. So a full 18 s profile costs about 10 s
# of extra wall time at 720p - cheap enough that full quality is the default
# rather than something to opt into.


def vertical_indices(model, mujoco, body_id):
    """(qpos index, qvel index) of the craft's vertical coordinate.

    Derived rather than hardcoded because it moves with the joint type: a
    slide joint on Z keeps its position at qpos[adr], while a freejoint's
    layout is [x, y, z, qw, qx, qy, qz] so z lives at qpos[adr+2]. Assuming
    index 0 works on 07_thrust_1D and silently reads the X coordinate on
    06_thrust_craft.
    """
    jadr = int(model.body_jntadr[body_id])
    jtype = int(model.jnt_type[jadr])
    qadr = int(model.jnt_qposadr[jadr])
    vadr = int(model.jnt_dofadr[jadr])
    if jtype == int(mujoco.mjtJoint.mjJNT_FREE):
        return qadr + 2, vadr + 2
    return qadr, vadr


def parse_gains(spec):
    """'KP_ALT=20,KI_ALT=4' -> {'KP_ALT': 20.0, 'KI_ALT': 4.0}.

    Accepts a comma-separated string or a list of NAME=VALUE strings, so
    --gains "a=1,b=2" and repeated --gain a=1 --gain b=2 both work. Values
    stay strings here; the controller's set_gains() does the coercion and
    the name validation, which is the one place that knows what is legal.
    """
    if not spec:
        return {}
    items = spec if isinstance(spec, (list, tuple)) else [spec]
    out = {}
    for item in items:
        for part in str(item).split(","):
            part = part.strip()
            if not part:
                continue
            if "=" not in part:
                raise ValueError(
                    f"bad gain {part!r}: expected NAME=VALUE")
            name, _, value = part.partition("=")
            out[name.strip()] = value.strip()
    return out


def ground_height(model, data, mujoco, body_id):
    """The body-origin z at which the craft rests on the floor.

    Derived, never hardcoded, because it differs per scene: 07_thrust_1D's
    capsule (half-length 0.25 + radius 0.15) rests with its origin 0.40 m up,
    while 06_thrust_craft's box rests at 0.25 m. Measuring it also picks up
    the small soft-contact penetration, so the value is the one the sim will
    actually settle at rather than the one geometry predicts.

    Found by dropping a scratch copy of the state with no thrust and waiting
    for it to stop moving.
    """
    _, vi = vertical_indices(model, mujoco, body_id)
    scratch = mujoco.MjData(model)
    mujoco.mj_forward(model, scratch)
    for _ in range(int(8.0 / model.opt.timestep)):
        mujoco.mj_step(model, scratch)
        if scratch.ncon > 0 and abs(float(scratch.qvel[vi])) < 1e-6:
            break
    return float(scratch.xpos[body_id][2])


def place_on_ground(model, data, mujoco, body_id, z_rest):
    """Start the run already resting on the floor, at rest.

    The scenes spawn clear of the floor (07 at 0.5 m, 0.1 m up) so that
    nothing starts interpenetrating. But this profile begins ON the ground,
    and letting the craft simply drop would inject a 0.570 m/s impact
    transient into the middle of the first phase - not a repeatable start.

    Writing the joint coordinate instead puts it exactly at rest: measured
    ncon=1, vz=0, and max |vz| of 4.6e-06 m/s across the whole 2 s sit.
    """
    qi, _ = vertical_indices(model, mujoco, body_id)
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    z_spawn = float(data.xpos[body_id][2])
    # A slide joint's qpos is an OFFSET from the XML spawn, not an absolute
    # altitude, so this adds a delta rather than assigning. For a freejoint
    # qpos[qi] is absolute and the delta happens to be equivalent.
    data.qpos[qi] += z_rest - z_spawn
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)


class Profile:
    """The phase state machine. Advanced once per physics tick.

    Holds all the profile state so the tick loop stays readable, and so the
    phase timings can be handed to the plotter and the metadata file.
    """

    def __init__(self, z_ground, target_alt, descent_rate, dt,
                 ground_sit_s=None, hold_target_s=None, hold_ground_s=None,
                 ascent_rate=0.0):
        """The profile's shape, per run rather than per module.

        Every timing used to be a module constant read straight out of
        update(), so a run could set the target and the descent rate and
        nothing else. They are instance state now, defaulting to those same
        constants, so the three hold times are settable without touching
        code - that is what the CLI flags and the GUI fields drive.

        ascent_rate RAMPS the climb instead of stepping it. 0.0 keeps the
        original behaviour, which is a step: the setpoint jumps to the
        target on the first ASCEND tick and the law chases it as fast as it
        can. That asymmetry (descent ramped, ascent stepped) was not a
        design decision so much as the order the two were written in, and a
        step is how you measure step response - so it stays the default.
        A nonzero rate is how you ask for a CONTROLLED climb, which is what
        a real flight profile specifies.
        """
        self.z_ground = z_ground
        self.target = target_alt
        self.rate = descent_rate
        self.ascent_rate = max(0.0, float(ascent_rate))
        self.dt = dt
        self.ground_sit_s = (GROUND_SIT_S if ground_sit_s is None
                             else max(0.0, float(ground_sit_s)))
        self.hold_target_s = (HOLD_AT_TARGET_S if hold_target_s is None
                              else max(0.0, float(hold_target_s)))
        self.hold_ground_s = (HOLD_AT_GROUND_S if hold_ground_s is None
                              else max(0.0, float(hold_ground_s)))
        self.phase = "GROUND_SIT"
        self.setpoint = z_ground
        self.t_phase = 0.0          # seconds spent in the current phase
        self.boundaries = []        # (sim_time, from_phase, to_phase)
        self.timeouts = []          # phases that gave up
        self.done = False
        # A ramp's timeout has to cover the ramp. The descent takes
        # (target - ground)/rate seconds, ~16 s at the 0.1 m/s default, so a
        # flat timeout would fire during a perfectly correct run. Same now
        # applies to a ramped ascent, which is why this is a helper rather
        # than two copies of the arithmetic.
        travel = max(0.0, target_alt - z_ground)
        self._descend_timeout = travel / max(descent_rate, 1e-9) + PHASE_TIMEOUT_S
        self._ascend_timeout = (
            travel / self.ascent_rate + PHASE_TIMEOUT_S
            if self.ascent_rate > 0.0 else PHASE_TIMEOUT_S)
        # Where a ramped ascent has got to. Separate from self.setpoint
        # because GROUND_SIT keeps writing the ground into that.
        self._ascend_sp = z_ground

    def duration_estimate_s(self):
        """How long this profile should take if every phase behaves.

        Used for the wall-clock budget and reported before the flight, so a
        profile that cannot finish inside its own timeouts is visible before
        it burns ten minutes proving it.
        """
        climb = (max(0.0, self.target - self.z_ground) / self.ascent_rate
                 if self.ascent_rate > 0.0 else 0.0)
        fall = max(0.0, self.target - self.z_ground) / max(self.rate, 1e-9)
        return (self.ground_sit_s + climb + self.hold_target_s + fall
                + self.hold_ground_s)

    def describe(self):
        """One line naming every segment, for the run log and run.json."""
        climb = (f"ramp up at {self.ascent_rate:g} m/s"
                 if self.ascent_rate > 0.0 else "step up")
        return (f"sit {self.ground_sit_s:g}s -> {climb} to "
                f"{self.target:g} m -> hold {self.hold_target_s:g}s -> "
                f"down at {self.rate:g} m/s -> sit {self.hold_ground_s:g}s"
                f"   (~{self.duration_estimate_s():.0f}s)")

    def _advance(self, t_now, nxt):
        self.boundaries.append((t_now, self.phase, nxt))
        self.phase = nxt
        self.t_phase = 0.0
        if nxt == "DONE":
            self.done = True

    def _timeout_for(self, phase):
        if phase == "DESCEND":
            return self._descend_timeout
        if phase == "ASCEND":
            return self._ascend_timeout
        return PHASE_TIMEOUT_S

    def _arrived(self, z, vz, goal):
        """Is the craft at `goal` AND no longer moving?

        Both halves matter. Position alone fires while the craft is still
        travelling: measured, hover() enters the +-5 cm band at t=2.67 s but
        is still climbing toward a 2.0458 m overshoot and does not actually
        settle until t=6.14 s. Gating on speed too means a hold phase is
        genuinely spent at altitude.

        Note this tests against the PHASE GOAL, not self.setpoint. On the
        first ASCEND tick the setpoint has not been stepped up yet, so
        testing the setpoint would compare the craft against the ground it
        is still sitting on and declare it arrived immediately - which is
        exactly what the first version of this did.
        """
        return abs(z - goal) < ARRIVE_TOL_M and abs(vz) < SETTLE_TOL_MPS

    def update(self, t_now, z, vz):
        """Advance the machine one tick and return the commanded setpoint."""
        self.t_phase += self.dt

        if self.phase == "GROUND_SIT":
            self.setpoint = self.z_ground
            self._ascend_sp = self.z_ground
            if self.t_phase >= self.ground_sit_s:
                self._advance(t_now, "ASCEND")

        elif self.phase == "ASCEND":
            if self.ascent_rate > 0.0:
                # A RAMP, mirroring the descent. Arrival is still tested
                # against the TARGET, not this setpoint: the ramp reaches the
                # target before the craft does, and testing the setpoint
                # would call the phase finished while still climbing.
                self._ascend_sp = min(
                    self.target, self._ascend_sp + self.ascent_rate * self.dt)
                self.setpoint = self._ascend_sp
            else:
                self.setpoint = self.target      # a STEP, the default
            if self._arrived(z, vz, self.target):
                self._advance(t_now, "HOLD_TARGET")
            elif self.t_phase >= self._timeout_for("ASCEND"):
                self.timeouts.append("ASCEND")
                self._advance(t_now, "HOLD_TARGET")

        elif self.phase == "HOLD_TARGET":
            self.setpoint = self.target
            if self.t_phase >= self.hold_target_s:
                self._advance(t_now, "DESCEND")

        elif self.phase == "DESCEND":
            # THE RAMP. This is what makes the descent rate mean anything to
            # a position-only control law.
            self.setpoint = max(self.z_ground, self.setpoint - self.rate * self.dt)
            at_floor = self.setpoint <= self.z_ground + 1e-9
            if at_floor and self._arrived(z, vz, self.z_ground):
                self._advance(t_now, "GROUND_HOLD")
            elif self.t_phase >= self._timeout_for("DESCEND"):
                self.timeouts.append("DESCEND")
                self._advance(t_now, "GROUND_HOLD")

        elif self.phase == "GROUND_HOLD":
            self.setpoint = self.z_ground
            if self.t_phase >= self.hold_ground_s:
                self._advance(t_now, "DONE")

        return self.setpoint


def make_logger(ThrustLogger, names, valve=False):
    """A ThrustLogger that also records the setpoint, acceleration and phase.

    Subclassed rather than rewritten so the existing column order and CSV
    formatting stay exactly as fly.py --log produces them, and anything that
    already reads those files keeps working.

    Two acceleration columns:
      accel_mps2  differenced from velocity across ticks - what an onboard
                  estimate from a rate signal would look like.
      qacc_mps2   MuJoCo's own solver value.
    Measured, these agree to 6e-05 m/s^2 across a whole run, touchdown
    included, so either serves. Both are kept because they are not
    guaranteed to agree once the timestep is larger or the control rate is
    decimated, and having both makes a disagreement visible rather than
    silent.

    And both applied thrust (<name>_N, from the parent) and raw command
    (ctrl_<name>) are kept, because they differ whenever the actuator
    saturates - that difference is the saturation.

    valve=True appends five more columns, and ONLY then, so a normal run's
    CSV stays byte-identical and the existing readers keep working:

      valve_u        the opening the tracker commanded, 0..1. At convergence
                     this is g^-1(demand) - the inverse of a curve nobody
                     computed.
      thrust_cmd_N   what the altitude law asked for.
      thrust_meas_N  what the tracker ESTIMATED from acceleration. This is
                     the signal the loop actually closes on, and it is not
                     the same as the delivery: on the pad it reads vehicle
                     weight however little is commanded.
      thrust_act_N   what the valve actually delivered, which the controller
                     never sees. Logging the unknowable alongside the
                     knowable is the whole point of a test rig.
      feed_frac      how much of the curve's amplitude the feed still
                     supports, 1.0 while the regulator holds.
    """

    class ScenarioLogger(ThrustLogger):
        @property
        def header(self):
            cols = (super().header
                    + ["setpoint_m", "accel_mps2", "qacc_mps2", "phase"]
                    + [f"ctrl_{n}" for n in self.names]
                    + ["mass_kg", "propellant_kg"])
            if valve:
                cols += ["valve_u", "thrust_cmd_N", "thrust_meas_N",
                         "thrust_act_N", "feed_frac"]
            return cols

        def record_tick(self, state, magnitudes, setpoint, accel, qacc,
                        phase, ctrls, mass_kg, propellant_kg, valve_row=None):
            super().record(state, magnitudes)
            self.rows[-1].extend([setpoint, accel, qacc, phase]
                                 + [float(c) for c in ctrls]
                                 + [mass_kg, propellant_kg])
            if valve:
                self.rows[-1].extend([float(x) for x in
                                      (valve_row or (0.0,) * 5)])

        def save(self, path):
            """Like the parent, but tolerant of the non-numeric phase column.

            ThrustLogger.save formats every cell with %.6g, which raises on a
            string. Overridden here rather than changed upstream so fly.py's
            CSV output stays byte-identical.
            """
            import csv
            q = pathlib.Path(path)
            q.parent.mkdir(parents=True, exist_ok=True)
            with open(q, "w", newline="") as fh:
                w = csv.writer(fh)
                w.writerow(self.header)
                for r in self.rows:
                    w.writerow([v if isinstance(v, str) else f"{v:.6g}"
                                for v in r])
            return q

    return ScenarioLogger(names)


def metrics(rows, cols, prof, target, z_ground, descent_rate, max_thrust,
            tank=None, valve=None):
    """Reduce the flight to the numbers worth comparing between controllers."""
    ix = {c: i for i, c in enumerate(cols)}
    t = [r[ix["time_s"]] for r in rows]
    z = [r[ix["pos_z"]] for r in rows]
    vz = [r[ix["vel_z"]] for r in rows]
    ph = [r[ix["phase"]] for r in rows]
    sp = [r[ix["setpoint_m"]] for r in rows]

    def during(name):
        return [i for i, p in enumerate(ph) if p == name]

    out = {}
    asc, hold, desc, gh = (during("ASCEND"), during("HOLD_TARGET"),
                           during("DESCEND"), during("GROUND_HOLD"))

    out["peak_altitude_m"] = max(z) if z else float("nan")
    out["overshoot_m"] = max(0.0, out["peak_altitude_m"] - target)
    out["ascend_duration_s"] = (t[asc[-1]] - t[asc[0]]) if asc else None
    out["hold_mean_alt_m"] = (sum(z[i] for i in hold) / len(hold)) if hold else None
    out["hold_max_err_m"] = max((abs(z[i] - target) for i in hold), default=None)
    out["hold_rms_err_m"] = ((sum((z[i] - target) ** 2 for i in hold) / len(hold)) ** 0.5
                             if hold else None)
    out["peak_descent_rate_mps"] = min((vz[i] for i in desc), default=None)
    out["commanded_descent_rate_mps"] = -descent_rate
    # Steady lag behind a constant-rate ramp. For a P+D law this is not a
    # tuning artifact but a structural offset: at steady descent the
    # acceleration is zero, so KP*e + KD*rate = 0 and e = -KD*rate/KP. With
    # the example gains that predicts -0.1222 m and measures -0.1137 m. An
    # integral term is what removes it, so this metric is the one to watch
    # when comparing a PD law against a PID one.
    ramp = [i for i in desc if z_ground + 0.1 < sp[i] < target - 0.1]
    out["descent_lag_mean_m"] = (sum(sp[i] - z[i] for i in ramp) / len(ramp)
                                 if ramp else None)
    out["final_altitude_m"] = z[-1] if z else None
    out["ground_height_m"] = z_ground
    out["return_err_m"] = abs(z[-1] - z_ground) if z else None
    out["ground_hold_max_err_m"] = max((abs(z[i] - z_ground) for i in gh), default=None)
    out["rms_tracking_err_m"] = ((sum((z[i] - sp[i]) ** 2 for i in range(len(z)))
                                  / max(len(z), 1)) ** 0.5)
    # Saturation: how long the main command sat at its ceiling.
    main_ctrl = [r[ix["ctrl_main"]] for r in rows] if "ctrl_main" in ix else []
    sat = [c for c in main_ctrl if c >= max_thrust - 1e-6]
    dt = (t[1] - t[0]) if len(t) > 1 else 0.0
    out["saturated_samples"] = len(sat)
    out["saturated_s"] = len(sat) * dt
    out["timeouts"] = list(prof.timeouts)
    out["total_duration_s"] = t[-1] if t else 0.0
    if tank is not None:
        mk = [r[ix["mass_kg"]] for r in rows]
        out["mass_initial_kg"] = mk[0] if mk else None
        out["mass_final_kg"] = mk[-1] if mk else None
        out["propellant_used_kg"] = tank.used_kg
        out["propellant_remaining_kg"] = tank.propellant_kg
        out["tank_emptied_at_s"] = tank.emptied_at_s
        # Cross-check: the mass the model actually lost must equal what the
        # tank thinks it burned. A mismatch means a write went missing.
        out["mass_bookkeeping_err_kg"] = (abs((mk[0] - mk[-1]) - tank.used_kg)
                                          if mk else None)
    if valve is not None and "thrust_cmd_N" in ix:
        # How well the tracker did its ONE job: hold measured thrust to the
        # demand. Judged in the air only - on the pad the estimator reads
        # vehicle weight however little is commanded, so a ground sample
        # would score a broken observation as a tracking error.
        cmd = [r[ix["thrust_cmd_N"]] for r in rows]
        mea = [r[ix["thrust_meas_N"]] for r in rows]
        act = [r[ix["thrust_act_N"]] for r in rows]
        u = [r[ix["valve_u"]] for r in rows]
        air = [i for i, p in enumerate(ph)
               if p in ("ASCEND", "HOLD_TARGET", "DESCEND")]
        errs = [cmd[i] - mea[i] for i in air]
        out["valve_curve"] = valve.curve_name
        out["valve_feed_mode"] = valve.feed_mode
        out["thrust_err_rms_N"] = ((sum(e * e for e in errs) / len(errs)) ** 0.5
                                   if errs else None)
        out["thrust_err_max_N"] = max((abs(e) for e in errs), default=None)
        # The last HOLD_TARGET sample: the steady-state error the integrator
        # is supposed to drive to zero whatever the curve. This is the number
        # that should be curve-INDEPENDENT; the settling time is not.
        hold_i = [i for i, p in enumerate(ph) if p == "HOLD_TARGET"]
        out["thrust_err_final_N"] = (cmd[hold_i[-1]] - mea[hold_i[-1]]
                                     if hold_i else None)
        out["valve_u_final"] = u[hold_i[-1]] if hold_i else None
        out["valve_travel_total"] = valve.travel_total
        out["valve_pinned_s"] = valve.pinned_s
        out["valve_starved_at_s"] = valve.starved_at_s
        out["feed_frac_final"] = (rows[-1][ix["feed_frac"]] if rows else None)
        # Delivery vs estimate. The controller never sees this; it is here
        # to show that closing on an ESTIMATE still delivers the force.
        out["delivery_err_rms_N"] = (
            (sum((act[i] - mea[i]) ** 2 for i in air) / len(air)) ** 0.5
            if air else None)
    return out


def gains_used_by(law_name, module_name="example_hover"):
    """Which settable gains a control law actually reads.

    Read off the law's own bytecode (the global names it references,
    intersected with the module's _GAIN_NAMES whitelist) and followed one
    level into helpers it calls. Same spirit as find_controllers: the GUI
    discovers this instead of carrying a per-law table that would quietly
    go stale the first time a law is edited.

    An unknown law returns EVERY gain, so the failure mode is "nothing is
    greyed out" rather than "the box you need is disabled".
    """
    import importlib
    mod = importlib.import_module(module_name)
    names = set(getattr(mod, "_GAIN_NAMES", ()))
    fn = getattr(mod, law_name, None)
    if not hasattr(fn, "__code__"):
        return names
    out, seen, stack = set(), set(), [fn]
    while stack:
        f = stack.pop()
        if id(f) in seen:
            continue
        seen.add(id(f))
        for n in f.__code__.co_names:
            if n in names:
                out.add(n)
                continue
            helper = getattr(mod, n, None)
            if (hasattr(helper, "__code__")
                    and getattr(helper, "__module__", None) == mod.__name__):
                stack.append(helper)
    return out


def active_thrusters(rows, cols, names, tol=THRUST_PLOT_TOL_N):
    """The thrusters that actually fired, for plotting.

    Every thrust scene declares four RCS nozzles alongside the main engine,
    and a vertical control law never touches them - measured, they are
    EXACTLY 0.0 N for the whole run. Plotting them puts four flat lines on
    the zero axis of two panels and four entries in each legend, which
    crowds out the only trace that carries information.

    The test is "did it fire", not "is it called rcs": a law that does use
    the RCS still gets its nozzles plotted, with no list to maintain here.

    Returns (kept, dropped). If NOTHING fired - a law that commanded zero
    thrust throughout, or a timed-out run - everything is kept, because an
    empty panel hides the finding that the engine never lit.
    """
    ix = {c: i for i, c in enumerate(cols)}
    kept, dropped = [], []
    for n in names:
        fired = False
        for key in (f"{n}_N", f"ctrl_{n}"):
            j = ix.get(key)
            if j is not None and any(abs(r[j]) > tol for r in rows):
                fired = True
                break
        (kept if fired else dropped).append(n)
    return (kept, dropped) if kept else (list(names), [])


def config_lines(controller, scene, gains, overridden, profile, mass_note,
                 metrics=None, gain_names=None):
    """The text for the figure's configuration panel, as (label, value) rows.

    A figure that does not say what produced it is not much use in a report:
    two flight.png files from different tunings are indistinguishable. This
    puts the law, the gains it actually read, and the headline numbers on
    the figure itself.

    Only the gains the law READS are listed - the same bytecode-derived set
    the GUI greys its boxes with. Printing all sixteen settable constants
    would bury the three that mattered, and listing KI_ALT under a cascade
    run would be actively misleading, since the cascade has no integrator.

    An overridden gain is marked, because "KP_ALT 18.0" alone cannot
    distinguish a deliberate choice from the default.
    """
    overridden = overridden or {}
    rows = [("controller", controller), ("scene", scene)]
    if profile:
        rows.append(("profile", profile))
    if mass_note:
        rows.append(("mass", mass_note))
    rows.append(("", ""))           # separator: setup above, gains below

    if gain_names is None:
        gain_names = set(gains or ())
    shown = [n for n in (gains or ()) if n in gain_names]
    # Stable, readable order: the gains this law reads, alphabetically,
    # rather than dict order, which depends on _GAIN_NAMES' layout.
    for n in sorted(shown):
        v = gains[n]
        v = f"{v:g}" if isinstance(v, (int, float)) and not isinstance(v, bool) \
            else str(v)
        rows.append((n, v + ("   (set)" if n in overridden else "")))

    # An override of a gain the law does not read is worth SAYING, not
    # hiding: it means a sweep is varying something with no effect.
    inert = [n for n in overridden if n not in gain_names]
    if inert:
        rows.append(("unused overrides", ", ".join(sorted(inert))))

    shown_metrics = [(k, f"{metrics[k]:+.5f}")
                     for k in ("overshoot_m", "hold_rms_err_m",
                               "descent_lag_mean_m", "return_err_m")
                     if metrics and k in metrics]
    if shown_metrics:
        rows.append(("", ""))       # separator: gains above, results below
        rows.extend(shown_metrics)
    return rows


def plot_flight(path, rows, cols, prof, target, z_ground, descent_rate,
                names, show_mass=False, all_thrusters=False,
                config=None, save=True, show_valve=False):
    """One stacked figure, shared time axis, phases shaded.

    Returns (path, fig). path is None when save=False, fig is None when it
    is True - the saved path closes its figure, the unsaved one hands it
    over.

    Agg is selected before pyplot is imported because these runs are
    headless; picking a backend afterwards has no effect. But it is only
    FORCED when no interactive backend is already live: the GUI selects
    TkAgg before calling in, and stomping that would hand it a figure no
    canvas can draw.

    save=False keeps the figure OPEN and writes nothing, which is what an
    embedded canvas needs - plt.close() is exactly the wrong thing to do to
    a figure somebody is about to render.

    all_thrusters=True plots every nozzle including the ones that never
    fired (--all-thrusters); by default those are dropped.

    config, from config_lines(), adds a text panel down the right-hand side
    naming the law, its gains and the headline metrics, so the figure says
    what produced it.

    show_valve=True adds two more rows for a --valve run: the thrust loop
    (demand, estimate and actual delivery on one axis) and the valve itself
    (opening and feed fraction). Those are the only panels where the test
    rig's point is visible - the controller closes on the ESTIMATE, and the
    delivery it never sees has to follow anyway.
    """
    import matplotlib
    if matplotlib.get_backend().lower() in ("agg", "template"):
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ix = {c: i for i, c in enumerate(cols)}
    col = lambda n: [r[ix[n]] for r in rows]
    t = col("time_s")

    if all_thrusters:
        plotted, hidden = list(names), []
    else:
        plotted, hidden = active_thrusters(rows, cols, names)

    # Panels are counted, not hard-coded, because --valve adds two and
    # variable mass adds one and they combine: 5, 6, 7 or 8 rows.
    n_panels = 5 + (1 if show_mass else 0) + (2 if show_valve else 0)
    i_mass = 5 if show_mass else None
    i_thr = 5 + (1 if show_mass else 0) if show_valve else None
    i_vlv = i_thr + 1 if show_valve else None
    if config:
        # A narrow second COLUMN rather than another row in the stack: the
        # text is about twenty short lines, and a full-width panel would
        # either waste most of its area or squash the plots it describes.
        # The text axes spans every row, so it can run the full height.
        fig = plt.figure(figsize=(13.6, 2.8 * n_panels),
                         constrained_layout=True)
        gs = fig.add_gridspec(n_panels, 2, width_ratios=[4.6, 1.0])
        ax = [fig.add_subplot(gs[i, 0]) for i in range(n_panels)]
        for a in ax[:-1]:
            a.sharex(ax[-1])
            a.tick_params(labelbottom=False)
        ax_cfg = fig.add_subplot(gs[:, 1])
    else:
        fig, ax = plt.subplots(n_panels, 1, figsize=(11, 2.8 * n_panels),
                               sharex=True, constrained_layout=True)
        ax_cfg = None

    # Phase bands, drawn first so the traces sit on top of them.
    shades = {"GROUND_SIT": "#e8e8e8", "ASCEND": "#dcecf7",
              "HOLD_TARGET": "#dff0df", "DESCEND": "#fdeadc",
              "GROUND_HOLD": "#e8e8e8"}
    spans, start, cur = [], t[0] if t else 0.0, col("phase")[0] if rows else ""
    for i, p in enumerate(col("phase")):
        if p != cur:
            spans.append((start, t[i], cur)); start, cur = t[i], p
    if rows:
        spans.append((start, t[-1], cur))
    for a in ax:
        for x0, x1, name in spans:
            a.axvspan(x0, x1, color=shades.get(name, "#ffffff"), zorder=0)
        a.grid(alpha=0.3, zorder=1)
    for x0, x1, name in spans:
        ax[0].text((x0 + x1) / 2, 0.97, name, transform=
                   ax[0].get_xaxis_transform(), ha="center", va="top",
                   fontsize=7.5, color="#444")

    ax[0].plot(t, col("setpoint_m"), "--", color="#c44", lw=1.4,
               label="commanded setpoint", zorder=3)
    ax[0].plot(t, col("pos_z"), color="#1f5fa8", lw=1.6, label="altitude", zorder=4)
    ax[0].axhline(target, color="#888", lw=0.8, ls=":", zorder=2)
    ax[0].axhline(z_ground, color="#888", lw=0.8, ls=":", zorder=2)
    ax[0].set_ylabel("altitude [m]")
    ax[0].legend(loc="center right", fontsize=8)
    ax[0].set_title("Standardized altitude profile", fontsize=11)

    ax[1].plot(t, col("vel_z"), color="#1f8a5f", lw=1.4, label="vz")
    ax[1].axhline(-descent_rate, color="#c44", ls="--", lw=1.0,
                  label=f"commanded descent {-descent_rate:+.2f} m/s")
    ax[1].axhline(0, color="#888", lw=0.8, ls=":")
    ax[1].set_ylabel("velocity [m/s]")
    ax[1].legend(loc="lower right", fontsize=8)

    ax[2].plot(t, col("qacc_mps2"), color="#8a5fbf", lw=1.0,
               label="qacc (solver, incl. contact)")
    ax[2].plot(t, col("accel_mps2"), color="#d08a00", lw=1.0, alpha=0.75,
               label="differenced from vz")
    ax[2].axhline(0, color="#888", lw=0.8, ls=":")
    ax[2].set_ylabel("accel [m/s$^2$]")
    ax[2].legend(loc="upper right", fontsize=8)

    ncol = max(1, len(plotted) // 2)
    for n in plotted:
        ax[3].plot(t, col(f"{n}_N"), lw=1.3, label=n)
    ax[3].set_ylabel("thrust applied [N]")
    ax[3].legend(loc="upper right", fontsize=8, ncol=ncol)

    for n in plotted:
        ax[4].plot(t, col(f"ctrl_{n}"), lw=1.3, label=f"ctrl_{n}")
    ax[4].set_ylabel("control input [N]")
    ax[4].legend(loc="upper right", fontsize=8, ncol=ncol)

    # Say what was left out, so a missing nozzle is never a mystery. The CSV
    # still has every column either way.
    if hidden:
        ax[4].text(0.005, 0.03,
                   f"not plotted (never fired): {', '.join(hidden)}"
                   f"   \u2014  --all-thrusters shows them",
                   transform=ax[4].transAxes, fontsize=7.5, color="#333",
                   ha="left", va="bottom",
                   bbox=dict(facecolor="white", alpha=0.75, edgecolor="none",
                             boxstyle="round,pad=0.25"))

    if show_mass:
        ax[i_mass].plot(t, col("mass_kg"), color="#1f5fa8", lw=1.6,
                        label="total mass")
        ax[i_mass].plot(t, col("propellant_kg"), color="#c44", lw=1.4, ls="--",
                        label="propellant remaining")
        ax[i_mass].set_ylabel("mass [kg]")
        ax[i_mass].legend(loc="upper right", fontsize=8)

    if show_valve and "thrust_cmd_N" in ix:
        # THE PANEL THAT SHOWS THE IDEA. Three traces on one axis:
        #   demand    what the altitude law asked for
        #   estimate  what the tracker measured - the only one it sees
        #   delivered what the valve actually produced, which it does not
        # demand vs estimate is the tracking error the loop is closing; a
        # gap between estimate and delivered is the estimator being wrong,
        # which on the pad it certainly is (it reads vehicle weight).
        a = ax[i_thr]
        a.plot(t, col("thrust_cmd_N"), "--", color="#c44", lw=1.5,
               label="demand (the altitude law)", zorder=4)
        a.plot(t, col("thrust_meas_N"), color="#1f5fa8", lw=1.3,
               label="estimated (what the loop sees)", zorder=3)
        a.plot(t, col("thrust_act_N"), color="#1f8a5f", lw=1.1, alpha=0.8,
               label="delivered (unknown to the loop)", zorder=2)
        a.set_ylabel("thrust loop [N]")
        # Shorter labels than the comment above, because at fontsize 7.5 the
        # explanatory versions ran past the panel's right edge and clipped.
        a.legend(loc="upper right", fontsize=7.5, framealpha=0.85)

        a = ax[i_vlv]
        a.plot(t, col("valve_u"), color="#8a5fbf", lw=1.5,
               label="valve opening u (the integrator)")
        a.set_ylabel("valve opening [0..1]")
        a.set_ylim(-0.05, 1.05)
        a.legend(loc="upper left", fontsize=7.5)
        # The feed on a twin axis, since it is a fraction on a different
        # meaning: u is commanded, feed is what the bottle can still support.
        ff = col("feed_frac")
        if min(ff, default=1.0) < 1.0 - 1e-9:
            a2 = a.twinx()
            a2.plot(t, ff, color="#d08a00", lw=1.2, ls=":",
                    label="feed fraction (blowdown)")
            a2.set_ylabel("feed fraction", color="#d08a00")
            a2.set_ylim(0.0, 1.05)
            a2.tick_params(axis="y", labelcolor="#d08a00")
            a2.legend(loc="lower left", fontsize=7.5)
        else:
            a.text(0.005, 0.03, "feed flat at 1.00 for the whole run "
                                "(regulator never dropped out)",
                   transform=a.transAxes, fontsize=7.5, color="#333",
                   ha="left", va="bottom",
                   bbox=dict(facecolor="white", alpha=0.75, edgecolor="none",
                             boxstyle="round,pad=0.25"))

    ax[-1].set_xlabel("time [s]")

    if ax_cfg is not None:
        ax_cfg.axis("off")
        ax_cfg.set_title("configuration", fontsize=10, loc="left")
        # Drawn as text at a fixed line pitch rather than as a matplotlib
        # table: a table forces every column to one width, which turns a
        # long controller name into an unreadable squeeze.
        import textwrap
        # Wrap before measuring, because the long rows (a module-qualified
        # controller name, the profile line) otherwise run off the panel -
        # the text is clipped at the figure edge with no warning.
        WRAP = 27
        drawn = []
        for label, value in config:
            if label == "":
                drawn.append(("", []))
            else:
                drawn.append((label, textwrap.wrap(str(value), WRAP) or [""]))
        # Pitch chosen from the ACTUAL line count so the column fills
        # whatever height the figure has, rather than a fixed guess that
        # leaves a third of the panel blank on a short run.
        n_lines = sum(1.75 + len(v) for lbl, v in drawn if lbl) \
            + sum(0.8 for lbl, _ in drawn if not lbl)
        pitch = min(0.021, 0.97 / max(n_lines, 1.0))
        y = 0.995
        for label, vlines in drawn:
            if not label:
                y -= pitch * 0.8
                continue
            ax_cfg.text(0.0, y, label, fontsize=7.4, color="#666",
                        va="top", ha="left", transform=ax_cfg.transAxes)
            y -= pitch * 0.95
            for vl in vlines:
                ax_cfg.text(0.0, y, vl, fontsize=8.4, color="#111",
                            va="top", ha="left", family="monospace",
                            transform=ax_cfg.transAxes)
                y -= pitch
            y -= pitch * 0.55

    # dpi=110 against the 11 in / 13.6 in figure widths is what makes the
    # PNG 1210 px / 1496 px wide, which the selftest pins. Do not change
    # either without updating that assertion.
    if save:
        fig.savefig(path, dpi=110)
        plt.close(fig)
        return path, None
    return None, fig


def load_run(path):
    """Read a finished run back off disk, into what plot_flight() wants.

    The GUI launches this script as a SUBPROCESS, so the figure object dies
    with the run that drew it and cannot be handed back. An interactive view
    therefore has to rebuild - and everything needed is already saved, so
    nothing extra has to be captured during a flight:

        flight.csv   rows and columns; `phase` is the one string column,
                     every other cell is a 6-significant-digit float
        run.json     target, descent rate, ground height, mass mode, and
                     the gains/metrics the config caption is built from

    Returns a dict ready to splat into plot_flight(). Raises FileNotFoundError
    if there is no CSV, because without it there is nothing to draw; a
    missing or corrupt run.json is tolerated instead, matching the way
    gui.find_runs() already survives one.
    """
    import csv as _csv

    path = pathlib.Path(path)
    csv_path = path / "flight.csv"
    if not csv_path.is_file():
        raise FileNotFoundError(f"{path.name} has no flight.csv to plot")

    with open(csv_path, newline="") as fh:
        reader = _csv.reader(fh)
        cols = next(reader)
        rows = []
        for line in reader:
            if not line:
                continue
            rows.append([v if c == "phase" else float(v)
                         for v, c in zip(line, cols)])

    meta = {}
    meta_path = path / "run.json"
    if meta_path.is_file():
        try:
            meta = json.loads(meta_path.read_text())
        except (json.JSONDecodeError, OSError):
            meta = {}          # same tolerance as gui.find_runs()

    prof = meta.get("profile") or {}
    # Thruster names come from the CSV's own header, so a run flown with a
    # different scene still plots without being told what was on it.
    #
    # A thruster column is <name>_N AND has a matching ctrl_<name>. That
    # second half matters: a --valve run adds thrust_cmd_N, thrust_meas_N
    # and thrust_act_N, which match the suffix but are not thrusters, and
    # taking them as names asked for a ctrl_thrust_cmd column that does not
    # exist (KeyError in the plotter, on a run whose own PNG was fine).
    names = [c[:-2] for c in cols
             if c.endswith("_N") and f"ctrl_{c[:-2]}" in cols]

    # The config caption, rebuilt so an interactive figure carries the same
    # provenance as the PNG. gains_used_by lives in this module, so the
    # "only the gains this law reads" rule applies here too.
    config = None
    if meta:
        law = str(meta.get("controller", "")).split(":")[-1]
        try:
            used = gains_used_by(law) if law else None
        except Exception:
            used = None
        z_ground = meta.get("ground_height_m")
        prop = meta.get("propellant")
        if prop:
            mass_note = (f"{prop.get('dry_mass_kg', 0):g} + "
                         f"{prop.get('propellant_initial_kg', 0):g} kg, "
                         f"Isp {prop.get('isp_s', 0):g} s")
        else:
            mass_note = "fixed mass"
        profile_note = ""
        if z_ground is not None and prof.get("target_alt_m") is not None:
            # "shape" is Profile.describe() as written at flight time, so a
            # rebuilt figure carries the same caption as the original PNG.
            # Older runs have no shape key, hence the fallback.
            shape = prof.get("shape") or (
                f"down at {prof.get('descent_rate_mps', 0):g} m/s")
            profile_note = (f"{z_ground:.3f} -> "
                            f"{prof['target_alt_m']:g} m; {shape}")
        config = config_lines(
            controller=meta.get("controller", "?"),
            scene=meta.get("scene", "?"),
            gains=meta.get("gains") or {},
            overridden=meta.get("gains_overridden"),
            profile=profile_note,
            mass_note=mass_note,
            metrics=meta.get("metrics"),
            gain_names=used)

    return {
        "rows": rows,
        "cols": cols,
        "names": names,
        "target": prof.get("target_alt_m", 0.0),
        "z_ground": meta.get("ground_height_m", 0.0),
        "descent_rate": prof.get("descent_rate_mps", 0.0),
        "show_mass": meta.get("propellant") is not None,
        # From the CSV's own header, not from run.json: the columns are what
        # the panels need, and a run.json that went missing must not silently
        # drop two panels that the data can still fill.
        "show_valve": "valve_u" in cols,
        "config": config,
        "meta": meta,
        "name": path.name,
    }


def figure_for_run(path, all_thrusters=False, config=True):
    """A run directory -> a live matplotlib figure, nothing written.

    The single seam the GUI uses. Keeping it here rather than in gui.py puts
    all knowledge of the run layout next to the code that wrote it, so the
    two cannot drift apart.
    """
    run = load_run(path)
    _, fig = plot_flight(
        None, run["rows"], run["cols"], None,
        run["target"], run["z_ground"], run["descent_rate"], run["names"],
        show_mass=run["show_mass"], all_thrusters=all_thrusters,
        show_valve=run["show_valve"],
        config=run["config"] if config else None, save=False)
    return fig, run


def run_directory(parent, name):
    """Create and return the directory for one run.

    The stamp resolves to one second, so two runs started inside the same
    second collide - and mkdir(exist_ok=True) would quietly overwrite the
    first one's files. Suffix instead, so no run can destroy another.

    Split out of main() because --video has to open its writer BEFORE the
    flight while everything else is written after it, and two copies of this
    logic could disagree about where a run lives.
    """
    rundir = parent / name
    if rundir.exists():
        for n in range(2, 100):
            alt = parent / f"{name}_{n}"
            if not alt.exists():
                rundir = alt
                break
    rundir.mkdir(parents=True, exist_ok=False)
    return rundir


class Recorder:
    """Writes an mp4 of a scenario run, filmed from the scene's chase camera.

    Deliberately NOT a second simulation. It is handed the same mjData the
    control loop is stepping, so the video is the run that produced the CSV.
    A re-flight would be a different run: a controller with an integrator,
    or any state at all, need not reproduce itself tick for tick.

    Frames are taken every Nth physics step, not every step: at a 2 ms
    timestep a full 18 s profile is ~9000 steps, and 30 fps needs ~540.
    """

    def __init__(self, mujoco, model, path, dt, fps=VIDEO_FPS,
                 width=VIDEO_WIDTH, height=VIDEO_HEIGHT, camera=VIDEO_CAMERA):
        import imageio
        self.mujoco = mujoco
        self.path = path
        self.fps = fps
        self.every = max(1, round((1.0 / fps) / dt))
        self.frames = 0
        self.renderer = mujoco.Renderer(model, height=height, width=width)
        self.camera, self.camera_name = self._resolve_camera(mujoco, model,
                                                             camera)
        self.camera_requested = camera
        # macro_block_size=1 keeps the requested resolution; ffmpeg otherwise
        # rounds up to a multiple of 16 and silently changes the dimensions.
        self.writer = imageio.get_writer(str(path), fps=fps,
                                         macro_block_size=1)

    @staticmethod
    def _resolve_camera(mujoco, model, name):
        """(id, name) for the camera to film from, falling back by name.

        Tries the requested name, then "chase", then the free camera (-1).
        Falling back rather than failing: a scene with no cameras at all
        should still record, just framed less usefully.
        """
        if name in (None, "", -1, "free"):
            return -1, "free"
        for candidate in (name, "chase"):
            cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA,
                                    candidate)
            if cid >= 0:
                return cid, candidate
        return -1, "free"

    def maybe_frame(self, data, step_i):
        """Capture if this step lands on a frame boundary."""
        if step_i % self.every:
            return
        # update_scene reads the CURRENT data. Omit it and every frame is
        # identical - which looks like a frozen video, not an error.
        self.renderer.update_scene(data, camera=self.camera)
        self.writer.append_data(self.renderer.render())
        self.frames += 1

    def close(self):
        try:
            self.writer.close()
        finally:
            self.renderer.close()


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--controller", metavar="MOD:FN", default=None,
                   help="control law to fly, e.g. example_hover:hover")
    p.add_argument("--scene", default="scenes/07_thrust_1D.xml",
                   help="MJCF scene (default: the 1-DOF vertical rig)")
    p.add_argument("--target", type=float, default=TARGET_ALT_M,
                   help=f"commanded altitude, m (default {TARGET_ALT_M})")
    p.add_argument("--descent-rate", type=float, default=DESCENT_RATE_MPS,
                   help=f"setpoint ramp rate down, m/s (default {DESCENT_RATE_MPS})")
    p.add_argument("--ascent-rate", type=float, default=0.0, metavar="MPS",
                   help="ramp the climb at this rate instead of stepping to "
                        "the target. 0 (default) keeps the step, which is "
                        "what makes the ASCEND phase a step-response "
                        "measurement; nonzero asks for a controlled climb")
    p.add_argument("--ground-sit", type=float, default=GROUND_SIT_S,
                   metavar="S",
                   help=f"seconds on the pad before launch "
                        f"(default {GROUND_SIT_S:g})")
    p.add_argument("--hold-target", type=float, default=HOLD_AT_TARGET_S,
                   metavar="S",
                   help=f"seconds of SETTLED hover at the target "
                        f"(default {HOLD_AT_TARGET_S:g})")
    p.add_argument("--hold-ground", type=float, default=HOLD_AT_GROUND_S,
                   metavar="S",
                   help=f"seconds on the pad after landing "
                        f"(default {HOLD_AT_GROUND_S:g})")
    p.add_argument("--control-hz", type=float, default=None,
                   help="run the law at this rate instead of every step")
    p.add_argument("--label", default=None,
                   help="extra suffix on the run directory name")
    p.add_argument("--outdir", default=None,
                   help="parent directory for run dirs (default: outputs/)")
    p.add_argument("--no-plot", action="store_true", help="skip the figure")
    p.add_argument("--no-config-panel", action="store_true",
                   help="omit the figure's controller/gains panel")
    p.add_argument("--all-thrusters", action="store_true",
                   help="plot every thruster, including ones that never "
                        "fired (by default those are left off)")
    p.add_argument("--video", action="store_true",
                   help="also record flight.mp4 into the run directory")
    p.add_argument("--view", action="store_true",
                   help="also watch the run live in the interactive viewer "
                        "(needs a display; paces to real time)")
    p.add_argument("--video-fps", type=int, default=VIDEO_FPS,
                   help=f"frame rate for --video (default {VIDEO_FPS})")
    p.add_argument("--video-size", default=f"{VIDEO_WIDTH}x{VIDEO_HEIGHT}",
                   metavar="WxH", help="frame size for --video")
    p.add_argument("--camera", default=VIDEO_CAMERA,
                   help=f"scene camera to film from (default "
                        f"{VIDEO_CAMERA!r}; 'free' for the free camera)")
    p.add_argument("--view-speed", type=float, default=1.0, metavar="X",
                   help="real-time multiplier for --view (2 = double speed)")
    p.add_argument("--gravity", default=None, help="preset or m/s^2")
    p.add_argument("--propellant", type=float, default=None,
                   help="usable propellant, kg (default: propellant.py's "
                        "PROPELLANT_KG). Mass falls as thrust is produced.")
    p.add_argument("--dry-mass", type=float, default=None,
                   help="vehicle mass with empty tanks, kg")
    p.add_argument("--isp", type=float, default=None,
                   help="specific impulse, s; sets mdot = F/(Isp*g0)")
    p.add_argument("--gains", default=None, metavar="N=V,...",
                   help="override controller gains, e.g. "
                        "\"KP_ALT=20,KI_ALT=4\". Applied through the "
                        "controller module's set_gains().")
    p.add_argument("--gain", action="append", default=None, metavar="N=V",
                   help="one gain override; repeatable")
    p.add_argument("--fixed-mass", action="store_true",
                   help="disable the propellant model and fly at the "
                        "scene's constant mass, for comparison")
    p.add_argument("--valve", action="store_true",
                   help="command the main engine through a valve whose "
                        "thrust curve the controller is not told. Requires "
                        "a law that returns a 0..1 opening, e.g. "
                        "example_hover:hover_valve")
    p.add_argument("--valve-curve", default=None, metavar="NAME",
                   help="which unknown relation to hide behind the valve: "
                        "ball, linear, quadratic, sqrt, deadzone, reversed. "
                        "Sweep it to test a FAMILY of plants against one "
                        "controller")
    p.add_argument("--valve-feed", default=None, metavar="MODE",
                   help="constant | regulated | from_tank. 'regulated' "
                        "holds the curve flat until the bottle drops below "
                        "the regulator setpoint, then scales it - the "
                        "time dependence as pressure leaves the system")
    p.add_argument("--valve-tau", type=float, default=None, metavar="S",
                   help="first-order valve actuator lag, s (default 0: the "
                        "real ~1 ms lag is below the physics step)")
    p.add_argument("--valve-tank-l", type=float, default=None, metavar="L",
                   help="gas bottle volume, L. The default is SIZED FROM "
                        "THE ENGINE so the regulated window is the same "
                        "number of seconds on any scene. Pass a small "
                        "value to make the regulator drop out inside a "
                        "profile and see the loop absorb the blowdown")
    p.add_argument("--selftest", action="store_true",
                   help="verify the profile machine numerically and exit")
    args = p.parse_args()

    if args.selftest:
        return run_selftest()

    # Headless throughout; and MUJOCO_GL must be set before mujoco imports.
    # --view is the exception: an interactive window needs a real GL context,
    # and osmesa (software, offscreen) cannot provide one. Setting this after
    # mujoco is imported has no effect, which is why it happens here.
    #
    # The display is checked FIRST because glfw aborts the whole process at
    # import time when DISPLAY is unset ("could not initialize GLFW", exit 1)
    # - earlier than any try/except around launch_passive could catch, so a
    # --view run on a headless box lost the run itself rather than just the
    # window. Falling back to a headless run keeps the data.
    if args.view and not (os.environ.get("DISPLAY")
                          or os.environ.get("WAYLAND_DISPLAY")):
        print("note: no DISPLAY, so --view cannot open a window; running "
              "headless.\n      Use --video to get a file you can watch "
              "instead.", file=sys.stderr)
        args.view = False
    if args.view:
        os.environ["MUJOCO_GL"] = os.environ.get("MUJOCO_GL_VIEW", "glfw")
    else:
        os.environ.setdefault("MUJOCO_GL", "osmesa")

    try:
        vid_w, vid_h = (int(x) for x in str(args.video_size).lower().split("x"))
    except ValueError:
        print(f"error: --video-size wants WxH, got {args.video_size!r}",
              file=sys.stderr)
        return 1
    if args.video_fps <= 0:
        print("error: --video-fps must be positive", file=sys.stderr)
        return 1

    from fly import GRAVITY_PRESETS, load_controller, resolve_scene, control_tick

    if not args.controller:
        p.error("--controller is required (or use --selftest)")

    scene = resolve_scene(args.scene)
    if not scene.exists():
        print(f"error: no such scene: {args.scene}", file=sys.stderr)
        return 1
    try:
        controller = load_controller(args.controller)
    except (ImportError, AttributeError, ValueError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1

    import mujoco
    from thrusters import CraftState, ThrusterSet, ThrustLogger

    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)

    if args.gravity is not None:
        g = GRAVITY_PRESETS.get(args.gravity.lower())
        if g is None:
            try:
                g = float(args.gravity)
            except ValueError:
                print(f"error: bad --gravity {args.gravity!r}", file=sys.stderr)
                return 1
        model.opt.gravity[2] = g

    thrusters = ThrusterSet(model)
    if not thrusters:
        print(f"error: {scene.name} declares no thrusters.", file=sys.stderr)
        return 1
    body_id = thrusters.body_id()

    # The driver needs one uniform way to command an altitude, because each
    # law keeps its setpoint somewhere different. A module that defines
    # set_target() can be driven; one that does not cannot.
    mod_name = args.controller.split(":")[0].rsplit(".", 1)[0]
    mod = sys.modules.get(mod_name)
    setter = getattr(mod, "set_target", None) if mod else None
    if setter is None:
        print(f"error: {mod_name!r} defines no set_target(alt_m), so this "
              f"scenario cannot command it an altitude.\n"
              f"  Add to {mod_name}.py:\n"
              f"      def set_target(alt_m):\n"
              f"          global TARGET_ALT_M\n"
              f"          TARGET_ALT_M = float(alt_m)\n"
              f"  See scripts/example_hover.py:set_target.", file=sys.stderr)
        return 1
    # Suppress any stdin listener: this is a scripted, non-interactive run.
    if mod is not None and hasattr(mod, "_listener"):
        mod._listener = False

    # Clear any controller state left from a previous run in this process -
    # an integrator especially. Without this, the second run of a gain sweep
    # starts with the first run's accumulator and only run one is honest.
    resetter = getattr(mod, "reset_state", None) if mod else None
    if resetter is not None:
        resetter()

    # Gains. Unlike set_target this is OPTIONAL: a controller without
    # set_gains is still perfectly runnable, it just cannot be retuned.
    try:
        wanted_gains = parse_gains(args.gains)
        wanted_gains.update(parse_gains(args.gain))
    except ValueError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    gain_setter = getattr(mod, "set_gains", None) if mod else None
    if wanted_gains:
        if gain_setter is None:
            print(f"error: {mod_name!r} defines no set_gains(**kw), so "
                  f"--gains cannot be applied.\n"
                  f"  See scripts/example_hover.py:set_gains.",
                  file=sys.stderr)
            return 1
        try:
            gain_setter(**wanted_gains)
        except (KeyError, ValueError, TypeError) as e:
            print(f"error: bad --gains: {e}", file=sys.stderr)
            return 1
    applied_gains = (mod.current_gains()
                     if mod is not None and hasattr(mod, "current_gains")
                     else None)

    # How the driver reads the tracker's internals, if the law publishes
    # them. A law without valve_state() still flies - it just logs zeros -
    # so this is optional in the same way set_gains is.
    _vs = getattr(mod, "valve_state", None) if mod else None
    valve_reader = _vs if callable(_vs) else (lambda: (0.0, 0.0, 0.0, 0.0))

    # And how it TELLS the law about contact. The sim knows (data.ncon), a
    # real airframe knows (a leg switch or a load cell), and a guess
    # reconstructed from acceleration cannot tell the pad from steady
    # cruise - both read a ~= 0 at vehicle weight. Measured: inferring it
    # froze the valve at u=0.5812 mid-climb and flew a 2 m profile to 61 m.
    _sg = getattr(mod, "set_grounded", None) if mod else None
    ground_teller = _sg if callable(_sg) else (lambda flag: None)

    dt = float(model.opt.timestep)
    qi, vi = vertical_indices(model, mujoco, body_id)

    # The tank owns the vehicle's mass, so build it BEFORE deriving the
    # ground height - a heavier craft settles deeper into the soft contact,
    # so the resting height depends on the mass that will actually fly.
    tank = None
    if not args.fixed_mass:
        from propellant import (PropellantTank, DRY_MASS_KG, PROPELLANT_KG,
                                ISP_S)
        tank = PropellantTank(
            model, body_id,
            dry_mass_kg=args.dry_mass if args.dry_mass is not None else DRY_MASS_KG,
            propellant_kg=args.propellant if args.propellant is not None else PROPELLANT_KG,
            isp_s=args.isp if args.isp is not None else ISP_S)

    # The valve layer. Built after the tank because it borrows the tank's
    # Isp for its blowdown mdot - the gas it spends has to match the impulse
    # it delivers, and inventing a second Isp would let the two disagree.
    valve = None
    if args.valve:
        import valve as valve_mod
        law = args.controller.split(":")[-1]
        # A law that returns NEWTONS, flown through the valve, has its 19.44
        # read as a valve opening, clamped to 1.0, and the engine sits wide
        # open for the whole flight. The failure looks like a wild controller
        # rather than a units mismatch, so refuse it by name instead.
        wants_valve = getattr(mod, "VALVE_LAWS", ("hover_valve",))
        if law not in wants_valve:
            print(f"error: --valve needs a law that returns a 0..1 valve "
                  f"opening, and {law!r} returns Newtons.\n"
                  f"  Laws in {mod_name!r} built for the valve: "
                  f"{', '.join(wants_valve)}\n"
                  f"  Without --valve, {law!r} flies normally.",
                  file=sys.stderr)
            return 1
        try:
            valve = valve_mod.ValveActuator(
                model, body_id,
                max_thrust_n=thrusters[valve_mod.VALVE_THRUSTER].max_thrust_n,
                curve_name=args.valve_curve,
                feed_mode=args.valve_feed,
                tau_s=0.0 if args.valve_tau is None else args.valve_tau,
                tank_volume_l=args.valve_tank_l,
                isp_s=(tank.isp_s if tank is not None
                       else (args.isp if args.isp is not None else None)))
        except (KeyError, ValueError) as e:
            print(f"error: --valve: {e}", file=sys.stderr)
            return 1

    z_rest = ground_height(model, data, mujoco, body_id)
    place_on_ground(model, data, mujoco, body_id, z_rest)

    prof = Profile(z_rest, args.target, args.descent_rate, dt,
                   ground_sit_s=args.ground_sit,
                   hold_target_s=args.hold_target,
                   hold_ground_s=args.hold_ground,
                   ascent_rate=args.ascent_rate)
    logger = make_logger(ThrustLogger, thrusters.names,
                         valve=valve is not None)
    ctrl_every = max(1, round((1.0 / args.control_hz) / dt)) if args.control_hz else 1

    print(f"scene       {scene.name}")
    print(f"controller  {args.controller}")
    print(f"ground      z = {z_rest:.6f} m (derived)   ncon at start "
          f"{data.ncon}  vz {float(data.qvel[vi]):+.2e}")
    if tank is not None:
        print(tank.describe())
    else:
        print(f"mass        {float(model.body_mass[body_id]):.3f} kg, fixed "
              f"(--fixed-mass)")
    if valve is not None:
        print(valve.describe())
    if wanted_gains:
        print("gains       " + "  ".join(f"{k}={v}" for k, v in
                                          wanted_gains.items()))
    print(f"profile     {prof.describe()}")

    # The run directory exists BEFORE the flight, because --video streams
    # frames into it as the run happens rather than buffering 540 frames of
    # RGB in memory and writing them at the end.
    stamp = _dt.datetime.now().strftime(STAMP_FMT)
    fn = args.controller.replace(":", "_").replace(".", "_")
    name = f"{scene.stem}_{fn}_{stamp}" + (f"_{args.label}" if args.label else "")
    parent = pathlib.Path(args.outdir) if args.outdir else (ROOT / "outputs")
    rundir = run_directory(parent, name)

    rec = None
    if args.video:
        try:
            rec = Recorder(mujoco, model, rundir / "flight.mp4", dt,
                           fps=args.video_fps, width=vid_w, height=vid_h,
                           camera=args.camera)
        except Exception as e:
            # A missing ffmpeg or an unavailable GL backend must not cost the
            # run: the CSV and metrics are the point, the video is a bonus.
            print(f"video       UNAVAILABLE ({type(e).__name__}: {e})")
            rec = None
        else:
            fell_back = rec.camera_name != rec.camera_requested
            print(f"video       {vid_w}x{vid_h} @ {args.video_fps} fps from "
                  f"camera {rec.camera_name!r}, every {rec.every} steps"
                  + (f"   <-- no {rec.camera_requested!r} camera in this scene"
                     if fell_back else ""))

    viewer = None
    if args.view:
        try:
            import mujoco.viewer
            viewer = mujoco.viewer.launch_passive(model, data)
        except Exception as e:
            print(f"viewer      UNAVAILABLE ({type(e).__name__}: {e})")
            print("            (needs a display; --video works headless)")
            viewer = None
        else:
            print(f"viewer      live at {args.view_speed:g}x real time; "
                  f"close the window to stop early")
    print()

    # The budget has to grow with the profile: a 60 s hover asked for from
    # the GUI must not be cut off by a ceiling computed from the defaults.
    max_wall = (prof.ground_sit_s + prof.hold_target_s + prof.hold_ground_s
                + prof._timeout_for("ASCEND") + prof._timeout_for("DESCEND") + 5.0)
    vz_prev = float(data.qvel[vi])
    step_i = 0
    last_phase = prof.phase

    # --view only. The profile is driven by SIM time, so pacing is purely a
    # playback concern: sleep until the wall clock catches up, and sync the
    # window at ~60 Hz rather than every 2 ms tick. Importantly the loop is
    # still stepped by this scenario, not by the viewer, so the run is
    # identical to a headless one - the viewer only watches.
    import time as _time
    paced = viewer is not None
    wall_start = _time.perf_counter()
    next_sync = 0.0
    sync_interval = 1.0 / 60.0

    try:
        while not prof.done and data.time < max_wall:
            if viewer is not None and not viewer.is_running():
                print("  viewer closed - stopping early")
                break
            z = float(data.xpos[body_id][2])
            vz = float(data.qvel[vi])
            setpoint = prof.update(data.time, z, vz)
            setter(setpoint)

            # Contact, measured rather than inferred. Both conditions are
            # needed: ncon alone is true for a glancing touch during a
            # bounce, and a slow vz alone is true at the top of a climb.
            # Resting means touching AND not moving.
            if valve is not None:
                ground_teller(data.ncon > 0 and abs(vz) < GROUND_VZ_MPS)

            if step_i % ctrl_every == 0:
                state = control_tick(mujoco, model, data, thrusters, body_id,
                                     controller, None, valve=valve)
            else:
                state = CraftState.from_data(model, data, body_id)

            # *** ORDER MATTERS HERE, three ways. ***
            #
            # 1. The valve advances at PHYSICS rate, like the tank: its lag
            #    and its blowdown are plant, not control. The zero-order hold
            #    belongs on the POSITION the controller commanded, not on the
            #    force, so the thrust follows the falling feed smoothly
            #    instead of stair-stepping at the control rate.
            # 2. refresh() must run BEFORE tank.update(), because that reads
            #    np.sum(np.abs(data.ctrl)) *as Newtons*. On a non-control tick
            #    data.ctrl would otherwise still hold the last conversion,
            #    and on any tick a raw 0..1 opening left there would
            #    under-report the burn by a factor of max_thrust (~60x).
            # 3. refresh() must run AFTER update(), so the force written is
            #    the one for the position and feed the vehicle has NOW.
            if valve is not None:
                if tank is not None:
                    # One source of truth for how much gas is left: in
                    # from_tank mode the curve is scaled by the sim's own
                    # tank rather than by a second blowdown model.
                    valve.set_tank_fraction(tank.fraction_remaining)
                valve.update(data, dt)
                valve.refresh(data, thrusters)
                u_valve, _a_valve, f_demand, f_valve = valve_reader()
                valve.note_starved(data.time, f_demand)

            # Propellant depletion runs at PHYSICS rate, not control rate: mass
            # leaves the vehicle continuously while the thrust is applied, not
            # only on the ticks the control law happens to run.
            if tank is not None:
                tank.enforce_cutoff(data)   # a dry tank produces nothing
                tank.update(data, dt)       # mdot -> mass + inertia + mj_setConst

            accel = (vz - vz_prev) / dt
            logger.record_tick(state, thrusters.magnitudes(data), setpoint, accel,
                               float(data.qacc[vi]), last_phase, list(data.ctrl),
                               float(model.body_mass[body_id]),
                               tank.propellant_kg if tank is not None else 0.0,
                               valve_row=None if valve is None else
                               (u_valve, f_demand, f_valve,
                                valve.last_thrust_n, valve.feed_fraction()))
            vz_prev = vz

            if prof.phase != last_phase:
                print(f"  t={data.time:6.2f}s  {last_phase:>11s} -> "
                      f"{prof.phase:<11s}  z={z:7.4f}  vz={vz:+7.4f}")
                last_phase = prof.phase

            # Capture BEFORE stepping, so frame 0 is the start of the run
            # rather than one tick into it.
            if rec is not None:
                rec.maybe_frame(data, step_i)

            mujoco.mj_step(model, data)
            step_i += 1

            if paced:
                wall = _time.perf_counter() - wall_start
                if wall >= next_sync:
                    viewer.sync()
                    next_sync = wall + sync_interval
                ahead = data.time / max(args.view_speed, 1e-9) - wall
                if ahead > 0:
                    _time.sleep(min(ahead, sync_interval))
    finally:
        # Always close both, even on a timeout or an exception: an unclosed
        # imageio writer leaves a truncated, unplayable mp4.
        if rec is not None:
            rec.close()
        if viewer is not None:
            viewer.close()

    # ---- outputs ---------------------------------------------------------
    # The directory was created before the flight (the recorder writes into
    # it), so this just uses it.

    cols = logger.header
    csv_path = logger.save(rundir / "flight.csv")

    main_max = thrusters["main"].max_thrust_n if "main" in thrusters.names else 0.0
    m = metrics(logger.rows, cols, prof, args.target, z_rest,
                args.descent_rate, main_max, tank, valve)

    meta = {
        "scene": scene.name, "controller": args.controller,
        "timestamp": stamp, "timestep_s": dt,
        "profile": {
            "target_alt_m": args.target,
            "descent_rate_mps": args.descent_rate,
            "ground_sit_s": prof.ground_sit_s,
            "hold_at_target_s": prof.hold_target_s,
            "hold_at_ground_s": prof.hold_ground_s,
            "ascent_rate_mps": prof.ascent_rate,
            "shape": prof.describe(),
            "arrive_tol_m": ARRIVE_TOL_M,
            "settle_tol_mps": SETTLE_TOL_MPS,
            "phase_timeout_s": PHASE_TIMEOUT_S,
        },
        "ground_height_m": z_rest,
        "gains": applied_gains,
        "gains_overridden": wanted_gains or None,
        "propellant": None if tank is None else {
            "dry_mass_kg": tank.dry_mass_kg,
            "propellant_initial_kg": tank.propellant_initial_kg,
            "isp_s": tank.isp_s,
        },
        # Recorded so a sweep's runs can be told apart afterwards: the whole
        # point of the rig is that ONE controller meets a family of plants,
        # and without this a run cannot say which member it flew.
        "valve": None if valve is None else {
            "curve": valve.curve_name,
            "feed_mode": valve.feed_mode,
            "max_thrust_n": valve.max_thrust_n,
            "tau_s": valve.tau_s,
            "tank_volume_l": valve.tank_volume_m3 * 1e3,
            "tank_sized_from_engine": valve.tank_scaled,
            "reg_setpoint_psig": valve.p_set_psig,
            # Two windows, because they are different numbers and quoting
            # one as "the" window is how I previously compared 29.5 s
            # against 12.5 s and called it a discrepancy. Wide open burns
            # fastest; the part-open hold point is where a flight sits.
            #
            # The part-open figure is quoted by FLOW, not by valve position:
            # VALVE_HOLD_POSITION is a flow fraction, and feeding 0.425 to
            # regulated_window_s() on the ball curve lands in the dead zone
            # and reports `inf`, which a real run did.
            "regulated_window_s": valve.regulated_window_s(),
            "regulated_window_at_hold_s": valve.window_at_flow(),
        },
        "phase_boundaries": [{"t": t, "from": a, "to": b}
                             for t, a, b in prof.boundaries],
        "metrics": m, "ticks": len(logger), "completed": prof.done,
        "video": None if rec is None else {
            "file": rec.path.name, "fps": rec.fps, "frames": rec.frames,
            "camera": rec.camera_name,
            "size": [vid_w, vid_h],
        },
        "mujoco": mujoco.__version__,
    }
    (rundir / "run.json").write_text(json.dumps(meta, indent=2))

    lines = [f"scenario run  {name}", f"scene         {scene.name}",
             f"controller    {args.controller}",
             f"completed     {prof.done}" +
             (f"   TIMEOUTS: {', '.join(prof.timeouts)}" if prof.timeouts else ""),
             ""]
    for k, v in m.items():
        lines.append(f"  {k:<30s} {v}" if not isinstance(v, float)
                     else f"  {k:<30s} {v:.6f}")
    (rundir / "summary.txt").write_text("\n".join(lines) + "\n")

    if not args.no_plot:
        # Which gains to caption with: the ones the law actually reads,
        # derived from its own code. Falls back to every applied gain if the
        # lookup fails, so a caption is never silently empty.
        try:
            _used = gains_used_by(args.controller.split(":")[-1], mod_name)
        except Exception:
            _used = None
        _cfg = config_lines(
            controller=args.controller,
            scene=scene.name,
            gains=applied_gains,
            overridden=wanted_gains,
            # From the Profile, so a custom one is CAPTIONED as custom.
            # This used to name only the target and the descent rate, which
            # captioned a 1s/8s/1.5s profile exactly like the default.
            profile=(f"{z_rest:.3f} -> {args.target:g} m; "
                     + prof.describe()),
            mass_note=(f"{float(model.body_mass[body_id]):.3f} kg fixed"
                       if tank is None else
                       f"{tank.dry_mass_kg:g} + {tank.propellant_initial_kg:g} kg, "
                       f"Isp {tank.isp_s:g} s"),
            metrics=m,
            gain_names=_used,
        )
        plot_flight(rundir / "flight.png", logger.rows, cols, prof,
                    args.target, z_rest, args.descent_rate, thrusters.names,
                    show_mass=tank is not None,
                    all_thrusters=args.all_thrusters,
                    show_valve=valve is not None,
                    config=None if args.no_config_panel else _cfg)
        _, _hidden = active_thrusters(logger.rows, cols, thrusters.names)
        if _hidden and not args.all_thrusters:
            print(f"plot         left off {len(_hidden)} thruster(s) that "
                  f"never fired: {', '.join(_hidden)}"
                  f"  (--all-thrusters to include them)")

    print()
    if prof.timeouts:
        print(f"TIMED OUT in: {', '.join(prof.timeouts)}")
    print(f"peak {m['peak_altitude_m']:.4f} m  overshoot {m['overshoot_m']:.4f} m  "
          f"hold RMS err {m['hold_rms_err_m']:.5f} m")
    print(f"peak descent {m['peak_descent_rate_mps']:+.4f} m/s "
          f"(commanded {m['commanded_descent_rate_mps']:+.4f})")
    print(f"returned to {m['final_altitude_m']:.4f} m "
          f"(ground {z_rest:.4f}, err {m['return_err_m']:.5f})")
    if rec is not None and rec.path.exists():
        print(f"video        {rec.frames} frames -> {rec.path.name} "
              f"({rec.path.stat().st_size / 1e6:.1f} MB)")
    print(f"\nwrote {len(logger)} ticks -> {rundir}")
    return 0




# =======================================================================
# SELF-TEST
# =======================================================================
def run_selftest() -> int:
    """Numeric checks on the profile machine, in selftest_thrust.py's style.

        uv run python scripts/scenario.py --selftest
    """
    import subprocess
    import inspect as _inspect_mod
    import numpy as _np
    from thrusters import ThrusterSet as _ThrusterSet
    bar = "=" * 74
    print(bar)
    print("SCENARIO SELF-TEST")
    print(bar)

    passed, failed = 0, []

    def ok(label, got, want, tol, unit=""):
        nonlocal passed
        good = abs(got - want) <= tol
        mark = "ok  " if good else "FAIL"
        print(f"  [{mark}] {label:<44} {got:10.5f} (want {want:.5f} +-{tol:g}){unit}")
        if good:
            passed += 1
        else:
            failed.append(label)

    def truth(label, cond, detail=""):
        nonlocal passed
        mark = "ok  " if cond else "FAIL"
        print(f"  [{mark}] {label:<44} {detail}")
        if cond:
            passed += 1
        else:
            failed.append(label)

    print("\n[1] the phase machine, driven by a perfect tracker")
    # Feed the machine exactly what it asks for, so only its own logic is
    # under test - no controller, no physics.
    dt = 0.002
    prof = Profile(0.4, 2.0, 0.1, dt)
    t, z, seen, guard = 0.0, 0.4, [], 0
    while not prof.done and guard < 200000:
        sp = prof.update(t, z, 0.0)
        z = sp                      # a tracker that arrives instantly
        if not seen or seen[-1][0] != prof.phase:
            seen.append((prof.phase, t))
        t += dt
        guard += 1
    order = [p for p, _ in seen]
    truth("phases run in order", order[:5] == list(PHASES) + [] or
          order == list(PHASES) + ["DONE"], "->".join(order))
    truth("no timeouts with a perfect tracker", not prof.timeouts,
          str(prof.timeouts))
    ok("total profile time", t, GROUND_SIT_S + HOLD_AT_TARGET_S
       + HOLD_AT_GROUND_S + (2.0 - 0.4) / 0.1 + 3 * dt, 0.05, " s")

    print("\n[2] ASCEND does not exit on the stale setpoint")
    # The bug this guards: arrival was tested against self.setpoint, which on
    # the first ASCEND tick is still the ground value, so a craft sitting on
    # the floor tested as 'arrived at the target' and the phase lasted 0 s.
    prof2 = Profile(0.4, 2.0, 0.1, dt)
    t2 = 0.0
    while prof2.phase == "GROUND_SIT":
        prof2.update(t2, 0.4, 0.0); t2 += dt
    n_ascend = 0
    while prof2.phase == "ASCEND" and n_ascend < 100:
        prof2.update(t2, 0.4, 0.0)   # still on the ground, not arriving
        t2 += dt; n_ascend += 1
    truth("ASCEND survives >=100 ticks on the ground", n_ascend >= 100,
          f"{n_ascend} ticks")

    print("\n[3] a phase that never arrives times out")
    prof3 = Profile(0.4, 2.0, 0.1, dt)
    t3, guard = 0.0, 0
    while not prof3.done and guard < 400000:
        prof3.update(t3, 0.4, 0.0)   # never leaves the ground
        t3 += dt; guard += 1
    truth("run completes anyway", prof3.done)
    truth("ASCEND recorded as timed out", "ASCEND" in prof3.timeouts,
          str(prof3.timeouts))

    print("\n[4] the descent ramp honours its rate")
    prof4 = Profile(0.4, 2.0, 0.5, dt)
    prof4.phase, prof4.setpoint = "DESCEND", 2.0
    n, z4 = 0, 2.0
    while prof4.phase == "DESCEND" and n < 100000:
        z4 = prof4.update(n * dt, z4, 0.0)   # perfect tracker follows the ramp
        n += 1
    ok("ramp duration at 0.5 m/s", n * dt, (2.0 - 0.4) / 0.5, 0.05, " s")

    print("\n[5] variable mass")
    import mujoco as _mj
    import numpy as _np
    from propellant import PropellantTank, G0_MS2

    # (a) F=ma must hold after a mass change, which needs mj_setConst.
    def _accel(mass, ctrl=30.0, setconst=True):
        mm = _mj.MjModel.from_xml_path(str(ROOT / "scenes" / "07_thrust_1D.xml"))
        bid = _ThrusterSet(mm).body_id()
        mm.opt.gravity[:] = 0.0
        d = _mj.MjData(mm); d.qpos[0] = 5.0
        mm.body_mass[bid] = mass
        if setconst:
            _mj.mj_setConst(mm, d)
        d.ctrl[0] = ctrl
        _mj.mj_forward(mm, d)
        return float(d.qacc[0])

    for mass in (12.0, 6.0, 3.0, 1.5):
        ok(f"  F=ma at {mass:.1f} kg", _accel(mass), 30.0 / mass, 1e-9, " m/s^2")

    # (b) THE TRAP: without mj_setConst the mass matrix stays stale, so the
    # acceleration does not change at all. Pinned so a regression that drops
    # the call fails loudly instead of returning a plausible wrong number.
    truth("  no mj_setConst -> acceleration unchanged (the trap)",
          abs(_accel(6.0, setconst=False) - _accel(12.0, setconst=False)) < 1e-12,
          f"{_accel(6.0, setconst=False):.5f} vs {_accel(12.0, setconst=False):.5f}")

    # (c) THE OTHER TRAP: mj_setConst resets qpos to the model defaults, so
    # update() must save and restore it or the craft teleports to its spawn
    # every tick (seen as a frozen position while velocity climbs).
    mm = _mj.MjModel.from_xml_path(str(ROOT / "scenes" / "07_thrust_1D.xml"))
    bid = _ThrusterSet(mm).body_id()
    d = _mj.MjData(mm)
    tk = PropellantTank(mm, bid, dry_mass_kg=8.0, propellant_kg=4.0, isp_s=57.80)
    d.qpos[0] = 1.234; d.qvel[0] = 5.0
    _mj.mj_forward(mm, d)
    d.ctrl[0] = 20.0
    tk.update(d, 0.002)
    ok("  update() preserves qpos", float(d.qpos[0]), 1.234, 1e-12, " m")

    # (d) inertia scales with mass, which mj_setConst does NOT do for you.
    mm = _mj.MjModel.from_xml_path(str(ROOT / "scenes" / "06_thrust_craft.xml"))
    bid = _ThrusterSet(mm).body_id()
    I_wet = float(mm.body_inertia[bid][0])
    d = _mj.MjData(mm)
    tk = PropellantTank(mm, bid, dry_mass_kg=6.0, propellant_kg=6.0, isp_s=57.80)
    d.qpos[2] = 5.0
    _mj.mj_forward(mm, d)
    d.ctrl[0] = 60.0
    for _ in range(2000):
        tk.update(d, 0.002)
        _mj.mj_step(mm, d)
    ratio_m = tk.mass_kg / 12.0
    ok("  inertia ratio tracks mass ratio",
       float(mm.body_inertia[bid][0]) / I_wet, ratio_m, 1e-9)

    # (e) the rocket equation, end to end in zero-g.
    mm = _mj.MjModel.from_xml_path(str(ROOT / "scenes" / "07_thrust_1D.xml"))
    bid = _ThrusterSet(mm).body_id()
    mm.opt.gravity[:] = 0.0
    d = _mj.MjData(mm); d.qpos[0] = 50.0
    tk = PropellantTank(mm, bid, dry_mass_kg=8.0, propellant_kg=4.0, isp_s=57.80)
    _mj.mj_setConst(mm, d); _mj.mj_forward(mm, d)
    m_start = tk.mass_kg
    while not tk.empty:
        d.ctrl[0] = 40.0
        tk.enforce_cutoff(d)
        tk.update(d, float(mm.opt.timestep))
        _mj.mj_step(mm, d)
    ve = 57.80 * G0_MS2
    import math as _math
    ok("  delta-v vs ve*ln(m0/m1)", float(d.qvel[0]),
       ve * _math.log(m_start / tk.mass_kg), 0.05, " m/s")
    ok("  propellant fully consumed", tk.used_kg, 4.0, 1e-9, " kg")
    ok("  mass floors at dry", tk.mass_kg, 8.0, 1e-12, " kg")

    # (f) hard cutoff: a dry tank produces nothing, whatever is commanded.
    d.ctrl[0] = 30.0
    tk.enforce_cutoff(d)
    ok("  dry tank applies no thrust", float(d.ctrl[0]), 0.0, 1e-12, " N")
    truth("  propellant never negative", tk.propellant_kg >= 0.0,
          f"{tk.propellant_kg:.3e} kg")

    print("\n[5b] gain plumbing")
    import example_hover as _eh
    truth("parse_gains handles a comma string",
          parse_gains("KP_ALT=20,KI_ALT=4") == {"KP_ALT": "20", "KI_ALT": "4"})
    truth("parse_gains handles a repeated list",
          parse_gains(["KP_ALT=20", "KI_ALT=4"]) == {"KP_ALT": "20",
                                                     "KI_ALT": "4"})
    try:
        parse_gains("KP_ALT")
        truth("parse_gains rejects a missing '='", False)
    except ValueError:
        truth("parse_gains rejects a missing '='", True)

    _kp0, _kd0, _ki0 = _eh.KP_ALT, _eh.KD_ALT, _eh.KI_ALT
    _eh.set_gains(KP_ALT=20.0)
    truth("set_gains writes the constant", _eh.KP_ALT == 20.0)
    try:
        _eh.set_gains(KP_NOPE=1.0)
        truth("set_gains rejects an unknown name", False)
    except KeyError:
        truth("set_gains rejects an unknown name", True)
    # bool("0") is True, so a naive coercion left ANTI_WINDUP permanently on
    # and the guard could never be disabled from the CLI.
    _eh.set_gains(ANTI_WINDUP="0")
    truth("ANTI_WINDUP='0' parses as False", _eh.ANTI_WINDUP is False)
    _eh.set_gains(ANTI_WINDUP="true")
    truth("ANTI_WINDUP='true' parses as True", _eh.ANTI_WINDUP is True)
    # EVERY flag gain, not just the first one anybody thought of. The bug:
    # GROUND_FREEZE was added to _GAIN_NAMES but the coercion special-cased
    # ANTI_WINDUP by name, so `--gain GROUND_FREEZE=False` died with
    # "could not convert string to float: 'False'". The gain was listed,
    # documented and settable in name only. Derived from _BOOL_GAINS so the
    # next flag gain is covered the moment it is added to that tuple.
    for _bg in _eh._BOOL_GAINS:
        _was = getattr(_eh, _bg)
        try:
            _eh.set_gains(**{_bg: "False"})
            _ok = getattr(_eh, _bg) is False
            _eh.set_gains(**{_bg: "True"})
            _ok = _ok and getattr(_eh, _bg) is True
            truth(f"  {_bg} accepts True/False from the CLI", _ok)
        except (TypeError, ValueError) as e:
            truth(f"  {_bg} accepts True/False from the CLI", False,
                  f"{type(e).__name__}: {e}")
        _eh.set_gains(**{_bg: _was})
    # A flag gain must not be left out of _BOOL_GAINS: float() on "False"
    # raises, so the symptom is a crash, but only for whoever tries to set
    # it. Checked by TYPE, which is what distinguishes the two coercions.
    _flags = tuple(n for n in _eh._GAIN_NAMES
                   if isinstance(getattr(_eh, n, None), bool))
    truth("every boolean-valued gain is listed in _BOOL_GAINS",
          set(_flags) == set(_eh._BOOL_GAINS),
          f"bool-valued {_flags}, declared {_eh._BOOL_GAINS}")
    _eh._integral = 5.0
    _eh.set_gains(KP_ALT=_kp0)
    truth("set_gains clears the integrator", _eh._integral == 0.0)
    _eh._integral = 5.0
    _eh.reset_state()
    truth("reset_state clears the integrator and setpoint history",
          _eh._integral == 0.0 and _eh._sp_prev is None)
    _eh.set_gains(KP_ALT=_kp0, KD_ALT=_kd0, KI_ALT=_ki0)

    print("\n[6] end to end on both scenes")
    for scene, z_expect in (("scenes/07_thrust_1D.xml", 0.399928),
                            ("scenes/06_thrust_craft.xml", 0.249977)):
        out = ROOT / "out" / "_selftest_scenario"
        r = subprocess.run(
            [sys.executable, str(HERE / "scenario.py"),
             "--controller", "example_hover:hover", "--scene", scene,
             "--no-plot", "--outdir", str(out)],
            capture_output=True, text=True, timeout=900)
        truth(f"{pathlib.Path(scene).stem} exits 0", r.returncode == 0,
              (r.stderr or "").strip()[-80:])
        runs = sorted(out.glob("*"), key=lambda p: p.stat().st_mtime)
        meta = json.loads((runs[-1] / "run.json").read_text())
        m = meta["metrics"]
        truth(f"  {pathlib.Path(scene).stem} completed, no timeouts",
              meta["completed"] and not m["timeouts"])
        ok(f"  {pathlib.Path(scene).stem} ground height derived",
           m["ground_height_m"], z_expect, 1e-4, " m")
        ok(f"  {pathlib.Path(scene).stem} returns to start",
           m["return_err_m"], 0.0, 0.01, " m")
        ok(f"  {pathlib.Path(scene).stem} peak descent rate",
           m["peak_descent_rate_mps"], -0.1, 0.025, " m/s")
        # The P+D structural lag against a ramp: e = -KD*rate/KP.
        import example_hover as eh
        ok(f"  {pathlib.Path(scene).stem} descent lag vs -KD*r/KP",
           m["descent_lag_mean_m"], -eh.KD_ALT * 0.1 / eh.KP_ALT, 0.02, " m")

    print("\n[7] the new laws do what they claim")
    out7 = ROOT / "out" / "_selftest_laws"

    def fly(law, extra=(), target=None, scene="scenes/07_thrust_1D.xml"):
        argv = [sys.executable, str(HERE / "scenario.py"),
                "--controller", f"example_hover:{law}", "--scene", scene,
                "--fixed-mass", "--no-plot", "--outdir", str(out7)]
        if target is not None:
            argv += ["--target", str(target)]
        argv += list(extra)
        r = subprocess.run(argv, capture_output=True, text=True, timeout=900)
        if r.returncode != 0:
            return None
        newest = max(out7.glob("*/run.json"), key=lambda q: q.stat().st_mtime)
        return json.loads(newest.read_text())["metrics"]

    # The PID and the cascade both exist to kill the PD's ramp lag.
    m_pd = fly("hover")
    m_pid = fly("hover_pid")
    m_casc = fly("hover_cascade")
    truth("all three laws complete the profile",
          all(m is not None for m in (m_pd, m_pid, m_casc)))
    ok("  PD ramp lag (the baseline)", m_pd["descent_lag_mean_m"],
       -0.1222, 0.02, " m")
    truth("  PID cuts the ramp lag at least 3x",
          abs(m_pid["descent_lag_mean_m"]) < abs(m_pd["descent_lag_mean_m"]) / 3,
          f"{m_pid['descent_lag_mean_m']:+.5f} vs {m_pd['descent_lag_mean_m']:+.5f} m")
    truth("  cascade cuts it at least 10x",
          abs(m_casc["descent_lag_mean_m"]) < abs(m_pd["descent_lag_mean_m"]) / 10,
          f"{m_casc['descent_lag_mean_m']:+.5f} m")
    truth("  cascade keeps PD-like overshoot",
          m_casc["overshoot_m"] < 0.10,
          f"{m_casc['overshoot_m']:.5f} m")

    # ANTI-WINDUP IS LOAD-BEARING. The disabled arm must stay bad, or this
    # check is not testing anything. I_LIMIT is raised out of the way so
    # ANTI_WINDUP is the only guard in play.
    big = 12.0
    m_aw = fly("hover_pid", ["--gains", "ANTI_WINDUP=1,I_LIMIT_N=1000"], big)
    m_no = fly("hover_pid", ["--gains", "ANTI_WINDUP=0,I_LIMIT_N=1000"], big)
    truth("  anti-windup on: overshoot under 6 m",
          m_aw["overshoot_m"] < 6.0, f"{m_aw['overshoot_m']:+.3f} m")
    truth("  anti-windup OFF: overshoot over 10 m (must stay bad)",
          m_no["overshoot_m"] > 10.0, f"{m_no['overshoot_m']:+.3f} m")
    truth("  so the guard is worth >5 m of overshoot",
          m_no["overshoot_m"] - m_aw["overshoot_m"] > 5.0,
          f"{m_no['overshoot_m'] - m_aw['overshoot_m']:+.3f} m")

    # The cascade's gains must not be tuned to one vehicle.
    m_smol = fly("hover_cascade", scene="scenes/08_thrust_1D_smol.xml")
    truth("  cascade also converges on 08_smol (1 kg, 10 N)",
          m_smol is not None and m_smol["return_err_m"] < 0.02,
          f"return err {m_smol['return_err_m']:.5f} m" if m_smol else "FAILED")

    # Gains must be recoverable from a run's own output.
    newest = max(out7.glob("*/run.json"), key=lambda q: q.stat().st_mtime)
    meta = json.loads(newest.read_text())
    truth("  run.json records the gains it flew",
          isinstance(meta.get("gains"), dict) and "KP_ALT" in meta["gains"])

    # BOTH stateful laws must behave at a DECIMATED control rate. They carry
    # per-call state (an integrator, a differenced setpoint), and anything
    # that divides or accumulates over "a tick" has to use the REAL interval
    # between its own calls, not the physics timestep. Before this was
    # fixed, the cascade's setpoint-rate estimate was high by the decimation
    # factor, which pinned v_cmd at the V_MAX_MPS clamp and drove a
    # commanded 0.1 m/s descent at 0.42 m/s at 50 Hz.
    print("  rate independence (--control-hz)")
    for law, tol in (("hover_cascade", 0.06), ("hover_pid", 0.06)):
        at = {}
        for hz in (None, 50, 10):
            extra = () if hz is None else ("--control-hz", str(hz))
            at[hz] = fly(law, extra)
        truth(f"  {law} runs at every rate",
              all(v is not None for v in at.values()))
        if all(v is not None for v in at.values()):
            # The commanded descent must be honoured, not just "a descent".
            worst = max(abs(v["peak_descent_rate_mps"]) for v in at.values())
            truth(f"  {law} honours the 0.1 m/s descent at 10-500 Hz",
                  worst < 0.1 + tol,
                  "  ".join(f"{k or 500}Hz {v['peak_descent_rate_mps']:+.4f}"
                            for k, v in at.items()))
            # And the ramp lag must not scale with the rate, which is what
            # an integrator accumulating a nominal dt would do.
            lags = [v["descent_lag_mean_m"] for v in at.values()]
            truth(f"  {law} ramp lag is rate-independent",
                  max(lags) - min(lags) < 0.03,
                  "  ".join(f"{x:+.5f}" for x in lags))
    # The timing helper itself: it must not trust a bad timestamp.
    _eh._t_prev = None
    truth("  _tick_dt falls back to nominal on the first call",
          _eh._tick_dt(1.0) == _eh.DT_NOMINAL_S)
    truth("  _tick_dt then measures the real interval",
          abs(_eh._tick_dt(1.02) - 0.02) < 1e-12)
    truth("  _tick_dt refuses a repeated or rewound timestamp",
          _eh._tick_dt(1.02) == _eh.DT_NOMINAL_S
          and _eh._tick_dt(0.5) == _eh.DT_NOMINAL_S)
    _eh.reset_state()
    truth("  reset_state clears the timestamp too", _eh._t_prev is None)

    print("\n[7c] the full-PID cascade")
    out7c = ROOT / "out" / "_selftest_pidcascade"

    # *** THE LOAD-BEARING CHECK. *** All four new gains default to 0, so the
    # law must BE hover_cascade. Asserted byte-for-byte rather than on
    # metrics: that makes it a strict superset, so any difference measured
    # later is attributable to the gain turned on and not to the law change.
    def cascade_dir(law, gains=None, target="2.0"):
        """Either cascade law, same profile and outdir, newest dir back."""
        argv = [sys.executable, str(HERE / "scenario.py"),
                "--controller", f"example_hover:{law}",
                "--scene", "scenes/07_thrust_1D.xml", "--target", target,
                "--fixed-mass", "--no-plot", "--outdir", str(out7c)]
        if gains:
            argv += ["--gains", gains]
        r = subprocess.run(argv, capture_output=True, text=True, timeout=900)
        if r.returncode != 0:
            return None, r
        return max(out7c.glob("*/"), key=lambda q: q.stat().st_mtime), r

    d_cp, r_cp = cascade_dir("hover_cascade_pid")
    d_base, _ = cascade_dir("hover_cascade")
    truth("a PID-cascade run completes", d_cp is not None,
          "" if d_cp else (r_cp.stderr or "")[-160:])
    if d_cp is not None and d_base is not None:
        import hashlib
        h_pid = hashlib.md5((d_cp / "flight.csv").read_bytes()).hexdigest()
        h_base = hashlib.md5((d_base / "flight.csv").read_bytes()).hexdigest()
        truth("  at zero I and D it IS hover_cascade, byte for byte",
              h_pid == h_base, f"{h_pid[:16]} vs {h_base[:16]}")

    # Each new gain must actually DO something - a gain that changes nothing
    # is a gain that is not wired up, which is the failure mode a superset
    # law invites.
    if d_cp is not None:
        x0 = json.loads((d_cp / "run.json").read_text())["metrics"]
        for gname, gval in (("KI_V", "5"), ("KI_POS_V", "1"),
                            ("KD_V", "2"), ("KD_POS_V", "1")):
            d_g, r_g = cascade_dir("hover_cascade_pid", f"{gname}={gval}")
            if d_g is None:
                truth(f"  {gname} run completes", False,
                      (r_g.stderr or "")[-120:])
                continue
            xg = json.loads((d_g / "run.json").read_text())["metrics"]
            truth(f"  {gname}={gval} changes the flight",
                  abs(xg["overshoot_m"] - x0["overshoot_m"]) > 1e-4
                  or abs(xg["hold_rms_err_m"] - x0["hold_rms_err_m"]) > 1e-5,
                  f"overshoot {xg['overshoot_m']:.4f} vs "
                  f"{x0['overshoot_m']:.4f}, RMS {xg['hold_rms_err_m']:.5f} "
                  f"vs {x0['hold_rms_err_m']:.5f}")
        # And the law must stay flyable with every term on at once.
        d_all, _ = cascade_dir("hover_cascade_pid",
                              "KI_V=5,KI_POS_V=0.2,KD_V=1,KD_POS_V=0.5")
        if d_all is not None:
            xa = json.loads((d_all / "run.json").read_text())["metrics"]
            truth("  all four gains on at once still flies",
                  not xa["timeouts"] and abs(xa["overshoot_m"]) < 0.5,
                  f"overshoot {xa['overshoot_m']:+.4f} m, "
                  f"timeouts {xa['timeouts']}")

    # WHAT KI_V IS FOR, benched rather than flown: the sim has no thrust-bias
    # knob, and this is the one claim that justifies the law existing.
    # Holding altitude against a bias the controller is never told about, P
    # alone needs a standing error of bias/(KP_V*KP_POS_V); the integral
    # removes it.
    import example_hover as _eh2

    class _S:
        def __init__(self, t, z, vz, mass=12.0):
            self.t, self.mass = t, mass
            self.pos = _np.array([0.0, 0.0, z])
            self.vel = _np.array([0.0, 0.0, vz])
            self.gravity = _np.array([0.0, 0.0, -1.62])
        def weight_n(self):
            return self.mass * 1.62

    def _hold(ki_v, bias_n, n=20000, dtb=0.002):
        _eh2.reset_state()
        _eh2.set_gains(KI_V=ki_v, KI_POS_V=0.0, KD_V=0.0, KD_POS_V=0.0)
        _eh2.set_target(2.0)
        z, vz, m, gg = 2.0, 0.0, 12.0, 1.62
        for i in range(n):
            th = float(_eh2.hover_cascade_pid(_S(i * dtb, z, vz))["main"][0])
            vz += (max(0.0, th + bias_n) / m - gg) * dtb
            z += vz * dtb
        return z

    _kp_eq = _eh2.KP_V * _eh2.KP_POS_V
    ok("P alone sits off by bias/(KP_V*KP_POS_V)", _hold(0.0, 2.0) - 2.0,
       2.0 / _kp_eq, 1e-3, " m")
    ok("  KI_V removes the offset entirely", _hold(5.0, 2.0) - 2.0,
       0.0, 1e-3, " m")
    ok("  and works on a negative bias too", _hold(5.0, -2.0) - 2.0,
       0.0, 1e-3, " m")
    _eh2.reset_state()

    # Anti-windup on BOTH loops, since a cascade gives an integral two places
    # to wind up. hover_pid's lesson: unguarded, the integral made that law
    # five times worse than the PD it was meant to improve.
    src_pid = _inspect_mod.getsource(_eh2.hover_cascade_pid)
    truth("both integrals are clamped",
          "I_LIMIT_V_MPS" in src_pid and "I_LIMIT_V_N" in src_pid)
    truth("  and neither accumulates while its output is saturated",
          src_pid.count("if not (") >= 2
          and "V_MAX_MPS" in src_pid and "THRUST_CEILING_N" in src_pid)
    # A big step is where windup shows. Guarded, it must stay sane.
    d_step, _ = cascade_dir("hover_cascade_pid", "KI_V=5,KI_POS_V=1",
                               target="12.0")
    if d_step is not None:
        xs2 = json.loads((d_step / "run.json").read_text())["metrics"]
        truth("  a 12 m step with both integrals on does not blow up",
              abs(xs2["overshoot_m"]) < 6.0,
              f"overshoot {xs2['overshoot_m']:+.4f} m "
              f"(unguarded hover_pid managed 15.2 m)")

    print("\n[7b] the two-rate cascade")
    # hover_cascade runs both loops in one call. hover_cascade_2rate runs
    # the OUTER loop at OUTER_HZ and holds its commanded rate between
    # updates, which is how real flight software is laid out. No harness
    # change was needed: the law decimates itself off state.t.
    import hashlib as _hl
    m_1r = fly("hover_cascade")
    m_2r_off = fly("hover_cascade_2rate", ["--gains", "OUTER_HZ=0"])
    truth("  OUTER_HZ=0 reproduces the single-rate law exactly",
          m_1r == m_2r_off,
          "metrics identical" if m_1r == m_2r_off else "DIFFER")
    # Equality of metrics is weaker than equality of trajectory, so compare
    # the CSVs: if the two code paths agree they must agree tick for tick.
    runs = sorted(out7.glob("*/flight.csv"), key=lambda q: q.stat().st_mtime)
    if len(runs) >= 2:
        h = [_hl.sha256(q.read_bytes()).hexdigest() for q in runs[-2:]]
        truth("  and byte-for-byte, not just metric-for-metric",
              h[0] == h[1], h[0][:16])

    # The rate RATIO is the thing this law exists to vary. Down to ~25 Hz
    # the held command is fresh enough that it tracks the single-rate law;
    # the point of the check is that a STALE outer loop is measurably worse,
    # or the law is not doing what it claims.
    m_25 = fly("hover_cascade_2rate", ["--gains", "OUTER_HZ=25"])
    m_2 = fly("hover_cascade_2rate", ["--gains", "OUTER_HZ=2"])
    truth("  25 Hz outer loop still tracks the single-rate law",
          abs(m_25["overshoot_m"] - m_1r["overshoot_m"]) < 0.01,
          f"{m_25['overshoot_m']:.5f} vs {m_1r['overshoot_m']:.5f} m")
    truth("  a stale 2 Hz outer loop is clearly worse",
          m_2["overshoot_m"] > 2 * m_1r["overshoot_m"],
          f"{m_2['overshoot_m']:.5f} vs {m_1r['overshoot_m']:.5f} m")
    truth("  but still completes the profile, not unstable",
          m_2["return_err_m"] < 0.05, f"return err {m_2['return_err_m']:.5f} m")

    # Scheduled on SIM TIME, not a call count. A counter would silently
    # mean a different frequency under --control-hz, so the outer period
    # must survive changing the inner rate.
    m_25_slow = fly("hover_cascade_2rate",
                    ["--gains", "OUTER_HZ=25", "--control-hz", "200"])
    truth("  the outer period holds when the inner rate changes",
          abs(m_25_slow["overshoot_m"] - m_25["overshoot_m"]) < 0.005,
          f"{m_25_slow['overshoot_m']:.5f} vs {m_25['overshoot_m']:.5f} m")
    # Degenerate case: the inner loop SLOWER than the outer period. The
    # outer loop then simply runs every call; it must not stall or spin.
    m_inv = fly("hover_cascade_2rate",
                ["--gains", "OUTER_HZ=25", "--control-hz", "10"])
    truth("  inner slower than outer degrades gracefully",
          m_inv is not None and m_inv["overshoot_m"] < 0.10,
          f"{m_inv['overshoot_m']:.5f} m" if m_inv else "FAILED")
    truth("  OUTER_HZ is attributed to the 2-rate law only",
          "OUTER_HZ" in gains_used_by("hover_cascade_2rate")
          and "OUTER_HZ" not in gains_used_by("hover_cascade"))
    _eh.reset_state()
    truth("  reset_state clears the held command and outer clock",
          _eh._outer_t is None and _eh._v_cmd_held == 0.0)

    print("\n[8] --video records the run without changing it")
    # The whole value of recording a run is that it IS the run. If rendering
    # perturbed the physics the video would be of something else, so this
    # compares a recorded run against a headless one bit for bit.
    out8 = ROOT / "out" / "_selftest_video"

    def fly_dir(law, extra=(), plot=False):
        argv = [sys.executable, str(HERE / "scenario.py"),
                "--controller", f"example_hover:{law}",
                "--scene", "scenes/07_thrust_1D.xml", "--target", "1.0",
                "--fixed-mass", "--outdir", str(out8)]
        if not plot:
            argv.append("--no-plot")
        argv += list(extra)
        r = subprocess.run(argv, capture_output=True, text=True, timeout=900)
        if r.returncode != 0:
            return None, r
        return max(out8.glob("*/"), key=lambda q: q.stat().st_mtime), r

    d_plain, _ = fly_dir("hover")
    d_video, r_video = fly_dir("hover", ["--video"])
    truth("a --video run completes", d_video is not None,
          "" if d_video else (r_video.stderr or "")[-160:])
    if d_video is not None:
        mp4 = d_video / "flight.mp4"
        truth("  flight.mp4 lands in the run directory", mp4.exists(),
              f"{mp4.stat().st_size / 1e6:.2f} MB" if mp4.exists() else "missing")
        meta_v = json.loads((d_video / "run.json").read_text())
        vid = meta_v.get("video") or {}
        truth("  run.json describes the video",
              vid.get("file") == "flight.mp4" and vid.get("frames", 0) > 100,
              f"{vid.get('frames')} frames @ {vid.get('fps')} fps, "
              f"camera {vid.get('camera')!r}")
        # The default camera must be the FIXED one. Filmed from "chase"
        # (mode="trackcom") the craft stays at the same screen position and
        # the altitude - the only thing the profile is about - is invisible.
        truth("  films from the fixed 'pad' camera, not the tracking one",
              vid.get("camera") == "pad", str(vid.get("camera")))
        # Frame cadence: 30 fps over a ~20 s profile, not one frame a tick.
        n_ticks = meta_v["ticks"]
        expect = n_ticks / round((1.0 / VIDEO_FPS) / meta_v["timestep_s"])
        truth("  frame count matches fps x duration",
              abs(vid.get("frames", 0) - expect) <= 2,
              f"{vid.get('frames')} vs {expect:.0f} expected")
        if d_plain is not None:
            meta_p = json.loads((d_plain / "run.json").read_text())
            truth("  metrics identical to the headless run",
                  meta_p["metrics"] == meta_v["metrics"])
            truth("  tick count identical",
                  meta_p["ticks"] == meta_v["ticks"],
                  f"{meta_p['ticks']} vs {meta_v['ticks']}")
            import hashlib
            h = [hashlib.sha256((d / "flight.csv").read_bytes()).hexdigest()
                 for d in (d_plain, d_video)]
            truth("  flight.csv byte-identical (recording cannot perturb)",
                  h[0] == h[1], h[0][:16])
        truth("  a run without --video records no video",
              json.loads((d_plain / "run.json").read_text())["video"] is None)
    # The camera falls back by name rather than failing on a scene with none.
    import mujoco as _mjv
    _mm = _mjv.MjModel.from_xml_path(str(ROOT / "scenes" / "07_thrust_1D.xml"))
    truth("camera resolves 'pad' when present",
          Recorder._resolve_camera(_mjv, _mm, "pad")[1] == "pad")
    truth("camera falls back to 'chase' when 'pad' is absent",
          Recorder._resolve_camera(_mjv, _mm, "nope")[1] == "chase")
    truth("camera falls back to the free camera when asked",
          Recorder._resolve_camera(_mjv, _mm, "free") == (-1, "free"))

    print("\n[9] the plotter hides thrusters that never fired")
    # Every thrust scene declares four RCS nozzles, and a vertical law never
    # touches them: measured, EXACTLY 0.0 N for the whole run. Four flat
    # lines on zero in two panels also drag the y-axis down to include zero,
    # which compresses the main engine's 17-30 N trace into a fifth of the
    # panel - so this is legibility, not just tidiness.
    _cols = ["time_s", "main_N", "rcs_px_N", "ctrl_main", "ctrl_rcs_px"]
    _rows = [[0.0, 19.4, 0.0, 19.4, 0.0], [0.1, 20.1, 0.0, 20.1, 0.0]]
    keep, drop = active_thrusters(_rows, _cols, ["main", "rcs_px"])
    truth("drops a thruster that is zero throughout",
          keep == ["main"] and drop == ["rcs_px"], f"kept {keep}")
    # A nozzle that fires ONCE must survive - the test is "did it fire",
    # not "is it called rcs", so a law that uses the RCS still sees it.
    _rows2 = [[0.0, 19.4, 0.0, 19.4, 0.0], [0.1, 20.1, 0.3, 20.1, 0.3]]
    keep2, drop2 = active_thrusters(_rows2, _cols, ["main", "rcs_px"])
    truth("keeps a thruster that fires even briefly",
          keep2 == ["main", "rcs_px"] and not drop2, f"kept {keep2}")
    # Only the COMMAND is non-zero (saturated to zero applied thrust, or a
    # dry tank): still worth plotting, since the command is the law's intent.
    _rows3 = [[0.0, 0.0, 0.0, 5.0, 0.0], [0.1, 0.0, 0.0, 5.0, 0.0]]
    keep3, _ = active_thrusters(_rows3, _cols, ["main", "rcs_px"])
    truth("keeps a thruster commanded but producing nothing",
          "main" in keep3, f"kept {keep3}")
    # If NOTHING fired, keep everything: an empty panel would hide the
    # finding that the engine never lit.
    _rows4 = [[0.0, 0.0, 0.0, 0.0, 0.0], [0.1, 0.0, 0.0, 0.0, 0.0]]
    keep4, drop4 = active_thrusters(_rows4, _cols, ["main", "rcs_px"])
    truth("keeps everything when no thruster fired at all",
          keep4 == ["main", "rcs_px"] and not drop4, f"kept {keep4}")

    # And end to end: the figure is written, and --all-thrusters still works.
    d_hide, _ = fly_dir("hover", plot=True)
    d_show, _ = fly_dir("hover", ["--all-thrusters"], plot=True)
    truth("a plotted run writes flight.png",
          d_hide is not None and (d_hide / "flight.png").exists())
    truth("--all-thrusters also writes flight.png",
          d_show is not None and (d_show / "flight.png").exists())
    # The CSV must keep every column either way - hiding is a PLOT choice,
    # and dropping the data would make a run unanalysable after the fact.
    if d_hide is not None:
        head = (d_hide / "flight.csv").read_text().splitlines()[0]
        truth("  the CSV still carries every thruster column",
              all(f"{n}_N" in head for n in
                  ("main", "rcs_px", "rcs_nx", "rcs_py", "rcs_ny")))
        truth("  hiding changes only the figure, not the metrics",
              json.loads((d_hide / "run.json").read_text())["metrics"]
              == json.loads((d_show / "run.json").read_text())["metrics"])

    print("\n[10] the figure says what produced it")
    # A figure with no provenance is useless in a report: two flight.png
    # files from different tunings are otherwise indistinguishable.
    cfg = config_lines(
        controller="example_hover:hover_cascade", scene="07_thrust_1D.xml",
        gains={"KP_ALT": 18.0, "KI_ALT": 6.0, "KP_POS_V": 0.6, "KP_V": 14.0},
        overridden={"KP_V": "14"},
        profile="0.400 -> 2 m, down at 0.1 m/s",
        mass_note="12.000 kg fixed",
        metrics={"overshoot_m": 0.0518, "hold_rms_err_m": 0.0317},
        gain_names={"KP_POS_V", "KP_V"})
    labels = [k for k, _ in cfg]
    byname = dict(cfg)
    truth("names the controller and scene",
          byname.get("controller", "").endswith("hover_cascade")
          and byname.get("scene") == "07_thrust_1D.xml")
    # Only the gains the LAW reads. Listing KI_ALT under a cascade run would
    # be actively misleading - the cascade has no integrator.
    truth("lists only the gains the law reads",
          "KP_POS_V" in labels and "KP_V" in labels
          and "KI_ALT" not in labels and "KP_ALT" not in labels,
          ", ".join(n for n in labels if n.startswith(("KP", "KI", "KD"))))
    truth("marks an overridden gain as set",
          "(set)" in byname["KP_V"] and "(set)" not in byname["KP_POS_V"],
          f"KP_V={byname['KP_V']!r}")
    # An override of a gain the law ignores must be SAID, not hidden: it
    # means a sweep is varying something with no effect on the run.
    cfg2 = config_lines(
        controller="example_hover:hover_pid", scene="s.xml",
        gains={"KI_ALT": 6.0}, overridden={"KP_POS_V": "0.9"},
        profile="", mass_note="", gain_names={"KI_ALT"})
    truth("flags an override the law cannot read",
          dict(cfg2).get("unused overrides") == "KP_POS_V")
    truth("carries the headline metrics",
          dict(cfg).get("overshoot_m") == "+0.05180")
    truth("gains are in a stable alphabetical order",
          [n for n in labels if n in ("KP_POS_V", "KP_V")] == ["KP_POS_V", "KP_V"])
    # One implementation, shared with the GUI. Compared by source rather
    # than identity: run as a script this module is __main__, so gui.py
    # imports a SECOND copy of it under the name "scenario" and an `is`
    # check would compare two objects that are both correct.
    import inspect as _inspect
    truth("one gains_used_by, shared with the GUI",
          _inspect.getsource(__import__("gui").gains_used_by)
          == _inspect.getsource(gains_used_by)
          and __import__("gui").gains_used_by.__module__ == "scenario")

    # End to end: the panel must widen the figure, and --no-config-panel
    # must restore the old one.
    d_cfg, _ = fly_dir("hover_cascade", plot=True)
    d_bare, _ = fly_dir("hover_cascade", ["--no-config-panel"], plot=True)
    if d_cfg is not None and d_bare is not None:
        from PIL import Image
        w_cfg = Image.open(d_cfg / "flight.png").size[0]
        w_bare = Image.open(d_bare / "flight.png").size[0]
        truth("  the panel widens the figure", w_cfg > w_bare,
              f"{w_cfg}px vs {w_bare}px")
        truth("  --no-config-panel restores the plain figure",
              w_bare == 1210, f"{w_bare}px")
        truth("  the panel does not change the run",
              json.loads((d_cfg / "run.json").read_text())["metrics"]
              == json.loads((d_bare / "run.json").read_text())["metrics"])

    # --view with no display must lose the WINDOW, not the run. glfw aborts
    # the process at import time when DISPLAY is unset, which is earlier
    # than any try/except around the viewer can reach.
    env = {k: v for k, v in os.environ.items()
           if k not in ("DISPLAY", "WAYLAND_DISPLAY")}
    r_nd = subprocess.run(
        [sys.executable, str(HERE / "scenario.py"),
         "--controller", "example_hover:hover",
         "--scene", "scenes/07_thrust_1D.xml", "--target", "1.0",
         "--fixed-mass", "--no-plot", "--view", "--outdir", str(out8)],
        capture_output=True, text=True, timeout=900, env=env)
    truth("--view without a display still completes the run",
          r_nd.returncode == 0, f"exit {r_nd.returncode}")
    truth("  and says why the window did not open",
          "no DISPLAY" in r_nd.stderr)

    print("\n[11] a finished run rebuilds into a live figure")
    # The GUI runs this script as a subprocess, so the figure object dies
    # with the run. An interactive view has to REBUILD from what was saved -
    # which only works if flight.csv + run.json really are sufficient. These
    # checks pin that, and pin that the rebuild is the same figure.
    if d_cfg is not None and d_bare is not None:
        run = load_run(d_cfg)
        truth("load_run recovers the full table",
              len(run["rows"]) > 1000 and len(run["rows"][0]) == len(run["cols"]),
              f"{len(run['rows'])} rows x {len(run['cols'])} cols")
        # phase is the one string column; everything else must be a number,
        # or the plotter's arithmetic silently concatenates strings.
        ixp = run["cols"].index("phase")
        r0 = run["rows"][0]
        truth("  phase stays a string, every other cell is a float",
              isinstance(r0[ixp], str)
              and all(isinstance(v, float)
                      for i, v in enumerate(r0) if i != ixp),
              f"phase={r0[ixp]!r}")
        truth("  thruster names come from the CSV header",
              run["names"] and all(f"{n}_N" in run["cols"] for n in run["names"]),
              ", ".join(run["names"]))
        truth("  the config caption is rebuilt from run.json",
              run["config"] and dict(run["config"]).get("controller", "")
              .endswith("hover_cascade"))

        # The load-bearing claim: the rebuilt figure is the SAME figure. Size
        # is the sharpest cheap proxy - panel count, the config panel and the
        # mass row all move it - and 110 dpi is what savefig used.
        from PIL import Image
        for d, want_cfg, want_px in ((d_cfg, True, w_cfg), (d_bare, False, w_bare)):
            fig, _ = figure_for_run(d, config=want_cfg)
            got = int(round(fig.get_size_inches()[0] * 110))
            truth(f"  rebuilt figure matches flight.png ({'with' if want_cfg else 'no'} panel)",
                  got == want_px, f"{got}px vs {want_px}px")
            hpx = int(round(fig.get_size_inches()[1] * 110))
            truth("    and its height too",
                  hpx == Image.open(d / "flight.png").size[1],
                  f"{hpx}px")
            import matplotlib.pyplot as _plt
            _plt.close(fig)

        # save=False must write NOTHING and hand the figure over. Closing it
        # is exactly the wrong thing to do to a figure about to be rendered.
        before = sorted(q.name for q in d_bare.iterdir())
        rb = load_run(d_bare)
        pth, fig = plot_flight(
            d_bare / "must_not_appear.png", rb["rows"], rb["cols"], None,
            rb["target"], rb["z_ground"], rb["descent_rate"], rb["names"],
            show_mass=rb["show_mass"], config=None, save=False)
        truth("  save=False returns the figure and no path",
              pth is None and fig is not None)
        truth("    and writes nothing",
              sorted(q.name for q in d_bare.iterdir()) == before)
        import matplotlib.pyplot as _plt
        truth("    and leaves the figure open",
              fig.number in _plt.get_fignums())
        _plt.close(fig)

    # The awkward runs. A missing CSV is fatal - there is nothing to draw -
    # but a missing or corrupt run.json must only cost the caption, the same
    # tolerance gui.find_runs() already has.
    import tempfile as _tf
    with _tf.TemporaryDirectory() as td:
        empty = pathlib.Path(td) / "no_csv"
        empty.mkdir()
        try:
            load_run(empty)
            truth("a run with no flight.csv raises", False, "no exception")
        except FileNotFoundError as e:
            truth("a run with no flight.csv raises", True, str(e)[-40:])
        if d_bare is not None:
            broke = pathlib.Path(td) / "broken"
            broke.mkdir()
            (broke / "flight.csv").write_text((d_bare / "flight.csv").read_text())
            (broke / "run.json").write_text("{not json at all")
            rbk = load_run(broke)
            truth("  a corrupt run.json costs the caption, not the plot",
                  rbk["config"] is None and len(rbk["rows"]) > 1000,
                  f"{len(rbk['rows'])} rows, config={rbk['config']}")
            nojs = pathlib.Path(td) / "nojson"
            nojs.mkdir()
            (nojs / "flight.csv").write_text((d_bare / "flight.csv").read_text())
            rnj = load_run(nojs)
            truth("  a missing run.json does the same",
                  rnj["config"] is None and rnj["show_mass"] is False)
            # --fixed-mass writes propellant: null, so the mass row must be
            # dropped - plotting an empty 6th panel would be a lie.
            truth("  a --fixed-mass run rebuilds without the mass panel",
                  load_run(d_bare)["show_mass"] is False)

    # Backend isolation, in subprocesses because a backend choice is global
    # and the selftest itself has already fixed one. Agg must still be
    # forced when nothing interactive is live (this is what keeps the
    # no-DISPLAY run above working), and must NOT be forced over a backend
    # the caller already picked - the GUI selects TkAgg before calling in.
    if d_bare is not None:
        probe = (
            "import sys, matplotlib\n"
            "pre = sys.argv[1]\n"
            "if pre:\n"
            "    matplotlib.use(pre)\n"
            "sys.path.insert(0, sys.argv[2])\n"
            "import scenario\n"
            "scenario.figure_for_run(sys.argv[3], config=False)\n"
            "print(matplotlib.get_backend())\n")
        env_nd = {k: v for k, v in os.environ.items()
                  if k not in ("DISPLAY", "WAYLAND_DISPLAY", "MPLBACKEND")}
        args_nd = [str(HERE), str(d_bare)]
        r_agg = subprocess.run([sys.executable, "-c", probe, ""] + args_nd,
                               capture_output=True, text=True, timeout=300,
                               env=env_nd)
        truth("headless, plot_flight still selects Agg",
              r_agg.stdout.strip().lower() == "agg",
              r_agg.stdout.strip() or (r_agg.stderr or "")[-120:])
        # "pdf" stands in for TkAgg here: it is non-interactive so it works
        # with no DISPLAY, but it is not Agg, so if the guard were an
        # unconditional matplotlib.use("Agg") this check would fail.
        r_keep = subprocess.run([sys.executable, "-c", probe, "pdf"] + args_nd,
                                capture_output=True, text=True, timeout=300,
                                env=env_nd)
        truth("  but does not stomp a backend the caller chose",
              r_keep.stdout.strip().lower() == "pdf",
              r_keep.stdout.strip() or (r_keep.stderr or "")[-120:])

    print("\n[11b] every profile segment is commandable")
    # The three hold times used to be module constants read straight out of
    # Profile.update(), so a run could set the target and the descent rate
    # and nothing else. These check they are honoured per run - which is
    # what the CLI flags and the GUI's profile box drive.
    dtp = 0.002
    pc = Profile(0.4, 3.0, 0.25, dtp, ground_sit_s=1.0, hold_target_s=8.0,
                 hold_ground_s=1.5)
    tt, zz, phases, guard = 0.0, 0.4, {}, 0
    while not pc.done and guard < 400000:
        sp = pc.update(tt, zz, 0.0)
        zz = sp                       # a tracker that arrives instantly
        phases[pc.phase] = phases.get(pc.phase, 0.0) + dtp
        tt += dtp
        guard += 1
    ok("GROUND_SIT honours its own duration", phases.get("GROUND_SIT", 0),
       1.0, 0.01, " s")
    ok("HOLD_TARGET honours its own duration", phases.get("HOLD_TARGET", 0),
       8.0, 0.01, " s")
    ok("GROUND_HOLD honours its own duration", phases.get("GROUND_HOLD", 0),
       1.5, 0.01, " s")
    ok("  and the descent honours its rate", phases.get("DESCEND", 0),
       (3.0 - 0.4) / 0.25, 0.05, " s")
    truth("  no phase timed out on a custom profile", not pc.timeouts,
          str(pc.timeouts))
    # The estimate must match what the machine actually took, or the figure
    # the GUI shows before launching is decoration.
    ok("duration_estimate_s matches the real run", tt,
       pc.duration_estimate_s(), 0.05, " s")

    # Zero-length holds are legal: "no pad sit, no hover" is a real request
    # and must not hang or divide by zero.
    pz = Profile(0.4, 1.0, 0.5, dtp, ground_sit_s=0.0, hold_target_s=0.0,
                 hold_ground_s=0.0)
    t0, z0, g0 = 0.0, 0.4, 0
    while not pz.done and g0 < 400000:
        z0 = pz.update(t0, z0, 0.0)
        t0 += dtp
        g0 += 1
    truth("zero-length holds complete rather than hang", pz.done,
          f"{t0:.2f}s, timeouts {pz.timeouts}")

    # THE ASCENT RAMP. ASCEND was a step while DESCEND ramped - not a design
    # decision so much as the order they were written in. A step is how you
    # measure step response, so it stays the default; a rate asks for a
    # controlled climb, which is what a flight profile actually specifies.
    pstep = Profile(0.4, 2.0, 0.1, dtp)
    pstep.update(0.0, 0.4, 0.0)
    while pstep.phase == "GROUND_SIT":
        pstep.update(0.0, 0.4, 0.0)
    sp_step = pstep.update(0.0, 0.4, 0.0)
    truth("ascent is a STEP by default", abs(sp_step - 2.0) < 1e-9,
          f"first ASCEND setpoint {sp_step:.4f} m (target 2.0)")
    pramp = Profile(0.4, 2.0, 0.1, dtp, ascent_rate=0.5)
    while pramp.phase == "GROUND_SIT":
        pramp.update(0.0, 0.4, 0.0)
    sp_r1 = pramp.update(0.0, 0.4, 0.0)
    truth("  --ascent-rate makes it a RAMP",
          0.4 < sp_r1 < 0.41, f"first ASCEND setpoint {sp_r1:.4f} m")
    for _ in range(500):
        sp_r2 = pramp.update(0.0, 0.4, 0.0)
    ok("  the ramp rises at the commanded rate",
       (sp_r2 - sp_r1) / (500 * dtp), 0.5, 0.01, " m/s")
    # A ramped ascent needs a timeout that COVERS the ramp, exactly like the
    # descent: a flat 30 s would fire during a correct slow climb.
    slow = Profile(0.0, 20.0, 0.1, dtp, ascent_rate=0.1)
    truth("  a slow ramp gets a timeout that covers it",
          slow._timeout_for("ASCEND") > 20.0 / 0.1,
          f"{slow._timeout_for('ASCEND'):.0f}s for a 200s climb")
    truth("  a stepped ascent keeps the flat timeout",
          pstep._timeout_for("ASCEND") == PHASE_TIMEOUT_S,
          f"{pstep._timeout_for('ASCEND'):.0f}s")
    # Arrival is tested against the TARGET, not the ramp's setpoint: the
    # ramp reaches the target before the craft does, and testing the
    # setpoint would end the phase while still climbing.
    truth("  a ramped ascent still waits for the CRAFT to arrive",
          pramp.phase == "ASCEND",
          f"setpoint {sp_r2:.3f} m with the craft still at 0.4 m")

    # End to end: the flags reach the flight and run.json records the shape.
    out11 = ROOT / "out" / "_selftest_profile"
    r_prof = subprocess.run(
        [sys.executable, str(HERE / "scenario.py"),
         "--controller", "example_hover:hover_cascade",
         "--scene", "scenes/07_thrust_1D.xml", "--target", "3.0",
         "--ascent-rate", "0.5", "--ground-sit", "1.0",
         "--hold-target", "8.0", "--hold-ground", "1.5",
         "--descent-rate", "0.25", "--fixed-mass", "--no-plot",
         "--outdir", str(out11)],
        capture_output=True, text=True, timeout=900)
    truth("a fully custom profile flies end to end",
          r_prof.returncode == 0, (r_prof.stderr or "")[-160:])
    if r_prof.returncode == 0:
        d_prof = max(out11.glob("*/"), key=lambda q: q.stat().st_mtime)
        mp = json.loads((d_prof / "run.json").read_text())
        pj = mp["profile"]
        truth("  run.json records every segment",
              pj["ground_sit_s"] == 1.0 and pj["hold_at_target_s"] == 8.0
              and pj["hold_at_ground_s"] == 1.5
              and pj["ascent_rate_mps"] == 0.5,
              f"sit {pj['ground_sit_s']}, hover {pj['hold_at_target_s']}, "
              f"hold {pj['hold_at_ground_s']}, climb {pj['ascent_rate_mps']}")
        truth("    and the shape in words",
              "ramp up at 0.5 m/s" in pj.get("shape", ""),
              pj.get("shape", "")[:56])
        # The phase boundaries in the flight must match what was asked for.
        bnd = {f"{b['from']}->{b['to']}": b["t"] for b in mp["phase_boundaries"]}
        ok("  GROUND_SIT lasted what was commanded",
           bnd.get("GROUND_SIT->ASCEND", 0.0), 1.0, 0.01, " s")
        hold = (bnd.get("HOLD_TARGET->DESCEND", 0.0)
                - bnd.get("ASCEND->HOLD_TARGET", 0.0))
        ok("  the hover lasted what was commanded", hold, 8.0, 0.02, " s")
        # A long hover must not be cut off by a wall-clock ceiling computed
        # from the DEFAULT profile, which is what max_wall used to be.
        truth("  the wall budget grew with the profile",
              not mp["metrics"]["timeouts"] and mp["completed"],
              f"timeouts {mp['metrics']['timeouts']}, "
              f"completed {mp['completed']}")

    print("\n[12] the valve layer, flown through the real driver")
    # These are the checks the standalone valve suite CANNOT make: valve.py
    # tests the plant with no sim, and every bug found while wiring this up
    # was an interaction bug that only a flight exposes.
    out12 = ROOT / "out" / "_selftest_valve"

    def valve_dir(extra=(), plot=False, law="hover_valve", fixed_mass=True):
        argv = [sys.executable, str(HERE / "scenario.py"),
                "--controller", f"example_hover:{law}",
                "--scene", "scenes/07_thrust_1D.xml", "--target", "2.0",
                "--isp", "57.8", "--valve", "--outdir", str(out12)]
        if fixed_mass:
            argv.append("--fixed-mass")
        if not plot:
            argv.append("--no-plot")
        argv += list(extra)
        r = subprocess.run(argv, capture_output=True, text=True, timeout=900)
        if r.returncode != 0:
            return None, r
        return max(out12.glob("*/"), key=lambda q: q.stat().st_mtime), r

    d_v, r_v = valve_dir()
    truth("a --valve run completes the profile", d_v is not None,
          "" if d_v else (r_v.stderr or "")[-200:])
    if d_v is not None:
        mv = json.loads((d_v / "run.json").read_text())
        xv = mv["metrics"]
        # *** THE TWO BUGS THIS WIRING ACTUALLY HAD. *** Both were freeze
        # logic, both flew, and neither was visible in valve.py's suite.
        #
        # 1. Inferring "grounded" from the measurement froze the valve at
        #    u=0.5812 mid-climb, because steady cruise reads a ~= 0 with
        #    F_meas ~= weight exactly like the pad does. The craft flew to
        #    61 m on a 2 m profile.
        truth("  does not run away (the cruise-looks-like-the-pad bug)",
              xv["peak_altitude_m"] < 3.0, f"peak {xv['peak_altitude_m']:.4f} m")
        # 2. Freezing on contact ALONE never launched: a 48.2 N demand
        #    against a 19.44 N weight sat at u=0.0000 for the whole 30 s
        #    ASCEND phase. The freeze needs the demand to be BELOW weight.
        truth("  does leave the ground (the never-launches bug)",
              xv["peak_altitude_m"] > 1.9 and not xv["timeouts"],
              f"peak {xv['peak_altitude_m']:.4f} m, timeouts {xv['timeouts']}")
        truth("  and tracks the profile as well as a direct-thrust law",
              abs(xv["overshoot_m"]) < 0.15 and xv["hold_rms_err_m"] < 0.05,
              f"overshoot {xv['overshoot_m']:+.4f} m, "
              f"hold RMS {xv['hold_rms_err_m']:.5f} m")
        # The tracker's own job: zero steady-state thrust error. This is the
        # claim the state-space analysis makes - DC gain is exactly 1 for
        # ANY positive slope, so accuracy does not depend on the curve.
        truth("  steady-state thrust error is ~zero",
              abs(xv["thrust_err_final_N"]) < 0.05,
              f"{xv['thrust_err_final_N']:+.5f} N at the end of the hold")
        truth("  the valve converged part-open, not pinned",
              0.0 < xv["valve_u_final"] < 1.0 and xv["valve_pinned_s"] == 0.0,
              f"u={xv['valve_u_final']:.4f}, pinned {xv['valve_pinned_s']:g}s")
        truth("  nothing starved on a full bottle",
              xv["valve_starved_at_s"] is None)
        # run.json must SAY which plant was flown, or a sweep's runs cannot
        # be told apart afterwards.
        truth("  run.json records the curve and the feed",
              mv["valve"]["curve"] == "ball"
              and mv["valve"]["feed_mode"] == "regulated"
              and mv["valve"]["tank_sized_from_engine"] is True)
        # The part-open window is quoted by FLOW. Quoted by POSITION,
        # VALVE_HOLD_POSITION=0.425 lands in the ball curve's dead zone and
        # a real run reported `inf` for it.
        truth("  both regulated windows are finite numbers",
              0.0 < mv["valve"]["regulated_window_s"] < 1e6
              and 0.0 < mv["valve"]["regulated_window_at_hold_s"] < 1e6,
              f"{mv['valve']['regulated_window_s']:.1f}s open, "
              f"{mv['valve']['regulated_window_at_hold_s']:.1f}s at hold")
        truth("  and the hold point lasts longer than wide open",
              mv["valve"]["regulated_window_at_hold_s"]
              > mv["valve"]["regulated_window_s"])

        # The CSV columns, and the bug in reading them back.
        import csv as _csvmod
        with open(d_v / "flight.csv", newline="") as fh:
            vcols = next(_csvmod.reader(fh))
        for c in ("valve_u", "thrust_cmd_N", "thrust_meas_N",
                  "thrust_act_N", "feed_frac"):
            truth(f"  flight.csv carries {c}", c in vcols)
        # thrust_cmd_N ends in _N but is NOT a thruster. Taking it as one
        # asked the plotter for a ctrl_thrust_cmd column that does not
        # exist - a KeyError rebuilding a run whose own PNG was fine.
        rv = load_run(d_v)
        truth("  load_run does not mistake thrust_*_N for thrusters",
              "thrust_cmd" not in rv["names"] and "main" in rv["names"],
              ", ".join(rv["names"]))
        truth("  and notices the valve panels are wanted",
              rv["show_valve"] is True)

        # Propellant accounting. data.ctrl has to hold NEWTONS before
        # PropellantTank.update() reads it, because that reads
        # np.sum(np.abs(data.ctrl)) as Newtons. A raw 0..1 opening left
        # there would under-report the burn by a factor of max_thrust.
        d_vt, r_vt = valve_dir(["--dry-mass", "10", "--propellant", "2"],
                               fixed_mass=False)
        truth("  a valve run with a tank completes", d_vt is not None,
              "" if d_vt else (r_vt.stderr or "")[-200:])
        if d_vt is not None:
            xt = json.loads((d_vt / "run.json").read_text())["metrics"]
            truth("    mass bookkeeping still balances",
                  xt["mass_bookkeeping_err_kg"] < 1e-9,
                  f"{xt['mass_bookkeeping_err_kg']:.2e} kg")
            # The scale check: ~19.4 N for ~30 s at Isp 57.8 burns ~1 kg.
            # Reading a 0..1 opening as Newtons instead would burn ~60x
            # less, so this interval catches the ordering bug by magnitude.
            truth("    propellant burned is the right ORDER, not 1/60th",
                  0.4 < xt["propellant_used_kg"] < 2.0,
                  f"{xt['propellant_used_kg']:.4f} kg over "
                  f"{xt['total_duration_s']:.1f}s")

    # A law that returns NEWTONS must be REFUSED, not flown: 19.44 read as
    # an opening clamps to 1.0 and the engine sits wide open all flight,
    # which looks like a wild controller rather than a units mismatch.
    d_bad, r_bad = valve_dir(law="hover")
    truth("  --valve refuses a law that returns Newtons",
          d_bad is None and "returns Newtons" in (r_bad.stderr or ""),
          (r_bad.stderr or "").strip().splitlines()[0][:70] if r_bad.stderr else "")

    # The curve FAMILY. This is what the rig is for: one controller, a
    # family of unknown plants, and the accuracy claim has to hold on all
    # of them while the transient is allowed to differ.
    fam = {}
    for cname in ("linear", "sqrt", "quadratic"):
        d_c, _ = valve_dir(["--valve-curve", cname])
        if d_c is not None:
            fam[cname] = json.loads((d_c / "run.json").read_text())["metrics"]
    truth("  every curve in the family flies the profile",
          len(fam) == 3 and all(not x["timeouts"] for x in fam.values()),
          ", ".join(f"{k} peak {v['peak_altitude_m']:.3f}"
                    for k, v in fam.items()))
    if len(fam) == 3:
        truth("    steady-state error is ~zero on ALL of them",
              all(abs(x["thrust_err_final_N"]) < 0.05 for x in fam.values()),
              ", ".join(f"{k} {v['thrust_err_final_N']:+.5f}N"
                        for k, v in fam.items()))
        # The openings must DIFFER - that is the proof each run inverted a
        # different curve without being told which. Identical openings
        # would mean the curve was not actually being varied.
        us = [x["valve_u_final"] for x in fam.values()]
        truth("    but each inverted a DIFFERENT curve",
              max(us) / max(min(us), 1e-9) > 1.5,
              ", ".join(f"{k} u={v['valve_u_final']:.4f}"
                        for k, v in fam.items()))

    # The time dependence. A small bottle drops the regulator out mid-flight
    # and the SAME valve position then delivers a fraction of the thrust;
    # the integrator has to absorb that with no model of it.
    d_bd, _ = valve_dir(["--valve-tank-l", "12"])
    if d_bd is not None:
        xb = json.loads((d_bd / "run.json").read_text())["metrics"]
        truth("  a small bottle really does drop the regulator out",
              xb["feed_frac_final"] < 0.5,
              f"feed down to {xb['feed_frac_final']:.3f} of nominal")
        truth("    and the loop absorbs the blowdown",
              abs(xb["thrust_err_final_N"]) < 0.05,
              f"{xb['thrust_err_final_N']:+.5f} N with the feed at "
              f"{xb['feed_frac_final']:.3f}")
        truth("    with the profile unharmed",
              abs(xb["overshoot_m"]) < 0.15 and not xb["timeouts"],
              f"overshoot {xb['overshoot_m']:+.4f} m")

    # A --valve run must not disturb a normal one: the extra columns appear
    # ONLY with the valve, so everything that reads flight.csv keeps working.
    d_nov, _ = fly_dir("hover")
    if d_nov is not None:
        import csv as _csvmod
        with open(d_nov / "flight.csv", newline="") as fh:
            ncols = next(_csvmod.reader(fh))
        truth("  a run WITHOUT --valve has no valve columns",
              not any(c.startswith(("valve_", "thrust_", "feed_"))
                      for c in ncols))
        truth("    and rebuilds with no valve panels",
              load_run(d_nov)["show_valve"] is False)

    print("\n[13] the valve under the CASCADE: a three-loop stack")
    # hover_cascade_valve puts the thrust tracker under the cascade instead
    # of under hover's single-loop PD, so there are three nested loops:
    # position -> rate -> thrust -> valve. The claim being tested is that
    # the cascade's advantage is STRUCTURAL and therefore survives having a
    # valve underneath it.
    #
    # First: the refactor that made this possible must not have changed
    # hover_cascade. _cascade_demand() IS its body now, so the direct law's
    # numbers have to be exactly what they were.
    def direct_dir(law, extra=()):
        """A thrust-commanding run on the SAME profile valve_dir uses.

        fly_dir() flies --target 1.0; every valve run above flies 2.0. The
        whole of this section compares a valve law against its direct
        equivalent, so they have to be the same flight.
        """
        argv = [sys.executable, str(HERE / "scenario.py"),
                "--controller", f"example_hover:{law}",
                "--scene", "scenes/07_thrust_1D.xml", "--target", "2.0",
                "--fixed-mass", "--no-plot", "--outdir", str(out12)]
        argv += list(extra)
        r = subprocess.run(argv, capture_output=True, text=True, timeout=900)
        if r.returncode != 0:
            return None, r
        return max(out12.glob("*/"), key=lambda q: q.stat().st_mtime), r

    d_casc, _ = direct_dir("hover_cascade")
    if d_casc is not None:
        xc = json.loads((d_casc / "run.json").read_text())["metrics"]
        # _cascade_demand() IS hover_cascade's body now, so the direct law's
        # numbers must be exactly what they were before the refactor.
        ok("hover_cascade is unchanged by the refactor (overshoot)",
           xc["overshoot_m"], 0.05176, 1e-4, " m")
        ok("  and its descent lag", xc["descent_lag_mean_m"],
           -0.00731, 1e-4, " m")

    d_cv, r_cv = valve_dir(law="hover_cascade_valve")
    truth("a cascade+valve run completes the profile", d_cv is not None,
          "" if d_cv else (r_cv.stderr or "")[-200:])
    if d_cv is not None and d_casc is not None:
        xv = json.loads((d_cv / "run.json").read_text())["metrics"]
        # THE HEADLINE. The cascade beats the PD on a descent ramp because
        # the thing it tracks IS a velocity, so no error has to accumulate.
        # That argument is about the OUTER loops, so a valve underneath
        # must not cost it. Measured: -0.00722 m through the valve against
        # -0.00731 m commanding thrust directly.
        truth("  the cascade's descent-lag advantage survives the valve",
              abs(xv["descent_lag_mean_m"]) < 0.02,
              f"{xv['descent_lag_mean_m']:+.5f} m vs "
              f"{xc['descent_lag_mean_m']:+.5f} m direct")
        truth("    and matches the direct law to within 5%",
              abs(xv["descent_lag_mean_m"] - xc["descent_lag_mean_m"])
              < 0.05 * abs(xc["descent_lag_mean_m"]),
              f"{xv['descent_lag_mean_m']:+.5f} vs "
              f"{xc['descent_lag_mean_m']:+.5f} m")
        # And it must be the CASCADE's lag, not the PD's - a 17x difference,
        # so this cannot pass by accident if the law silently used hover().
        d_pdv, _ = valve_dir(law="hover_valve")
        if d_pdv is not None:
            xp = json.loads((d_pdv / "run.json").read_text())["metrics"]
            truth("    an order of magnitude better than the PD+valve",
                  abs(xv["descent_lag_mean_m"]) < 0.2 * abs(
                      xp["descent_lag_mean_m"]),
                  f"{xv['descent_lag_mean_m']:+.5f} vs "
                  f"{xp['descent_lag_mean_m']:+.5f} m "
                  f"({abs(xp['descent_lag_mean_m'] / xv['descent_lag_mean_m']):.0f}x)")
        truth("  and the tracker still zeroes the thrust error",
              abs(xv["thrust_err_final_N"]) < 0.05,
              f"{xv['thrust_err_final_N']:+.5f} N")
        truth("  run.json names the cascade valve law",
              json.loads((d_cv / "run.json").read_text())["controller"]
              .endswith("hover_cascade_valve"))

    # Gain attribution: the union of the cascade's gains and the tracker's,
    # and crucially NOT KP_ALT/KD_ALT - listing those would say the law reads
    # a PD gain it never touches, which is how a sweep becomes unreadable.
    gcv = gains_used_by("hover_cascade_valve")
    truth("  it reads the cascade's gains",
          {"KP_POS_V", "KP_V", "V_MAX_MPS"} <= gcv,
          ", ".join(sorted(n for n in gcv if n.startswith(("KP", "V_")))))
    truth("    and the tracker's",
          {"K_VALVE", "GROUND_FREEZE"} <= gcv)
    truth("    but NOT the single-loop PD's",
          not ({"KP_ALT", "KD_ALT"} & gcv),
          ", ".join(sorted(gcv & {"KP_ALT", "KD_ALT"})) or "neither")

    # The timescale separation, which is the one thing three loops can break
    # that two cannot. Starve the INNER loop and the failure has to show up
    # in overshoot (the rate loop chasing a thrust that arrives late), while
    # the descent lag - an outer-loop property - stays good. That signature
    # is what tells you to raise K_VALVE rather than retune the cascade.
    d_slow, _ = valve_dir(["--gain", "K_VALVE=0.005"],
                          law="hover_cascade_valve")
    if d_slow is not None and d_cv is not None:
        xs = json.loads((d_slow / "run.json").read_text())["metrics"]
        truth("  a too-slow thrust loop inverts the cascade's assumption",
              xs["overshoot_m"] > 3 * xv["overshoot_m"],
              f"overshoot {xs['overshoot_m']:.4f} m at K_VALVE=0.005 "
              f"vs {xv['overshoot_m']:.4f} m at 0.05")
        truth("    and it shows in OVERSHOOT, not in the descent lag",
              abs(xs["descent_lag_mean_m"]) < 0.02,
              f"lag still {xs['descent_lag_mean_m']:+.5f} m")
    # The other end: a fast enough tracker is indistinguishable from
    # commanding thrust directly, which is the separation-of-timescales
    # argument closing.
    d_fast, _ = valve_dir(["--gain", "K_VALVE=0.5"],
                          law="hover_cascade_valve")
    if d_fast is not None and d_casc is not None:
        xf = json.loads((d_fast / "run.json").read_text())["metrics"]
        truth("  a fast tracker converges on the direct cascade",
              abs(xf["overshoot_m"] - xc["overshoot_m"]) < 0.005
              and abs(xf["descent_lag_mean_m"]
                      - xc["descent_lag_mean_m"]) < 0.001,
              f"overshoot {xf['overshoot_m']:.4f} vs {xc['overshoot_m']:.4f}, "
              f"lag {xf['descent_lag_mean_m']:+.5f} vs "
              f"{xc['descent_lag_mean_m']:+.5f}")

    # Every curve, since three loops need the separation to hold for each
    # one and gamma is what sets the inner loop's speed.
    cfam = {}
    for cname in ("linear", "sqrt", "deadzone"):
        d_c, _ = valve_dir(["--valve-curve", cname], law="hover_cascade_valve")
        if d_c is not None:
            cfam[cname] = json.loads((d_c / "run.json").read_text())["metrics"]
    truth("  the three-loop stack flies every curve tried",
          len(cfam) == 3 and all(not x["timeouts"] for x in cfam.values()),
          ", ".join(f"{k} {v['descent_lag_mean_m']:+.5f}m"
                    for k, v in cfam.items()))
    if len(cfam) == 3:
        lags = [abs(x["descent_lag_mean_m"]) for x in cfam.values()]
        truth("    with the lag advantage intact on all of them",
              max(lags) < 0.02, f"worst {max(lags):.5f} m")

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


if __name__ == "__main__":
    sys.exit(main())
