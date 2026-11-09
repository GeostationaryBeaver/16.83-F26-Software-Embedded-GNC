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

    def __init__(self, z_ground, target_alt, descent_rate, dt):
        self.z_ground = z_ground
        self.target = target_alt
        self.rate = descent_rate
        self.dt = dt
        self.phase = "GROUND_SIT"
        self.setpoint = z_ground
        self.t_phase = 0.0          # seconds spent in the current phase
        self.boundaries = []        # (sim_time, from_phase, to_phase)
        self.timeouts = []          # phases that gave up
        self.done = False
        # The descent ramp itself takes (target - ground)/rate seconds, which
        # at the 0.1 m/s default is ~16 s. A flat timeout would fire during a
        # perfectly correct run, so the descent gets the ramp time plus the
        # allowance.
        self._descend_timeout = (target_alt - z_ground) / max(descent_rate, 1e-9) \
            + PHASE_TIMEOUT_S

    def _advance(self, t_now, nxt):
        self.boundaries.append((t_now, self.phase, nxt))
        self.phase = nxt
        self.t_phase = 0.0
        if nxt == "DONE":
            self.done = True

    def _timeout_for(self, phase):
        return self._descend_timeout if phase == "DESCEND" else PHASE_TIMEOUT_S

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
            if self.t_phase >= GROUND_SIT_S:
                self._advance(t_now, "ASCEND")

        elif self.phase == "ASCEND":
            self.setpoint = self.target          # a STEP, not a ramp
            if self._arrived(z, vz, self.target):
                self._advance(t_now, "HOLD_TARGET")
            elif self.t_phase >= self._timeout_for("ASCEND"):
                self.timeouts.append("ASCEND")
                self._advance(t_now, "HOLD_TARGET")

        elif self.phase == "HOLD_TARGET":
            self.setpoint = self.target
            if self.t_phase >= HOLD_AT_TARGET_S:
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
            if self.t_phase >= HOLD_AT_GROUND_S:
                self._advance(t_now, "DONE")

        return self.setpoint


def make_logger(ThrustLogger, names):
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
    """

    class ScenarioLogger(ThrustLogger):
        @property
        def header(self):
            return (super().header
                    + ["setpoint_m", "accel_mps2", "qacc_mps2", "phase"]
                    + [f"ctrl_{n}" for n in self.names]
                    + ["mass_kg", "propellant_kg"])

        def record_tick(self, state, magnitudes, setpoint, accel, qacc,
                        phase, ctrls, mass_kg, propellant_kg):
            super().record(state, magnitudes)
            self.rows[-1].extend([setpoint, accel, qacc, phase]
                                 + [float(c) for c in ctrls]
                                 + [mass_kg, propellant_kg])

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
            tank=None):
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
                config=None):
    """One stacked figure, shared time axis, phases shaded.

    Agg is selected before pyplot is imported because these runs are
    headless; picking a backend afterwards has no effect.

    all_thrusters=True plots every nozzle including the ones that never
    fired (--all-thrusters); by default those are dropped.

    config, from config_lines(), adds a text panel down the right-hand side
    naming the law, its gains and the headline metrics, so the figure says
    what produced it.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ix = {c: i for i, c in enumerate(cols)}
    col = lambda n: [r[ix[n]] for r in rows]
    t = col("time_s")

    if all_thrusters:
        plotted, hidden = list(names), []
    else:
        plotted, hidden = active_thrusters(rows, cols, names)

    n_panels = 6 if show_mass else 5
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
        ax[5].plot(t, col("mass_kg"), color="#1f5fa8", lw=1.6,
                   label="total mass")
        ax[5].plot(t, col("propellant_kg"), color="#c44", lw=1.4, ls="--",
                   label="propellant remaining")
        ax[5].set_ylabel("mass [kg]")
        ax[5].legend(loc="upper right", fontsize=8)

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

    fig.savefig(path, dpi=110)
    plt.close(fig)
    return path


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

    z_rest = ground_height(model, data, mujoco, body_id)
    place_on_ground(model, data, mujoco, body_id, z_rest)

    prof = Profile(z_rest, args.target, args.descent_rate, dt)
    logger = make_logger(ThrustLogger, thrusters.names)
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
    if wanted_gains:
        print("gains       " + "  ".join(f"{k}={v}" for k, v in
                                          wanted_gains.items()))
    print(f"profile     sit {GROUND_SIT_S}s -> {args.target} m -> hold "
          f"{HOLD_AT_TARGET_S}s -> down at {args.descent_rate} m/s -> "
          f"sit {HOLD_AT_GROUND_S}s")

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

    max_wall = (GROUND_SIT_S + HOLD_AT_TARGET_S + HOLD_AT_GROUND_S
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

            if step_i % ctrl_every == 0:
                state = control_tick(mujoco, model, data, thrusters, body_id,
                                     controller, None)
            else:
                state = CraftState.from_data(model, data, body_id)

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
                               tank.propellant_kg if tank is not None else 0.0)
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
                args.descent_rate, main_max, tank)

    meta = {
        "scene": scene.name, "controller": args.controller,
        "timestamp": stamp, "timestep_s": dt,
        "profile": {
            "target_alt_m": args.target,
            "descent_rate_mps": args.descent_rate,
            "ground_sit_s": GROUND_SIT_S,
            "hold_at_target_s": HOLD_AT_TARGET_S,
            "hold_at_ground_s": HOLD_AT_GROUND_S,
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
            profile=(f"{z_rest:.3f} -> {args.target:g} m, "
                     f"down at {args.descent_rate:g} m/s"),
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
