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
                    + [f"ctrl_{n}" for n in self.names])

        def record_tick(self, state, magnitudes, setpoint, accel, qacc,
                        phase, ctrls):
            super().record(state, magnitudes)
            self.rows[-1].extend([setpoint, accel, qacc, phase]
                                 + [float(c) for c in ctrls])

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


def metrics(rows, cols, prof, target, z_ground, descent_rate, max_thrust):
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
    return out


def plot_flight(path, rows, cols, prof, target, z_ground, descent_rate, names):
    """One stacked figure, shared time axis, phases shaded.

    Agg is selected before pyplot is imported because these runs are
    headless; picking a backend afterwards has no effect.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ix = {c: i for i, c in enumerate(cols)}
    col = lambda n: [r[ix[n]] for r in rows]
    t = col("time_s")

    fig, ax = plt.subplots(5, 1, figsize=(11, 14), sharex=True,
                           constrained_layout=True)

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

    for n in names:
        ax[3].plot(t, col(f"{n}_N"), lw=1.3, label=n)
    ax[3].set_ylabel("thrust applied [N]")
    ax[3].legend(loc="upper right", fontsize=8, ncol=max(1, len(names) // 2))

    for n in names:
        ax[4].plot(t, col(f"ctrl_{n}"), lw=1.3, label=f"ctrl_{n}")
    ax[4].set_ylabel("control input [N]")
    ax[4].set_xlabel("time [s]")
    ax[4].legend(loc="upper right", fontsize=8, ncol=max(1, len(names) // 2))

    fig.savefig(path, dpi=110)
    plt.close(fig)
    return path


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
    p.add_argument("--gravity", default=None, help="preset or m/s^2")
    p.add_argument("--selftest", action="store_true",
                   help="verify the profile machine numerically and exit")
    args = p.parse_args()

    if args.selftest:
        return run_selftest()

    # Headless throughout; and MUJOCO_GL must be set before mujoco imports.
    os.environ.setdefault("MUJOCO_GL", "osmesa")

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

    dt = float(model.opt.timestep)
    qi, vi = vertical_indices(model, mujoco, body_id)
    z_rest = ground_height(model, data, mujoco, body_id)
    place_on_ground(model, data, mujoco, body_id, z_rest)

    prof = Profile(z_rest, args.target, args.descent_rate, dt)
    logger = make_logger(ThrustLogger, thrusters.names)
    ctrl_every = max(1, round((1.0 / args.control_hz) / dt)) if args.control_hz else 1

    print(f"scene       {scene.name}")
    print(f"controller  {args.controller}")
    print(f"ground      z = {z_rest:.6f} m (derived)   ncon at start "
          f"{data.ncon}  vz {float(data.qvel[vi]):+.2e}")
    print(f"profile     sit {GROUND_SIT_S}s -> {args.target} m -> hold "
          f"{HOLD_AT_TARGET_S}s -> down at {args.descent_rate} m/s -> "
          f"sit {HOLD_AT_GROUND_S}s")
    print()

    max_wall = (GROUND_SIT_S + HOLD_AT_TARGET_S + HOLD_AT_GROUND_S
                + prof._timeout_for("ASCEND") + prof._timeout_for("DESCEND") + 5.0)
    vz_prev = float(data.qvel[vi])
    step_i = 0
    last_phase = prof.phase

    while not prof.done and data.time < max_wall:
        z = float(data.xpos[body_id][2])
        vz = float(data.qvel[vi])
        setpoint = prof.update(data.time, z, vz)
        setter(setpoint)

        if step_i % ctrl_every == 0:
            state = control_tick(mujoco, model, data, thrusters, body_id,
                                 controller, None)
        else:
            state = CraftState.from_data(model, data, body_id)

        accel = (vz - vz_prev) / dt
        logger.record_tick(state, thrusters.magnitudes(data), setpoint, accel,
                           float(data.qacc[vi]), last_phase, list(data.ctrl))
        vz_prev = vz

        if prof.phase != last_phase:
            print(f"  t={data.time:6.2f}s  {last_phase:>11s} -> "
                  f"{prof.phase:<11s}  z={z:7.4f}  vz={vz:+7.4f}")
            last_phase = prof.phase

        mujoco.mj_step(model, data)
        step_i += 1

    # ---- outputs ---------------------------------------------------------
    stamp = _dt.datetime.now().strftime(STAMP_FMT)
    fn = args.controller.replace(":", "_").replace(".", "_")
    name = f"{scene.stem}_{fn}_{stamp}" + (f"_{args.label}" if args.label else "")
    parent = pathlib.Path(args.outdir) if args.outdir else (ROOT / "outputs")
    rundir = parent / name
    rundir.mkdir(parents=True, exist_ok=True)

    cols = logger.header
    csv_path = logger.save(rundir / "flight.csv")

    main_max = thrusters["main"].max_thrust_n if "main" in thrusters.names else 0.0
    m = metrics(logger.rows, cols, prof, args.target, z_rest,
                args.descent_rate, main_max)

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
        "phase_boundaries": [{"t": t, "from": a, "to": b}
                             for t, a, b in prof.boundaries],
        "metrics": m, "ticks": len(logger), "completed": prof.done,
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
        plot_flight(rundir / "flight.png", logger.rows, cols, prof,
                    args.target, z_rest, args.descent_rate, thrusters.names)

    print()
    if prof.timeouts:
        print(f"TIMED OUT in: {', '.join(prof.timeouts)}")
    print(f"peak {m['peak_altitude_m']:.4f} m  overshoot {m['overshoot_m']:.4f} m  "
          f"hold RMS err {m['hold_rms_err_m']:.5f} m")
    print(f"peak descent {m['peak_descent_rate_mps']:+.4f} m/s "
          f"(commanded {m['commanded_descent_rate_mps']:+.4f})")
    print(f"returned to {m['final_altitude_m']:.4f} m "
          f"(ground {z_rest:.4f}, err {m['return_err_m']:.5f})")
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

    print("\n[5] end to end on both scenes")
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
