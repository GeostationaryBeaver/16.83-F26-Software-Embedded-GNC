#!/usr/bin/env python3
"""Run a thruster-equipped scene under closed-loop control.

    # hold altitude with the example law, drawing thrust vectors
    uv run python scripts/fly.py scenes/06_thrust_craft.xml \
        --controller example_hover:hover --arrows

    # your own law, logged to CSV
    uv run python scripts/fly.py scenes/06_thrust_craft.xml \
        --controller my_module:my_law --log out/flight.csv

    # no window
    uv run python scripts/fly.py scenes/06_thrust_craft.xml \
        --controller example_hover:hover --headless

    # prove the physics is right
    uv run python scripts/fly.py --selftest

A control law is any function taking a CraftState and returning
{thruster_name: magnitude} or {thruster_name: (magnitude, direction)}.
See scripts/example_hover.py.

This is run.py's sibling: same CLI conventions, same MUJOCO_GL handling,
plus the control loop and thrust visualization.
"""
import argparse
import importlib
import os
import pathlib
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))   # so --controller can import siblings

GRAVITY_PRESETS = {"earth": -9.80665, "moon": -1.62, "mars": -3.72, "zero": 0.0}


def resolve_scene(arg: str) -> pathlib.Path:
    p = pathlib.Path(arg)
    if not p.is_absolute():
        for base in (pathlib.Path.cwd(), ROOT):
            if (base / p).exists():
                return (base / p).resolve()
    return p


def load_controller(spec: str):
    """'module:function' -> the function. Also accepts 'module.function'."""
    if ":" in spec:
        mod_name, fn_name = spec.split(":", 1)
    elif "." in spec:
        mod_name, fn_name = spec.rsplit(".", 1)
    else:
        raise ValueError(
            f"--controller wants 'module:function', got {spec!r}")
    try:
        mod = importlib.import_module(mod_name)
    except ImportError as e:
        raise ImportError(
            f"could not import {mod_name!r} for --controller {spec!r}: {e}\n"
            f"  (scripts/ is on sys.path, so 'example_hover:hover' works)") from None
    try:
        fn = getattr(mod, fn_name)
    except AttributeError:
        avail = [n for n in dir(mod) if not n.startswith("_") and callable(getattr(mod, n))]
        raise AttributeError(
            f"{mod_name!r} has no {fn_name!r}; callables there: "
            f"{', '.join(avail)}") from None
    return fn


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("scene", nargs="?", help="path to an MJCF .xml file")
    p.add_argument("--controller", metavar="MOD:FN",
                   help="control law to run, e.g. example_hover:hover")
    p.add_argument("--duration", type=float, default=10.0,
                   help="seconds of SIM time to run (default 10)")
    p.add_argument("--control-hz", type=float, default=None,
                   help="run the control law at this rate instead of every "
                        "physics step - a real flight computer is slower "
                        "than the physics, and this lets you feel that")
    p.add_argument("--headless", action="store_true", help="render PNGs, no window")
    p.add_argument("--video", type=float, metavar="SEC", help="render SEC seconds to MP4")
    p.add_argument("--arrows", action="store_true",
                   help="draw a live thrust vector per thruster")
    p.add_argument("--log", metavar="CSV", help="write a per-tick CSV log here")
    p.add_argument("--backend", choices=["glfw", "egl", "osmesa"], default=None)
    p.add_argument("--gravity", default=None,
                   help=f"preset ({'/'.join(GRAVITY_PRESETS)}) or m/s^2")
    p.add_argument("--speed", type=float, default=1.0, help="real-time multiplier")
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--camera", default=None,
                   help="named camera to render from (e.g. 'chase'); the "
                        "default free camera zooms out to fit the whole "
                        "scene, which can make thrust arrows tiny")
    p.add_argument("--selftest", action="store_true",
                   help="verify the thrust physics numerically and exit")
    args = p.parse_args()

    headless = args.headless or args.video is not None

    # MUJOCO_GL must be set BEFORE `import mujoco` - the backend is linked
    # at import time, so a later change has no effect. Hence the late import.
    backend = args.backend or ("egl" if headless else "glfw")
    os.environ["MUJOCO_GL"] = backend if args.backend else os.environ.get(
        "MUJOCO_GL", backend)

    if args.selftest:
        from selftest_thrust import run_selftest
        return run_selftest()

    if not args.scene:
        p.error("a scene is required (or use --selftest)")

    scene = resolve_scene(args.scene)
    if not scene.exists():
        print(f"error: no such scene: {args.scene}", file=sys.stderr)
        return 1

    import mujoco
    import numpy as np
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
        print(f"error: {scene.name} declares no thrusters.\n"
              f"  A thruster is a <general> actuator bound to a <site>, named\n"
              f"  with the 'thr_' prefix. See scenes/06_thrust_craft.xml.",
              file=sys.stderr)
        return 1

    body_id = thrusters.body_id()
    try:
        controller = load_controller(args.controller) if args.controller else None
    except (ImportError, AttributeError, ValueError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1

    # Derived state must exist before the first control tick or the first
    # render: a fresh MjData has xpos all zeros until something computes it.
    mujoco.mj_forward(model, data)
    state0 = CraftState.from_data(model, data, body_id)

    print(f"scene       {scene.name}")
    print(f"backend     MUJOCO_GL={os.environ['MUJOCO_GL']}")
    print(f"timestep    {model.opt.timestep*1000:.1f} ms "
          f"({1/model.opt.timestep:.0f} Hz)")
    print(f"gravity     {list(np.array(model.opt.gravity).round(3))} m/s^2")
    print(f"craft       {state0.mass:.2f} kg -> hover needs "
          f"{state0.weight_n():.2f} N")
    print(thrusters.describe())
    ctrl_hz = args.control_hz or (1.0 / model.opt.timestep)
    print(f"controller  {args.controller or '(none - thrusters idle)'} "
          f"@ {ctrl_hz:.0f} Hz")
    print()

    logger = ThrustLogger(thrusters.names) if args.log else None
    ctx = dict(model=model, data=data, thrusters=thrusters, body_id=body_id,
               controller=controller, logger=logger, args=args)

    if args.video is not None:
        rc = render_video(mujoco, np, **ctx)
    elif headless:
        rc = render_stills(mujoco, np, **ctx)
    else:
        rc = run_viewer(mujoco, np, **ctx)

    if logger is not None and len(logger):
        path = logger.save(args.log if pathlib.Path(args.log).is_absolute()
                           else ROOT / args.log)
        print(f"logged {len(logger)} ticks -> {path}")
    return rc


# =======================================================================
# THE CONTROL LOOP
# =======================================================================
def control_tick(mujoco, model, data, thrusters, body_id, controller,
                 logger, mass=None):
    """One closed-loop tick: read state -> decide -> command.

    This is the whole idea, and it is only four lines:

        state = CraftState.from_data(...)   # 1. what is the craft doing?
        cmds  = controller(state)           # 2. what should it do?
        thrusters.apply(data, cmds)         # 3. write ctrl + gear
        mujoco.mj_step(model, data)         # 4. integrate (done by caller)

    The commands persist in data.ctrl and model.actuator_gear until
    something overwrites them, so a thruster keeps firing between ticks.
    That is a zero-order hold, and it is what real hardware does too.

    Running the law less often than the physics (--control-hz) is more
    realistic: a flight computer at 50 Hz cannot correct a 500 Hz
    disturbance. The alternative is mujoco.set_mjcb_control, which MuJoCo
    calls inside mj_step - necessary if you use the RK4 integrator and
    want the law evaluated at substeps too.
    """
    from thrusters import CraftState
    state = CraftState.from_data(model, data, body_id, mass)
    if controller is not None:
        cmds = controller(state)
        if cmds:
            thrusters.apply(data, cmds)
    if logger is not None:
        logger.record(state, thrusters.magnitudes(data))
    return state


def draw_thrust_arrows(mujoco, np, scene_obj, model, data, thrusters,
                       full_scale_m=1.1, min_frac=0.10):
    """Draw one arrow per firing thruster, into a render or viewer scene.

    Arrows read as exhaust plumes: they start at the nozzle and point
    OPPOSITE the force, because that is the direction mass leaves the
    vehicle. The craft is pushed the other way.

    Length is scaled by THROTTLE FRACTION, not by absolute Newtons. A
    5 N RCS thruster at full tilt and a 60 N main engine at full tilt
    both draw a full-length arrow, so you can see what every thruster is
    doing in one glance. Scaling by absolute force instead makes the RCS
    arrows ~8 cm and effectively invisible next to the main engine.

    Arrows are added as extra scene geoms. Two things to know if you
    write your own:
      * mjv_initGeom first (it sets colour and clears the geom), then
        mjv_connector to place it between two points.
      * respect scene_obj.maxgeom, or you silently overrun the buffer.
    """
    n = scene_obj.ngeom
    for t in thrusters:
        mag = t.magnitude(data)
        if mag <= 1e-9 or n >= scene_obj.maxgeom:
            continue
        f_world = t.force_world(model, data)
        norm = float(np.linalg.norm(f_world))
        if norm < 1e-9:
            continue

        # Throttle fraction, so small thrusters stay visible.
        frac = mag / t.max_thrust_n if t.max_thrust_n > 0 else 1.0
        length = full_scale_m * max(frac, min_frac)
        width = 0.02 + 0.02 * frac

        start = t.site_world(data)
        end = start - (f_world / norm) * length

        g = scene_obj.geoms[n]
        mujoco.mjv_initGeom(
            g, int(mujoco.mjtGeom.mjGEOM_ARROW), np.zeros(3), np.zeros(3),
            np.eye(3).flatten(),
            np.array([1.0, 0.45, 0.1, 0.95], dtype=np.float32))
        mujoco.mjv_connector(g, int(mujoco.mjtGeom.mjGEOM_ARROW), width,
                             start, end)
        n += 1
    scene_obj.ngeom = n


# =======================================================================
# INTERACTIVE
# =======================================================================
def run_viewer(mujoco, np, model, data, thrusters, body_id, controller,
               logger, args):
    try:
        import mujoco.viewer
    except ImportError as e:
        print(f"error: viewer unavailable ({e})", file=sys.stderr)
        return 1

    print("viewer: drag=orbit  scroll=zoom  space=pause  ctrl+drag=push")
    if args.arrows:
        print("        orange arrows = live thrust vectors")
    print()

    ctrl_every = max(1, round((1.0 / args.control_hz) / model.opt.timestep)) \
        if args.control_hz else 1
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            wall_start = time.perf_counter()
            sim_start = data.time
            next_sync = 0.0
            next_report = 0.0
            sync_interval = 1.0 / 60.0
            step_i = 0

            while viewer.is_running():
                if args.duration and data.time - sim_start >= args.duration:
                    break

                # Detect backspace reset (data.time rolled back). MuJoCo's GUI
                # thread fires this outside our loop, so we must re-baseline.
                if data.time < sim_start:
                    wall_now = time.perf_counter()
                    wall_start = wall_now - data.time / max(args.speed, 1e-9)
                    sim_start = data.time
                    next_sync = wall_now
                    next_report = 0.0

                wall = time.perf_counter() - wall_start
                target = sim_start + wall * args.speed

                budget = 0
                while data.time < target and budget < 200:
                    if step_i % ctrl_every == 0:
                        control_tick(mujoco, model, data, thrusters, body_id,
                                     controller, logger)
                    mujoco.mj_step(model, data)
                    step_i += 1
                    budget += 1

                if data.time >= next_report:
                    from thrusters import CraftState
                    st = CraftState.from_data(model, data, body_id)
                    mags = thrusters.magnitudes(data)
                    active = " ".join(f"{k}={v:.1f}N" for k, v in mags.items()
                                      if v > 1e-6) or "all off"
                    print(f"  {st}  | {active}")
                    next_report += 0.5

                if wall >= next_sync:
                    if args.arrows:
                        viewer.user_scn.ngeom = 0
                        draw_thrust_arrows(mujoco, np, viewer.user_scn,
                                           model, data, thrusters)
                    viewer.sync()
                    next_sync = wall + sync_interval

                ahead = (data.time - target) / max(args.speed, 1e-9)
                if ahead > 0:
                    time.sleep(min(ahead, sync_interval))

            elapsed = time.perf_counter() - wall_start
            rate = (data.time - sim_start) / max(elapsed, 1e-9)
            print(f"\nstopped at t={data.time:.2f}s "
                  f"({elapsed:.1f}s wall, {rate:.2f}x real time)")
    except KeyboardInterrupt:
        print("\ninterrupted")
    except Exception as e:  # noqa: BLE001
        print(f"\nerror: could not open the viewer: {e}", file=sys.stderr)
        print("try --backend osmesa, or --headless", file=sys.stderr)
        return 1
    return 0


# =======================================================================
# HEADLESS
# =======================================================================
def render_stills(mujoco, np, model, data, thrusters, body_id, controller,
                  logger, args):
    import imageio.v3 as iio
    from thrusters import CraftState

    duration = args.duration or 5.0
    outdir = ROOT / "out"
    outdir.mkdir(exist_ok=True)
    stem = pathlib.Path(args.scene).stem
    ctrl_every = max(1, round((1.0 / args.control_hz) / model.opt.timestep)) \
        if args.control_hz else 1

    cam = args.camera if args.camera is not None else -1
    with mujoco.Renderer(model, height=720, width=1280) as renderer:
        scene_obj = renderer.scene
        shots = [0.0, duration * 0.25, duration * 0.5, duration]
        step_i = 0
        for target in shots:
            while data.time < target:
                if step_i % ctrl_every == 0:
                    control_tick(mujoco, model, data, thrusters, body_id,
                                 controller, logger)
                mujoco.mj_step(model, data)
                step_i += 1
            renderer.update_scene(data, camera=cam)
            if args.arrows:
                draw_thrust_arrows(mujoco, np, scene_obj, model, data, thrusters)
            frame = renderer.render()
            path = outdir / f"{stem}_t{target:05.2f}.png"
            iio.imwrite(path, frame)
            st = CraftState.from_data(model, data, body_id)
            print(f"  {st}  -> {path.name}")
    print(f"\nwrote {len(shots)} PNG(s) to {outdir}")
    return 0


def render_video(mujoco, np, model, data, thrusters, body_id, controller,
                 logger, args):
    import imageio
    from thrusters import CraftState

    duration = float(args.video)
    outdir = ROOT / "out"
    outdir.mkdir(exist_ok=True)
    out = outdir / f"{pathlib.Path(args.scene).stem}_flight.mp4"
    steps_per_frame = max(1, round((1.0 / args.fps) / model.opt.timestep))
    n_frames = int(duration * args.fps)
    ctrl_every = max(1, round((1.0 / args.control_hz) / model.opt.timestep)) \
        if args.control_hz else 1
    cam = args.camera if args.camera is not None else -1
    print(f"rendering {n_frames} frames at {args.fps} fps")

    step_i = 0
    with mujoco.Renderer(model, height=720, width=1280) as renderer:
        with imageio.get_writer(out, fps=args.fps, macro_block_size=1) as writer:
            for i in range(n_frames):
                for _ in range(steps_per_frame):
                    if step_i % ctrl_every == 0:
                        control_tick(mujoco, model, data, thrusters, body_id,
                                     controller, logger)
                    mujoco.mj_step(model, data)
                    step_i += 1
                renderer.update_scene(data, camera=cam)
                if args.arrows:
                    draw_thrust_arrows(mujoco, np, renderer.scene, model,
                                       data, thrusters)
                writer.append_data(renderer.render())
                if i % args.fps == 0:
                    print(f"  {CraftState.from_data(model, data, body_id)}")
    print(f"\nwrote {out}  ({out.stat().st_size/1e6:.1f} MB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
