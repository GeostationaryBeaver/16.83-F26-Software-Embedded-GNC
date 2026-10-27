#!/usr/bin/env python3
"""Load an MJCF scene and run it - interactively, or headless to PNG/MP4.

    uv run python scripts/run.py scenes/01_ball_drop.xml              # viewer
    uv run python scripts/run.py scenes/02_collisions.xml --headless  # -> out/*.png
    uv run python scripts/run.py scenes/02_collisions.xml --video 5   # -> out/*.mp4
    uv run python scripts/run.py scenes/01_ball_drop.xml --gravity moon

This is deliberately written to be read. The simulation loop is the
plain, idiomatic MuJoCo one and is not hidden behind a class.
"""
import argparse
import os
import pathlib
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent

# Gravity presets, m/s^2. Handy for feeling the difference immediately.
GRAVITY = {
    "earth": -9.81,
    "moon": -1.62,
    "mars": -3.72,
    "zero": 0.0,
}


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("scene", help="path to an MJCF .xml file")
    p.add_argument("--headless", action="store_true",
                   help="render PNG frames instead of opening a window")
    p.add_argument("--video", type=float, metavar="SEC",
                   help="render SEC seconds to an MP4 (implies headless)")
    p.add_argument("--duration", type=float, default=None,
                   help="stop after this many seconds of SIM time")
    p.add_argument("--backend", choices=["glfw", "egl", "osmesa"], default=None,
                   help="OpenGL backend; sets MUJOCO_GL. Default: glfw "
                        "interactive, egl headless. Try osmesa if both fail.")
    p.add_argument("--gravity", default=None,
                   help=f"preset ({'/'.join(GRAVITY)}) or a number in m/s^2")
    p.add_argument("--speed", type=float, default=1.0,
                   help="real-time multiplier for the viewer (2 = double speed)")
    p.add_argument("--fps", type=int, default=30, help="render frame rate")
    args = p.parse_args()

    scene = pathlib.Path(args.scene)
    if not scene.is_absolute():
        # Allow running from either the project root or scripts/.
        for base in (pathlib.Path.cwd(), ROOT):
            if (base / scene).exists():
                scene = (base / scene).resolve()
                break
    if not scene.exists():
        print(f"error: no such scene: {args.scene}", file=sys.stderr)
        return 1

    headless = args.headless or args.video is not None

    # ---- MUJOCO_GL MUST BE SET BEFORE `import mujoco` -------------------
    # MuJoCo picks and links its OpenGL backend at import time, so setting
    # this afterwards has no effect whatsoever. This is why the import sits
    # down here instead of at the top of the file.
    #   glfw   - on-screen window. Works via WSLg on this machine.
    #   egl    - headless, GPU-accelerated. Best for offscreen rendering.
    #   osmesa - headless, pure software. Slow but works anywhere; this is
    #            the fallback when a GPU/driver problem breaks the others.
    backend = args.backend or ("egl" if headless else "glfw")
    os.environ.setdefault("MUJOCO_GL", backend)
    if args.backend:
        os.environ["MUJOCO_GL"] = args.backend

    import mujoco

    # ---- THE TWO CORE OBJECTS -------------------------------------------
    # mjModel: everything CONSTANT about the simulation - the compiled
    #          result of the XML. Bodies, geoms, masses, the physics
    #          options. You can edit fields on it at runtime (see the
    #          gravity change below), but it does not change on its own.
    # mjData:  everything that CHANGES - positions, velocities, forces,
    #          the contact list, the clock. One step mutates this, never
    #          the model.
    # The split is the key idea: one model can back many independent
    # mjData instances (handy for parallel rollouts).
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)

    if args.gravity is not None:
        g = GRAVITY.get(args.gravity.lower())
        if g is None:
            try:
                g = float(args.gravity)
            except ValueError:
                print(f"error: --gravity wants a preset or a number, "
                      f"got {args.gravity!r}", file=sys.stderr)
                return 1
        # model.opt mirrors the <option> element. Writing to it takes
        # effect on the very next step - no recompile needed.
        model.opt.gravity[2] = g
        print(f"gravity overridden: {g:+.2f} m/s^2 on Z")

    print(f"scene       {scene.name}")
    print(f"backend     MUJOCO_GL={os.environ['MUJOCO_GL']}")
    print(f"timestep    {model.opt.timestep*1000:.1f} ms "
          f"({1/model.opt.timestep:.0f} Hz)")
    print(f"gravity     {list(model.opt.gravity)}")
    print(f"bodies      {model.nbody-1} (+world)   geoms {model.ngeom}")
    print(f"DOFs        nq={model.nq} (positions)  nv={model.nv} (velocities)")
    print()

    if args.video is not None:
        return render_video(mujoco, model, data, args)
    if headless:
        return render_stills(mujoco, model, data, args)
    return run_viewer(mujoco, model, data, args)


# =======================================================================
# INTERACTIVE
# =======================================================================
def run_viewer(mujoco, model, data, args) -> int:
    """Open a window and step in real time.

    Uses launch_passive, the non-blocking viewer: it hands control back
    to us so OUR loop drives the physics. The alternative,
    mujoco.viewer.launch(model, data), runs its own loop and blocks -
    simpler, but then you have nowhere to put your own code.
    """
    try:
        import mujoco.viewer
    except ImportError as e:
        print(f"error: viewer unavailable ({e})", file=sys.stderr)
        return 1

    print("viewer controls")
    print("  drag            orbit camera        scroll      zoom")
    print("  right-drag      pan                 space       pause")
    print("  ctrl+drag       apply force to selected body")
    print("  double-click    select a body       backspace   reset")
    print("  ]  [            cycle cameras       F1          help overlay")
    print("  Ctrl+C here or close the window to quit")
    print()

    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # Physics and rendering run at DIFFERENT rates, on purpose.
            # The timestep here is 2 ms (500 Hz), but no screen needs 500
            # frames a second - and syncing the window that often is slow
            # enough to drag the whole sim below real time. So: step
            # physics as often as the clock demands, and only sync at
            # ~60 Hz.
            wall_start = time.perf_counter()
            sim_start = data.time
            next_sync = 0.0
            next_report = 0.5
            sync_interval = 1.0 / 60.0

            while viewer.is_running():
                if args.duration and data.time >= args.duration:
                    break

                # Detect backspace reset (data.time rolled back). MuJoCo's GUI
                # thread fires this outside our loop, so we must re-baseline.
                if data.time < sim_start:
                    wall_now = time.perf_counter()
                    wall_start = wall_now - data.time / max(args.speed, 1e-9)
                    sim_start = data.time
                    next_sync = wall_now
                    next_report = data.time

                # How much sim time SHOULD have elapsed by now.
                wall = time.perf_counter() - wall_start
                target = sim_start + wall * args.speed

                # Catch up to it. The cap stops a slow frame from
                # snowballing into a huge catch-up burst ("spiral of
                # death") - better to run slightly slow than to freeze.
                budget = 0
                while data.time < target and budget < 200:
                    # ---- ONE PHYSICS STEP --------------------------------
                    # mj_step advances time by exactly model.opt.timestep:
                    # it computes forces, solves contacts, and integrates.
                    mujoco.mj_step(model, data)
                    budget += 1

                    # --- poke the state here ----------------------------
                    # This is where you reach in and change things. The
                    # state lives in flat arrays on `data`:
                    #
                    #   data.qpos  positions.   A freejoint takes 7 slots:
                    #              x, y, z, then a quaternion (w, x, y, z).
                    #   data.qvel  velocities.  A freejoint takes 6:
                    #              vx, vy, vz, then angular wx, wy, wz.
                    #   data.xpos  per-body world positions (3 each)
                    #   data.ncon  how many contacts exist right now
                    #
                    # Writing qpos/qvel TELEPORTS the body, and derived
                    # quantities are stale until recomputed - so follow a
                    # write with mj_forward, which refreshes everything
                    # without advancing the clock:
                    #
                    #   if abs(data.time - 1.0) < model.opt.timestep:
                    #       data.qvel[0:3] = [2.0, 0.0, 3.0]   # launch it
                    #       mujoco.mj_forward(model, data)
                    #
                    # Model fields can change mid-run too, gravity included:
                    #
                    #   if data.time > 2.0:
                    #       model.opt.gravity[2] = -1.62       # to the Moon
                    # ------------------------------------------------------

                if data.time >= next_report:
                    print(f"  t={data.time:6.2f}s  contacts={data.ncon:3d}")
                    next_report += 0.5

                # Reflect the new state into the window. Nothing you change
                # on model or data appears on screen until you sync().
                if wall >= next_sync:
                    viewer.sync()
                    next_sync = wall + sync_interval

                # Yield the CPU briefly if we are ahead of schedule, rather
                # than spinning at 100%.
                ahead = (data.time - target) / max(args.speed, 1e-9)
                if ahead > 0:
                    time.sleep(min(ahead, sync_interval))

            elapsed = time.perf_counter() - wall_start
            rate = (data.time - sim_start) / max(elapsed, 1e-9)
            print(f"\nstopped at t={data.time:.2f}s "
                  f"({elapsed:.1f}s wall, {rate:.2f}x real time)")
    except KeyboardInterrupt:
        print("\ninterrupted")
    except Exception as e:  # noqa: BLE001 - we want the hint, not a traceback
        print(f"\nerror: could not open the viewer: {e}", file=sys.stderr)
        print("\nOn WSL this usually means the GL backend failed. Try:", file=sys.stderr)
        print("  --backend osmesa        software rendering, always works",
              file=sys.stderr)
        print("  --headless              skip the window, write PNGs instead",
              file=sys.stderr)
        return 1
    return 0


# =======================================================================
# HEADLESS
# =======================================================================
def render_stills(mujoco, model, data, args) -> int:
    """Step the sim and write a few PNG snapshots."""
    import imageio.v3 as iio

    duration = args.duration or 2.0
    outdir = ROOT / "out"
    outdir.mkdir(exist_ok=True)
    stem = pathlib.Path(args.scene).stem

    # A freshly created MjData has uninitialized derived state: body world
    # positions (xpos) are all zero until something computes them. Rendering
    # before that gives a BLACK FRAME, because every geom is stacked at the
    # origin behind the camera. mj_forward fills it all in without advancing
    # the clock, so the t=0 frame looks like the scene you actually wrote.
    mujoco.mj_forward(model, data)

    # A Renderer owns an offscreen framebuffer. Keep it in a `with` block
    # so the GL context is released promptly.
    with mujoco.Renderer(model, height=720, width=1280) as renderer:
        shots = [0.0, duration * 0.25, duration * 0.5, duration]
        written = []
        for target in shots:
            while data.time < target:
                mujoco.mj_step(model, data)
            # update_scene reads the CURRENT data into the render scene.
            # Forget this and every frame looks identical.
            renderer.update_scene(data, camera=-1)  # -1 = the free camera
            frame = renderer.render()
            path = outdir / f"{stem}_t{target:04.2f}.png"
            iio.imwrite(path, frame)
            written.append(path)
            print(f"  t={data.time:5.2f}s  contacts={data.ncon:3d}  -> {path.name}")

    print(f"\nwrote {len(written)} PNG(s) to {outdir}")
    return 0


def render_video(mujoco, model, data, args) -> int:
    """Step the sim and write an MP4."""
    import imageio

    duration = float(args.video)
    outdir = ROOT / "out"
    outdir.mkdir(exist_ok=True)
    out = outdir / f"{pathlib.Path(args.scene).stem}.mp4"

    # Render one frame every N physics steps so the video plays at the
    # requested fps regardless of the timestep.
    steps_per_frame = max(1, round((1.0 / args.fps) / model.opt.timestep))
    n_frames = int(duration * args.fps)
    print(f"rendering {n_frames} frames at {args.fps} fps "
          f"({steps_per_frame} steps/frame)")

    # Initialize derived state before the first render - see render_stills.
    mujoco.mj_forward(model, data)

    with mujoco.Renderer(model, height=720, width=1280) as renderer:
        with imageio.get_writer(out, fps=args.fps, macro_block_size=1) as writer:
            for i in range(n_frames):
                for _ in range(steps_per_frame):
                    mujoco.mj_step(model, data)
                renderer.update_scene(data, camera=-1)
                writer.append_data(renderer.render())
                if i % args.fps == 0:
                    print(f"  t={data.time:5.2f}s  contacts={data.ncon:3d}")

    print(f"\nwrote {out}  ({out.stat().st_size/1e6:.1f} MB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
