#!/usr/bin/env python3
"""The viewer process: flies a scene in MuJoCo's native viewer, driven over a socket.

Spawned by scripts/gui.py, not usually run by hand:

    uv run python scripts/sim_worker.py --port 45575 \
        --scene scenes/07_thrust_1D.xml --controller example_hover:hover

It owns the GLFW window and nothing else owns an event loop, which is the
whole point - see scripts/simlink.py for the measured heap corruption that
forces the split.

Reuses the existing machinery rather than re-deriving it: control_tick and
load_controller from fly.py, PropellantTank from propellant.py, and run.py's
real-time pacing including the reset re-baselining.
"""
import argparse
import os
import pathlib
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", type=int, required=True)
    p.add_argument("--scene", default="scenes/07_thrust_1D.xml")
    p.add_argument("--controller", default="example_hover:hover")
    p.add_argument("--target", type=float, default=2.0)
    p.add_argument("--propellant", type=float, default=None)
    p.add_argument("--dry-mass", type=float, default=None)
    p.add_argument("--isp", type=float, default=None)
    p.add_argument("--fixed-mass", action="store_true")
    p.add_argument("--gravity", default=None)
    p.add_argument("--speed", type=float, default=1.0)
    args = p.parse_args()

    # Before `import mujoco`: the GL backend is linked at import time.
    os.environ["MUJOCO_GL"] = "glfw"

    import socket
    import mujoco
    import mujoco.viewer
    from simlink import LineReader, send
    from fly import GRAVITY_PRESETS, control_tick, load_controller, resolve_scene
    from thrusters import CraftState, ThrusterSet

    sock = socket.socket()
    sock.connect(("127.0.0.1", args.port))
    sock.setblocking(False)
    link = LineReader(sock)

    scene = resolve_scene(args.scene)
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)

    if args.gravity is not None:
        g = GRAVITY_PRESETS.get(str(args.gravity).lower())
        if g is None:
            g = float(args.gravity)
        model.opt.gravity[2] = g

    thrusters = ThrusterSet(model)
    body_id = thrusters.body_id()
    controller = load_controller(args.controller)

    mod_name = args.controller.split(":")[0]
    mod = sys.modules.get(mod_name)
    setter = getattr(mod, "set_target", None) if mod else None
    if mod is not None and hasattr(mod, "_listener"):
        mod._listener = False          # no stdin thread; we drive the setpoint

    tank = None
    if not args.fixed_mass:
        from propellant import (DRY_MASS_KG, ISP_S, PROPELLANT_KG,
                                PropellantTank)
        tank = PropellantTank(
            model, body_id,
            dry_mass_kg=args.dry_mass if args.dry_mass is not None else DRY_MASS_KG,
            propellant_kg=(args.propellant if args.propellant is not None
                           else PROPELLANT_KG),
            isp_s=args.isp if args.isp is not None else ISP_S)

    mujoco.mj_forward(model, data)
    target = float(args.target)
    if setter is not None:
        setter(target)

    dt = float(model.opt.timestep)
    running = True
    quit_requested = False

    def reset():
        """Back to the scene's initial state, tank refilled."""
        nonlocal target
        mujoco.mj_resetData(model, data)
        if tank is not None:
            tank.propellant_kg = tank.propellant_initial_kg
            tank.used_kg = 0.0
            tank.emptied_at_s = None
            tank._write(model)
            mujoco.mj_setConst(model, data)
        mujoco.mj_forward(model, data)

    # Explicit handle rather than a `with` block, because the exit path
    # matters here - see the end of this function.
    viewer = mujoco.viewer.launch_passive(model, data)
    try:
        if True:
            wall_start = time.perf_counter()
            sim_start = data.time
            next_sync = 0.0
            sync_interval = 1.0 / 60.0

            while viewer.is_running() and not quit_requested:
                # ---- commands ------------------------------------------
                for msg in link.poll():
                    cmd = msg.get("cmd")
                    if cmd == "quit":
                        quit_requested = True
                    elif cmd == "target":
                        target = float(msg["value"])
                        if setter is not None:
                            setter(target)
                    elif cmd == "propellant" and tank is not None:
                        tank.propellant_kg = max(0.0, float(msg["value"]))
                        tank._write(model)
                        mujoco.mj_setConst(model, data)
                    elif cmd == "pause":
                        running = False
                    elif cmd == "resume":
                        running = True
                        wall_start = time.perf_counter()
                        sim_start = data.time
                        # next_sync is measured from wall_start, and
                        # wall_start just moved to now - so wall restarts
                        # near 0 while next_sync still holds its old value.
                        # Leaving it stale gates off rendering AND telemetry
                        # for however long the pause lasted.
                        next_sync = 0.0
                    elif cmd == "reset":
                        reset()
                        wall_start = time.perf_counter()
                        sim_start = data.time
                        next_sync = 0.0
                    elif cmd is not None:
                        send(sock, {"error": f"unknown command {cmd!r}"})
                for err in link.errors():
                    send(sock, {"error": err})
                if link.closed:            # GUI went away
                    break
                if quit_requested:
                    break

                # ---- physics, paced to the wall clock -------------------
                if running:
                    # A backspace reset inside the viewer rolls data.time
                    # back, so re-baseline or the catch-up loop fast-forwards.
                    if data.time < sim_start:
                        now = time.perf_counter()
                        wall_start = now - data.time / max(args.speed, 1e-9)
                        sim_start = data.time
                        next_sync = now - wall_start

                    wall = time.perf_counter() - wall_start
                    goal = sim_start + wall * args.speed
                    budget = 0
                    while data.time < goal and budget < 200:
                        control_tick(mujoco, model, data, thrusters, body_id,
                                     controller, None)
                        if tank is not None:
                            tank.enforce_cutoff(data)
                            tank.update(data, dt)
                        mujoco.mj_step(model, data)
                        budget += 1
                else:
                    wall = time.perf_counter() - wall_start

                # ---- render + telemetry at ~60 Hz ----------------------
                if wall >= next_sync:
                    viewer.sync()
                    next_sync = wall + sync_interval
                    st = CraftState.from_data(model, data, body_id)
                    if not send(sock, {
                            "t": round(float(data.time), 4),
                            "z": round(float(st.pos[2]), 5),
                            "vz": round(float(st.vel[2]), 5),
                            "mass": round(float(model.body_mass[body_id]), 5),
                            "prop": round(tank.propellant_kg, 5) if tank else 0.0,
                            "thrust": round(float(data.ctrl[0]), 4),
                            "setpoint": round(target, 4),
                            "running": running}):
                        break              # control process gone

                if running:
                    ahead = (data.time - goal) / max(args.speed, 1e-9)
                    if ahead > 0:
                        time.sleep(min(ahead, sync_interval))
                else:
                    time.sleep(sync_interval)
    finally:
        try:
            viewer.close()
        except Exception:
            pass
        try:
            sock.close()
        except OSError:
            pass

    # *** os._exit, deliberately. ***  Letting CPython tear down normally
    # after the GLFW viewer has been closed crashes in the interpreter's own
    # cleanup of the viewer's GL resources - measured, reproducibly, with
    # nothing of ours left on the stack: a `with` block hangs forever
    # (exit 124) and an explicit close() segfaults (exit 139, SIGSEGV), both
    # AFTER main() has finished its work. os._exit skips teardown and gives
    # a clean 0.
    #
    # Safe here because this process is a short-lived child whose only
    # resource is the socket just closed above: nothing is buffered, and the
    # scenario runner - not this worker - owns all file output.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)


if __name__ == "__main__":
    sys.exit(main())
