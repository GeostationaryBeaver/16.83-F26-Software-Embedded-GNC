#!/usr/bin/env python3
"""Print what MuJoCo actually compiled from your XML.

    uv run python scripts/inspect_model.py scenes/02_collisions.xml
    uv run python scripts/inspect_model.py scenes/02_collisions.xml --settle 3

This is the debugging tool. The XML you wrote and the model MuJoCo built
are not the same thing - defaults are applied, masses are computed from
geometry, meshes are replaced by their convex hulls. When a scene
behaves oddly, look here first:

  * a body with 0 DOFs is STATIC - it will never move (missing freejoint)
  * a mass of 0 or an absurd mass means density/size trouble
  * 0 contacts after settling means nothing is touching anything
"""
import argparse
import pathlib
import sys

import mujoco
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent.parent

# mjtGeom enum -> readable name.
GEOM_TYPES = {
    mujoco.mjtGeom.mjGEOM_PLANE: "plane",
    mujoco.mjtGeom.mjGEOM_HFIELD: "hfield",
    mujoco.mjtGeom.mjGEOM_SPHERE: "sphere",
    mujoco.mjtGeom.mjGEOM_CAPSULE: "capsule",
    mujoco.mjtGeom.mjGEOM_ELLIPSOID: "ellipsoid",
    mujoco.mjtGeom.mjGEOM_CYLINDER: "cylinder",
    mujoco.mjtGeom.mjGEOM_BOX: "box",
    mujoco.mjtGeom.mjGEOM_MESH: "mesh",
    mujoco.mjtGeom.mjGEOM_SDF: "sdf",
}


def name_of(model, objtype, idx):
    n = mujoco.mj_id2name(model, objtype, idx)
    return n if n else f"<unnamed {idx}>"


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("scene")
    p.add_argument("--settle", type=float, default=1.0,
                   help="seconds of sim to run before listing contacts "
                        "(default 1.0; use 0 to inspect the initial state)")
    args = p.parse_args()

    scene = pathlib.Path(args.scene)
    if not scene.is_absolute():
        for base in (pathlib.Path.cwd(), ROOT):
            if (base / scene).exists():
                scene = (base / scene).resolve()
                break
    if not scene.exists():
        print(f"error: no such scene: {args.scene}", file=sys.stderr)
        return 1

    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)

    bar = "=" * 72
    print(bar)
    print(f"MODEL  {scene.name}")
    print(bar)

    # ---- mjModel: the constant part -------------------------------------
    print("\nPHYSICS OPTIONS  (from <option>, i.e. model.opt)")
    print(f"  timestep      {model.opt.timestep*1000:.2f} ms "
          f"= {1/model.opt.timestep:.0f} Hz")
    print(f"  gravity       {np.array(model.opt.gravity)} m/s^2")
    print(f"  integrator    {mujoco.mjtIntegrator(model.opt.integrator).name}")
    print(f"  solver        {mujoco.mjtSolver(model.opt.solver).name}, "
          f"{model.opt.iterations} iterations")

    print("\nSIZES")
    print(f"  nbody         {model.nbody}  (includes the world body)")
    print(f"  ngeom         {model.ngeom}")
    print(f"  njnt          {model.njnt}")
    print(f"  nmesh         {model.nmesh}")
    print(f"  nq            {model.nq}   <- generalized POSITION coords")
    print(f"  nv            {model.nv}   <- generalized VELOCITY coords (DOFs)")
    print("                     a freejoint contributes 7 to nq, 6 to nv:")
    print("                     qpos = x y z qw qx qy qz,  qvel = vx vy vz wx wy wz")

    # ---- bodies ----------------------------------------------------------
    print(f"\nBODIES  ({model.nbody - 1} besides the world)")
    print(f"  {'name':<18} {'mass':>9}  {'DOFs':>4}  {'pos (x y z)':<24} kind")
    print(f"  {'-'*18} {'-'*9}  {'-'*4}  {'-'*24} {'-'*14}")
    total_mass = 0.0
    for i in range(model.nbody):
        nm = name_of(model, mujoco.mjtObj.mjOBJ_BODY, i)
        if i == 0:
            print(f"  {'world':<18} {'-':>9}  {0:>4}  {'-':<24} root (static)")
            continue
        mass = model.body_mass[i]
        total_mass += mass
        ndof = model.body_dofnum[i]
        pos = np.array(model.body_pos[i])
        kind = "DYNAMIC" if ndof > 0 else "STATIC (no joint!)"
        print(f"  {nm:<18} {mass:>8.3f}kg  {ndof:>4}  "
              f"{str(pos.round(3)):<24} {kind}")
    print(f"  {'-'*18} {'-'*9}")
    print(f"  {'TOTAL':<18} {total_mass:>8.3f}kg")

    # ---- geoms -----------------------------------------------------------
    print(f"\nGEOMS  ({model.ngeom})")
    print(f"  {'name':<18} {'type':<10} {'size':<22} {'fric':>5} "
          f"{'cond':>4}  collides")
    print(f"  {'-'*18} {'-'*10} {'-'*22} {'-'*5} {'-'*4}  {'-'*8}")
    for i in range(model.ngeom):
        nm = name_of(model, mujoco.mjtObj.mjOBJ_GEOM, i)
        gtype = GEOM_TYPES.get(int(model.geom_type[i]), str(model.geom_type[i]))
        size = np.array(model.geom_size[i]).round(4)
        fric = model.geom_friction[i][0]
        cond = model.geom_condim[i]
        # Both zero => excluded from collision detection entirely.
        collides = "no (ghost)" if (model.geom_contype[i] == 0
                                    and model.geom_conaffinity[i] == 0) else "yes"
        print(f"  {nm:<18} {gtype:<10} {str(size):<22} {fric:>5.2f} "
              f"{cond:>4}  {collides}")

    # ---- meshes ----------------------------------------------------------
    if model.nmesh:
        print(f"\nMESHES  ({model.nmesh})")
        print("  NOTE: a mesh geom collides as its CONVEX HULL, not as the")
        print("  surface you see drawn. 'hull verts' is what physics uses.")
        for i in range(model.nmesh):
            nm = name_of(model, mujoco.mjtObj.mjOBJ_MESH, i)
            nvert = model.mesh_vertnum[i]
            nface = model.mesh_facenum[i]
            print(f"  {nm:<18} {nvert:>7} verts  {nface:>7} faces")

    # ---- mjData: the changing part ---------------------------------------
    print(f"\n{bar}")
    print(f"STATE AFTER {args.settle:.2f}s OF SIMULATION")
    print(bar)

    # mj_forward computes all derived quantities (including xpos) for the
    # current state WITHOUT advancing time. xpos is uninitialized until it
    # runs, so the baseline has to be taken after this call, not before.
    mujoco.mj_forward(model, data)
    z0 = {i: float(data.xpos[i][2])
          for i in range(1, model.nbody) if model.body_dofnum[i] > 0}
    n_steps = int(args.settle / model.opt.timestep)
    for _ in range(n_steps):
        mujoco.mj_step(model, data)

    print(f"\n  stepped {n_steps} times -> t = {data.time:.3f}s")
    print(f"  active contacts: {data.ncon}")

    if data.ncon:
        print(f"\n  {'geom A':<18} {'geom B':<18} {'penetration':>12} "
              f"{'normal force':>13}")
        print(f"  {'-'*18} {'-'*18} {'-'*12} {'-'*13}")
        for c in range(data.ncon):
            con = data.contact[c]
            a = name_of(model, mujoco.mjtObj.mjOBJ_GEOM, con.geom1)
            b = name_of(model, mujoco.mjtObj.mjOBJ_GEOM, con.geom2)
            # dist < 0 means the geoms overlap. MuJoCo's contacts are soft,
            # so a little overlap under load is normal and expected.
            force = np.zeros(6)
            mujoco.mj_contactForce(model, data, c, force)
            print(f"  {a:<18} {b:<18} {con.dist*1000:>10.3f}mm "
                  f"{force[0]:>12.2f}N")
    else:
        print("\n  Nothing is touching. If you expected contact:")
        print("    * is anything actually dynamic? (check the DOFs column)")
        print("    * did it fall past the floor? (timestep too large)")
        print("    * are both geoms set contype=0 conaffinity=0?")
        print("    * try a longer --settle")

    # Vertical motion is the quickest sanity check that gravity worked.
    print("\n  VERTICAL MOTION of dynamic bodies (z at t=0 -> now)")
    for i, z_start in z0.items():
        nm = name_of(model, mujoco.mjtObj.mjOBJ_BODY, i)
        z_now = float(data.xpos[i][2])
        print(f"    {nm:<18} {z_start:>7.3f} -> {z_now:>7.3f} m "
              f"({z_now - z_start:+.3f})")

    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
