#!/usr/bin/env python3
"""Turn an STL/OBJ mesh into MJCF, checking the things that bite you.

    uv run python scripts/import_mesh.py ../misc/pokeball.stl
    uv run python scripts/import_mesh.py "../03 - Checks/base_frame.stl" \
        --name frame --write scenes/04_mesh_import.xml

MuJoCo will happily load a mesh that is wrong in ways it cannot detect,
so this script checks the three that actually cause trouble:

  1. FILE FORMAT   MuJoCo reads BINARY STL only, plus OBJ and MSH.
                   An ASCII STL fails to load with an unhelpful error.
  2. UNITS         MuJoCo is strictly SI - metres. CAD tools export
                   millimetres. Without scale="0.001" a 490 mm part
                   becomes a 490 METRE part, and you spend an afternoon
                   wondering why it falls in slow motion.
  3. CONVEXITY     A mesh geom collides as its CONVEX HULL. Any pocket,
                   vent, bore or open frame is invisible to collision -
                   filled in solid. Nothing warns you.
"""
import argparse
import pathlib
import struct
import sys

import numpy as np
import trimesh

ROOT = pathlib.Path(__file__).resolve().parent.parent


def stl_is_binary(path: pathlib.Path) -> bool | None:
    """True=binary, False=ASCII, None=not an STL.

    An ASCII STL starts with the token 'solid'. That alone is not proof,
    because a binary STL's 80-byte header may also begin with 'solid',
    so confirm using the size the binary layout implies:
    80-byte header + uint32 count + 50 bytes per triangle.
    """
    if path.suffix.lower() != ".stl":
        return None
    with open(path, "rb") as fh:
        head = fh.read(5)
        fh.seek(80)
        count_bytes = fh.read(4)
        if len(count_bytes) < 4:
            return False
        n_tri = struct.unpack("<I", count_bytes)[0]
    if path.stat().st_size == 84 + n_tri * 50:
        return True
    return not head.lower().startswith(b"solid")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("mesh", help="path to an .stl or .obj file")
    p.add_argument("--name", default=None, help="mesh/body name (default: filename)")
    p.add_argument("--scale", type=float, default=None,
                   help="override the auto-detected unit scale")
    p.add_argument("--drop-height", type=float, default=1.0,
                   help="metres above the floor to spawn it (default 1.0)")
    p.add_argument("--mass", type=float, default=None,
                   help="explicit mass in kg (default: MuJoCo's density)")
    p.add_argument("--write", metavar="XML",
                   help="write a complete runnable scene to this path")
    args = p.parse_args()

    src = pathlib.Path(args.mesh)
    if not src.is_absolute():
        for base in (pathlib.Path.cwd(), ROOT):
            if (base / src).exists():
                src = (base / src).resolve()
                break
    if not src.exists():
        print(f"error: no such mesh: {args.mesh}", file=sys.stderr)
        return 1

    name = args.name or src.stem.replace("-", "_").replace(" ", "_")
    bar = "=" * 72
    print(bar)
    print(f"MESH  {src.name}")
    print(bar)

    problems, warnings = [], []

    # ---- 1. FORMAT ------------------------------------------------------
    print("\n[1] FILE FORMAT")
    if src.suffix.lower() not in (".stl", ".obj", ".msh"):
        problems.append(f"MuJoCo does not read {src.suffix} files. "
                        f"Re-export as binary STL or OBJ.")
        print(f"    {src.suffix}  -> UNSUPPORTED")
    else:
        binary = stl_is_binary(src)
        if binary is None:
            print(f"    {src.suffix}  -> supported")
        elif binary:
            print("    binary STL -> supported")
        else:
            problems.append("ASCII STL. MuJoCo reads binary STL only. Convert:\n"
                            f"      python -c \"import trimesh; "
                            f"trimesh.load(r'{src}').export(r'{src.with_suffix('')}"
                            f"_bin.stl')\"")
            print("    ASCII STL -> NOT SUPPORTED by MuJoCo")

    mesh = trimesh.load(src, force="mesh")
    extents = np.asarray(mesh.extents, dtype=float)
    print(f"    {len(mesh.vertices):,} vertices, {len(mesh.faces):,} faces")

    # ---- 2. UNITS -------------------------------------------------------
    print("\n[2] UNITS")
    print(f"    bounding box in FILE units: "
          f"{extents[0]:.2f} x {extents[1]:.2f} x {extents[2]:.2f}")
    biggest = float(extents.max())
    if args.scale is not None:
        scale = args.scale
        print(f"    scale {scale} (given on the command line)")
    elif biggest > 10.0:
        # Nothing you model for a physics sim is >10 m across, so a number
        # this large means the file is in millimetres.
        scale = 0.001
        print(f"    largest dimension is {biggest:.1f} -> these are MILLIMETRES")
        print("    using scale=\"0.001\" to convert mm -> m")
        warnings.append(f"Units inferred as mm. Without scale=\"0.001\" this part "
                        f"would be {biggest:.0f} m across. "
                        f"Override with --scale if that is wrong.")
    else:
        scale = 1.0
        print(f"    largest dimension is {biggest:.3f} -> already METRES")
    scaled = extents * scale
    print(f"    in MuJoCo: {scaled[0]:.3f} x {scaled[1]:.3f} x {scaled[2]:.3f} m")

    # ---- 3. CONVEXITY ---------------------------------------------------
    print("\n[3] CONVEXITY  (what collision will actually see)")
    print(f"    watertight: {mesh.is_watertight}")
    if not mesh.is_watertight:
        warnings.append("Mesh is not watertight. Volume and inertia cannot be "
                        "computed reliably - set an explicit mass with --mass, "
                        "or MuJoCo may infer nonsense inertia.")
    try:
        hull = mesh.convex_hull
        v_mesh, v_hull = abs(float(mesh.volume)), abs(float(hull.volume))
        ratio = v_mesh / v_hull if v_hull > 0 else 0.0
        print(f"    mesh volume {v_mesh*scale**3:.6f} m^3")
        print(f"    hull volume {v_hull*scale**3:.6f} m^3")
        print(f"    mesh/hull   {ratio:.1%}")
        if ratio < 0.75:
            print("\n    *** THE COLLISION SHAPE WILL NOT MATCH THE DRAWING ***")
            print(f"    Only {ratio:.0%} of the hull is solid material, so this")
            print("    part is substantially open or concave. MuJoCo collides it")
            print("    as the filled hull: holes, vents and bores do not exist")
            print("    to the physics, and nothing can pass through or sit")
            print("    inside the part.")
            print("\n    Two ways to fix it:")
            print("      a) Build the collision shape from convex PRIMITIVES in")
            print("         one body (see the compound body in 03_shapes.xml),")
            print("         and keep the mesh purely visual with")
            print("         contype=\"0\" conaffinity=\"0\".")
            print("      b) Split the CAD part into convex pieces, export each,")
            print("         and add one mesh geom per piece in the same body.")
            warnings.append(f"Mesh is only {ratio:.0%} of its convex hull - "
                            f"collision will be a filled block.")
        else:
            print("    near-convex, so the hull is a good collision shape")
    except Exception as e:  # noqa: BLE001
        print(f"    could not compute a convex hull: {e}")

    # ---- the MJCF -------------------------------------------------------
    # MuJoCo resolves mesh file paths relative to <compiler meshdir>, or to
    # the XML's own directory. An absolute path always works.
    mass_attr = f' mass="{args.mass}"' if args.mass else ""
    snippet = f"""  <asset>
    <mesh name="{name}" file="{src}" scale="{scale} {scale} {scale}"/>
  </asset>

  <worldbody>
    <body name="{name}_body" pos="0 0 {args.drop_height}">
      <freejoint/>
      <geom name="g_{name}" type="mesh" mesh="{name}"{mass_attr} rgba="0.7 0.7 0.75 1"/>
    </body>
  </worldbody>"""

    print(f"\n{bar}")
    print("MJCF")
    print(bar)
    print(snippet)

    if problems:
        print(f"\n{bar}")
        print("MUST FIX")
        for i, m in enumerate(problems, 1):
            print(f"  {i}. {m}")
    if warnings:
        print(f"\n{bar}")
        print("WARNINGS")
        for i, m in enumerate(warnings, 1):
            print(f"  {i}. {m}")

    if args.write:
        out = pathlib.Path(args.write)
        if not out.is_absolute():
            out = (pathlib.Path.cwd() / out) if (pathlib.Path.cwd() / out).parent.exists() \
                  else (ROOT / out)
        out.parent.mkdir(parents=True, exist_ok=True)
        hull_note = ""
        if warnings:
            hull_note = "\n".join(f"       - {w}" for w in warnings)
            hull_note = f"\n\n     Reported when this file was generated:\n{hull_note}"
        out.write_text(f"""<!-- ===================================================================
     {out.name}  -  GENERATED by scripts/import_mesh.py
     ===================================================================
     Source mesh: {src}
     Unit scale:  {scale}  (file units -> metres){hull_note}

     Remember: a mesh geom collides as its CONVEX HULL. If this part is
     open or concave, the physics sees a filled block. Use primitives for
     collision and keep the mesh visual-only if that matters to you.

     Run it:  uv run python scripts/run.py {out.as_posix().split('/')[-2]}/{out.name}
     =================================================================== -->

<mujoco model="{name}_import">

  <option timestep="0.002" gravity="0 0 -9.81"/>

  <!-- Bigger offscreen framebuffer, for headless rendering. -->
  <visual>
    <global offwidth="1280" offheight="720"/>
  </visual>

  <asset>
    <texture name="grid" type="2d" builtin="checker"
             rgb1="0.2 0.3 0.4" rgb2="0.1 0.15 0.2" width="300" height="300"/>
    <material name="grid_mat" texture="grid" texrepeat="10 10" reflectance="0.1"/>

    <!-- scale converts the mesh's file units into metres. -->
    <mesh name="{name}" file="{src}" scale="{scale} {scale} {scale}"/>
  </asset>

  <worldbody>
    <light pos="0 0 4" dir="0 0 -1" diffuse="0.8 0.8 0.8"/>
    <geom name="floor" type="plane" size="10 10 0.1" material="grid_mat"/>

    <body name="{name}_body" pos="0 0 {args.drop_height}">
      <freejoint/>
      <geom name="g_{name}" type="mesh" mesh="{name}"{mass_attr} rgba="0.7 0.7 0.75 1"/>
    </body>
  </worldbody>
</mujoco>
""")
        print(f"\nwrote {out}")

    print()
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
