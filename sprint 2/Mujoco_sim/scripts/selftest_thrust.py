#!/usr/bin/env python3
"""Numeric checks on the thrust implementation.

    uv run python scripts/fly.py --selftest

Every check asserts a measured number against the physics, not just that
nothing crashed. If you change thrusters.py, run this.
"""
import pathlib
import sys

import mujoco
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from thrusters import CraftState, ThrustLogger, ThrusterSet  # noqa: E402

# A 1 kg cube in zero gravity with one steerable thruster at its centre:
# the cleanest possible test article, since F = ma with m = 1.
UNIT_XML = """
<mujoco model="unit_test">
  <option timestep="0.001" gravity="0 0 0"/>
  <worldbody>
    <body name="craft" pos="0 0 0">
      <freejoint/>
      <geom type="box" size="0.1 0.1 0.1" mass="1"/>
      <site name="noz" pos="0 0 0"/>
      <site name="noz_off" pos="0.1 0 0"/>
    </body>
  </worldbody>
  <actuator>
    <general name="thr_c" site="noz" gear="0 0 1 0 0 0" ctrlrange="0 60"/>
    <general name="thr_off" site="noz_off" gear="0 0 1 0 0 0" ctrlrange="0 60"/>
  </actuator>
</mujoco>
"""


class Checks:
    def __init__(self):
        self.passed = 0
        self.failed: list[str] = []

    def ok(self, label, got, want, tol, unit=""):
        good = abs(float(got) - float(want)) <= tol
        mark = "ok  " if good else "FAIL"
        print(f"  [{mark}] {label:<44} {float(got):10.5f} "
              f"(want {float(want):.5f} +-{tol:g}){unit}")
        if good:
            self.passed += 1
        else:
            self.failed.append(label)

    def truth(self, label, cond, detail=""):
        mark = "ok  " if cond else "FAIL"
        print(f"  [{mark}] {label:<44} {detail}")
        if cond:
            self.passed += 1
        else:
            self.failed.append(label)


def run_selftest() -> int:
    c = Checks()
    bar = "=" * 74
    print(bar)
    print("THRUST SELF-TEST")
    print(bar)

    m = mujoco.MjModel.from_xml_string(UNIT_XML)
    thr = ThrusterSet(m)
    bid = thr.body_id()

    # -- 1. magnitude: F = ma -------------------------------------------
    print("\n[1] thrust magnitude (10 N on 1 kg, 1.0 s, zero-g)")
    d = mujoco.MjData(m)
    thr["c"].set(m, d, 10.0, [0, 0, 1])
    for _ in range(1000):
        mujoco.mj_step(m, d)
    c.ok("v_z after 1 s", d.qvel[2], 10.0, 0.01, " m/s")
    c.ok("z after 1 s", d.qpos[2], 5.0, 0.01, " m")

    # -- 2. direction is obeyed ------------------------------------------
    print("\n[2] commanded direction is obeyed, magnitude preserved")
    for ang in (0.0, 15.0, 30.0, 60.0):
        d = mujoco.MjData(m)
        thr["c"].gimbal(m, d, 10.0, pitch_deg=ang)
        mujoco.mj_forward(m, d)
        f = np.array(d.qfrc_actuator[:3])
        c.ok(f"force angle at pitch={ang:g}deg",
             np.degrees(np.arctan2(f[0], f[2])), ang, 0.1, " deg")
        c.ok(f"|F| at pitch={ang:g}deg", np.linalg.norm(f), 10.0, 1e-6, " N")

    # -- 3. the normalization trap ---------------------------------------
    print("\n[3] non-unit directions are normalized (gear SCALES force)")
    for dirn in ([0, 0, 1], [0, 0, 2], [1, 1, 1], [3, 4, 0], [0, 0, 1e-3]):
        d = mujoco.MjData(m)
        thr["c"].set(m, d, 10.0, dirn)
        mujoco.mj_forward(m, d)
        c.ok(f"|F| with direction {dirn}",
             np.linalg.norm(d.qfrc_actuator[:3]), 10.0, 1e-6, " N")
    d = mujoco.MjData(m)
    try:
        thr["c"].set(m, d, 10.0, [0, 0, 0])
        c.truth("zero-length direction raises", False, "no exception raised")
    except ValueError:
        c.truth("zero-length direction raises", True, "ValueError")

    # -- 4. clamping ------------------------------------------------------
    print("\n[4] magnitude clamped to ctrlrange [0, 60]")
    d = mujoco.MjData(m)
    for cmd, want in ((999.0, 60.0), (-5.0, 0.0), (25.0, 25.0)):
        c.ok(f"commanding {cmd:g} N", thr["c"].set(m, d, cmd, [0, 0, 1]),
             want, 1e-9, " N")

    # -- 5. thrust is body-frame -----------------------------------------
    print("\n[5] thrust rotates with the body (body-frame gear)")
    d = mujoco.MjData(m)
    d.qpos[3:7] = [np.cos(np.pi / 4), 0.0, np.sin(np.pi / 4), 0.0]  # 90deg /Y
    thr["c"].set(m, d, 20.0, [0, 0, 1])
    mujoco.mj_forward(m, d)
    fw = thr["c"].force_world(m, d)
    c.ok("craft pitched 90deg: world F_x", fw[0], 20.0, 1e-4, " N")
    c.ok("craft pitched 90deg: world F_z", fw[2], 0.0, 1e-4, " N")
    st = CraftState.from_data(m, d, bid)
    c.ok("reported tilt", st.tilt_deg(), 90.0, 1e-3, " deg")

    # -- 6. offset thrust makes torque -----------------------------------
    print("\n[6] offset thruster produces torque (r x F)")
    d = mujoco.MjData(m)
    thr["off"].set(m, d, 10.0, [0, 0, 1])
    mujoco.mj_forward(m, d)
    # site at x=+0.1, force +z  =>  torque about -Y of 0.1*10 = 1.0 N*m
    c.ok("torque about Y", d.qfrc_actuator[4], -1.0, 1e-6, " N*m")
    for _ in range(500):
        mujoco.mj_step(m, d)
    st = CraftState.from_data(m, d, bid)
    c.truth("offset thrust spins the craft", abs(st.angvel[1]) > 0.1,
            f"angvel_y = {st.angvel[1]:+.3f} rad/s")

    # -- 6b. state frames -------------------------------------------------
    # Regression guard. angvel must be BODY frame (what a gyro reads) and
    # vel must be WORLD frame. An earlier version rotated cvel (already
    # world) by the body quaternion, which read correctly only while the
    # craft was upright - exactly the case a hover test cannot catch.
    print("\n[6b] state frame conventions")
    tilt_q = [np.cos(np.pi / 4), np.sin(np.pi / 4), 0.0, 0.0]   # 90deg /X
    d = mujoco.MjData(m)
    d.qpos[3:7] = tilt_q
    d.qvel[3:6] = [0.0, 0.0, 1.0]        # 1 rad/s about the BODY z axis
    mujoco.mj_forward(m, d)
    st = CraftState.from_data(m, d, bid)
    c.ok("angvel is body-frame (z)", st.angvel[2], 1.0, 1e-6, " rad/s")
    c.ok("angvel is body-frame (y)", st.angvel[1], 0.0, 1e-6, " rad/s")
    d = mujoco.MjData(m)
    d.qpos[3:7] = tilt_q
    d.qvel[:3] = [1.0, 2.0, 3.0]
    mujoco.mj_forward(m, d)
    st = CraftState.from_data(m, d, bid)
    c.ok("vel is world-frame (y)", st.vel[1], 2.0, 1e-6, " m/s")
    c.ok("vel is world-frame (z)", st.vel[2], 3.0, 1e-6, " m/s")

    # -- 7 & 8. closed loop on the real scene ----------------------------
    print("\n[7] hover: m*g feedforward holds altitude")
    scene = ROOT / "scenes" / "06_thrust_craft.xml"
    m2 = mujoco.MjModel.from_xml_path(str(scene))
    d2 = mujoco.MjData(m2)
    thr2 = ThrusterSet(m2)
    bid2 = thr2.body_id()
    mujoco.mj_forward(m2, d2)
    st0 = CraftState.from_data(m2, d2, bid2)
    c.ok("hover thrust = m*g", st0.weight_n(), 12.0 * 1.62, 1e-6, " N")
    thr2["main"].set(m2, d2, st0.weight_n(), [0, 0, 1])
    z_start = float(d2.xpos[bid2][2])
    for _ in range(2500):          # 5.0 s
        mujoco.mj_step(m2, d2)
    c.ok("altitude drift over 5 s", d2.xpos[bid2][2] - z_start, 0.0, 1e-3, " m")

    print("\n[8] closed loop: state actually reaches the control law")
    import example_hover
    d3 = mujoco.MjData(m2)
    d3.qpos[2] = example_hover.TARGET_ALT_M - 0.5   # start 0.5 m low
    mujoco.mj_forward(m2, d3)
    err0 = abs(example_hover.TARGET_ALT_M - float(d3.xpos[bid2][2]))
    logger = ThrustLogger(thr2.names)
    ticks = 0
    for _ in range(5000):          # 10 s
        state = CraftState.from_data(m2, d3, bid2)
        thr2.apply(d3, example_hover.hover(state))
        logger.record(state, thr2.magnitudes(d3))
        ticks += 1
        mujoco.mj_step(m2, d3)
    err1 = abs(example_hover.TARGET_ALT_M - float(d3.xpos[bid2][2]))
    print(f"         altitude error {err0:.4f} m -> {err1:.4f} m")
    c.truth("error reduced by >90%", err1 < 0.1 * err0,
            f"{(1 - err1/max(err0,1e-12))*100:.1f}% reduction")

    # -- 8b. RCS geometry: torque without parasitic lift ------------------
    print("\n[8b] RCS quad makes torque, not lift")
    for name, f_axis, t_axis, sign in (("rcs_px", 0, 1, +1),
                                       ("rcs_nx", 0, 1, -1),
                                       ("rcs_py", 1, 0, -1),
                                       ("rcs_ny", 1, 0, +1)):
        d = mujoco.MjData(m2)
        thr2[name].set(m2, d, 20.0)
        mujoco.mj_forward(m2, d)
        q = np.array(d.qfrc_actuator)
        c.ok(f"{name} horizontal force", abs(q[f_axis]), 20.0, 1e-4, " N")
        c.ok(f"{name} vertical force (parasitic lift)", q[2], 0.0, 1e-4, " N")
        c.ok(f"{name} torque", q[3 + t_axis], sign * 5.0, 1e-4, " N*m")

    # -- 8c. authority margin --------------------------------------------
    # The design calculation that makes the attitude loop winnable at all.
    print("\n[8c] attitude authority beats gimbal disturbance")
    rcs_torque = 20.0 * 0.25
    # Worst case: main engine at its 60 N ceiling, canted to MAX_CANT.
    worst_gimbal = 60.0 * (example_hover.MAX_CANT /
                           np.hypot(1.0, example_hover.MAX_CANT)) * 0.25
    print(f"         RCS {rcs_torque:.2f} N*m vs worst-case gimbal "
          f"{worst_gimbal:.2f} N*m")
    c.truth("RCS out-torques the gimbal at full throttle",
            rcs_torque > 1.4 * worst_gimbal,
            f"margin {rcs_torque/worst_gimbal:.2f}x (want >1.4x)")

    # -- 8d. thrust vectoring: translate while staying upright -----------
    # This is the whole feature working at once: steer the main engine to
    # move sideways, and hold attitude with the RCS quad while doing it.
    print("\n[8d] vectored translation holds attitude")
    d4 = mujoco.MjData(m2)
    mujoco.mj_forward(m2, d4)
    max_tilt = 0.0
    for _ in range(10000):          # 20 s
        st = CraftState.from_data(m2, d4, bid2)
        max_tilt = max(max_tilt, st.tilt_deg())
        thr2.apply(d4, example_hover.vectored_translate(st))
        mujoco.mj_step(m2, d4)
    st = CraftState.from_data(m2, d4, bid2)
    c.ok("reached target x", st.pos[0], example_hover.TARGET_X_M, 0.02, " m")
    c.ok("held target altitude", st.pos[2], example_hover.TARGET_ALT_M,
         0.02, " m")
    c.ok("ended upright", st.tilt_deg(), 0.0, 0.5, " deg")
    c.truth("never flipped", max_tilt < 90.0,
            f"peak tilt {max_tilt:.1f} deg")

    # -- 9. logging -------------------------------------------------------
    print("\n[9] CSV log")
    path = logger.save(ROOT / "out" / "selftest_flight.csv")
    text = path.read_text().splitlines()
    c.truth("header row present", text[0].startswith("time_s,main_N"),
            text[0][:46])
    c.ok("one row per control tick", len(text) - 1, ticks, 0, " rows")

    # -- summary ----------------------------------------------------------
    print(f"\n{bar}")
    total = c.passed + len(c.failed)
    if c.failed:
        print(f"FAILED  {len(c.failed)}/{total}")
        for f in c.failed:
            print(f"  - {f}")
        print(bar)
        return 1
    print(f"ALL {total} CHECKS PASSED")
    print(bar)
    return 0


if __name__ == "__main__":
    sys.exit(run_selftest())
