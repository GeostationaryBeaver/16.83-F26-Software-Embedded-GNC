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

import numpy as np

# --- PARAMETERS -----------------------------------------------------------
TARGET_ALT_M = 2.0      # m, commanded altitude
KP_ALT = 18.0           # N per m of altitude error
KD_ALT = 22.0           # N per m/s of vertical speed


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
