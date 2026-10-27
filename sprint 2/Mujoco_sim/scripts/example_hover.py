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
import sys
import threading

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


# --- live-retargetable hover ----------------------------------------------
# Same law as hover() above; the only difference is where the setpoint comes
# from. hover() reads the TARGET_ALT_M constant, so changing it means editing
# this file and restarting. hover_interactive() reads a module-level variable
# that a background thread updates from whatever you type in the terminal.
ALT_MIN_M = 0.3         # m, floor clearance - refuse targets below this
ALT_MAX_M = 15.0        # m, keeps the craft in frame and inside thrust authority
ALT_STEP_M = 0.5        # m, nudge applied by a bare "+" or "-"

_target_alt = TARGET_ALT_M   # the live setpoint, written by the reader thread
_listener = None             # stdin thread; started on the first control tick


def _parse_target(line, current):
    """Turn one typed line into a new target altitude, or None for no change.

    Pure function - no thread, no I/O beyond printing - so it can be tested
    directly. Accepts an absolute value ("3.5"), a relative step ("+0.5",
    "-1"), a bare nudge ("+", "-"), or "?" to report the current target.

    Out-of-range requests are clamped AND announced. Silently clamping would
    leave you typing 20 and wondering why the craft stops at 15.
    """
    s = line.strip()
    if not s:
        return None
    if s in ("?", "h", "help"):
        print(f"  [alt] target is {current:.2f} m   "
              f"(type a number, or +/-, range {ALT_MIN_M}-{ALT_MAX_M} m)")
        return None
    if s == "+":
        want = current + ALT_STEP_M
    elif s == "-":
        want = current - ALT_STEP_M
    else:
        try:
            want = current + float(s) if s[0] in "+-" else float(s)
        except ValueError:
            print(f"  [alt] not a number: {s!r}  (try 3.5, +0.5, -1, or ?)")
            return None
    clamped = min(max(want, ALT_MIN_M), ALT_MAX_M)
    if clamped != want:
        print(f"  [alt] {want:.2f} m is outside "
              f"{ALT_MIN_M}-{ALT_MAX_M} m; using {clamped:.2f} m")
    print(f"  [alt] target -> {clamped:.2f} m")
    return clamped


def _start_listener():
    """Start the stdin reader, or report why we did not.

    Deliberately lazy: selftest_thrust.py imports this module, so starting a
    thread at import time would have --selftest competing for stdin. It is
    started on the first control tick instead.

    With no TTY (--headless, --video, piped input) there is nothing to read,
    so no thread is started and the target simply stays at its default. A
    thread looping on a closed stdin would spin on an immediate empty return.
    """
    try:
        interactive = sys.stdin is not None and sys.stdin.isatty()
    except (ValueError, AttributeError):
        interactive = False

    if not interactive:
        print(f"  [alt] no terminal attached; target fixed at "
              f"{_target_alt:.2f} m")
        return False

    def reader():
        global _target_alt
        for line in sys.stdin:                 # ends when stdin closes
            new = _parse_target(line, _target_alt)
            if new is not None:
                _target_alt = new              # atomic under the GIL

    t = threading.Thread(target=reader, daemon=True, name="alt-input")
    t.start()
    print(f"  [alt] target {_target_alt:.2f} m - type a new altitude "
          f"(or +/-, ?) and press Enter")
    return t


def set_target(alt_m):
    """Command a new target altitude. The scenario-runner convention.

    A control law keeps its setpoint wherever that law chose to keep it, so
    an outside driver has no way to guess the variable's name. A module that
    defines set_target() can be driven by scripts/scenario.py; one that does
    not has to be poked by hand.

    Both names are assigned because the laws in this module disagree about
    where the setpoint lives: hover() and vectored_translate() read the
    TARGET_ALT_M constant, while hover_interactive() reads the _target_alt
    global that its stdin thread writes. Setting both makes all three
    drivable through one call.
    """
    global _target_alt, TARGET_ALT_M
    TARGET_ALT_M = float(alt_m)
    _target_alt = float(alt_m)


def hover_interactive(state):
    """hover(), but the target altitude can be changed while the sim runs.

    Type a number in the terminal and press Enter. Everything else - the
    feedforward/P/D structure, the gains, the vertical-only assumption - is
    exactly hover(), so this is purely a demonstration of a live setpoint.

        uv run python scripts/fly.py scenes/06_thrust_craft.xml \
            --controller example_hover:hover_interactive --duration 60

    WORTH WATCHING: KP_ALT and KD_ALT were tuned for the small errors of
    holding an altitude, not for large setpoint jumps. Measured on
    06_thrust_craft, stepping the target from a settled hover:

        2 -> 3 m    overshoots 0.029 m, actuator never saturates
        3 -> 12 m   overshoots 1.193 m, actuator pinned at 60 N for 1.12 s

    A 9 m error asks KP_ALT for 162 N from a 60 N engine, so the command
    saturates: the loop is open until the error shrinks, and the D term only
    starts arresting the climb once it is too late. That is a real property
    of a PD loop against a saturating actuator, not a bug - and being able
    to retarget live is what makes it visible. A real vehicle fixes it by
    ramping the setpoint (flying a trajectory) rather than stepping it.
    """
    global _listener
    if _listener is None:
        _listener = _start_listener()

    err_alt = _target_alt - state.pos[2]
    err_vel = -state.vel[2]

    thrust = state.weight_n() + KP_ALT * err_alt + KD_ALT * err_vel

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
