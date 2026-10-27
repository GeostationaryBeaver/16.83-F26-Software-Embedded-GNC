#!/usr/bin/env python3
"""Thrust vectors with runtime-steerable magnitude and direction.

A thruster in MuJoCo is a <site> (where the force is applied) plus a
<general> actuator bound to it (what applies the force). The actuator's
`gear` attribute is the force direction in the site frame:

    data.ctrl[i]               -> MAGNITUDE, Newtons
    model.actuator_gear[i][:3] -> DIRECTION, unit vector in the site frame

Both are writable every step, so a thruster can be throttled AND steered
in flight. This module wraps that pair so you work in (magnitude,
direction) instead of raw array indices.

    from thrusters import ThrusterSet, CraftState

    thr = ThrusterSet(model)
    thr["main"].set(data, 19.44)                     # straight down the axis
    thr["main"].set(data, 25.0, direction=[0.1, 0, 1])  # vectored

    state = CraftState.from_data(model, data, thr.total_mass(model))
    thr.apply(data, my_control_law(state))

*** THE TRAP this module exists to prevent ***
`gear` SCALES the force, it does not merely aim it. gear=[0,0,2] with
ctrl=10 gives 20 N, and gear=[1,1,1] gives 10*sqrt(3) = 17.32 N. Every
direction here is normalized before it is written, so commanded Newtons
are the Newtons you get.

Conventions, matching the course material (base_frame.py and the GNC
firmware): Z is up, the main thrust axis is X = Y = 0, and everything is
SI internally - metres, kilograms, seconds, Newtons, radians. Angles are
accepted in DEGREES at the interface (gimbal(), euler_deg) and converted
immediately, which is the pattern base_frame.py already uses.
"""
from __future__ import annotations

import csv
import math
import pathlib
from dataclasses import dataclass, field

import mujoco
import numpy as np

# --- PARAMETERS -----------------------------------------------------------
# One labeled block, unit suffix in every name, source in the comment -
# the convention from assignment1/lunar-hopper/02-cad/base_frame.py.

MASS_KG = 12.0            # kg, the abstract craft in scenes/06_thrust_craft.xml
THRUST_MAX_N = 60.0       # N, main engine ceiling; matches that scene's ctrlrange
RCS_MAX_N = 5.0           # N, per attitude thruster in that scene

G_MOON_MS2 = 1.62         # m/s^2, lunar surface
G_MARS_MS2 = 3.72         # m/s^2, Mars surface
G_EARTH_MS2 = 9.80665     # m/s^2, standard gravity; the same constant the
                          #   load-cell firmware uses for its calibration

THRUSTER_PREFIX = "thr_"  # actuator-name prefix that marks a thruster, so
                          #   thr_main is reachable as thrusters["main"]

_EPS = 1e-12              # guards normalization against a zero-length vector


# =========================================================================
# ONE THRUSTER
# =========================================================================
class Thruster:
    """A single site actuator, addressed as (magnitude, direction).

    You normally get these from a ThrusterSet rather than building them
    directly.
    """

    __slots__ = ("name", "actuator_name", "actuator_id", "site_id",
                 "body_id", "min_thrust_n", "max_thrust_n", "_nominal_dir")

    def __init__(self, model: mujoco.MjModel, actuator_id: int):
        self.actuator_id = int(actuator_id)
        self.actuator_name = mujoco.mj_id2name(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, self.actuator_id) or f"act{actuator_id}"
        # Friendly name: thr_main -> main
        self.name = (self.actuator_name[len(THRUSTER_PREFIX):]
                     if self.actuator_name.startswith(THRUSTER_PREFIX)
                     else self.actuator_name)

        # actuator_trnid[i][0] is the transmission target - the site id for
        # a site actuator.
        self.site_id = int(model.actuator_trnid[self.actuator_id][0])
        self.body_id = int(model.site_bodyid[self.site_id])

        lo, hi = model.actuator_ctrlrange[self.actuator_id]
        self.min_thrust_n = float(lo)
        self.max_thrust_n = float(hi)

        # The direction declared in the XML, so reset() can restore it.
        self._nominal_dir = np.array(
            model.actuator_gear[self.actuator_id][:3], dtype=float)
        n = np.linalg.norm(self._nominal_dir)
        if n > _EPS:
            self._nominal_dir = self._nominal_dir / n

    # ---- commanding --------------------------------------------------
    def set(self, model: mujoco.MjModel, data: mujoco.MjData,
            magnitude: float, direction=None) -> float:
        """Command this thruster. Returns the magnitude actually applied.

        magnitude  Newtons. Clamped to the actuator's ctrlrange.
        direction  optional 3-vector in the SITE frame. Normalized before
                   use, so its length is irrelevant - only its heading
                   matters. Omit it to keep the current direction.
        """
        if direction is not None:
            d = np.asarray(direction, dtype=float).reshape(3)
            n = float(np.linalg.norm(d))
            if n < _EPS:
                raise ValueError(
                    f"thruster {self.name!r}: direction has zero length; "
                    "a thrust vector needs a heading")
            # NORMALIZE. gear scales force, so a non-unit vector would
            # silently change the magnitude.
            model.actuator_gear[self.actuator_id][:3] = d / n

        # Clamp here as well as relying on MuJoCo's own ctrlrange clamp, so
        # the value we report and log is the value that was applied.
        m = float(np.clip(magnitude, self.min_thrust_n, self.max_thrust_n))
        data.ctrl[self.actuator_id] = m
        return m

    def set_throttle(self, model: mujoco.MjModel, data: mujoco.MjData,
                     fraction: float, direction=None) -> float:
        """Command as a 0..1 fraction of max thrust.

        Mirrors the setPower(0..1) convention in the course's valve
        firmware (MAC_35_driver_v1.ino), which is how the real hardware is
        commanded.
        """
        frac = float(np.clip(fraction, 0.0, 1.0))
        return self.set(model, data, frac * self.max_thrust_n, direction)

    def gimbal(self, model: mujoco.MjModel, data: mujoco.MjData,
               magnitude: float, pitch_deg: float = 0.0,
               yaw_deg: float = 0.0) -> float:
        """Aim by angle instead of by vector.

        pitch_deg tilts toward +X, yaw_deg toward +Y, both measured from
        the thruster's nominal axis. Degrees in, radians internally.
        """
        p = math.radians(pitch_deg)
        y = math.radians(yaw_deg)
        # Small-angle-friendly construction: tilt the nominal +Z axis.
        d = np.array([math.sin(p), math.sin(y),
                      math.cos(p) * math.cos(y)], dtype=float)
        return self.set(model, data, magnitude, d)

    def off(self, data: mujoco.MjData) -> None:
        """Shut down. Leaves the direction where it was."""
        data.ctrl[self.actuator_id] = 0.0

    def reset_direction(self, model: mujoco.MjModel) -> None:
        """Restore the direction declared in the XML."""
        model.actuator_gear[self.actuator_id][:3] = self._nominal_dir

    # ---- reading back ------------------------------------------------
    def magnitude(self, data: mujoco.MjData) -> float:
        """The commanded magnitude, in Newtons."""
        return float(data.ctrl[self.actuator_id])

    def direction_body(self, model: mujoco.MjModel) -> np.ndarray:
        """Current thrust direction as a unit vector in the body frame."""
        d = np.array(model.actuator_gear[self.actuator_id][:3], dtype=float)
        n = np.linalg.norm(d)
        return d / n if n > _EPS else d

    def force_world(self, model: mujoco.MjModel,
                    data: mujoco.MjData) -> np.ndarray:
        """The force this thruster is applying, as a world-frame vector.

        The direction is stored in the body frame, so it must be rotated by
        the body's current orientation. This is what makes body-frame
        thrust work: the craft can tumble and the engine still pushes
        "along its own axis" with no bookkeeping from you.
        """
        f_body = self.direction_body(model) * self.magnitude(data)
        f_world = np.zeros(3)
        mujoco.mju_rotVecQuat(f_world, f_body,
                              np.array(data.xquat[self.body_id], dtype=float))
        return f_world

    def site_world(self, data: mujoco.MjData) -> np.ndarray:
        """Where the nozzle is right now, in world coordinates."""
        return np.array(data.site_xpos[self.site_id], dtype=float)

    def __repr__(self) -> str:
        return (f"Thruster({self.name!r}, max={self.max_thrust_n:g}N, "
                f"actuator={self.actuator_id})")


# =========================================================================
# A SET OF THRUSTERS
# =========================================================================
class ThrusterSet:
    """Every site actuator in a model, addressable by name.

    Discovered from the model itself - add a thruster to the XML and it
    shows up here, with no index table to maintain.

        thr = ThrusterSet(model)
        thr.names                 -> ['main', 'rcs_px', ...]
        thr["main"].set(model, data, 20.0)
        thr.apply(data, {"main": 20.0, "rcs_px": (2.0, [0, 0, 1])})
    """

    def __init__(self, model: mujoco.MjModel, prefix: str | None = None):
        self._model = model
        self._thrusters: dict[str, Thruster] = {}
        want = THRUSTER_PREFIX if prefix is None else prefix

        site_trn = int(mujoco.mjtTrn.mjTRN_SITE)
        for i in range(model.nu):
            if int(model.actuator_trntype[i]) != site_trn:
                continue  # not a site actuator, so not a thruster
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) or ""
            if want and not name.startswith(want):
                continue
            t = Thruster(model, i)
            self._thrusters[t.name] = t

    # ---- dict-ish ----------------------------------------------------
    def __getitem__(self, name: str) -> Thruster:
        try:
            return self._thrusters[name]
        except KeyError:
            raise KeyError(
                f"no thruster {name!r}; this model has: "
                f"{', '.join(self.names) or '(none)'}") from None

    def __iter__(self):
        return iter(self._thrusters.values())

    def __len__(self) -> int:
        return len(self._thrusters)

    def __contains__(self, name: object) -> bool:
        return name in self._thrusters

    @property
    def names(self) -> list[str]:
        return list(self._thrusters)

    # ---- commanding --------------------------------------------------
    def apply(self, data: mujoco.MjData, commands) -> dict[str, float]:
        """Apply a whole command dict at once. Returns what was applied.

        Each value is either a bare magnitude, or a (magnitude, direction)
        pair:

            {"main": 19.44}                      magnitude only
            {"main": (25.0, [0.1, 0.0, 1.0])}    magnitude + direction

        Thrusters you do not mention are left alone, so a control law can
        touch only what it cares about.
        """
        applied: dict[str, float] = {}
        for name, cmd in commands.items():
            t = self[name]
            if isinstance(cmd, (tuple, list)) and len(cmd) == 2 \
                    and np.ndim(cmd[1]) == 1:
                mag, direction = cmd
            else:
                mag, direction = cmd, None
            applied[name] = t.set(self._model, data, float(mag), direction)
        return applied

    def all_off(self, data: mujoco.MjData) -> None:
        for t in self._thrusters.values():
            t.off(data)

    def reset_directions(self) -> None:
        for t in self._thrusters.values():
            t.reset_direction(self._model)

    # ---- reading back ------------------------------------------------
    def magnitudes(self, data: mujoco.MjData) -> dict[str, float]:
        return {n: t.magnitude(data) for n, t in self._thrusters.items()}

    def total_force_world(self, data: mujoco.MjData) -> np.ndarray:
        """Vector sum of every thruster's force, in world coordinates."""
        total = np.zeros(3)
        for t in self._thrusters.values():
            total += t.force_world(self._model, data)
        return total

    def total_mass(self, model: mujoco.MjModel | None = None) -> float:
        """Mass of the body the thrusters are mounted on."""
        m = model if model is not None else self._model
        if not self._thrusters:
            return 0.0
        bid = next(iter(self._thrusters.values())).body_id
        return float(m.body_mass[bid])

    def body_id(self) -> int:
        if not self._thrusters:
            raise ValueError("this ThrusterSet is empty")
        return next(iter(self._thrusters.values())).body_id

    def describe(self, data: mujoco.MjData | None = None) -> str:
        lines = [f"{len(self)} thruster(s):"]
        for t in self._thrusters.values():
            d = t.direction_body(self._model)
            cur = f"  cmd={t.magnitude(data):6.2f}N" if data is not None else ""
            lines.append(f"  {t.name:<10} max={t.max_thrust_n:5.1f}N  "
                         f"dir=[{d[0]:+.3f} {d[1]:+.3f} {d[2]:+.3f}]{cur}")
        return "\n".join(lines)


# =========================================================================
# STATE
# =========================================================================
@dataclass
class CraftState:
    """A snapshot of the vehicle, handed to your control law each tick.

    Everything is world-frame SI except euler_deg (degrees, for reading)
    and angvel (body frame, which is what a rate gyro measures).

        t          seconds of sim time
        pos        [x, y, z]            world position of the body origin
        vel        [vx, vy, vz]         world linear velocity
        quat       [w, x, y, z]         orientation
        angvel     [wx, wy, wz]         angular velocity, BODY frame
        euler_deg  [roll, pitch, yaw]   degrees, for human reading
        up         body +Z expressed in world - the craft's thrust axis
        mass       kg
    """

    t: float
    pos: np.ndarray
    vel: np.ndarray
    quat: np.ndarray
    angvel: np.ndarray
    euler_deg: np.ndarray
    up: np.ndarray
    mass: float
    gravity: np.ndarray = field(default_factory=lambda: np.zeros(3))

    @classmethod
    def from_data(cls, model: mujoco.MjModel, data: mujoco.MjData,
                  body_id: int, mass: float | None = None) -> "CraftState":
        """Read the current state straight off mjData.

        These fields are all computed by mj_step/mj_forward, so the state
        is current as of the last call. (data.qvel is used for linear
        velocity because a freejoint's first three qvel slots ARE the
        world-frame linear velocity of the body.)
        """
        quat = np.array(data.xquat[body_id], dtype=float)

        # Body +Z in world coordinates: the direction the craft's nose
        # points, i.e. where its main engine pushes it.
        up = np.zeros(3)
        mujoco.mju_rotVecQuat(up, np.array([0.0, 0.0, 1.0]), quat)

        # Angular velocity in the BODY frame, which is what a rate gyro
        # reads. mj_objectVelocity with flg_local=1 does the frame
        # conversion correctly for any body.
        #
        # Frames here are genuinely confusing and easy to get wrong:
        #   data.qvel[3:6]      freejoint angular velocity, BODY frame
        #   data.cvel[bid][:3]  angular velocity, WORLD frame
        # so rotating cvel by the body quaternion rotates it a second
        # time and gives a wrong answer whenever the craft is tilted.
        # Verified: tilt 90 deg about X with qvel[3:6]=[0,0,1]; the body
        # rate is [0,0,1] and the world rate is [0,-1,0].
        _vel6 = np.zeros(6)
        mujoco.mj_objectVelocity(model, data, int(mujoco.mjtObj.mjOBJ_BODY),
                                 body_id, _vel6, 1)
        angvel_body = _vel6[:3].copy()

        # World-frame linear velocity of the body's centre. flg_local=0.
        # (For a single free body this equals data.qvel[:3], but going
        # through the API keeps it right for a body inside a chain.)
        _lin6 = np.zeros(6)
        mujoco.mj_objectVelocity(model, data, int(mujoco.mjtObj.mjOBJ_BODY),
                                 body_id, _lin6, 0)

        return cls(
            t=float(data.time),
            pos=np.array(data.xpos[body_id], dtype=float),
            vel=_lin6[3:].copy(),
            quat=quat,
            angvel=angvel_body,
            euler_deg=quat_to_euler_deg(quat),
            up=up,
            mass=float(model.body_mass[body_id]) if mass is None else float(mass),
            gravity=np.array(model.opt.gravity, dtype=float),
        )

    def weight_n(self) -> float:
        """Magnitude of m*g - the thrust needed to hover."""
        return float(self.mass * np.linalg.norm(self.gravity))

    def tilt_deg(self) -> float:
        """Angle between the craft's thrust axis and world +Z."""
        c = float(np.clip(self.up[2], -1.0, 1.0))
        return math.degrees(math.acos(c))

    def __str__(self) -> str:
        return (f"t={self.t:6.2f}s  pos=[{self.pos[0]:+.3f} {self.pos[1]:+.3f} "
                f"{self.pos[2]:+.3f}]  vel=[{self.vel[0]:+.3f} {self.vel[1]:+.3f} "
                f"{self.vel[2]:+.3f}]  tilt={self.tilt_deg():5.1f}deg")


def quat_to_euler_deg(quat) -> np.ndarray:
    """[w,x,y,z] -> roll/pitch/yaw in degrees (ZYX / aerospace order)."""
    w, x, y, z = (float(v) for v in quat)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = (math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0
             else math.asin(sinp))
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return np.degrees([roll, pitch, yaw])


# =========================================================================
# LOGGING
# =========================================================================
class ThrustLogger:
    """Collect per-tick state and thrust into a CSV.

    The schema mirrors the course's load-cell firmware
    (millis,raw,counts,force_N): time first, then one column per
    thruster in Newtons, then state. That way a simulated thrust curve
    can be diffed against a measured one from the bench.
    """

    def __init__(self, thruster_names):
        self.names = list(thruster_names)
        self.rows: list[list[float]] = []

    @property
    def header(self) -> list[str]:
        return (["time_s"]
                + [f"{n}_N" for n in self.names]
                + ["pos_x", "pos_y", "pos_z",
                   "vel_x", "vel_y", "vel_z",
                   "roll_deg", "pitch_deg", "yaw_deg", "tilt_deg"])

    def record(self, state: CraftState, magnitudes: dict[str, float]) -> None:
        self.rows.append(
            [state.t]
            + [float(magnitudes.get(n, 0.0)) for n in self.names]
            + [*state.pos, *state.vel, *state.euler_deg, state.tilt_deg()])

    def save(self, path) -> pathlib.Path:
        p = pathlib.Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        with open(p, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(self.header)
            for r in self.rows:
                w.writerow([f"{v:.6g}" for v in r])
        return p

    def __len__(self) -> int:
        return len(self.rows)
