#!/usr/bin/env python3
"""Variable vehicle mass: propellant leaves the craft as thrust is produced.

Mass is not a constant on a real vehicle. Every Newton-second of impulse is
mass thrown overboard, so the craft gets lighter as it flies and the m*g
feedforward every control law relies on slowly stops being true.

    from propellant import PropellantTank
    tank = PropellantTank(model, body_id)
    ...
    tank.enforce_cutoff(data)   # a dry tank produces nothing
    tank.update(data, dt)       # integrate mdot, write mass + inertia
    mujoco.mj_step(model, data)

TO CHANGE HOW MASS VARIES, edit mass_flow() below. It is the only place the
physics of depletion lives; everything else here is bookkeeping.

*** THE TRAP ***  Writing model.body_mass is NOT enough. It updates the
gravity bias but leaves the MASS MATRIX stale, so you get a craft that
weighs less while still resisting acceleration like the heavy one - wrong,
and silently so. Measured on 07_thrust_1D at ctrl=30 N:

    baseline, mass 12            qacc = 0.88
    write body_mass = 6 only     qacc = 1.69   <- not 2x, and M[0,0] still 12
    then mj_setConst             qacc = 3.38   <- correct

In zero-g the proof is starker: writing body_mass alone leaves qacc EXACTLY
unchanged. mujoco.mj_setConst(model, data) is mandatory after every write,
and update() calls it for you. With it, F=ma holds to 4e-16 across an 8x
mass range.

mj_setConst does NOT rescale body_inertia - verified unchanged before and
after. So update() scales it explicitly. That matters on a 6-DOF scene:
firing one RCS jet at half mass gives angular qacc 14.706 (frozen at the
wet-mass value) without scaling and 29.412 with it. On the 1-DOF rig it
makes no difference at all, since there is no rotational freedom to get
wrong.

ASSUMPTION: the propellant is distributed like the rest of the vehicle, so
mass and inertia scale together and the centre of mass does not move
(body_ipos is left alone). Right for a centred tank; wrong for one mounted
off-axis, which would need body_ipos shifted as it drains.
"""
import mujoco
import numpy as np

# --- YOUR PARAMETERS ------------------------------------------------------
# Set these for the vehicle you are modelling. One labeled block, unit
# suffix in every name, provenance in the comment - the convention from
# assignment1/lunar-hopper/02-cad/base_frame.py.

DRY_MASS_KG   = 8.0       # kg, vehicle with empty tanks
PROPELLANT_KG = 4.0       # kg, usable propellant at t = 0
ISP_S         = 57.80     # s, specific impulse; sets mdot = F / (ISP_S * g0)

G0_MS2 = 9.80665          # m/s^2, standard gravity - the g0 in the Isp
                          #   definition. This is a property of the Isp
                          #   convention, NOT the gravity the craft flies in,
                          #   so it stays 9.80665 on the Moon.

# REAL HARDWARE, for reference. Sources: the structures subteam's "6.06N
# Nozzle Calculations" (sprint 1/Claude generated thrust model/) and the
# blowdown model at sprint 2/thrust_sim/thrust_sim.py:41-43.
#
#   Isp 57.80 s, mdot 10.688 g/s, v_e 570.8 m/s, F 6.058 N at 200 psig
#   tank 2 L at 1000 psig, regulator 200 psig -> 127 g usable, 12.2 s burn
#   (12.2 s lands inside the mission brief's stated 10-20 s burn window)
#
# The Isp above IS the real one. The masses are not, deliberately. At the
# real propellant load, 127 g against a 12 kg craft is a 1.06% mass change
# and 3.7 s of lunar hover - measured, the tank empties during the initial
# settle and the craft never leaves the ground. Physically honest, but
# nothing you can see. The defaults keep the real Isp and give a 33%
# propellant fraction so the effect is actually observable.
#
# For the real article:  DRY_MASS_KG = 11.873, PROPELLANT_KG = 0.127


class PropellantTank:
    """Tracks propellant and keeps the model's mass and inertia in step.

    One tank per run. It owns the vehicle's mass: the XML's mass= attribute
    is overwritten on construction, so DRY_MASS_KG + PROPELLANT_KG is the
    wet mass that actually flies, not whatever the scene declared.
    """

    def __init__(self, model, body_id, dry_mass_kg=DRY_MASS_KG,
                 propellant_kg=PROPELLANT_KG, isp_s=ISP_S):
        if dry_mass_kg <= 0.0:
            raise ValueError(f"dry mass must be positive, got {dry_mass_kg}")
        if propellant_kg < 0.0:
            raise ValueError(f"propellant cannot be negative, got {propellant_kg}")
        if isp_s <= 0.0:
            raise ValueError(f"Isp must be positive, got {isp_s}")

        self._model = model
        self.body_id = int(body_id)
        self.dry_mass_kg = float(dry_mass_kg)
        self.propellant_initial_kg = float(propellant_kg)
        self.propellant_kg = float(propellant_kg)
        self.isp_s = float(isp_s)
        self.used_kg = 0.0
        self.emptied_at_s = None      # sim time the tank ran dry, or None

        # The inertia the XML compiled, treated as the WET reference. Every
        # later scaling is relative to this, never to the current value -
        # scaling an already-scaled number compounds the error each tick.
        self._inertia_wet = np.array(model.body_inertia[self.body_id],
                                     dtype=float).copy()
        self._mass_wet = self.dry_mass_kg + self.propellant_initial_kg

        # Take ownership of the mass immediately, so t=0 is the wet vehicle
        # rather than whatever placeholder the scene declared. No
        # mj_setConst here: the caller runs mj_forward before the first
        # tick anyway, and there is no MjData to pass yet.
        self._write(model)

    # ---- the physics ----------------------------------------------------
    def mass_flow(self, thrust_n):
        """Propellant consumed per second, kg/s, for a given total thrust.

        *** THIS IS THE FUNCTION TO REPLACE. ***

        Constant specific impulse, the textbook relation:

            mdot = F / (Isp * g0)                 [kg/s]

        One constant sets everything, and flow is proportional to thrust, so
        a hovering craft sips and a hard burn gulps.

        Other laws drop straight in here:

          fixed mass flow     return MDOT_KGS if thrust_n > 0 else 0.0
                              (a valve that is either open or shut)

          falling Isp         Isp drops as the tank depletes, which is what
                              a real blowdown does:
                                  isp = self.isp_s * self.fraction_remaining
                                  return thrust_n / (isp * G0_MS2)

          full blowdown       sprint 2/thrust_sim/thrust_sim.py already
                              models tank + regulator + valve + choked
                              nozzle. Step it alongside this sim and return
                              its mdot. The mission brief calls blowdown
                              "the part your model has to get right", so
                              this is the upgrade that matters.

        Whatever you return, update() handles the integration, the floor at
        empty, and the MuJoCo bookkeeping.
        """
        if thrust_n <= 0.0:
            return 0.0
        return thrust_n / (self.isp_s * G0_MS2)

    # ---- bookkeeping ----------------------------------------------------
    @property
    def mass_kg(self):
        return self.dry_mass_kg + self.propellant_kg

    @property
    def empty(self):
        return self.propellant_kg <= 0.0

    @property
    def fraction_remaining(self):
        if self.propellant_initial_kg <= 0.0:
            return 0.0
        return self.propellant_kg / self.propellant_initial_kg

    def _write(self, model):
        """Push the current mass and scaled inertia into the model."""
        mass = self.mass_kg
        model.body_mass[self.body_id] = mass
        # Scale from the WET reference, so repeated calls cannot compound.
        model.body_inertia[self.body_id] = self._inertia_wet * (
            mass / self._mass_wet)

    def enforce_cutoff(self, data):
        """A dry tank produces no thrust, whatever was commanded.

        Called AFTER the control law has written data.ctrl, so the law goes
        on asking for thrust and simply does not get it - which is what a
        real empty tank does to a controller that has not noticed yet.
        """
        if self.empty:
            data.ctrl[:] = 0.0

    def update(self, data, dt):
        """Integrate one tick of depletion. Returns the new total mass.

        Reads the thrust actually being applied from data.ctrl, so it sees
        whatever the control law commanded (post-clamping, post-cutoff).
        """
        if self.empty:
            return self.mass_kg

        thrust_n = float(np.sum(np.abs(data.ctrl)))
        burned = self.mass_flow(thrust_n) * dt
        if burned <= 0.0:
            return self.mass_kg

        burned = min(burned, self.propellant_kg)   # never past empty
        self.propellant_kg -= burned
        self.used_kg += burned
        if self.empty and self.emptied_at_s is None:
            self.emptied_at_s = float(data.time)

        self._write(self._model)

        # MANDATORY. Without this the mass matrix keeps the old value and
        # the dynamics are wrong without any error being raised. See the
        # module docstring for the measured numbers.
        #
        # *** AND IT RESETS qpos TO THE MODEL DEFAULTS. *** Measured: a
        # craft at qpos=1.234 comes back at 0.0, i.e. teleported to its
        # spawn, every single tick. qvel survives, so the craft appears
        # frozen in place while its velocity climbs - which is exactly how
        # this showed up: pos_z pinned at 0.5000 while vel_z reached +1.23.
        # Saving and restoring qpos around the call is the fix. (A zero-g
        # rocket-equation check does not catch this, because it only reads
        # velocity.)
        qpos_hold = np.array(data.qpos, dtype=float).copy()
        mujoco.mj_setConst(self._model, data)
        data.qpos[:] = qpos_hold
        return self.mass_kg

    def describe(self):
        return (f"tank        {self.dry_mass_kg:.3f} kg dry + "
                f"{self.propellant_initial_kg:.3f} kg propellant = "
                f"{self._mass_wet:.3f} kg wet\n"
                f"            Isp {self.isp_s:.2f} s -> "
                f"{1.0 / (self.isp_s * G0_MS2) * 1000:.4f} g/s per Newton")
