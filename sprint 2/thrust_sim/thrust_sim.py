#!/usr/bin/env python3
"""
Cold gas thruster simulation with ball-valve throttling and tank blowdown.

Three equal-length phases (each PHASE_DURATION_S):
    phase 1 : ball valve CLOSED (0 deg)                  -> no thrust
    phase 2 : ball valve held at VALVE_THROTTLE_ANGLE_DEG -> throttled thrust
    phase 3 : ball valve FULL OPEN (90 deg)               -> full thrust

Model (lumped parameter):
    tank (blowing down) --[regulator]--> feed --[ball valve]--> plenum --[nozzle]--> ambient

  * Tank: finite volume, isothermal blowdown as mass is drawn out
  * Regulator: ideal, holds its setpoint while the tank can support it, then
    "drops out" and the feed pressure tracks the falling tank pressure
  * Ball valve: effective flow area is the lens-shaped overlap between the
    fixed seat bore and the ball's through-bore, as a function of rotation
    angle (0 deg closed, 90 deg full open) -- this is what makes a ball valve's
    throttling curve strongly nonlinear (e.g. ~20% area at 45 deg, not 50%)
  * Valve angle follows the commanded angle with a first-order lag
  * Chamber (plenum) pressure is integrated from mass balance (ideal gas, isothermal)
  * Nozzle: choked throat, ideal isentropic expansion to a fixed area ratio
  * Thrust = mdot * v_exit + (P_exit - P_ambient) * A_exit

Requires: numpy, matplotlib
"""

from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

SCRIPT_DIR = Path(__file__).resolve().parent

# =============================================================================
# USER PARAMETERS -- edit these to explore the design space. See README.md in
# this folder for a full explanation of every variable below.
# =============================================================================

# --- Supply: tank + regulator ------------------------------------------------
TANK_INITIAL_PRESSURE_PSIG = 1000.0    # bottle pressure at t = 0 (gauge)
TANK_VOLUME_M3 = 2.0e-3                # tank internal volume, m^3 (2 L)
REGULATOR_SETPOINT_PSIG = 200.0        # regulated feed pressure (gauge)
AMBIENT_PRESSURE_PA = 101325.0         # sea level ambient pressure

# --- Gas (default: nitrogen) -------------------------------------------------
GAMMA = 1.4                     # ratio of specific heats
MOLAR_MASS = 0.028013           # kg/mol
GAS_TEMP_K = 293.0              # K (assumed constant -- isothermal)

# --- Ball valve ---------------------------------------------------------------
BALL_BORE_DIAMETER_M = 4.020e-3        # valve through-bore diameter (matches nozzle inlet)
BALL_DIAMETER_M = 10.0e-3              # ball (sphere) outer diameter
VALVE_CD = 0.70                        # valve discharge coefficient
VALVE_TAU_S = 1.0e-3                   # actuator first-order lag, commanded -> actual angle (s)
VALVE_THROTTLE_ANGLE_DEG = 45.0        # phase-2 hold angle (0 = closed, 90 = full open)

# --- Nozzle geometry (structures subteam, "6.06N Nozzle Calculations") -------
THROAT_DIAMETER_M = 2.010e-3    # nozzle throat diameter
EXIT_DIAMETER_M = 3.115e-3      # nozzle exit diameter
INLET_DIAMETER_M = 4.020e-3     # nozzle inlet diameter (reference only, not used below)
NOZZLE_CD = 0.95                # nozzle discharge coefficient
CHAMBER_VOLUME_M3 = 1.0e-6      # plenum volume between valve and nozzle (1 cm^3)

# --- Run / numerics ------------------------------------------------------------
PHASE_DURATION_S = 5.0          # length of each of the three phases (s)
DT = 50e-6                      # integration step (s)
SHOW_VALVE_CHARACTERISTIC = True  # also plot steady thrust/area vs. valve angle

# =============================================================================
# DERIVED CONSTANTS
# =============================================================================
TANK_INITIAL_PRESSURE_PA = AMBIENT_PRESSURE_PA + TANK_INITIAL_PRESSURE_PSIG * 6894.76
REGULATOR_SETPOINT_PA = AMBIENT_PRESSURE_PA + REGULATOR_SETPOINT_PSIG * 6894.76

R_UNIV = 8.314462618
R_GAS = R_UNIV / MOLAR_MASS                     # J/(kg K)

BALL_BORE_RADIUS_M = BALL_BORE_DIAMETER_M / 2
BALL_RADIUS_M = BALL_DIAMETER_M / 2
A_BORE_FULL = np.pi * BALL_BORE_RADIUS_M ** 2   # fully-open valve flow area

EXPANSION_RATIO = (EXIT_DIAMETER_M / THROAT_DIAMETER_M) ** 2   # Ae / At
A_THROAT = np.pi * (THROAT_DIAMETER_M / 2) ** 2
A_EXIT = A_THROAT * EXPANSION_RATIO

CRIT_RATIO = (2 / (GAMMA + 1)) ** (GAMMA / (GAMMA - 1))   # critical P2/P1
CHOKED_COEF = np.sqrt(GAMMA / (R_GAS * GAS_TEMP_K)) * \
    (2 / (GAMMA + 1)) ** ((GAMMA + 1) / (2 * (GAMMA - 1)))


def exit_pressure_ratio(eps, g=GAMMA):
    """Solve the area-Mach relation for the supersonic exit Mach number and
    return the exit/chamber pressure ratio Pe/Pc."""
    def area_ratio(M):
        return (1 / M) * ((2 / (g + 1)) * (1 + (g - 1) / 2 * M**2)) ** ((g + 1) / (2 * (g - 1)))
    lo, hi = 1.0, 20.0
    for _ in range(100):                       # bisection
        mid = 0.5 * (lo + hi)
        if area_ratio(mid) < eps:
            lo = mid
        else:
            hi = mid
    M = 0.5 * (lo + hi)
    return (1 + (g - 1) / 2 * M**2) ** (-g / (g - 1))


PE_OVER_PC = exit_pressure_ratio(EXPANSION_RATIO)


def orifice_mdot(p_up, p_down, area, cd):
    """Compressible mass flow through an orifice (choked or subsonic)."""
    if p_up <= p_down or area <= 0:
        return 0.0
    r = p_down / p_up
    if r <= CRIT_RATIO:
        return cd * area * p_up * CHOKED_COEF
    term = (r ** (2 / GAMMA)) - (r ** ((GAMMA + 1) / GAMMA))
    return cd * area * p_up * np.sqrt(2 * GAMMA / ((GAMMA - 1) * R_GAS * GAS_TEMP_K) * term)


def valve_area(theta_deg):
    """Effective ball-valve flow area at rotation angle theta_deg (0 = closed,
    90 = full open), modeled as the lens-shaped overlap between the fixed
    seat bore and the ball's through-bore as the ball rotates.

    The ball's bore mouth is a circle of the same radius as the seat bore,
    whose center is offset from the seat bore's center by
    e(theta) = BALL_RADIUS_M * cos(theta). At theta = 0 deg the offset is at
    its maximum (BALL_RADIUS_M) and the overlap vanishes (closed), as long as
    BALL_DIAMETER_M >= 2 * BALL_BORE_DIAMETER_M; at theta = 90 deg the offset
    is zero, the two bores are concentric, and the overlap is the full bore
    area (open).
    """
    theta = np.clip(theta_deg, 0.0, 90.0) * np.pi / 180.0
    a = BALL_BORE_RADIUS_M
    e = BALL_RADIUS_M * np.cos(theta)
    if e >= 2 * a:
        return 0.0
    if e <= 0:
        return A_BORE_FULL
    # Area of intersection of two equal circles of radius a, centers 2*e... -
    # using the standard lens/overlap formula for two circles of radius a
    # whose centers are separated by distance e:
    half_e = e / 2
    term = np.clip(half_e / a, -1.0, 1.0)
    area = 2 * a ** 2 * np.arccos(term) - half_e * np.sqrt(np.clip(4 * a ** 2 - e ** 2, 0.0, None))
    return float(np.clip(area, 0.0, A_BORE_FULL))


def valve_command(t):
    """Commanded ball-valve angle (deg) at time t."""
    if t < PHASE_DURATION_S:                        # phase 1: closed
        return 0.0
    if t < 2 * PHASE_DURATION_S:                     # phase 2: throttled
        return VALVE_THROTTLE_ANGLE_DEG
    return 90.0                                      # phase 3: full open


def simulate():
    n = int(round(3 * PHASE_DURATION_S / DT))
    t = np.arange(n) * DT
    p_tank = np.zeros(n)
    p_feed = np.zeros(n)
    pc = np.zeros(n)
    thrust = np.zeros(n)
    mdot_in_arr = np.zeros(n)
    mdot_out_arr = np.zeros(n)
    angle_cmd = np.zeros(n)
    angle = np.zeros(n)

    p_t = TANK_INITIAL_PRESSURE_PA
    p_c = AMBIENT_PRESSURE_PA      # chamber starts at ambient (empty)
    theta = 0.0                    # actual valve angle (deg)
    alpha = DT / VALVE_TAU_S

    for i in range(n):
        c = valve_command(t[i])
        theta += (c - theta) * min(alpha, 1.0)

        p_f = min(REGULATOR_SETPOINT_PA, p_t)   # ideal regulator with dropout

        mdot_in = orifice_mdot(p_f, p_c, valve_area(theta), VALVE_CD)

        # Nozzle (choked whenever the chamber is sufficiently above ambient)
        if p_c > 0 and p_c * CRIT_RATIO >= AMBIENT_PRESSURE_PA:
            mdot_out = NOZZLE_CD * A_THROAT * p_c * CHOKED_COEF
            p_e = p_c * PE_OVER_PC
            v_e = np.sqrt(2 * GAMMA / (GAMMA - 1) * R_GAS * GAS_TEMP_K *
                          (1 - PE_OVER_PC ** ((GAMMA - 1) / GAMMA)))
            f = mdot_out * v_e + (p_e - AMBIENT_PRESSURE_PA) * A_EXIT
        else:
            mdot_out = orifice_mdot(p_c, AMBIENT_PRESSURE_PA, A_THROAT, NOZZLE_CD)
            f = 0.0   # unchoked flow: negligible thrust in this simple model
        f = max(f, 0.0)

        # Tank mass balance (isothermal blowdown): dP/dt = -(R T / V) * mdot_in
        p_t -= DT * R_GAS * GAS_TEMP_K / TANK_VOLUME_M3 * mdot_in
        p_t = max(p_t, AMBIENT_PRESSURE_PA)

        # Chamber mass balance: dP/dt = (R T / V)(mdot_in - mdot_out)
        p_c += DT * R_GAS * GAS_TEMP_K / CHAMBER_VOLUME_M3 * (mdot_in - mdot_out)
        p_c = min(max(p_c, AMBIENT_PRESSURE_PA), p_f)   # chamber can't exceed its feed

        p_tank[i], p_feed[i], pc[i] = p_t, p_f, p_c
        thrust[i], mdot_in_arr[i], mdot_out_arr[i] = f, mdot_in, mdot_out
        angle_cmd[i], angle[i] = c, theta

    return t, p_tank, p_feed, pc, thrust, mdot_in_arr, mdot_out_arr, angle_cmd, angle


def steady_thrust_vs_angle(n_points=91):
    """Steady-state thrust and valve area fraction vs. commanded angle, holding
    the feed at the regulator setpoint (i.e. before any tank dropout)."""
    angles = np.linspace(0, 90, n_points)
    thrusts = np.zeros(n_points)
    area_frac = np.zeros(n_points)
    p_f = REGULATOR_SETPOINT_PA

    for k, ang in enumerate(angles):
        area = valve_area(ang)
        area_frac[k] = area / A_BORE_FULL if A_BORE_FULL > 0 else 0.0
        if area <= 0:
            continue
        # Bisection on chamber pressure p_c in [ambient, p_f] for mdot_in == mdot_out
        lo, hi = AMBIENT_PRESSURE_PA, p_f
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            mdot_in = orifice_mdot(p_f, mid, area, VALVE_CD)
            if mid * CRIT_RATIO >= AMBIENT_PRESSURE_PA:
                mdot_out = NOZZLE_CD * A_THROAT * mid * CHOKED_COEF
            else:
                mdot_out = orifice_mdot(mid, AMBIENT_PRESSURE_PA, A_THROAT, NOZZLE_CD)
            if mdot_in > mdot_out:
                lo = mid
            else:
                hi = mid
        p_c = 0.5 * (lo + hi)
        if p_c * CRIT_RATIO >= AMBIENT_PRESSURE_PA:
            mdot_out = NOZZLE_CD * A_THROAT * p_c * CHOKED_COEF
            p_e = p_c * PE_OVER_PC
            v_e = np.sqrt(2 * GAMMA / (GAMMA - 1) * R_GAS * GAS_TEMP_K *
                          (1 - PE_OVER_PC ** ((GAMMA - 1) / GAMMA)))
            thrusts[k] = max(mdot_out * v_e + (p_e - AMBIENT_PRESSURE_PA) * A_EXIT, 0.0)

    return angles, thrusts, area_frac


def main():
    print(f"Tank: {TANK_INITIAL_PRESSURE_PSIG:.0f} psig in {TANK_VOLUME_M3*1e3:.2f} L | "
          f"Regulator setpoint: {REGULATOR_SETPOINT_PSIG:.0f} psig | "
          f"Throttle angle: {VALVE_THROTTLE_ANGLE_DEG:.0f} deg")
    t, p_tank, p_feed, pc, thrust, mdot_in, mdot_out, angle_cmd, angle = simulate()

    # Per-phase stats (skip first 20% of each phase to let transients settle)
    for name, k in (("No thrust", 0), ("Throttled", 1), ("Full thrust", 2)):
        m = (t >= (k + 0.2) * PHASE_DURATION_S) & (t < (k + 1) * PHASE_DURATION_S)
        print(f"{name:12s}: mean thrust = {thrust[m].mean():7.4f} N | "
              f"peak = {thrust[m].max():7.4f} N | mean mdot = {mdot_out[m].mean()*1e3:7.4f} g/s")

    dropout_mask = p_tank <= REGULATOR_SETPOINT_PA
    if dropout_mask.any():
        t_dropout = t[dropout_mask][0]
        print(f"Regulator dropout at t = {t_dropout:.2f} s "
              f"(tank pressure fell to the {REGULATOR_SETPOINT_PSIG:.0f} psig setpoint)")
    else:
        print("Regulator never dropped out during this run.")
    print(f"Tank pressure remaining at end of run: "
          f"{(p_tank[-1] - AMBIENT_PRESSURE_PA) / 6894.76:.1f} psig")

    # ------------------------------------------------------------------ Figure 1
    fig = plt.figure(figsize=(12, 9))
    gs = fig.add_gridspec(3, 2, height_ratios=[1.3, 1, 1])

    ax1 = fig.add_subplot(gs[0, :])
    ax1.plot(t, thrust, lw=0.6, color="tab:blue", label="Instantaneous thrust")
    w = int(0.05 / DT)      # 50 ms moving average for visual clarity
    avg = np.convolve(thrust, np.ones(w) / w, mode="same")
    ax1.plot(t, avg, lw=1.5, color="tab:red", label="50 ms moving average")
    for k, lbl in enumerate(["NO THRUST", "THROTTLED", "FULL THRUST"]):
        ax1.axvspan(k * PHASE_DURATION_S, (k + 1) * PHASE_DURATION_S,
                    alpha=0.07, color=["gray", "orange", "green"][k])
        ax1.text((k + 0.5) * PHASE_DURATION_S, thrust.max() * 1.05, lbl,
                 ha="center", va="bottom", fontsize=10, weight="bold")
    ax1.set_ylabel("Thrust (N)")
    ax1.set_xlim(0, 3 * PHASE_DURATION_S)
    ax1.set_ylim(0, thrust.max() * 1.2 if thrust.max() > 0 else 1.0)
    ax1.set_title(f"Cold gas thruster thrust profile  (tank = {TANK_INITIAL_PRESSURE_PSIG:.0f} psig, "
                  f"regulator = {REGULATOR_SETPOINT_PSIG:.0f} psig, "
                  f"throttle angle = {VALVE_THROTTLE_ANGLE_DEG:.0f} deg)")
    ax1.legend(loc="center left")
    ax1.grid(alpha=0.3)

    ax2 = fig.add_subplot(gs[1, :])
    ax2.plot(t, (p_tank - AMBIENT_PRESSURE_PA) / 6894.76, color="tab:purple", label="Tank (gauge)")
    ax2.plot(t, (p_feed - AMBIENT_PRESSURE_PA) / 6894.76, color="tab:orange", label="Regulator outlet (gauge)")
    ax2.plot(t, (pc - AMBIENT_PRESSURE_PA) / 6894.76, color="tab:blue", label="Chamber (gauge)")
    ax2.set_ylabel("Pressure (psig)")
    ax2.set_xlabel("Time (s)")
    ax2.set_title("Tank blowdown, regulator outlet, and chamber pressure")
    ax2.legend(loc="center left")
    ax2.grid(alpha=0.3)
    ax2.set_xlim(0, 3 * PHASE_DURATION_S)

    ax3 = fig.add_subplot(gs[2, 0])
    ax3.plot(t, angle_cmd, "k--", lw=1, label="Commanded angle")
    ax3.plot(t, angle, color="tab:green", label="Actual angle")
    ax3.set_ylabel("Valve angle (deg)")
    ax3.set_xlabel("Time (s)")
    ax3.set_title("Ball valve angle: commanded vs. actual")
    ax3.legend()
    ax3.grid(alpha=0.3)
    ax3.set_xlim(0, 3 * PHASE_DURATION_S)
    ax3.set_ylim(-5, 95)

    ax4 = fig.add_subplot(gs[2, 1])
    ax4.plot(t, mdot_in * 1e3, color="tab:blue", label="Valve mdot (in)")
    ax4.plot(t, mdot_out * 1e3, color="tab:red", label="Nozzle mdot (out)")
    ax4.set_ylabel("Mass flow (g/s)")
    ax4.set_xlabel("Time (s)")
    ax4.set_title("Valve and nozzle mass flow")
    ax4.legend()
    ax4.grid(alpha=0.3)
    ax4.set_xlim(0, 3 * PHASE_DURATION_S)

    fig.tight_layout()
    fig.savefig(SCRIPT_DIR / "thruster_profile.png", dpi=150)
    print(f"Saved plot to {SCRIPT_DIR / 'thruster_profile.png'}")

    # ------------------------------------------------------------------ Figure 2
    if SHOW_VALVE_CHARACTERISTIC:
        angles, steady_thrust, area_frac = steady_thrust_vs_angle()

        fig2, (axc1, axc2) = plt.subplots(1, 2, figsize=(11, 4.5))
        axc1.plot(angles, area_frac, color="tab:blue")
        axc1.set_xlabel("Valve angle (deg)")
        axc1.set_ylabel("Effective area / full-open area")
        axc1.set_title("Ball valve flow-area characteristic")
        axc1.grid(alpha=0.3)
        axc1.set_xlim(0, 90)
        axc1.set_ylim(0, 1.05)

        axc2.plot(angles, steady_thrust, color="tab:green")
        axc2.set_xlabel("Valve angle (deg)")
        axc2.set_ylabel("Steady-state thrust (N)")
        axc2.set_title(f"Steady thrust vs. angle (feed = {REGULATOR_SETPOINT_PSIG:.0f} psig)")
        axc2.grid(alpha=0.3)
        axc2.set_xlim(0, 90)

        fig2.tight_layout()
        fig2.savefig(SCRIPT_DIR / "valve_characteristic.png", dpi=150)
        print(f"Saved plot to {SCRIPT_DIR / 'valve_characteristic.png'}")

    plt.show()


if __name__ == "__main__":
    main()
