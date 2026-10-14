#!/usr/bin/env python3
"""
Cold gas thruster simulation with PWM (pulse-width modulated) throttling.

15 s demo:
    0 -  5 s : valve held CLOSED        -> no thrust
    5 - 10 s : valve PWM'd (duty cycle) -> throttled thrust
   10 - 15 s : valve held OPEN          -> full thrust

Model (lumped parameter):
    tank (constant pressure) --[valve orifice]--> plenum/chamber --[nozzle]--> ambient

  * Valve orifice flow: compressible (choked or subsonic), scaled by valve position
  * Valve position follows the commanded open/close with a first-order lag
  * Chamber pressure is integrated from mass balance (ideal gas, isothermal)
  * Nozzle: choked throat, ideal isentropic expansion to a fixed area ratio
  * Thrust = mdot * v_exit + (P_exit - P_ambient) * A_exit

Requires: numpy, matplotlib
"""

import numpy as np
import matplotlib.pyplot as plt

# =============================================================================
# USER PARAMETERS
# =============================================================================
TANK_PRESSURE_PSI = 1e3        # Units of our tank
TANK_PRESSURE_PA =  TANK_PRESSURE_PSI * 6894.76    # Convert to Pa
AMBIENT_PRESSURE_PA = 101325   # Sea level pressure

# Gas (default: nitrogen)
GAMMA = 1.4                    # ratio of specific heats
MOLAR_MASS = 0.028013          # kg/mol
GAS_TEMP_K = 293.0             # K (assumed constant)

# Hardware
VALVE_DIAMETER_M = 1.0e-3      # valve orifice diameter
VALVE_CD = 0.8                 # valve discharge coefficient
VALVE_TAU_S = 1.0e-3           # valve response time constant (first-order lag)
CHAMBER_VOLUME_M3 = 1.0e-6     # volume between valve and nozzle (1 cm^3)
THROAT_DIAMETER_M = 0.5e-3     # nozzle throat diameter
NOZZLE_CD = 0.95               # nozzle discharge coefficient
EXPANSION_RATIO = 20.0         # exit area / throat area

# PWM throttling
PWM_FREQUENCY_HZ = 20.0        # valve cycles per second
PWM_DUTY_CYCLE = 0.5           # fraction of each cycle the valve is open (0-1)

# =============================================================================
# LOAD CELL DATA ACQUISITION (from loadcells.ino, Arduino + HX711)
# These describe how the *physical* thrust rig logs data, not the simulation's
# own integration step (DT, below). Use them to downsample/align the simulated
# thrust profile against real test-stand data.
# =============================================================================
LOADCELL_SERIAL_BAUD = 115200          # Serial.begin(115200) in loadcells.ino
HX711_GAIN = 128                       # scale.begin(DT_PIN, CLK_PIN, 128) -> Channel A, gain 128
HX711_SAMPLES_PER_READING = 10         # scale.read_average() called with no arg in loop() ->
                                        # HX711 library default of 10 raw ADC reads averaged per logged line
HX711_READ_TIMEOUT_MS = 1500           # TIMEOUT_MS in loadcells.ino: max wait for a ready
                                        # conversion before the Arduino reports a timeout error

# --- HARDWARE-DEPENDENT: ASK STRUCTURES SUBTEAM ------------------------------
# The HX711's own ADC conversion rate (10 SPS or 80 SPS) is set by a physical
# pin strap (RATE pin tied low/high) on the load-cell amplifier board itself,
# not by anything in loadcells.ino. Assumed here as the common default (10 SPS);
# confirm the actual board wiring before trusting this number.
HX711_ADC_RATE_HZ = 10.0               # ASSUMED default - confirm RATE pin wiring with structures
LOADCELL_SAMPLE_PERIOD_S = HX711_SAMPLES_PER_READING / HX711_ADC_RATE_HZ
                                        # time to produce one logged force_N sample (~1 s at defaults)

# --- HARDWARE-DEPENDENT: ASK STRUCTURES SUBTEAM ------------------------------
# The load cell's rated full-scale capacity sets the noise floor and the max
# measurable thrust, and isn't recoverable from the firmware alone.
LOADCELL_RATED_CAPACITY_N = None       # ASK STRUCTURES - load cell's rated full-scale force (N)

# Timeline
PHASE_DURATION_S = 5.0         # each phase lasts 5 s -> 15 s total
DT = 50e-6                     # integration step (s)

# =============================================================================
# ANIMATION DISPLAY (visual only - does not change the simulated data or the
# final saved PNG, which is always drawn from the complete, fully-revealed
# curves regardless of these settings)
# =============================================================================
ANIMATION_FRAMES = 120         # number of redraw steps while sweeping the plot
ANIMATION_PAUSE_S = 0.02       # pause between redraws (s) -> ~2.4 s total sweep

# =============================================================================
# DERIVED CONSTANTS
# =============================================================================
R_UNIV = 8.314462618
R_GAS = R_UNIV / MOLAR_MASS                     # J/(kg K)
A_VALVE = np.pi * (VALVE_DIAMETER_M / 2) ** 2
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


def valve_command(t):
    """Commanded valve state (0 = closed, 1 = open) at time t."""
    if t < PHASE_DURATION_S:                         # phase 1: no thrust
        return 0.0
    if t < 2 * PHASE_DURATION_S:                     # phase 2: PWM
        t_in = t - PHASE_DURATION_S
        phase = (t_in * PWM_FREQUENCY_HZ) % 1.0
        return 1.0 if phase < PWM_DUTY_CYCLE else 0.0
    return 1.0                                       # phase 3: full open


def simulate():
    n = int(round(3 * PHASE_DURATION_S / DT))
    t = np.arange(n) * DT
    pc = np.zeros(n)           # chamber pressure
    thrust = np.zeros(n)
    mdot_out_arr = np.zeros(n)
    cmd = np.zeros(n)
    pos = np.zeros(n)

    p_c = AMBIENT_PRESSURE_PA  # chamber starts at ambient (empty)
    x = 0.0                    # valve position
    alpha = DT / VALVE_TAU_S

    for i in range(n):
        c = valve_command(t[i])
        x += (c - x) * min(alpha, 1.0)

        mdot_in = orifice_mdot(TANK_PRESSURE_PA, p_c, A_VALVE * x, VALVE_CD)

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

        # Chamber mass balance: dP/dt = (R T / V)(mdot_in - mdot_out)
        p_c += DT * R_GAS * GAS_TEMP_K / CHAMBER_VOLUME_M3 * (mdot_in - mdot_out)
        p_c = max(p_c, AMBIENT_PRESSURE_PA)

        pc[i], thrust[i], mdot_out_arr[i], cmd[i], pos[i] = p_c, f, mdot_out, c, x

    return t, pc, thrust, mdot_out_arr, cmd, pos


def main():
    print(f"Tank pressure: {TANK_PRESSURE_PSI:.1f} psi | "
          f"PWM: {PWM_FREQUENCY_HZ:.0f} Hz @ {PWM_DUTY_CYCLE*100:.0f}% duty")
    t, pc, thrust, mdot, cmd, pos = simulate()

    # Per-phase stats (skip first 0.5 s of full-thrust phase? no: use settled last 4 s)
    for name, k in (("No thrust", 0), ("Throttled", 1), ("Full thrust", 2)):
        m = (t >= (k + 0.2) * PHASE_DURATION_S) & (t < (k + 1) * PHASE_DURATION_S)
        print(f"{name:12s}: mean thrust = {thrust[m].mean():7.4f} N | "
              f"peak = {thrust[m].max():7.4f} N | mean mdot = {mdot[m].mean()*1e3:7.4f} g/s")

    # ------------------------------------------------------------------ plots
    fig = plt.figure(figsize=(12, 6))
    gs = fig.add_gridspec(2, 2, height_ratios=[1.3, 1])

    ax1 = fig.add_subplot(gs[0, :])
    line_thrust, = ax1.plot([], [], lw=0.6, color="tab:blue", label="Instantaneous thrust")
    # 50 ms moving average for visual clarity of the throttled phase
    w = int(0.05 / DT)
    avg = np.convolve(thrust, np.ones(w) / w, mode="same")
    line_avg, = ax1.plot([], [], lw=1.5, color="tab:red", label="50 ms moving average")
    for k, lbl in enumerate(["NO THRUST", "THROTTLED (PWM)", "FULL THRUST"]):
        ax1.axvspan(k * PHASE_DURATION_S, (k + 1) * PHASE_DURATION_S,
                    alpha=0.07, color=["gray", "orange", "green"][k])
        ax1.text((k + 0.5) * PHASE_DURATION_S, thrust.max() * 1.05, lbl,
                 ha="center", va="bottom", fontsize=10, weight="bold")
    ax1.set_ylabel("Thrust (N)")
    ax1.set_xlim(0, 3 * PHASE_DURATION_S)
    ax1.set_ylim(0, thrust.max() * 1.2)
    ax1.set_title(f"Cold gas thruster thrust profile  (tank = {TANK_PRESSURE_PSI:.0f} psi, "
                  f"PWM {PWM_FREQUENCY_HZ:.0f} Hz / {PWM_DUTY_CYCLE*100:.0f}% duty)")
    ax1.legend(loc="center left")
    ax1.grid(alpha=0.3)

    # Zoom on a few PWM cycles
    t0 = PHASE_DURATION_S + 1.0
    t1 = t0 + 3.0 / PWM_FREQUENCY_HZ
    z = (t >= t0) & (t < t1)
    t_zoom, thrust_zoom, cmd_zoom, pos_zoom = t[z], thrust[z], cmd[z], pos[z]

    ax2 = fig.add_subplot(gs[1, 0])
    line_zoom_thrust, = ax2.plot([], [], color="tab:blue")
    ax2.set_ylabel("Thrust (N)")
    ax2.set_xlabel("Time (s)")
    ax2.set_title("Zoom: PWM cycles (thrust)")
    ax2.grid(alpha=0.3)
    ax2.set_xlim(t_zoom.min(), t_zoom.max())
    ax2.set_ylim(thrust_zoom.min() - 0.05 * np.ptp(thrust_zoom),
                 thrust_zoom.max() + 0.05 * np.ptp(thrust_zoom))

    ax3 = fig.add_subplot(gs[1, 1])
    line_zoom_cmd, = ax3.plot([], [], "k--", lw=1, label="Valve command")
    line_zoom_pos, = ax3.plot([], [], color="tab:green", label="Valve position")
    ax3.set_xlabel("Time (s)")
    ax3.set_title("Zoom: valve command vs. position")
    ax3.legend()
    ax3.grid(alpha=0.3)
    ax3.set_xlim(t_zoom.min(), t_zoom.max())
    ax3.set_ylim(-0.05, 1.05)

    fig.tight_layout()

    # ---------------------------------------------------- animated reveal
    # Axis limits above are fixed to their final (full-data) extents before
    # any drawing happens, so sweeping the line data in over time only ever
    # fills in within those limits - the saved PNG is identical to a static
    # one-shot plot of the complete data.
    #
    # Single synchronized loop: the top chart sweeps continuously across the
    # full 0-15 s timeline. The bottom zoom charts stay empty until the top
    # chart's sweep reaches the zoom window's own start time (t0, simulated
    # 6 s), then reveal their data over the remaining frames so both finish
    # together on the final frame.
    idxs_top = np.linspace(0, len(t), ANIMATION_FRAMES + 1, dtype=int)[1:]
    frame_at_t0 = np.searchsorted(idxs_top, np.searchsorted(t, t0))
    idxs_zoom = np.zeros(ANIMATION_FRAMES, dtype=int)
    zoom_frames = ANIMATION_FRAMES - frame_at_t0
    if zoom_frames > 0:
        idxs_zoom[frame_at_t0:] = np.linspace(0, len(t_zoom), zoom_frames + 1, dtype=int)[1:]

    for idx_top, idx_zoom in zip(idxs_top, idxs_zoom):
        line_thrust.set_data(t[:idx_top], thrust[:idx_top])
        line_avg.set_data(t[:idx_top], avg[:idx_top])
        line_zoom_thrust.set_data(t_zoom[:idx_zoom], thrust_zoom[:idx_zoom])
        line_zoom_cmd.set_data(t_zoom[:idx_zoom], cmd_zoom[:idx_zoom])
        line_zoom_pos.set_data(t_zoom[:idx_zoom], pos_zoom[:idx_zoom])
        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        plt.pause(ANIMATION_PAUSE_S)

    fig.savefig("thruster_profile.png", dpi=150)
    print("Saved plot to thruster_profile.png")
    plt.show()


if __name__ == "__main__":
    main()

