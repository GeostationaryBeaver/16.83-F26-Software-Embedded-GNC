// Flight phases: the hop's guidance. Runs only while the safety state machine is ARMED
// (FAILSAFE and the E-stop always win), and decides where the hopper should be. The
// controller picked by cmd_t.ctrl_mode (controllers.h) decides how to get there.
//
//   IDLE:     valve follows the ground's valve_pct (manual, bench tests). go rising edge +
//             pre-flight checks -> ASCENT. A refused go is shown in abort_reason.
//   ASCENT:   setpoint ramps from the altitude at go up to the hop height at v_up.
//             -> HOVER once the setpoint is at the top and the hopper within HOP_TOL_M.
//   HOVER:    setpoint holds the hop height for hover_ms -> DESCENT.
//   DESCENT:  setpoint ramps down at v_down to below the ground, so the controller keeps
//             pushing down until touchdown. Touchdown -> LANDED.
//   ABORT:    go dropped, too long, too high, or the power protection tripped: same as
//             DESCENT at ABORT_SINK_MPS from wherever the setpoint was. If it hasn't landed
//             in ABORT_LAND_MS, valve shut.
//   LANDED:   valve shut until the ground disarms. Also where a sensor or servo-power loss
//             goes straight away: without them a controlled descent isn't possible.
// Disarming (or FAILSAFE) resets it to IDLE.
//
// The hop profile (height, rates, hover time) comes from the ground in every command,
// clamped to the limits in flight.cpp and latched at go. The abort behaviour and those
// limits are fixed here: changing them takes a reflash.
#pragma once

#include <stdint.h>

// What the control loop does with the valve this loop
typedef enum {
  FLIGHT_MANUAL = 0,  // valve = the ground's valve_pct (IDLE)
  FLIGHT_TRACK = 1,   // run the controller on z_sp / vz_sp
  FLIGHT_CUT = 2,     // valve shut
} flight_action_t;

typedef struct {
  uint8_t action;  // flight_action_t
  float z_sp;      // altitude setpoint (m), NaN unless TRACK
  float vz_sp;     // its rate (m/s): the controllers' velocity feedforward. NaN unless TRACK
} flight_ref_t;

typedef struct {
  float hop_m;       // hop height (m)
  float v_up;        // climb rate (m/s)
  float v_down;      // descent rate (m/s)
  uint32_t hover_ms; // time at the hop height
} flight_profile_t;

typedef struct {
  float z, vz;              // altitude filter (m, m/s)
  bool go;                  // from the latest command
  bool closed_loop;         // ctrl_mode is a closed-loop controller (not MANUAL)
  bool nav_ok;              // IMU ok and a fresh in-range laser range: the filter is trustworthy
  uint8_t health;           // HEALTH_* bits. HEALTH_SERVO_PWR here is the rail itself: its loss
                            // cuts the valve, a power protection trip (below) lands instead
  bool power_trip;          // power protection tripped (power_guard.h): ABORT, go refused
  flight_profile_t request; // the ground's profile, already through flight_clamp_profile()
  bool request_clamped;     // and it had to be clamped
} flight_inputs_t;

// The ground's profile clamped to the allowed limits. *clamped (if not NULL) is set when
// any field was out of range.
flight_profile_t flight_clamp_profile(int16_t hop_mm, uint16_t v_up_mms, uint16_t v_down_mms,
                                      uint16_t hover_ms, bool *clamped);

// Back to IDLE. Call every loop the hopper is not ARMED.
void flight_reset();

// Call every loop while ARMED
flight_ref_t flight_update(uint32_t now_ms, const flight_inputs_t *in);

uint8_t flight_phase();                   // flight_phase_t
uint8_t flight_abort_reason();            // abort_reason_t: last ABORT, or why go was refused
const flight_profile_t *flight_profile(); // latched at the last go
