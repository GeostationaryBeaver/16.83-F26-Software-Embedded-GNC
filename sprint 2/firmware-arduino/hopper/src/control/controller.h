// Outer-loop altitude controller. The only file that decides how far the valve opens
// while ARMED; safety.cpp is still the only file that moves it.
//
// One law serves every mode: a PD tracking a setpoint that may be moving.
//   demand = valve_hover + kp*(sp_z - z) + kd*(sp_vz - vz)
// MODE_HOVER is that setpoint standing still, MODE_VELOCITY is it moving at a constant
// rate, MODE_LAND is it moving down at land_rate. Because velocity tracking carries the
// altitude term too, a descent that falls behind is corrected instead of accumulating,
// and switching modes is bumpless: the setpoint starts from wherever the vehicle is.
#pragma once

#include <stdint.h>
#include "altitude_kf.h"
#include "gains.h"
#include "../protocol.h"

typedef struct {
  uint8_t mode;        // control_mode_t being run
  uint8_t flags;       // CTRL_* bits
  float sp_z;          // altitude setpoint in use (m), NaN in MANUAL
  float sp_vz;         // climb-rate setpoint in use (m/s)
  float demand;        // output before clamping (%), for telemetry
  uint8_t valve_pct;   // what safety.cpp is given
} controller_out_t;

// Zeroes the setpoint state. Call from control_start().
void controller_init();

// Call every tick while ARMED, after the filter has run.
//   kf:          current estimate
//   cmd:         latest command from the ground
//   est_ok:      the laser has corrected the filter recently enough to trust z
//   dt:          seconds since the last call
const controller_out_t *controller_update(const altitude_kf_t *kf, const cmd_t *cmd,
                                          bool est_ok, float dt);

// Call whenever the vehicle is not ARMED. Re-latches the setpoint to the vehicle's
// current altitude so the next arm does not start with a stale error.
void controller_idle(const altitude_kf_t *kf);

// Latest output, for telemetry when the loop did not run this tick.
const controller_out_t *controller_last();

// The gain set compiled in, for telemetry.
const gain_set_t *controller_gains();
