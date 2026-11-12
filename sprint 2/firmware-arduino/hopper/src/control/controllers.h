// Altitude control laws. The ground picks one with cmd_t.ctrl_mode (ctrl_mode_t in
// protocol.h); control.cpp latches it while the hopper is not ARMED, and calls reset() on
// the way into ARMED. A mode change sent while ARMED waits until the next arm.
//
// To add a controller: write ctrl_<name>.cpp with its own reset/update, add it to
// ctrl_mode_t in protocol.h (both copies) and to CONTROLLERS[] in controllers.cpp, and add
// it to the dashboards' controller lists.
#pragma once

#include <stdint.h>
#include "../protocol.h"

// Everything a controller may read. Filled once per control loop by control.cpp.
typedef struct {
  float dt;            // s since the last loop, measured (nominal 0.004, at most 0.02)
  float z, vz;         // altitude filter: altitude (m), vertical velocity (m/s), z up
  float az;            // vertical acceleration from the IMU, gravity and bias removed (m/s^2)
  float z_sp;          // altitude setpoint from the flight phase (m)
  float vz_sp;         // its rate (m/s): velocity feedforward, so a ramp is tracked without lag
  uint8_t manual_pct;  // cmd_t.valve_pct, the ground's slider
} ctrl_in_t;

typedef struct {
  const char *name;
  void (*reset)();                    // clears integrators and memory, called on arm
  float (*update)(const ctrl_in_t *); // valve opening (%), clamped to 0-100 by the caller
} controller_t;

// Index = ctrl_mode_t
extern const controller_t CONTROLLERS[CTRL_COUNT];

// The controller for a mode from the ground. Unknown modes fall back to CTRL_MANUAL.
uint8_t controller_mode(uint8_t requested);

// Runs one controller and clamps its output to a valve opening. NaN = shut.
uint8_t controller_run(uint8_t mode, const ctrl_in_t *in);

// Gains (ctrl_gains_t in protocol.h). Tunable from the ground without reflashing: control.cpp
// takes MSG_SET_GAINS while not ARMED, and gains_store.cpp keeps them in flash.
// Only the control task may call these.
extern const ctrl_gains_t CTRL_DEFAULT_GAINS;  // compiled in, used until the ground sends others

// Gains in use. The controllers read these on every update.
const ctrl_gains_t *controller_gains();

// Checks every gain is finite and in range (controllers.cpp), then puts them in use.
// Returns false and keeps the old gains if any isn't. Doesn't save them.
bool controller_set_gains(const ctrl_gains_t *g);
