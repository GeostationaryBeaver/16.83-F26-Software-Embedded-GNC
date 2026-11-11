// SAFE -> ARMED -> FAILSAFE state machine, and the only code that moves the valve.
//   SAFE:     valve shut. Goes ARMED when the ground asks, if the link is up, there is no
//             E-stop, and the ground has sent "disarmed" at least once since boot / FAILSAFE.
//   ARMED:    valve follows the controller. Ground disarms -> SAFE.
//   FAILSAFE: no command for 150 ms, E-stop, or the power protection tripping while ARMED on
//             the pad. Valve driven shut, latched until the ground disarms (then SAFE), and
//             has to arm again.
// The power protection (power_guard.h) also blocks arming while it is tripped. While flying
// it does not FAILSAFE: flight.cpp lands instead, while the servo still works.
// The onboard LED shows it: BLUE waiting for the ground, GREEN safe, YELLOW armed,
// RED failsafe or link lost.
#pragma once

#include <stdint.h>

typedef struct {
  bool heard;     // any command received since boot
  bool link_up;   // a command in the last 150 ms
  bool arm;       // from the latest command
  bool estop;
  bool power_trip;  // power protection tripped (power_guard.h)
  bool flying;      // a hop is in the air (ASCENT, HOVER, DESCENT or ABORT)
} safety_inputs_t;

// Valve opening (0-100) the controller wants, called every loop while ARMED
typedef uint8_t (*valve_demand_fn_t)(uint32_t now_ms);

// Holds the valve shut and starts SAFE. Call first thing in setup().
void safety_init();

void safety_update(uint32_t now_ms, const safety_inputs_t *in, valve_demand_fn_t demand);

uint8_t safety_state();      // flight_state_t
uint8_t safety_reason();     // failsafe_reason_t
uint8_t safety_valve_pct();  // opening sent to the servo
uint16_t safety_failsafes(); // since boot
