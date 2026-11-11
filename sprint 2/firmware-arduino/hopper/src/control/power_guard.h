// Power protection, from the avionics vehicle firmware v1.3a: protects the servo, the valve
// and the battery. Thresholds in drivers/pins.h (PG_*).
//
// Trips when:
//   - the servo rail is below 5.4 V for 0.5 s (battery flat, UBEC or wiring can't keep up).
//     Only with some servo power at all (above 1 V), so a USB-only bench is not a fault
//   - the servo rail is above 6.9 V for 0.3 s (UBEC jumper not on 6.0 V: cooks the servo)
//   - the rail crosses 4.5 V 4 times in 10 s (brown-out loop)
//   - the pressure loop falls under 3.3 mA for 2 s after it was healthy since power-up
//     (12 V battery too low for the transmitter, or it was unplugged)
//
// What a trip does (wired in control.cpp, safety.cpp and flight.cpp):
//   - arming and go are refused; telemetry drops HEALTH_SERVO_PWR and sets FLAG_POWER_TRIP
//   - ARMED on the pad: FAILSAFE (REASON_POWER), valve shut
//   - flying: ABORT (ABORT_POWER), a controlled descent while the servo still works
//   - once not flying and the valve has been commanded shut for 0.6 s, the servo signal goes
//     OFF so the servo stops pulling current (FLAG_SERVO_OFF)
// It clears by itself after 5 s of a steady 5.6-6.6 V rail (and a healthy pressure loop)
// while the ground is not asking to arm. After 3 trips since power-up it stays tripped until
// the hopper restarts (FLAG_POWER_LATCH).
//
// It never moves the valve: safety.cpp does that. It only switches the servo signal off and on.
#pragma once

#include <stdint.h>

typedef enum {
  PG_CAUSE_NONE = 0,
  PG_CAUSE_LOW = 1,      // servo rail low
  PG_CAUSE_HIGH = 2,     // servo rail high
  PG_CAUSE_FLICKER = 3,  // servo rail switching off and on
  PG_CAUSE_LOOP = 4,     // pressure loop starved
} pg_cause_t;

// Call every control loop, after the sensors.
//   servo_v:   servo rail (V)
//   pt_ma:     smoothed pressure loop current (mA)
//   can_cut:   not flying and the valve is commanded shut, so the signal may go off
//   may_clear: the ground is not asking to arm, so a trip may clear by itself
void power_guard_update(uint32_t now_ms, float servo_v, float pt_ma, bool can_cut, bool may_clear);

bool power_guard_tripped();
bool power_guard_latched();
uint8_t power_guard_cause();  // pg_cause_t of the latest trip (kept after it clears)
uint8_t power_guard_trips();  // since power-up
