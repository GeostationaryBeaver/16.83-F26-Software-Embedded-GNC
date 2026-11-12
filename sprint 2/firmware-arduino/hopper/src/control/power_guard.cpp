#include <Arduino.h>
#include "power_guard.h"
#include "../drivers/pins.h"
#include "../drivers/servo.h"

static bool s_trip = false, s_latched = false;
static bool s_pt_seen = false;   // pressure loop healthy at some point since power-up
static bool s_last_pwr = false;  // rail above SERVO_V_MIN last update, for the flicker count
static uint8_t s_trips = 0, s_cause = PG_CAUSE_NONE;
static uint8_t s_flip_n = 0;
static uint32_t s_flip_ms[PG_FLIPS] = {};
static uint32_t s_trip_ms = 0, s_low_ms = 0, s_high_ms = 0, s_loop_ms = 0, s_ok_ms = 0, s_can_cut_ms = 0;

// 0 = not running. Keeps 0 free so a timer started at millis() == 0 still counts as started.
static uint32_t stamp(uint32_t now_ms) { return now_ms ? now_ms : 1; }

// Starts *since_ms when cond becomes true, clears it when false. True once cond has held
// for longer than hold_ms.
static bool held(bool cond, uint32_t now_ms, uint32_t *since_ms, uint32_t hold_ms) {
  if (!cond) { *since_ms = 0; return false; }
  if (!*since_ms) *since_ms = stamp(now_ms);
  return now_ms - *since_ms > hold_ms;
}

static void trip(uint32_t now_ms, uint8_t cause) {
  if (s_trip) return;
  s_trip = true;
  s_cause = cause;
  s_trip_ms = stamp(now_ms);
  s_ok_ms = 0;
  if (++s_trips >= PG_LATCH_TRIPS) s_latched = true;
}

static void clear() {
  s_trip = false;
  s_ok_ms = s_low_ms = s_high_ms = s_loop_ms = s_can_cut_ms = 0;
  s_flip_n = 0;
  servo_signal(true);  // the valve is commanded shut (can_cut), so it comes back pointing at shut
}

void power_guard_update(uint32_t now_ms, float v, float pt_ma, bool can_cut, bool may_clear) {
  // Brown-out loop: the oldest of the last PG_FLIPS rail crossings within the window
  bool pwr = v > SERVO_V_MIN;
  if (pwr != s_last_pwr) {
    s_last_pwr = pwr;
    s_flip_ms[s_flip_n % PG_FLIPS] = now_ms;
    s_flip_n++;
    if (s_flip_n >= PG_FLIPS && now_ms - s_flip_ms[s_flip_n % PG_FLIPS] <= PG_FLIP_WINDOW_MS)
      trip(now_ms, PG_CAUSE_FLICKER);
    if (s_flip_n >= 2 * PG_FLIPS) s_flip_n -= PG_FLIPS;  // same slots, no overflow
  }
  if (held(v >= SERVO_V_ABSENT && v < PG_LOW_V, now_ms, &s_low_ms, PG_LOW_MS)) trip(now_ms, PG_CAUSE_LOW);
  if (held(v > PG_HIGH_V, now_ms, &s_high_ms, PG_HIGH_MS)) trip(now_ms, PG_CAUSE_HIGH);
  if (pt_ma > PRESSURE_MA_PRESENT && pt_ma < PRESSURE_MA_FAULT) s_pt_seen = true;
  bool starved = s_pt_seen && v > SERVO_V_ABSENT && pt_ma < PG_LOOP_MA;
  if (held(starved, now_ms, &s_loop_ms, PG_LOOP_MS)) trip(now_ms, PG_CAUSE_LOOP);

  if (!s_trip) return;

  // Signal off once the valve has been commanded shut for a while and nothing is flying, so the
  // servo has had time to get there while it still had power
  bool cut_ready = held(can_cut, now_ms, &s_can_cut_ms, PG_CUT_DELAY_MS);
  if (servo_signal_on() && cut_ready && now_ms - s_trip_ms > PG_CUT_DELAY_MS) servo_signal(false);

  bool steady = v >= PG_OK_LO_V && v <= PG_OK_HI_V && !(s_pt_seen && pt_ma < PRESSURE_MA_PRESENT);
  if (held(steady, now_ms, &s_ok_ms, PG_OK_MS) && !s_latched && may_clear && can_cut) clear();
}

bool power_guard_tripped() { return s_trip; }
bool power_guard_latched() { return s_latched; }
uint8_t power_guard_cause() { return s_cause; }
uint8_t power_guard_trips() { return s_trips; }
