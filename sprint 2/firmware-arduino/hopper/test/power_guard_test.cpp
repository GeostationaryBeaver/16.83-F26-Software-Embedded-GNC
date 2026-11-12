// Host test of the power protection (control/power_guard.cpp), with a fake servo signal.
// No hardware needed. From firmware-arduino/hopper:
//   c++ -std=c++17 -Itest/stub -Isrc -o /tmp/pg_test test/power_guard_test.cpp src/control/power_guard.cpp \
//     && /tmp/pg_test
#include "Arduino.h"
#include "control/power_guard.h"
#include "drivers/servo.h"

// Fake servo driver: only the signal on/off matters here
static bool g_on = true;
void servo_signal(bool on) { g_on = on; }
bool servo_signal_on() { return g_on; }

static int fails = 0;
#define CHECK(c, msg) do { if (!(c)) { ::printf("  FAIL: %s\n", msg); fails++; } else ::printf("  ok: %s\n", msg); } while (0)

static uint32_t t = 1000;
// Runs the guard every 4 ms for ms milliseconds at a fixed rail and loop current
static void run(uint32_t ms, float v, float ma = 12.0f, bool can_cut = true, bool may_clear = true) {
  for (uint32_t end = t + ms; t < end; t += 4) power_guard_update(t, v, ma, can_cut, may_clear);
}

int main() {
  ::printf("== healthy supply\n");
  run(3000, 6.0f);
  CHECK(!power_guard_tripped() && g_on, "6.0 V: no trip, signal on");

  ::printf("== USB only (no servo power)\n");
  run(3000, 0.2f, 0.0f);
  CHECK(!power_guard_tripped(), "0.2 V rail and no loop is not a fault... ");
  run(3000, 6.0f);  // battery back in

  ::printf("== low rail\n");
  run(400, 5.2f);
  CHECK(!power_guard_tripped(), "5.2 V for 0.4 s: not yet");
  run(200, 5.2f);
  CHECK(power_guard_tripped() && power_guard_cause() == PG_CAUSE_LOW, "5.2 V for 0.6 s: tripped, cause low");
  CHECK(g_on, "signal still on right after the trip");
  run(700, 5.2f);
  CHECK(!g_on, "signal off 0.6 s later (valve commanded shut, not flying)");
  run(4000, 6.0f);
  CHECK(power_guard_tripped(), "4 s of 6.0 V: still tripped");
  run(1200, 6.0f);
  CHECK(!power_guard_tripped() && g_on, "after 5 s steady: cleared, signal back on");

  ::printf("== high rail, flying\n");
  run(400, 7.2f, 12.0f, false);
  CHECK(power_guard_tripped() && power_guard_cause() == PG_CAUSE_HIGH, "7.2 V for 0.4 s: tripped, cause high");
  run(2000, 7.2f, 12.0f, false);
  CHECK(g_on, "flying (can_cut false): signal stays on");
  run(700, 6.0f, 12.0f, true, false);
  CHECK(!g_on, "landed and shut: signal off");
  run(6000, 6.0f, 12.0f, true, false);
  CHECK(power_guard_tripped(), "ground asking to arm: does not clear");
  run(5200, 6.0f);
  CHECK(!power_guard_tripped() && g_on, "disarmed: clears");

  ::printf("== latch after 3 trips\n");
  CHECK(power_guard_trips() == 2, "2 trips so far");
  run(600, 5.0f);
  CHECK(power_guard_latched(), "3rd trip latches");
  run(10000, 6.0f);
  CHECK(power_guard_tripped() && !g_on, "latched: stays tripped and off");

  ::printf("\n%s (%d failures)\n", fails ? "FAILED" : "ALL PASSED", fails);
  return fails != 0;
}
