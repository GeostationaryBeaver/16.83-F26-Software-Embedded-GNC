#include <Arduino.h>
#include "sensors.h"
#include "../drivers/pins.h"
#include "../drivers/i2c_bus.h"
#include "../drivers/laser.h"
#include "../drivers/temp_probe.h"
#include "../drivers/pressure.h"
#include "../drivers/servo.h"

#define RETRY_MS 2000      // re-try any sensor that dropped out
#define IMU_MAX_FAILS 10   // IMU read errors in a row before it counts as dropped out (40 ms)
#define LASER_MAX_FAILS 5  // laser errors or stalls in a row before it counts as dropped out
#define PT_AVG_ALPHA 0.02f // pressure smoothing per 4 ms update (~0.2 s)
#define LASER_AVG_ALPHA 0.1f

static sensors_t s = {};
static uint8_t s_imu_fails = 0, s_laser_fails = 0;
static uint32_t s_imu_ms = 0, s_laser_ms = 5, s_retry_ms = 0;

// True when a read every period_ms is due. Steps the schedule by period_ms rather than
// resetting it to now, so a period that isn't a multiple of the 4 ms control loop still
// averages out right (10 ms -> reads 8 / 12 ms apart, not every 12 ms). Re-syncs to now
// after a long stall instead of firing a burst of catch-up reads.
static bool due(uint32_t now_ms, uint32_t *last_ms, uint32_t period_ms) {
  if (now_ms - *last_ms < period_ms) return false;
  *last_ms += period_ms;
  if (now_ms - *last_ms >= period_ms) *last_ms = now_ms;
  return true;
}

static void read_analog() {
  s.pt_ma = pressure_read_ma();
  s.servo_v = servo_rail_volts();
  s.pt_ma_avg = s.pt_ma_avg < 0 ? s.pt_ma : s.pt_ma_avg + PT_AVG_ALPHA * (s.pt_ma - s.pt_ma_avg);
  s.pt_ok = s.pt_ma_avg > PRESSURE_MA_PRESENT && s.pt_ma_avg < PRESSURE_MA_FAULT;
  s.servo_power = s.servo_v > SERVO_V_MIN;
}

static void read_imu(uint32_t now_ms) {
  // Own controller: every control loop (250 Hz; the IMU outputs at 416 Hz). Period 0 rather
  // than 4 so a millis() edge landing 3 ms after the last read can't skip a loop.
  if (!s.imu_ok || !due(now_ms, &s_imu_ms, i2c_bus_dual() ? 0u : 20u)) return;
  if (!imu_read(&s.imu)) {
    s.i2c_errors++;
    if (++s_imu_fails > IMU_MAX_FAILS) {
      if (i2c_bus_dual()) {
        i2c_bus_use_shared();
        s.imu_ok = imu_init();
      } else {
        s.imu_ok = false;
      }
      s_imu_fails = 0;
    }
    return;
  }
  s_imu_fails = 0;
  s.imu_new = true;
  s.imu_us = micros();
}

static void read_laser(uint32_t now_ms) {
  if (!s.laser_ok || !due(now_ms, &s_laser_ms, i2c_bus_dual() ? 10u : 33u)) return;
  uint16_t mm;
  laser_result_t r = laser_read(&mm);
  if (r == LASER_NO_DATA) return;
  if (r == LASER_ERROR || r == LASER_STALLED) {
    if (r == LASER_ERROR) s.i2c_errors++;
    if (++s_laser_fails >= LASER_MAX_FAILS) {
      s.laser_ok = false;
      s_laser_fails = 0;
    }
    return;
  }
  s_laser_fails = 0;
  s.laser_new = true;
  s.laser_in_range = mm < LASER_OUT_OF_RANGE_MM;
  if (s.laser_in_range) {
    s.laser_raw_mm = mm;
    s.laser_fix_ms = now_ms ? now_ms : 1;
    s.laser_avg_mm = s.laser_avg_mm < 0 ? mm : s.laser_avg_mm + LASER_AVG_ALPHA * (mm - s.laser_avg_mm);
  }
}

static void retry(uint32_t now_ms) {
  if (now_ms - s_retry_ms < RETRY_MS) return;
  s_retry_ms = now_ms;
  if (!s.imu_ok) {
    i2c_bus_restart(I2C_BUS_IMU);
    s.imu_ok = imu_init();
  }
  if (!s.laser_ok) {
    i2c_bus_restart(I2C_BUS_LASER);
    s.laser_ok = laser_init();
  }
}

void sensors_init() {
  s.laser_avg_mm = -1;
  s.pt_ma_avg = -1;
  s.temp_c = NAN;

  i2c_bus_init();
  s.imu_ok = imu_init() && imu_steady(40);
  if (!s.imu_ok) {
    i2c_bus_use_shared();
    s.imu_ok = imu_init() && imu_steady(20);
  }
  s.laser_ok = laser_init();
  temp_probe_init();
  s.temp_ok = false;  // until the first good reading
  s.pt_ma_avg = pressure_read_ma();  // start the average at the real value, not 0
  read_analog();
}

const sensors_t *sensors_update(uint32_t now_ms, bool allow_retry) {
  s.imu_new = false;
  s.laser_new = false;
  read_analog();
  read_imu(now_ms);
  read_laser(now_ms);
  temp_probe_update(now_ms);
  s.temp_ok = temp_probe_ok();
  s.temp_c = temp_probe_c();
  if (allow_retry) retry(now_ms);
  return &s;
}
