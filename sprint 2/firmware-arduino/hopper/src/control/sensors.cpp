#include <Arduino.h>
#include "sensors.h"
#include "../drivers/pins.h"
#include "../drivers/i2c_bus.h"
#include "../drivers/laser.h"
#include "../drivers/temp_probe.h"
#include "../drivers/pressure.h"
#include "../drivers/servo.h"

#define RETRY_MS 2000      // re-try any sensor that dropped out
#define MAX_FAILS 10       // I2C read errors in a row before a sensor counts as dropped out
#define PT_AVG_ALPHA 0.05f // pressure smoothing per 10 ms update (~0.2 s)
#define LASER_AVG_ALPHA 0.1f

static sensors_t s = {};
static uint8_t s_imu_fails = 0, s_laser_fails = 0;
static uint32_t s_imu_ms = 0, s_laser_ms = 5, s_retry_ms = 0;

static void read_analog() {
  s.pt_ma = pressure_read_ma();
  s.servo_v = servo_rail_volts();
  s.pt_ma_avg = s.pt_ma_avg < 0 ? s.pt_ma : s.pt_ma_avg + PT_AVG_ALPHA * (s.pt_ma - s.pt_ma_avg);
  s.pt_ok = s.pt_ma > PRESSURE_MA_PRESENT;

  bool power = s.servo_v > SERVO_V_MIN;
  if (power != s.servo_power) {
    s.servo_power = power;
    Serial.printf(">>> servo supply %s (%.2f V)\n", power ? "ON" : "OFF", s.servo_v);
  }
}

static void read_imu(uint32_t now_ms) {
  if (!s.imu_ok || now_ms - s_imu_ms < (i2c_bus_dual() ? 10u : 20u)) return;
  s_imu_ms = now_ms;
  if (!imu_read(&s.imu)) {
    s.i2c_errors++;
    if (++s_imu_fails > MAX_FAILS) {
      if (i2c_bus_dual()) {
        i2c_bus_use_shared("IMU stopped answering on its own controller");
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
  if (!s.laser_ok || now_ms - s_laser_ms < (i2c_bus_dual() ? 10u : 33u)) return;
  s_laser_ms = now_ms;
  uint16_t mm;
  laser_result_t r = laser_read(&mm);
  if (r == LASER_NO_DATA) return;
  if (r == LASER_ERROR) {
    s.i2c_errors++;
    if (++s_laser_fails > MAX_FAILS) { s.laser_ok = false; s_laser_fails = 0; }
    return;
  }
  s_laser_fails = 0;
  s.laser_new = true;
  s.laser_in_range = mm < LASER_OUT_OF_RANGE_MM;
  if (s.laser_in_range) {
    s.laser_raw_mm = mm;
    s.laser_avg_mm = s.laser_avg_mm < 0 ? mm : s.laser_avg_mm + LASER_AVG_ALPHA * (mm - s.laser_avg_mm);
  }
}

static void retry(uint32_t now_ms) {
  if (now_ms - s_retry_ms < RETRY_MS) return;
  s_retry_ms = now_ms;
  if (!s.imu_ok) { i2c_bus_restart(I2C_BUS_IMU); s.imu_ok = imu_init(); }
  if (!s.laser_ok) { i2c_bus_restart(I2C_BUS_LASER); s.laser_ok = laser_init(); }
  if (!s.imu_ok || !s.laser_ok) {
    Serial.printf("retrying sensors: laser %s, IMU %s\n", s.laser_ok ? "ok" : "MISSING", s.imu_ok ? "ok" : "MISSING");
  }
}

void sensors_init() {
  s.laser_avg_mm = -1;
  s.pt_ma_avg = -1;
  s.temp_c = NAN;

  i2c_bus_init();
  s.imu_ok = imu_init() && imu_steady(40);
  if (!s.imu_ok) {
    i2c_bus_use_shared("IMU not steady on its own controller");
    s.imu_ok = imu_init() && imu_steady(20);
  }
  s.laser_ok = laser_init();
  s.temp_ok = temp_probe_init();
  read_analog();

  Serial.printf("laser %s | IMU %s (%s) | temp probe %s | pressure loop %.2f mA | servo supply %.2f V (%s)\n",
                s.laser_ok ? "ok" : "MISSING", s.imu_ok ? "ok" : "MISSING",
                i2c_bus_dual() ? "own I2C controller" : "shared controller", s.temp_ok ? "ok" : "MISSING",
                s.pt_ma, s.servo_v, s.servo_power ? "UBEC on" : "no servo power");
}

const sensors_t *sensors_update(uint32_t now_ms) {
  s.imu_new = false;
  s.laser_new = false;
  read_analog();
  read_imu(now_ms);
  read_laser(now_ms);
  temp_probe_update(now_ms);
  s.temp_ok = temp_probe_ok();
  s.temp_c = temp_probe_c();
  retry(now_ms);
  return &s;
}
