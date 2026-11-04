// Reads every sensor on its own schedule from the control loop, keeps their health, and
// retries any that drop out. Laser and IMU each every 10 ms on their own I2C controllers,
// or every 33 / 20 ms on the shared one (every switch costs ~1 ms).
#pragma once

#include <stdint.h>
#include "../drivers/imu.h"

typedef struct {
  // Health
  bool laser_ok, imu_ok, temp_ok;
  bool pt_ok;            // loop current above 3.5 mA (transmitter powered)
  bool servo_power;      // servo rail above 4.5 V (UBEC working)

  // IMU
  bool imu_new;          // a new sample was read in this update
  uint32_t imu_us;       // micros() when it was read
  imu_sample_t imu;      // latest sample

  // Laser
  bool laser_new;        // a new range was read in this update
  bool laser_in_range;   // the latest range saw a surface
  uint16_t laser_raw_mm; // latest in-range raw range
  float laser_avg_mm;    // smoothed raw range (for calibration), -1 = none yet

  float temp_c;          // NaN = no reading
  float pt_ma;           // pressure loop current, latest
  float pt_ma_avg;       // smoothed (~0.2 s)
  float servo_v;
  uint16_t i2c_errors;   // since boot
} sensors_t;

// Starts every sensor and prints what it found. Checks the IMU is steady on its own I2C
// controller (the servo signal must already be running), else switches to the shared one.
void sensors_init();

// Call every control loop (10 ms)
const sensors_t *sensors_update(uint32_t now_ms);
