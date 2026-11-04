// LSM6DS3 IMU on I2C D2/D3, raw registers (no library). 416 Hz, +-8 g, +-1000 dps.
#pragma once

#include <stdint.h>

typedef struct {
  float accel[3];  // x, y, z in m/s^2 (sensor axes)
  float gyro[3];   // x, y, z in rad/s
} imu_sample_t;

// Finds the IMU at 0x6A or 0x6B, checks WHO_AM_I and sets the ranges.
// Returns false if any step fails.
bool imu_init();

// True if the IMU answers this many reads in a row, 5 ms apart: it has to keep answering,
// not just once. Used at boot to check it is steady on its own I2C controller.
bool imu_steady(int reads);

// Reads the latest sample (does not wait for a new one). Returns false on an I2C error.
bool imu_read(imu_sample_t *out);
