// Altitude Kalman filter: estimates altitude, vertical velocity and
// accelerometer bias from the IMU (predict) and the laser rangefinder (update).
//
// Assumptions:
// - The IMU z axis is vertical (no attitude estimate yet), so accel_z is used
//   directly. Any tilt shows up as a fake bias (see altitude_kf.cpp).
// - The laser points straight down and measures z directly. Any mounting
//   offset is the caller's job.
// - Call predict every control loop tick. Call update only when the laser has
//   a valid new reading (it is slower than the IMU and can go out of range).
// - Units: m, m/s, m/s^2, s. z is up.
#pragma once

typedef struct {
  float z;        // altitude (m)
  float vz;       // vertical velocity (m/s)
  float b;        // accelerometer z bias (m/s^2)
  float P[3][3];  // state covariance, order [z, vz, b]
} altitude_kf_t;

// Starts at rest at altitude z0 with zero bias.
void altitude_kf_init(altitude_kf_t *kf, float z0);

// accel_z: measured vertical specific force in m/s^2 (reads about +9.81 at
// rest). dt: time since the last predict in s.
void altitude_kf_predict(altitude_kf_t *kf, float accel_z, float dt);

// z_meas: laser altitude in m.
void altitude_kf_update(altitude_kf_t *kf, float z_meas);
