// Automatic laser and IMU calibration: with the hopper still on the pad, averages the sensors
// for a while and takes the averages as their biases:
//   laser: the pad reads 0 mm, so offset = -(average raw range)
//   gyro:  reads 0 at rest, so bias = average
//   accel: the up axis reads +-g at rest and the others 0 (hopper level), so bias = average
//          minus that
// Runs at boot and again on CAL_AUTO from the ground, only while disarmed. Sampling restarts
// if the hopper moves (spread too large) and stops if it is armed.
#pragma once

#include <stdint.h>
#include "../control/sensors.h"

// up_axis / up_sign: which IMU axis points up and its sign (as control.cpp)
void autocal_init(int up_axis, float up_sign, float gravity);

// Starts sampling (again). The biases in use stay until the new ones are ready.
void autocal_start(uint32_t now_ms);

// Call every control loop with the raw sensors. armed stops a calibration in progress (the
// old biases stay). Returns true in the loop the new biases were taken.
bool autocal_update(uint32_t now_ms, const sensors_t *sn, bool armed);

bool autocal_running();

// Removes the IMU biases from a raw sample
void autocal_correct_imu(imu_sample_t *s);

// IMU biases back to 0 (CAL_CLEAR)
void autocal_clear_imu();
