// Pressure, laser and IMU calibrations, set from the ground (cmd_t.cal_op) and kept in
// flash. The pressure and laser keys are the ones the avionics firmware uses, so a board
// keeps those when switched between the two; the accel key is ours.
#pragma once

#include <stdint.h>

// What the sensors read right now, for the ops that calibrate against a live reading.
// The control loop fills this in; calibration_apply() decides whether each op can use it.
typedef struct {
  float pt_ma;       // smoothed pressure loop current (mA)
  float laser_mm;    // smoothed raw range (mm), < 0 = no reading yet
  bool laser_ok;     // the laser is working and its latest range was in range
  float accel_up;    // up-axis acceleration, mounting sign applied (m/s^2)
  float gyro_mag;    // gyro magnitude (rad/s), to check the vehicle is holding still
  bool imu_ok;
} cal_inputs_t;

// Reads the calibration from flash (defaults if there is none)
void calibration_load();

// Applies a cal_op_t from the ground and saves it to flash. Prints the result, or why it
// was refused: a refused calibration is still acknowledged, so the reason appears only on
// the hopper's serial monitor.
void calibration_apply(uint8_t op, int16_t value, const cal_inputs_t *in);

// Calibrated pressure (psi) from the loop current (mA)
float calibration_psi(float pt_ma);

// Calibrated laser height (mm, at least 0) from a raw range
int calibration_height_mm(uint16_t raw_mm);

float calibration_pt_zero_ma();
float calibration_pt_span();
int16_t calibration_laser_offset_mm();

// Accelerometer zero-g offset on the up axis (m/s^2): what the up axis reads at rest minus
// gravity. Subtracted from the up-axis reading before the filter sees it, so the filter's
// bias estimate starts near zero instead of converging from scratch every boot.
float calibration_accel_zero();
