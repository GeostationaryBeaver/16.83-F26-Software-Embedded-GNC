// Pressure and laser calibrations, set from the ground (cmd_t.cal_op) and kept in flash.
// Uses the same flash keys as the avionics firmware, so a board keeps its calibration
// when switched between the two.
#pragma once

#include <stdint.h>

// Reads the calibration from flash (defaults if there is none)
void calibration_load();

// Applies a cal_op_t from the ground and saves it to flash. pt_ma and laser_mm are the
// smoothed readings now (laser_mm < 0 = no reading). Prints the result, or why it was refused.
void calibration_apply(uint8_t op, int16_t value, float pt_ma, float laser_mm);

// Calibrated pressure (psi) from the loop current (mA)
float calibration_psi(float pt_ma);

// Calibrated laser height (mm, at least 0) from a raw range
int calibration_height_mm(uint16_t raw_mm);

float calibration_pt_zero_ma();
float calibration_pt_span();
int16_t calibration_laser_offset_mm();
float calibration_pt_full_psi();  // pressure transmitter full scale in use (CAL_PTRANGE)
