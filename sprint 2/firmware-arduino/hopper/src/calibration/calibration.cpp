#include <Arduino.h>
#include <Preferences.h>
#include "calibration.h"
#include "../protocol.h"
#include "../drivers/pins.h"

static Preferences s_prefs;
static float s_zero_ua = PRESSURE_MA_ZERO * 1000.0f;
static float s_span = 1.0f;
static int16_t s_laser_off_mm = 0;
static float s_pt_full_psi = PRESSURE_PSI_FULL_SCALE;  // transmitter full scale (20 mA)

#define PT_FULL_MIN_PSI 100.0f    // same limits as the avionics ptrange
#define PT_FULL_MAX_PSI 20000.0f

static float uncalibrated_psi(float pt_ma) {
  return (pt_ma * 1000.0f - s_zero_ua) / (PRESSURE_MA_SPAN * 1000.0f) * s_pt_full_psi;
}

static void save() {
  s_prefs.begin("tg-cal", false);
  s_prefs.putFloat("zero_uA", s_zero_ua);
  s_prefs.putFloat("span", s_span);
  s_prefs.putShort("laserOff", s_laser_off_mm);
  s_prefs.putFloat("ptFull", s_pt_full_psi);
  s_prefs.end();
}

void calibration_load() {
  s_prefs.begin("tg-cal", true);
  s_zero_ua = s_prefs.getFloat("zero_uA", PRESSURE_MA_ZERO * 1000.0f);
  s_span = s_prefs.getFloat("span", 1.0f);
  s_laser_off_mm = s_prefs.getShort("laserOff", 0);
  float full = s_prefs.getFloat("ptFull", PRESSURE_PSI_FULL_SCALE);
  s_prefs.end();
  s_pt_full_psi = (isfinite(full) && full >= PT_FULL_MIN_PSI && full <= PT_FULL_MAX_PSI)
                      ? full : PRESSURE_PSI_FULL_SCALE;
}

void calibration_apply(uint8_t op, int16_t value, float pt_ma, float laser_mm) {
  if (op == CAL_ZERO) {
    if (pt_ma < 3.0f || pt_ma > 5.0f) {
      return;
    }
    s_zero_ua = pt_ma * 1000.0f;
  } else if (op == CAL_SPAN) {
    float measured = uncalibrated_psi(pt_ma);
    if (measured < 20) {
      return;
    }
    s_span = value / measured;
  } else if (op == CAL_LASER) {
    if (!(laser_mm > 0)) {
      return;
    }
    s_laser_off_mm = (int16_t)lroundf(value - laser_mm);
  } else if (op == CAL_PTRANGE) {
    if (value < PT_FULL_MIN_PSI || value > PT_FULL_MAX_PSI) {
      return;
    }
    // A new transmitter: its span is unknown, so back to 1. Vent, zero, then span against a gauge.
    s_pt_full_psi = value;
    s_span = 1.0f;
  } else if (op == CAL_CLEAR) {
    s_zero_ua = PRESSURE_MA_ZERO * 1000.0f;
    s_span = 1.0f;
    s_laser_off_mm = 0;  // the transmitter range is kept: it is the part fitted, not a calibration
  } else {
    return;
  }
  save();
}

float calibration_psi(float pt_ma) { return uncalibrated_psi(pt_ma) * s_span; }

int calibration_height_mm(uint16_t raw_mm) { return max(0, (int)raw_mm + s_laser_off_mm); }

float calibration_pt_zero_ma() { return s_zero_ua / 1000.0f; }
float calibration_pt_span() { return s_span; }
int16_t calibration_laser_offset_mm() { return s_laser_off_mm; }
void calibration_set_laser_offset_mm(int16_t mm) { s_laser_off_mm = mm; }
float calibration_pt_full_psi() { return s_pt_full_psi; }
