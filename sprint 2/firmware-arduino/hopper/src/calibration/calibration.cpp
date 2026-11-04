#include <Arduino.h>
#include <Preferences.h>
#include "calibration.h"
#include "../protocol.h"
#include "../drivers/pins.h"

static Preferences s_prefs;
static float s_zero_ua = PRESSURE_MA_ZERO * 1000.0f;
static float s_span = 1.0f;
static int16_t s_laser_off_mm = 0;

static float uncalibrated_psi(float pt_ma) {
  return (pt_ma * 1000.0f - s_zero_ua) / (PRESSURE_MA_SPAN * 1000.0f) * PRESSURE_PSI_FULL_SCALE;
}

static void save() {
  s_prefs.begin("tg-cal", false);
  s_prefs.putFloat("zero_uA", s_zero_ua);
  s_prefs.putFloat("span", s_span);
  s_prefs.putShort("laserOff", s_laser_off_mm);
  s_prefs.end();
}

void calibration_load() {
  s_prefs.begin("tg-cal", true);
  s_zero_ua = s_prefs.getFloat("zero_uA", PRESSURE_MA_ZERO * 1000.0f);
  s_span = s_prefs.getFloat("span", 1.0f);
  s_laser_off_mm = s_prefs.getShort("laserOff", 0);
  s_prefs.end();
}

void calibration_apply(uint8_t op, int16_t value, float pt_ma, float laser_mm) {
  if (op == CAL_ZERO) {
    if (pt_ma < 3.0f || pt_ma > 5.0f) {
      Serial.printf("CAL: zero refused, loop reads %.2f mA (expect ~4 mA vented). Transmitter powered?\n", pt_ma);
      return;
    }
    s_zero_ua = pt_ma * 1000.0f;
    Serial.printf("CAL: pressure zero set at %.3f mA\n", pt_ma);
  } else if (op == CAL_SPAN) {
    float measured = uncalibrated_psi(pt_ma);
    if (measured < 20) {
      Serial.println("CAL: span needs at least 20 psi on the line");
      return;
    }
    s_span = value / measured;
    Serial.printf("CAL: span %.4f (gauge %d psi, sensor %.1f psi)\n", s_span, value, measured);
  } else if (op == CAL_LASER) {
    if (!(laser_mm > 0)) {
      Serial.println("CAL: laser has no reading, point it at a surface");
      return;
    }
    s_laser_off_mm = (int16_t)lroundf(value - laser_mm);
    Serial.printf("CAL: laser offset %+d mm (true %d, raw %.0f)\n", s_laser_off_mm, value, laser_mm);
  } else if (op == CAL_CLEAR) {
    s_zero_ua = PRESSURE_MA_ZERO * 1000.0f;
    s_span = 1.0f;
    s_laser_off_mm = 0;
    Serial.println("CAL: cleared to defaults");
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
