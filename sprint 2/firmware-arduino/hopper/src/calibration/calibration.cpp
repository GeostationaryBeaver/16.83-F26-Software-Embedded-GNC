#include <Arduino.h>
#include <Preferences.h>
#include "calibration.h"
#include "../protocol.h"
#include "../drivers/pins.h"

// CAL_IMU reads the accelerometer's error against gravity, so real motion or a tilt is
// indistinguishable from an offset. Both are checked before it will store anything.
#define CAL_IMU_GYRO_MAX   0.05f   // rad/s, about 3 deg/s
#define CAL_IMU_OFFSET_MAX 2.0f    // m/s^2. Beyond this it is a mounting error, not an offset
#define CAL_GRAVITY        9.80665f

static Preferences s_prefs;
static float s_zero_ua = PRESSURE_MA_ZERO * 1000.0f;
static float s_span = 1.0f;
static int16_t s_laser_off_mm = 0;
static float s_accel_zero = 0.0f;

static float uncalibrated_psi(float pt_ma) {
  return (pt_ma * 1000.0f - s_zero_ua) / (PRESSURE_MA_SPAN * 1000.0f) * PRESSURE_PSI_FULL_SCALE;
}

static void save() {
  s_prefs.begin("tg-cal", false);
  s_prefs.putFloat("zero_uA", s_zero_ua);
  s_prefs.putFloat("span", s_span);
  s_prefs.putShort("laserOff", s_laser_off_mm);
  s_prefs.putFloat("accelZero", s_accel_zero);
  s_prefs.end();
}

void calibration_load() {
  s_prefs.begin("tg-cal", true);
  s_zero_ua = s_prefs.getFloat("zero_uA", PRESSURE_MA_ZERO * 1000.0f);
  s_span = s_prefs.getFloat("span", 1.0f);
  s_laser_off_mm = s_prefs.getShort("laserOff", 0);
  s_accel_zero = s_prefs.getFloat("accelZero", 0.0f);
  s_prefs.end();
}

void calibration_apply(uint8_t op, int16_t value, const cal_inputs_t *in) {
  if (op == CAL_ZERO) {
    if (in->pt_ma < 3.0f || in->pt_ma > 5.0f) {
      Serial.printf("CAL: zero refused, loop reads %.2f mA (expect ~4 mA vented). Transmitter powered?\n",
                    in->pt_ma);
      return;
    }
    s_zero_ua = in->pt_ma * 1000.0f;
    Serial.printf("CAL: pressure zero set at %.3f mA\n", in->pt_ma);
  } else if (op == CAL_SPAN) {
    float measured = uncalibrated_psi(in->pt_ma);
    if (measured < 20) {
      Serial.println("CAL: span needs at least 20 psi on the line");
      return;
    }
    s_span = value / measured;
    Serial.printf("CAL: span %.4f (gauge %d psi, sensor %.1f psi)\n", s_span, value, measured);
  } else if (op == CAL_LASER) {
    if (!(in->laser_mm > 0)) {
      Serial.println("CAL: laser has no reading, point it at a surface");
      return;
    }
    s_laser_off_mm = (int16_t)lroundf(value - in->laser_mm);
    Serial.printf("CAL: laser offset %+d mm (true %d, raw %.0f)\n",
                  s_laser_off_mm, value, in->laser_mm);
  } else if (op == CAL_GROUND) {
    // CAL_LASER with a true height of zero: whatever the laser reads with the vehicle
    // standing on the pad becomes 0 mm, so z = 0 at rest and every setpoint is measured
    // from the pad. Saves measuring the standoff by hand, which is the usual reason a
    // hover settles a few cm off its commanded height.
    if (!in->laser_ok || !(in->laser_mm > 0)) {
      Serial.println("CAL: ground refused, no laser reading. On the pad, sensor pointing down?");
      return;
    }
    s_laser_off_mm = (int16_t)lroundf(-in->laser_mm);
    Serial.printf("CAL: ground zero set, laser offset %+d mm (%.0f mm standoff now reads 0)\n",
                  s_laser_off_mm, in->laser_mm);
  } else if (op == CAL_IMU) {
    // At rest the up axis should read exactly +g; what it reads instead is its zero-g
    // offset. Storing it means the filter's bias estimate starts right, rather than
    // converging over the first seconds of every flight.
    if (!in->imu_ok) {
      Serial.println("CAL: IMU refused, no IMU");
      return;
    }
    if (!(in->gyro_mag < CAL_IMU_GYRO_MAX)) {
      Serial.printf("CAL: IMU refused, vehicle is moving (gyro %.3f rad/s, needs < %.2f). Hold it still\n",
                    in->gyro_mag, CAL_IMU_GYRO_MAX);
      return;
    }
    float offset = in->accel_up - CAL_GRAVITY;
    if (fabsf(offset) > CAL_IMU_OFFSET_MAX) {
      Serial.printf("CAL: IMU refused, up axis reads %.2f m/s^2 (%+.2f off %.2f). Check IMU_UP_AXIS "
                    "and IMU_UP_SIGN in control.cpp, and that the vehicle is level\n",
                    in->accel_up, offset, CAL_GRAVITY);
      return;
    }
    s_accel_zero = offset;
    Serial.printf("CAL: IMU accel zero %+.3f m/s^2 (up axis read %.3f at rest)\n",
                  s_accel_zero, in->accel_up);
  } else if (op == CAL_CLEAR) {
    s_zero_ua = PRESSURE_MA_ZERO * 1000.0f;
    s_span = 1.0f;
    s_laser_off_mm = 0;
    s_accel_zero = 0.0f;
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
float calibration_accel_zero() { return s_accel_zero; }
