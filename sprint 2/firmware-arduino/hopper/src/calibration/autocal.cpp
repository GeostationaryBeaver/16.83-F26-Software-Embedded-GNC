#include <Arduino.h>
#include <math.h>
#include <string.h>
#include "autocal.h"
#include "calibration.h"

#define AUTOCAL_SETTLE_MS 500   // wait after a start before sampling (boot, servo moving shut)
#define AUTOCAL_SAMPLE_MS 3000  // how long to average for
#define AUTOCAL_MIN_IMU 100     // fewest samples to accept (250 Hz own bus, 50 Hz shared)
#define AUTOCAL_MIN_LASER 20    // (100 Hz own bus, 30 Hz shared)
// Largest spread (standard deviation) for the hopper to count as still. Above it, someone
// moved it: sampling starts over.
#define AUTOCAL_MAX_ACCEL_SD 0.3f   // m/s^2
#define AUTOCAL_MAX_GYRO_SD 0.05f   // rad/s
#define AUTOCAL_MAX_LASER_SD 15.0f  // mm

typedef struct {
  uint32_t n;
  double sum, sum2;
} stat_t;

static void stat_add(stat_t *s, double x) {
  s->n++;
  s->sum += x;
  s->sum2 += x * x;
}
static double stat_mean(const stat_t *s) { return s->sum / s->n; }
static double stat_sd(const stat_t *s) {
  double m = stat_mean(s);
  double v = s->sum2 / s->n - m * m;
  return v > 0 ? sqrt(v) : 0;
}

static int s_up_axis = 2;
static float s_up_sign = 1.0f, s_gravity = 9.80665f;

static bool s_running = false;
static uint32_t s_start_ms = 0;
static stat_t s_accel[3], s_gyro[3], s_laser;

static float s_accel_bias[3] = {0, 0, 0};
static float s_gyro_bias[3] = {0, 0, 0};

static void reset_stats() {
  memset(s_accel, 0, sizeof(s_accel));
  memset(s_gyro, 0, sizeof(s_gyro));
  memset(&s_laser, 0, sizeof(s_laser));
}

void autocal_init(int up_axis, float up_sign, float gravity) {
  s_up_axis = up_axis;
  s_up_sign = up_sign;
  s_gravity = gravity;
}

void autocal_start(uint32_t now_ms) {
  s_running = true;
  s_start_ms = now_ms;
  reset_stats();
}

bool autocal_running() { return s_running; }

// True if the samples so far are steady enough to use
static bool still(bool imu, bool laser) {
  for (int i = 0; imu && i < 3; i++) {
    if (stat_sd(&s_accel[i]) > AUTOCAL_MAX_ACCEL_SD || stat_sd(&s_gyro[i]) > AUTOCAL_MAX_GYRO_SD)
      return false;
  }
  return !laser || stat_sd(&s_laser) <= AUTOCAL_MAX_LASER_SD;
}

static void finish() {
  bool imu = s_accel[0].n >= AUTOCAL_MIN_IMU;
  bool laser = s_laser.n >= AUTOCAL_MIN_LASER;
  if (imu) {
    for (int i = 0; i < 3; i++) {
      float expect = i == s_up_axis ? s_up_sign * s_gravity : 0.0f;
      s_accel_bias[i] = (float)stat_mean(&s_accel[i]) - expect;
      s_gyro_bias[i] = (float)stat_mean(&s_gyro[i]);
    }
  }  // too few IMU samples (IMU out): its biases stay
  if (laser) {
    calibration_set_laser_offset_mm((int16_t)lround(-stat_mean(&s_laser)));
  }  // too few in-range laser readings: the offset stays
  s_running = false;
}

bool autocal_update(uint32_t now_ms, const sensors_t *sn, bool armed) {
  if (!s_running) return false;
  if (armed) {
    s_running = false;
    return false;
  }
  uint32_t t = now_ms - s_start_ms;
  if (t < AUTOCAL_SETTLE_MS) return false;

  if (sn->imu_ok && sn->imu_new) {
    for (int i = 0; i < 3; i++) {
      stat_add(&s_accel[i], sn->imu.accel[i]);
      stat_add(&s_gyro[i], sn->imu.gyro[i]);
    }
  }
  if (sn->laser_ok && sn->laser_new && sn->laser_in_range) stat_add(&s_laser, sn->laser_raw_mm);

  if (t < AUTOCAL_SETTLE_MS + AUTOCAL_SAMPLE_MS) return false;
  bool imu = s_accel[0].n >= AUTOCAL_MIN_IMU;
  bool laser = s_laser.n >= AUTOCAL_MIN_LASER;
  if (!still(imu, laser)) {
    s_start_ms = now_ms - AUTOCAL_SETTLE_MS;  // no settle again, sample straight away
    reset_stats();
    return false;
  }
  finish();
  return true;
}

void autocal_correct_imu(imu_sample_t *s) {
  for (int i = 0; i < 3; i++) {
    s->accel[i] -= s_accel_bias[i];
    s->gyro[i] -= s_gyro_bias[i];
  }
}

void autocal_clear_imu() {
  for (int i = 0; i < 3; i++) s_accel_bias[i] = s_gyro_bias[i] = 0;
}
