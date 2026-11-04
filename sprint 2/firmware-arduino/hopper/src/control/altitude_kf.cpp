#include "altitude_kf.h"

#define KF_GRAVITY 9.80665f

// Tuning. All values are 1-sigma.
//
// Accelerometer noise (m/s^2), applied as white noise on every predict.
// The LSM6DS3 itself is only ~90 ug/sqrt(Hz), about 0.007 m/s^2 rms at
// 104 Hz ODR, but in flight this term also has to cover motor vibration,
// small tilts and timing jitter, so it is set far above the datasheet.
// Larger = trust the laser more and the IMU less.
#define KF_ACCEL_NOISE 0.5f
// Bias random walk (m/s^2 per sqrt(s)): how fast the accel bias may drift,
// e.g. with temperature. Guess; larger tracks bias changes faster but makes
// the bias estimate noisier.
#define KF_BIAS_RW 0.01f
// Laser noise (m). VL53L0X is a few cm of noise within its ~1.2 m range.
#define KF_LASER_NOISE 0.03f
// Initial uncertainty. Starts on the pad: z is known from the first laser
// reading or the pad height, v_z is ~0. The bias is unknown up to the
// LSM6DS3 zero-g offset (+-40 mg typ = 0.4 m/s^2), so 0.5 m/s^2.
#define KF_INIT_Z 0.05f
#define KF_INIT_VZ 0.1f
#define KF_INIT_B 0.5f

void altitude_kf_init(altitude_kf_t *kf, float z0) {
  kf->z = z0;
  kf->vz = 0.0f;
  kf->b = 0.0f;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) kf->P[i][j] = 0.0f;
  kf->P[0][0] = KF_INIT_Z * KF_INIT_Z;
  kf->P[1][1] = KF_INIT_VZ * KF_INIT_VZ;
  kf->P[2][2] = KF_INIT_B * KF_INIT_B;
}

// x' = F x + G (accel_z - g) + w, with
//   F = [1 dt -dt^2/2]   G = [dt^2/2]
//       [0  1  -dt   ]       [dt    ]
//       [0  0   1    ]       [0     ]
// Q = KF_ACCEL_NOISE^2 * G G^T, plus KF_BIAS_RW^2 * dt on the bias.
void altitude_kf_predict(altitude_kf_t *kf, float accel_z, float dt) {
  float dt2 = 0.5f * dt * dt;
  float a = accel_z - KF_GRAVITY - kf->b;
  kf->z += kf->vz * dt + a * dt2;
  kf->vz += a * dt;

  // A = F P
  float (*P)[3] = kf->P;
  float A[3][3];
  for (int j = 0; j < 3; j++) {
    A[0][j] = P[0][j] + dt * P[1][j] - dt2 * P[2][j];
    A[1][j] = P[1][j] - dt * P[2][j];
    A[2][j] = P[2][j];
  }

  // P = A F^T + Q, upper triangle then mirrored to keep P symmetric
  float qa = KF_ACCEL_NOISE * KF_ACCEL_NOISE;
  float qb = KF_BIAS_RW * KF_BIAS_RW;
  P[0][0] = A[0][0] + dt * A[0][1] - dt2 * A[0][2] + qa * dt2 * dt2;
  P[0][1] = A[0][1] - dt * A[0][2] + qa * dt2 * dt;
  P[0][2] = A[0][2];
  P[1][1] = A[1][1] - dt * A[1][2] + qa * dt * dt;
  P[1][2] = A[1][2];
  P[2][2] = A[2][2] + qb * dt;
  P[1][0] = P[0][1];
  P[2][0] = P[0][2];
  P[2][1] = P[1][2];
}

// H = [1 0 0], R = KF_LASER_NOISE^2
void altitude_kf_update(altitude_kf_t *kf, float z_meas) {
  float (*P)[3] = kf->P;
  float S = P[0][0] + KF_LASER_NOISE * KF_LASER_NOISE;
  float K[3] = {P[0][0] / S, P[1][0] / S, P[2][0] / S};
  float y = z_meas - kf->z;
  kf->z += K[0] * y;
  kf->vz += K[1] * y;
  kf->b += K[2] * y;

  // P = (I - K H) P, i.e. P_ij -= K_i P_0j. Uses the old row 0, so save it.
  float p0[3] = {P[0][0], P[0][1], P[0][2]};
  for (int i = 0; i < 3; i++)
    for (int j = i; j < 3; j++) {
      P[i][j] -= K[i] * p0[j];
      P[j][i] = P[i][j];
    }
}
