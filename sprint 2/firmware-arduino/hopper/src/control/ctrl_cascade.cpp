// Cascaded PID on altitude.
//   outer (position): vz_sp feedforward + PID from altitude error to a velocity setpoint,
//                     limited to +-vz_max. The PID only corrects drift off the ramp.
//   inner (velocity): PID from velocity error to valve opening, around hover_pct
// Both derivative terms act on the measurement (vz - vz_sp for the outer loop, the IMU's vertical
// acceleration for the inner one) instead of differentiating the error. That way a setpoint
// step doesn't spike the valve, and no noisy signal gets differentiated.
// The inner loop should be several times faster than the outer one: tune it first with the
// outer gains at zero and a fixed velocity setpoint, then close the outer loop.
#include <math.h>
#include "controllers.h"

// Gains: pos_* (outer), vel_* (inner), vz_max and hover_pct in ctrl_gains_t (defaults in
// controllers.cpp). The outer integrator is limited to +-vz_max.

// Integrator state: ki * integral of the error, so changing ki doesn't step the output
static float s_pos_i = 0.0f;
static float s_vel_i = 0.0f;

// rate: time derivative of the measured signal, so the D term is -kd * rate.
// i_max: limit on the integrator, in output units. lo, hi: output limits. The integrator
// holds while the output is pinned at a limit and the error would push it further (anti-windup).
static float pid_step(float *i, float kp, float ki, float kd, float i_max, float err, float rate,
                      float dt, float bias, float lo, float hi) {
  float u = bias + kp * err + *i - kd * rate;
  bool pinned = (u >= hi && err > 0.0f) || (u <= lo && err < 0.0f);
  if (!pinned) {
    *i += ki * err * dt;
    *i = fminf(fmaxf(*i, -i_max), i_max);
  }
  return fminf(fmaxf(u, lo), hi);
}

void cascade_reset() {
  s_pos_i = 0.0f;
  s_vel_i = 0.0f;
}

float cascade_update(const ctrl_in_t *in) {
  const ctrl_gains_t *g = controller_gains();
  float vz_cmd = pid_step(&s_pos_i, g->pos_kp, g->pos_ki, g->pos_kd, g->vz_max,
                          in->z_sp - in->z, in->vz - in->vz_sp, in->dt, in->vz_sp,
                          -g->vz_max, g->vz_max);
  return pid_step(&s_vel_i, g->vel_kp, g->vel_ki, g->vel_kd, g->vel_i_max,
                  vz_cmd - in->vz, in->az, in->dt, g->hover_pct, 0.0f, 100.0f);
}
