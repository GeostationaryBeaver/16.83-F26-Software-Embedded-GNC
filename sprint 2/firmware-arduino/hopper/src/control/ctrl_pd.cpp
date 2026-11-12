// PD on altitude: one loop from position error straight to valve opening.
//   u = hover + kp * (z_sp - z) - kd * (vz - vz_sp)
// The D term uses the filter's vz against the setpoint's own rate, not a differentiated
// error, so a setpoint step doesn't kick the valve. Without vz_sp a ramp at v would be
// tracked (kd / kp) * v behind: 19 cm at 0.25 m/s with the default gains. No integrator: any
// error in hover_pct (tank blowing down, wrong guess) shows up as a steady altitude offset of
// (hover error) / kp.
// Gains: pd_kp, pd_kd and hover_pct in ctrl_gains_t (defaults in controllers.cpp).
#include "controllers.h"

void pd_reset() {}

float pd_update(const ctrl_in_t *in) {
  const ctrl_gains_t *g = controller_gains();
  return g->hover_pct + g->pd_kp * (in->z_sp - in->z) - g->pd_kd * (in->vz - in->vz_sp);
}
