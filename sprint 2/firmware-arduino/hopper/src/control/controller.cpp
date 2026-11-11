#include <Arduino.h>
#include <math.h>
#include "controller.h"

static const gain_set_t s_g = ACTIVE_GAINS;
static controller_out_t s_out;
static float s_sp_z = 0.0f;     // ramped altitude setpoint
static bool s_sp_valid = false; // s_sp_z has been latched to the vehicle
static bool s_gnd_shut = false; // latched by MODE_LAND at touchdown

static void clear_out() {
  s_out.mode = MODE_MANUAL;
  s_out.flags = 0;
  s_out.sp_z = NAN;
  s_out.sp_vz = 0.0f;
  s_out.demand = 0.0f;
  s_out.valve_pct = 0;
}

void controller_init() {
  clear_out();
  s_sp_z = 0.0f;
  s_sp_valid = false;
  s_gnd_shut = false;
}

static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

void controller_idle(const altitude_kf_t *kf) {
  s_sp_z = kf->z;
  s_sp_valid = true;
  s_gnd_shut = false;
  clear_out();
}

const controller_out_t *controller_update(const altitude_kf_t *kf, const cmd_t *cmd,
                                          bool est_ok, float dt) {
  uint8_t mode = cmd->mode;
  if (mode > MODE_LAND) mode = MODE_MANUAL;  // a mode we don't know is not a mode we fly
  s_out.mode = mode;
  s_out.flags = 0;

  if (mode == MODE_MANUAL) {
    s_sp_valid = false;  // re-latch on the way back into a closed-loop mode
    s_gnd_shut = false;
    s_out.sp_z = NAN;
    s_out.sp_vz = 0.0f;
    s_out.demand = cmd->valve_pct;
    s_out.valve_pct = cmd->valve_pct > 100 ? 100 : cmd->valve_pct;
    return &s_out;
  }

  // Closed loop from here on, and z is the thing being held. Without a recent laser
  // fix the filter is coasting on the IMU and z drifts, so there is nothing safe to
  // hold: shut the valve rather than chase a number that is quietly running away.
  if (!est_ok) {
    s_sp_valid = false;
    s_out.flags |= CTRL_NO_EST;
    s_out.sp_z = NAN;
    s_out.sp_vz = 0.0f;
    s_out.demand = 0.0f;
    s_out.valve_pct = 0;
    return &s_out;
  }

  if (!s_sp_valid) {  // first closed-loop tick: start from where the vehicle actually is
    s_sp_z = kf->z;
    s_sp_valid = true;
  }

  // ---- setpoint ----
  float sp_vz = 0.0f;
  if (mode == MODE_HOVER) {
    float target = clampf(isnan(cmd->setpoint) ? s_sp_z : cmd->setpoint, 0.0f, SP_Z_MAX);
    float step = s_g.sp_rate * dt;
    float err = target - s_sp_z;
    if (fabsf(err) <= step) {
      s_sp_z = target;
    } else {
      s_sp_z += (err > 0 ? step : -step);
      sp_vz = (err > 0 ? s_g.sp_rate : -s_g.sp_rate);  // feed the ramp forward
      s_out.flags |= CTRL_SP_RAMP;
    }
  } else {
    sp_vz = (mode == MODE_LAND) ? -s_g.land_rate
                                : clampf(isnan(cmd->setpoint) ? 0.0f : cmd->setpoint,
                                         -SP_VZ_MAX, SP_VZ_MAX);
    s_sp_z += sp_vz * dt;
    // Integrating the rate command would otherwise walk the setpoint past the range the
    // laser can see, or underground, and the altitude term would wind up against a
    // number the vehicle can never reach.
    s_sp_z = clampf(s_sp_z, 0.0f, SP_Z_MAX);
  }

  // ---- touchdown ----
  // Latched: without the latch the vehicle settles at the cutoff, the valve reopens as
  // the error grows, and it lifts off again into a limit cycle a few cm over the pad.
  if (mode == MODE_LAND && kf->z <= s_g.land_cutoff) s_gnd_shut = true;
  if (s_gnd_shut) {
    s_out.flags |= CTRL_GND_SHUT;
    s_out.sp_z = s_sp_z;
    s_out.sp_vz = 0.0f;
    s_out.demand = 0.0f;
    s_out.valve_pct = 0;
    return &s_out;
  }

  // ---- PD ----
  float demand = s_g.valve_hover + s_g.kp * (s_sp_z - kf->z) + s_g.kd * (sp_vz - kf->vz);
  s_out.demand = demand;
  if (demand >= s_g.valve_max) s_out.flags |= CTRL_SAT_HIGH;
  if (demand <= s_g.valve_min) s_out.flags |= CTRL_SAT_LOW;
  float v = clampf(demand, s_g.valve_min, s_g.valve_max);
  v = clampf(v, 0.0f, 100.0f);

  s_out.sp_z = s_sp_z;
  s_out.sp_vz = sp_vz;
  s_out.valve_pct = (uint8_t)lroundf(v);
  return &s_out;
}

const controller_out_t *controller_last() { return &s_out; }
const gain_set_t *controller_gains() { return &s_g; }
