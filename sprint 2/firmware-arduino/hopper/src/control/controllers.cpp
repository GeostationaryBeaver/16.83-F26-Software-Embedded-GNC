#include <math.h>
#include "controllers.h"

// Manual: the ground's valve slider, unchanged
static void manual_reset() {}
static float manual_update(const ctrl_in_t *in) { return in->manual_pct; }

// In ctrl_pd.cpp and ctrl_cascade.cpp
void pd_reset();
float pd_update(const ctrl_in_t *in);
void cascade_reset();
float cascade_update(const ctrl_in_t *in);

// Same order as ctrl_mode_t
const controller_t CONTROLLERS[CTRL_COUNT] = {
  {"manual", manual_reset, manual_update},          // CTRL_MANUAL
  {"PD position", pd_reset, pd_update},             // CTRL_PD_POS
  {"cascaded PID", cascade_reset, cascade_update},  // CTRL_CASCADE
};

// TUNE these defaults, or tune live from the ground station (Gains panel) and copy the
// numbers back here once they're good. hover_pct depends on the tank pressure and the
// valve's flow curve (thrust_sim/valve_characteristic.png), so measure it on the stand.
const ctrl_gains_t CTRL_DEFAULT_GAINS = {
  .hover_pct = 50.0f,
  // PD position (ctrl_pd.cpp)
  .pd_kp = 20.0f,
  .pd_kd = 15.0f,
  // Cascade outer loop (ctrl_cascade.cpp). vz_max above the fastest ramp (0.5 m/s climb) so
  // the feedforward is never clipped
  .pos_kp = 1.5f,
  .pos_ki = 0.0f,
  .pos_kd = 0.0f,
  .vz_max = 0.6f,
  // Cascade inner loop
  .vel_kp = 25.0f,
  .vel_ki = 10.0f,
  .vel_kd = 2.0f,
  .vel_i_max = 20.0f,
};

// Limits the ground can set. Generous: they catch typos and garbage, not bad tuning.
#define GAIN_MAX 1000.0f
#define VZ_MAX_LIMIT 2.0f  // m/s

static ctrl_gains_t s_gains = CTRL_DEFAULT_GAINS;

static bool in_range(float x, float lo, float hi) { return isfinite(x) && x >= lo && x <= hi; }

const ctrl_gains_t *controller_gains() { return &s_gains; }

bool controller_set_gains(const ctrl_gains_t *g) {
  bool ok = in_range(g->hover_pct, 0.0f, 100.0f) &&
            in_range(g->pd_kp, 0.0f, GAIN_MAX) && in_range(g->pd_kd, 0.0f, GAIN_MAX) &&
            in_range(g->pos_kp, 0.0f, GAIN_MAX) && in_range(g->pos_ki, 0.0f, GAIN_MAX) &&
            in_range(g->pos_kd, 0.0f, GAIN_MAX) && in_range(g->vz_max, 0.01f, VZ_MAX_LIMIT) &&
            in_range(g->vel_kp, 0.0f, GAIN_MAX) && in_range(g->vel_ki, 0.0f, GAIN_MAX) &&
            in_range(g->vel_kd, 0.0f, GAIN_MAX) && in_range(g->vel_i_max, 0.0f, 100.0f);
  if (ok) s_gains = *g;
  return ok;
}

uint8_t controller_mode(uint8_t requested) {
  return requested < CTRL_COUNT ? requested : CTRL_MANUAL;
}

uint8_t controller_run(uint8_t mode, const ctrl_in_t *in) {
  float u = CONTROLLERS[mode].update(in);
  if (!isfinite(u) || u <= 0.0f) return 0;
  if (u >= 100.0f) return 100;
  return (uint8_t)lroundf(u);
}
