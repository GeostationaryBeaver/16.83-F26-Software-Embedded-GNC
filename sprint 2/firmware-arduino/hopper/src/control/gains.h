// Controller tuning. Everything you change between test runs lives in this file.
//
// To fly a different set: change ACTIVE_GAINS to its name, re-upload. That is the
// only line to edit. The set that flew is echoed in telemetry (kp, kd), so a
// recording always says which gains produced it.
//
// To add a set: copy a block below, give it a new name. Keep the flight set as it
// is so there is always a known-good one to go back to.
#pragma once

typedef struct {
  const char *name;

  // PD on altitude. Error is in metres, output in percent of valve opening.
  float kp;            // %/m
  float kd;            // %/(m/s)

  // Feedforward: the opening that roughly balances the vehicle's weight. The PD
  // only has to trim around it, which keeps the trim term small and the loop
  // linear. Get it from a manual hover: whatever valve_pct held altitude.
  //
  // This is the most important number here, more than kp or kd. A PD has no
  // integral term, so it cannot remove a constant error: the vehicle settles
  // where the PD's output happens to equal the opening that holds it, i.e.
  //
  //     steady-state offset  =  (true hover opening - valve_hover) / kp
  //
  // At kp = 18 that is 5.6 cm of altitude error for every 1 % this is wrong by,
  // and the vehicle sits low if it is set too low. If a hover settles short of
  // its setpoint by a constant amount, raise this (by kp * offset percent)
  // rather than reaching for kp. Mass, supply pressure and feed temperature all
  // move the true value, so re-check it whenever the vehicle changes.
  float valve_hover;   // %

  // Authority limits. valve_min above 0 keeps the plenum from collapsing and the
  // flow from restarting cold on every correction; it also means the vehicle
  // always has some thrust, so a descent is braked rather than a free fall.
  // Keep valve_min well under the hover opening: at or above it the floor alone
  // holds the vehicle up and no descent is possible at all.
  float valve_min;     // %
  float valve_max;     // %

  // How fast the altitude setpoint may chase a new hover command (m/s). A step
  // from the ground becomes a ramp, so a "hover at 2 ft" from the pad does not
  // ask for full valve on the first tick.
  float sp_rate;       // m/s

  // MODE_LAND
  float land_rate;     // descent rate (m/s, positive)
  float land_cutoff;   // below this altitude the valve latches shut (m)
} gain_set_t;

// ---- Gain sets ------------------------------------------------------------

// Known-good flight set.
static const gain_set_t GAINS_FLIGHT = {
  .name = "flight",
  .kp = 18.0f,
  .kd = 22.0f,
  .valve_hover = 50.0f,
  .valve_min = 10.0f,
  .valve_max = 90.0f,
  .sp_rate = 0.15f,
  .land_rate = 0.10f,
  .land_cutoff = 0.05f,
};

// First power-on with the vehicle on its tether: half the authority and a slow
// setpoint, so a sign error or a bad mounting axis shows up as a drift you can
// watch rather than a slam into the stops.
static const gain_set_t GAINS_BENCH = {
  .name = "bench",
  .kp = 9.0f,
  .kd = 11.0f,
  .valve_hover = 40.0f,
  .valve_min = 0.0f,
  .valve_max = 60.0f,
  .sp_rate = 0.05f,
  .land_rate = 0.05f,
  .land_cutoff = 0.08f,
};

// ---- Pick one -------------------------------------------------------------
#define ACTIVE_GAINS GAINS_FLIGHT

// ---- Estimator trust ------------------------------------------------------
// The loop closes on the Kalman filter's z, which only stays true while the
// laser keeps correcting it. With no laser the filter coasts on the IMU and z
// drifts without bound, so an altitude hold on it would quietly fly away.
// If no laser update lands for this long, the loop is inhibited and the valve
// shut (CTRL_NO_EST). 300 ms is several laser cycles: long enough to ride out a
// dropped reading, short enough that the drift stays centimetres.
#define EST_LASER_TIMEOUT_MS 300
