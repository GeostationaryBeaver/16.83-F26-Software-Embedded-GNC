#include <Arduino.h>
#include <math.h>
#include "flight.h"
#include "../protocol.h"

// Limits on the profile the ground may send. A request outside them is clamped.
#define HOP_MIN_M          0.10f   // above laser noise and the exhaust's ground effect
#define HOP_MAX_M          0.80f   // + CEILING_MARGIN_M stays inside the laser's ~1.2 m
#define V_UP_MIN_MPS       0.05f
#define V_UP_MAX_MPS       0.50f
#define V_DOWN_MIN_MPS     0.05f
#define V_DOWN_MAX_MPS     0.30f   // touchdown speed: the setpoint is still moving at it
#define HOVER_MAX_MS       10000

// Fixed behaviour. Placeholders: tune with GNC and the sims before a hot test.
#define ABORT_SINK_MPS     0.40f   // setpoint ramp down in ABORT
#define HOP_TOL_M          0.05f   // within this of the hop height = there
#define LAND_SP_M         -0.10f   // descent setpoint stops this far below the ground
#define LAND_Z_M           0.05f   // below this = on the ground (pre-flight, and the setpoint
                                   // must be below it too before touchdown counts)
#define TOUCH_Z_M          0.03f   // touchdown: below this
#define TOUCH_VZ_MPS       0.05f   // and stopped (slower than this)
#define LAND_CONFIRM_MS    200     // for this long
#define SETTLED_VZ_MPS     0.10f   // pre-flight: filter must read slower than this
#define CEILING_MARGIN_M   0.30f   // above the hop height + this = ABORT
#define TIMEOUT_MARGIN_MS  5000    // flight timeout = 2 x the profile's own time + this
#define ABORT_LAND_MS      5000    // an ABORT descent that hasn't landed by then: valve shut

static uint8_t s_phase = PHASE_IDLE;
static uint8_t s_abort = ABORT_NONE;
static bool s_go_was = true;      // so go already held at ARM doesn't start a flight
static flight_profile_t s_prof = {0.5f, 0.25f, 0.20f, 3000};
static float s_z_sp = NAN;        // setpoint last loop
static float s_start_z = 0.0f;    // setpoint when the current phase started
static uint32_t s_phase_ms = 0;   // when the current phase started
static uint32_t s_flight_ms = 0;  // when go started the flight
static uint32_t s_timeout_ms = 0; // flight timeout for this profile
static uint32_t s_landed_ms = 0;  // when touchdown was first seen, 0 = not seen

static float clampf(float x, float lo, float hi) { return x < lo ? lo : x > hi ? hi : x; }

flight_profile_t flight_clamp_profile(int16_t hop_mm, uint16_t v_up_mms, uint16_t v_down_mms,
                                      uint16_t hover_ms, bool *clamped) {
  flight_profile_t p;
  p.hop_m = clampf(hop_mm / 1000.0f, HOP_MIN_M, HOP_MAX_M);
  p.v_up = clampf(v_up_mms / 1000.0f, V_UP_MIN_MPS, V_UP_MAX_MPS);
  p.v_down = clampf(v_down_mms / 1000.0f, V_DOWN_MIN_MPS, V_DOWN_MAX_MPS);
  p.hover_ms = hover_ms > HOVER_MAX_MS ? HOVER_MAX_MS : hover_ms;
  if (clamped) {
    *clamped = fabsf(p.hop_m * 1000.0f - hop_mm) > 0.5f || fabsf(p.v_up * 1000.0f - v_up_mms) > 0.5f ||
               fabsf(p.v_down * 1000.0f - v_down_mms) > 0.5f || p.hover_ms != hover_ms;
  }
  return p;
}

static void go_phase(uint8_t phase, uint32_t now_ms) {
  s_phase = phase;
  s_phase_ms = now_ms;
  s_landed_ms = 0;
  s_start_z = s_z_sp;  // DESCENT and ABORT ramp down from wherever the setpoint was
}

// Controlled descent at ABORT_SINK_MPS
static void go_abort(uint8_t why, uint32_t now_ms) {
  s_abort = why;
  go_phase(PHASE_ABORT, now_ms);
}

// Valve shut now: no controlled descent possible, or it didn't work
static void go_cut(uint8_t why, uint32_t now_ms) {
  s_abort = why;
  go_phase(PHASE_LANDED, now_ms);
}

// Touchdown: resting on the ground, with the setpoint low too (so not the first moments of a
// climb), for LAND_CONFIRM_MS. The setpoint below the ground keeps the controller pushing
// down until then. Shutting the valve any earlier, even a few cm up, drops the hopper:
// 5 cm of free fall is ~1 m/s at touchdown.
static bool touched_down(uint32_t now_ms, const flight_inputs_t *in, float z_sp) {
  if (in->z < TOUCH_Z_M && fabsf(in->vz) < TOUCH_VZ_MPS && z_sp < LAND_Z_M) {
    if (!s_landed_ms) s_landed_ms = now_ms ? now_ms : 1;
    return now_ms - s_landed_ms >= LAND_CONFIRM_MS;
  }
  s_landed_ms = 0;
  return false;
}

static uint8_t preflight_fail(const flight_inputs_t *in) {
  if (!in->closed_loop) return ABORT_MANUAL;
  if (in->power_trip) return ABORT_POWER;
  const uint8_t need = HEALTH_IMU | HEALTH_LASER | HEALTH_PT | HEALTH_SERVO_PWR;
  if ((in->health & need) != need || !in->nav_ok) return ABORT_PREFLIGHT;
  if (!(in->z < LAND_Z_M) || !(fabsf(in->vz) < SETTLED_VZ_MPS)) return ABORT_PREFLIGHT;
  return ABORT_NONE;
}

// Faults that leave nothing to fly with: valve shut. Checked in every flying phase.
static uint8_t cut_fault(const flight_inputs_t *in) {
  if (!in->nav_ok) return ABORT_SENSOR;
  if (!(in->health & HEALTH_SERVO_PWR)) return ABORT_SERVO;
  return ABORT_NONE;
}

// Faults a controlled descent can handle. Checked in ASCENT, HOVER and DESCENT.
static uint8_t abort_fault(uint32_t now_ms, const flight_inputs_t *in) {
  // Power protection: land while the servo still works. Already in DESCENT it just carries on
  // (that is already a landing, and slower than ABORT's).
  if (in->power_trip && s_phase != PHASE_DESCENT) return ABORT_POWER;
  if (!in->go) return ABORT_GROUND;
  if (now_ms - s_flight_ms >= s_timeout_ms) return ABORT_TIMEOUT;
  if (in->z > s_prof.hop_m + CEILING_MARGIN_M) return ABORT_CEILING;
  return ABORT_NONE;
}

static void start_flight(uint32_t now_ms, const flight_inputs_t *in) {
  s_prof = in->request;
  float t_s = s_prof.hop_m / s_prof.v_up + s_prof.hover_ms / 1000.0f + s_prof.hop_m / s_prof.v_down;
  s_timeout_ms = (uint32_t)(2.0f * t_s * 1000.0f) + TIMEOUT_MARGIN_MS;
  s_flight_ms = now_ms;
  s_abort = ABORT_NONE;
  s_z_sp = in->z;  // the climb starts from where the hopper is: no setpoint step
  go_phase(PHASE_ASCENT, now_ms);
}

void flight_reset() {
  s_phase = PHASE_IDLE;
  s_z_sp = NAN;
  s_go_was = true;
  s_landed_ms = 0;
}

flight_ref_t flight_update(uint32_t now_ms, const flight_inputs_t *in) {
  bool go_edge = in->go && !s_go_was;
  s_go_was = in->go;

  if (s_phase == PHASE_IDLE) {
    if (go_edge) {
      s_abort = preflight_fail(in);
      if (s_abort == ABORT_NONE) start_flight(now_ms, in);
    }
    if (s_phase == PHASE_IDLE) {
      s_z_sp = NAN;
      return {FLIGHT_MANUAL, NAN, NAN};
    }
  }

  if (s_phase != PHASE_LANDED) {
    uint8_t why = cut_fault(in);
    if (why != ABORT_NONE) go_cut(why, now_ms);
  }
  if (s_phase == PHASE_ASCENT || s_phase == PHASE_HOVER || s_phase == PHASE_DESCENT) {
    uint8_t why = abort_fault(now_ms, in);
    if (why != ABORT_NONE) go_abort(why, now_ms);
  }

  float t_s = (now_ms - s_phase_ms) / 1000.0f;  // time in the current phase
  float z_sp, vz_sp;
  switch (s_phase) {
    case PHASE_ASCENT:
      z_sp = s_start_z + s_prof.v_up * t_s;
      vz_sp = s_prof.v_up;
      if (z_sp >= s_prof.hop_m) {
        z_sp = s_prof.hop_m;
        vz_sp = 0.0f;
        if (fabsf(in->z - s_prof.hop_m) < HOP_TOL_M) {
          s_z_sp = z_sp;
          go_phase(PHASE_HOVER, now_ms);
        }
      }
      break;

    case PHASE_HOVER:
      z_sp = s_prof.hop_m;
      vz_sp = 0.0f;
      if (now_ms - s_phase_ms >= s_prof.hover_ms) {
        s_z_sp = z_sp;
        go_phase(PHASE_DESCENT, now_ms);
      }
      break;

    case PHASE_DESCENT:
    case PHASE_ABORT: {
      float rate = s_phase == PHASE_ABORT ? ABORT_SINK_MPS : s_prof.v_down;
      z_sp = s_start_z - rate * t_s;
      vz_sp = -rate;
      if (z_sp <= LAND_SP_M) {
        z_sp = LAND_SP_M;
        vz_sp = 0.0f;
      }
      if (touched_down(now_ms, in, z_sp)) {
        go_phase(PHASE_LANDED, now_ms);
      } else if (s_phase == PHASE_ABORT && now_ms - s_phase_ms >= ABORT_LAND_MS) {
        go_phase(PHASE_LANDED, now_ms);
      }
      break;
    }

    default:
      break;
  }

  if (s_phase == PHASE_LANDED) {
    s_z_sp = NAN;
    return {FLIGHT_CUT, NAN, NAN};
  }
  s_z_sp = z_sp;
  return {FLIGHT_TRACK, z_sp, vz_sp};
}

uint8_t flight_phase() { return s_phase; }
uint8_t flight_abort_reason() { return s_abort; }
const flight_profile_t *flight_profile() { return &s_prof; }
