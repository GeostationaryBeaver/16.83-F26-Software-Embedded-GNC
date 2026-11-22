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
// Setpoint acceleration limits, so vz_sp is a velocity the hopper can actually follow (no steps).
// Asymmetric: speeding up upward (or braking a descent) needs thrust above hover, which the
// valve has little of; slowing a climb (or starting a descent) only needs less thrust.
#define ACCEL_UP_MPS2      0.50f   // vz_sp may rise this fast: ~5% thrust over hover
#define ACCEL_DOWN_MPS2    1.00f   // vz_sp may fall this fast: ~10% thrust under hover
#define TOUCH_SP_MPS       0.10f   // the descent's speed from FLARE_Z_M down. Even after a slower
                                   // descent: well above TOUCH_VZ_MPS, so a hopper still sinking
                                   // can't pass for one resting on the ground
#define FLARE_Z_M          0.10f   // to the ground: long enough for the controller to settle on it
#define TIMEOUT_MARGIN_MS  5000    // flight timeout = 2 x the profile's own time + this
#define ABORT_LAND_MS      5000    // an ABORT descent that hasn't landed by then: valve shut

static uint8_t s_phase = PHASE_IDLE;
static uint8_t s_abort = ABORT_NONE;
static bool s_go_was = true;      // so go already held at ARM doesn't start a flight
static flight_profile_t s_prof = {0.5f, 0.25f, 0.20f, 3000};
static float s_z_sp = NAN;        // setpoint last loop
static float s_vz_sp = 0.0f;      // its rate last loop
static uint32_t s_last_ms = 0;    // when the setpoint was last moved
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

// Moves the setpoint (s_z_sp, s_vz_sp) one step of dt toward z_end, at most v_max fast, so it
// arrives at speed v_end (a descent's flare speed: 0 otherwise). vz_sp changes at
// most ACCEL_UP_MPS2 upward and ACCEL_DOWN_MPS2 downward per second, and brakes in time to stop
// at z_end: a climb brakes by easing off (ACCEL_DOWN), a descent by thrusting (ACCEL_UP). Starts
// from whatever the setpoint is doing, so an ABORT mid-climb first stops the climb smoothly.
// Returns true once at z_end and stopped.
static bool move_setpoint(float z_end, float v_max, float v_end, float dt) {
  float d = z_end - s_z_sp;
  float dir = d >= 0.0f ? 1.0f : -1.0f;
  float brake = dir > 0.0f ? ACCEL_DOWN_MPS2 : ACCEL_UP_MPS2;
  // Fastest speed toward z_end that can still slow to v_end by it: v^2 = v_end^2 + 2 a d, in its
  // discrete form (losing brake * dt per loop) so it arrives without a last-loop velocity step
  float h = 0.5f * brake * dt;
  float v_ok = fminf(v_max, sqrtf(h * h + v_end * v_end + 2.0f * brake * fabsf(d)) - h);
  float v_was = s_vz_sp;
  s_vz_sp = v_was + clampf(dir * v_ok - v_was, -ACCEL_DOWN_MPS2 * dt, ACCEL_UP_MPS2 * dt);
  float step = s_vz_sp * dt;
  // Arrived (or would pass it this step while heading there): sit on it. Loop timing jitter can
  // leave a little speed: that comes off at the accel limits too, over a loop or two
  if (fabsf(d) <= fabsf(step) && step * dir >= 0.0f) {
    s_z_sp = z_end;
    s_vz_sp = v_was + clampf(dir * fminf(fabsf(v_was), v_end) - v_was, -ACCEL_DOWN_MPS2 * dt,
                             ACCEL_UP_MPS2 * dt);
    return s_vz_sp == 0.0f;
  }
  s_z_sp += step;
  return false;
}

static void go_phase(uint8_t phase, uint32_t now_ms) {
  s_phase = phase;
  s_phase_ms = now_ms;
  s_landed_ms = 0;
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
  // TEMPORARY (bench testing): pressure sensor not required to fly. Restore HEALTH_PT before
  // flying with real pressure.
  const uint8_t need = HEALTH_IMU | HEALTH_LASER | /* HEALTH_PT | */ HEALTH_SERVO_PWR;
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
  // Ramp time at the set rates, plus the time spent speeding up and slowing down
  float t_s = s_prof.hop_m / s_prof.v_up + s_prof.hover_ms / 1000.0f + s_prof.hop_m / s_prof.v_down +
              2.0f * (s_prof.v_up + s_prof.v_down) / ACCEL_UP_MPS2 + (FLARE_Z_M - LAND_SP_M) / TOUCH_SP_MPS;
  s_timeout_ms = (uint32_t)(2.0f * t_s * 1000.0f) + TIMEOUT_MARGIN_MS;
  s_flight_ms = now_ms;
  s_abort = ABORT_NONE;
  s_z_sp = in->z;  // the climb starts from where the hopper is, at rest: no setpoint step
  s_vz_sp = 0.0f;
  s_last_ms = now_ms;
  go_phase(PHASE_ASCENT, now_ms);
}

void flight_reset() {
  s_phase = PHASE_IDLE;
  s_z_sp = NAN;
  s_vz_sp = 0.0f;
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

  float dt = (now_ms - s_last_ms) / 1000.0f;  // since the setpoint last moved
  s_last_ms = now_ms;
  if (!(dt > 0.0f)) dt = 0.0f;
  if (dt > 0.05f) dt = 0.05f;
  switch (s_phase) {
    case PHASE_ASCENT:
      if (move_setpoint(s_prof.hop_m, s_prof.v_up, 0.0f, dt) &&
          fabsf(in->z - s_prof.hop_m) < HOP_TOL_M) {
        go_phase(PHASE_HOVER, now_ms);
      }
      break;

    case PHASE_HOVER:
      s_z_sp = s_prof.hop_m;
      s_vz_sp = 0.0f;
      if (now_ms - s_phase_ms >= s_prof.hover_ms) go_phase(PHASE_DESCENT, now_ms);
      break;

    case PHASE_DESCENT:
    case PHASE_ABORT: {
      float rate = s_phase == PHASE_ABORT ? ABORT_SINK_MPS : s_prof.v_down;
      // Down to FLARE_Z_M, braking to TOUCH_SP_MPS by it, then on at that to LAND_SP_M
      if (s_z_sp > FLARE_Z_M) move_setpoint(FLARE_Z_M, rate, fminf(TOUCH_SP_MPS, rate), dt);
      else move_setpoint(LAND_SP_M, TOUCH_SP_MPS, 0.0f, dt);
      if (touched_down(now_ms, in, s_z_sp)) {
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
    s_vz_sp = 0.0f;
    return {FLIGHT_CUT, NAN, NAN};
  }
  return {FLIGHT_TRACK, s_z_sp, s_vz_sp};
}

uint8_t flight_phase() { return s_phase; }
uint8_t flight_abort_reason() { return s_abort; }
const flight_profile_t *flight_profile() { return &s_prof; }
