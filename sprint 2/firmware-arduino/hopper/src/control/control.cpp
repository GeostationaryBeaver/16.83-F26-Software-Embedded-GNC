// Control loop: reads the sensors, takes commands from the ground, runs the safety state
// machine, the altitude Kalman filter, the flight phases (guidance: where to be) and the
// controller (how to get there), and sends telemetry.
#include <Arduino.h>
#include <atomic>
#include <math.h>
#include "control.h"
#include "altitude_kf.h"
#include "controllers.h"
#include "flight.h"
#include "gains_store.h"
#include "power_guard.h"
#include "safety.h"
#include "sensors.h"
#include "../calibration/autocal.h"
#include "../calibration/calibration.h"
#include "../drivers/i2c_bus.h"
#include "../drivers/radio.h"
#include "../drivers/servo.h"
#include "../telemetry/telemetry.h"

#define CONTROL_CORE 1
#define CONTROL_PRIORITY 20
#define CONTROL_PERIOD_MS 4      // 250 Hz
#define TELEMETRY_EVERY 5     // send telemetry every Nth control loop: 250 Hz / 5 = 50 Hz
#define LINK_TIMEOUT_MS 150   // no command for this long = FAILSAFE
#define GAP_WINDOW_MS 2000    // max_gap_ms resets this often
#define CTRL_DT_MAX 0.02f      // s: controller dt limit after a stalled loop (5 periods)
#define GAINS_REPORT_MS 1000  // MSG_GAINS this often, besides the ack of each request
#define NAV_LASER_STALE_MS 200  // no in-range laser range for this long = altitude filter not trusted

// Set these for how the board is mounted: which IMU axis points up (0 = x,
// 1 = y, 2 = z) and its sign (+1 if the axis reads about +9.81 at rest, -1 if
// about -9.81).
#define IMU_UP_AXIS 2
#define IMU_UP_SIGN 1.0f
#define GRAVITY 9.80665f

static altitude_kf_t s_kf;

// Commands: written by the radio callback on core 0, read by the control loop
static QueueHandle_t s_cmd_mailbox;  // length 1, the newest command wins
static std::atomic<uint32_t> s_last_cmd_ms{0};
static std::atomic<uint32_t> s_max_gap_ms{0};

static QueueHandle_t s_gains_mailbox;  // MSG_SET_GAINS, length 1, the newest wins

static cmd_t s_cmd = {};  // latest command taken by the control loop
// Last gains request handled, and how. Kept so repeats of it (the ground resends until it
// sees the ack) aren't applied again.
static uint8_t s_gains_ack = 0;
static uint8_t s_gains_result = GAINS_OK;
static uint8_t s_cal_ack = 0;
static uint8_t s_autocal_seq = 0;  // cal_seq of a CAL_AUTO in progress, acked when it finishes
static uint8_t s_tx_power_cmd = 0;

// Controller in use. Follows cmd_t.ctrl_mode while not ARMED, held while ARMED.
static uint8_t s_ctrl_mode = CTRL_MANUAL;
static ctrl_in_t s_ctrl_in = {};  // filled every loop by control_task

// Flight phases. Inputs filled every loop by control_task; ref is the latest output.
static flight_inputs_t s_flight_in = {};
static flight_ref_t s_ref = {FLIGHT_MANUAL, NAN, NAN};

void control_on_command(const cmd_t *cmd) {
  uint32_t now = millis();
  uint32_t last = s_last_cmd_ms.load();
  uint32_t gap = last ? now - last : 0;
  if (gap > 65535) gap = 65535;
  if (gap > s_max_gap_ms.load()) s_max_gap_ms.store(gap);
  s_last_cmd_ms.store(now ? now : 1);  // 0 means none yet
  xQueueOverwrite(s_cmd_mailbox, cmd);
}

void control_on_gains(const gains_packet_t *pkt) { xQueueOverwrite(s_gains_mailbox, pkt); }

static void report_gains() {
  gains_packet_t r = {};
  r.type = MSG_GAINS;
  r.seq = s_gains_ack;
  r.result = s_gains_result;
  r.gains = *controller_gains();
  telemetry_send_gains(&r);
}

// Takes a gains request from the ground. Only while not ARMED: the gains never change during
// a hop, and the flash write (a few ms of stalled loop) never happens while flying.
// Returns true if a request was handled (applied or refused), so it gets acked now.
static bool take_gains() {
  gains_packet_t req;
  if (xQueueReceive(s_gains_mailbox, &req, 0) != pdTRUE) return false;
  if (req.seq == 0 || req.seq == s_gains_ack) return false;  // a repeat: the ack is on its way
  if (safety_state() == STATE_ARMED) {
    s_gains_result = GAINS_REFUSED_ARMED;
  } else if (req.op == GAINS_DEFAULTS) {
    controller_set_gains(&CTRL_DEFAULT_GAINS);
    gains_store_clear();
    s_gains_result = GAINS_OK;
  } else if (req.op == GAINS_SET && controller_set_gains(&req.gains)) {
    gains_store_save();
    s_gains_result = GAINS_OK;
  } else {
    s_gains_result = GAINS_REFUSED_INVALID;
  }
  s_gains_ack = req.seq;
  return true;
}

// Called every loop while ARMED. Returns the valve opening, 0-100 %: the ground's slider in
// IDLE, otherwise the controller picked by the ground (controllers.h) tracking the flight
// phase's setpoint, or shut. FAILSAFE and the E-stop override it.
static uint8_t valve_demand(uint32_t now_ms) {
  uint8_t phase_was = flight_phase();
  s_ref = flight_update(now_ms, &s_flight_in);
  switch (s_ref.action) {
    case FLIGHT_MANUAL:
      return s_cmd.valve_pct;
    case FLIGHT_TRACK:
      if (phase_was == PHASE_IDLE) CONTROLLERS[s_ctrl_mode].reset();  // go: clean start
      s_ctrl_in.z_sp = s_ref.z_sp;
      s_ctrl_in.vz_sp = s_ref.vz_sp;
      return controller_run(s_ctrl_mode, &s_ctrl_in);
    default:
      return 0;
  }
}

static uint8_t health_bits(const sensors_t *sn) {
  return (sn->laser_ok ? HEALTH_LASER : 0) | (sn->imu_ok ? HEALTH_IMU : 0) |
         (sn->temp_ok ? HEALTH_TEMP : 0) | (sn->pt_ok ? HEALTH_PT : 0) |
         (sn->servo_power ? HEALTH_SERVO_PWR : 0);
}

// A hop in the air: the power protection lands it rather than shutting the valve
static bool flying() {
  uint8_t p = flight_phase();
  return p == PHASE_ASCENT || p == PHASE_HOVER || p == PHASE_DESCENT || p == PHASE_ABORT;
}

static uint8_t power_flags() {
  return (power_guard_tripped() ? FLAG_POWER_TRIP : 0) | (servo_signal_on() ? 0 : FLAG_SERVO_OFF) |
         (power_guard_latched() ? FLAG_POWER_LATCH : 0) |
         ((power_guard_cause() << FLAG_POWER_CAUSE_SHIFT) & FLAG_POWER_CAUSE_MASK);
}

// Takes the ground's controller choice while not ARMED, so the law never changes mid-hop
static void latch_controller() {
  if (safety_state() == STATE_ARMED) return;
  uint8_t mode = controller_mode(s_cmd.ctrl_mode);
  if (mode != s_ctrl_mode) {
    s_ctrl_mode = mode;
  }
}

static void take_command(uint32_t now_ms, const sensors_t *sn) {
  if (xQueueReceive(s_cmd_mailbox, &s_cmd, 0) != pdTRUE) return;
  if (s_cmd.tx_power != s_tx_power_cmd) {
    s_tx_power_cmd = s_cmd.tx_power;
    radio_set_tx_power(s_cmd.tx_power);
  }
  if (s_cmd.cal_op != CAL_NONE && s_cmd.cal_seq != s_cal_ack && s_cmd.cal_seq != s_autocal_seq) {
    if (s_cmd.cal_op == CAL_AUTO) {
      if (safety_state() == STATE_ARMED) {
        s_cal_ack = s_cmd.cal_seq;  // refused: only on the pad
      } else {
        autocal_start(now_ms);
        s_autocal_seq = s_cmd.cal_seq;  // acked once it has finished (see autocal_step)
      }
      return;
    }
    if (s_cmd.cal_op == CAL_CLEAR) autocal_clear_imu();
    // laser_avg_mm keeps its last value after the laser drops out: don't calibrate on that
    float laser_mm = sn->laser_ok && sn->laser_in_range ? sn->laser_avg_mm : -1.0f;
    calibration_apply(s_cmd.cal_op, s_cmd.cal_value, sn->pt_ma_avg, laser_mm);
    s_cal_ack = s_cmd.cal_seq;
  }
}

// Runs the automatic laser/IMU calibration (autocal.h). When it has new biases the altitude
// filter starts again from them, on the pad.
static void autocal_step(uint32_t now_ms, const sensors_t *sn) {
  if (autocal_update(now_ms, sn, safety_state() == STATE_ARMED)) altitude_kf_init(&s_kf, 0.0f);
  if (s_autocal_seq && !autocal_running()) {  // finished, or stopped by arming
    s_cal_ack = s_autocal_seq;
    s_autocal_seq = 0;
  }
}

static void send_telemetry(const sensors_t *sn, const imu_sample_t *imu, float laser_z) {
  telemetry_data_t d = {};
  d.z = s_kf.z;
  d.vz = s_kf.vz;
  d.accel_bias = s_kf.b;
  d.laser_z = laser_z;

  for (int i = 0; i < 3; i++) {
    d.accel[i] = imu->accel[i];
    d.gyro[i] = imu->gyro[i];
  }
  d.temp_c = sn->temp_c;
  d.pt_ma = sn->pt_ma_avg;
  d.psi = calibration_psi(sn->pt_ma_avg);
  d.servo_v = sn->servo_v;

  d.state = safety_state();
  d.reason = safety_reason();
  d.valve_pct = safety_valve_pct();
  // As avionics: a tripped power protection reads as no servo power
  d.health = health_bits(sn) & (power_guard_tripped() ? ~HEALTH_SERVO_PWR : 0xFF);
  d.failsafes = safety_failsafes();

  d.cmd_seq = s_cmd.seq;
  d.cmd_time_us = s_cmd.time_us;
  d.max_gap_ms = s_max_gap_ms.load();
  d.rssi = radio_rssi();
  d.tx_power = radio_tx_power();

  d.laser_offset_m = calibration_laser_offset_mm() / 1000.0f;
  d.pt_zero_ma = calibration_pt_zero_ma();
  d.pt_span = calibration_pt_span();
  d.cal_ack = s_cal_ack;

  d.flags = (i2c_bus_dual() ? FLAG_DUAL_I2C : 0) | power_flags() |
            (autocal_running() ? FLAG_AUTOCAL : 0);
  d.i2c_errors = sn->i2c_errors;

  d.ctrl_mode = s_ctrl_mode;
  d.z_sp = s_ref.z_sp;
  d.vz_sp = s_ref.vz_sp;
  d.phase = flight_phase();
  d.abort_reason = flight_abort_reason();
  // In IDLE the profile go would take, during a hop the one it took
  const flight_profile_t *p = d.phase == PHASE_IDLE ? &s_flight_in.request : flight_profile();
  d.hop_mm = (uint16_t)lroundf(p->hop_m * 1000.0f);
  d.v_up_mms = (uint16_t)lroundf(p->v_up * 1000.0f);
  d.v_down_mms = (uint16_t)lroundf(p->v_down * 1000.0f);
  d.hover_ms = (uint16_t)p->hover_ms;
  telemetry_push(&d);
}

static void control_task(void *arg) {
  TickType_t last_wake = xTaskGetTickCount();
  uint32_t last_imu_us = micros();
  uint32_t last_loop_us = micros() - CONTROL_PERIOD_MS * 1000;  // first dt = nominal
  uint32_t gap_window_ms = millis();
  bool link_up = false;
  uint32_t loop_count = 0;
  uint32_t last_gains_report_ms = 0;
  while (true) {
    // Real time since the last loop, for the controllers. Measured at the top, right after
    // the wake-up, so the time spent reading sensors doesn't add jitter.
    uint32_t loop_us = micros();
    float loop_dt = (loop_us - last_loop_us) / 1e6f;
    last_loop_us = loop_us;
    uint32_t now = millis();
    // No sensor restarts while ARMED: a laser restart blocks the loop for tens of ms
    const sensors_t *sn = sensors_update(now, safety_state() != STATE_ARMED);
    // IMU with the automatic calibration's biases removed: used by everything below
    imu_sample_t imu = sn->imu;
    autocal_correct_imu(&imu);

    // ---- commands and link ----
    take_command(now, sn);
    autocal_step(now, sn);
    if (take_gains() || now - last_gains_report_ms >= GAINS_REPORT_MS) {
      last_gains_report_ms = now;
      report_gains();
    }
    uint32_t last_cmd = s_last_cmd_ms.load();
    // millis() again, not now: a command can arrive during sensors_update(), after now was
    // read, and now - last_cmd would wrap to ~4e9 and count as link lost
    bool link = last_cmd && millis() - last_cmd < LINK_TIMEOUT_MS;
    if (link != link_up) {
      link_up = link;
      if (!link_up && radio_tx_power()) {
        radio_set_tx_power(0);
        s_tx_power_cmd = 0;
      }
    }
    if (now - gap_window_ms >= GAP_WINDOW_MS) {
      gap_window_ms = now;
      s_max_gap_ms.store(0);
    }

    // ---- power protection ----
    // Before the safety state machine, so a trip is acted on in the same loop. The signal may
    // only go off once the valve is commanded shut and nothing is in the air.
    power_guard_update(now, sn->servo_v, sn->pt_ma_avg, !flying() && safety_valve_pct() == 0,
                       !(link_up && s_cmd.arm));

    // ---- altitude filter ----
    if (sn->imu_new) {
      float dt = (sn->imu_us - last_imu_us) / 1e6f;
      last_imu_us = sn->imu_us;
      if (dt > 0.1f) dt = 0.1f;  // after an IMU dropout
      altitude_kf_predict(&s_kf, IMU_UP_SIGN * imu.accel[IMU_UP_AXIS], dt);
    }
    bool laser_reading = sn->laser_ok && sn->laser_in_range;
    float laser_z = laser_reading ? calibration_height_mm(sn->laser_raw_mm) / 1000.0f : NAN;
    if (sn->laser_new && laser_reading) altitude_kf_update(&s_kf, laser_z);

    // ---- controller inputs ----
    latch_controller();
    // Limited so a stall (laser restart, flash write: only while disarmed) can't put one
    // big step into an integrator
    s_ctrl_in.dt = fminf(loop_dt, CTRL_DT_MAX);
    s_ctrl_in.z = s_kf.z;
    s_ctrl_in.vz = s_kf.vz;
    s_ctrl_in.az = IMU_UP_SIGN * imu.accel[IMU_UP_AXIS] - GRAVITY - s_kf.b;
    s_ctrl_in.manual_pct = s_cmd.valve_pct;

    // ---- flight phase inputs ----
    s_flight_in.z = s_kf.z;
    s_flight_in.vz = s_kf.vz;
    s_flight_in.go = s_cmd.go != 0;
    s_flight_in.closed_loop = s_ctrl_mode != CTRL_MANUAL;
    // The filter is only trustworthy with the IMU and a fresh laser range; without them it
    // is dead reckoning and drifts
    s_flight_in.nav_ok = sn->imu_ok && sn->laser_ok && sn->laser_fix_ms &&
                         now - sn->laser_fix_ms < NAV_LASER_STALE_MS;
    s_flight_in.health = health_bits(sn);  // the rail itself: losing it cuts the valve
    s_flight_in.power_trip = power_guard_tripped();  // a trip lands instead
    s_flight_in.request = flight_clamp_profile(s_cmd.z_sp_mm, s_cmd.v_up_mms, s_cmd.v_down_mms,
                                               s_cmd.hover_ms, &s_flight_in.request_clamped);

    // ---- safety and valve ----
    safety_inputs_t in = {
      .heard = last_cmd != 0,
      .link_up = link_up,
      .arm = s_cmd.arm != 0,
      .estop = s_cmd.estop != 0,
      .power_trip = power_guard_tripped(),
      .flying = flying(),
    };
    safety_update(now, &in, valve_demand);
    if (safety_state() != STATE_ARMED) {  // disarmed or FAILSAFE: any hop is over
      flight_reset();
      s_ref = {FLIGHT_MANUAL, NAN, NAN};
    }

    if (loop_count++ % TELEMETRY_EVERY == 0) send_telemetry(sn, &imu, laser_z);
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
  }
}

void control_start() {
  s_cmd_mailbox = xQueueCreate(1, sizeof(cmd_t));
  s_gains_mailbox = xQueueCreate(1, sizeof(gains_packet_t));
  calibration_load();
  gains_store_load();
  sensors_init();
  altitude_kf_init(&s_kf, 0.0f);  // starts on the ground
  autocal_init(IMU_UP_AXIS, IMU_UP_SIGN, GRAVITY);
  autocal_start(millis());  // on the pad at power-up: laser offset and IMU biases from it

  xTaskCreatePinnedToCore(control_task, "control", 8192, NULL,
                          CONTROL_PRIORITY, NULL, CONTROL_CORE);
}
