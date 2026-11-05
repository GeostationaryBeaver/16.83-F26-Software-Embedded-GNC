// Control loop: reads the sensors, takes commands from the ground, runs the safety state
// machine and the altitude Kalman filter, and sends telemetry.
#include <Arduino.h>
#include <atomic>
#include <math.h>
#include "control.h"
#include "altitude_kf.h"
#include "safety.h"
#include "sensors.h"
#include "../calibration/calibration.h"
#include "../drivers/i2c_bus.h"
#include "../drivers/radio.h"
#include "../telemetry/telemetry.h"

#define CONTROL_CORE 1
#define CONTROL_PRIORITY 20
#define CONTROL_PERIOD_MS 10
#define LINK_TIMEOUT_MS 150   // no command for this long = FAILSAFE
#define GAP_WINDOW_MS 2000    // max_gap_ms resets this often

// Set these for how the board is mounted: which IMU axis points up (0 = x,
// 1 = y, 2 = z) and its sign (+1 if the axis reads about +9.81 at rest, -1 if
// about -9.81).
#define IMU_UP_AXIS 2
#define IMU_UP_SIGN 1.0f

static altitude_kf_t s_kf;

// Commands: written by the radio callback on core 0, read by the control loop
static QueueHandle_t s_cmd_mailbox;  // length 1, the newest command wins
static std::atomic<uint32_t> s_last_cmd_ms{0};
static std::atomic<uint32_t> s_max_gap_ms{0};

static cmd_t s_cmd = {};  // latest command taken by the control loop
static uint8_t s_cal_ack = 0;
static uint8_t s_tx_power_cmd = 0;

void control_on_command(const cmd_t *cmd) {
  uint32_t now = millis();
  uint32_t last = s_last_cmd_ms.load();
  uint32_t gap = last ? now - last : 0;
  if (gap > 65535) gap = 65535;
  if (gap > s_max_gap_ms.load()) s_max_gap_ms.store(gap);
  s_last_cmd_ms.store(now ? now : 1);  // 0 means none yet
  xQueueOverwrite(s_cmd_mailbox, cmd);
}

// ======================= GNC CONTROLLER GOES HERE =======================
// Called every loop while ARMED. Return the valve opening, 0-100 %.
// Today it is manual: the ground's valve_pct command. GNC: replace the body with the
// controller, using s_kf (z, vz) and the sensors. Keep it fast: no delay(), no blocking
// reads. FAILSAFE and the E-stop override it.
static uint8_t valve_demand(uint32_t now_ms) {
  (void)now_ms;
  return s_cmd.valve_pct;
}

static void take_command(const sensors_t *sn) {
  if (xQueueReceive(s_cmd_mailbox, &s_cmd, 0) != pdTRUE) return;
  if (s_cmd.tx_power != s_tx_power_cmd) {
    s_tx_power_cmd = s_cmd.tx_power;
    radio_set_tx_power(s_cmd.tx_power);
    if (s_cmd.tx_power) Serial.printf("radio power %u dBm\n", s_cmd.tx_power);
    else Serial.println("radio power full");
  }
  if (s_cmd.cal_op != CAL_NONE && s_cmd.cal_seq != s_cal_ack) {
    calibration_apply(s_cmd.cal_op, s_cmd.cal_value, sn->pt_ma_avg, sn->laser_avg_mm);
    s_cal_ack = s_cmd.cal_seq;
  }
}

static void send_telemetry(const sensors_t *sn, float laser_z) {
  telemetry_data_t d = {};
  d.z = s_kf.z;
  d.vz = s_kf.vz;
  d.accel_bias = s_kf.b;
  d.laser_z = laser_z;

  for (int i = 0; i < 3; i++) {
    d.accel[i] = sn->imu.accel[i];
    d.gyro[i] = sn->imu.gyro[i];
  }
  d.temp_c = sn->temp_c;
  d.pt_ma = sn->pt_ma_avg;
  d.psi = calibration_psi(sn->pt_ma_avg);
  d.servo_v = sn->servo_v;

  d.state = safety_state();
  d.reason = safety_reason();
  d.valve_pct = safety_valve_pct();
  d.health = (sn->laser_ok ? HEALTH_LASER : 0) | (sn->imu_ok ? HEALTH_IMU : 0) |
             (sn->temp_ok ? HEALTH_TEMP : 0) | (sn->pt_ok ? HEALTH_PT : 0) |
             (sn->servo_power ? HEALTH_SERVO_PWR : 0);
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

  d.flags = i2c_bus_dual() ? FLAG_DUAL_I2C : 0;
  d.i2c_errors = sn->i2c_errors;
  telemetry_push(&d);
}

static void control_task(void *arg) {
  TickType_t last_wake = xTaskGetTickCount();
  uint32_t last_imu_us = micros();
  uint32_t gap_window_ms = millis();
  bool link_up = false;
  while (true) {
    uint32_t now = millis();
    const sensors_t *sn = sensors_update(now);

    // ---- commands and link ----
    take_command(sn);
    uint32_t last_cmd = s_last_cmd_ms.load();
    // millis() again, not now: a command can arrive during sensors_update(), after now was
    // read, and now - last_cmd would wrap to ~4e9 and count as link lost
    bool link = last_cmd && millis() - last_cmd < LINK_TIMEOUT_MS;
    if (link != link_up) {
      link_up = link;
      Serial.println(link_up ? ">>> LINK UP" : ">>> LINK LOST");
      if (!link_up && radio_tx_power()) {
        radio_set_tx_power(0);
        s_tx_power_cmd = 0;
        Serial.println("radio back to full power");
      }
    }
    if (now - gap_window_ms >= GAP_WINDOW_MS) {
      gap_window_ms = now;
      s_max_gap_ms.store(0);
    }

    // ---- altitude filter ----
    if (sn->imu_new) {
      float dt = (sn->imu_us - last_imu_us) / 1e6f;
      last_imu_us = sn->imu_us;
      if (dt > 0.1f) dt = 0.1f;  // after an IMU dropout
      altitude_kf_predict(&s_kf, IMU_UP_SIGN * sn->imu.accel[IMU_UP_AXIS], dt);
    }
    bool laser_reading = sn->laser_ok && sn->laser_in_range;
    float laser_z = laser_reading ? calibration_height_mm(sn->laser_raw_mm) / 1000.0f : NAN;
    if (sn->laser_new && laser_reading) altitude_kf_update(&s_kf, laser_z);

    // ---- safety and valve ----
    safety_inputs_t in = {
      .heard = last_cmd != 0,
      .link_up = link_up,
      .arm = s_cmd.arm != 0,
      .estop = s_cmd.estop != 0,
    };
    safety_update(now, &in, valve_demand);

    send_telemetry(sn, laser_z);
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
  }
}

void control_start() {
  s_cmd_mailbox = xQueueCreate(1, sizeof(cmd_t));
  calibration_load();
  Serial.printf("calibration: pressure zero %.3f mA, span %.4f, laser offset %+d mm\n",
                calibration_pt_zero_ma(), calibration_pt_span(), calibration_laser_offset_mm());
  sensors_init();
  altitude_kf_init(&s_kf, 0.0f);  // starts on the ground

  xTaskCreatePinnedToCore(control_task, "control", 8192, NULL,
                          CONTROL_PRIORITY, NULL, CONTROL_CORE);
}
