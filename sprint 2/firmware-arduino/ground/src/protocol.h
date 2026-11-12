// Packet definitions shared by the hopper, the ground ESP, and the dashboard.
// Two copies, which MUST stay identical:
//   firmware-arduino/hopper/src/protocol.h
//   firmware-arduino/ground/src/protocol.h
// If you change a packet, update both and the matching decode/encode in
// sprint 2/ground_station/protocol.py (and index.html, which doesn't know the gains packets).
#pragma once

#include <stdint.h>

// Fill these in with each board's MAC address. To read a board's MAC:
//   python -m esptool --chip esp32s3 read-mac
#define HOPPER_MAC  {0xE8, 0xF6, 0x0A, 0xD5, 0x12, 0x9C}
#define GROUND_MAC  {0xE8, 0xF6, 0x0A, 0xD4, 0xD7, 0x24}

#define RADIO_CHANNEL 1

// First byte of every packet
typedef enum {
    MSG_TELEMETRY = 1,  // hopper -> ground, every 5th control loop (50 Hz)
    MSG_CMD = 2,        // ground -> hopper, 50 per second. None for 150 ms = FAILSAFE
    MSG_SET_GAINS = 3,  // ground -> hopper, gains_packet_t: new controller gains. Resent until acked
    MSG_GAINS = 4,      // hopper -> ground, gains_packet_t: gains in use. Once a second, and
                        // straight after each MSG_SET_GAINS (the ack)
} msg_type_t;

// Hopper state. A reboot always comes up SAFE.
typedef enum {
    STATE_SAFE = 0,      // valve shut
    STATE_ARMED = 1,     // valve follows the flight phase (control/flight.h)
    STATE_FAILSAFE = 2,  // valve shut, latched until the ground disarms and arms again
} flight_state_t;

typedef enum {
    REASON_NONE = 0,
    REASON_LINK = 1,   // no command for 150 ms
    REASON_ESTOP = 2,
    REASON_POWER = 3,  // power protection tripped while ARMED on the pad (control/power_guard.h)
} failsafe_reason_t;

// Flight phase, only moves while ARMED (control/flight.h). Back to IDLE when not ARMED.
typedef enum {
    PHASE_IDLE = 0,     // valve follows cmd_t.valve_pct (manual). go -> ASCENT
    PHASE_ASCENT = 1,   // setpoint ramps up to the hop height at v_up
    PHASE_HOVER = 2,    // holding the hop height for hover_ms
    PHASE_DESCENT = 3,  // setpoint ramps down at v_down
    PHASE_LANDED = 4,   // valve shut until the ground disarms
    PHASE_ABORT = 5,    // faster controlled descent, then LANDED (or valve shut if it can't)
} flight_phase_t;

typedef enum {
    ABORT_NONE = 0,
    ABORT_GROUND = 1,     // go dropped mid-flight
    ABORT_SENSOR = 2,     // IMU or laser lost: valve shut, no controlled descent
    ABORT_SERVO = 3,      // servo rail lost
    ABORT_TIMEOUT = 4,    // flight took too long, or an ABORT descent never landed: valve shut
    ABORT_CEILING = 5,    // too high
    ABORT_PREFLIGHT = 6,  // go refused in IDLE: a sensor not healthy, not on the ground, or moving
    ABORT_MANUAL = 7,     // go refused in IDLE: ctrl_mode is MANUAL, pick a closed-loop controller
    ABORT_POWER = 8,      // power protection tripped: controlled descent while the servo still works
                          // (or go refused in IDLE)
} abort_reason_t;

// Altitude controllers (hopper/src/control/controllers.h). Picked by cmd_t.ctrl_mode, which
// the hopper only takes while not ARMED: a change sent while ARMED waits for the next arm.
// They track the flight phase's setpoint during a hop; in IDLE the valve is always manual.
typedef enum {
    CTRL_MANUAL = 0,   // no hop possible (go refused). Also used for an unknown mode
    CTRL_PD_POS = 1,   // PD on altitude
    CTRL_CASCADE = 2,  // cascaded PID: altitude -> velocity setpoint -> valve
    CTRL_COUNT
} ctrl_mode_t;

// Controller gains, tunable from the ground (MSG_SET_GAINS) and kept in the hopper's flash.
// Defaults in hopper/src/control/controllers.cpp. The hopper takes new gains only while not
// ARMED, so they never change mid-hop.
typedef struct __attribute__((packed)) {
    float hover_pct;   // valve % that roughly holds a hover, shared by both laws (0-100)
    // CTRL_PD_POS: valve % per m of altitude error, per m/s of velocity error
    float pd_kp, pd_kd;
    // CTRL_CASCADE outer loop: (m/s) per m, per m*s, per m/s. vz_max: velocity setpoint limit (m/s)
    float pos_kp, pos_ki, pos_kd, vz_max;
    // CTRL_CASCADE inner loop: valve % per m/s, per m, per m/s^2. vel_i_max: integrator limit (valve %)
    float vel_kp, vel_ki, vel_kd, vel_i_max;
} ctrl_gains_t;

typedef enum {
    GAINS_SET = 1,       // MSG_SET_GAINS: use the gains in the packet
    GAINS_DEFAULTS = 2,  // MSG_SET_GAINS: back to the compiled-in defaults (gains ignored)
} gains_op_t;

typedef enum {
    GAINS_OK = 0,
    GAINS_REFUSED_ARMED = 1,    // only taken while not ARMED
    GAINS_REFUSED_INVALID = 2,  // a gain negative, NaN, or out of range (control/controllers.cpp)
} gains_result_t;

typedef struct __attribute__((packed)) {
    uint8_t type;    // MSG_SET_GAINS or MSG_GAINS
    uint8_t seq;     // MSG_SET_GAINS: change it to send a new request (1-255).
                     // MSG_GAINS: seq of the last request handled, 0 = none since boot
    uint8_t op;      // MSG_SET_GAINS: gains_op_t. MSG_GAINS: 0
    uint8_t result;  // MSG_GAINS: gains_result_t of that request. MSG_SET_GAINS: 0
    ctrl_gains_t gains;  // MSG_SET_GAINS: requested. MSG_GAINS: in use
} gains_packet_t;

// Calibrations, set from the ground and kept in the hopper's flash
typedef enum {
    CAL_NONE = 0,
    CAL_ZERO = 1,   // pressure zero: line vented, takes the loop current now as 0 psi
    CAL_SPAN = 2,   // pressure span: cal_value = true psi from a gauge (at least 20 psi)
    CAL_LASER = 3,  // laser offset: cal_value = true height in mm
    CAL_CLEAR = 4,  // zero, span and laser offset back to defaults (transmitter range kept)
    CAL_PTRANGE = 5,  // pressure transmitter full scale: cal_value = psi at 20 mA (100-20000).
                      // Resets the span to 1: vent and zero, then span. Same number as avionics
} cal_op_t;

// telemetry_data_t.health bits: set when that sensor is working
#define HEALTH_LASER      0x01
#define HEALTH_IMU        0x02
#define HEALTH_TEMP       0x04
#define HEALTH_PT         0x08  // pressure loop current above 3.5 mA
#define HEALTH_SERVO_PWR  0x10  // servo rail above 4.5 V (UBEC working), and power protection not tripped

// telemetry_data_t.flags bits
#define FLAG_DUAL_I2C     0x01  // laser and IMU each on their own I2C controller
#define FLAG_POWER_TRIP   0x02  // power protection tripped: arming and go refused (control/power_guard.h)
#define FLAG_SERVO_OFF    0x04  // servo signal switched off by the power protection
#define FLAG_POWER_LATCH  0x08  // tripped 3 times since power-up: stays off until the hopper restarts

// Fields sent by the control loop. NaN = no reading.
typedef struct __attribute__((packed)) {
    // Altitude filter
    float z;               // altitude estimate (m)
    float vz;              // vertical velocity estimate (m/s)
    float accel_bias;      // accelerometer bias estimate (m/s^2)
    float laser_z;         // calibrated laser height (m), NaN = no reading

    // Sensors
    float accel[3];        // x, y, z (m/s^2), sensor axes
    float gyro[3];         // x, y, z (rad/s)
    float temp_c;          // valve inlet temperature (deg C), NaN = no reading
    float pt_ma;           // pressure loop current, smoothed (mA)
    float psi;             // calibrated pressure (psi), from pt_ma
    float servo_v;         // servo rail (V)

    // Safety and valve
    uint8_t state;         // flight_state_t
    uint8_t reason;        // failsafe_reason_t, why the last FAILSAFE happened
    uint8_t valve_pct;     // valve opening sent to the servo (0-100)
    uint8_t health;        // HEALTH_* bits
    uint16_t failsafes;    // FAILSAFEs since boot

    // Link
    uint32_t cmd_seq;      // seq of the latest command, echoed
    uint32_t cmd_time_us;  // time_us of the latest command, echoed for round-trip time
    uint16_t max_gap_ms;   // worst gap between commands in the current 2 s window
    int8_t rssi;           // signal of the ground's commands at the hopper (dBm), 0 = unknown
    uint8_t tx_power;      // hopper radio power in use (dBm), 0 = full

    // Calibrations in use
    float laser_offset_m;  // added to the raw laser range
    float pt_zero_ma;      // loop current at 0 psi
    float pt_span;         // pressure scale factor
    uint8_t cal_ack;       // cal_seq of the last calibration applied

    // Diagnostics
    uint8_t flags;         // FLAG_* bits
    uint16_t i2c_errors;   // I2C read errors since boot

    // Controller and flight phase
    uint8_t ctrl_mode;     // ctrl_mode_t in use (latched at arm), not just requested
    float z_sp;            // altitude setpoint from the flight phase (m), NaN outside a hop
    float vz_sp;           // its rate (m/s), the controllers' feedforward. NaN outside a hop
    uint8_t phase;         // flight_phase_t
    uint8_t abort_reason;  // abort_reason_t of the last ABORT or refused go
    uint16_t hop_mm;       // hop profile in use: latched at go, the clamped request in IDLE
    uint16_t v_up_mms;
    uint16_t v_down_mms;
    uint16_t hover_ms;
} telemetry_data_t;

typedef struct __attribute__((packed)) {
    uint8_t type;      // MSG_TELEMETRY
    uint32_t seq;      // increments by 1 per packet; a gap means packets were lost
    uint32_t time_ms;  // hopper uptime when the data was pushed
    telemetry_data_t data;
} telemetry_packet_t;

typedef struct __attribute__((packed)) {
    uint8_t type;       // MSG_CMD
    uint8_t estop;      // 1 = E-stop: valve shut, FAILSAFE
    uint8_t arm;        // 1 = ARMED requested. Must be sent as 0 once after boot or a FAILSAFE
    uint8_t valve_pct;  // valve opening while ARMED (0-100)
    uint8_t cal_op;     // cal_op_t, applied once per new cal_seq
    uint8_t cal_seq;    // change it to apply a new calibration; echoed as cal_ack
    int16_t cal_value;  // true psi for CAL_SPAN, true mm for CAL_LASER
    uint8_t tx_power;   // hopper radio power (dBm, 2-20), 0 = full
    uint32_t seq;       // increments by 1 per command
    uint32_t time_us;   // sender's clock, echoed in telemetry
    uint8_t ctrl_mode;  // ctrl_mode_t, taken only while not ARMED

    // Hop. go rising edge in IDLE starts one. The profile below is clamped by the hopper
    // (control/flight.cpp) and latched at go: changes mid-hop wait for the next one.
    int16_t z_sp_mm;     // hop height (mm), 100-800
    uint8_t go;          // 1 = fly the hop. Dropping it mid-hop = ABORT
    uint16_t v_up_mms;   // climb rate (mm/s), 50-500
    uint16_t v_down_mms; // descent rate (mm/s), 50-300
    uint16_t hover_ms;   // time at the hop height (ms), 0-10000
} cmd_t;
