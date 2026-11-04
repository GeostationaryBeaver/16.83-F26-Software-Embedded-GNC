// Packet definitions shared by the hopper, the ground ESP, and the dashboard.
// Two copies, which MUST stay identical:
//   firmware-arduino/hopper/src/protocol.h
//   firmware-arduino/ground/src/protocol.h
// If you change a packet, update both and the matching decode/encode in
// sprint 2/ground_station/index.html.
#pragma once

#include <stdint.h>

// Fill these in with each board's MAC address. To read a board's MAC:
//   python -m esptool --chip esp32s3 read-mac
#define HOPPER_MAC  {0xE8, 0xF6, 0x0A, 0xBE, 0x45, 0x04}
#define GROUND_MAC  {0xE8, 0xF6, 0x0A, 0xD4, 0x09, 0x68}

#define RADIO_CHANNEL 1

// First byte of every packet
typedef enum {
    MSG_TELEMETRY = 1,  // hopper -> ground, every control loop (100 Hz)
    MSG_CMD = 2,        // ground -> hopper, 50 per second. None for 150 ms = FAILSAFE
} msg_type_t;

// Hopper state. A reboot always comes up SAFE.
typedef enum {
    STATE_SAFE = 0,      // valve shut
    STATE_ARMED = 1,     // valve follows the controller (today: cmd_t.valve_pct)
    STATE_FAILSAFE = 2,  // valve shut, latched until the ground disarms and arms again
} flight_state_t;

typedef enum {
    REASON_NONE = 0,
    REASON_LINK = 1,   // no command for 150 ms
    REASON_ESTOP = 2,
} failsafe_reason_t;

// Calibrations, set from the ground and kept in the hopper's flash
typedef enum {
    CAL_NONE = 0,
    CAL_ZERO = 1,   // pressure zero: line vented, takes the loop current now as 0 psi
    CAL_SPAN = 2,   // pressure span: cal_value = true psi from a gauge (at least 20 psi)
    CAL_LASER = 3,  // laser offset: cal_value = true height in mm
    CAL_CLEAR = 4,  // back to defaults
} cal_op_t;

// telemetry_data_t.health bits: set when that sensor is working
#define HEALTH_LASER      0x01
#define HEALTH_IMU        0x02
#define HEALTH_TEMP       0x04
#define HEALTH_PT         0x08  // pressure loop current above 3.5 mA
#define HEALTH_SERVO_PWR  0x10  // servo rail above 4.5 V (UBEC working)

// telemetry_data_t.flags bits
#define FLAG_DUAL_I2C     0x01  // laser and IMU each on their own I2C controller

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
} cmd_t;
