// Vehicle board pins, I2C addresses and analog constants (board rev F).
// Taken from the avionics vehicle firmware TG-AV-003 rev F (v1.1, updated to v1.3a).
#pragma once

#include <Arduino.h>

// I2C: laser on Wire (A4/A5), IMU on Wire1 (D2/D3), or one shared controller (see i2c_bus.h)
static const uint8_t PIN_LASER_SDA = A4;
static const uint8_t PIN_LASER_SCL = A5;
static const uint8_t PIN_IMU_SDA = D2;
static const uint8_t PIN_IMU_SCL = D3;
static const uint32_t I2C_FREQ_HZ = 100000;  // what every bench test used. Try 400000 on the perfboard
static const uint16_t I2C_TIMEOUT_MS = 5;

static const uint8_t LASER_ADDR = 0x29;     // VL53L0X
static const uint8_t IMU_ADDR = 0x6A;       // LSM6DS3, SA0 to GND (as designed)
static const uint8_t IMU_ADDR_ALT = 0x6B;   // LSM6DS3, SA0 high or floating

// DS18B20 data, R7 4.7k pull-up to 3V3
static const uint8_t PIN_TEMP_OW = D4;

// Servo signal on D5 via R1. The core translates Arduino pin names for the LEDC calls only
// if they have a remap macro, otherwise the chip pin is passed.
#if defined(ledcAttachPin) || defined(ledcAttach)
static const uint8_t PIN_SERVO_PWM = D5;
#else
static const uint8_t PIN_SERVO_PWM = 8;     // GPIO8 = D5
#endif
static const uint8_t SERVO_LEDC_CHANNEL = 0;  // core 2.x only
static const uint32_t SERVO_PWM_HZ = 50;
static const uint8_t SERVO_PWM_BITS = 14;
// Valve end points. Defaults only: the ones in use come from flash (servo.h), where the
// avionics firmware's setshut / setopen saves them. Avionics set these on the bench on
// 2026-11-10 after re-indexing the servo on the valve; before that shut was 500 us.
static const uint16_t SERVO_SHUT_US_DEFAULT = 2500;  // valve shut
static const uint16_t SERVO_OPEN_US_DEFAULT = 1500;  // 90 deg from shut = fully open
static const uint16_t SERVO_US_MIN = 100;            // accepted from flash: anything outside
static const uint16_t SERVO_US_MAX = 2900;           // this is a corrupt entry, not an end point

// Analog inputs, same rule as the servo pin for analogReadMilliVolts()
#if defined(analogReadMilliVolts)
static const uint8_t PIN_PRESSURE = A0;
static const uint8_t PIN_SERVO_V = A1;
#else
static const uint8_t PIN_PRESSURE = 1;      // GPIO1 = A0
static const uint8_t PIN_SERVO_V = 2;       // GPIO2 = A1
#endif

// Pressure transmitter PT1: 4-20 mA loop through the R3 shunt, 4 mA = 0 psi, 20 mA = full scale
static const float PRESSURE_SHUNT_OHMS = 100.0f;
static const float PRESSURE_MA_ZERO = 4.0f;   // default zero, until calibrated
static const float PRESSURE_MA_SPAN = 16.0f;
static const float PRESSURE_PSI_FULL_SCALE = 1000.0f;  // default: CAL_PTRANGE changes it (5000 for McMaster 3196K611)
static const float PRESSURE_MA_PRESENT = 3.5f;  // below this the transmitter is off or unplugged
static const float PRESSURE_MA_FAULT = 21.0f;   // above this: over range, or a short across R3

// Servo rail (UBEC 6 V) into A1 through R5 100k / R6 47k. Rev F has no arming plug: the
// UBEC, and so the servo, is powered whenever the battery is in.
static const float SERVO_V_DIVIDER = (100.0f + 47.0f) / 47.0f;
static const float SERVO_V_MIN = 4.5f;        // above this = UBEC working
static const float SERVO_V_ABSENT = 1.0f;     // below this = no servo power at all (USB-only bench)

// Power protection (control/power_guard.h), avionics v1.3a thresholds. UBEC set to 6.0 V.
static const float PG_LOW_V = 5.4f;           // trips below this for PG_LOW_MS: battery flat / UBEC can't keep up
static const uint32_t PG_LOW_MS = 500;
static const float PG_HIGH_V = 6.9f;          // trips above this for PG_HIGH_MS: UBEC jumper not on 6.0 V
static const uint32_t PG_HIGH_MS = 300;
static const uint8_t PG_FLIPS = 4;            // rail crossing SERVO_V_MIN this many times...
static const uint32_t PG_FLIP_WINDOW_MS = 10000;  // ...within this = brown-out loop
static const float PG_LOOP_MA = 3.3f;         // pressure loop below this (after it was healthy)...
static const uint32_t PG_LOOP_MS = 2000;      // ...for this long = 12 V battery too low for the transmitter
static const float PG_OK_LO_V = 5.6f;         // clears after PG_OK_MS steady inside this band, disarmed
static const float PG_OK_HI_V = 6.6f;
static const uint32_t PG_OK_MS = 5000;
static const uint32_t PG_CUT_DELAY_MS = 600;  // valve commanded shut this long before the signal goes off
static const uint8_t PG_LATCH_TRIPS = 3;      // this many trips since power-up: stays off until a reboot
