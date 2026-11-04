// Vehicle board pins, I2C addresses and analog constants (board rev F).
// Taken from the avionics vehicle firmware TG-AV-003 rev F v1.1.
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
static const uint16_t SERVO_SHUT_US = 500;    // valve shut. Check both ends with the linkage on
static const uint16_t SERVO_OPEN_US = 1500;   // 90 deg = fully open

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
static const float PRESSURE_PSI_FULL_SCALE = 1000.0f;
static const float PRESSURE_MA_PRESENT = 3.5f;  // below this the transmitter is off or unplugged

// Servo rail (UBEC 6 V) into A1 through R5 100k / R6 47k. Rev F has no arming plug: the
// UBEC, and so the servo, is powered whenever the battery is in.
static const float SERVO_V_DIVIDER = (100.0f + 47.0f) / 47.0f;
static const float SERVO_V_MIN = 4.5f;        // above this = UBEC working
