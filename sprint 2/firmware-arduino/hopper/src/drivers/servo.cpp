#include <Arduino.h>
#include "servo.h"
#include "pins.h"

#define NUM_SAMPLES 4

static void write_us(uint32_t pulse_us) {
  uint32_t period_us = 1000000UL / SERVO_PWM_HZ;
  uint32_t duty = pulse_us * (1UL << SERVO_PWM_BITS) / period_us;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_SERVO_PWM, duty);
#else
  ledcWrite(SERVO_LEDC_CHANNEL, duty);
#endif
}

bool servo_init() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  if (!ledcAttach(PIN_SERVO_PWM, SERVO_PWM_HZ, SERVO_PWM_BITS)) return false;
#else
  if (ledcSetup(SERVO_LEDC_CHANNEL, SERVO_PWM_HZ, SERVO_PWM_BITS) == 0) return false;
  ledcAttachPin(PIN_SERVO_PWM, SERVO_LEDC_CHANNEL);
#endif
  servo_set_valve(0);
  return true;
}

void servo_set_valve(uint8_t pct) {
  if (pct > 100) pct = 100;
  write_us(SERVO_SHUT_US + (uint32_t)(SERVO_OPEN_US - SERVO_SHUT_US) * pct / 100);
}

float servo_rail_volts() {
  uint32_t sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) sum += analogReadMilliVolts(PIN_SERVO_V);
  return (float)sum / NUM_SAMPLES * SERVO_V_DIVIDER / 1000.0f;
}
