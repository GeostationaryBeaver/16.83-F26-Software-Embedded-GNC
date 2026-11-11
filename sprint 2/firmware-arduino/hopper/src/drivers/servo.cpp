#include <Arduino.h>
#include <Preferences.h>
#include "servo.h"
#include "pins.h"

#define NUM_SAMPLES 4

static uint16_t s_shut_us = SERVO_SHUT_US_DEFAULT;
static uint16_t s_open_us = SERVO_OPEN_US_DEFAULT;
static uint16_t s_pulse_us = SERVO_SHUT_US_DEFAULT;
static bool s_on = true;

static void write_duty(uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_SERVO_PWM, duty);
#else
  ledcWrite(SERVO_LEDC_CHANNEL, duty);
#endif
}

static void write_us(uint16_t pulse_us) {
  s_pulse_us = pulse_us;
  if (!s_on) return;
  write_duty((uint32_t)pulse_us * (1UL << SERVO_PWM_BITS) / (1000000UL / SERVO_PWM_HZ));
}

static bool end_point_ok(uint16_t us) { return us >= SERVO_US_MIN && us <= SERVO_US_MAX; }

// Same keys as the avionics firmware. A pair that is out of range or equal is ignored, so a
// corrupt entry can't leave "shut" pointing somewhere else.
static void load_end_points() {
  Preferences prefs;
  prefs.begin("tg-servo", true);
  uint16_t shut = prefs.getUShort("shutUs", SERVO_SHUT_US_DEFAULT);
  uint16_t open = prefs.getUShort("openUs", SERVO_OPEN_US_DEFAULT);
  prefs.end();
  if (end_point_ok(shut) && end_point_ok(open) && shut != open) {
    s_shut_us = shut;
    s_open_us = open;
  }
}

bool servo_init() {
  load_end_points();  // a few ms, before the first pulse so "shut" is the real shut
  s_pulse_us = s_shut_us;
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
  // Signed: open can be below shut (it is with the defaults)
  int32_t us = (int32_t)s_shut_us + ((int32_t)s_open_us - (int32_t)s_shut_us) * pct / 100;
  write_us((uint16_t)us);
}

void servo_signal(bool on) {
  if (on == s_on) return;
  s_on = on;
  if (on) write_us(s_pulse_us);
  else write_duty(0);  // line held low: a servo with no pulses stops driving
}

bool servo_signal_on() { return s_on; }
uint16_t servo_shut_us() { return s_shut_us; }
uint16_t servo_open_us() { return s_open_us; }
uint16_t servo_pulse_us() { return s_pulse_us; }

float servo_rail_volts() {
  uint32_t sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) sum += analogReadMilliVolts(PIN_SERVO_V);
  return (float)sum / NUM_SAMPLES * SERVO_V_DIVIDER / 1000.0f;
}
