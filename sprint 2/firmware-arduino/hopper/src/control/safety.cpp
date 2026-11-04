#include <Arduino.h>
#include "safety.h"
#include "../protocol.h"
#include "../drivers/servo.h"
#include "../drivers/led.h"

static uint8_t s_state = STATE_SAFE;
static uint8_t s_reason = REASON_NONE;
static uint8_t s_valve = 0;
static bool s_need_disarm = true;
static uint16_t s_failsafes = 0;

static void set_valve(uint8_t pct) {
  if (pct > 100) pct = 100;
  s_valve = pct;
  servo_set_valve(pct);
}

static void go_failsafe(uint8_t why) {
  s_state = STATE_FAILSAFE;
  s_reason = why;
  s_need_disarm = true;
  s_failsafes++;
  set_valve(0);
  Serial.printf(">>> FAILSAFE (%s): valve driven shut\n", why == REASON_LINK ? "link lost" : "E-stop");
}

void safety_init() {
  servo_init();  // valve shut before anything else happens
  led_init();
  led_set(false, false, true);
}

void safety_update(uint32_t now_ms, const safety_inputs_t *in, valve_demand_fn_t demand) {
  if (!in->arm && in->link_up) s_need_disarm = false;

  switch (s_state) {
    case STATE_SAFE:
      if (s_valve) set_valve(0);
      if (in->link_up && in->arm && !in->estop && !s_need_disarm) {
        s_state = STATE_ARMED;
        Serial.println(">>> ARMED");
      }
      break;
    case STATE_ARMED:
      if (!in->link_up) {
        go_failsafe(REASON_LINK);
      } else if (in->estop) {
        go_failsafe(REASON_ESTOP);
      } else if (!in->arm) {
        s_state = STATE_SAFE;
        set_valve(0);
        Serial.println(">>> SAFE (disarmed)");
      } else {
        uint8_t v = demand(now_ms);
        if (v != s_valve) set_valve(v);
      }
      break;
    case STATE_FAILSAFE:
      if (s_valve) set_valve(0);
      if (in->link_up && !in->arm) {
        s_state = STATE_SAFE;
        s_reason = REASON_NONE;
        Serial.println(">>> SAFE (failsafe cleared)");
      }
      break;
  }

  if (!in->heard) led_set(false, false, true);
  else if (s_state == STATE_FAILSAFE || !in->link_up) led_set(true, false, false);
  else if (s_state == STATE_ARMED) led_set(true, true, false);
  else led_set(false, true, false);
}

uint8_t safety_state() { return s_state; }
uint8_t safety_reason() { return s_reason; }
uint8_t safety_valve_pct() { return s_valve; }
uint16_t safety_failsafes() { return s_failsafes; }
