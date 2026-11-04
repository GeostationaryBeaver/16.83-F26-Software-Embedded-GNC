// Ground station board pins (TG-AV-003 rev F).
#pragma once

#include <Arduino.h>

static const uint8_t PIN_ARMED_LED = D5;  // red
static const uint8_t PIN_LINK_LED = D6;   // green
static const uint8_t PIN_ESTOP = D2;      // normally closed to GND, HIGH = stop
