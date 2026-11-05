#include <Arduino.h>
#include "lights.h"
#include "pins.h"

void lights_init() {
  pinMode(PIN_ARMED_LED, OUTPUT);
  pinMode(PIN_LINK_LED, OUTPUT);
  lights_set(true, true);
}

void lights_set(bool link, bool armed) {
  digitalWrite(PIN_LINK_LED, link);
  digitalWrite(PIN_ARMED_LED, armed);
}
