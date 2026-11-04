#include <Arduino.h>
#include "estop.h"
#include "pins.h"

void estop_init() {
  pinMode(PIN_ESTOP, INPUT_PULLUP);
}

bool estop_open() {
  return digitalRead(PIN_ESTOP) == HIGH;
}
