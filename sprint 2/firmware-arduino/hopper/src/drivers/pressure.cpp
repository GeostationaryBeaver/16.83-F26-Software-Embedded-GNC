#include <Arduino.h>
#include "pressure.h"
#include "pins.h"

#define NUM_SAMPLES 4

float pressure_read_ma() {
  uint32_t sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) sum += analogReadMilliVolts(PIN_PRESSURE);
  return (float)sum / NUM_SAMPLES / PRESSURE_SHUNT_OHMS;  // mV / ohm = mA
}
