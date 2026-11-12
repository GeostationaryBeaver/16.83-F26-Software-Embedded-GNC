#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include "laser.h"
#include "i2c_bus.h"

static VL53L0X s_tof;
static bool s_ok = false;
static uint32_t s_new_ms = 0;   // last new range, for the stall check
static uint32_t s_poll_ms = 0;  // last laser_read() call

bool laser_init() {
  s_ok = false;
  s_tof.setBus(&i2c_bus_get(I2C_BUS_LASER));
  s_tof.setTimeout(60);
  if (!s_tof.init()) return false;  // also checks the model ID (0xEE)
  s_tof.setMeasurementTimingBudget(33000);
  s_tof.startContinuous();          // back to back
  s_new_ms = s_poll_ms = millis();
  s_ok = true;
  return true;
}

laser_result_t laser_read(uint16_t *mm) {
  if (!s_ok) return LASER_ERROR;
  uint32_t now = millis();
  if (now - s_poll_ms > 100) s_new_ms = now;  // polling was paused: not the laser's fault
  s_poll_ms = now;
  i2c_bus_get(I2C_BUS_LASER);

  // Low 3 bits of the interrupt status are set when a new range is ready
  uint8_t status = s_tof.readReg(VL53L0X::RESULT_INTERRUPT_STATUS);
  if (s_tof.last_status != 0) return LASER_ERROR;
  if ((status & 0x07) == 0) {
    // A laser that lost power for a moment answers again but has stopped ranging
    if (now - s_new_ms > LASER_STALL_MS) { s_new_ms = now; return LASER_STALLED; }
    return LASER_NO_DATA;
  }

  // A range is ready, so this reads it and clears the flag without waiting
  uint16_t range = s_tof.readRangeContinuousMillimeters();
  if (s_tof.last_status != 0 || s_tof.timeoutOccurred() || range == 0xFFFF) return LASER_ERROR;
  s_new_ms = now;
  *mm = range;
  return LASER_NEW;
}
