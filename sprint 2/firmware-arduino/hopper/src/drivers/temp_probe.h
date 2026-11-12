// DS18B20 temperature probe (valve inlet) on 1-Wire D4, bit-banged (no library). One probe
// on the line, powered from VDD. Non-blocking: temp_probe_update() starts a conversion, then
// 800 ms later reads the result one byte per call, so no call takes more than about 2 ms.
#pragma once

#include <stdint.h>

// Why the last reading failed
typedef enum {
  TEMP_OK = 0,
  TEMP_NO_PROBE,   // no presence pulse: wrong part, leg in the wrong screw, broken joint
  TEMP_LINE_LOW,   // data line held LOW: short to GND, or the probe in backwards
  TEMP_CRC,        // garbled scratchpad (loose data wire)
  TEMP_GARBAGE,    // answers, but the config bytes are wrong (the all-zeros "0 C" fault)
  TEMP_POWER_ON,   // 85.00 C power-on value: the probe lost power and restarted
} temp_fault_t;

// Sets up the pin. Returns true if a probe answers. It only counts as ok (temp_probe_ok)
// once a full reading has passed every check, about 1 s later.
bool temp_probe_init();

// Call every loop. Does at most one step of a reading.
void temp_probe_update(uint32_t now_ms);

// False until the first good reading, and after 4 failed readings in a row (true again on
// the next good one)
bool temp_probe_ok();

// Why the latest reading failed, TEMP_OK after a good one
temp_fault_t temp_probe_fault();

// Latest reading in deg C, NaN if there is none yet or the probe is not ok
float temp_probe_c();
