// DS18B20 temperature probe (valve inlet) on 1-Wire D4, bit-banged (no library). One probe
// on the line, powered from VDD. Non-blocking: temp_probe_update() starts a conversion, then
// 800 ms later reads the result one byte per call, so no call takes more than about 2 ms.
#pragma once

#include <stdint.h>

// Sets up the pin. Returns false if no probe answers.
bool temp_probe_init();

// Call every loop. Does at most one step of a reading.
void temp_probe_update(uint32_t now_ms);

// False after 4 failed readings in a row (true again on the next good one)
bool temp_probe_ok();

// Latest reading in deg C, NaN if there is none yet or the probe is not ok
float temp_probe_c();
