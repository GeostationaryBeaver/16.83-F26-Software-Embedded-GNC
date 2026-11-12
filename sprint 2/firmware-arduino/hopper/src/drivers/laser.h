// VL53L0X laser rangefinder on I2C A4/A5, via the Pololu "VL53L0X" library.
// Runs in continuous mode so reads never wait for a measurement.
#pragma once

#include <stdint.h>

// Range the sensor reports when nothing is in range (about 1.2 m or more)
#define LASER_OUT_OF_RANGE_MM 8190

typedef enum {
  LASER_NO_DATA,  // no new range ready yet
  LASER_NEW,      // a new range was read
  LASER_ERROR,    // I2C error, timeout, or the 65535 an unplugged laser returns
  LASER_STALLED,  // answering, but no new range for LASER_STALL_MS (laser rebooted or froze)
} laser_result_t;

// No new range for this long while being polled = LASER_STALLED (a range takes ~33 ms)
#define LASER_STALL_MS 250

// Starts the sensor and continuous ranging (33 ms timing budget, about 30 ranges/s).
// Blocks for the library's calibration. Returns false if it is not found or fails to start.
bool laser_init();

// If a new range is ready, writes it to *mm (millimetres, >= LASER_OUT_OF_RANGE_MM means
// nothing in range). Returns straight away. Call it at least every 100 ms: a longer pause
// restarts the stall clock instead of counting as a stall.
laser_result_t laser_read(uint16_t *mm);
