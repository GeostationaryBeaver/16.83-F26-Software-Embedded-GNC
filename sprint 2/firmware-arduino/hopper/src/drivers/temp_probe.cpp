#include <Arduino.h>
#include "temp_probe.h"
#include "pins.h"

// Bit timings are short, so each one runs with interrupts off on this core
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static inline void ow_low()     { digitalWrite(PIN_TEMP_OW, LOW); }
static inline void ow_release() { digitalWrite(PIN_TEMP_OW, HIGH); }  // open drain: HIGH = let R7 pull it up

// Returns true if a probe answered with a presence pulse
static bool ow_reset() {
  ow_release(); delayMicroseconds(10);
  ow_low(); delayMicroseconds(480);
  portENTER_CRITICAL(&s_mux);
  ow_release(); delayMicroseconds(70);
  bool present = digitalRead(PIN_TEMP_OW) == LOW;
  portEXIT_CRITICAL(&s_mux);
  delayMicroseconds(410);
  return present;
}

static void ow_write_bit(bool b) {
  portENTER_CRITICAL(&s_mux);
  ow_low(); delayMicroseconds(b ? 6 : 60);
  ow_release(); delayMicroseconds(b ? 64 : 10);
  portEXIT_CRITICAL(&s_mux);
}

static bool ow_read_bit() {
  portENTER_CRITICAL(&s_mux);
  ow_low(); delayMicroseconds(3);
  ow_release(); delayMicroseconds(10);
  bool b = digitalRead(PIN_TEMP_OW);
  portEXIT_CRITICAL(&s_mux);
  delayMicroseconds(53);
  return b;
}

static void ow_write(uint8_t v) {
  for (int i = 0; i < 8; i++) ow_write_bit((v >> i) & 1);
}

static uint8_t ow_read() {
  uint8_t v = 0;
  for (int i = 0; i < 8; i++) if (ow_read_bit()) v |= 1 << i;
  return v;
}

// Dallas/Maxim CRC-8
static uint8_t crc8(const uint8_t *d, int n) {
  uint8_t c = 0;
  for (int i = 0; i < n; i++) {
    uint8_t x = d[i];
    for (int b = 0; b < 8; b++) {
      uint8_t mix = (c ^ x) & 1;
      c >>= 1;
      if (mix) c ^= 0x8C;
      x >>= 1;
    }
  }
  return c;
}

// A reading: start a conversion, wait 800 ms (12 bits takes up to 750), then read the
// 9-byte scratchpad one byte per call (about 0.6 ms each)
typedef enum { PHASE_IDLE, PHASE_CONVERT, PHASE_READ } phase_t;
#define START_EVERY_MS 50   // after a reading or a failed step
#define CONVERT_MS 800

static phase_t s_phase = PHASE_IDLE;
static uint32_t s_phase_ms = 0;
static uint8_t s_sp[9], s_sp_n = 0;
static uint8_t s_fails = 0;
static bool s_ok = false;
static float s_temp_c = NAN;

static void fail() {
  if (++s_fails > 3) { s_ok = false; s_temp_c = NAN; }
  s_phase = PHASE_IDLE;
}

bool temp_probe_init() {
  pinMode(PIN_TEMP_OW, OUTPUT_OPEN_DRAIN);
  ow_release();
  s_ok = ow_reset();
  return s_ok;
}

void temp_probe_update(uint32_t now_ms) {
  if (s_phase == PHASE_IDLE) {
    if (now_ms - s_phase_ms < START_EVERY_MS) return;
    s_phase_ms = now_ms;
    if (!ow_reset()) { fail(); return; }
    ow_write(0xCC);  // skip ROM (one probe on the line)
    ow_write(0x44);  // convert T
    s_phase = PHASE_CONVERT;
  } else if (s_phase == PHASE_CONVERT) {
    if (now_ms - s_phase_ms < CONVERT_MS) return;
    s_phase_ms = now_ms;
    if (!ow_reset()) { fail(); return; }
    ow_write(0xCC);  // skip ROM
    ow_write(0xBE);  // read scratchpad
    s_phase = PHASE_READ;
    s_sp_n = 0;
  } else {
    s_sp[s_sp_n++] = ow_read();
    if (s_sp_n < 9) return;
    s_phase = PHASE_IDLE;
    s_phase_ms = now_ms;
    if (crc8(s_sp, 8) != s_sp[8]) { fail(); return; }
    s_temp_c = (int16_t)(s_sp[0] | (s_sp[1] << 8)) / 16.0f;
    s_ok = true;
    s_fails = 0;
  }
}

bool temp_probe_ok() { return s_ok; }

float temp_probe_c() { return s_ok ? s_temp_c : NAN; }
