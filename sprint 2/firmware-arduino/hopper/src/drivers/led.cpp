#include <Arduino.h>
#include "led.h"

// LED_RED, LED_GREEN, LED_BLUE come from the Nano ESP32 board definition.
// The RGB LED is active low.
void led_init() {
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  led_set(false, false, false);
}

void led_set(bool r, bool g, bool b) {
  digitalWrite(LED_RED, !r);
  digitalWrite(LED_GREEN, !g);
  digitalWrite(LED_BLUE, !b);
}
