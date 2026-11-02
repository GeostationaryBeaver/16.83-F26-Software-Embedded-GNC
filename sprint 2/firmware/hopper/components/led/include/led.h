// Arduino Nano ESP32 onboard RGB LED.
#pragma once

#include <stdbool.h>

void led_init(void);
void led_set(bool r, bool g, bool b);
