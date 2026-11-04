// Valve servo: 50 Hz PWM signal on D5 (500 us = shut, 1500 us = fully open), and the servo
// rail (UBEC 6 V) voltage on A1. Rev F has no arming plug: the servo is powered whenever the
// battery is in, so the signal is held at SHUT from servo_init() on. Only safety.cpp should
// move the valve.
#pragma once

#include <stdint.h>

// Starts the PWM signal with the valve shut. Call first thing in setup().
// Returns false if the timer can't be set up.
bool servo_init();

// Valve opening, 0 (shut) to 100 (fully open) percent
void servo_set_valve(uint8_t pct);

// Servo rail voltage in volts (about 0 V with the battery out). Averages 4 ADC samples.
float servo_rail_volts();
