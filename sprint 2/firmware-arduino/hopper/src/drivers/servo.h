// Valve servo: 50 Hz PWM signal on D5, and the servo rail (UBEC 6 V) voltage on A1.
// Rev F has no arming plug: the servo is powered whenever the battery is in, so the signal is
// held at SHUT from servo_init() on. Only safety.cpp should move the valve.
//
// End points (shut / open pulse widths) come from flash, under the same keys the avionics
// firmware's setshut / setopen write ("tg-servo": shutUs, openUs), so a board keeps them when
// switched between the two. Defaults in pins.h: shut 2500 us, open 1500 us.
#pragma once

#include <stdint.h>

// Reads the end points, then starts the PWM signal with the valve shut. Call first thing in
// setup(). Returns false if the timer can't be set up.
bool servo_init();

// Valve opening, 0 (shut) to 100 (fully open) percent
void servo_set_valve(uint8_t pct);

// Servo signal on or off. Off = no pulses at all, so the servo stops driving and stops pulling
// current (power protection). Valve commands while off are remembered and sent when it comes
// back on. Command the valve shut before switching it off.
void servo_signal(bool on);
bool servo_signal_on();

uint16_t servo_shut_us();
uint16_t servo_open_us();
uint16_t servo_pulse_us();   // latest pulse commanded (sent only while the signal is on)

// Servo rail voltage in volts (about 0 V with the battery out). Averages 4 ADC samples.
float servo_rail_volts();
