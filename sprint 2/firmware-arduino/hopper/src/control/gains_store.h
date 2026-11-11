// Keeps the controller gains (ctrl_gains_t, controllers.h) in flash, so gains tuned from the
// ground survive a reboot. Same flash library as the calibration, its own namespace.
#pragma once

#include "../protocol.h"

// Puts the saved gains in use (controller_set_gains). Keeps the defaults if there are none,
// or if they were saved by firmware with a different ctrl_gains_t.
void gains_store_load();

// Saves the gains in use. Writing flash stalls the loop for a few ms: only while not ARMED.
void gains_store_save();

// Forgets the saved gains, so the next boot uses the defaults.
void gains_store_clear();
