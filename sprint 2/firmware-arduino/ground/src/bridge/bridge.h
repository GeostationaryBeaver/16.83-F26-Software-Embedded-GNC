// Relays packets between the hopper (ESP-NOW) and the dashboard (USB), unchanged except
// for one thing: while the E-stop switch is open, every command goes out with estop = 1,
// arm = 0 and valve_pct = 0, whatever the dashboard sent. The lights follow the hopper's
// telemetry.
#pragma once

#include <esp_err.h>

// Starts the radio link to HOPPER_MAC (src/protocol.h)
esp_err_t bridge_init();

// Moves everything waiting in each direction. Call every loop, never blocks.
void bridge_update();
