// Relays packets between the hopper (ESP-NOW) and the dashboard (USB), unchanged. The
// E-stop is digital only: the dashboard sets cmd_t.estop. The lights follow the hopper's
// telemetry.
#pragma once

#include <esp_err.h>

// Starts the radio link to HOPPER_MAC (src/protocol.h)
esp_err_t bridge_init();

// Moves everything waiting in each direction. Call every loop, never blocks.
void bridge_update();
