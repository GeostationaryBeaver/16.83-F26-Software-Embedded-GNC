// Telemetry link to the ground ESP. Runs its own task on core 0.
#pragma once

#include <stdint.h>
#include "../protocol.h"

// Called for every command packet received from the ground ESP.
// Runs in the WiFi task on core 0, so keep it short.
typedef void (*telemetry_cmd_handler_t)(const uint8_t *data, int len);

void telemetry_init(telemetry_cmd_handler_t cmd_handler);

// Queues data to be sent to the ground. Lock-free and non-blocking, safe to
// call from the control loop on core 1. Must only be called from ONE task.
// Returns false (and drops the data) if the buffer is full.
bool telemetry_push(const telemetry_data_t *data);

// Sends a gains report (MSG_GAINS) to the ground on the telemetry task's next pass. Only the
// newest one waits: a report not yet sent is replaced. Non-blocking, safe from the control loop.
void telemetry_send_gains(const gains_packet_t *pkt);
