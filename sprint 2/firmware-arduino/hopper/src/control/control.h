// Hard real-time control loop: sensors, commands from the ground, the safety state machine,
// the altitude filter, the controller (controllers.h), and telemetry. Runs its own task on core 1.
#pragma once

#include "../protocol.h"

// Starts the sensors and the control task. Call after safety_init().
void control_start();

// Hands a command from the ground to the control loop. Call from the radio callback (WiFi
// task, core 0). Never blocks; the control loop takes the newest one.
void control_on_command(const cmd_t *cmd);

// Hands a gains request (MSG_SET_GAINS) from the ground to the control loop. Same rules as
// control_on_command. The loop takes it only while not ARMED and answers with MSG_GAINS.
void control_on_gains(const gains_packet_t *pkt);
