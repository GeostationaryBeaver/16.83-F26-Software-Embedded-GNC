// Hopper flight computer (Arduino Nano ESP32, vehicle board rev F)
//   core 0: telemetry to the ground ESP over ESP-NOW (src/telemetry)
//   core 1: control loop: sensors, safety state machine, valve (src/control)
// Valve servo held SHUT from power-up. It moves only while ARMED by the ground, and a
// FAILSAFE (no command for 150 ms, or E-stop) drives it shut. A reboot always comes up SAFE.
#include "src/protocol.h"
#include "src/telemetry/telemetry.h"
#include "src/control/control.h"
#include "src/control/safety.h"

// Packets from the ground ESP. Runs in the WiFi task on core 0.
void handleCommand(const uint8_t *data, int len) {
  if (len < 1) return;
  switch (data[0]) {
    case MSG_CMD:
      if (len == sizeof(cmd_t)) control_on_command((const cmd_t *)data);
      break;
    case MSG_SET_GAINS:
      if (len == sizeof(gains_packet_t)) control_on_gains((const gains_packet_t *)data);
      break;
  }
}

void setup() {
  safety_init();  // valve shut before anything else happens
  control_start();                // core 1. Sensors start with the servo signal running
  telemetry_init(handleCommand);  // core 0
}

void loop() {
  // Everything runs in its own task. Delete the Arduino loop task so it
  // doesn't take time on core 1 from the control loop.
  vTaskDelete(NULL);
}
