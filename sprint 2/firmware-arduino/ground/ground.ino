// Ground station (Arduino Nano ESP32, TG-AV-003 rev F ground board)
// Bridge between the hopper (ESP-NOW) and the dashboard on the laptop (USB,
// sprint 2/ground_station/index.html). Packets go through unchanged in both directions.
//
// Lights: GREEN = hearing the hopper. RED solid = hopper ARMED, RED blinking = FAILSAFE.
// E-stop is digital only: the dashboard sends it in cmd_t.estop.
//
// The USB carries only framed packets; no diagnostic text is printed.
#include "src/drivers/lights.h"
#include "src/usb/usb_link.h"
#include "src/bridge/bridge.h"

#define LAMP_TEST_MS 500

void setup() {
  lights_init();  // both on: lamp test
  usb_link_init();
  esp_err_t err = bridge_init();
  if (err != ESP_OK) {
    while (true) {
      delay(1000);
    }
  }
  delay(LAMP_TEST_MS);
  lights_set(false, false);
}

void loop() {
  bridge_update();
}
