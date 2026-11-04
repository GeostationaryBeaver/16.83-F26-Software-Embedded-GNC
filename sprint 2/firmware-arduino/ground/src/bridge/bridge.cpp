#include <Arduino.h>
#include <string.h>
#include "bridge.h"
#include "../protocol.h"
#include "../drivers/estop.h"
#include "../drivers/lights.h"
#include "../drivers/radio.h"
#include "../usb/usb_link.h"

#define RX_QUEUE_LEN 16        // ~150 ms of telemetry at 100 Hz
#define LINK_TIMEOUT_MS 150    // same as the hopper's failsafe
#define FAILSAFE_BLINK_MS 125

typedef struct {
  uint8_t len;
  uint8_t data[USB_MAX_PACKET_LEN];
} radio_packet_t;

// Hopper -> loop(). The WiFi task must never block, so packets wait here for the USB.
static QueueHandle_t s_rx_queue;

// From the hopper's telemetry, for the lights
static uint32_t s_last_tlm_ms = 0;
static uint8_t s_hopper_state = STATE_SAFE;

// Runs in the WiFi task on core 0
static void on_radio_rx(const uint8_t *data, int len) {
  if (len < 1 || len > USB_MAX_PACKET_LEN) return;
  radio_packet_t pkt;
  pkt.len = len;
  memcpy(pkt.data, data, len);
  xQueueSend(s_rx_queue, &pkt, 0);  // full = dropped; the dashboard sees the seq gap
}

static void hopper_to_usb(uint32_t now) {
  radio_packet_t pkt;
  while (xQueueReceive(s_rx_queue, &pkt, 0) == pdTRUE) {
    usb_link_send(pkt.data, pkt.len);
    if (pkt.data[0] == MSG_TELEMETRY && pkt.len == sizeof(telemetry_packet_t)) {
      s_last_tlm_ms = now ? now : 1;  // 0 means none yet
      s_hopper_state = ((const telemetry_packet_t *)pkt.data)->data.state;
    }
  }
}

static void usb_to_hopper() {
  uint8_t pkt[USB_MAX_PACKET_LEN];
  int len;
  while ((len = usb_link_receive(pkt)) > 0) {
    if (pkt[0] == MSG_CMD && len == sizeof(cmd_t) && estop_open()) {
      cmd_t *cmd = (cmd_t *)pkt;
      cmd->estop = 1;
      cmd->arm = 0;
      cmd->valve_pct = 0;
    }
    radio_send(pkt, len);
  }
}

// Green = hearing the hopper. Red solid = hopper ARMED, red blinking = hopper FAILSAFE.
static void update_lights(uint32_t now) {
  bool link = s_last_tlm_ms && now - s_last_tlm_ms < LINK_TIMEOUT_MS;
  bool red = link && (s_hopper_state == STATE_ARMED ||
                      (s_hopper_state == STATE_FAILSAFE && (now / FAILSAFE_BLINK_MS) % 2));
  lights_set(link, red);
}

esp_err_t bridge_init() {
  s_rx_queue = xQueueCreate(RX_QUEUE_LEN, sizeof(radio_packet_t));
  const uint8_t hopper_mac[6] = HOPPER_MAC;
  return radio_init(hopper_mac, on_radio_rx);
}

void bridge_update() {
  uint32_t now = millis();
  hopper_to_usb(now);
  usb_to_hopper();
  update_lights(now);
}
