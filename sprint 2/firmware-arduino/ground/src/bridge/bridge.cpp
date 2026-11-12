#include <Arduino.h>
#include <string.h>
#include "bridge.h"
#include "../protocol.h"
#include "../drivers/lights.h"
#include "../drivers/radio.h"
#include "../usb/usb_link.h"

#define RX_QUEUE_LEN 16        // ~320 ms of telemetry at 50 Hz
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

// DIAGNOSTIC (temporary): ground board status to the laptop twice a second, type 0x7E.
// The dashboard ignores packet types it doesn't know.
#define MSG_GROUND_DIAG 0x7E
#define DIAG_PERIOD_MS 500
static uint32_t s_radio_ok = 0, s_radio_fail = 0;
static int32_t s_radio_last_err = 0;
static uint32_t s_loops = 0, s_max_loop_us = 0, s_last_loop_us = 0, s_last_diag_ms = 0;

typedef struct __attribute__((packed)) {
  uint8_t type;
  uint32_t uptime_ms, loops, max_loop_us;
  uint32_t rx_bytes, rx_packets, rx_bad, tx_dropped, tx_queued;
  int32_t serial_available, serial_room;
  uint8_t serial_connected;
  uint32_t radio_ok, radio_fail;
  int32_t radio_last_err;
  uint32_t rx_queue_waiting;
} ground_diag_t;

static void send_diag(uint32_t now) {
  if (now - s_last_diag_ms < DIAG_PERIOD_MS) return;
  s_last_diag_ms = now;
  ground_diag_t d;
  d.type = MSG_GROUND_DIAG;
  d.uptime_ms = now;
  d.loops = s_loops;
  d.max_loop_us = s_max_loop_us;
  s_max_loop_us = 0;
  d.rx_bytes = usb_link_stats.rx_bytes;
  d.rx_packets = usb_link_stats.rx_packets;
  d.rx_bad = usb_link_stats.rx_bad;
  d.tx_dropped = usb_link_stats.tx_dropped;
  d.tx_queued = usb_link_stats.tx_queued;
  d.serial_available = Serial.available();
  d.serial_room = Serial.availableForWrite();
  d.serial_connected = (bool)Serial;
  d.radio_ok = s_radio_ok;
  d.radio_fail = s_radio_fail;
  d.radio_last_err = s_radio_last_err;
  d.rx_queue_waiting = uxQueueMessagesWaiting(s_rx_queue);
  usb_link_send((const uint8_t *)&d, sizeof(d));
}

static void usb_to_hopper() {
  uint8_t pkt[USB_MAX_PACKET_LEN];
  int len;
  while ((len = usb_link_receive(pkt)) > 0) {
    esp_err_t err = radio_send(pkt, len);
    if (err == ESP_OK) {
      s_radio_ok++;
    } else {
      s_radio_fail++;
      s_radio_last_err = err;
    }
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
  uint32_t t_us = micros();
  if (s_loops && t_us - s_last_loop_us > s_max_loop_us) s_max_loop_us = t_us - s_last_loop_us;
  s_last_loop_us = t_us;
  s_loops++;
  send_diag(now);
  hopper_to_usb(now);
  usb_link_service();
  usb_to_hopper();
  update_lights(now);
}
