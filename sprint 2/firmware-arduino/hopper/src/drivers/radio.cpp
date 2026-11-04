#include <string.h>
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "radio.h"
#include "../protocol.h"

static uint8_t s_peer_mac[6];
static radio_rx_cb_t s_rx_cb;
static uint8_t s_tx_power = 0;
static volatile int8_t s_rssi = 0;

// The receive callback signature changed in ESP32 Arduino core 3.x
#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  const uint8_t *src = info->src_addr;
#else
static void on_recv(const uint8_t *src, const uint8_t *data, int len) {
#endif
  // Ignore packets from anything other than our peer
  if (memcmp(src, s_peer_mac, 6) != 0) return;
  s_rx_cb(data, len);
}

// Signal strength: every received radio frame passes here. ESP-NOW frames are management
// frames; keep the ones sent by our peer (sender address at byte 10).
static void on_sniff(void *buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT) return;
  const wifi_promiscuous_pkt_t *p = (const wifi_promiscuous_pkt_t *)buf;
  if (memcmp(p->payload + 10, s_peer_mac, 6) == 0) s_rssi = (int8_t)p->rx_ctrl.rssi;
}

esp_err_t radio_init(const uint8_t peer_mac[6], radio_rx_cb_t rx_cb) {
  memcpy(s_peer_mac, peer_mac, 6);
  s_rx_cb = rx_cb;

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(RADIO_CHANNEL, WIFI_SECOND_CHAN_NONE);

  esp_err_t err = esp_now_init();
  if (err != ESP_OK) return err;
  esp_now_register_recv_cb(on_recv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, peer_mac, 6);
  peer.channel = RADIO_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  err = esp_now_add_peer(&peer);
  if (err != ESP_OK) return err;

  radio_set_tx_power(0);

  wifi_promiscuous_filter_t filter = {};
  filter.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT;
  esp_wifi_set_promiscuous_filter(&filter);
  esp_wifi_set_promiscuous_rx_cb(on_sniff);
  return esp_wifi_set_promiscuous(true);
}

esp_err_t radio_send(const void *data, size_t len) {
  return esp_now_send(s_peer_mac, (const uint8_t *)data, len);
}

void radio_set_tx_power(uint8_t dbm) {
  int8_t quarter_db = dbm ? constrain(dbm, 2, 20) * 4 : 84;  // quarter-dB units, 8..84
  esp_wifi_set_max_tx_power(quarter_db);
  s_tx_power = dbm;
}

uint8_t radio_tx_power() { return s_tx_power; }

int8_t radio_rssi() { return s_rssi; }
