// Point-to-point ESP-NOW link to a single peer board.
#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

// Called from the WiFi task for every packet received from the peer. Keep it short.
typedef void (*radio_rx_cb_t)(const uint8_t *data, int len);

esp_err_t radio_init(const uint8_t peer_mac[6], radio_rx_cb_t rx_cb);

// Queues a packet (max 250 bytes) for sending to the peer. Does not block.
esp_err_t radio_send(const void *data, size_t len);
