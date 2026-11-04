#include <Arduino.h>
#include <atomic>
#include "telemetry.h"
#include "../drivers/radio.h"

#define TELEMETRY_CORE 0
#define TELEMETRY_PRIORITY 5
#define TELEMETRY_PERIOD_MS 5

// Single-producer single-consumer ring buffer. Only the control task writes
// head and only the telemetry task writes tail, so no locks are needed.
// RING_SIZE must be a power of two.
#define RING_SIZE 64
static telemetry_packet_t s_ring[RING_SIZE];
static std::atomic<uint32_t> s_head{0};
static std::atomic<uint32_t> s_tail{0};

bool telemetry_push(const telemetry_data_t *data) {
  uint32_t head = s_head.load(std::memory_order_relaxed);
  uint32_t tail = s_tail.load(std::memory_order_acquire);
  if (head - tail == RING_SIZE) return false;

  telemetry_packet_t *pkt = &s_ring[head % RING_SIZE];
  pkt->time_ms = millis();
  pkt->data = *data;

  s_head.store(head + 1, std::memory_order_release);
  return true;
}

static void telemetry_task(void *arg) {
  uint32_t seq = 0;
  while (true) {
    uint32_t tail = s_tail.load(std::memory_order_relaxed);
    uint32_t head = s_head.load(std::memory_order_acquire);
    while (tail != head) {
      telemetry_packet_t pkt = s_ring[tail % RING_SIZE];
      tail++;
      s_tail.store(tail, std::memory_order_release);

      pkt.type = MSG_TELEMETRY;
      pkt.seq = seq++;
      radio_send(&pkt, sizeof(pkt));
    }
    vTaskDelay(pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
  }
}

void telemetry_init(telemetry_cmd_handler_t cmd_handler) {
  const uint8_t ground_mac[6] = GROUND_MAC;
  esp_err_t err = radio_init(ground_mac, cmd_handler);
  if (err != ESP_OK) Serial.printf("telemetry: radio init failed (%s)\n", esp_err_to_name(err));
  xTaskCreatePinnedToCore(telemetry_task, "telemetry", 4096, NULL,
                          TELEMETRY_PRIORITY, NULL, TELEMETRY_CORE);
}
