#include <stdatomic.h>
#include "telemetry.h"
#include "radio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#define TELEMETRY_CORE 0
#define TELEMETRY_PRIORITY 5
#define TELEMETRY_PERIOD_MS 5

// Single-producer single-consumer ring buffer. Only the control task writes
// head and only the telemetry task writes tail, so no locks are needed.
// RING_SIZE must be a power of two.
#define RING_SIZE 64
static telemetry_packet_t s_ring[RING_SIZE];
static atomic_uint s_head;
static atomic_uint s_tail;

bool telemetry_push(const telemetry_data_t *data)
{
    unsigned head = atomic_load_explicit(&s_head, memory_order_relaxed);
    unsigned tail = atomic_load_explicit(&s_tail, memory_order_acquire);
    if (head - tail == RING_SIZE) {
        return false;
    }

    telemetry_packet_t *pkt = &s_ring[head % RING_SIZE];
    pkt->time_ms = (uint32_t)(esp_timer_get_time() / 1000);
    pkt->data = *data;

    atomic_store_explicit(&s_head, head + 1, memory_order_release);
    return true;
}

static void telemetry_task(void *arg)
{
    uint32_t seq = 0;
    while (1) {
        unsigned tail = atomic_load_explicit(&s_tail, memory_order_relaxed);
        unsigned head = atomic_load_explicit(&s_head, memory_order_acquire);
        while (tail != head) {
            telemetry_packet_t pkt = s_ring[tail % RING_SIZE];
            tail++;
            atomic_store_explicit(&s_tail, tail, memory_order_release);

            pkt.type = MSG_TELEMETRY;
            pkt.seq = seq++;
            radio_send(&pkt, sizeof(pkt));
        }
        vTaskDelay(pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
    }
}

void telemetry_init(telemetry_cmd_handler_t cmd_handler)
{
    const uint8_t ground_mac[6] = GROUND_MAC;
    ESP_ERROR_CHECK(radio_init(ground_mac, cmd_handler));
    xTaskCreatePinnedToCore(telemetry_task, "telemetry", 4096, NULL,
                            TELEMETRY_PRIORITY, NULL, TELEMETRY_CORE);
}
