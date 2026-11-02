// PLACEHOLDER: stands in for the real control loop so telemetry can be tested.
// Replace the body of control_task with the real loop.
#include <math.h>
#include "control.h"
#include "telemetry.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define CONTROL_CORE 1
#define CONTROL_PRIORITY 20
#define CONTROL_PERIOD_MS 10

static void control_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();
    float t = 0.0f;
    while (1) {
        telemetry_data_t data = {
            .placeholder = { sinf(t), cosf(t), sinf(2.0f * t), t },
        };
        telemetry_push(&data);

        t += CONTROL_PERIOD_MS / 1000.0f;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

void control_start(void)
{
    xTaskCreatePinnedToCore(control_task, "control", 4096, NULL,
                            CONTROL_PRIORITY, NULL, CONTROL_CORE);
}
