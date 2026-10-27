#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

#include "load_cell.h"

void app_main(void)
{
    ESP_ERROR_CHECK(load_cell_init());

    uint32_t count = 0;
    while (1) {
        printf("Hello world! %" PRIu32 "\n", count++);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
