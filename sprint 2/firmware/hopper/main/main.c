#include "led.h"
#include "telemetry.h"
#include "control.h"

// Commands from the ground ESP. Runs in the WiFi task on core 0.
static void handle_command(const uint8_t *data, int len)
{
    if (len < 1) {
        return;
    }
    switch (data[0]) {
    case MSG_CMD_SET_LED:
        if (len == sizeof(cmd_set_led_t)) {
            const cmd_set_led_t *cmd = (const cmd_set_led_t *)data;
            led_set(cmd->r, cmd->g, cmd->b);
        }
        break;
    }
}

void app_main(void)
{
    led_init();
    telemetry_init(handle_command);  // core 0
    control_start();                 // core 1
}
