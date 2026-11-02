#include "led.h"
#include "driver/gpio.h"

// The RGB LED is active low
#define LED_RED_GPIO   GPIO_NUM_46
#define LED_GREEN_GPIO GPIO_NUM_0
#define LED_BLUE_GPIO  GPIO_NUM_45

void led_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << LED_RED_GPIO) | (1ULL << LED_GREEN_GPIO) | (1ULL << LED_BLUE_GPIO),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&cfg);
    led_set(false, false, false);
}

void led_set(bool r, bool g, bool b)
{
    gpio_set_level(LED_RED_GPIO, !r);
    gpio_set_level(LED_GREEN_GPIO, !g);
    gpio_set_level(LED_BLUE_GPIO, !b);
}
