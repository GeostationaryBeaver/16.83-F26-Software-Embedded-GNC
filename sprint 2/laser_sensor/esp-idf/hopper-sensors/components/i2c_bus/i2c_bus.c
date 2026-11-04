/* ===========================================================================
 * i2c_bus.c -- implementation of the shared I2C bus helpers.
 * ===========================================================================
 */

#include "i2c_bus.h"
#include "esp_log.h"

/* A file-local log tag. `static` means "not visible outside this file", so
 * every .c file can have its own TAG without a name collision at link time.
 * This is the universal ESP-IDF convention. */
static const char *TAG = "i2c_bus";

esp_err_t i2c_bus_init(int sda_gpio, int scl_gpio, uint32_t freq_hz,
                       i2c_master_bus_handle_t *out_handle)
{
    if (out_handle == NULL) return ESP_ERR_INVALID_ARG;

    /* `freq_hz` is accepted for API symmetry but is not used here: in the
     * v5.2+ driver the clock speed is a per-DEVICE property, set when each
     * driver calls i2c_master_bus_add_device(). The bus itself only owns the
     * pins and the port. The cast to void silences the unused-parameter
     * warning without deleting the parameter. */
    (void)freq_hz;

    /* Designated initialisers: `.field = value`. Any field you do not mention
     * is zero-initialised, which is why this is safe even though the struct
     * has more members than we set. Much less error-prone than positional
     * initialisation, and the standard style in ESP-IDF. */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = I2C_NUM_0,
        .sda_io_num        = sda_gpio,
        .scl_io_num        = scl_gpio,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,      /* 7 is the recommended default; filters
                                      * short noise spikes on the lines */
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&bus_cfg, out_handle);
    if (err != ESP_OK) {
        /* esp_err_to_name turns the numeric code into a readable string --
         * always use it in error logs. "ESP_ERR_INVALID_STATE" tells you far
         * more than "0x103". */
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "bus up on SDA=%d SCL=%d", sda_gpio, scl_gpio);
    return ESP_OK;
}

esp_err_t i2c_bus_probe(i2c_master_bus_handle_t bus, uint8_t addr_7bit)
{
    /* 50 ms timeout. A device that is present answers in microseconds; this
     * only has to be long enough to not false-negative under load. */
    return i2c_master_probe(bus, addr_7bit, 50);
}

void i2c_bus_scan(i2c_master_bus_handle_t bus)
{
    ESP_LOGI(TAG, "scanning bus...");

    int found = 0;

    /* 0x00-0x07 and 0x78-0x7F are reserved by the I2C spec, so a scan covers
     * 0x08..0x77 only. */
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_master_probe(bus, addr, 20) == ESP_OK) {
            ESP_LOGI(TAG, "  found device at 0x%02X", addr);
            found++;
        }
    }

    if (found == 0) {
        ESP_LOGW(TAG, "  no devices responded -- check power, wiring, pull-ups");
    } else {
        ESP_LOGI(TAG, "scan complete: %d device(s)", found);
    }
}
