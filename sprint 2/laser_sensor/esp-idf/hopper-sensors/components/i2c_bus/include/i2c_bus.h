/* ===========================================================================
 * i2c_bus.h -- one shared I2C master bus for every I2C sensor on the board.
 *
 * WHY A SEPARATE COMPONENT
 * ------------------------
 * I2C is a bus: many devices share two wires, distinguished by address. The
 * bus is initialised ONCE, then each driver adds itself as a device on it.
 * If every driver called i2c_new_master_bus() you would get a "port already
 * in use" error on the second one.
 *
 * So: main.c brings the bus up, then hands the handle to each driver.
 *
 * API VERSION WARNING
 * -------------------
 * This uses the driver/i2c_master.h API, new in ESP-IDF v5.2. Most tutorials
 * online use the older driver/i2c.h API (i2c_param_config,
 * i2c_master_cmd_begin). They are not interchangeable. If you are on IDF
 * v5.1 or earlier, either upgrade or port this file -- the concepts map, but
 * every function name differs.
 * ===========================================================================
 */

#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

/* --- Default pins ---------------------------------------------------------
 * These are fallbacks only. main.c passes explicit pins per board, which is
 * what you should do too -- the right numbers differ between ESP32 variants
 * and there is no safe universal default.
 *
 * The defaults here are chosen to be valid on BOTH the classic ESP32 and the
 * ESP32-S3, which rules out the usual SDA=21/SCL=22 pairing: the S3 has no
 * GPIO 22 at all (its numbering skips 22-25). GPIO 21 and 19 exist on both.
 *
 * Two related traps, both worth internalising:
 *
 *   - Pin numbers here are RAW GPIO numbers, not Arduino labels. On the
 *     Arduino Nano ESP32 the pin silkscreened `A4` is raw GPIO 11; the
 *     Arduino core does that remapping for you, ESP-IDF does not.
 *
 *   - GPIO 28 -- the valve pin in MAC_35_driver_v1.ino on the Mega -- exists
 *     on no ESP32 variant as a usable pin. Pin numbers never carry over
 *     between boards; always re-derive them from the schematic.
 * ----------------------------------------------------------------------- */
#define I2C_BUS_DEFAULT_SDA_GPIO   21
#define I2C_BUS_DEFAULT_SCL_GPIO   19
#define I2C_BUS_DEFAULT_FREQ_HZ    400000   /* 400 kHz: VL53L0X maximum */

/* Bring up the shared bus.
 *
 * `out_handle` is an out-parameter: you pass the address of your own handle
 * variable and this function writes into it. C functions return one value and
 * here that value is the error code, so real results come back via pointers.
 * You will see this pattern constantly in ESP-IDF.
 *
 *   i2c_master_bus_handle_t bus;
 *   ESP_ERROR_CHECK(i2c_bus_init(21, 22, 400000, &bus));
 *                                             ^ address-of
 *
 * Internal pull-ups are enabled, but they are weak (tens of kOhm). Most
 * breakouts -- including the GY-530 family -- carry their own stronger
 * pull-ups, which is what you actually want at 400 kHz. If you ever run long
 * wires with a bare chip, add external 4.7k resistors.
 */
esp_err_t i2c_bus_init(int sda_gpio, int scl_gpio, uint32_t freq_hz,
                       i2c_master_bus_handle_t *out_handle);

/* Probe a 7-bit address. Returns ESP_OK if a device acknowledges.
 *
 * Use this during bring-up before blaming the driver: if the VL53L0X does not
 * answer at 0x29, the problem is wiring or power, not software. */
esp_err_t i2c_bus_probe(i2c_master_bus_handle_t bus, uint8_t addr_7bit);

/* Log every address that responds, 0x08..0x77. The I2C equivalent of a
 * continuity check -- run it first whenever something is not working. */
void i2c_bus_scan(i2c_master_bus_handle_t bus);
