/* ===========================================================================
 * vl53l0x_iface.h -- sensor_iface adapter for the VL53L0X.
 *
 * Include this (rather than vl53l0x.h) when you want the sensor as a generic
 * sensor_t that main.c can poll alongside everything else.
 * ===========================================================================
 */

#pragma once

#include "sensor_iface.h"
#include "vl53l0x.h"

/* Median filter width. Must be ODD so there is a unique middle element.
 *
 * Trade-off: more taps reject outliers harder but add lag of roughly
 * taps/2 samples. At a 33 ms budget, 5 taps is about 80 ms of group delay.
 * For a hopper, consider 3 here with VL53L0X_PROFILE_HIGH_SPEED. */
#define VL53L0X_FILTER_TAPS   5

/* Per-instance private state.
 *
 * The caller allocates one of these per sensor and hands it to
 * vl53l0x_sensor_create, which stores it in sensor_t.ctx. It is in the header
 * only so that callers can declare one statically -- treat the fields as
 * private to vl53l0x_iface.c. */
typedef struct {
    vl53l0x_t        dev;      /* the low-level driver instance */
    vl53l0x_config_t cfg;      /* kept so init() can be re-run  */

    /* Median filter ring buffer. */
    uint16_t ring[VL53L0X_FILTER_TAPS];
    uint8_t  fill;             /* samples collected so far      */
    uint8_t  idx;              /* next write position           */
    bool     primed;           /* window full; filter is valid  */
} vl53l0x_sensor_ctx_t;

/* Wire a sensor_t to a VL53L0X. Does not touch the hardware -- call
 * sensor_init() for that. Allocates nothing: you own both structs.
 *
 *     static sensor_t            tof;
 *     static vl53l0x_sensor_ctx_t tof_ctx;
 *
 *     vl53l0x_config_t cfg;
 *     vl53l0x_config_default(&cfg, bus);
 *     cfg.xshut_gpio = 5;
 *     vl53l0x_sensor_create(&tof, &tof_ctx, &cfg, "altimeter");
 *     sensor_init(&tof);
 *     sensor_start(&tof);
 */
esp_err_t vl53l0x_sensor_create(sensor_t *sensor, vl53l0x_sensor_ctx_t *ctx,
                                const vl53l0x_config_t *cfg,
                                const char *label);
