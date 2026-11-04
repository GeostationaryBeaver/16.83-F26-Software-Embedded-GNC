/* ===========================================================================
 * vl53l0x.h -- register-level driver for the ST VL53L0X time-of-flight
 *              rangefinder, for ESP-IDF v5.2+.
 *
 * SCOPE
 * -----
 * This is the low-level driver: it owns the I2C conversation with the chip.
 * It knows nothing about the sensor_iface vtable -- that adapter lives in
 * vl53l0x_iface.c. The split means you can use this driver directly if you
 * ever want to, and that the generic plumbing is replaceable.
 *
 * PROVENANCE, AND WHAT THIS DELIBERATELY OMITS
 * --------------------------------------------
 * ST never published the VL53L0X register map. The sequence here is derived
 * from the publicly documented subset plus Pololu's clean-room
 * reimplementation of ST's API, and covers:
 *
 *     model-ID check, 2V8 mode, SPAD reference management, the ~80-register
 *     undocumented tuning blob, interrupt configuration, VHV/phase
 *     calibration, timing budget, VCSEL pulse periods, single + continuous
 *     ranging, offset correction.
 *
 * It does NOT implement crosstalk compensation (only needed behind cover
 * glass) or multi-sensor XSHUT address reassignment. Both are noted at the
 * point where they would go.
 *
 * If you need either, the honest answer is to port ST's official API (it is
 * C, so it is idiomatic here) rather than extend this.
 * ===========================================================================
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

/* --- Addresses and identity ---------------------------------------------- */

/* The 7-bit address. The datasheet says 0x52, which is THIS value shifted
 * left one bit to make room for the read/write bit. ESP-IDF wants the 7-bit
 * form. This discrepancy confuses almost everyone once. */
#define VL53L0X_I2C_ADDR_DEFAULT   0x29

/* Register 0xC0 must read back exactly this. If it does not, you are either
 * not talking to a VL53L0X or not talking to anything. */
#define VL53L0X_MODEL_ID           0xEE

/* --- Sentinel and limit values ------------------------------------------- */

/* The chip reports these when it cannot find a target. They are not
 * distances -- never feed them to a control loop. */
#define VL53L0X_OUT_OF_RANGE_MM    8190

/* Below ~30 mm the internal crosstalk return competes with the real one. */
#define VL53L0X_SANE_MIN_MM        30

/* Be honest about the ceiling: ~2 m in darkness, ~0.8-1.2 m in daylight.
 * 1600 mm is a defensible default for indoor use. */
#define VL53L0X_SANE_MAX_MM        1600

/* --- Range status codes -------------------------------------------------- */

/* Decoded from bits 3:7 of RESULT_RANGE_STATUS. Anything but
 * VL53L0X_RANGE_VALID means the distance is untrustworthy even if the I2C
 * read itself succeeded -- this is exactly why sensor_sample_t carries a
 * `valid` flag separate from its esp_err_t. */
typedef enum {
    VL53L0X_RANGE_VALID          = 0,
    VL53L0X_RANGE_SIGMA_FAIL     = 1,  /* noise too high to trust the peak  */
    VL53L0X_RANGE_SIGNAL_FAIL    = 2,  /* too little returned light         */
    VL53L0X_RANGE_MIN_CLIP       = 3,  /* target closer than measurable     */
    VL53L0X_RANGE_PHASE_FAIL     = 4,  /* wrap-around ambiguity             */
    VL53L0X_RANGE_HW_FAIL        = 5,  /* hardware fault                    */
    VL53L0X_RANGE_UNKNOWN        = 255,
} vl53l0x_range_status_t;

/* --- VCSEL pulse period selector ---------------------------------------- */

typedef enum {
    VL53L0X_VCSEL_PERIOD_PRE_RANGE,
    VL53L0X_VCSEL_PERIOD_FINAL_RANGE,
} vl53l0x_vcsel_period_type_t;

/* --- Range profiles ------------------------------------------------------
 * Convenience bundles of (timing budget, VCSEL periods, signal rate limit).
 * See section 9 of the accompanying notes for the numbers behind each.
 * ---------------------------------------------------------------------- */
typedef enum {
    VL53L0X_PROFILE_DEFAULT,        /* 33 ms, ~1.2 m, balanced             */
    VL53L0X_PROFILE_HIGH_SPEED,     /* 20 ms, 50 Hz, shortest range        */
    VL53L0X_PROFILE_LONG_RANGE,     /* 33 ms, ~2 m, ambient-sensitive      */
    VL53L0X_PROFILE_HIGH_ACCURACY,  /* 200 ms, 5 Hz, best precision        */
} vl53l0x_profile_t;

/* --- Configuration passed to vl53l0x_init() ----------------------------- */

typedef struct {
    i2c_master_bus_handle_t bus;         /* from i2c_bus_init()             */
    uint8_t                 i2c_addr;    /* usually VL53L0X_I2C_ADDR_DEFAULT */
    uint32_t                i2c_freq_hz; /* 400000 max                      */

    int                     xshut_gpio;  /* -1 if not wired                 */
    vl53l0x_profile_t       profile;

    /* Added to every raw reading. Determine it once per unit: put a flat
     * white target at exactly 100 mm, average ~50 readings, and set this to
     * (100 - measured). A raw offset of 10-20 mm is normal. See section 10. */
    int16_t                 offset_mm;
} vl53l0x_config_t;

/* --- Driver state -------------------------------------------------------
 * This struct is the C substitute for a C++ object. In Arduino you wrote:
 *
 *     VL53L0X sensor;          sensor.init();
 *
 * Here you write:
 *
 *     vl53l0x_t sensor;        vl53l0x_init(&sensor, &cfg);
 *
 * Same thing: the struct holds what the object's members would have held,
 * and every function takes a pointer to it as its first argument. The `&`
 * means "address of". Inside those functions, fields are reached with `->`
 * instead of `.` because what they hold is a pointer.
 *
 * Treat these fields as private -- read them if you like, but write them
 * only through the functions below. The one exception is `offset_mm`, which
 * is safe to change at runtime.
 * ---------------------------------------------------------------------- */
typedef struct {
    i2c_master_dev_handle_t dev;            /* this device on the bus       */
    uint8_t                 stop_variable;  /* opaque; captured during init
                                             * and re-sent before each
                                             * measurement. ST's own code
                                             * does this and does not say
                                             * why. Do not remove it. */
    uint32_t                timing_budget_us;
    int                     xshut_gpio;
    int16_t                 offset_mm;
    bool                    continuous;     /* continuous mode is running   */
    bool                    initialised;
} vl53l0x_t;

/* --- One measurement ---------------------------------------------------- */

typedef struct {
    uint16_t              raw_mm;      /* straight from the chip            */
    uint16_t             mm;           /* raw + offset, what you want       */
    vl53l0x_range_status_t status;
    bool                  valid;       /* status OK and within sane window  */
} vl53l0x_measurement_t;

/* =======================================================================
 * API
 * ======================================================================= */

/* Fill a config with sensible defaults. Call this first, then override only
 * what you need -- that way new fields added later get safe values instead of
 * whatever was on the stack.
 *
 *     vl53l0x_config_t cfg;
 *     vl53l0x_config_default(&cfg, bus);
 *     cfg.profile = VL53L0X_PROFILE_LONG_RANGE;
 */
void vl53l0x_config_default(vl53l0x_config_t *cfg,
                            i2c_master_bus_handle_t bus);

/* Hardware-reset (if XSHUT is wired), probe, and configure the chip.
 * Takes ~50 ms, mostly calibration. Must succeed before anything else. */
esp_err_t vl53l0x_init(vl53l0x_t *dev, const vl53l0x_config_t *cfg);

/* Release the I2C device handle. */
esp_err_t vl53l0x_deinit(vl53l0x_t *dev);

/* --- Single-shot ranging ------------------------------------------------ */

/* Trigger one measurement and BLOCK until it completes (~33 ms at the
 * default budget). Fine for bring-up and calibration; wrong inside a control
 * loop -- use continuous mode there. */
esp_err_t vl53l0x_read_single(vl53l0x_t *dev, vl53l0x_measurement_t *out);

/* --- Continuous ranging (what the GNC loop should use) ------------------ */

/* Start back-to-back ranging. `period_ms` of 0 means "as fast as the timing
 * budget allows"; a larger value paces it slower. */
esp_err_t vl53l0x_start_continuous(vl53l0x_t *dev, uint32_t period_ms);

esp_err_t vl53l0x_stop_continuous(vl53l0x_t *dev);

/* Non-blocking: is a new sample ready? */
esp_err_t vl53l0x_data_ready(vl53l0x_t *dev, bool *ready);

/* Non-blocking read. Returns ESP_ERR_NOT_FINISHED if no new sample is ready,
 * which is normal and not an error -- just try again next loop iteration.
 * Also clears the interrupt, which re-arms the GPIO1 output. */
esp_err_t vl53l0x_read_continuous(vl53l0x_t *dev, vl53l0x_measurement_t *out);

/* --- Tuning -------------------------------------------------------------- */

/* Microseconds; 20000 minimum. Longer = more range and precision, more lag.
 * Remember the effective measurement delay is roughly half the budget. */
esp_err_t vl53l0x_set_timing_budget(vl53l0x_t *dev, uint32_t budget_us);

/* Megacounts per second. Default 0.25; lower (e.g. 0.1) accepts weaker
 * returns, extending range at the cost of more bad samples. */
esp_err_t vl53l0x_set_signal_rate_limit(vl53l0x_t *dev, float mcps);

/* Pre-range accepts 12/14/16/18; final-range 8/10/12/14. Longer periods
 * reach further. Changing these silently alters the timing budget, so
 * re-apply the budget afterwards (vl53l0x_set_profile does this for you). */
esp_err_t vl53l0x_set_vcsel_pulse_period(vl53l0x_t *dev,
                                         vl53l0x_vcsel_period_type_t type,
                                         uint8_t period_pclks);

/* Apply a profile bundle in the correct order. */
esp_err_t vl53l0x_set_profile(vl53l0x_t *dev, vl53l0x_profile_t profile);

/* --- Helpers ------------------------------------------------------------- */

/* Static string for a status code, for logs. Never returns NULL. */
const char *vl53l0x_status_str(vl53l0x_range_status_t status);

/* Average `samples` single-shot readings and return the offset that would
 * centre them on `true_distance_mm`. Blocks for samples * budget.
 *
 *     int16_t off;
 *     vl53l0x_calibrate_offset(&dev, 100, 50, &off);  // target at 100 mm
 *     dev.offset_mm = off;                             // then persist it
 *
 * Store the result in NVS rather than recompiling it. */
esp_err_t vl53l0x_calibrate_offset(vl53l0x_t *dev, uint16_t true_distance_mm,
                                   uint8_t samples, int16_t *out_offset_mm);
