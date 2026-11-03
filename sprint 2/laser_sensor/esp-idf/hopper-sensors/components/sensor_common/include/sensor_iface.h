/* ===========================================================================
 * sensor_iface.h -- a uniform interface that every sensor driver implements.
 *
 * WHY THIS EXISTS
 * ---------------
 * You will have several sensors on this ESP32: a rangefinder, an IMU, load
 * cells, maybe a barometer. Without a common interface, main.c ends up as a
 * pile of per-sensor special cases:
 *
 *     vl53l0x_read(&tof, &mm);
 *     mpu6050_read_accel(&imu, &ax, &ay, &az);
 *     hx711_read(&cell, &counts);
 *     ... and so on, each with its own error handling
 *
 * With one, main.c loops over an array and does not care what is in it:
 *
 *     for (int i = 0; i < n; i++) sensor_poll(sensors[i], &sample[i]);
 *
 * Adding a sensor then means writing a driver and appending one line to the
 * table -- main.c does not change at all.
 *
 * HOW IT WORKS: THE VTABLE PATTERN
 * --------------------------------
 * C has no classes or virtual methods. The standard way to get the same effect
 * is a "vtable": a struct of function pointers. Each driver fills one in with
 * its own functions, and generic code calls through the pointers without
 * knowing which driver is on the other end.
 *
 * This is literally how C++ implements virtual methods under the hood -- you
 * are just writing the mechanism by hand instead of the compiler doing it.
 * ===========================================================================
 */

#pragma once   /* Include guard: process this header only once per .c file,
                * even if several headers all include it. Without it you get
                * "redefinition of struct" errors. */

#include <stddef.h>    /* NULL -- needed by the null checks in the wrappers
                       * below. Easy to omit, because many other headers drag
                       * it in indirectly and the code then compiles by luck. */
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* In C, a struct must be declared before you can mention it. sensor_t and the
 * vtable refer to each other (each vtable function takes a sensor_t*), so we
 * forward-declare sensor_t here to break the circular dependency. */
typedef struct sensor_s sensor_t;

/* ---------------------------------------------------------------------------
 * What a sensor measures.
 *
 * Tagging each sample with its physical quantity means a consumer can assert
 * it got what it expected, and that logs are self-describing. Units are in the
 * enum name on purpose: SI everywhere, converted at the driver boundary, so no
 * code downstream has to remember whether a number is mm or m.
 * ------------------------------------------------------------------------ */
typedef enum {
    SENSOR_QTY_NONE = 0,
    SENSOR_QTY_DISTANCE_M,      /* value[0] = metres                        */
    SENSOR_QTY_ACCEL_MPS2,      /* value[0..2] = x, y, z in m/s^2           */
    SENSOR_QTY_GYRO_RPS,        /* value[0..2] = x, y, z in rad/s           */
    SENSOR_QTY_FORCE_N,         /* value[0] = newtons (load cell)           */
    SENSOR_QTY_PRESSURE_PA,     /* value[0] = pascals                       */
    SENSOR_QTY_TEMPERATURE_C,   /* value[0] = degrees celsius               */
} sensor_qty_t;

/* ---------------------------------------------------------------------------
 * One reading from one sensor.
 *
 * Three fields deserve comment:
 *
 *  - `valid` is NOT redundant with the esp_err_t that poll() returns. A poll
 *    can succeed at the bus level (we talked to the chip fine) while the
 *    sample itself is garbage (the VL53L0X reported "signal too low"). The
 *    return code describes the transaction; `valid` describes the number.
 *
 *  - `timestamp_us` is captured by the driver as close to the measurement as
 *    it can manage, not by the consumer. A GNC loop needs to know WHEN a
 *    reading was taken, and the delay between measurement and your reading it
 *    is not constant.
 *
 *  - `value[3]` is a fixed array rather than a union of scalar/vector types.
 *    Scalars use value[0] and leave the rest zero. Wastes 8 bytes, removes an
 *    entire class of "read the wrong union member" bug. Worth it.
 * ------------------------------------------------------------------------ */
typedef struct {
    sensor_qty_t qty;
    bool         valid;
    int64_t      timestamp_us;   /* esp_timer_get_time() at measurement      */
    float        value[3];
    uint8_t      status;         /* driver-specific detail; 0 = nominal      */
} sensor_sample_t;

/* ---------------------------------------------------------------------------
 * The vtable: the set of operations every driver must provide.
 *
 * `esp_err_t (*poll)(sensor_t *self, sensor_sample_t *out)` reads as:
 *   a pointer, named poll, to a function taking (sensor_t*, sensor_sample_t*)
 *   and returning esp_err_t.
 *
 * The parenthesis around *poll are required. Without them --
 * `esp_err_t *poll(...)` -- you would be declaring a function that returns a
 * pointer, which is a completely different thing. This syntax is genuinely
 * awkward; you get used to reading it.
 *
 * `self` is the explicit version of C++'s implicit `this`. Every call passes
 * the sensor it is operating on.
 * ------------------------------------------------------------------------ */
typedef struct {
    const char *type_name;  /* e.g. "vl53l0x", for logs                     */

    /* Bring the hardware up: probe it, load configuration, calibrate.
     * Called once at startup. Required. */
    esp_err_t (*init)(sensor_t *self);

    /* Begin producing data (e.g. enter continuous ranging mode).
     * Optional -- leave NULL if init() is enough. */
    esp_err_t (*start)(sensor_t *self);

    /* Fetch the newest sample if one is available.
     *
     * MUST NOT BLOCK. If no new data is ready, return ESP_ERR_NOT_FINISHED
     * immediately rather than waiting. The whole point is that one slow
     * sensor cannot stall the control loop. Required. */
    esp_err_t (*poll)(sensor_t *self, sensor_sample_t *out);

    /* Stop producing data and/or power down. Optional. */
    esp_err_t (*stop)(sensor_t *self);
} sensor_vtable_t;

/* ---------------------------------------------------------------------------
 * A sensor instance.
 *
 * `ctx` is the type-erased pointer to driver-private state. The VL53L0X
 * adapter stores its own context struct (device handle, filter ring buffer,
 * calibration) there, and casts it back inside its own functions. Generic
 * code never looks inside it -- that is what makes this interface generic.
 *
 * `void *` means "pointer to something unspecified". It is C's escape hatch
 * for exactly this situation, and the cost is that the compiler can no longer
 * check the type for you: if you cast ctx to the wrong struct you get silent
 * memory corruption, not an error. Keep each ctx cast in the driver that owns
 * it and this stays safe.
 * ------------------------------------------------------------------------ */
struct sensor_s {
    const sensor_vtable_t *vtable;  /* shared, const, one per driver type   */
    void                  *ctx;     /* per-instance private state           */
    const char            *label;   /* human name, e.g. "altimeter"         */

    /* Health bookkeeping, maintained by the wrappers below. */
    bool     online;                /* init() succeeded                     */
    uint32_t sample_count;          /* valid samples delivered              */
    uint32_t error_count;           /* consecutive failures; 0 on success   */
};

/* ---------------------------------------------------------------------------
 * Generic wrappers.
 *
 * `static inline` in a header means: compile a private copy into each .c file
 * that includes this, and inline it at the call site. This is how you write a
 * zero-overhead helper in a header without a linker "multiple definition"
 * error. (`static` alone would also work but might not inline; `inline` alone
 * has subtle linkage rules that are easy to get wrong.)
 *
 * These exist so that the null checks and the health counters live in exactly
 * one place instead of being repeated in main.c for every sensor.
 * ------------------------------------------------------------------------ */

static inline esp_err_t sensor_init(sensor_t *s)
{
    if (s == NULL || s->vtable == NULL || s->vtable->init == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = s->vtable->init(s);
    s->online = (err == ESP_OK);
    return err;
}

static inline esp_err_t sensor_start(sensor_t *s)
{
    if (s == NULL || s->vtable == NULL) return ESP_ERR_INVALID_ARG;
    if (s->vtable->start == NULL) return ESP_OK;   /* optional: no-op */
    return s->vtable->start(s);
}

static inline esp_err_t sensor_poll(sensor_t *s, sensor_sample_t *out)
{
    if (s == NULL || out == NULL || s->vtable == NULL ||
        s->vtable->poll == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s->online) return ESP_ERR_INVALID_STATE;

    /* Clear the output first. If poll() returns early, the caller still sees
     * a well-defined sample with valid == false rather than stack garbage. */
    *out = (sensor_sample_t){0};

    esp_err_t err = s->vtable->poll(s, out);

    if (err == ESP_OK && out->valid) {
        s->sample_count++;
        s->error_count = 0;
    } else if (err != ESP_OK && err != ESP_ERR_NOT_FINISHED) {
        /* NOT_FINISHED just means "no new data yet" -- not an error. */
        s->error_count++;
    }
    return err;
}

static inline esp_err_t sensor_stop(sensor_t *s)
{
    if (s == NULL || s->vtable == NULL) return ESP_ERR_INVALID_ARG;
    if (s->vtable->stop == NULL) return ESP_OK;
    return s->vtable->stop(s);
}
