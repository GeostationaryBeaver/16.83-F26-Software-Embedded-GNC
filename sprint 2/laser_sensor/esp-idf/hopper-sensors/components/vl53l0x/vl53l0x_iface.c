/* ===========================================================================
 * vl53l0x_iface.c -- adapter that plugs the VL53L0X driver into the generic
 *                    sensor_iface vtable.
 *
 * WHY THIS IS A SEPARATE FILE
 * ---------------------------
 * vl53l0x.c knows about the chip and nothing else. This file knows about both
 * the chip driver and the generic interface, and translates between them. The
 * split means:
 *
 *   - you can use vl53l0x.c directly, without the vtable, if you ever want to
 *   - the generic interface can be replaced without touching the driver
 *   - the filtering policy (what counts as a good sample) lives in one place
 *
 * This is also where unit conversion happens: the chip speaks millimetres,
 * sensor_iface speaks metres. Converting at the boundary means no code
 * downstream has to remember which is which.
 * ===========================================================================
 */

#include "vl53l0x_iface.h"

#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "vl53l0x_if";

/* --- Median filter -------------------------------------------------------
 * VL53L0X failures are isolated gross outliers, not Gaussian noise: a dust
 * mote or a plume flicker gives you one wildly short reading. A median
 * rejects that completely; a moving average would smear it across the next
 * N samples instead.
 *
 * The cost is group delay: a window of N adds about N/2 samples of lag. At a
 * 33 ms budget a 5-tap window is ~80 ms of delay, which is a lot inside a
 * hopper control loop. Hence VL53L0X_FILTER_TAPS is small and configurable --
 * consider 3 taps with a 20 ms budget if phase margin is tight.
 * ---------------------------------------------------------------------- */

static uint16_t median_of(const uint16_t *src, uint8_t n)
{
    /* Copy before sorting: the caller's ring buffer must stay in insertion
     * order, so sorting it in place would corrupt the history. */
    uint16_t tmp[VL53L0X_FILTER_TAPS];
    memcpy(tmp, src, n * sizeof(tmp[0]));

    /* Insertion sort. O(n^2), but n <= 7 here, and for tiny n it beats
     * anything cleverer -- no recursion, no extra memory, and it is short
     * enough to verify by eye. */
    for (uint8_t i = 1; i < n; i++) {
        uint16_t key = tmp[i];
        int8_t   j   = (int8_t)i - 1;

        /* int8_t for j, not uint8_t: the loop relies on j reaching -1 to
         * terminate. An unsigned j would wrap to 255 and run off the end of
         * the array. A real bug that looks like a typo. */
        while (j >= 0 && tmp[j] > key) {
            tmp[j + 1] = tmp[j];
            j--;
        }
        tmp[j + 1] = key;
    }
    return tmp[n / 2];
}

/* --- vtable implementations ---------------------------------------------
 * Each takes sensor_t* and recovers its own context from self->ctx. The cast
 * is where type safety is lost: if ctx were ever set to a different struct,
 * this would compile cleanly and then corrupt memory. Keeping the cast inside
 * the driver that owns the ctx is what makes the pattern safe in practice.
 * ---------------------------------------------------------------------- */

static esp_err_t iface_init(sensor_t *self)
{
    vl53l0x_sensor_ctx_t *ctx = (vl53l0x_sensor_ctx_t *)self->ctx;

    esp_err_t err = vl53l0x_init(&ctx->dev, &ctx->cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[%s] init failed: %s", self->label,
                 esp_err_to_name(err));
        return err;
    }

    /* Reset filter state so a re-init does not mix old and new samples. */
    ctx->fill   = 0;
    ctx->idx    = 0;
    ctx->primed = false;

    ESP_LOGI(TAG, "[%s] ready", self->label);
    return ESP_OK;
}

static esp_err_t iface_start(sensor_t *self)
{
    vl53l0x_sensor_ctx_t *ctx = (vl53l0x_sensor_ctx_t *)self->ctx;

    /* Period 0 = back-to-back, paced by the timing budget. */
    return vl53l0x_start_continuous(&ctx->dev, 0);
}

static esp_err_t iface_poll(sensor_t *self, sensor_sample_t *out)
{
    vl53l0x_sensor_ctx_t *ctx = (vl53l0x_sensor_ctx_t *)self->ctx;

    vl53l0x_measurement_t m;
    esp_err_t err = vl53l0x_read_continuous(&ctx->dev, &m);

    /* NOT_FINISHED propagates unchanged: sensor_poll() in sensor_iface.h
     * knows not to count it as an error. */
    if (err != ESP_OK) return err;

    /* Stamp the time as soon as we have the data, before any filtering. The
     * true measurement instant is earlier still -- roughly half a timing
     * budget ago, since the reading is an average over the window -- so for
     * tight control you would subtract budget/2 here. Left explicit rather
     * than hidden. */
    out->timestamp_us = esp_timer_get_time();
    out->qty          = SENSOR_QTY_DISTANCE_M;
    out->status       = (uint8_t)m.status;

    if (!m.valid) {
        /* A rejected sample is a successful poll that produced no usable
         * number. Report it as such: the transaction worked, the datum did
         * not. The caller sees valid == false. */
        ESP_LOGD(TAG, "[%s] rejected: %s (raw %u mm)", self->label,
                 vl53l0x_status_str(m.status), m.raw_mm);
        out->valid = false;
        return ESP_OK;
    }

    /* Insert into the ring buffer. */
    ctx->ring[ctx->idx] = m.mm;
    ctx->idx = (uint8_t)((ctx->idx + 1) % VL53L0X_FILTER_TAPS);
    /* Modulo wrap rather than `if (idx >= N) idx = 0;` -- same effect, and
     * it cannot be got wrong by an off-by-one. */

    if (ctx->fill < VL53L0X_FILTER_TAPS) {
        ctx->fill++;
    }
    if (ctx->fill >= VL53L0X_FILTER_TAPS) {
        ctx->primed = true;
    }

    /* Before the window fills, report the unfiltered value rather than
     * nothing. Startup transparency beats a few samples of silence -- and the
     * alternative (returning NOT_FINISHED) would look like a sensor fault. */
    uint16_t mm = ctx->primed
                  ? median_of(ctx->ring, VL53L0X_FILTER_TAPS)
                  : m.mm;

    /* Convert to metres at the boundary. The `f` suffix on 1000.0f keeps this
     * single-precision: without it the literal is a double, the whole
     * expression promotes to double, and on the ESP32 -- whose FPU is
     * single-precision only -- that becomes a slow software emulation. */
    out->value[0] = (float)mm / 1000.0f;
    out->valid    = true;

    return ESP_OK;
}

static esp_err_t iface_stop(sensor_t *self)
{
    vl53l0x_sensor_ctx_t *ctx = (vl53l0x_sensor_ctx_t *)self->ctx;
    return vl53l0x_stop_continuous(&ctx->dev);
}

/* The vtable itself.
 *
 * `static const` and one instance for the whole program: every VL53L0X
 * sensor_t points at this same table, because the functions are identical --
 * only the ctx differs per instance. That is exactly how a C++ vtable works.
 *
 * const means it lives in flash, not RAM, and cannot be corrupted by a stray
 * write. Worth having for a table of function pointers. */
static const sensor_vtable_t vl53l0x_vtable = {
    .type_name = "vl53l0x",
    .init      = iface_init,
    .start     = iface_start,
    .poll      = iface_poll,
    .stop      = iface_stop,
};

esp_err_t vl53l0x_sensor_create(sensor_t *sensor, vl53l0x_sensor_ctx_t *ctx,
                                const vl53l0x_config_t *cfg,
                                const char *label)
{
    if (sensor == NULL || ctx == NULL || cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Zero both structs, then wire them together. Note what this function
     * does NOT do: allocate. The caller owns the storage (typically static or
     * on the stack in app_main), which is the right default on a
     * microcontroller -- no malloc means no fragmentation and no leak, and
     * memory use is known at link time. */
    *ctx = (vl53l0x_sensor_ctx_t){0};
    ctx->cfg = *cfg;                      /* struct copy: ctx owns its config */

    *sensor = (sensor_t){0};
    sensor->vtable = &vl53l0x_vtable;
    sensor->ctx    = ctx;                 /* the type-erased back-pointer */
    sensor->label  = label ? label : "vl53l0x";

    return ESP_OK;
}
