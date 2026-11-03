/* ===========================================================================
 * vl53l0x.c -- register-level VL53L0X driver for ESP-IDF v5.2+.
 *
 * READING THIS FILE
 * -----------------
 * It is long, and most of the length is the init sequence. That sequence is
 * not something you can derive -- ST never published the register map, so the
 * magic numbers below come from ST's own API and Pololu's clean-room
 * reimplementation of it. Where a write has no explanation, it is because
 * there is no public explanation. Those are marked "opaque".
 *
 * The parts worth actually understanding are:
 *
 *   - reg_write8 / reg_read8 etc.  how an I2C register access works
 *   - the timing-budget arithmetic  why budget trades against range
 *   - vl53l0x_read_continuous      the non-blocking read pattern
 *
 * The rest you can treat as a fixed incantation.
 * ===========================================================================
 */

#include "vl53l0x.h"

#include <inttypes.h>        /* PRIu32 etc: portable printf macros for the
                             * fixed-width types. ESP_LOGx uses printf format
                             * specifiers, and the strictly-correct specifier
                             * for uint32_t is "%" PRIu32 rather than %u. */
#include <string.h>          /* memcpy */
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"       /* esp_timer_get_time: microsecond clock */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"   /* vTaskDelay, pdMS_TO_TICKS */

static const char *TAG = "vl53l0x";

/* ===========================================================================
 * Register addresses.
 *
 * Only a handful of these are documented by ST; the rest are known from the
 * reverse-engineered API. Names follow ST's convention so that you can match
 * them against other VL53L0X code you find.
 * ======================================================================== */

#define REG_SYSRANGE_START                          0x00
#define REG_SYSTEM_SEQUENCE_CONFIG                  0x01
#define REG_SYSTEM_INTERMEASUREMENT_PERIOD          0x04
#define REG_SYSTEM_INTERRUPT_CONFIG_GPIO            0x0A
#define REG_SYSTEM_INTERRUPT_CLEAR                  0x0B
#define REG_RESULT_INTERRUPT_STATUS                 0x13
#define REG_RESULT_RANGE_STATUS                     0x14
#define REG_MSRC_CONFIG_TIMEOUT_MACROP              0x46
#define REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW      0x47
#define REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH     0x48
#define REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD     0x4E
#define REG_DYNAMIC_SPAD_REF_EN_START_OFFSET        0x4F
#define REG_PRE_RANGE_CONFIG_VCSEL_PERIOD           0x50
#define REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI      0x51
#define REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW        0x56
#define REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH       0x57
#define REG_MSRC_CONFIG_CONTROL                     0x60
#define REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN   0x44
#define REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD         0x70
#define REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI    0x71
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0        0xB0
#define REG_GPIO_HV_MUX_ACTIVE_HIGH                 0x84
#define REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV        0x89
#define REG_ALGO_PHASECAL_CONFIG_TIMEOUT            0x30
#define REG_ALGO_PHASECAL_LIM                       0x30
#define REG_IDENTIFICATION_MODEL_ID                 0xC0

/* I2C transaction timeout in ms. Generous: the chip answers in microseconds,
 * this only guards against a wiring fault hanging the task forever. */
#define I2C_TIMEOUT_MS        100

/* How long to wait for a measurement or calibration before giving up. */
#define RANGE_TIMEOUT_MS      500

/* Forward declarations of file-local helpers.
 *
 * C has no auto-generated prototypes -- unlike an .ino, where the Arduino IDE
 * writes these for you. A function must be declared before it is called, so
 * either you define everything in dependency order, or you declare up front
 * like this. The second is far easier to maintain. */
static esp_err_t reg_write8(vl53l0x_t *dev, uint8_t reg, uint8_t val);
static esp_err_t reg_write16(vl53l0x_t *dev, uint8_t reg, uint16_t val);
static esp_err_t reg_read8(vl53l0x_t *dev, uint8_t reg, uint8_t *val);
static esp_err_t reg_read16(vl53l0x_t *dev, uint8_t reg, uint16_t *val);
static esp_err_t reg_read_multi(vl53l0x_t *dev, uint8_t reg,
                                uint8_t *buf, size_t len);
static esp_err_t load_tuning_settings(vl53l0x_t *dev);
static esp_err_t get_spad_info(vl53l0x_t *dev, uint8_t *count,
                               bool *is_aperture);
static esp_err_t perform_ref_calibration(vl53l0x_t *dev, uint8_t vhv_init_byte);
static esp_err_t get_vcsel_pulse_period(vl53l0x_t *dev,
                                        vl53l0x_vcsel_period_type_t type,
                                        uint8_t *period_pclks);
static esp_err_t get_timing_budget(vl53l0x_t *dev, uint32_t *budget_us);
static void      decode_measurement(vl53l0x_t *dev, uint8_t status_byte,
                                    uint16_t raw_mm,
                                    vl53l0x_measurement_t *out);

/* ===========================================================================
 * SECTION 1 -- I2C register access
 *
 * Everything else is built on these four functions. The VL53L0X uses the
 * ordinary "write the register address, then read or write data" convention.
 * ======================================================================== */

/* Write one byte to one register.
 *
 * The wire sequence is [reg_addr][value], so we build a 2-byte buffer and
 * send it as a single transaction. `i2c_master_transmit` handles the start
 * condition, address byte, ACK checking and stop condition for us. */
static esp_err_t reg_write8(vl53l0x_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(dev->dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

/* Write a 16-bit value, big-endian (most significant byte first).
 *
 * The VL53L0X is big-endian but the ESP32 is little-endian, so we cannot just
 * memcpy a uint16_t -- the bytes would come out reversed. Shifting explicitly
 * is endianness-independent and always correct.
 *
 * `val >> 8` takes the high byte; `val & 0xFF` masks off everything but the
 * low byte. The cast to uint8_t would truncate anyway, but masking documents
 * the intent. */
static esp_err_t reg_write16(vl53l0x_t *dev, uint8_t reg, uint16_t val)
{
    uint8_t buf[3] = { reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    return i2c_master_transmit(dev->dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

/* Read one byte from one register.
 *
 * This is a combined write-then-read transaction: send the register address,
 * then without releasing the bus, read the data back.
 * `i2c_master_transmit_receive` does exactly that, which matters because
 * another task must not be able to interleave a transaction between the two
 * halves. */
static esp_err_t reg_read8(vl53l0x_t *dev, uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(dev->dev, &reg, 1, val, 1,
                                       I2C_TIMEOUT_MS);
}

static esp_err_t reg_read16(vl53l0x_t *dev, uint8_t reg, uint16_t *val)
{
    uint8_t buf[2];
    esp_err_t err = i2c_master_transmit_receive(dev->dev, &reg, 1, buf,
                                                sizeof(buf), I2C_TIMEOUT_MS);
    if (err != ESP_OK) return err;

    /* Reassemble big-endian: high byte first. The cast on buf[0] matters --
     * without it the shift happens in `int` width and, while harmless here,
     * the habit avoids real bugs with wider types. */
    *val = ((uint16_t)buf[0] << 8) | buf[1];
    return ESP_OK;
}

/* Read several consecutive registers in one transaction. The chip
 * auto-increments its internal pointer, so this is both legal and much
 * faster than `len` separate reads. */
static esp_err_t reg_read_multi(vl53l0x_t *dev, uint8_t reg,
                                uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(dev->dev, &reg, 1, buf, len,
                                       I2C_TIMEOUT_MS);
}

/* ===========================================================================
 * SECTION 2 -- timing arithmetic
 *
 * These conversions are the one genuinely interesting piece of maths in the
 * driver. The chip expresses timeouts in "macro periods" (MCLKs), which are
 * derived from the VCSEL pulse period. That coupling is why changing a VCSEL
 * period silently changes your timing budget, and why you must re-apply the
 * budget afterwards.
 * ======================================================================== */

/* The chip stores VCSEL periods as (period / 2) - 1. These two macros convert
 * between register and real units.
 *
 * Macros, not functions, because they are used in contexts where a constant
 * expression is wanted. Note every parameter is parenthesised -- without that,
 * `VCSEL_DECODE(a + b)` would expand wrongly. This is the classic macro trap. */
#define VCSEL_DECODE(reg_val)      (((reg_val) + 1) << 1)
#define VCSEL_ENCODE(period_pclks) ((((period_pclks) >> 1) - 1))

/* One macro period in nanoseconds, given a VCSEL period in PLL clocks.
 * 2304 and 1655 are from ST's API; the + 500 / 1000 is rounding to the
 * nearest rather than truncating. */
static uint32_t calc_macro_period_ns(uint8_t vcsel_period_pclks)
{
    return ((((uint32_t)2304 * vcsel_period_pclks * 1655) + 500) / 1000);
}

/* The chip's timeout register format is "(LSByte * 2^MSByte) + 1" -- a crude
 * floating-point encoding that buys a wide range in 16 bits. */
static uint16_t decode_timeout(uint16_t reg_val)
{
    return (uint16_t)((reg_val & 0x00FF) << ((reg_val & 0xFF00) >> 8)) + 1;
}

static uint16_t encode_timeout(uint32_t timeout_mclks)
{
    if (timeout_mclks == 0) return 0;

    uint32_t ls_byte = timeout_mclks - 1;
    uint16_t ms_byte = 0;

    /* Shift right until the value fits in 8 bits, counting the shifts. That
     * count becomes the exponent. */
    while ((ls_byte & 0xFFFFFF00) > 0) {
        ls_byte >>= 1;
        ms_byte++;
    }
    return (uint16_t)((ms_byte << 8) | (ls_byte & 0xFF));
}

static uint32_t timeout_mclks_to_us(uint16_t timeout_mclks,
                                    uint8_t vcsel_period_pclks)
{
    uint32_t macro_ns = calc_macro_period_ns(vcsel_period_pclks);
    return (((uint32_t)timeout_mclks * macro_ns) + 500) / 1000;
}

static uint32_t timeout_us_to_mclks(uint32_t timeout_us,
                                    uint8_t vcsel_period_pclks)
{
    uint32_t macro_ns = calc_macro_period_ns(vcsel_period_pclks);
    return (((timeout_us * 1000) + (macro_ns / 2)) / macro_ns);
}

/* --- Sequence step bookkeeping ------------------------------------------
 * A measurement runs as a sequence of enabled steps. SYSTEM_SEQUENCE_CONFIG
 * is a bitmask of which are on, and each has its own timeout. To set a total
 * budget we must subtract every other step's cost and give the remainder to
 * the final range step.
 * ---------------------------------------------------------------------- */

typedef struct {
    bool tcc, dss, msrc, pre_range, final_range;
} seq_enables_t;

typedef struct {
    uint8_t  pre_range_vcsel_period_pclks;
    uint8_t  final_range_vcsel_period_pclks;
    uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
    uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
} seq_timeouts_t;

static esp_err_t get_seq_enables(vl53l0x_t *dev, seq_enables_t *en)
{
    uint8_t cfg;
    esp_err_t err = reg_read8(dev, REG_SYSTEM_SEQUENCE_CONFIG, &cfg);
    if (err != ESP_OK) return err;

    /* Extract single bits: shift the bit you want down to position 0, then
     * mask everything else off. Standard bit-field idiom in C. */
    en->tcc         = (cfg >> 4) & 0x01;
    en->dss         = (cfg >> 3) & 0x01;
    en->msrc        = (cfg >> 2) & 0x01;
    en->pre_range   = (cfg >> 6) & 0x01;
    en->final_range = (cfg >> 7) & 0x01;
    return ESP_OK;
}

static esp_err_t get_seq_timeouts(vl53l0x_t *dev, const seq_enables_t *en,
                                  seq_timeouts_t *t)
{
    esp_err_t err;
    uint16_t raw;

    err = get_vcsel_pulse_period(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE,
                                 &t->pre_range_vcsel_period_pclks);
    if (err != ESP_OK) return err;

    uint8_t msrc;
    err = reg_read8(dev, REG_MSRC_CONFIG_TIMEOUT_MACROP, &msrc);
    if (err != ESP_OK) return err;
    t->msrc_dss_tcc_mclks = msrc + 1;
    t->msrc_dss_tcc_us = timeout_mclks_to_us(t->msrc_dss_tcc_mclks,
                                             t->pre_range_vcsel_period_pclks);

    err = reg_read16(dev, REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, &raw);
    if (err != ESP_OK) return err;
    t->pre_range_mclks = decode_timeout(raw);
    t->pre_range_us = timeout_mclks_to_us(t->pre_range_mclks,
                                          t->pre_range_vcsel_period_pclks);

    err = get_vcsel_pulse_period(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE,
                                 &t->final_range_vcsel_period_pclks);
    if (err != ESP_OK) return err;

    err = reg_read16(dev, REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, &raw);
    if (err != ESP_OK) return err;
    t->final_range_mclks = decode_timeout(raw);

    /* The final-range timeout register holds pre-range + final-range when
     * pre-range is enabled, so subtract to get the final-range part alone. */
    if (en->pre_range) {
        if (t->final_range_mclks > t->pre_range_mclks) {
            t->final_range_mclks -= t->pre_range_mclks;
        } else {
            t->final_range_mclks = 0;
        }
    }
    t->final_range_us = timeout_mclks_to_us(
        t->final_range_mclks, t->final_range_vcsel_period_pclks);

    return ESP_OK;
}

/* Fixed per-step overheads in microseconds, from ST's API. */
#define OVERHEAD_START       1910
#define OVERHEAD_END          960
#define OVERHEAD_MSRC         660
#define OVERHEAD_TCC          590
#define OVERHEAD_DSS          690
#define OVERHEAD_PRE_RANGE    660
#define OVERHEAD_FINAL_RANGE  550
#define MIN_TIMING_BUDGET_US 20000

esp_err_t vl53l0x_set_timing_budget(vl53l0x_t *dev, uint32_t budget_us)
{
    if (dev == NULL) return ESP_ERR_INVALID_ARG;
    if (budget_us < MIN_TIMING_BUDGET_US) {
        ESP_LOGW(TAG, "budget %" PRIu32 " us below %d us minimum, clamping",
                 budget_us, MIN_TIMING_BUDGET_US);
        budget_us = MIN_TIMING_BUDGET_US;
    }

    seq_enables_t en;
    seq_timeouts_t t;
    esp_err_t err;

    err = get_seq_enables(dev, &en);
    if (err != ESP_OK) return err;
    err = get_seq_timeouts(dev, &en, &t);
    if (err != ESP_OK) return err;

    /* Add up everything that is not the final range step. */
    uint32_t used = OVERHEAD_START + OVERHEAD_END;

    if (en.tcc)  used += t.msrc_dss_tcc_us + OVERHEAD_TCC;
    if (en.dss)  used += 2 * (t.msrc_dss_tcc_us + OVERHEAD_DSS);
    else if (en.msrc) used += t.msrc_dss_tcc_us + OVERHEAD_MSRC;
    if (en.pre_range) used += t.pre_range_us + OVERHEAD_PRE_RANGE;

    if (!en.final_range) {
        /* No final range step: nothing to apportion the remainder to. */
        dev->timing_budget_us = budget_us;
        return ESP_OK;
    }

    used += OVERHEAD_FINAL_RANGE;
    if (used > budget_us) {
        ESP_LOGE(TAG, "budget %" PRIu32 " us too small; overheads need %"
                 PRIu32 " us", budget_us, used);
        return ESP_ERR_INVALID_ARG;
    }

    /* Whatever is left over becomes the final range integration time -- this
     * is the knob that actually buys range and precision. */
    uint32_t final_us = budget_us - used;
    uint32_t final_mclks = timeout_us_to_mclks(
        final_us, t.final_range_vcsel_period_pclks);

    /* Write back in the combined form the register expects. */
    if (en.pre_range) final_mclks += t.pre_range_mclks;

    err = reg_write16(dev, REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                      encode_timeout(final_mclks));
    if (err != ESP_OK) return err;

    dev->timing_budget_us = budget_us;
    return ESP_OK;
}

static esp_err_t get_timing_budget(vl53l0x_t *dev, uint32_t *budget_us)
{
    seq_enables_t en;
    seq_timeouts_t t;
    esp_err_t err;

    err = get_seq_enables(dev, &en);
    if (err != ESP_OK) return err;
    err = get_seq_timeouts(dev, &en, &t);
    if (err != ESP_OK) return err;

    uint32_t budget = OVERHEAD_START + OVERHEAD_END;
    if (en.tcc)  budget += t.msrc_dss_tcc_us + OVERHEAD_TCC;
    if (en.dss)  budget += 2 * (t.msrc_dss_tcc_us + OVERHEAD_DSS);
    else if (en.msrc) budget += t.msrc_dss_tcc_us + OVERHEAD_MSRC;
    if (en.pre_range)   budget += t.pre_range_us + OVERHEAD_PRE_RANGE;
    if (en.final_range) budget += t.final_range_us + OVERHEAD_FINAL_RANGE;

    *budget_us = budget;
    return ESP_OK;
}

/* ===========================================================================
 * SECTION 3 -- VCSEL pulse period
 *
 * Longer pulse periods put more light on target, extending range. The valid
 * values are constrained, and each period requires its own matched set of
 * phase-check thresholds -- which is why this function has a switch full of
 * magic numbers rather than just writing the period.
 * ======================================================================== */

static esp_err_t get_vcsel_pulse_period(vl53l0x_t *dev,
                                        vl53l0x_vcsel_period_type_t type,
                                        uint8_t *period_pclks)
{
    uint8_t reg = (type == VL53L0X_VCSEL_PERIOD_PRE_RANGE)
                  ? REG_PRE_RANGE_CONFIG_VCSEL_PERIOD
                  : REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD;
    uint8_t val;
    esp_err_t err = reg_read8(dev, reg, &val);
    if (err != ESP_OK) return err;

    *period_pclks = VCSEL_DECODE(val);
    return ESP_OK;
}

esp_err_t vl53l0x_set_vcsel_pulse_period(vl53l0x_t *dev,
                                         vl53l0x_vcsel_period_type_t type,
                                         uint8_t period_pclks)
{
    if (dev == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t encoded = VCSEL_ENCODE(period_pclks);
    esp_err_t err;

    seq_enables_t en;
    seq_timeouts_t t;
    err = get_seq_enables(dev, &en);
    if (err != ESP_OK) return err;
    err = get_seq_timeouts(dev, &en, &t);
    if (err != ESP_OK) return err;

    if (type == VL53L0X_VCSEL_PERIOD_PRE_RANGE) {
        /* Each valid period needs its own phase-check high threshold. These
         * pairings are from ST's API -- do not interpolate between them. */
        switch (period_pclks) {
        case 12:
            err = reg_write8(dev, REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
            break;
        case 14:
            err = reg_write8(dev, REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
            break;
        case 16:
            err = reg_write8(dev, REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
            break;
        case 18:
            err = reg_write8(dev, REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
            break;
        default:
            ESP_LOGE(TAG, "pre-range period %u invalid (need 12/14/16/18)",
                     period_pclks);
            return ESP_ERR_INVALID_ARG;
        }
        if (err != ESP_OK) return err;

        err = reg_write8(dev, REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
        if (err != ESP_OK) return err;

        err = reg_write8(dev, REG_PRE_RANGE_CONFIG_VCSEL_PERIOD, encoded);
        if (err != ESP_OK) return err;

        /* The timeout was expressed in macro periods, and we just changed how
         * long a macro period is. Re-encode it so the real duration is
         * preserved. Skip this and your timing budget silently shifts. */
        err = reg_write16(dev, REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                          encode_timeout(timeout_us_to_mclks(t.pre_range_us,
                                                             period_pclks)));
        if (err != ESP_OK) return err;

        err = reg_write8(dev, REG_MSRC_CONFIG_TIMEOUT_MACROP,
                         (uint8_t)((timeout_us_to_mclks(t.msrc_dss_tcc_us,
                                                        period_pclks) > 256)
                                   ? 255
                                   : timeout_us_to_mclks(t.msrc_dss_tcc_us,
                                                         period_pclks) - 1));
        if (err != ESP_OK) return err;

    } else {  /* final range */
        /* Each final-range period needs a matched set of FOUR registers:
         * phase high/low, phase-cal limit, and the width thresholds. */
        switch (period_pclks) {
        case 8:
            err = reg_write8(dev, REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
            if (err != ESP_OK) return err;
            err = reg_write8(dev, REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
            if (err != ESP_OK) return err;
            err = reg_write8(dev, REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 - 0x80 + 0x02, 0x0C);
            if (err != ESP_OK) return err;
            err = reg_write8(dev, 0xFF, 0x01);
            if (err != ESP_OK) return err;
            err = reg_write8(dev, REG_ALGO_PHASECAL_LIM, 0x30);
            if (err != ESP_OK) return err;
            err = reg_write8(dev, 0xFF, 0x00);
            break;
        case 10:
            err = reg_write8(dev, REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
            if (err != ESP_OK) return err;
            err = reg_write8(dev, REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
            if (err != ESP_OK) return err;
            err = reg_write8(dev, 0x32, 0x09);
            if (err != ESP_OK) return err;
            err = reg_write8(dev, 0xFF, 0x01);
            if (err != ESP_OK) return err;
            err = reg_write8(dev, REG_ALGO_PHASECAL_LIM, 0x20);
            if (err != ESP_OK) return err;
            err = reg_write8(dev, 0xFF, 0x00);
            break;
        case 12:
            err = reg_write8(dev, REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
            if (err != ESP_OK) return err;
            err = reg_write8(dev, REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
            if (err != ESP_OK) return err;
            err = reg_write8(dev, 0x32, 0x08);
            if (err != ESP_OK) return err;
            err = reg_write8(dev, 0xFF, 0x01);
            if (err != ESP_OK) return err;
            err = reg_write8(dev, REG_ALGO_PHASECAL_LIM, 0x20);
            if (err != ESP_OK) return err;
            err = reg_write8(dev, 0xFF, 0x00);
            break;
        case 14:
            err = reg_write8(dev, REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
            if (err != ESP_OK) return err;
            err = reg_write8(dev, REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
            if (err != ESP_OK) return err;
            err = reg_write8(dev, 0x32, 0x07);
            if (err != ESP_OK) return err;
            err = reg_write8(dev, 0xFF, 0x01);
            if (err != ESP_OK) return err;
            err = reg_write8(dev, REG_ALGO_PHASECAL_LIM, 0x20);
            if (err != ESP_OK) return err;
            err = reg_write8(dev, 0xFF, 0x00);
            break;
        default:
            ESP_LOGE(TAG, "final-range period %u invalid (need 8/10/12/14)",
                     period_pclks);
            return ESP_ERR_INVALID_ARG;
        }
        if (err != ESP_OK) return err;

        err = reg_write8(dev, REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD, encoded);
        if (err != ESP_OK) return err;

        uint32_t new_final_mclks =
            timeout_us_to_mclks(t.final_range_us, period_pclks);
        if (en.pre_range) new_final_mclks += t.pre_range_mclks;

        err = reg_write16(dev, REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                          encode_timeout(new_final_mclks));
        if (err != ESP_OK) return err;
    }

    /* Changing a period perturbs the budget; restore what the user asked for. */
    return vl53l0x_set_timing_budget(dev, dev->timing_budget_us);
}

esp_err_t vl53l0x_set_signal_rate_limit(vl53l0x_t *dev, float mcps)
{
    if (dev == NULL) return ESP_ERR_INVALID_ARG;
    if (mcps < 0.0f || mcps > 511.99f) return ESP_ERR_INVALID_ARG;

    /* The register is 9.7 fixed-point: 9 integer bits, 7 fractional. So
     * multiply by 2^7 = 128 and truncate. Fixed-point is common in sensor
     * registers -- no FPU needed on the chip side. */
    return reg_write16(dev, REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN,
                       (uint16_t)(mcps * (1 << 7)));
}

esp_err_t vl53l0x_set_profile(vl53l0x_t *dev, vl53l0x_profile_t profile)
{
    if (dev == NULL || !dev->initialised) return ESP_ERR_INVALID_STATE;

    esp_err_t err;
    float    signal_limit;
    uint8_t  pre_period, final_period;
    uint32_t budget_us;

    /* Values from section 9 of the notes. */
    switch (profile) {
    case VL53L0X_PROFILE_HIGH_SPEED:
        signal_limit = 0.25f; pre_period = 14; final_period = 10;
        budget_us = 20000; break;
    case VL53L0X_PROFILE_LONG_RANGE:
        signal_limit = 0.10f; pre_period = 18; final_period = 14;
        budget_us = 33000; break;
    case VL53L0X_PROFILE_HIGH_ACCURACY:
        signal_limit = 0.25f; pre_period = 14; final_period = 10;
        budget_us = 200000; break;
    case VL53L0X_PROFILE_DEFAULT:
    default:
        signal_limit = 0.25f; pre_period = 14; final_period = 10;
        budget_us = 33000; break;
    }

    /* Order matters: signal limit, then periods, then budget last -- because
     * setting a period recomputes the budget from dev->timing_budget_us. */
    err = vl53l0x_set_signal_rate_limit(dev, signal_limit);
    if (err != ESP_OK) return err;

    err = vl53l0x_set_vcsel_pulse_period(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE,
                                         pre_period);
    if (err != ESP_OK) return err;

    err = vl53l0x_set_vcsel_pulse_period(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE,
                                         final_period);
    if (err != ESP_OK) return err;

    return vl53l0x_set_timing_budget(dev, budget_us);
}

/* ===========================================================================
 * SECTION 4 -- initialisation
 *
 * This is the opaque part. The sequence below must happen in this order; it
 * is ST's data-init + static-init + reference-calibration flow. Comments
 * explain the structure, not the individual magic writes, because the
 * individual writes have no public explanation.
 * ======================================================================== */

/* The tuning blob: ~80 register writes that configure the ranging algorithm's
 * internals. Entirely undocumented. Stored as {register, value} pairs so the
 * loop below can just walk it.
 *
 * `static const` means: file-local, and in flash rather than RAM. On a
 * microcontroller that distinction matters -- this is ~160 bytes you do not
 * want to spend from your 320 KB of RAM. */
static const uint8_t tuning_settings[][2] = {
    {0xFF, 0x01}, {0x00, 0x00}, {0xFF, 0x00}, {0x09, 0x00},
    {0x10, 0x00}, {0x11, 0x00}, {0x24, 0x01}, {0x25, 0xFF},
    {0x75, 0x00}, {0xFF, 0x01}, {0x4E, 0x2C}, {0x48, 0x00},
    {0x30, 0x20}, {0xFF, 0x00}, {0x30, 0x09}, {0x54, 0x00},
    {0x31, 0x04}, {0x32, 0x03}, {0x40, 0x83}, {0x46, 0x25},
    {0x60, 0x00}, {0x27, 0x00}, {0x50, 0x06}, {0x51, 0x00},
    {0x52, 0x96}, {0x56, 0x08}, {0x57, 0x30}, {0x61, 0x00},
    {0x62, 0x00}, {0x64, 0x00}, {0x65, 0x00}, {0x66, 0xA0},
    {0xFF, 0x01}, {0x22, 0x32}, {0x47, 0x14}, {0x49, 0xFF},
    {0x4A, 0x00}, {0xFF, 0x00}, {0x7A, 0x0A}, {0x7B, 0x00},
    {0x78, 0x21}, {0xFF, 0x01}, {0x23, 0x34}, {0x42, 0x00},
    {0x44, 0xFF}, {0x45, 0x26}, {0x46, 0x05}, {0x40, 0x40},
    {0x0E, 0x06}, {0x20, 0x1A}, {0x43, 0x40}, {0xFF, 0x00},
    {0x34, 0x03}, {0x35, 0x44}, {0xFF, 0x01}, {0x31, 0x04},
    {0x4B, 0x09}, {0x4C, 0x05}, {0x4D, 0x04}, {0xFF, 0x00},
    {0x44, 0x00}, {0x45, 0x20}, {0x47, 0x08}, {0x48, 0x28},
    {0x67, 0x00}, {0x70, 0x04}, {0x71, 0x01}, {0x72, 0xFE},
    {0x76, 0x00}, {0x77, 0x00}, {0xFF, 0x01}, {0x0D, 0x01},
    {0xFF, 0x00}, {0x80, 0x01}, {0x01, 0xF8}, {0xFF, 0x01},
    {0x8E, 0x01}, {0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00},
};

static esp_err_t load_tuning_settings(vl53l0x_t *dev)
{
    /* sizeof(array) / sizeof(element) is C's idiom for array length. There is
     * no .length in C -- and note this only works where the real array is
     * visible, not on a pointer parameter, which is a classic trap. */
    size_t n = sizeof(tuning_settings) / sizeof(tuning_settings[0]);

    for (size_t i = 0; i < n; i++) {
        esp_err_t err = reg_write8(dev, tuning_settings[i][0],
                                        tuning_settings[i][1]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "tuning write %u failed", (unsigned)i);
            return err;
        }
    }
    return ESP_OK;
}

/* Read how many reference SPADs this particular die has, and whether they are
 * the aperture type. Every chip is slightly different -- this is per-unit
 * factory data burned into its internal NVM. */
static esp_err_t get_spad_info(vl53l0x_t *dev, uint8_t *count,
                               bool *is_aperture)
{
    esp_err_t err;
    uint8_t tmp;

    /* An undocumented unlock dance to expose the NVM registers. */
    const uint8_t pre[][2] = {
        {0x80, 0x01}, {0xFF, 0x01}, {0x00, 0x00}, {0xFF, 0x06},
    };
    for (size_t i = 0; i < 4; i++) {
        err = reg_write8(dev, pre[i][0], pre[i][1]);
        if (err != ESP_OK) return err;
    }

    err = reg_read8(dev, 0x83, &tmp);
    if (err != ESP_OK) return err;
    err = reg_write8(dev, 0x83, tmp | 0x04);   /* set a bit, preserve others */
    if (err != ESP_OK) return err;

    const uint8_t mid[][2] = {
        {0xFF, 0x07}, {0x81, 0x01}, {0x80, 0x01},
        {0x94, 0x6B}, {0x83, 0x00},
    };
    for (size_t i = 0; i < 5; i++) {
        err = reg_write8(dev, mid[i][0], mid[i][1]);
        if (err != ESP_OK) return err;
    }

    /* Poll for the NVM read to complete. Always bound a poll loop with a
     * timeout -- an unbounded `while (!ready)` on a faulty sensor hangs the
     * task, and on ESP32 the watchdog then reboots the chip. */
    int64_t deadline = esp_timer_get_time() + (RANGE_TIMEOUT_MS * 1000);
    do {
        err = reg_read8(dev, 0x83, &tmp);
        if (err != ESP_OK) return err;
        if (tmp != 0x00) break;
        if (esp_timer_get_time() > deadline) {
            ESP_LOGE(TAG, "SPAD info read timed out");
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1));   /* yield, do not spin */
    } while (1);

    err = reg_write8(dev, 0x83, 0x01);
    if (err != ESP_OK) return err;
    err = reg_read8(dev, 0x92, &tmp);
    if (err != ESP_OK) return err;

    *count       = tmp & 0x7F;          /* low 7 bits = SPAD count */
    *is_aperture = (tmp >> 7) & 0x01;   /* top bit = type          */

    /* Re-lock. */
    const uint8_t post[][2] = {
        {0x81, 0x00}, {0xFF, 0x06},
    };
    for (size_t i = 0; i < 2; i++) {
        err = reg_write8(dev, post[i][0], post[i][1]);
        if (err != ESP_OK) return err;
    }
    err = reg_read8(dev, 0x83, &tmp);
    if (err != ESP_OK) return err;
    err = reg_write8(dev, 0x83, tmp & ~0x04);   /* clear the bit we set */
    if (err != ESP_OK) return err;

    const uint8_t fin[][2] = {
        {0xFF, 0x01}, {0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00},
    };
    for (size_t i = 0; i < 4; i++) {
        err = reg_write8(dev, fin[i][0], fin[i][1]);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

/* Run VHV or phase calibration. `vhv_init_byte` selects which:
 * 0x01 = VHV (SPAD bias trim), 0x02 = phase.
 * Both must run once at init, VHV first. */
static esp_err_t perform_ref_calibration(vl53l0x_t *dev, uint8_t vhv_init_byte)
{
    esp_err_t err = reg_write8(dev, REG_SYSRANGE_START, 0x01 | vhv_init_byte);
    if (err != ESP_OK) return err;

    int64_t deadline = esp_timer_get_time() + (RANGE_TIMEOUT_MS * 1000);
    uint8_t status;
    do {
        err = reg_read8(dev, REG_RESULT_INTERRUPT_STATUS, &status);
        if (err != ESP_OK) return err;
        if ((status & 0x07) != 0) break;
        if (esp_timer_get_time() > deadline) {
            ESP_LOGE(TAG, "ref calibration 0x%02X timed out", vhv_init_byte);
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    } while (1);

    err = reg_write8(dev, REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    if (err != ESP_OK) return err;

    return reg_write8(dev, REG_SYSRANGE_START, 0x00);
}

/* --- Public init --------------------------------------------------------- */

void vl53l0x_config_default(vl53l0x_config_t *cfg,
                            i2c_master_bus_handle_t bus)
{
    if (cfg == NULL) return;

    /* Zero the whole struct first, then set what we care about. This is why
     * callers should always use this function: a field added to the config in
     * future gets a defined value instead of stack garbage. */
    *cfg = (vl53l0x_config_t){0};

    cfg->bus         = bus;
    cfg->i2c_addr    = VL53L0X_I2C_ADDR_DEFAULT;
    cfg->i2c_freq_hz = 400000;
    cfg->xshut_gpio  = -1;                      /* not wired by default */
    cfg->profile     = VL53L0X_PROFILE_DEFAULT;
    cfg->offset_mm   = 0;
}

esp_err_t vl53l0x_init(vl53l0x_t *dev, const vl53l0x_config_t *cfg)
{
    if (dev == NULL || cfg == NULL) return ESP_ERR_INVALID_ARG;

    *dev = (vl53l0x_t){0};
    dev->xshut_gpio = cfg->xshut_gpio;
    dev->offset_mm  = cfg->offset_mm;

    esp_err_t err;
    uint8_t   tmp;

    /* --- Step 0: hardware reset via XSHUT, if wired -----------------------
     * This matters more on ESP32 than you would expect. The MCU reboots on
     * every flash upload and on any watchdog trip, but the sensor keeps its
     * power and its state. A sensor left mid-measurement in continuous mode
     * will not initialise cleanly. XSHUT gives us a known starting point.
     *
     * XSHUT is ACTIVE LOW: pull low to reset, release high to run. */
    if (dev->xshut_gpio >= 0) {
        gpio_config_t io = {
            .pin_bit_mask = (1ULL << dev->xshut_gpio),  /* 1ULL: must be 64-bit */
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        err = gpio_config(&io);
        if (err != ESP_OK) return err;

        gpio_set_level(dev->xshut_gpio, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(dev->xshut_gpio, 1);
        vTaskDelay(pdMS_TO_TICKS(10));   /* boot is ~1.2 ms; be generous */
    }

    /* --- Step 1: register as a device on the shared bus ------------------
     * In the v5.2+ I2C API each device gets its own handle, carrying its
     * address and clock speed. Several devices coexist on one bus this way,
     * and the driver layer serialises access for us. */
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = cfg->i2c_addr,
        .scl_speed_hz    = cfg->i2c_freq_hz,
    };
    err = i2c_master_bus_add_device(cfg->bus, &dev_cfg, &dev->dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "bus_add_device failed: %s", esp_err_to_name(err));
        return err;
    }

    /* --- Step 2: confirm it is really a VL53L0X -------------------------
     * Do this before anything else. If the model ID is wrong, every
     * subsequent write is going somewhere unknown -- far better to fail here
     * with a clear message than to debug mysterious behaviour later. */
    err = reg_read8(dev, REG_IDENTIFICATION_MODEL_ID, &tmp);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "no response at 0x%02X: %s -- check wiring and power",
                 cfg->i2c_addr, esp_err_to_name(err));
        goto fail;
    }
    if (tmp != VL53L0X_MODEL_ID) {
        ESP_LOGE(TAG, "model ID 0x%02X, expected 0x%02X -- not a VL53L0X?",
                 tmp, VL53L0X_MODEL_ID);
        err = ESP_ERR_NOT_SUPPORTED;
        goto fail;
    }
    ESP_LOGI(TAG, "found VL53L0X at 0x%02X", cfg->i2c_addr);

    /* --- Step 3: 2.8 V mode ---------------------------------------------
     * The die runs at 2.8 V internally. This tells it the I2C pads are at
     * 2.8 V logic rather than 1.8 V. Breakout boards all use 2.8 V. */
    err = reg_read8(dev, REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, &tmp);
    if (err != ESP_OK) goto fail;
    err = reg_write8(dev, REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, tmp | 0x01);
    if (err != ESP_OK) goto fail;

    /* --- Step 4: set I2C standard mode and capture stop_variable ---------
     * stop_variable is opaque: ST's code reads it here and writes it back
     * before every measurement. Nobody outside ST knows what it holds. It is
     * not optional -- omit it and ranging misbehaves. */
    const uint8_t pre_stop[][2] = {
        {0x88, 0x00}, {0x80, 0x01}, {0xFF, 0x01}, {0x00, 0x00},
    };
    for (size_t i = 0; i < 4; i++) {
        err = reg_write8(dev, pre_stop[i][0], pre_stop[i][1]);
        if (err != ESP_OK) goto fail;
    }

    err = reg_read8(dev, 0x91, &dev->stop_variable);
    if (err != ESP_OK) goto fail;

    const uint8_t post_stop[][2] = {
        {0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00},
    };
    for (size_t i = 0; i < 3; i++) {
        err = reg_write8(dev, post_stop[i][0], post_stop[i][1]);
        if (err != ESP_OK) goto fail;
    }

    /* --- Step 5: disable SIGNAL_RATE_MSRC and SIGNAL_RATE_PRE_RANGE limit
     * checks, then set a default signal rate limit.
     *
     * `& ~0x12` clears bits 1 and 4 while leaving every other bit alone:
     * ~0x12 is 0xED, and ANDing with it zeroes exactly those two positions.
     * This read-modify-write idiom is everywhere in embedded code -- plain
     * assignment would clobber the bits you did not mean to touch. */
    err = reg_read8(dev, REG_MSRC_CONFIG_CONTROL, &tmp);
    if (err != ESP_OK) goto fail;
    err = reg_write8(dev, REG_MSRC_CONFIG_CONTROL, tmp | 0x12);
    if (err != ESP_OK) goto fail;

    err = vl53l0x_set_signal_rate_limit(dev, 0.25f);
    if (err != ESP_OK) goto fail;

    /* Enable all sequence steps. 0xFF = everything on. */
    err = reg_write8(dev, REG_SYSTEM_SEQUENCE_CONFIG, 0xFF);
    if (err != ESP_OK) goto fail;

    /* --- Step 6: reference SPAD management -------------------------------
     * Read this die's factory SPAD data, then enable exactly that many
     * reference SPADs starting at the right offset. Get this wrong and the
     * chip ranges, but badly. */
    uint8_t spad_count;
    bool    spad_is_aperture;
    err = get_spad_info(dev, &spad_count, &spad_is_aperture);
    if (err != ESP_OK) goto fail;

    uint8_t ref_spad_map[6];
    err = reg_read_multi(dev, REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0,
                         ref_spad_map, sizeof(ref_spad_map));
    if (err != ESP_OK) goto fail;

    const uint8_t spad_pre[][2] = {
        {0xFF, 0x01},
        {REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00},
        {REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C},
        {0xFF, 0x00},
        {REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 - 0x80 + 0x4F, 0xB4},
    };
    for (size_t i = 0; i < 5; i++) {
        err = reg_write8(dev, spad_pre[i][0], spad_pre[i][1]);
        if (err != ESP_OK) goto fail;
    }

    /* Aperture-type SPADs start at index 12; non-aperture at 0. */
    uint8_t first_spad = spad_is_aperture ? 12 : 0;
    uint8_t enabled    = 0;

    /* Walk all 48 reference SPADs (6 bytes x 8 bits) and enable the first
     * `spad_count` of the right type, clearing the rest. */
    for (uint8_t i = 0; i < 48; i++) {
        uint8_t byte_idx = i / 8;
        uint8_t bit_idx  = i % 8;

        if (i < first_spad || enabled == spad_count) {
            ref_spad_map[byte_idx] &= ~(1 << bit_idx);      /* clear bit */
        } else if ((ref_spad_map[byte_idx] >> bit_idx) & 0x01) {
            enabled++;                                       /* already set */
        }
    }

    /* Write the modified map back. There is no reg_write_multi helper, so
     * build the buffer by hand: [reg][6 data bytes]. */
    uint8_t spad_tx[1 + sizeof(ref_spad_map)];
    spad_tx[0] = REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0;
    memcpy(&spad_tx[1], ref_spad_map, sizeof(ref_spad_map));
    err = i2c_master_transmit(dev->dev, spad_tx, sizeof(spad_tx),
                              I2C_TIMEOUT_MS);
    if (err != ESP_OK) goto fail;

    /* --- Step 7: the undocumented tuning blob --------------------------- */
    err = load_tuning_settings(dev);
    if (err != ESP_OK) goto fail;

    /* --- Step 8: interrupt configuration --------------------------------
     * Configure GPIO1 to assert on "new sample ready". We set this up even if
     * you never wire GPIO1, because RESULT_INTERRUPT_STATUS -- which the
     * polling path reads -- is driven by the same mechanism. */
    err = reg_write8(dev, REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    if (err != ESP_OK) goto fail;

    /* Make the interrupt active LOW. The pin is open-drain, so it needs a
     * pull-up -- on the breakout, or in your gpio_config. */
    err = reg_read8(dev, REG_GPIO_HV_MUX_ACTIVE_HIGH, &tmp);
    if (err != ESP_OK) goto fail;
    err = reg_write8(dev, REG_GPIO_HV_MUX_ACTIVE_HIGH, tmp & ~0x10);
    if (err != ESP_OK) goto fail;

    err = reg_write8(dev, REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    if (err != ESP_OK) goto fail;

    /* --- Step 9: capture the budget the tuning blob left us -------------- */
    err = get_timing_budget(dev, &dev->timing_budget_us);
    if (err != ESP_OK) goto fail;

    /* --- Step 10: reference calibration ---------------------------------
     * VHV re-trims the SPAD breakdown bias; phase calibrates the timing
     * reference. Each needs only its own sequence step enabled, hence the
     * SEQUENCE_CONFIG writes around them. Order is not negotiable. */
    err = reg_write8(dev, REG_SYSTEM_SEQUENCE_CONFIG, 0x01);
    if (err != ESP_OK) goto fail;
    err = perform_ref_calibration(dev, 0x40);      /* VHV */
    if (err != ESP_OK) goto fail;

    err = reg_write8(dev, REG_SYSTEM_SEQUENCE_CONFIG, 0x02);
    if (err != ESP_OK) goto fail;
    err = perform_ref_calibration(dev, 0x00);      /* phase */
    if (err != ESP_OK) goto fail;

    /* Restore all steps for normal ranging. */
    err = reg_write8(dev, REG_SYSTEM_SEQUENCE_CONFIG, 0xE8);
    if (err != ESP_OK) goto fail;

    dev->initialised = true;

    /* --- Step 11: apply the requested profile --------------------------- */
    err = vl53l0x_set_profile(dev, cfg->profile);
    if (err != ESP_OK) {
        dev->initialised = false;
        goto fail;
    }

    ESP_LOGI(TAG, "initialised: budget %" PRIu32 " us, offset %d mm, "
                  "%u ref SPADs%s",
             dev->timing_budget_us, dev->offset_mm, spad_count,
             spad_is_aperture ? " (aperture)" : "");
    return ESP_OK;

/* A single cleanup path. `goto fail` on error is idiomatic C for this -- it is
 * the closest thing the language has to try/finally. Without it you would
 * repeat the device-handle removal at twenty return sites, and eventually
 * forget one and leak the handle. This is one of the few genuinely good uses
 * of goto. */
fail:
    if (dev->dev != NULL) {
        i2c_master_bus_rm_device(dev->dev);
        dev->dev = NULL;
    }
    return err;
}

esp_err_t vl53l0x_deinit(vl53l0x_t *dev)
{
    if (dev == NULL) return ESP_ERR_INVALID_ARG;

    if (dev->continuous) {
        vl53l0x_stop_continuous(dev);
    }
    if (dev->dev != NULL) {
        i2c_master_bus_rm_device(dev->dev);
        dev->dev = NULL;
    }
    dev->initialised = false;
    return ESP_OK;
}

/* ===========================================================================
 * SECTION 5 -- measurement decoding
 * ======================================================================== */

/* Turn a raw status byte and distance into a validated measurement.
 *
 * Two independent gates, and both matter:
 *   1. the chip's own status code  -- did IT trust the reading?
 *   2. our sanity window          -- is it physically plausible for us?
 *
 * The chip will hand you a confident-looking number with a sigma-fail flag on
 * it. Checking only the distance, or only the status, lets bad data through. */
static void decode_measurement(vl53l0x_t *dev, uint8_t status_byte,
                               uint16_t raw_mm, vl53l0x_measurement_t *out)
{
    /* Status lives in bits 3:7, so shift down by 3 and mask to 5 bits. */
    uint8_t code = (status_byte & 0x78) >> 3;

    /* Map the chip's internal codes onto our enum. These mappings are from
     * ST's API; codes not listed are lumped into UNKNOWN. */
    switch (code) {
    case 11: out->status = VL53L0X_RANGE_VALID;       break;
    case 1:
    case 2:
    case 3:  out->status = VL53L0X_RANGE_HW_FAIL;     break;
    case 6:
    case 9:  out->status = VL53L0X_RANGE_PHASE_FAIL;  break;
    case 8:
    case 10: out->status = VL53L0X_RANGE_SIGMA_FAIL;  break;
    case 4:  out->status = VL53L0X_RANGE_SIGNAL_FAIL; break;
    case 5:  out->status = VL53L0X_RANGE_MIN_CLIP;    break;
    default: out->status = VL53L0X_RANGE_UNKNOWN;     break;
    }

    out->raw_mm = raw_mm;

    /* Apply the offset, in signed arithmetic, then clamp at zero.
     *
     * The cast to int32_t is load-bearing: raw_mm is unsigned, and
     * `raw_mm + offset_mm` with a negative offset would wrap around to a huge
     * positive number rather than going negative. Mixing signed and unsigned
     * is one of C's sharpest edges. */
    int32_t corrected = (int32_t)raw_mm + dev->offset_mm;
    if (corrected < 0) corrected = 0;
    out->mm = (uint16_t)corrected;

    out->valid = (out->status == VL53L0X_RANGE_VALID) &&
                 (raw_mm < VL53L0X_OUT_OF_RANGE_MM) &&
                 (out->mm >= VL53L0X_SANE_MIN_MM) &&
                 (out->mm <= VL53L0X_SANE_MAX_MM);
}

const char *vl53l0x_status_str(vl53l0x_range_status_t status)
{
    /* Returning a string literal is safe: literals have static storage
     * duration and live for the whole program. Returning a local char[] here
     * would be a dangling pointer -- a classic C bug. */
    switch (status) {
    case VL53L0X_RANGE_VALID:       return "valid";
    case VL53L0X_RANGE_SIGMA_FAIL:  return "sigma too high";
    case VL53L0X_RANGE_SIGNAL_FAIL: return "signal too low";
    case VL53L0X_RANGE_MIN_CLIP:    return "below minimum range";
    case VL53L0X_RANGE_PHASE_FAIL:  return "phase/wraparound fail";
    case VL53L0X_RANGE_HW_FAIL:     return "hardware fail";
    default:                        return "unknown";
    }
}

/* ===========================================================================
 * SECTION 6 -- ranging
 * ======================================================================== */

/* Re-send stop_variable. ST's API does this before every measurement; the
 * reason is undocumented. Treat it as required. */
static esp_err_t write_stop_variable(vl53l0x_t *dev)
{
    const uint8_t seq[][2] = {
        {0x80, 0x01}, {0xFF, 0x01}, {0x00, 0x00},
    };
    for (size_t i = 0; i < 3; i++) {
        esp_err_t err = reg_write8(dev, seq[i][0], seq[i][1]);
        if (err != ESP_OK) return err;
    }

    esp_err_t err = reg_write8(dev, 0x91, dev->stop_variable);
    if (err != ESP_OK) return err;

    const uint8_t seq2[][2] = {
        {0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00},
    };
    for (size_t i = 0; i < 3; i++) {
        err = reg_write8(dev, seq2[i][0], seq2[i][1]);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

esp_err_t vl53l0x_read_single(vl53l0x_t *dev, vl53l0x_measurement_t *out)
{
    if (dev == NULL || out == NULL) return ESP_ERR_INVALID_ARG;
    if (!dev->initialised) return ESP_ERR_INVALID_STATE;
    if (dev->continuous) {
        ESP_LOGE(TAG, "single read while in continuous mode");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = write_stop_variable(dev);
    if (err != ESP_OK) return err;

    /* Kick off one measurement. */
    err = reg_write8(dev, REG_SYSRANGE_START, 0x01);
    if (err != ESP_OK) return err;

    /* Wait for the start bit to clear, meaning the measurement began. */
    int64_t deadline = esp_timer_get_time() + (RANGE_TIMEOUT_MS * 1000);
    uint8_t tmp;
    do {
        err = reg_read8(dev, REG_SYSRANGE_START, &tmp);
        if (err != ESP_OK) return err;
        if ((tmp & 0x01) == 0) break;
        if (esp_timer_get_time() > deadline) return ESP_ERR_TIMEOUT;
        vTaskDelay(pdMS_TO_TICKS(1));
    } while (1);

    /* Then wait for data-ready. Note vTaskDelay rather than a busy spin: it
     * yields the CPU to other tasks instead of burning it, and keeps the
     * watchdog fed. A bare `while(1);` here would reboot the chip. */
    do {
        err = reg_read8(dev, REG_RESULT_INTERRUPT_STATUS, &tmp);
        if (err != ESP_OK) return err;
        if ((tmp & 0x07) != 0) break;
        if (esp_timer_get_time() > deadline) return ESP_ERR_TIMEOUT;
        vTaskDelay(pdMS_TO_TICKS(1));
    } while (1);

    /* Read status and distance. RESULT_RANGE_STATUS is at 0x14, and the
     * 16-bit distance sits at 0x14 + 10 = 0x1E. */
    uint8_t  status_byte;
    uint16_t raw_mm;
    err = reg_read8(dev, REG_RESULT_RANGE_STATUS, &status_byte);
    if (err != ESP_OK) return err;
    err = reg_read16(dev, REG_RESULT_RANGE_STATUS + 10, &raw_mm);
    if (err != ESP_OK) return err;

    /* Clear the interrupt so the next measurement can assert it. */
    err = reg_write8(dev, REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    if (err != ESP_OK) return err;

    decode_measurement(dev, status_byte, raw_mm, out);
    return ESP_OK;
}

esp_err_t vl53l0x_start_continuous(vl53l0x_t *dev, uint32_t period_ms)
{
    if (dev == NULL) return ESP_ERR_INVALID_ARG;
    if (!dev->initialised) return ESP_ERR_INVALID_STATE;

    esp_err_t err = write_stop_variable(dev);
    if (err != ESP_OK) return err;

    if (period_ms != 0) {
        /* Timed mode: pace measurements at a fixed interval.
         *
         * The period register counts in units of the oscillator calibration
         * value, so read that and scale. If OSC_CALIBRATE_VAL is zero (some
         * chips) skip the scaling rather than multiplying by zero -- which
         * would set a zero period and silently give you continuous-as-fast-
         * as-possible instead. */
        uint16_t osc_cal;
        err = reg_read16(dev, 0xF8, &osc_cal);
        if (err != ESP_OK) return err;

        uint32_t period = period_ms;
        if (osc_cal != 0) period *= osc_cal;

        err = reg_write16(dev, REG_SYSTEM_INTERMEASUREMENT_PERIOD,
                          (uint16_t)period);
        if (err != ESP_OK) return err;

        err = reg_write8(dev, REG_SYSRANGE_START, 0x04);  /* timed */
    } else {
        /* Back-to-back: the sensor re-ranges as fast as the timing budget
         * allows. This is what a control loop usually wants. */
        err = reg_write8(dev, REG_SYSRANGE_START, 0x02);
    }
    if (err != ESP_OK) return err;

    dev->continuous = true;
    ESP_LOGI(TAG, "continuous mode started (period %" PRIu32 " ms)", period_ms);
    return ESP_OK;
}

esp_err_t vl53l0x_stop_continuous(vl53l0x_t *dev)
{
    if (dev == NULL) return ESP_ERR_INVALID_ARG;

    esp_err_t err = reg_write8(dev, REG_SYSRANGE_START, 0x01);  /* stop */
    if (err != ESP_OK) return err;

    const uint8_t seq[][2] = {
        {0xFF, 0x01}, {0x00, 0x00}, {0x91, 0x00}, {0x00, 0x01}, {0xFF, 0x00},
    };
    for (size_t i = 0; i < 5; i++) {
        err = reg_write8(dev, seq[i][0], seq[i][1]);
        if (err != ESP_OK) return err;
    }

    dev->continuous = false;
    return ESP_OK;
}

esp_err_t vl53l0x_data_ready(vl53l0x_t *dev, bool *ready)
{
    if (dev == NULL || ready == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t status;
    esp_err_t err = reg_read8(dev, REG_RESULT_INTERRUPT_STATUS, &status);
    if (err != ESP_OK) return err;

    *ready = ((status & 0x07) != 0);
    return ESP_OK;
}

/* The function a control loop calls. Note what it does NOT do: block, retry,
 * or sleep. If there is no new sample it says so and returns immediately, so
 * the caller can get on with the rest of its work. */
esp_err_t vl53l0x_read_continuous(vl53l0x_t *dev, vl53l0x_measurement_t *out)
{
    if (dev == NULL || out == NULL) return ESP_ERR_INVALID_ARG;
    if (!dev->continuous) return ESP_ERR_INVALID_STATE;

    bool ready;
    esp_err_t err = vl53l0x_data_ready(dev, &ready);
    if (err != ESP_OK) return err;

    if (!ready) {
        /* Not an error -- the sensor is simply still integrating. The caller
         * should treat this as "try again next iteration". */
        return ESP_ERR_NOT_FINISHED;
    }

    uint8_t  status_byte;
    uint16_t raw_mm;
    err = reg_read8(dev, REG_RESULT_RANGE_STATUS, &status_byte);
    if (err != ESP_OK) return err;
    err = reg_read16(dev, REG_RESULT_RANGE_STATUS + 10, &raw_mm);
    if (err != ESP_OK) return err;

    /* Clearing the interrupt is mandatory, not housekeeping: it re-arms the
     * data-ready mechanism. Skip it and data_ready() stays true forever while
     * you re-read the same stale sample. */
    err = reg_write8(dev, REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    if (err != ESP_OK) return err;

    decode_measurement(dev, status_byte, raw_mm, out);
    return ESP_OK;
}

/* ===========================================================================
 * SECTION 7 -- offset calibration
 * ======================================================================== */

esp_err_t vl53l0x_calibrate_offset(vl53l0x_t *dev, uint16_t true_distance_mm,
                                   uint8_t samples, int16_t *out_offset_mm)
{
    if (dev == NULL || out_offset_mm == NULL || samples == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!dev->initialised) return ESP_ERR_INVALID_STATE;

    bool was_continuous = dev->continuous;
    if (was_continuous) {
        esp_err_t e = vl53l0x_stop_continuous(dev);
        if (e != ESP_OK) return e;
    }

    /* Zero the offset while measuring, or we would calibrate against an
     * already-corrected value and the result would be meaningless. */
    int16_t saved_offset = dev->offset_mm;
    dev->offset_mm = 0;

    ESP_LOGI(TAG, "calibrating against %u mm target, %u samples...",
             true_distance_mm, samples);

    /* uint32_t accumulator, not uint16_t: 50 samples x up to 8190 mm would
     * overflow 16 bits at about 8 samples. Accumulator overflow is a quiet,
     * easy mistake to make. */
    uint32_t sum = 0;
    uint8_t  good = 0;

    for (uint8_t i = 0; i < samples; i++) {
        vl53l0x_measurement_t m;
        esp_err_t e = vl53l0x_read_single(dev, &m);
        if (e == ESP_OK && m.valid) {
            sum += m.raw_mm;
            good++;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    dev->offset_mm = saved_offset;   /* restore before any early return */

    if (was_continuous) {
        vl53l0x_start_continuous(dev, 0);
    }

    /* Demand a solid majority of good samples. If more than a third failed,
     * the setup is wrong -- bad target, bad lighting, wrong distance -- and
     * an offset computed from the rest would bake that error in permanently. */
    if (good < (samples * 2 / 3)) {
        ESP_LOGE(TAG, "only %u/%u samples valid; check target and lighting",
                 good, samples);
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* Rounded mean: adding half the divisor before dividing rounds to nearest
     * instead of truncating toward zero. */
    uint16_t mean = (uint16_t)((sum + good / 2) / good);

    *out_offset_mm = (int16_t)true_distance_mm - (int16_t)mean;

    ESP_LOGI(TAG, "mean %u mm over %u samples -> offset %+d mm",
             mean, good, *out_offset_mm);

    /* NOTE: crosstalk calibration is not implemented. You only need it if the
     * sensor sits behind cover glass, where internal reflection biases every
     * reading short. The procedure is in section 10 of the notes; implementing
     * it means porting ST's VL53L0X_PerformXTalkCalibration. */

    return ESP_OK;
}
