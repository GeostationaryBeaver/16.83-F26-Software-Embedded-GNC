/* ===========================================================================
 * main.c -- hopper sensor suite entry point.
 *
 * WHAT TO NOTICE
 * --------------
 * The control loop at the bottom contains NO VL53L0X-specific code. It walks
 * an array of sensor_t and polls each one. To add the IMU or the load cells
 * you write a driver, add one entry to the table, and this loop does not
 * change at all.
 *
 * That is the whole point of the sensor_iface indirection, and it is worth the
 * extra layer precisely because you have several sensors coming.
 *
 * ARDUINO EQUIVALENCE
 * -------------------
 *   Arduino                    ESP-IDF
 *   -------                    -------
 *   setup()                    the first half of app_main()
 *   loop()                     the while(1) at the end of app_main()
 *   Serial.begin(115200)       nothing -- the console is already up
 *   Serial.println(x)          ESP_LOGI(TAG, "%d", x)
 *   delay(20)                  vTaskDelay(pdMS_TO_TICKS(20))
 *
 * The last row is the important one: delay() on Arduino just burns CPU, while
 * vTaskDelay() yields to the scheduler. On ESP32 you MUST yield -- a loop that
 * never blocks starves the idle task and the watchdog reboots the chip.
 * ===========================================================================
 */

#include <inttypes.h>
#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "i2c_bus.h"
#include "sensor_iface.h"
#include "vl53l0x_iface.h"

static const char *TAG = "main";

/* --- Board selection ----------------------------------------------------
 * Set this to the board you are flashing. Pin numbers below are RAW ESP32
 * GPIO numbers, which is what ESP-IDF wants -- NOT Arduino "D" or "A" labels.
 *
 * This distinction is the single biggest gotcha on the Nano ESP32. In the
 * Arduino IDE, `A4` is a label that the core remaps to a GPIO for you. In
 * ESP-IDF there is no core and no remapping: you must give it the real GPIO
 * number from the schematic. On the Nano ESP32, the silkscreen pin `A4` is
 * raw GPIO 11, and `A5` is raw GPIO 12.
 *
 * Why it matters that the ESP32-S3 is a different chip: its GPIO numbering
 * skips 22 through 25 entirely. The classic ESP32 default of SDA=21/SCL=22
 * therefore CANNOT work on an S3 -- GPIO 22 does not exist, and gpio_config()
 * will reject it.
 * ---------------------------------------------------------------------- */

#define BOARD_NANO_ESP32   1   /* Arduino Nano ESP32 (ESP32-S3 / NORA-W106) */
#define BOARD_ESP32_DEVKIT 2   /* classic ESP32 DevKitC                     */

/* ---- CHANGE THIS LINE to match your hardware ---- */
#define BOARD              BOARD_NANO_ESP32

#if BOARD == BOARD_NANO_ESP32
    /* Arduino Nano ESP32. Silkscreen label -> raw GPIO:
     *
     *     A4  (SDA)  -> GPIO 11
     *     A5  (SCL)  -> GPIO 12
     *     D2         -> GPIO 5
     *     D3         -> GPIO 6
     *     D10        -> GPIO 21
     *
     * Connect the breakout to the pins labelled A4 and A5 on the board, and
     * leave these numbers alone -- they are already the raw GPIOs those
     * labels correspond to.
     *
     * Build with: idf.py set-target esp32s3
     */
    #define PIN_I2C_SDA    11      /* silkscreen A4 */
    #define PIN_I2C_SCL    12      /* silkscreen A5 */
    #define PIN_TOF_XSHUT   5      /* silkscreen D2 */

#elif BOARD == BOARD_ESP32_DEVKIT
    /* Classic ESP32 DevKitC: the GPIO numbers are printed on the board, so
     * label and raw number are the same thing.
     *
     * Build with: idf.py set-target esp32
     */
    #define PIN_I2C_SDA    21
    #define PIN_I2C_SCL    22
    #define PIN_TOF_XSHUT   5

#else
    /* #error stops the build with a message instead of letting it fail later
     * with something cryptic about undeclared identifiers. Cheap, and it
     * catches a typo in the BOARD define immediately. */
    #error "Set BOARD to BOARD_NANO_ESP32 or BOARD_ESP32_DEVKIT"
#endif

#define I2C_FREQ_HZ        400000

/* A compile-time check that the chosen pins exist on the chosen target.
 *
 * The ESP32-S3 has GPIO 0-21 and 26-48: 22-25 are absent, and 26-32 are wired
 * to the SPI flash and PSRAM inside the module. Using any of those gets you
 * either a runtime error or -- worse -- a board that will not boot.
 *
 * _Static_assert is evaluated by the compiler, so a bad pin is a build error
 * rather than something you discover with a multimeter. The condition must be
 * a constant expression, which these defines are. */
#if CONFIG_IDF_TARGET_ESP32S3
_Static_assert(PIN_I2C_SDA <= 21 || PIN_I2C_SDA >= 33,
               "SDA: ESP32-S3 has no GPIO 22-25, and 26-32 are flash/PSRAM");
_Static_assert(PIN_I2C_SCL <= 21 || PIN_I2C_SCL >= 33,
               "SCL: ESP32-S3 has no GPIO 22-25, and 26-32 are flash/PSRAM");
_Static_assert(PIN_TOF_XSHUT < 0 || PIN_TOF_XSHUT <= 21 || PIN_TOF_XSHUT >= 33,
               "XSHUT: ESP32-S3 has no GPIO 22-25, and 26-32 are flash/PSRAM");
#endif

/* Offset from calibration, in mm. Run vl53l0x_calibrate_offset() once with a
 * target at a known distance, then put the number here -- or better, store it
 * in NVS and load it at boot so it survives a reflash. */
#define TOF_OFFSET_MM      0

/* Control loop period. 20 ms = 50 Hz, which comfortably outpaces a 33 ms
 * sensor budget -- polls that find no new data simply return NOT_FINISHED. */
#define LOOP_PERIOD_MS     20

/* --- The sensor table ---------------------------------------------------
 * Static storage, deliberately. On a microcontroller you want allocation
 * fixed at link time: no malloc means no fragmentation, no leaks, and a
 * memory figure you can actually verify.
 *
 * `static` at file scope here means "private to main.c", not "persists
 * between calls" -- same keyword, different meaning depending on where it
 * appears. One of C's genuine wrinkles.
 * ---------------------------------------------------------------------- */

/* Index into the sensor array. An enum rather than bare integers so that
 * adding a sensor cannot silently shift the others, and SENSOR_COUNT updates
 * itself. */
typedef enum {
    SENSOR_ALTIMETER = 0,
    /* SENSOR_IMU, */
    /* SENSOR_LOAD_CELL, */
    SENSOR_COUNT          /* must stay last: equals the number above it */
} sensor_id_t;

static sensor_t             g_sensors[SENSOR_COUNT];
static vl53l0x_sensor_ctx_t g_tof_ctx;

/* Latest sample from each sensor. A GNC loop reads from here. */
static sensor_sample_t      g_samples[SENSOR_COUNT];

static i2c_master_bus_handle_t g_i2c_bus;

/* ===========================================================================
 * Setup
 * ======================================================================== */

static esp_err_t sensors_setup(void)
{
    esp_err_t err;

    /* --- The shared I2C bus, once ------------------------------------- */
    err = i2c_bus_init(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ_HZ, &g_i2c_bus);
    if (err != ESP_OK) return err;

    /* Scan and log. Keep this in: when a sensor stops working, the scan tells
     * you in one line whether it is an electrical problem or a code problem.
     * You want that distinction before you start reading driver source. */
    i2c_bus_scan(g_i2c_bus);

    /* --- Sensor 1: VL53L0X rangefinder -------------------------------- */
    vl53l0x_config_t tof_cfg;
    vl53l0x_config_default(&tof_cfg, g_i2c_bus);   /* defaults first ... */
    tof_cfg.xshut_gpio  = PIN_TOF_XSHUT;           /* ... then overrides */
    tof_cfg.offset_mm   = TOF_OFFSET_MM;
    tof_cfg.i2c_freq_hz = I2C_FREQ_HZ;

    /* LONG_RANGE reaches ~2 m but is more ambient-sensitive. For a hopper in
     * daylight, HIGH_SPEED (20 ms, less lag) is often the better trade --
     * you are range-limited by sunlight either way. See notes section 9. */
    tof_cfg.profile = VL53L0X_PROFILE_LONG_RANGE;

    err = vl53l0x_sensor_create(&g_sensors[SENSOR_ALTIMETER], &g_tof_ctx,
                                &tof_cfg, "altimeter");
    if (err != ESP_OK) return err;

    /* --- Adding the next sensor goes here ------------------------------
     * The pattern is identical for every sensor:
     *
     *   1. write components/mpu6050/ with the same two-file split
     *      (mpu6050.c for registers, mpu6050_iface.c for the vtable)
     *   2. add SENSOR_IMU to sensor_id_t above
     *   3. declare its ctx struct next to g_tof_ctx
     *   4. call mpu6050_sensor_create(&g_sensors[SENSOR_IMU], ...) here
     *
     * The loop below does not change. Neither does anything else.
     * ---------------------------------------------------------------- */

    /* --- Bring every sensor up ----------------------------------------
     * One failure does not abort the others. A hopper with a dead
     * rangefinder and a live IMU is still worth flying in a degraded mode;
     * a hopper that refuses to boot is not. sensor_t.online records which
     * ones made it. */
    int online = 0;
    for (int i = 0; i < SENSOR_COUNT; i++) {
        err = sensor_init(&g_sensors[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "[%s] OFFLINE: %s", g_sensors[i].label,
                     esp_err_to_name(err));
            continue;      /* skip to the next sensor */
        }

        err = sensor_start(&g_sensors[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "[%s] start failed: %s", g_sensors[i].label,
                     esp_err_to_name(err));
            g_sensors[i].online = false;
            continue;
        }
        online++;
    }

    ESP_LOGI(TAG, "%d/%d sensors online", online, SENSOR_COUNT);

    /* ESP_ERR_NOT_FOUND if nothing came up at all -- the caller decides
     * whether that is fatal. */
    return (online > 0) ? ESP_OK : ESP_ERR_NOT_FOUND;
}

/* ===========================================================================
 * Entry point
 *
 * app_main is ESP-IDF's equivalent of setup() + loop() combined. It runs as a
 * FreeRTOS task. Returning from it is legal (the task just exits), but here we
 * loop forever, as a control application would.
 * ======================================================================== */

void app_main(void)
{
    ESP_LOGI(TAG, "hopper sensor suite starting");

    esp_err_t err = sensors_setup();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "no sensors available: %s", esp_err_to_name(err));

        /* Do not fall through into the control loop with no sensors, and do
         * not return either -- that would leave the console silent with no
         * indication of what happened. Idle visibly instead.
         *
         * In flight software this is where you would enter a safe state:
         * valves closed, telemetry still running. */
        while (1) {
            ESP_LOGE(TAG, "halted -- check wiring, then reset");
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }

    /* --- Optional: offset calibration ---------------------------------
     * Uncomment, place a flat white target at exactly 100 mm, flash, and read
     * the offset off the console. Then put it in TOF_OFFSET_MM (or NVS) and
     * comment this back out. Blocks for ~2 s.
     *
     *   int16_t offset;
     *   if (vl53l0x_calibrate_offset(&g_tof_ctx.dev, 100, 50, &offset)
     *       == ESP_OK) {
     *       ESP_LOGW(TAG, "set TOF_OFFSET_MM to %d", offset);
     *   }
     * ---------------------------------------------------------------- */

    /* --- Control loop ------------------------------------------------- */

    /* Rate-limit the console. At 50 Hz, logging every sample floods the
     * monitor and the UART itself becomes the bottleneck -- a classic way to
     * make a control loop miss its deadline. */
    int64_t last_print_us = 0;
    const int64_t PRINT_INTERVAL_US = 250000;   /* 4 Hz */

    /* Track the loop period for real rather than assuming it. If this grows
     * past LOOP_PERIOD_MS you are overrunning, and you want to know. */
    int64_t last_loop_us = esp_timer_get_time();

    while (1) {
        int64_t now_us = esp_timer_get_time();
        int64_t dt_us  = now_us - last_loop_us;
        last_loop_us   = now_us;

        /* --- Poll every sensor ---------------------------------------
         * Generic: no sensor type is named anywhere in this block. Each
         * poll is non-blocking, so a sensor with nothing new costs one I2C
         * status read and nothing more. */
        for (int i = 0; i < SENSOR_COUNT; i++) {
            sensor_sample_t s;
            esp_err_t e = sensor_poll(&g_sensors[i], &s);

            if (e == ESP_OK && s.valid) {
                g_samples[i] = s;          /* keep the newest good sample */
            } else if (e != ESP_OK && e != ESP_ERR_NOT_FINISHED) {
                /* A real bus error. ESP_ERR_NOT_FINISHED just means the
                 * sensor is still integrating, which is expected most
                 * iterations and is not worth logging. */
                ESP_LOGW(TAG, "[%s] poll error: %s", g_sensors[i].label,
                         esp_err_to_name(e));
            }

            /* Dead-sensor detection: 50 consecutive failures at 50 Hz is one
             * second of silence. Mark it offline so sensor_poll stops trying
             * and the GNC layer can see it is gone. */
            if (g_sensors[i].error_count > 50) {
                ESP_LOGE(TAG, "[%s] offline after %" PRIu32 " errors",
                         g_sensors[i].label, g_sensors[i].error_count);
                g_sensors[i].online = false;
            }
        }

        /* --- Where the GNC code goes --------------------------------
         * Read from g_samples[] -- it holds the newest valid sample from
         * each sensor, each with its own timestamp.
         *
         *   const sensor_sample_t *alt = &g_samples[SENSOR_ALTIMETER];
         *   if (alt->valid) {
         *       float altitude_m = alt->value[0];
         *
         *       // Two corrections worth applying, from notes section 11:
         *       //
         *       // 1. TILT. The sensor measures slant range, not altitude:
         *       //    altitude = range * cos(tilt). At 20 deg you read 6%
         *       //    high. Needs the IMU.
         *       //
         *       // 2. LATENCY. The reading is an average over the timing
         *       //    budget, so its true epoch is ~budget/2 before
         *       //    alt->timestamp_us, plus median group delay. Model
         *       //    this; do not tune around it.
         *
         *       // ... then your valve command, per MAC_35_driver_v1.ino
         *   }
         * ---------------------------------------------------------- */

        /* --- Telemetry ---------------------------------------------- */
        if (now_us - last_print_us >= PRINT_INTERVAL_US) {
            last_print_us = now_us;

            const sensor_sample_t *alt = &g_samples[SENSOR_ALTIMETER];

            if (alt->valid) {
                /* Age of the sample: how stale is what we are acting on?
                 * In a real loop you would reject anything older than a few
                 * budgets rather than just reporting it. */
                int64_t age_ms = (now_us - alt->timestamp_us) / 1000;

                /* PRId64 rather than %lld: int64_t is `long long` on the
                 * 32-bit ESP32 but `long` on a 64-bit host, so a hardcoded
                 * length modifier is only right on one of them. The macro
                 * expands to whichever is correct for the target. */
                ESP_LOGI(TAG, "alt %6.3f m | age %3" PRId64 " ms | "
                              "loop %3" PRId64 " ms | n=%" PRIu32,
                         alt->value[0], age_ms, dt_us / 1000,
                         g_sensors[SENSOR_ALTIMETER].sample_count);
            } else {
                ESP_LOGW(TAG, "alt --- no valid sample | loop %3" PRId64
                              " ms", dt_us / 1000);
            }
        }

        /* Yield. This is mandatory, not politeness: without it the idle task
         * never runs, the task watchdog fires, and the chip reboots. It is
         * also what makes the 50 Hz cadence real. */
        vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
}
