# ---------------------------------------------------------------------------
# main -- the application entry point. "main" is a special component name:
# ESP-IDF always builds it and calls its app_main() after boot.
# ---------------------------------------------------------------------------
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES sensor_common i2c_bus vl53l0x esp_timer
)
