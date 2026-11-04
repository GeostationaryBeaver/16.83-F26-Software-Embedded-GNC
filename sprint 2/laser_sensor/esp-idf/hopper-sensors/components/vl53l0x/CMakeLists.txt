# ---------------------------------------------------------------------------
# vl53l0x -- the register-level driver plus its sensor_iface adapter.
#
# Two source files, both listed explicitly. ESP-IDF never globs: if you add a
# .c file and forget to add it here, it simply is not built, and you get a
# confusing "undefined reference" at link time.
# ---------------------------------------------------------------------------
idf_component_register(
    SRCS "vl53l0x.c"
         "vl53l0x_iface.c"
    INCLUDE_DIRS "include"
    REQUIRES driver sensor_common esp_timer
)
