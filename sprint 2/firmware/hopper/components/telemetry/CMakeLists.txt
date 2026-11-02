idf_component_register(SRCS "telemetry.c"
                       INCLUDE_DIRS "include"
                       REQUIRES protocol
                       PRIV_REQUIRES radio esp_timer)
