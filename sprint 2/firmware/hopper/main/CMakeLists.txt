# esp_psram is listed so MINIMAL_BUILD keeps the Nano ESP32's 8 MB PSRAM enabled
idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES esp_psram led telemetry control
                       INCLUDE_DIRS "")
