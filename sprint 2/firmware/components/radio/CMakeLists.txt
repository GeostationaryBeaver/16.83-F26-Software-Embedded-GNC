idf_component_register(SRCS "radio.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES protocol esp_wifi esp_netif esp_event nvs_flash)
