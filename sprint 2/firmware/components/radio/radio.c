#include <string.h>
#include "radio.h"
#include "protocol.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"

static uint8_t s_peer_mac[6];
static radio_rx_cb_t s_rx_cb;

static void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    // Ignore packets from anything other than our peer
    if (memcmp(info->src_addr, s_peer_mac, 6) != 0) {
        return;
    }
    s_rx_cb(data, len);
}

esp_err_t radio_init(const uint8_t peer_mac[6], radio_rx_cb_t rx_cb)
{
    memcpy(s_peer_mac, peer_mac, 6);
    s_rx_cb = rx_cb;

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(RADIO_CHANNEL, WIFI_SECOND_CHAN_NONE));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_recv));

    esp_now_peer_info_t peer = {
        .channel = RADIO_CHANNEL,
        .ifidx = WIFI_IF_STA,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, peer_mac, 6);
    return esp_now_add_peer(&peer);
}

esp_err_t radio_send(const void *data, size_t len)
{
    return esp_now_send(s_peer_mac, data, len);
}
