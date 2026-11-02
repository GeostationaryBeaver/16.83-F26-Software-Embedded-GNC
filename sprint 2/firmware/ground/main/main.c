// Bridge between the hopper (ESP-NOW) and the laptop (USB).
// Packets go through unchanged in both directions. Over USB, each packet is
// wrapped in a frame:
//   0xAA 0x55 <len> <packet: len bytes> <checksum: sum of packet bytes, mod 256>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "protocol.h"
#include "radio.h"

#define SYNC0 0xAA
#define SYNC1 0x55
#define MAX_PACKET_LEN 250  // ESP-NOW limit

static uint8_t checksum(const uint8_t *data, int len)
{
    uint8_t sum = 0;
    for (int i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

// Hopper -> laptop. Runs in the WiFi task, so never block.
static void on_radio_rx(const uint8_t *data, int len)
{
    if (len < 1 || len > MAX_PACKET_LEN) {
        return;
    }
    uint8_t frame[3 + MAX_PACKET_LEN + 1];
    frame[0] = SYNC0;
    frame[1] = SYNC1;
    frame[2] = len;
    memcpy(&frame[3], data, len);
    frame[3 + len] = checksum(data, len);
    usb_serial_jtag_write_bytes(frame, len + 4, 0);
}

// Laptop -> hopper
void app_main(void)
{
    usb_serial_jtag_driver_config_t usb_cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));

    const uint8_t hopper_mac[6] = HOPPER_MAC;
    ESP_ERROR_CHECK(radio_init(hopper_mac, on_radio_rx));

    enum { WAIT_SYNC0, WAIT_SYNC1, WAIT_LEN, WAIT_PAYLOAD, WAIT_CHECKSUM } state = WAIT_SYNC0;
    uint8_t packet[MAX_PACKET_LEN];
    int len = 0;
    int received = 0;

    while (1) {
        uint8_t byte;
        if (usb_serial_jtag_read_bytes(&byte, 1, portMAX_DELAY) != 1) {
            continue;
        }
        switch (state) {
        case WAIT_SYNC0:
            if (byte == SYNC0) {
                state = WAIT_SYNC1;
            }
            break;
        case WAIT_SYNC1:
            state = (byte == SYNC1) ? WAIT_LEN : WAIT_SYNC0;
            break;
        case WAIT_LEN:
            len = byte;
            received = 0;
            state = (len > 0 && len <= MAX_PACKET_LEN) ? WAIT_PAYLOAD : WAIT_SYNC0;
            break;
        case WAIT_PAYLOAD:
            packet[received++] = byte;
            if (received == len) {
                state = WAIT_CHECKSUM;
            }
            break;
        case WAIT_CHECKSUM:
            if (byte == checksum(packet, len)) {
                radio_send(packet, len);
            }
            state = WAIT_SYNC0;
            break;
        }
    }
}
