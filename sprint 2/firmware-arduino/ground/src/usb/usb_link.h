// Framed packets to and from the laptop over USB (the dashboard, ground_station/index.html):
//   0xAA 0x55 <len> <packet: len bytes> <checksum: sum of packet bytes, mod 256>
#pragma once

#include <stdint.h>

#define USB_MAX_PACKET_LEN 250  // ESP-NOW limit

void usb_link_init();

// Writes one framed packet. Drops it and returns false if nothing on the laptop has the
// port open. While something does, the OS keeps reading, so this doesn't stall.
bool usb_link_send(const uint8_t *packet, uint8_t len);

// Reads whatever bytes have arrived. Once a whole packet with a good checksum is in, copies
// it to packet (USB_MAX_PACKET_LEN bytes) and returns its length; otherwise returns 0.
// Never waits.
int usb_link_receive(uint8_t *packet);
