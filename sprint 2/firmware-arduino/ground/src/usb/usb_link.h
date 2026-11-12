// Framed packets to and from the laptop over USB (the dashboard, ground_station/index.html):
//   0xAA 0x55 <len> <packet: len bytes> <checksum: sum of packet bytes, mod 256>
#pragma once

#include <stdint.h>

#define USB_MAX_PACKET_LEN 250  // ESP-NOW limit

void usb_link_init();

// Queues one framed packet and sends what fits now. Never waits. Drops the packet and
// returns false if nothing on the laptop has the port open, or if the laptop has fallen
// more than ~2 KB behind.
bool usb_link_send(const uint8_t *packet, uint8_t len);

// Sends queued bytes as the USB buffer frees up. Call every loop.
void usb_link_service();

// DIAGNOSTIC (temporary): counters since boot
typedef struct {
  uint32_t rx_bytes;      // bytes read from the laptop
  uint32_t rx_packets;    // packets with a good checksum
  uint32_t rx_bad;        // bad checksums
  uint32_t tx_dropped;    // frames dropped: laptop behind
  uint32_t tx_queued;     // bytes waiting to go to the laptop
} usb_link_stats_t;
extern usb_link_stats_t usb_link_stats;

// Reads whatever bytes have arrived. Once a whole packet with a good checksum is in, copies
// it to packet (USB_MAX_PACKET_LEN bytes) and returns its length; otherwise returns 0.
// Never waits.
int usb_link_receive(uint8_t *packet);
