#include <Arduino.h>
#include <string.h>
#include "usb_link.h"

#define SYNC0 0xAA
#define SYNC1 0x55

// Frames wait here for room in the USB buffer. 2 KB = 16 telemetry frames, ~320 ms at 50 Hz.
// Must be a power of two (the indices wrap).
#define TX_RING_SIZE 2048

typedef enum { WAIT_SYNC0, WAIT_SYNC1, WAIT_LEN, WAIT_PAYLOAD, WAIT_CHECKSUM } parse_state_t;

static parse_state_t s_state = WAIT_SYNC0;
static uint8_t s_buf[USB_MAX_PACKET_LEN];
static int s_len = 0;
static int s_received = 0;

static uint8_t s_tx[TX_RING_SIZE];
static uint32_t s_tx_head = 0;  // next byte to fill
static uint32_t s_tx_tail = 0;  // next byte to send

static uint8_t checksum(const uint8_t *data, int len) {
  uint8_t sum = 0;
  for (int i = 0; i < len; i++) sum += data[i];
  return sum;
}

usb_link_stats_t usb_link_stats = {};

static uint32_t tx_used() { return s_tx_head - s_tx_tail; }

static void tx_put(const uint8_t *data, uint32_t len) {
  for (uint32_t i = 0; i < len; i++) s_tx[(s_tx_head + i) % TX_RING_SIZE] = data[i];
  s_tx_head += len;
}

void usb_link_init() {
  Serial.begin(115200);  // native USB: the baud rate is ignored
}

void usb_link_service() {
  usb_link_stats.tx_queued = tx_used();
  if (!Serial) {  // nothing on the laptop has the port open: drop what's waiting
    s_tx_tail = s_tx_head;
    return;
  }
  // Never hand write() more than the USB buffer has room for. A write that has to wait
  // for room (the laptop not reading for a moment) jammed the USB output for good: no
  // telemetry reached the laptop again until the board was reset.
  while (tx_used() > 0) {
    int room = Serial.availableForWrite();
    if (room <= 0) return;
    uint32_t start = s_tx_tail % TX_RING_SIZE;
    uint32_t chunk = min(tx_used(), (uint32_t)(TX_RING_SIZE - start));
    chunk = min(chunk, (uint32_t)room);
    size_t sent = Serial.write(&s_tx[start], chunk);
    s_tx_tail += sent;
    if (sent < chunk) return;
  }
}

bool usb_link_send(const uint8_t *packet, uint8_t len) {
  if (len < 1 || len > USB_MAX_PACKET_LEN) return false;
  if (!Serial) return false;
  // A whole frame or nothing: if the laptop has fallen behind, this frame is dropped (the
  // dashboard sees the seq gap) rather than waiting
  if (TX_RING_SIZE - tx_used() < (uint32_t)len + 4) {
    usb_link_stats.tx_dropped++;
    return false;
  }
  const uint8_t head[3] = {SYNC0, SYNC1, len};
  const uint8_t sum = checksum(packet, len);
  tx_put(head, 3);
  tx_put(packet, len);
  tx_put(&sum, 1);
  usb_link_service();
  return true;
}

int usb_link_receive(uint8_t *packet) {
  while (Serial.available()) {
    uint8_t byte = Serial.read();
    usb_link_stats.rx_bytes++;
    switch (s_state) {
      case WAIT_SYNC0:
        if (byte == SYNC0) s_state = WAIT_SYNC1;
        break;
      case WAIT_SYNC1:
        s_state = byte == SYNC1 ? WAIT_LEN : WAIT_SYNC0;
        break;
      case WAIT_LEN:
        s_len = byte;
        s_received = 0;
        s_state = (s_len > 0 && s_len <= USB_MAX_PACKET_LEN) ? WAIT_PAYLOAD : WAIT_SYNC0;
        break;
      case WAIT_PAYLOAD:
        s_buf[s_received++] = byte;
        if (s_received == s_len) s_state = WAIT_CHECKSUM;
        break;
      case WAIT_CHECKSUM:
        s_state = WAIT_SYNC0;
        if (byte == checksum(s_buf, s_len)) {
          usb_link_stats.rx_packets++;
          memcpy(packet, s_buf, s_len);
          return s_len;
        }
        usb_link_stats.rx_bad++;
        break;
    }
  }
  return 0;
}
