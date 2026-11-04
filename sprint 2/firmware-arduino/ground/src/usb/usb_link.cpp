#include <Arduino.h>
#include <string.h>
#include "usb_link.h"

#define SYNC0 0xAA
#define SYNC1 0x55

typedef enum { WAIT_SYNC0, WAIT_SYNC1, WAIT_LEN, WAIT_PAYLOAD, WAIT_CHECKSUM } parse_state_t;

static parse_state_t s_state = WAIT_SYNC0;
static uint8_t s_buf[USB_MAX_PACKET_LEN];
static int s_len = 0;
static int s_received = 0;

static uint8_t checksum(const uint8_t *data, int len) {
  uint8_t sum = 0;
  for (int i = 0; i < len; i++) sum += data[i];
  return sum;
}

void usb_link_init() {
  Serial.begin(115200);  // native USB: the baud rate is ignored
}

bool usb_link_send(const uint8_t *packet, uint8_t len) {
  if (len < 1 || len > USB_MAX_PACKET_LEN) return false;
  // Not availableForWrite(): the USB buffer is only 64 bytes, smaller than one frame.
  // write() returns 0 straight away when nothing has the port open.
  if (!Serial) return false;
  uint8_t frame[3 + USB_MAX_PACKET_LEN + 1];
  frame[0] = SYNC0;
  frame[1] = SYNC1;
  frame[2] = len;
  memcpy(&frame[3], packet, len);
  frame[3 + len] = checksum(packet, len);
  return Serial.write(frame, len + 4) == (size_t)(len + 4);
}

int usb_link_receive(uint8_t *packet) {
  while (Serial.available()) {
    uint8_t byte = Serial.read();
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
          memcpy(packet, s_buf, s_len);
          return s_len;
        }
        break;
    }
  }
  return 0;
}
