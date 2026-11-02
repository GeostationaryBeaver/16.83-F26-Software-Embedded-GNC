// Packet definitions shared by the hopper, the ground ESP, and the dashboard.
// If you change a packet here, update the matching decode/encode in
// ground_station/index.html.
#pragma once

#include <stdint.h>

// Fill these in with each board's MAC address. To read a board's MAC:
//   python -m esptool --chip esp32s3 read-mac
#define HOPPER_MAC  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define GROUND_MAC  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}

#define RADIO_CHANNEL 1

// First byte of every packet
typedef enum {
    MSG_TELEMETRY = 1,  // hopper -> ground
    MSG_CMD_SET_LED = 2,  // ground -> hopper
} msg_type_t;

// Fields sent by the control loop. Replace the placeholders with real fields.
typedef struct __attribute__((packed)) {
    float placeholder[4];
} telemetry_data_t;

typedef struct __attribute__((packed)) {
    uint8_t type;      // MSG_TELEMETRY
    uint32_t seq;      // increments by 1 per packet; a gap means packets were lost
    uint32_t time_ms;  // hopper uptime when the data was pushed
    telemetry_data_t data;
} telemetry_packet_t;

typedef struct __attribute__((packed)) {
    uint8_t type;  // MSG_CMD_SET_LED
    uint8_t r;     // 0 = off, 1 = on
    uint8_t g;
    uint8_t b;
} cmd_set_led_t;
