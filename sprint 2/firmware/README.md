# Firmware

Two ESP-IDF projects for Arduino Nano ESP32 boards, linked over ESP-NOW.

```
firmware/
├── components/          Shared by both boards
│   ├── protocol/        Packet definitions + board MAC addresses (protocol.h)
│   └── radio/           ESP-NOW link to one peer board
├── hopper/
│   ├── main/main.c      Starts everything; handles commands from the ground
│   └── components/
│       ├── control/     Control loops, core 1 (currently a placeholder)
│       ├── telemetry/   Lock-free ring buffer + radio task, core 0
│       └── led/         Onboard RGB LED
└── ground/
    └── main/main.c      Bridges ESP-NOW <-> USB for the dashboard (../ground_station)
```

Each piece of hardware or subsystem is its own component, so you only edit the component you own. To add one, copy an existing component folder and add its name to `PRIV_REQUIRES` in `main/CMakeLists.txt`.

## Sending telemetry from the control loop

```c
#include "telemetry.h"

telemetry_data_t data = { ... };
telemetry_push(&data);  // non-blocking, never takes a lock
```

`telemetry_push` must only be called from one task. To change what is sent, edit `telemetry_data_t` in `components/protocol/include/protocol.h` and the matching decode in `ground_station/index.html`.

## First-time setup

1. Read each board's MAC address:
   ```
   python -m esptool --chip esp32s3 read-mac
   ```
2. Put them in `HOPPER_MAC` and `GROUND_MAC` in `components/protocol/include/protocol.h`.

## Build and flash

```
cd hopper   # or ground
idf.py build
idf.py -p PORT flash monitor
```

The ground board has no log console: its USB port only carries binary frames to the dashboard. Flashing erases the Arduino bootloader. If flashing can't connect, hold B1 (or short B1 to GND) while pressing reset.

## Dashboard

From the repo root:

```
python3 -m http.server -d ground_station 8000
```

Open http://localhost:8000 in Chrome or Edge (Web Serial is required), then click Connect and pick the ground ESP.
