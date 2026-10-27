# GNC Firmware (ESP32-S3)

ESP-IDF firmware for the GNC system. `main/main.c` is the single entry point (`app_main`); each piece of hardware or subsystem lives in its own component under `components/`.

## Structure

```
├── CMakeLists.txt              Project definition
├── sdkconfig                   Build configuration (idf.py menuconfig)
├── main
│   ├── CMakeLists.txt          Lists the components main depends on
│   └── main.c                  app_main(): initializes components, runs the main loop
└── components
    ├── load_cell
    │   ├── CMakeLists.txt
    │   ├── include/load_cell.h Public API
    │   └── load_cell.c
    └── solenoid_valve
        ├── CMakeLists.txt
        ├── include/solenoid_valve.h
        └── solenoid_valve.c
```

## Adding a component

1. Create `components/<name>/` with `CMakeLists.txt`, `include/<name>.h`, and `<name>.c` (copy an existing component as a template).
2. Add any ESP-IDF components it uses (e.g. `esp_driver_gpio`, `esp_driver_i2c`) to `REQUIRES` in its `CMakeLists.txt`.
3. Add `<name>` to `PRIV_REQUIRES` in `main/CMakeLists.txt` and `#include "<name>.h"` in `main.c`.

The project uses `MINIMAL_BUILD`, so a component is only compiled if `main` (or another required component) depends on it.

## Build and flash

```
idf.py set-target esp32s3   # first time only
idf.py build
idf.py -p PORT flash monitor
```

See the [ESP-IDF build system guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-guides/build-system.html) for more detail.
