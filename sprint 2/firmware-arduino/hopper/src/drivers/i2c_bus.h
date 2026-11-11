// I2C for the laser and the IMU. Normally each has its own controller: laser on Wire (A4/A5),
// IMU on Wire1 (D2/D3). On our board Wire1 has not always kept the IMU, so if it fails there,
// i2c_bus_use_shared() switches to one controller (Wire) moved between the two pin pairs
// (about 1 ms per switch, the arrangement every doctor run used).
// Not thread safe: use the laser and IMU from one task only.
#pragma once

#include <stdint.h>
#include <Wire.h>

typedef enum { I2C_BUS_LASER, I2C_BUS_IMU } i2c_bus_t;

// Starts both controllers. Frees any stuck sensor first.
void i2c_bus_init();

// The controller for this device. On the shared controller, moves Wire to its pins first.
TwoWire &i2c_bus_get(i2c_bus_t bus);

// Frees and restarts the device's bus, to retry a sensor that dropped out.
void i2c_bus_restart(i2c_bus_t bus);

// Gives up on Wire1 and shares Wire between both devices from now on.
void i2c_bus_use_shared();

// True while the laser and IMU each have their own controller
bool i2c_bus_dual();

bool i2c_bus_write_reg(i2c_bus_t bus, uint8_t addr, uint8_t reg, uint8_t value);
bool i2c_bus_read_regs(i2c_bus_t bus, uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n);
