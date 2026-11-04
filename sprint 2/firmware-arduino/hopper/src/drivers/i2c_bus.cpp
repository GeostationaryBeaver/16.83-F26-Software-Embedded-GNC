#include <Arduino.h>
#include <Wire.h>
#include "i2c_bus.h"
#include "pins.h"

static bool s_dual = true;
static int s_shared_on = -1;  // shared controller: which device's pins Wire is on, -1 = none

// Clocks SCL 9 times then sends a stop, to free a sensor stuck half-way through a message
static void bus_clear(uint8_t sda, uint8_t scl) {
  pinMode(sda, INPUT_PULLUP);
  pinMode(scl, OUTPUT_OPEN_DRAIN);
  for (int i = 0; i < 9; i++) {
    digitalWrite(scl, LOW); delayMicroseconds(5);
    digitalWrite(scl, HIGH); delayMicroseconds(5);
  }
  pinMode(sda, OUTPUT_OPEN_DRAIN);
  digitalWrite(sda, LOW); delayMicroseconds(5);
  digitalWrite(scl, HIGH); delayMicroseconds(5);
  digitalWrite(sda, HIGH);
  pinMode(sda, INPUT);
  pinMode(scl, INPUT);
}

// warm: transfers to empty addresses before talking to a real device
static void start_bus(TwoWire &wire, uint8_t sda, uint8_t scl, int warm) {
  bus_clear(sda, scl);
  wire.begin(sda, scl, I2C_FREQ_HZ);
  wire.setTimeOut(I2C_TIMEOUT_MS);
  for (int a = 0; a < warm; a++) {
    wire.beginTransmission(0x08 + a);
    wire.endTransmission();
  }
}

static void start_device_bus(TwoWire &wire, i2c_bus_t bus, int warm) {
  if (bus == I2C_BUS_LASER) start_bus(wire, PIN_LASER_SDA, PIN_LASER_SCL, warm);
  else start_bus(wire, PIN_IMU_SDA, PIN_IMU_SCL, warm);
}

void i2c_bus_init() {
  start_device_bus(Wire, I2C_BUS_LASER, 8);
  start_device_bus(Wire1, I2C_BUS_IMU, 8);
}

TwoWire &i2c_bus_get(i2c_bus_t bus) {
  if (s_dual) return bus == I2C_BUS_LASER ? Wire : Wire1;
  if (s_shared_on != bus) {
    Wire.end();
    start_device_bus(Wire, bus, 2);
    s_shared_on = bus;
  }
  return Wire;
}

void i2c_bus_restart(i2c_bus_t bus) {
  if (s_dual) {
    TwoWire &wire = bus == I2C_BUS_LASER ? Wire : Wire1;
    wire.end();
    start_device_bus(wire, bus, 8);
  } else {
    s_shared_on = -1;  // the next i2c_bus_get() restarts it
  }
}

void i2c_bus_use_shared(const char *why) {
  if (!s_dual) return;
  Wire1.end();
  s_dual = false;
  s_shared_on = -1;
  Serial.printf("I2C: %s, switching to one shared controller\n", why);
}

bool i2c_bus_dual() { return s_dual; }

bool i2c_bus_write_reg(i2c_bus_t bus, uint8_t addr, uint8_t reg, uint8_t value) {
  TwoWire &wire = i2c_bus_get(bus);
  wire.beginTransmission(addr);
  wire.write(reg);
  wire.write(value);
  return wire.endTransmission() == 0;
}

bool i2c_bus_read_regs(i2c_bus_t bus, uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
  TwoWire &wire = i2c_bus_get(bus);
  wire.beginTransmission(addr);
  wire.write(reg);
  if (wire.endTransmission(false) != 0) return false;  // repeated start
  if (wire.requestFrom(addr, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = wire.read();
  return true;
}
