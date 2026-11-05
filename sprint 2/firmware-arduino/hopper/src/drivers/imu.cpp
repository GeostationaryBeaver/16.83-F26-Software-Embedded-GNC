#include <Arduino.h>
#include "imu.h"
#include "i2c_bus.h"
#include "pins.h"

#define REG_WHO_AM_I  0x0F
#define REG_CTRL1_XL  0x10
#define REG_CTRL2_G   0x11
#define REG_CTRL3_C   0x12
#define REG_OUTX_L_G  0x22  // gyro x, y, z then accel x, y, z, 12 bytes

// Scale factors from the datasheet for the ranges set in imu_init()
#define ACCEL_G_PER_LSB    0.000244f   // +-8 g
#define GYRO_DPS_PER_LSB   0.035f      // +-1000 dps
#define G_TO_MS2           9.80665f
#define DEG_TO_RADF        0.017453293f

static uint8_t s_addr = 0;

// 0x69 = LSM6DS3, 0x6A = LSM6DS3TR-C, 0x6C = LSM6DSO. Same registers for what we use.
static bool who_am_i_ok(uint8_t addr) {
  for (int i = 0; i < 3; i++) {
    uint8_t who = 0;
    if (i2c_bus_read_regs(I2C_BUS_IMU, addr, REG_WHO_AM_I, &who, 1) &&
        (who == 0x69 || who == 0x6A || who == 0x6C)) return true;
    delay(2);
  }
  return false;
}

bool imu_init() {
  s_addr = 0;
  uint8_t addr = who_am_i_ok(IMU_ADDR) ? IMU_ADDR : who_am_i_ok(IMU_ADDR_ALT) ? IMU_ADDR_ALT : 0;
  if (!addr) return false;

  if (!i2c_bus_write_reg(I2C_BUS_IMU, addr, REG_CTRL1_XL, 0x6C)) return false;  // 416 Hz, +-8 g
  if (!i2c_bus_write_reg(I2C_BUS_IMU, addr, REG_CTRL2_G, 0x68)) return false;   // 416 Hz, 1000 dps
  if (!i2c_bus_write_reg(I2C_BUS_IMU, addr, REG_CTRL3_C, 0x44)) return false;   // block data update, auto-increment
  s_addr = addr;
  return true;
}

bool imu_steady(int reads) {
  if (!s_addr) return false;
  uint8_t b[12];
  for (int i = 0; i < reads; i++) {
    if (!i2c_bus_read_regs(I2C_BUS_IMU, s_addr, REG_OUTX_L_G, b, 12)) return false;
    delay(5);
  }
  return true;
}

bool imu_read(imu_sample_t *out) {
  if (!s_addr) return false;
  uint8_t b[12];
  if (!i2c_bus_read_regs(I2C_BUS_IMU, s_addr, REG_OUTX_L_G, b, 12)) return false;
  for (int i = 0; i < 3; i++) {
    int16_t g = (int16_t)(b[2 * i] | (b[2 * i + 1] << 8));
    int16_t a = (int16_t)(b[6 + 2 * i] | (b[6 + 2 * i + 1] << 8));
    out->gyro[i] = g * GYRO_DPS_PER_LSB * DEG_TO_RADF;
    out->accel[i] = a * ACCEL_G_PER_LSB * G_TO_MS2;
  }
  return true;
}
