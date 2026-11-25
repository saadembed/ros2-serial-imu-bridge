/*
 * Minimal MPU-6050 driver over an abstract I2C bus, so it runs on the target
 * (HAL callbacks) and on a PC (fake bus in tests). Portable C99, no HAL.
 */
#ifndef MPU6050_H
#define MPU6050_H

#include <stddef.h>
#include <stdint.h>

#define MPU6050_ADDR_AD0_LOW   0x68u  /* 7-bit address, AD0 pin low (GY-521 default) */
#define MPU6050_ADDR_AD0_HIGH  0x69u

#define MPU6050_REG_GYRO_CONFIG   0x1Bu
#define MPU6050_REG_ACCEL_CONFIG  0x1Cu
#define MPU6050_REG_ACCEL_XOUT_H  0x3Bu
#define MPU6050_REG_PWR_MGMT_1    0x6Bu
#define MPU6050_REG_WHO_AM_I      0x75u

#define MPU6050_WHO_AM_I_VALUE    0x68u
#define MPU6050_BURST_LEN         14u   /* accel(6) + temp(2) + gyro(6) */

/* PWR_MGMT_1: SLEEP bit cleared, clock source = PLL with X gyro (CLKSEL=1). */
#define MPU6050_PWR_MGMT_1_WAKE   0x01u
/* FS_SEL / AFS_SEL = 0 -> +/-250 deg/s and +/-2 g. */
#define MPU6050_GYRO_FS_250       0x00u
#define MPU6050_ACCEL_FS_2G       0x00u

/*
 * I2C abstraction. Both callbacks return 0 on success, non-zero on failure.
 * addr is the 7-bit device address; reg is the register address.
 */
typedef struct {
    int (*write_reg)(void *ctx, uint8_t addr, uint8_t reg,
                     const uint8_t *data, size_t len);
    int (*read_reg)(void *ctx, uint8_t addr, uint8_t reg,
                    uint8_t *data, size_t len);
    void *ctx;
} mpu6050_bus_t;

typedef struct {
    mpu6050_bus_t bus;
    uint8_t addr;
} mpu6050_t;

typedef struct {
    int16_t ax, ay, az;
    int16_t temp;   /* raw; deg C = temp / 340 + 36.53 */
    int16_t gx, gy, gz;
} mpu6050_raw_t;

typedef enum {
    MPU6050_OK = 0,
    MPU6050_ERR_ARG = -1,
    MPU6050_ERR_BUS = -2,
    MPU6050_ERR_WHO_AM_I = -3
} mpu6050_status_t;

/*
 * Check WHO_AM_I, wake the device (PWR_MGMT_1), set +/-250 deg/s and +/-2 g.
 * Register writes happen only after WHO_AM_I matched.
 */
mpu6050_status_t mpu6050_init(mpu6050_t *dev, const mpu6050_bus_t *bus, uint8_t addr);

/* Burst-read 14 bytes from ACCEL_XOUT_H and convert big-endian pairs to int16. */
mpu6050_status_t mpu6050_read_raw(mpu6050_t *dev, mpu6050_raw_t *out);

/* Convert a 14-byte burst buffer (big-endian register pairs) to raw values. */
void mpu6050_parse_burst(const uint8_t buf[MPU6050_BURST_LEN], mpu6050_raw_t *out);

#endif /* MPU6050_H */
