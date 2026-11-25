#include "mpu6050.h"

static int16_t be16(const uint8_t *p)
{
    /* Build the 16-bit pattern unsigned, then reinterpret as two's complement
     * without relying on implementation-defined signed conversion. */
    uint16_t u = (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
    return (u & 0x8000u) ? (int16_t)(-(int32_t)(0x10000u - u)) : (int16_t)u;
}

static mpu6050_status_t write_u8(mpu6050_t *dev, uint8_t reg, uint8_t value)
{
    if (dev->bus.write_reg(dev->bus.ctx, dev->addr, reg, &value, 1) != 0) {
        return MPU6050_ERR_BUS;
    }
    return MPU6050_OK;
}

mpu6050_status_t mpu6050_init(mpu6050_t *dev, const mpu6050_bus_t *bus, uint8_t addr)
{
    uint8_t who = 0;
    mpu6050_status_t st;

    if (dev == NULL || bus == NULL || bus->read_reg == NULL || bus->write_reg == NULL) {
        return MPU6050_ERR_ARG;
    }
    dev->bus = *bus;
    dev->addr = addr;

    if (dev->bus.read_reg(dev->bus.ctx, addr, MPU6050_REG_WHO_AM_I, &who, 1) != 0) {
        return MPU6050_ERR_BUS;
    }
    /* WHO_AM_I reports 0x68 regardless of the AD0 pin. */
    if (who != MPU6050_WHO_AM_I_VALUE) {
        return MPU6050_ERR_WHO_AM_I;
    }

    st = write_u8(dev, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_MGMT_1_WAKE);
    if (st != MPU6050_OK) {
        return st;
    }
    st = write_u8(dev, MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_FS_250);
    if (st != MPU6050_OK) {
        return st;
    }
    return write_u8(dev, MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_FS_2G);
}

void mpu6050_parse_burst(const uint8_t buf[MPU6050_BURST_LEN], mpu6050_raw_t *out)
{
    out->ax = be16(&buf[0]);
    out->ay = be16(&buf[2]);
    out->az = be16(&buf[4]);
    out->temp = be16(&buf[6]);  /* TEMP_OUT sits between accel and gyro */
    out->gx = be16(&buf[8]);
    out->gy = be16(&buf[10]);
    out->gz = be16(&buf[12]);
}

mpu6050_status_t mpu6050_read_raw(mpu6050_t *dev, mpu6050_raw_t *out)
{
    uint8_t buf[MPU6050_BURST_LEN];

    if (dev == NULL || out == NULL) {
        return MPU6050_ERR_ARG;
    }
    if (dev->bus.read_reg(dev->bus.ctx, dev->addr, MPU6050_REG_ACCEL_XOUT_H,
                          buf, sizeof buf) != 0) {
        return MPU6050_ERR_BUS;
    }
    mpu6050_parse_burst(buf, out);
    return MPU6050_OK;
}
