#include "mpu6050.h"
#include "test_harness.h"

/* Fake I2C bus: a 128-byte register file plus a log of writes. */
typedef struct {
    uint8_t regs[128];
    uint8_t expected_addr;
    int fail_read;       /* make every read fail */
    int fail_write;      /* make every write fail */
    int reads;
    int writes;
    uint8_t write_reg_log[16];
    uint8_t write_val_log[16];
    uint8_t last_read_reg;
    size_t last_read_len;
    int wrong_addr;      /* set if a call used a different device address */
} fake_bus_t;

static int fake_write(void *ctx, uint8_t addr, uint8_t reg, const uint8_t *data, size_t len)
{
    fake_bus_t *f = (fake_bus_t *)ctx;
    size_t i;
    if (addr != f->expected_addr) f->wrong_addr = 1;
    if (f->fail_write) return -1;
    for (i = 0; i < len; i++) {
        if (f->writes < 16) {
            f->write_reg_log[f->writes] = (uint8_t)(reg + i);
            f->write_val_log[f->writes] = data[i];
        }
        f->writes++;
        f->regs[(reg + i) & 0x7F] = data[i];
    }
    return 0;
}

static int fake_read(void *ctx, uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
    fake_bus_t *f = (fake_bus_t *)ctx;
    size_t i;
    if (addr != f->expected_addr) f->wrong_addr = 1;
    f->reads++;
    f->last_read_reg = reg;
    f->last_read_len = len;
    if (f->fail_read) return -1;
    for (i = 0; i < len; i++) {
        data[i] = f->regs[(reg + i) & 0x7F];  /* auto-increment like the chip */
    }
    return 0;
}

static void fake_setup(fake_bus_t *f, mpu6050_bus_t *bus)
{
    memset(f, 0, sizeof *f);
    f->expected_addr = MPU6050_ADDR_AD0_LOW;
    f->regs[MPU6050_REG_WHO_AM_I] = 0x68;
    f->regs[MPU6050_REG_PWR_MGMT_1] = 0x40;   /* power-on reset: SLEEP set */
    f->regs[MPU6050_REG_GYRO_CONFIG] = 0x18;  /* garbage to be overwritten */
    f->regs[MPU6050_REG_ACCEL_CONFIG] = 0x18;
    bus->write_reg = fake_write;
    bus->read_reg = fake_read;
    bus->ctx = f;
}

static void test_register_addresses(void)
{
    CHECK_EQ_INT(MPU6050_REG_WHO_AM_I, 0x75);
    CHECK_EQ_INT(MPU6050_REG_PWR_MGMT_1, 0x6B);
    CHECK_EQ_INT(MPU6050_REG_GYRO_CONFIG, 0x1B);
    CHECK_EQ_INT(MPU6050_REG_ACCEL_CONFIG, 0x1C);
    CHECK_EQ_INT(MPU6050_REG_ACCEL_XOUT_H, 0x3B);
}

static void test_init_ok_wakes_and_configures(void)
{
    fake_bus_t f;
    mpu6050_bus_t bus;
    mpu6050_t dev;
    fake_setup(&f, &bus);
    CHECK_EQ_INT(mpu6050_init(&dev, &bus, MPU6050_ADDR_AD0_LOW), MPU6050_OK);
    CHECK_EQ_INT(f.writes, 3);
    CHECK_EQ_INT(f.write_reg_log[0], 0x6B);   /* wake first */
    CHECK_EQ_INT(f.write_val_log[0], 0x01);   /* SLEEP=0, CLKSEL=PLL X gyro */
    CHECK_EQ_INT(f.write_reg_log[1], 0x1B);
    CHECK_EQ_INT(f.write_val_log[1], 0x00);   /* +/-250 deg/s */
    CHECK_EQ_INT(f.write_reg_log[2], 0x1C);
    CHECK_EQ_INT(f.write_val_log[2], 0x00);   /* +/-2 g */
    CHECK_EQ_INT(f.regs[0x6B] & 0x40, 0);      /* SLEEP bit cleared */
    CHECK_EQ_INT(f.wrong_addr, 0);
}

static void test_init_who_am_i_mismatch(void)
{
    fake_bus_t f;
    mpu6050_bus_t bus;
    mpu6050_t dev;
    fake_setup(&f, &bus);
    f.regs[MPU6050_REG_WHO_AM_I] = 0x70;  /* e.g. an MPU-6500 or wrong chip */
    CHECK_EQ_INT(mpu6050_init(&dev, &bus, MPU6050_ADDR_AD0_LOW), MPU6050_ERR_WHO_AM_I);
    CHECK_EQ_INT(f.writes, 0);            /* nothing written to a wrong device */
    CHECK_EQ_INT(f.last_read_reg, 0x75);
}

static void test_init_bus_errors(void)
{
    fake_bus_t f;
    mpu6050_bus_t bus;
    mpu6050_t dev;

    fake_setup(&f, &bus);
    f.fail_read = 1;
    CHECK_EQ_INT(mpu6050_init(&dev, &bus, MPU6050_ADDR_AD0_LOW), MPU6050_ERR_BUS);
    CHECK_EQ_INT(f.writes, 0);

    fake_setup(&f, &bus);
    f.fail_write = 1;
    CHECK_EQ_INT(mpu6050_init(&dev, &bus, MPU6050_ADDR_AD0_LOW), MPU6050_ERR_BUS);

    CHECK_EQ_INT(mpu6050_init(NULL, &bus, MPU6050_ADDR_AD0_LOW), MPU6050_ERR_ARG);
    CHECK_EQ_INT(mpu6050_init(&dev, NULL, MPU6050_ADDR_AD0_LOW), MPU6050_ERR_ARG);
}

static void test_read_raw_big_endian_and_sign(void)
{
    fake_bus_t f;
    mpu6050_bus_t bus;
    mpu6050_t dev;
    mpu6050_raw_t s;
    /* ACCEL_XOUT_H .. GYRO_ZOUT_L, big endian register pairs. */
    const uint8_t burst[14] = {
        0x40, 0x00,  /* ax =  16384 (1 g)   */
        0xFF, 0xFF,  /* ay =     -1         */
        0x80, 0x00,  /* az = -32768         */
        0xF2, 0x30,  /* temp = -3536        */
        0x7F, 0xFF,  /* gx =  32767         */
        0xFF, 0x7D,  /* gy =   -131 (-1 dps) */
        0x01, 0x02   /* gz =    258         */
    };
    fake_setup(&f, &bus);
    CHECK_EQ_INT(mpu6050_init(&dev, &bus, MPU6050_ADDR_AD0_LOW), MPU6050_OK);
    memcpy(&f.regs[MPU6050_REG_ACCEL_XOUT_H], burst, sizeof burst);

    CHECK_EQ_INT(mpu6050_read_raw(&dev, &s), MPU6050_OK);
    CHECK_EQ_INT(f.last_read_reg, 0x3B);
    CHECK_EQ_INT(f.last_read_len, 14);
    CHECK_EQ_INT(s.ax, 16384);
    CHECK_EQ_INT(s.ay, -1);
    CHECK_EQ_INT(s.az, -32768);
    CHECK_EQ_INT(s.temp, -3536);
    CHECK_EQ_INT(s.gx, 32767);
    CHECK_EQ_INT(s.gy, -131);
    CHECK_EQ_INT(s.gz, 258);
}

static void test_read_raw_bus_error(void)
{
    fake_bus_t f;
    mpu6050_bus_t bus;
    mpu6050_t dev;
    mpu6050_raw_t s;
    fake_setup(&f, &bus);
    CHECK_EQ_INT(mpu6050_init(&dev, &bus, MPU6050_ADDR_AD0_LOW), MPU6050_OK);
    f.fail_read = 1;
    CHECK_EQ_INT(mpu6050_read_raw(&dev, &s), MPU6050_ERR_BUS);
    CHECK_EQ_INT(mpu6050_read_raw(&dev, NULL), MPU6050_ERR_ARG);
}

int main(void)
{
    RUN_TEST(test_register_addresses);
    RUN_TEST(test_init_ok_wakes_and_configures);
    RUN_TEST(test_init_who_am_i_mismatch);
    RUN_TEST(test_init_bus_errors);
    RUN_TEST(test_read_raw_big_endian_and_sign);
    RUN_TEST(test_read_raw_bus_error);
    return TEST_SUMMARY("test_mpu6050");
}
