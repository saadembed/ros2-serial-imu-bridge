/*
 * Virtual board: runs the real firmware code (drivers/mpu6050.c and
 * protocol/frame.c) on Linux against a simulated MPU-6050, and writes the
 * frames to a serial device (for example one end of a socat pty pair).
 *
 * This is software-in-the-loop: the driver and frame encoder are the same C
 * files that go on the STM32, but the HAL, the I2C peripheral and the UART are
 * replaced by a register model and a file descriptor.
 *
 * Usage:
 *   virtual_board --port /tmp/ttyMCU [--rate 100] [--roll 20] [--pitch -10] [--seconds 0]
 *
 * The simulated sensor is held at a fixed roll and pitch (degrees), so the
 * host can check that its filter converges to those angles.
 */
#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "frame.h"
#include "mpu6050.h"

#define SIM_PI 3.14159265358979323846
#define ACCEL_LSB_PER_G 16384.0   /* AFS_SEL = 0, +/-2 g */
#define PWR_MGMT_1_SLEEP 0x40u
#define TEMP_OUT_H 0x41u
#define GYRO_XOUT_H 0x43u

typedef struct {
    uint8_t regs[128];
    double roll_rad;
    double pitch_rad;
    uint32_t noise_state;
} sim_mpu_t;

static volatile sig_atomic_t stop_requested = 0;

static void on_signal(int sig)
{
    (void)sig;
    stop_requested = 1;
}

/* Small deterministic noise, about +/-16 LSB, so the data is not perfectly flat. */
static int noise(sim_mpu_t *m)
{
    m->noise_state = m->noise_state * 1664525u + 1013904223u;
    return (int)((m->noise_state >> 24) & 0x1Fu) - 16;
}

static int16_t sat16(double v)
{
    if (v > 32767.0) return 32767;
    if (v < -32768.0) return -32768;
    return (int16_t)lround(v);
}

static void put_be16(uint8_t *p, int16_t v)
{
    uint16_t u = (uint16_t)v;
    p[0] = (uint8_t)(u >> 8);
    p[1] = (uint8_t)(u & 0xFFu);
}

/* Refresh ACCEL/TEMP/GYRO registers from the tilt, as the sensor would on each sample. */
static void update_data_registers(sim_mpu_t *m)
{
    uint8_t *r = m->regs;
    if (r[MPU6050_REG_PWR_MGMT_1] & PWR_MGMT_1_SLEEP) {
        memset(&r[MPU6050_REG_ACCEL_XOUT_H], 0, MPU6050_BURST_LEN);
        return;
    }
    /* Gravity in the sensor frame for roll phi and pitch theta: g * [-sin(theta), sin(phi)cos(theta), cos(phi)cos(theta)]. */
    double ax = -sin(m->pitch_rad) * ACCEL_LSB_PER_G;
    double ay = sin(m->roll_rad) * cos(m->pitch_rad) * ACCEL_LSB_PER_G;
    double az = cos(m->roll_rad) * cos(m->pitch_rad) * ACCEL_LSB_PER_G;
    put_be16(&r[MPU6050_REG_ACCEL_XOUT_H + 0], sat16(ax + noise(m)));
    put_be16(&r[MPU6050_REG_ACCEL_XOUT_H + 2], sat16(ay + noise(m)));
    put_be16(&r[MPU6050_REG_ACCEL_XOUT_H + 4], sat16(az + noise(m)));
    put_be16(&r[TEMP_OUT_H], 0);                       /* about 36.5 deg C */
    put_be16(&r[GYRO_XOUT_H + 0], sat16(noise(m) / 4.0)); /* sensor at rest */
    put_be16(&r[GYRO_XOUT_H + 2], sat16(noise(m) / 4.0));
    put_be16(&r[GYRO_XOUT_H + 4], sat16(noise(m) / 4.0));
}

static int sim_write(void *ctx, uint8_t addr, uint8_t reg, const uint8_t *data, size_t len)
{
    sim_mpu_t *m = ctx;
    if (addr != MPU6050_ADDR_AD0_LOW || (size_t)reg + len > sizeof m->regs) return -1;
    memcpy(&m->regs[reg], data, len);
    return 0;
}

static int sim_read(void *ctx, uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
    sim_mpu_t *m = ctx;
    if (addr != MPU6050_ADDR_AD0_LOW || (size_t)reg + len > sizeof m->regs) return -1;
    if (reg == MPU6050_REG_ACCEL_XOUT_H) update_data_registers(m);
    memcpy(data, &m->regs[reg], len);
    return 0;
}

static int write_all(int fd, const uint8_t *buf, size_t len)
{
    while (len > 0) {
        ssize_t n = write(fd, buf, len);
        if (n < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        buf += n;
        len -= (size_t)n;
    }
    return 0;
}

static void usage(const char *prog)
{
    fprintf(stderr, "usage: %s --port PATH [--rate HZ] [--roll DEG] [--pitch DEG] [--seconds N]\n", prog);
}

int main(int argc, char **argv)
{
    const char *port = NULL;
    double rate_hz = 100.0, roll_deg = 0.0, pitch_deg = 0.0, seconds = 0.0;
    for (int i = 1; i < argc; i++) {
        if (i + 1 >= argc) { usage(argv[0]); return 2; }
        if (strcmp(argv[i], "--port") == 0) port = argv[++i];
        else if (strcmp(argv[i], "--rate") == 0) rate_hz = atof(argv[++i]);
        else if (strcmp(argv[i], "--roll") == 0) roll_deg = atof(argv[++i]);
        else if (strcmp(argv[i], "--pitch") == 0) pitch_deg = atof(argv[++i]);
        else if (strcmp(argv[i], "--seconds") == 0) seconds = atof(argv[++i]);
        else { usage(argv[0]); return 2; }
    }
    if (port == NULL || rate_hz <= 0.0) { usage(argv[0]); return 2; }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGPIPE, SIG_IGN);

    int fd = open(port, O_WRONLY | O_NOCTTY);
    if (fd < 0) { fprintf(stderr, "cannot open %s: %s\n", port, strerror(errno)); return 1; }

    sim_mpu_t sim;
    memset(&sim, 0, sizeof sim);
    sim.regs[MPU6050_REG_WHO_AM_I] = MPU6050_WHO_AM_I_VALUE;
    sim.regs[MPU6050_REG_PWR_MGMT_1] = PWR_MGMT_1_SLEEP;   /* power-on default: asleep */
    sim.roll_rad = roll_deg * SIM_PI / 180.0;
    sim.pitch_rad = pitch_deg * SIM_PI / 180.0;
    sim.noise_state = 12345u;

    const mpu6050_bus_t bus = { sim_write, sim_read, &sim };
    mpu6050_t imu;
    if (mpu6050_init(&imu, &bus, MPU6050_ADDR_AD0_LOW) != MPU6050_OK) {
        fprintf(stderr, "mpu6050_init failed\n");
        close(fd);
        return 1;
    }

    const long period_ns = (long)(1e9 / rate_hz);
    const unsigned long max_frames = seconds > 0.0 ? (unsigned long)(seconds * rate_hz) : 0ul;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    uint8_t seq = 0;
    unsigned long sent = 0;

    fprintf(stderr, "virtual_board: %s at %.0f Hz, roll %.1f deg, pitch %.1f deg\n",
            port, rate_hz, roll_deg, pitch_deg);
    while (!stop_requested && (max_frames == 0 || sent < max_frames)) {
        mpu6050_raw_t s;
        if (mpu6050_read_raw(&imu, &s) == MPU6050_OK) {
            const int16_t v[6] = { s.ax, s.ay, s.az, s.gx, s.gy, s.gz };
            uint8_t payload[IMU_PAYLOAD_LEN];
            uint8_t frame[IMU_FRAME_LEN];
            imu_payload_pack(v, payload, sizeof payload);
            size_t n = frame_encode(seq, payload, IMU_PAYLOAD_LEN, frame, sizeof frame);
            if (n == 0 || write_all(fd, frame, n) != 0) {
                fprintf(stderr, "virtual_board: write failed: %s\n", strerror(errno));
                break;
            }
            sent++;
        }
        seq++;   /* uint8_t: wraps 255 -> 0, like the firmware */

        next.tv_nsec += period_ns;
        while (next.tv_nsec >= 1000000000L) { next.tv_nsec -= 1000000000L; next.tv_sec++; }
        while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL) == EINTR && !stop_requested) {
        }
    }
    fprintf(stderr, "virtual_board: sent %lu frames\n", sent);
    close(fd);
    return 0;
}
