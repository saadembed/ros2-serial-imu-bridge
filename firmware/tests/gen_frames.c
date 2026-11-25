/*
 * Golden-file generator: writes a few IMU frames to a binary file using the
 * full firmware path (MPU-6050 burst parse -> payload pack -> frame encode).
 * check_golden.py decodes the file with the Python parser.
 *
 * Usage: gen_frames <output.bin>
 * Keep SAMPLES in sync with EXPECTED in check_golden.py.
 */
#include <stdio.h>
#include "frame.h"
#include "mpu6050.h"

/* Raw values ax, ay, az, gx, gy, gz; seq starts at 254 to cover the wrap. */
static const int16_t SAMPLES[4][6] = {
    {      0,      0,  16384,     0,    0,     0 },
    {  -1638,      1, -16384,  -131,  131,    -1 },
    { -32768,  32767,     -1, 32767, -32768,  258 },
    {    100,   -200,    300,  -400,  500,  -600 },
};
#define FIRST_SEQ 254u

static void put_be16(uint8_t *p, int16_t v)
{
    uint16_t u = (uint16_t)v;
    p[0] = (uint8_t)(u >> 8);
    p[1] = (uint8_t)(u & 0xFFu);
}

int main(int argc, char **argv)
{
    FILE *fp;
    size_t i;
    uint8_t seq = (uint8_t)FIRST_SEQ;

    if (argc != 2) {
        fprintf(stderr, "usage: %s <output.bin>\n", argv[0]);
        return 2;
    }
    fp = fopen(argv[1], "wb");
    if (fp == NULL) {
        perror("fopen");
        return 1;
    }
    for (i = 0; i < sizeof SAMPLES / sizeof SAMPLES[0]; i++) {
        uint8_t burst[MPU6050_BURST_LEN];
        mpu6050_raw_t raw;
        int16_t v[6];
        uint8_t payload[IMU_PAYLOAD_LEN];
        uint8_t frame[IMU_FRAME_LEN];
        size_t n;

        /* Simulate the sensor's big-endian register layout, temp in the middle. */
        put_be16(&burst[0], SAMPLES[i][0]);
        put_be16(&burst[2], SAMPLES[i][1]);
        put_be16(&burst[4], SAMPLES[i][2]);
        put_be16(&burst[6], (int16_t)(-1000 - (int)i));  /* temperature, dropped */
        put_be16(&burst[8], SAMPLES[i][3]);
        put_be16(&burst[10], SAMPLES[i][4]);
        put_be16(&burst[12], SAMPLES[i][5]);
        mpu6050_parse_burst(burst, &raw);

        v[0] = raw.ax; v[1] = raw.ay; v[2] = raw.az;
        v[3] = raw.gx; v[4] = raw.gy; v[5] = raw.gz;
        imu_payload_pack(v, payload, sizeof payload);
        n = frame_encode(seq, payload, IMU_PAYLOAD_LEN, frame, sizeof frame);
        if (n != IMU_FRAME_LEN || fwrite(frame, 1, n, fp) != n) {
            fprintf(stderr, "encode/write failed\n");
            fclose(fp);
            return 1;
        }
        seq++;
    }
    fclose(fp);
    printf("wrote %u frames to %s\n", (unsigned)(sizeof SAMPLES / sizeof SAMPLES[0]), argv[1]);
    return 0;
}
