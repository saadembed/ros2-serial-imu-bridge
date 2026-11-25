#include "frame.h"

uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    size_t i;
    int bit;

    for (i = 0; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (bit = 0; bit < 8; bit++) {
            if (crc & 0x8000u) {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

size_t frame_encode(uint8_t seq, const uint8_t *payload, uint8_t len,
                    uint8_t *out, size_t out_cap)
{
    size_t total = FRAME_OVERHEAD + (size_t)len;
    size_t i;
    uint16_t crc;

    if (out == NULL || len > FRAME_MAX_PAYLOAD || out_cap < total) {
        return 0;
    }
    if (payload == NULL && len > 0) {
        return 0;
    }

    out[0] = FRAME_SYNC0;
    out[1] = FRAME_SYNC1;
    out[2] = len;
    out[3] = seq;
    for (i = 0; i < len; i++) {
        out[FRAME_HEADER_LEN + i] = payload[i];
    }
    /* CRC over length, seq and payload (bytes 2 .. 3+len). */
    crc = crc16_ccitt(&out[2], 2u + (size_t)len);
    out[FRAME_HEADER_LEN + len] = (uint8_t)(crc >> 8);       /* big endian */
    out[FRAME_HEADER_LEN + len + 1u] = (uint8_t)(crc & 0xFFu);
    return total;
}

size_t imu_payload_pack(const int16_t values[6], uint8_t *out, size_t out_cap)
{
    size_t i;

    if (values == NULL || out == NULL || out_cap < IMU_PAYLOAD_LEN) {
        return 0;
    }
    for (i = 0; i < 6; i++) {
        /* Two's complement bit pattern via uint16_t, then explicit LE bytes. */
        uint16_t u = (uint16_t)values[i];
        out[2 * i] = (uint8_t)(u & 0xFFu);
        out[2 * i + 1] = (uint8_t)(u >> 8);
    }
    return IMU_PAYLOAD_LEN;
}
