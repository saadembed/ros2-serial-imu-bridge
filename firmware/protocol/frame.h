/*
 * UART frame encoder, mirror of serial_imu_bridge/frame.py and imu.py.
 *
 *   0xAA 0x55 | length (1) | seq (1) | payload (length) | CRC-16 (2, big endian)
 *
 * CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no xorout)
 * over length, seq and payload. Portable C99, no HAL dependency.
 */
#ifndef FRAME_H
#define FRAME_H

#include <stddef.h>
#include <stdint.h>

#define FRAME_SYNC0        0xAAu
#define FRAME_SYNC1        0x55u
#define FRAME_HEADER_LEN   4u   /* sync (2) + length (1) + seq (1) */
#define FRAME_CRC_LEN      2u
#define FRAME_MAX_PAYLOAD  64u  /* must match MAX_PAYLOAD in frame.py */
#define FRAME_OVERHEAD     (FRAME_HEADER_LEN + FRAME_CRC_LEN)
#define FRAME_MAX_LEN      (FRAME_OVERHEAD + FRAME_MAX_PAYLOAD)

#define IMU_PAYLOAD_LEN    12u  /* six int16: ax, ay, az, gx, gy, gz */
#define IMU_FRAME_LEN      (FRAME_OVERHEAD + IMU_PAYLOAD_LEN)

/* CRC-16/CCITT-FALSE. Check value: crc16_ccitt("123456789", 9) == 0x29B1. */
uint16_t crc16_ccitt(const uint8_t *data, size_t len);

/*
 * Encode one frame into out. Returns the number of bytes written, or 0 if
 * out is NULL, payload is NULL with len > 0, len > FRAME_MAX_PAYLOAD, or
 * out_cap is too small.
 */
size_t frame_encode(uint8_t seq, const uint8_t *payload, uint8_t len,
                    uint8_t *out, size_t out_cap);

/*
 * Pack six int16 values (ax, ay, az, gx, gy, gz) as little-endian into out.
 * Returns IMU_PAYLOAD_LEN, or 0 if out is NULL or out_cap < IMU_PAYLOAD_LEN.
 */
size_t imu_payload_pack(const int16_t values[6], uint8_t *out, size_t out_cap);

#endif /* FRAME_H */
