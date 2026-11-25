#include "frame.h"
#include "test_harness.h"

static void test_crc_check_value(void)
{
    const char *s = "123456789";
    CHECK_EQ_INT(crc16_ccitt((const uint8_t *)s, strlen(s)), 0x29B1);
}

static void test_crc_empty(void)
{
    /* No data: CRC stays at the init value. */
    CHECK_EQ_INT(crc16_ccitt(NULL, 0), 0xFFFF);
}

static void test_payload_pack_endianness_and_sign(void)
{
    const int16_t v[6] = { 1, -1, 16384, -32768, 32767, -131 };
    const uint8_t expected[12] = {
        0x01, 0x00,  /*      1 */
        0xFF, 0xFF,  /*     -1 */
        0x00, 0x40,  /*  16384 */
        0x00, 0x80,  /* -32768 */
        0xFF, 0x7F,  /*  32767 */
        0x7D, 0xFF   /*   -131 = 0xFF7D */
    };
    uint8_t out[12];
    memset(out, 0xEE, sizeof out);
    CHECK_EQ_INT(imu_payload_pack(v, out, sizeof out), 12);
    CHECK_MEM(out, expected, 12);
}

static void test_payload_pack_errors(void)
{
    const int16_t v[6] = { 0 };
    uint8_t out[12];
    CHECK_EQ_INT(imu_payload_pack(v, out, 11), 0);
    CHECK_EQ_INT(imu_payload_pack(v, NULL, 12), 0);
    CHECK_EQ_INT(imu_payload_pack(NULL, out, 12), 0);
}

static void test_frame_encode_imu(void)
{
    /* Same values as above, seq 7. Expected bytes cross-checked against
     * serial_imu_bridge.frame.encode_frame(7, encode_raw(...)). */
    const int16_t v[6] = { 1, -1, 16384, -32768, 32767, -131 };
    const uint8_t expected[18] = {
        0xAA, 0x55, 0x0C, 0x07,
        0x01, 0x00, 0xFF, 0xFF, 0x00, 0x40, 0x00, 0x80, 0xFF, 0x7F, 0x7D, 0xFF,
        0xB7, 0xB3
    };
    uint8_t payload[12];
    uint8_t out[IMU_FRAME_LEN];
    imu_payload_pack(v, payload, sizeof payload);
    CHECK_EQ_INT(frame_encode(7, payload, 12, out, sizeof out), 18);
    CHECK_MEM(out, expected, 18);
    /* CRC in the frame equals crc16 over length+seq+payload, big endian. */
    {
        uint16_t crc = crc16_ccitt(&out[2], 14);
        CHECK_EQ_INT(out[16], crc >> 8);
        CHECK_EQ_INT(out[17], crc & 0xFF);
    }
}

static void test_frame_encode_small(void)
{
    const uint8_t p[2] = { 0x12, 0x34 };
    const uint8_t expected_small[8] = { 0xAA, 0x55, 0x02, 0x00, 0x12, 0x34, 0x7A, 0x6E };
    const uint8_t expected_empty[6] = { 0xAA, 0x55, 0x00, 0xFF, 0x03, 0xFF };
    uint8_t out[16];
    CHECK_EQ_INT(frame_encode(0, p, 2, out, sizeof out), 8);
    CHECK_MEM(out, expected_small, 8);
    CHECK_EQ_INT(frame_encode(255, NULL, 0, out, sizeof out), 6);
    CHECK_MEM(out, expected_empty, 6);
}

static void test_frame_encode_errors(void)
{
    uint8_t p[FRAME_MAX_PAYLOAD + 1];
    uint8_t out[FRAME_MAX_LEN + 8];
    memset(p, 0, sizeof p);
    /* Too small output buffer (needs 18). */
    CHECK_EQ_INT(frame_encode(0, p, 12, out, 17), 0);
    /* Exactly big enough. */
    CHECK_EQ_INT(frame_encode(0, p, 12, out, 18), 18);
    /* Max payload accepted, one more rejected (parser drops > 64). */
    CHECK_EQ_INT(frame_encode(0, p, FRAME_MAX_PAYLOAD, out, sizeof out), FRAME_MAX_LEN);
    CHECK_EQ_INT(frame_encode(0, p, FRAME_MAX_PAYLOAD + 1, out, sizeof out), 0);
    CHECK_EQ_INT(frame_encode(0, NULL, 1, out, sizeof out), 0);
    CHECK_EQ_INT(frame_encode(0, p, 1, NULL, 100), 0);
}

int main(void)
{
    RUN_TEST(test_crc_check_value);
    RUN_TEST(test_crc_empty);
    RUN_TEST(test_payload_pack_endianness_and_sign);
    RUN_TEST(test_payload_pack_errors);
    RUN_TEST(test_frame_encode_imu);
    RUN_TEST(test_frame_encode_small);
    RUN_TEST(test_frame_encode_errors);
    return TEST_SUMMARY("test_frame");
}
