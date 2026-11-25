import math

import pytest

from serial_imu_bridge.imu import (
    ACCEL_LSB_PER_G,
    GYRO_LSB_PER_DPS,
    STANDARD_GRAVITY,
    decode_imu,
    encode_raw,
)


def test_one_g_on_z():
    ax, ay, az, *_ = decode_imu(encode_raw(0, 0, int(ACCEL_LSB_PER_G), 0, 0, 0))
    assert (ax, ay) == (0.0, 0.0)
    assert az == pytest.approx(STANDARD_GRAVITY)


def test_gyro_is_converted_to_rad_per_second():
    *_, gz = decode_imu(encode_raw(0, 0, 0, 0, 0, int(GYRO_LSB_PER_DPS * 90)))
    assert gz == pytest.approx(math.radians(90), rel=1e-3)


def test_wrong_payload_size_raises():
    with pytest.raises(ValueError):
        decode_imu(b"\x00\x01")
