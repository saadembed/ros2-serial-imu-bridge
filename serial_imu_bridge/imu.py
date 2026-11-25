"""IMU payload encoding and decoding (no ROS dependency).

Payload: six little-endian int16 values: ax, ay, az, gx, gy, gz.
The scale factors match an MPU-6050 at +/-2 g and +/-250 deg/s. Change them
here if your sensor is configured differently.
"""
import math
import struct
from typing import Tuple

PAYLOAD_FORMAT = "<6h"
PAYLOAD_SIZE = struct.calcsize(PAYLOAD_FORMAT)
ACCEL_LSB_PER_G = 16384.0
GYRO_LSB_PER_DPS = 131.0
STANDARD_GRAVITY = 9.80665

Sample = Tuple[float, float, float, float, float, float]


def encode_raw(ax: int, ay: int, az: int, gx: int, gy: int, gz: int) -> bytes:
    return struct.pack(PAYLOAD_FORMAT, ax, ay, az, gx, gy, gz)


def decode_imu(payload: bytes) -> Sample:
    """Return (ax, ay, az) in m/s^2 followed by (gx, gy, gz) in rad/s."""
    if len(payload) != PAYLOAD_SIZE:
        raise ValueError(f"expected {PAYLOAD_SIZE} bytes, got {len(payload)}")
    ax, ay, az, gx, gy, gz = struct.unpack(PAYLOAD_FORMAT, payload)
    accel = tuple(v / ACCEL_LSB_PER_G * STANDARD_GRAVITY for v in (ax, ay, az))
    gyro = tuple(math.radians(v / GYRO_LSB_PER_DPS) for v in (gx, gy, gz))
    return accel + gyro
