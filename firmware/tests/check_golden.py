"""Cross-language golden check: frames written by the C firmware code
(gen_frames.c) must decode with the Python parser used by the ROS node.

Usage (from the repo root, after building gen_frames):
    python firmware/tests/check_golden.py firmware/build/golden_frames.bin
"""
import math
import os
import struct
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from serial_imu_bridge.frame import FrameParser  # noqa: E402
from serial_imu_bridge.imu import (  # noqa: E402
    ACCEL_LSB_PER_G,
    GYRO_LSB_PER_DPS,
    PAYLOAD_SIZE,
    STANDARD_GRAVITY,
    decode_imu,
)

# Must match SAMPLES / FIRST_SEQ in gen_frames.c.
FIRST_SEQ = 254
EXPECTED = [
    (0, 0, 16384, 0, 0, 0),
    (-1638, 1, -16384, -131, 131, -1),
    (-32768, 32767, -1, 32767, -32768, 258),
    (100, -200, 300, -400, 500, -600),
]
FRAME_LEN = 4 + PAYLOAD_SIZE + 2


def to_si(raw):
    a = [v / ACCEL_LSB_PER_G * STANDARD_GRAVITY for v in raw[:3]]
    g = [math.radians(v / GYRO_LSB_PER_DPS) for v in raw[3:]]
    return a + g


def check(data: bytes, chunk: int) -> int:
    parser = FrameParser()
    frames = []
    for i in range(0, len(data), chunk):
        frames.extend(parser.feed(data[i : i + chunk]))

    assert len(frames) == len(EXPECTED), f"got {len(frames)} frames"
    assert parser.frames_ok == len(EXPECTED)
    assert parser.crc_errors == 0, parser.crc_errors
    assert parser.resyncs == 0, parser.resyncs
    assert parser.lost_frames == 0, parser.lost_frames  # 254, 255, 0, 1: wrap is not a loss

    checks = 0
    for i, (frame, raw) in enumerate(zip(frames, EXPECTED)):
        assert frame.seq == (FIRST_SEQ + i) & 0xFF, (i, frame.seq)
        assert struct.unpack("<6h", frame.payload) == raw, (i, frame.payload.hex())
        got = decode_imu(frame.payload)
        for g, e in zip(got, to_si(raw)):
            assert math.isclose(g, e, rel_tol=0, abs_tol=1e-9), (i, g, e)
        checks += 1
    return checks


def main() -> int:
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(__file__), "..", "build", "golden_frames.bin")
    with open(path, "rb") as f:
        data = f.read()
    assert len(data) == FRAME_LEN * len(EXPECTED), f"file is {len(data)} bytes"

    total = 0
    for chunk in (len(data), 1, 7):  # all at once, byte by byte, odd chunks
        total += check(data, chunk)

    # Spot-check physical values for the 1 g frame.
    first = decode_imu(data[4 : 4 + PAYLOAD_SIZE])
    assert math.isclose(first[2], STANDARD_GRAVITY)

    print(f"check_golden: OK, {len(EXPECTED)} frames from C decoded by "
          f"serial_imu_bridge.frame.FrameParser ({total} frame checks over 3 feed patterns)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
