"""A fake serial source, so the node runs without any hardware.

It emits valid IMU frames at a fixed rate and can corrupt or drop some of them,
which exercises the CRC check and the lost-frame counter.

The clock is injectable (any zero-argument callable returning seconds), so tests
can drive a simulated period without sleeping. The default is time.monotonic.
"""
import math
import random
import time
from typing import Callable

from .frame import encode_frame
from .imu import ACCEL_LSB_PER_G, encode_raw


class SimulatedImuSource:
    def __init__(
        self,
        rate_hz: float = 100.0,
        error_rate: float = 0.02,
        seed: int = 0,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if rate_hz <= 0:
            raise ValueError(f"rate_hz must be > 0, got {rate_hz}")
        self._period = 1.0 / rate_hz
        self._error_rate = error_rate
        self._rng = random.Random(seed)
        self._clock = clock
        self._seq = 0
        self._t0 = clock()
        self._next = self._t0
        # Ground truth for tests: what the "wire" did to each generated frame.
        self.frames_generated = 0
        self.frames_dropped = 0
        self.frames_corrupted = 0

    def read(self) -> bytes:
        """Return all bytes that 'arrived' since the previous call."""
        now = self._clock()
        out = bytearray()
        while self._next <= now:
            t = self._next - self._t0
            self._next += self._period
            seq = self._seq
            self._seq = (self._seq + 1) & 0xFF
            self.frames_generated += 1
            if self._rng.random() < self._error_rate / 2:
                self.frames_dropped += 1
                continue  # frame lost on the wire
            frame = bytearray(encode_frame(seq, self.payload_at(t)))
            if self._rng.random() < self._error_rate / 2:
                frame[self._rng.randrange(len(frame))] ^= 0xFF  # corrupt one byte
                self.frames_corrupted += 1
            out.extend(frame)
        return bytes(out)

    @staticmethod
    def payload_at(t: float) -> bytes:
        """Raw payload the simulator sends for a sample taken t seconds after start."""
        ax = int(0.1 * ACCEL_LSB_PER_G * math.sin(2 * math.pi * 0.5 * t))
        az = int(ACCEL_LSB_PER_G)  # 1 g on z while the sensor sits still
        gz = int(100 * math.sin(2 * math.pi * 0.2 * t))
        return encode_raw(ax, 0, az, 0, 0, gz)
