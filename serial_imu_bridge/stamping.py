"""Timestamps for frames that arrive together in one serial read (no ROS dependency).

The node reads the UART at `poll_hz`, so one read can return several frames
(USB-serial adapters also batch bytes, e.g. FTDI's 16 ms latency timer). Stamping
all of them with the same read time gives downstream consumers dt = 0 between
samples. Instead the last frame gets the read time and earlier ones are
back-dated by the nominal sample period. Stamps are also kept strictly increasing.

This is still host arrival time, not sample time: transport latency is not
removed. A sensor-side timestamp in the payload would be needed for that.
"""
from typing import List, Optional


def batch_stamps_ns(
    read_time_ns: int, count: int, period_ns: int, last_stamp_ns: Optional[int] = None
) -> List[int]:
    stamps = []
    prev = last_stamp_ns
    for i in range(count):
        stamp = read_time_ns - (count - 1 - i) * period_ns
        if prev is not None and stamp <= prev:
            stamp = prev + 1
        stamps.append(stamp)
        prev = stamp
    return stamps
