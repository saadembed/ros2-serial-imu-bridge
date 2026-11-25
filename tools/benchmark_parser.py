"""Throughput of FrameParser.feed on this machine (no ROS needed).

This measures the host-side Python parser only. It says nothing about the robot,
the UART, USB-serial latency or the ROS executor. Run from the repo root:

    python tools/benchmark_parser.py [--frames N] [--repeat R]
"""
import argparse
import os
import platform
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from serial_imu_bridge.frame import FrameParser, crc16_ccitt, encode_frame  # noqa: E402
from serial_imu_bridge.imu import encode_raw  # noqa: E402


def make_stream(n: int) -> bytes:
    return b"".join(encode_frame(i & 0xFF, encode_raw(i % 30000, -(i % 30000), 16384, 3, -3, i % 100)) for i in range(n))


def bench(stream: bytes, n: int, chunk: int, repeat: int) -> float:
    """Best-of-`repeat` frames per second, feeding `chunk` bytes per call."""
    best = 0.0
    chunks = [stream[i : i + chunk] for i in range(0, len(stream), chunk)]
    for _ in range(repeat):
        parser = FrameParser()
        t0 = time.perf_counter()
        for c in chunks:
            parser.feed(c)
        elapsed = time.perf_counter() - t0
        assert parser.frames_ok == n and parser.crc_errors == 0, "parser lost frames during benchmark"
        best = max(best, n / elapsed)
    return best


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--frames", type=int, default=100_000)
    ap.add_argument("--repeat", type=int, default=5)
    args = ap.parse_args()

    stream = make_stream(args.frames)
    frame_len = len(stream) // args.frames
    print(f"python {platform.python_version()} ({platform.python_implementation()}), "
          f"{platform.system()} {platform.release()}, {platform.machine()}")
    print(f"{args.frames} IMU frames of {frame_len} bytes, best of {args.repeat}")

    for chunk, label in [(frame_len, "one frame per feed()"), (64, "64-byte reads"), (4096, "4096-byte reads")]:
        fps = bench(stream, args.frames, chunk, args.repeat)
        print(f"  {label:22s} {fps:12,.0f} frames/s  ({1e6 / fps:6.2f} us/frame)")

    body = stream[2 : 2 + 2 + 12]
    crc_us = float("inf")
    for _ in range(args.repeat):
        t0 = time.perf_counter()
        for _ in range(args.frames):
            crc16_ccitt(body)
        crc_us = min(crc_us, (time.perf_counter() - t0) / args.frames * 1e6)
    print(f"  crc16_ccitt alone      {crc_us:6.2f} us/frame (bitwise, no lookup table)")

    # Reference point, derived not measured: 8N1 = 10 bits on the wire per byte.
    link_fps = 115200 / 10 / frame_len
    print(f"  for scale: a 115200 baud 8N1 link carries at most {link_fps:,.0f} frames/s of this size")


if __name__ == "__main__":
    main()
