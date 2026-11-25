"""Simulator -> FrameParser -> decode_imu, without ROS, on a simulated clock."""
import pytest

from serial_imu_bridge.frame import FrameParser
from serial_imu_bridge.imu import PAYLOAD_SIZE, decode_imu
from serial_imu_bridge.simulator import SimulatedImuSource
from serial_imu_bridge.stamping import batch_stamps_ns


class FakeClock:
    def __init__(self) -> None:
        self.t = 100.0

    def __call__(self) -> float:
        return self.t


def run_link(error_rate, seconds=60.0, rate_hz=100.0, poll_hz=200.0, seed=1):
    clock = FakeClock()
    sim = SimulatedImuSource(rate_hz=rate_hz, error_rate=error_rate, seed=seed, clock=clock)
    parser = FrameParser()
    frames = []
    start = clock.t
    # Poll half a poll period off the sample grid so float rounding cannot move a
    # frame across a poll boundary.
    for k in range(int(seconds * poll_hz)):
        clock.t = start + (k + 0.5) / poll_hz
        frames.extend(parser.feed(sim.read()))
    return sim, parser, frames


def unwrap(seqs):
    """8-bit sequence numbers -> frame index since start (assumes < 256 missing in a row)."""
    out, base, prev = [], 0, None
    for s in seqs:
        if prev is not None and s <= prev:
            base += 256
        out.append(base + s)
        prev = s
    return out


def test_clean_link_every_frame_arrives_and_decodes():
    sim, parser, frames = run_link(error_rate=0.0, seconds=60.0)
    assert sim.frames_generated == 6000  # t = 0 .. 59.99 s at 100 Hz
    assert len(frames) == sim.frames_generated
    assert parser.frames_ok == sim.frames_generated
    assert parser.crc_errors == 0
    assert parser.lost_frames == 0
    assert parser.resyncs == 0
    indices = unwrap([f.seq for f in frames])
    assert indices == list(range(sim.frames_generated))
    for i, f in zip(indices, frames):
        assert len(f.payload) == PAYLOAD_SIZE
        assert f.payload == SimulatedImuSource.payload_at(i / 100.0)
        decode_imu(f.payload)  # must not raise


def test_noisy_link_errors_are_counted_and_valid_frames_still_decode():
    sim, parser, frames = run_link(error_rate=0.05, seconds=60.0)
    assert sim.frames_dropped > 0 and sim.frames_corrupted > 0
    # Every frame that left the simulator intact is recovered.
    assert parser.frames_ok == sim.frames_generated - sim.frames_dropped - sim.frames_corrupted
    # A flipped byte inside length/seq/payload/CRC is a CRC error; a flipped sync
    # byte makes the frame invisible (resync) instead. Extra CRC failures can come
    # from false sync words found while rescanning a corrupted frame.
    assert 0 < parser.crc_errors
    assert parser.resyncs > 0
    # Every missing frame (dropped or corrupted) shows up as a sequence gap,
    # except ones at the very start or end of the run, which have no later/earlier frame.
    indices = unwrap([f.seq for f in frames])
    missing_inside = (indices[-1] - indices[0] + 1) - len(indices)
    assert parser.lost_frames == missing_inside
    edge = indices[0] + (sim.frames_generated - 1 - indices[-1])
    assert missing_inside + edge == sim.frames_dropped + sim.frames_corrupted
    for i, f in zip(indices, frames):
        assert f.payload == SimulatedImuSource.payload_at(i / 100.0)
        decode_imu(f.payload)


def test_default_clock_is_still_monotonic_time():
    import time

    sim = SimulatedImuSource()
    assert sim._clock is time.monotonic


def test_batch_stamps_are_back_dated_and_strictly_increasing():
    period = 10_000_000
    assert batch_stamps_ns(1_000_000_000, 3, period) == [980_000_000, 990_000_000, 1_000_000_000]
    # A back-dated stamp never goes before the previous batch's last stamp.
    assert batch_stamps_ns(1_000_000_000, 3, period, last_stamp_ns=985_000_000) == [
        985_000_001,
        990_000_000,
        1_000_000_000,
    ]
    assert batch_stamps_ns(5, 2, period, last_stamp_ns=10) == [11, 12]
    assert batch_stamps_ns(1, 0, period) == []


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_noisy_link_is_deterministic_per_seed(seed):
    a = run_link(error_rate=0.05, seconds=5.0, seed=seed)[1]
    b = run_link(error_rate=0.05, seconds=5.0, seed=seed)[1]
    assert (a.frames_ok, a.crc_errors, a.lost_frames) == (b.frames_ok, b.crc_errors, b.lost_frames)
