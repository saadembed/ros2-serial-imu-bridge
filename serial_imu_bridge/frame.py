"""Framing for the UART link between the microcontroller and the ROS 2 node.

Frame layout (see README for the full table):

    0xAA 0x55 | length (1 byte) | seq (1 byte) | payload (length bytes) | CRC-16 (2 bytes, big endian)

The CRC covers length, seq and payload. This module has no ROS dependency, so
it can be unit tested with plain pytest and mirrored one-to-one in the firmware.
"""
from dataclasses import dataclass
from typing import List

SYNC = b"\xAA\x55"
HEADER_LEN = 4  # sync (2) + length (1) + seq (1)
CRC_LEN = 2
MAX_PAYLOAD = 64


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE: polynomial 0x1021, init 0xFFFF, no reflection."""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def encode_frame(seq: int, payload: bytes) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload too long: {len(payload)} > {MAX_PAYLOAD}")
    body = bytes([len(payload), seq & 0xFF]) + payload
    return SYNC + body + crc16_ccitt(body).to_bytes(CRC_LEN, "big")


@dataclass(frozen=True)
class Frame:
    seq: int
    payload: bytes


class FrameParser:
    """Incremental parser: feed it whatever bytes the serial port returned.

    It resynchronises after garbage or a bad CRC and counts what it saw, so the
    ROS node can publish link health as diagnostics.
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self._last_seq = None
        self.frames_ok = 0
        self.crc_errors = 0
        self.resyncs = 0
        self.lost_frames = 0

    def reset_link(self) -> None:
        """Forget buffered bytes and the last sequence number (e.g. after a reconnect),
        so the gap across a reconnect is not counted as lost frames. Counters are kept."""
        self._buf.clear()
        self._last_seq = None

    def feed(self, data: bytes) -> List[Frame]:
        self._buf.extend(data)
        frames: List[Frame] = []
        while True:
            start = self._buf.find(SYNC)
            if start < 0:
                # A lone 0xAA at the end may be the first half of the next sync word.
                keep = 1 if self._buf.endswith(SYNC[:1]) else 0
                if len(self._buf) > keep:
                    self.resyncs += 1
                del self._buf[: len(self._buf) - keep]
                break
            if start > 0:
                self.resyncs += 1
                del self._buf[:start]
            if len(self._buf) < HEADER_LEN:
                break
            length = self._buf[2]
            if length > MAX_PAYLOAD:
                self.resyncs += 1
                del self._buf[:1]
                continue
            total = HEADER_LEN + length + CRC_LEN
            if len(self._buf) < total:
                break
            body = bytes(self._buf[2 : HEADER_LEN + length])
            received = int.from_bytes(self._buf[HEADER_LEN + length : total], "big")
            if crc16_ccitt(body) != received:
                self.crc_errors += 1
                del self._buf[:1]  # skip this sync byte and look for the next one
                continue
            seq = self._buf[3]
            payload = bytes(self._buf[HEADER_LEN : HEADER_LEN + length])
            del self._buf[:total]
            if self._last_seq is not None:
                self.lost_frames += (seq - self._last_seq - 1) & 0xFF
            self._last_seq = seq
            self.frames_ok += 1
            frames.append(Frame(seq=seq, payload=payload))
        return frames
