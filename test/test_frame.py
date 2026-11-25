import pytest

from serial_imu_bridge.frame import MAX_PAYLOAD, FrameParser, crc16_ccitt, encode_frame


def test_crc_known_vector():
    # Standard check value for CRC-16/CCITT-FALSE.
    assert crc16_ccitt(b"123456789") == 0x29B1


def test_roundtrip():
    frames = FrameParser().feed(encode_frame(7, b"hello"))
    assert [(f.seq, f.payload) for f in frames] == [(7, b"hello")]


def test_frame_split_across_reads():
    parser = FrameParser()
    data = encode_frame(1, b"abcdef")
    assert parser.feed(data[:3]) == []
    assert parser.feed(data[3:7]) == []
    assert [f.payload for f in parser.feed(data[7:])] == [b"abcdef"]


def test_garbage_before_frame_is_skipped():
    parser = FrameParser()
    frames = parser.feed(b"\x00\x13\x37" + encode_frame(2, b"ok"))
    assert [f.payload for f in frames] == [b"ok"]
    assert parser.resyncs >= 1


def test_corrupted_crc_is_dropped_and_next_frame_survives():
    parser = FrameParser()
    bad = bytearray(encode_frame(1, b"first"))
    bad[-1] ^= 0xFF
    frames = parser.feed(bytes(bad) + encode_frame(2, b"second"))
    assert [f.payload for f in frames] == [b"second"]
    assert parser.crc_errors == 1


def test_lost_frames_are_counted_with_wraparound():
    parser = FrameParser()
    parser.feed(encode_frame(254, b"a"))
    parser.feed(encode_frame(1, b"b"))  # 255 and 0 are missing
    assert parser.lost_frames == 2


def test_reset_link_does_not_count_the_reconnect_gap_as_lost():
    parser = FrameParser()
    parser.feed(encode_frame(10, b"a") + encode_frame(11, b"b")[:5])  # partial frame buffered
    parser.reset_link()
    frames = parser.feed(encode_frame(200, b"c"))
    assert [f.payload for f in frames] == [b"c"]
    assert parser.lost_frames == 0
    assert parser.frames_ok == 2


def test_oversized_length_field_is_rejected():
    parser = FrameParser()
    frames = parser.feed(b"\xAA\x55\xFF\x00" + encode_frame(3, b"x"))
    assert [f.payload for f in frames] == [b"x"]


def test_encode_rejects_oversized_payload():
    with pytest.raises(ValueError):
        encode_frame(0, bytes(MAX_PAYLOAD + 1))
