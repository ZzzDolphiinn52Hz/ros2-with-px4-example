import socket

import pytest

from px4_uavcup_perception.cameras.picamera2_protocol import (
    HEADER,
    pack_header,
    receive_exact,
    unpack_header,
)


def test_frame_header_round_trip():
    header = pack_header(640, 480)
    assert len(header) == HEADER.size
    assert unpack_header(header) == (640, 480, 640 * 480 * 3)


def test_frame_header_rejects_invalid_values():
    with pytest.raises(ValueError):
        pack_header(0, 480)
    with pytest.raises(ValueError):
        unpack_header(b'bad')


def test_receive_exact_handles_multiple_stream_chunks():
    sender, receiver = socket.socketpair()
    try:
        sender.sendall(b'abc')
        sender.sendall(b'def')
        assert receive_exact(receiver, 6) == b'abcdef'
    finally:
        sender.close()
        receiver.close()
