"""Length-prefixed RGB frame protocol shared by the Pi host and ROS node."""

from __future__ import annotations

import socket
import struct


MAGIC = b'P2CF'
CHANNELS = 3
HEADER = struct.Struct('!4sHHI')


def pack_header(width: int, height: int) -> bytes:
    """Pack and validate one RGB frame header."""
    if width <= 0 or height <= 0:
        raise ValueError('frame width and height must be positive')
    payload_size = width * height * CHANNELS
    return HEADER.pack(MAGIC, width, height, payload_size)


def unpack_header(data: bytes) -> tuple[int, int, int]:
    """Return width, height and byte count from a validated header."""
    if len(data) != HEADER.size:
        raise ValueError(f'frame header must contain {HEADER.size} bytes')
    magic, width, height, payload_size = HEADER.unpack(data)
    if magic != MAGIC:
        raise ValueError('invalid Picamera2 frame magic')
    expected_size = width * height * CHANNELS
    if width <= 0 or height <= 0 or payload_size != expected_size:
        raise ValueError('invalid Picamera2 frame dimensions or byte count')
    return width, height, payload_size


def receive_exact(connection: socket.socket, size: int) -> bytes:
    """Receive exactly size bytes or raise when the peer disconnects."""
    if size < 0:
        raise ValueError('receive size cannot be negative')
    output = bytearray(size)
    view = memoryview(output)
    received = 0
    while received < size:
        count = connection.recv_into(view[received:])
        if count == 0:
            raise ConnectionError('Picamera2 frame socket disconnected')
        received += count
    return bytes(output)
