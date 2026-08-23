"""Conversions between ROS Image messages and NumPy arrays.

The package intentionally does not depend on cv_bridge. This keeps the Gazebo
bridge usable even before OpenCV and the neural-network environment are ready.
"""

from __future__ import annotations

import sys
from typing import Optional

import numpy as np
from sensor_msgs.msg import Image


_COLOR_CHANNELS = {
    'mono8': 1,
    'rgb8': 3,
    'bgr8': 3,
    'rgba8': 4,
    'bgra8': 4,
}


def ros_image_to_array(message: Image) -> np.ndarray:
    """Convert a supported ROS image into an independent NumPy array."""
    height = int(message.height)
    width = int(message.width)
    step = int(message.step)
    if height <= 0 or width <= 0 or step <= 0:
        raise ValueError('Image dimensions and step must be positive')
    if len(message.data) < height * step:
        raise ValueError('Image data is shorter than height * step')

    if message.encoding == '32FC1':
        byte_order = '>' if message.is_bigendian else '<'
        array = np.ndarray(
            shape=(height, width),
            dtype=np.dtype(byte_order + 'f4'),
            buffer=bytes(message.data),
            strides=(step, 4),
        )
        return array.astype(np.float32, copy=True)

    channels = _COLOR_CHANNELS.get(message.encoding)
    if channels is None:
        raise ValueError(f'Unsupported image encoding: {message.encoding}')
    row_bytes = width * channels
    if step < row_bytes:
        raise ValueError('Image step is smaller than the encoded row width')
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(height, step)
    pixels = rows[:, :row_bytes]
    if channels == 1:
        return pixels.reshape(height, width).copy()
    return pixels.reshape(height, width, channels).copy()


def ros_image_to_bgr(message: Image) -> np.ndarray:
    """Convert common 8-bit ROS encodings to DA-V2's BGR input."""
    image = ros_image_to_array(message)
    if message.encoding == 'bgr8':
        return image
    if message.encoding == 'rgb8':
        return image[:, :, ::-1].copy()
    if message.encoding == 'bgra8':
        return image[:, :, :3].copy()
    if message.encoding == 'rgba8':
        return image[:, :, :3][:, :, ::-1].copy()
    if message.encoding == 'mono8':
        return np.repeat(image[:, :, None], 3, axis=2)
    raise ValueError(
        f'Expected an 8-bit color or mono image, got {message.encoding}')


def array_to_image(
        array: np.ndarray,
        encoding: str,
        source: Optional[Image] = None) -> Image:
    """Create a ROS Image and optionally copy a source image header."""
    output = Image()
    if source is not None:
        output.header = source.header

    contiguous = np.ascontiguousarray(array)
    if encoding == '32FC1':
        if contiguous.ndim != 2:
            raise ValueError('32FC1 requires a two-dimensional array')
        contiguous = contiguous.astype(np.float32, copy=False)
        output.height, output.width = contiguous.shape
        output.step = int(output.width * 4)
        output.is_bigendian = sys.byteorder == 'big'
    elif encoding == 'mono8':
        if contiguous.ndim != 2:
            raise ValueError('mono8 requires a two-dimensional array')
        contiguous = contiguous.astype(np.uint8, copy=False)
        output.height, output.width = contiguous.shape
        output.step = int(output.width)
        output.is_bigendian = False
    else:
        raise ValueError(f'Unsupported output encoding: {encoding}')

    output.encoding = encoding
    output.data = contiguous.tobytes()
    return output


def resize_image_nearest(
        data: bytes,
        width: int,
        height: int,
        step: int,
        channels: int,
        output_width: int,
        output_height: int) -> bytes:
    """Resize an interleaved 8-bit image without requiring OpenCV.

    This function deliberately stays independent of the Gazebo Python
    bindings so its unit tests can run on the Jetson deployment host.
    """
    if min(width, height, step, channels, output_width, output_height) <= 0:
        raise ValueError('Image dimensions, step and channels must be positive')
    row_bytes = width * channels
    if step < row_bytes:
        raise ValueError('Image step is smaller than the encoded row width')
    if len(data) < height * step:
        raise ValueError('Image data is shorter than height * step')

    rows = np.frombuffer(data, dtype=np.uint8).reshape(height, step)
    pixels = rows[:, :row_bytes].reshape(height, width, channels)
    row_indices = np.linspace(0, height - 1, output_height, dtype=np.intp)
    column_indices = np.linspace(0, width - 1, output_width, dtype=np.intp)
    resized = pixels[row_indices][:, column_indices]
    return np.ascontiguousarray(resized).tobytes()
