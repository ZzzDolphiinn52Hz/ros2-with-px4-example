"""Convert NumPy output arrays to ROS Image messages without cv_bridge."""

import sys

import numpy as np
from sensor_msgs.msg import Image


_COLOR_CHANNELS = {
    'mono8': 1,
    'rgb8': 3,
    'bgr8': 3,
    'rgba8': 4,
    'bgra8': 4,
}


def image_to_array(message: Image) -> np.ndarray:
    """Convert a common ROS Image encoding to a tightly packed NumPy array."""
    encoding = message.encoding.lower()
    if encoding == '32fc1':
        dtype = np.dtype('>f4' if message.is_bigendian else '<f4')
        channels = 1
    elif encoding == 'mono8':
        dtype = np.dtype(np.uint8)
        channels = 1
    elif encoding in _COLOR_CHANNELS:
        dtype = np.dtype(np.uint8)
        channels = _COLOR_CHANNELS[encoding]
    else:
        raise ValueError(f'Unsupported input encoding: {message.encoding}')

    height = int(message.height)
    width = int(message.width)
    if height <= 0 or width <= 0:
        raise ValueError('Image width and height must be positive')
    row_values = int(message.step) // dtype.itemsize
    required_values = row_values * height
    values = np.frombuffer(message.data, dtype=dtype)
    if values.size < required_values:
        raise ValueError(
            f'Image data has {values.size} values, expected at least '
            f'{required_values}')
    values = values[:required_values].reshape(height, row_values)
    packed_values = width * channels
    if row_values < packed_values:
        raise ValueError('Image step is smaller than its packed row width')
    values = values[:, :packed_values]
    if channels == 1:
        return np.ascontiguousarray(values.reshape(height, width))
    return np.ascontiguousarray(values.reshape(height, width, channels))


def image_to_bgr(message: Image) -> np.ndarray:
    """Convert mono/RGB/BGR ROS images to BGR without cv_bridge."""
    array = image_to_array(message)
    encoding = message.encoding.lower()
    if encoding == 'bgr8':
        return array
    if encoding == 'rgb8':
        return np.ascontiguousarray(array[:, :, ::-1])
    if encoding == 'mono8':
        return np.repeat(array[:, :, None], 3, axis=2)
    if encoding == 'bgra8':
        return np.ascontiguousarray(array[:, :, :3])
    if encoding == 'rgba8':
        return np.ascontiguousarray(array[:, :, [2, 1, 0]])
    raise ValueError(f'Cannot convert {message.encoding} to BGR')


def array_to_image(array: np.ndarray, encoding: str) -> Image:
    """Create a ROS Image from a depth, mono or colour NumPy array."""
    output = Image()

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
    elif encoding in _COLOR_CHANNELS:
        channels = _COLOR_CHANNELS[encoding]
        if contiguous.ndim != 3 or contiguous.shape[2] != channels:
            raise ValueError(
                f'{encoding} requires an HxWx{channels} array')
        contiguous = contiguous.astype(np.uint8, copy=False)
        output.height, output.width, _ = contiguous.shape
        output.step = int(output.width * channels)
        output.is_bigendian = False
    else:
        raise ValueError(f'Unsupported output encoding: {encoding}')

    output.encoding = encoding
    output.data = contiguous.tobytes()
    return output
