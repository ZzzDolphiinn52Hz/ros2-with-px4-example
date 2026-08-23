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
