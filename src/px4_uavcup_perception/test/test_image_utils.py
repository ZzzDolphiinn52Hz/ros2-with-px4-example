import numpy as np
from sensor_msgs.msg import Image

from px4_uavcup_perception.image_utils import (
    array_to_image,
    resize_image_nearest,
    ros_image_to_array,
    ros_image_to_bgr,
)


def test_rgb_image_converts_to_bgr():
    message = Image()
    message.width = 2
    message.height = 1
    message.step = 6
    message.encoding = 'rgb8'
    message.data = bytes([10, 20, 30, 40, 50, 60])

    converted = ros_image_to_bgr(message)

    assert converted.tolist() == [[[30, 20, 10], [60, 50, 40]]]


def test_metric_depth_round_trip():
    source = np.array([[1.25, 2.5], [3.75, 5.0]], dtype=np.float32)

    message = array_to_image(source, '32FC1')
    restored = ros_image_to_array(message)

    np.testing.assert_allclose(restored, source)


def test_bridge_nearest_resize_reduces_rgb_image():
    source = np.arange(4 * 4 * 3, dtype=np.uint8).reshape(4, 4, 3)

    resized_bytes = resize_image_nearest(
        source.tobytes(),
        width=4,
        height=4,
        step=12,
        channels=3,
        output_width=2,
        output_height=2,
    )
    resized = np.frombuffer(resized_bytes, dtype=np.uint8).reshape(2, 2, 3)

    np.testing.assert_array_equal(
        resized,
        source[[0, 3]][:, [0, 3]],
    )
