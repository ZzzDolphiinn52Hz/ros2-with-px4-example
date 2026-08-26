import numpy as np

from px4_uavcup_perception.common.image import array_to_image


def test_metric_depth_message_layout():
    source = np.array([[1.25, 2.5], [3.75, 5.0]], dtype=np.float32)

    message = array_to_image(source, '32FC1')
    assert message.width == 2
    assert message.height == 2
    assert message.step == 8
    assert message.encoding == '32FC1'
    np.testing.assert_allclose(
        np.frombuffer(message.data, dtype=np.float32).reshape(2, 2), source)


def test_bgr_image_message_layout():
    source = np.array(
        [[[10, 20, 30], [40, 50, 60]]],
        dtype=np.uint8,
    )

    message = array_to_image(source, 'bgr8')
    assert message.width == 2
    assert message.height == 1
    assert message.step == 6
    assert message.encoding == 'bgr8'
    np.testing.assert_array_equal(
        np.frombuffer(message.data, dtype=np.uint8).reshape(1, 2, 3),
        source,
    )
