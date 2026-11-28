import threading

from gz.msgs10.camera_info_pb2 import CameraInfo as GzCameraInfo
from gz.msgs10.image_pb2 import Image as GzImage
from gz.msgs10.image_pb2 import PixelFormatType

from px4_uavcup_simulation.gz_camera_bridge import (
    GazeboCameraBridge,
    camera_info_to_ros,
    image_to_ros,
)


def _set_header(message, frame_id='down_camera_optical_frame'):
    message.header.stamp.sec = 12
    message.header.stamp.nsec = 345
    entry = message.header.data.add()
    entry.key = 'frame_id'
    entry.value.append(frame_id)


def test_rgb_image_conversion():
    message = GzImage()
    _set_header(message)
    message.width = 2
    message.height = 1
    message.step = 6
    message.pixel_format_type = PixelFormatType.RGB_INT8
    message.data = bytes([1, 2, 3, 4, 5, 6])

    output = image_to_ros(message, 'fallback')

    assert output.header.frame_id == 'down_camera_optical_frame'
    assert output.header.stamp.sec == 12
    assert output.header.stamp.nanosec == 345
    assert output.encoding == 'rgb8'
    assert output.step == 6
    assert bytes(output.data) == message.data


def test_camera_info_conversion():
    message = GzCameraInfo()
    _set_header(message)
    message.width = 640
    message.height = 480
    message.distortion.k.extend([0.0] * 5)
    message.intrinsics.k.extend([
        480.125104, 0.0, 320.272190,
        0.0, 480.224260, 228.696135,
        0.0, 0.0, 1.0,
    ])
    message.rectification_matrix.extend([
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
    ])
    message.projection.p.extend([
        480.125104, 0.0, 320.272190, 0.0,
        0.0, 480.224260, 228.696135, 0.0,
        0.0, 0.0, 1.0, 0.0,
    ])

    output = camera_info_to_ros(message, 'fallback')

    assert output.width == 640
    assert output.height == 480
    assert output.distortion_model == 'plumb_bob'
    assert list(output.d) == [0.0] * 5
    assert list(output.k) == list(message.intrinsics.k)
    assert list(output.r) == list(message.rectification_matrix)
    assert list(output.p) == list(message.projection.p)


def test_stop_blocks_future_transport_callbacks():
    class FakeTransportNode:
        def __init__(self):
            self.unsubscribed = []

        def unsubscribe(self, topic):
            self.unsubscribed.append(topic)
            return True

    transport = FakeTransportNode()
    bridge = GazeboCameraBridge.__new__(GazeboCameraBridge)
    bridge._callback_lock = threading.Lock()
    bridge._shutting_down = False
    bridge._image_topic = '/image'
    bridge._camera_info_topic = '/camera_info'
    bridge._clock_topic = '/clock'
    bridge._gz_node = transport

    bridge.stop()

    assert bridge._shutting_down is True
    assert bridge._gz_node is None
    assert transport.unsubscribed == ['/image', '/camera_info', '/clock']


def test_pending_image_is_replaced_before_publish():
    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    bridge = GazeboCameraBridge.__new__(GazeboCameraBridge)
    bridge._callback_lock = threading.Lock()
    bridge._shutting_down = False
    bridge._latest_info = None
    bridge._latest_clock = None
    bridge._fallback_frame_id = 'camera'
    bridge._image_pub = Publisher()
    bridge._frames = 0
    for seq in range(1, 11):
        message = GzImage(
            width=1, height=1, pixel_format_type=PixelFormatType.L_INT8,
            data=b'\x01')
        message.header.stamp.sec = seq
        bridge._on_image(message)
    bridge._publish_pending()
    bridge._publish_pending()
    assert [m.header.stamp.sec for m in bridge._image_pub.messages] == [10]
