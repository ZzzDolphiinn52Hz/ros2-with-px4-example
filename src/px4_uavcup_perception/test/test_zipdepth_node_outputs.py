from px4_uavcup_perception.depth.zipdepth_node import ZipDepthNode


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def make_node_without_ros_runtime():
    node = object.__new__(ZipDepthNode)
    node._free_space_publisher = RecordingPublisher()
    node._relative_free_space_publisher = RecordingPublisher()
    node._relative_corridor_publisher = RecordingPublisher()
    return node


def test_metric_disabled_does_not_invalidate_relative_outputs():
    node = make_node_without_ros_runtime()

    node._publish_invalid_metric_free_space()

    assert len(node._free_space_publisher.messages) == 1
    assert not node._relative_free_space_publisher.messages
    assert not node._relative_corridor_publisher.messages


def test_camera_failure_invalidates_metric_and_relative_outputs():
    node = make_node_without_ros_runtime()

    node._publish_invalid_depth_outputs()

    assert len(node._free_space_publisher.messages) == 1
    assert len(node._relative_free_space_publisher.messages) == 1
    assert len(node._relative_corridor_publisher.messages) == 1
