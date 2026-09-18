from pathlib import Path

from px4_uavcup_perception.cameras.picamera2_supervisor import (
    default_servers,
)


def test_default_topology_assigns_zipdepth_to_cam0():
    front, down = default_servers(Path('/vehicle'))

    assert (front.index, front.model) == (0, 'imx219')
    assert front.socket_path == Path('/vehicle/run/front_camera.sock')
    assert (down.index, down.model) == (1, 'imx500')
    assert down.socket_path == Path('/vehicle/run/down_camera.sock')
    assert (front.fps, down.fps) == (30.0, 15.0)
    assert front.socket_path != down.socket_path
