from pathlib import Path

from px4_uavcup_perception.cameras.picamera2_supervisor import (
    default_servers,
    server_command,
)


def test_default_topology_assigns_zipdepth_to_cam0():
    front, down = default_servers(Path('/vehicle'))

    assert (front.index, front.model) == (0, 'imx219')
    assert front.socket_path == Path('/vehicle/run/front_camera.sock')
    assert (down.index, down.model) == (1, 'imx500')
    assert down.socket_path == Path('/vehicle/run/down_camera.sock')


def test_server_command_passes_identity_socket_and_resolution():
    front, _ = default_servers(Path('/vehicle'))

    command = server_command(
        '/usr/bin/python3', Path('/vehicle/frame_server.py'), front, 640, 480)

    assert command == [
        '/usr/bin/python3',
        '/vehicle/frame_server.py',
        '--camera-index', '0',
        '--camera-model', 'imx219',
        '--socket', '/vehicle/run/front_camera.sock',
        '--width', '640',
        '--height', '480',
        '--fps', '30.0',
    ]
