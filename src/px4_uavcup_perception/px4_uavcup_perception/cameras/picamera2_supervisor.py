"""Run the two Raspberry Pi camera frame servers as one host process."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import queue
import signal
import socket
import threading

from px4_uavcup_perception.cameras.picamera2_protocol import pack_header
from px4_uavcup_perception.cameras.picamera2_selection import \
    select_capture_format, select_sensor_size


@dataclass(frozen=True)
class CameraServer:
    """Configuration for one Picamera2-to-Unix-socket server."""

    role: str
    index: int
    model: str
    socket_path: Path
    fps: float


def default_servers(workspace: Path) -> tuple[CameraServer, CameraServer]:
    """Return the vehicle camera topology agreed for the Raspberry Pi 5."""
    return (
        CameraServer(
            role='front/ZipDepth',
            index=0,
            model='imx219',
            socket_path=workspace / 'run/front_camera.sock',
            fps=30.0,
        ),
        CameraServer(
            role='down/ArUco',
            index=1,
            model='imx500',
            socket_path=workspace / 'run/down_camera.sock',
            fps=15.0,
        ),
    )


def _serve_camera(
        server: CameraServer,
        width: int,
        height: int,
        stop_event: threading.Event,
        camera_open_lock: threading.Lock,
        errors: queue.SimpleQueue[BaseException]) -> None:
    """Serve one camera while sharing Picamera2's manager in this process."""
    # These imports intentionally stay inside the runtime path so unit tests
    # can inspect the topology on hosts without Raspberry Pi camera packages.
    import numpy as np
    from picamera2 import Picamera2

    capture_format, capture_channels = select_capture_format(server.model)
    sensor_size = select_sensor_size(
        model=server.model,
        output_width=width,
        output_height=height,
    )
    configuration_arguments = {
        'main': {'size': (width, height), 'format': capture_format},
        'controls': {'FrameRate': server.fps},
    }
    if sensor_size is not None:
        configuration_arguments['raw'] = {'size': sensor_size}

    server.socket_path.parent.mkdir(parents=True, exist_ok=True)
    server.socket_path.unlink(missing_ok=True)
    listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    listener.bind(str(server.socket_path))
    listener.listen(1)
    listener.settimeout(1.0)
    header = pack_header(width, height)
    print(
        f'Picamera2 frame server ready: expected={server.model} '
        f'index={server.index} {width}x{height} {capture_format} '
        f'@ {server.fps:.1f} FPS -> {server.socket_path}',
        flush=True,
    )

    try:
        while not stop_event.is_set():
            try:
                connection, _ = listener.accept()
            except socket.timeout:
                continue
            print(
                f'CAM{server.index} {server.model}: ROS client connected',
                flush=True,
            )
            camera = None
            camera_running = False
            try:
                # Picamera2 uses a process-wide libcamera manager. Creating
                # cameras in separate processes makes both managers claim the
                # same PiSP pipeline. A lock also prevents a first-use race
                # while the shared manager is initialized.
                with camera_open_lock:
                    camera = Picamera2(server.index)
                    actual_model = str(
                        camera.camera_properties.get('Model', 'unknown'))
                    if server.model.lower() not in actual_model.lower():
                        raise RuntimeError(
                            f'camera index {server.index} is {actual_model}, '
                            f'expected {server.model}')
                    configuration = camera.create_preview_configuration(
                        **configuration_arguments)
                    camera.configure(configuration)
                    camera.start()
                    camera_running = True
                print(
                    f'CAM{server.index}: opened {actual_model}', flush=True)

                connection.settimeout(2.0)
                with connection:
                    while not stop_event.is_set():
                        frame = np.ascontiguousarray(camera.capture_array())
                        expected_shape = (
                            height, width, capture_channels)
                        if frame.shape != expected_shape:
                            raise RuntimeError(
                                'unexpected Picamera2 frame shape '
                                f'{frame.shape}')
                        if capture_channels == 4:
                            frame = np.ascontiguousarray(frame[:, :, :3])
                        connection.sendall(header)
                        connection.sendall(frame.data)
            except (
                    BrokenPipeError, ConnectionError, OSError,
                    RuntimeError) as error:
                if not stop_event.is_set():
                    print(
                        f'CAM{server.index} client disconnected: {error}',
                        flush=True,
                    )
            finally:
                if camera_running:
                    camera.stop()
                if camera is not None:
                    camera.close()
    except BaseException as error:
        errors.put(error)
        stop_event.set()
    finally:
        listener.close()
        server.socket_path.unlink(missing_ok=True)


def main() -> int:
    workspace = Path(__file__).resolve().parents[4]
    defaults = default_servers(workspace)
    parser = argparse.ArgumentParser(
        description=(
            'Start CAM0 IMX219 for ZipDepth and CAM1 IMX500 for ArUco.'))
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--front-fps', type=float, default=defaults[0].fps)
    parser.add_argument('--down-fps', type=float, default=defaults[1].fps)
    args = parser.parse_args()
    if args.width <= 0 or args.height <= 0:
        parser.error('width and height must be positive')
    if args.front_fps <= 0.0 or args.down_fps <= 0.0:
        parser.error('camera frame rates must be positive')

    servers = (
        CameraServer(
            role=defaults[0].role,
            index=defaults[0].index,
            model=defaults[0].model,
            socket_path=defaults[0].socket_path,
            fps=args.front_fps,
        ),
        CameraServer(
            role=defaults[1].role,
            index=defaults[1].index,
            model=defaults[1].model,
            socket_path=defaults[1].socket_path,
            fps=args.down_fps,
        ),
    )
    stop_event = threading.Event()

    def request_stop(_signum, _frame) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    camera_open_lock = threading.Lock()
    errors: queue.SimpleQueue[BaseException] = queue.SimpleQueue()
    threads: list[threading.Thread] = []
    print('Pi camera topology:', flush=True)
    for server in servers:
        print(
            f'  CAM{server.index}: {server.model.upper()} -> '
            f'{server.role} -> {server.socket_path}',
            flush=True,
        )
        thread = threading.Thread(
            target=_serve_camera,
            args=(
                server, args.width, args.height, stop_event,
                camera_open_lock, errors),
            name=f'camera-{server.index}-{server.model}',
        )
        thread.start()
        threads.append(thread)

    try:
        while not stop_event.wait(0.25):
            if any(not thread.is_alive() for thread in threads):
                stop_event.set()
                break
    finally:
        stop_event.set()
        for thread in threads:
            thread.join(timeout=5.0)

    if not errors.empty():
        raise errors.get()
    if any(thread.is_alive() for thread in threads):
        raise RuntimeError('camera server did not stop cleanly')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
