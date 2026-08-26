#!/usr/bin/env python3
"""Capture IMX500 frames on the Pi host and serve them to the ROS container."""

from __future__ import annotations

import argparse
from pathlib import Path
import signal
import socket

import numpy as np
from picamera2 import Picamera2

from px4_uavcup_perception.cameras.picamera2_protocol import pack_header


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--socket', type=Path,
        default=Path('/home/dolphiinn/ros2_ws/run/down_camera.sock'))
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=float, default=15.0)
    args = parser.parse_args()
    if args.width <= 0 or args.height <= 0 or args.fps <= 0.0:
        raise ValueError('width, height and fps must be positive')

    running = True

    def stop(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    args.socket.parent.mkdir(parents=True, exist_ok=True)
    args.socket.unlink(missing_ok=True)

    camera = Picamera2()
    configuration = camera.create_preview_configuration(
        main={'size': (args.width, args.height), 'format': 'RGB888'},
        controls={'FrameRate': args.fps},
    )
    camera.configure(configuration)
    camera.start()

    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(args.socket))
    server.listen(1)
    server.settimeout(1.0)
    header = pack_header(args.width, args.height)
    print(
        f'Picamera2 frame server ready: IMX500 {args.width}x{args.height} '
        f'@ {args.fps:.1f} FPS -> {args.socket}',
        flush=True,
    )

    try:
        while running:
            try:
                connection, _ = server.accept()
            except socket.timeout:
                continue
            print('ROS camera client connected', flush=True)
            try:
                with connection:
                    while running:
                        frame = np.ascontiguousarray(camera.capture_array())
                        if frame.shape != (args.height, args.width, 3):
                            raise RuntimeError(
                                'unexpected Picamera2 frame shape '
                                f'{frame.shape}')
                        connection.sendall(header)
                        connection.sendall(frame.data)
            except (BrokenPipeError, ConnectionError, OSError) as error:
                if running:
                    print(
                        f'ROS camera client disconnected: {error}',
                        flush=True)
    finally:
        server.close()
        camera.stop()
        camera.close()
        args.socket.unlink(missing_ok=True)


if __name__ == '__main__':
    main()
