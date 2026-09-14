#!/usr/bin/env python3
"""Capture a selected Pi camera and serve frames to the ROS container."""

from __future__ import annotations

import argparse
from pathlib import Path
import signal
import socket

import numpy as np
from picamera2 import Picamera2

from px4_uavcup_perception.cameras.picamera2_protocol import pack_header
from px4_uavcup_perception.cameras.picamera2_selection import \
    select_sensor_size


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--socket', type=Path,
        default=Path(__file__).resolve().parents[3] / 'run/down_camera.sock')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=float, default=15.0)
    parser.add_argument(
        '--camera-model', default='imx500',
        help='unique model substring, for example imx500 or imx219')
    parser.add_argument(
        '--camera-index', type=int, default=0,
        help=(
            'Picamera2 camera number; use an explicit index with two cameras'))
    parser.add_argument(
        '--sensor-width', type=int, default=0,
        help='optional raw sensor mode width; set together with height')
    parser.add_argument(
        '--sensor-height', type=int, default=0,
        help='optional raw sensor mode height; set together with width')
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

    sensor_size = select_sensor_size(
        model=args.camera_model,
        output_width=args.width,
        output_height=args.height,
        sensor_width=args.sensor_width,
        sensor_height=args.sensor_height,
    )
    configuration_arguments = {
        'main': {'size': (args.width, args.height), 'format': 'RGB888'},
        'controls': {'FrameRate': args.fps},
    }
    if sensor_size is not None:
        configuration_arguments['raw'] = {'size': sensor_size}

    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(args.socket))
    server.listen(1)
    server.settimeout(1.0)
    header = pack_header(args.width, args.height)
    print(
        f'Picamera2 frame server ready: expected={args.camera_model} '
        f'index={args.camera_index} '
        f'{args.width}x{args.height} @ {args.fps:.1f} FPS -> {args.socket}',
        flush=True,
    )
    if sensor_size is not None:
        print(
            f'Raw sensor mode: {sensor_size[0]}x{sensor_size[1]}',
            flush=True)

    try:
        while running:
            try:
                connection, _ = server.accept()
            except socket.timeout:
                continue
            print('ROS camera client connected', flush=True)
            camera = None
            camera_running = False
            try:
                # Open, configure and start only after a consumer is ready.
                # Leaving a configured sensor idle while a heavy ROS client
                # loads can cause the CSI frontend to time out on IMX219.
                # Do not call Picamera2.global_camera_info() here. On the
                # tested Pi camera stack, enumerating and then opening from
                # the same process can leave IMX219 without CSI frames.
                camera = Picamera2(args.camera_index)
                actual_model = str(
                    camera.camera_properties.get('Model', 'unknown'))
                if args.camera_model.lower() not in actual_model.lower():
                    raise RuntimeError(
                        f'camera index {args.camera_index} is {actual_model}, '
                        f'expected {args.camera_model}')
                print(
                    f'Opened Picamera2 camera: {actual_model} '
                    f'(index={args.camera_index})', flush=True)
                configuration = camera.create_preview_configuration(
                    **configuration_arguments)
                camera.configure(configuration)
                camera.start()
                camera_running = True
                with connection:
                    while running:
                        frame = np.ascontiguousarray(camera.capture_array())
                        if frame.shape != (args.height, args.width, 3):
                            raise RuntimeError(
                                'unexpected Picamera2 frame shape '
                                f'{frame.shape}')
                        connection.sendall(header)
                        connection.sendall(frame.data)
            except (
                    BrokenPipeError, ConnectionError, OSError,
                    RuntimeError) as error:
                if running:
                    print(
                        f'ROS camera client disconnected: {error}',
                        flush=True)
            finally:
                if camera_running:
                    camera.stop()
                if camera is not None:
                    camera.close()
    finally:
        server.close()
        args.socket.unlink(missing_ok=True)


if __name__ == '__main__':
    main()
