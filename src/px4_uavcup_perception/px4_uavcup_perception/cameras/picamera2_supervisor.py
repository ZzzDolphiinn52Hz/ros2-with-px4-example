"""Run the two Raspberry Pi camera frame servers as one host process."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import signal
import subprocess
import sys
import time
from typing import Sequence


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


def server_command(
        python: str,
        frame_server: Path,
        server: CameraServer,
        width: int,
        height: int) -> list[str]:
    """Build one child command without invoking Picamera2."""
    return [
        python,
        str(frame_server),
        '--camera-index', str(server.index),
        '--camera-model', server.model,
        '--socket', str(server.socket_path),
        '--width', str(width),
        '--height', str(height),
        '--fps', str(server.fps),
    ]


def _stop_children(children: Sequence[subprocess.Popen]) -> None:
    for child in children:
        if child.poll() is None:
            child.terminate()
    deadline = time.monotonic() + 5.0
    for child in children:
        if child.poll() is None:
            try:
                child.wait(timeout=max(0.0, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                child.kill()
    for child in children:
        child.wait()


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
    frame_server = workspace / (
        'src/px4_uavcup_perception/scripts/picamera2_frame_server.py')
    if not frame_server.is_file():
        raise FileNotFoundError(f'frame server not found: {frame_server}')

    stop_requested = False

    def request_stop(_signum, _frame) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    children: list[subprocess.Popen] = []
    try:
        print('Pi camera topology:', flush=True)
        for server in servers:
            print(
                f'  CAM{server.index}: {server.model.upper()} -> '
                f'{server.role} -> {server.socket_path}',
                flush=True,
            )
            children.append(subprocess.Popen(server_command(
                sys.executable,
                frame_server,
                server,
                args.width,
                args.height,
            )))

        while not stop_requested:
            for server, child in zip(servers, children):
                return_code = child.poll()
                if return_code is not None:
                    print(
                        f'CAM{server.index} {server.model} server exited '
                        f'with code {return_code}',
                        file=sys.stderr,
                        flush=True,
                    )
                    return 1
            time.sleep(0.25)
        return 0
    finally:
        _stop_children(children)


if __name__ == '__main__':
    raise SystemExit(main())
