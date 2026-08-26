#!/usr/bin/env python3
"""Read one ROS depth image and report its numeric validity."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from px4_uavcup_perception.common.image import image_to_array
from px4_uavcup_perception.depth.zipdepth_backend import (
    normalize_inverse_depth_for_display,
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/uav/depth/zipdepth_raw')
    parser.add_argument('--timeout', type=float, default=20.0)
    parser.add_argument(
        '--save-color', type=Path,
        help='save a Turbo colormap made directly from the received raw image')
    args = parser.parse_args()
    rclpy.init()
    node = Node('check_ros_depth_topic')
    received = []
    node.create_subscription(
        Image, args.topic, received.append, qos_profile_sensor_data)
    deadline = time.monotonic() + args.timeout
    while not received and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.25)
    if not received:
        node.destroy_node()
        rclpy.shutdown()
        raise TimeoutError(f'no image received from {args.topic}')
    message = received[0]
    values = image_to_array(message)
    finite = np.isfinite(values)
    report = {
        'topic': args.topic,
        'width': int(message.width),
        'height': int(message.height),
        'encoding': message.encoding,
        'step': int(message.step),
        'finite_ratio': float(np.mean(finite)),
        'minimum': float(np.min(values[finite])),
        'median': float(np.median(values[finite])),
        'maximum': float(np.max(values[finite])),
    }
    if args.save_color is not None:
        # OpenCV is only needed when writing the optional color preview.  Keep
        # the normal ROS topic check usable on hosts whose apt OpenCV was built
        # against NumPy 1.x but which currently have NumPy 2.x installed.
        import cv2

        depth_u8, display_low, display_high = \
            normalize_inverse_depth_for_display(values)
        color = cv2.applyColorMap(depth_u8, cv2.COLORMAP_TURBO)
        cv2.putText(
            color, 'ZipDepth inverse depth: red=near, blue=far',
            (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
            (255, 255, 255), 1, cv2.LINE_AA)
        args.save_color.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(args.save_color), color):
            raise RuntimeError(f'failed to save {args.save_color}')
        report['display_low_percentile_2'] = display_low
        report['display_high_percentile_98'] = display_high
        report['color_preview'] = str(args.save_color)
    print(json.dumps(report, indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
