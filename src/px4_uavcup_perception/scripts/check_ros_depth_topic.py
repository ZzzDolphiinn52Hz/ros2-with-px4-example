#!/usr/bin/env python3
"""Read one ROS depth image and report its numeric validity."""

from __future__ import annotations

import argparse
import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from px4_uavcup_perception.image_utils import image_to_array


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/uav/depth/zipdepth_raw')
    parser.add_argument('--timeout', type=float, default=20.0)
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
    print(json.dumps(report, indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
