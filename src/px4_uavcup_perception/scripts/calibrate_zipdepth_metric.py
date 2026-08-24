#!/usr/bin/env python3
"""Collect measured ZipDepth samples and fit metric scale/shift."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from px4_uavcup_perception.image_utils import image_to_array
from px4_uavcup_perception.zipdepth_calibration import (
    central_roi_median,
    fit_metric_inverse_depth,
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest='command', required=True)
    collect = subparsers.add_parser('collect')
    collect.add_argument('--distance-m', type=float, required=True)
    collect.add_argument('--samples', type=int, default=20)
    collect.add_argument('--roi-fraction', type=float, default=0.25)
    collect.add_argument('--timeout', type=float, default=30.0)
    collect.add_argument('--topic', default='/uav/depth/zipdepth_raw')
    collect.add_argument('--output', type=Path, required=True)
    fit = subparsers.add_parser('fit')
    fit.add_argument('--input', type=Path, required=True)
    return parser


def _collect(args) -> None:
    if args.distance_m <= 0.0:
        raise ValueError('--distance-m must be positive')
    if args.samples <= 0 or args.timeout <= 0.0:
        raise ValueError('--samples and --timeout must be positive')
    rclpy.init()
    node = Node('calibrate_zipdepth_metric')
    medians = []
    image_shape = None

    def on_image(message: Image) -> None:
        nonlocal image_shape
        values = image_to_array(message)
        medians.append(central_roi_median(values, args.roi_fraction))
        image_shape = [int(message.width), int(message.height)]

    node.create_subscription(
        Image, args.topic, on_image, qos_profile_sensor_data)
    deadline = time.monotonic() + args.timeout
    while len(medians) < args.samples and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.25)
    node.destroy_node()
    rclpy.shutdown()
    if len(medians) < args.samples:
        raise TimeoutError(
            f'received {len(medians)}/{args.samples} frames from {args.topic}')

    dataset = {'schema_version': 1, 'records': []}
    if args.output.exists():
        dataset = json.loads(args.output.read_text(encoding='utf-8'))
        if dataset.get('schema_version') != 1:
            raise ValueError('unsupported calibration dataset schema')
    record = {
        'distance_m': float(args.distance_m),
        'raw_inverse_depth_median': float(np.median(medians)),
        'raw_inverse_depth_frame_stddev': float(np.std(medians)),
        'frames': len(medians),
        'roi_fraction': float(args.roi_fraction),
        'image_size': image_shape,
        'captured_at_utc': datetime.now(timezone.utc).isoformat(),
    }
    dataset.setdefault('records', []).append(record)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(dataset, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({
        'saved_to': str(args.output),
        'record_count': len(dataset['records']),
        'latest': record,
    }, indent=2))


def _fit(args) -> None:
    dataset = json.loads(args.input.read_text(encoding='utf-8'))
    result = fit_metric_inverse_depth(dataset.get('records', []))
    print(json.dumps(result, indent=2))


def main() -> None:
    args = _parser().parse_args()
    if args.command == 'collect':
        _collect(args)
    else:
        _fit(args)


if __name__ == '__main__':
    main()
