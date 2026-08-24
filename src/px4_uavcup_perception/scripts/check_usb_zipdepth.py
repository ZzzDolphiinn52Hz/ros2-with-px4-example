#!/usr/bin/env python3
"""Bench-test a real USB camera and ZipDepth ONNX without ROS/PX4 control."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import cv2
import numpy as np

from px4_uavcup_perception.zipdepth_onnx_backend import (
    ZipDepthOnnx,
    normalize_inverse_depth_for_display,
)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--device', default='/dev/video0')
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--camera-frames', type=int, default=120)
    parser.add_argument('--depth-frames', type=int, default=30)
    return parser.parse_args()


def open_camera(device: str):
    camera = cv2.VideoCapture(device, cv2.CAP_V4L2)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    camera.set(cv2.CAP_PROP_FPS, 30.0)
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not camera.isOpened():
        raise RuntimeError(f'cannot open {device}')
    return camera


def main() -> None:
    args = parse_args()
    if args.camera_frames <= 0 or args.depth_frames <= 0:
        raise ValueError('frame counts must be positive')
    args.output.mkdir(parents=True, exist_ok=True)
    camera = open_camera(args.device)
    backend = ZipDepthOnnx(args.model, threads=3)

    for _ in range(20):
        camera.read()

    camera_success = 0
    camera_failures = 0
    camera_started = time.perf_counter()
    last_frame = None
    for _ in range(args.camera_frames):
        received, frame = camera.read()
        if received and frame is not None:
            camera_success += 1
            last_frame = frame
        else:
            camera_failures += 1
    camera_elapsed = time.perf_counter() - camera_started

    latencies = []
    finite_ratios = []
    percentiles = []
    temporal_differences = []
    previous_normalized = None
    depth_success = 0
    depth_failures = 0
    last_raw = None
    depth_started = time.perf_counter()
    for _ in range(args.depth_frames):
        received, frame = camera.read()
        if not received or frame is None:
            depth_failures += 1
            continue
        started = time.perf_counter()
        raw = backend.infer(frame, cv2)
        latencies.append((time.perf_counter() - started) * 1000.0)
        finite = np.isfinite(raw)
        finite_ratios.append(float(np.mean(finite)))
        values = raw[finite]
        percentiles.append(np.percentile(values, [2, 50, 98]).tolist())
        depth_u8, _, _ = normalize_inverse_depth_for_display(raw)
        normalized = depth_u8.astype(np.float32) / 255.0
        if previous_normalized is not None:
            temporal_differences.append(float(np.mean(
                np.abs(normalized - previous_normalized))))
        previous_normalized = normalized
        depth_success += 1
        last_frame = frame
        last_raw = raw
    depth_elapsed = time.perf_counter() - depth_started
    camera.release()

    if last_frame is None or last_raw is None or not latencies:
        raise RuntimeError('no valid camera/depth pair was produced')

    depth_u8, display_low, display_high = \
        normalize_inverse_depth_for_display(last_raw)
    depth_color = cv2.applyColorMap(depth_u8, cv2.COLORMAP_TURBO)
    rgb_model = cv2.resize(last_frame, (backend.width, backend.height))
    cv2.putText(depth_color, 'inverse depth: red=near, blue=far',
                (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                (255, 255, 255), 1, cv2.LINE_AA)
    montage = np.hstack([rgb_model, depth_color])
    cv2.imwrite(str(args.output / 'usb_rgb.png'), last_frame)
    cv2.imwrite(str(args.output / 'zipdepth_inverse_color.png'), depth_color)
    cv2.imwrite(str(args.output / 'rgb_depth_montage.png'), montage)
    np.save(str(args.output / 'zipdepth_inverse_raw.npy'), last_raw)

    p = np.asarray(percentiles, dtype=np.float64)
    report = {
        'camera': {
            'device': args.device,
            'requested_frames': args.camera_frames,
            'successful_frames': camera_success,
            'failed_frames': camera_failures,
            'measured_capture_fps': camera_success / camera_elapsed,
            'resolution': [int(last_frame.shape[1]), int(last_frame.shape[0])],
            'mean_brightness_0_255': float(np.mean(cv2.cvtColor(
                last_frame, cv2.COLOR_BGR2GRAY))),
            'laplacian_variance_focus_score': float(cv2.Laplacian(
                cv2.cvtColor(last_frame, cv2.COLOR_BGR2GRAY),
                cv2.CV_64F).var()),
        },
        'zipdepth': {
            'requested_frames': args.depth_frames,
            'successful_frames': depth_success,
            'failed_frames': depth_failures,
            'end_to_end_fps': depth_success / depth_elapsed,
            'latency_ms_median': float(np.median(latencies)),
            'latency_ms_p95': float(np.percentile(latencies, 95)),
            'finite_ratio_min': float(np.min(finite_ratios)),
            'inverse_depth_p02_median': float(np.median(p[:, 0])),
            'inverse_depth_p50_median': float(np.median(p[:, 1])),
            'inverse_depth_p98_median': float(np.median(p[:, 2])),
            'normalized_temporal_mad_mean': (
                float(np.mean(temporal_differences))
                if temporal_differences else None),
            'output_shape': list(last_raw.shape),
            'metric_depth_available': False,
            'display_clip': [display_low, display_high],
        },
    }
    report_path = args.output / 'report.json'
    report_path.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
