#!/usr/bin/env python3
"""Publish robust left/centre/right distances from a metric depth image."""

from __future__ import annotations

import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

from .free_space import summarize_free_space
from .image_utils import ros_image_to_array


class FreeSpaceNode(Node):
    def __init__(self) -> None:
        super().__init__('free_space_node')
        self.declare_parameter('depth_topic', '/uav/depth/image')
        self.declare_parameter('output_topic', '/uav/depth/free_space')
        self.declare_parameter('status_topic', '/uav/depth/free_space_status')
        self.declare_parameter('minimum_depth_m', 0.15)
        self.declare_parameter('maximum_depth_m', 20.0)
        self.declare_parameter('near_percentile', 15.0)
        self.declare_parameter('roi_top_fraction', 0.25)
        self.declare_parameter('roi_bottom_fraction', 0.85)
        self.declare_parameter('minimum_valid_fraction', 0.25)

        self._minimum_depth = float(
            self.get_parameter('minimum_depth_m').value)
        self._maximum_depth = float(
            self.get_parameter('maximum_depth_m').value)
        self._percentile = float(
            self.get_parameter('near_percentile').value)
        self._roi_top = float(
            self.get_parameter('roi_top_fraction').value)
        self._roi_bottom = float(
            self.get_parameter('roi_bottom_fraction').value)
        self._minimum_valid_fraction = float(
            self.get_parameter('minimum_valid_fraction').value)

        self._publisher = self.create_publisher(
            Float32MultiArray,
            str(self.get_parameter('output_topic').value),
            10,
        )
        self._status_publisher = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter('status_topic').value),
            10,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('depth_topic').value),
            self._on_depth,
            qos_profile_sensor_data,
        )

    def _on_depth(self, message: Image) -> None:
        try:
            depth = ros_image_to_array(message)
            if message.encoding != '32FC1':
                raise ValueError(
                    f'Expected 32FC1 metric depth, got {message.encoding}')
            summary = summarize_free_space(
                depth,
                minimum_depth_m=self._minimum_depth,
                maximum_depth_m=self._maximum_depth,
                near_percentile=self._percentile,
                roi_top_fraction=self._roi_top,
                roi_bottom_fraction=self._roi_bottom,
            )
        except ValueError as error:
            self.get_logger().error(str(error), throttle_duration_sec=2.0)
            self._publish_status(DiagnosticStatus.ERROR, str(error), 0.0)
            return

        output = Float32MultiArray()
        dimension = MultiArrayDimension()
        dimension.label = 'left,center,right,nearest,valid_fraction'
        dimension.size = 5
        dimension.stride = 5
        output.layout.dim = [dimension]
        output.data = summary.as_list()
        self._publisher.publish(output)

        all_sectors_valid = all(math.isfinite(value) for value in (
            summary.left_m, summary.center_m, summary.right_m))
        healthy = (
            all_sectors_valid
            and summary.valid_fraction >= self._minimum_valid_fraction
        )
        level = DiagnosticStatus.OK if healthy else DiagnosticStatus.WARN
        status_message = 'valid' if healthy else 'insufficient valid depth'
        self._publish_status(
            level, status_message, summary.valid_fraction)

    def _publish_status(
            self, level: int, message: str, valid_fraction: float) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.level = level
        status.name = 'px4_uavcup/free_space'
        status.hardware_id = 'front_camera'
        status.message = message
        status.values = [KeyValue(
            key='valid_fraction', value=f'{valid_fraction:.3f}')]
        array.status = [status]
        self._status_publisher.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FreeSpaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
