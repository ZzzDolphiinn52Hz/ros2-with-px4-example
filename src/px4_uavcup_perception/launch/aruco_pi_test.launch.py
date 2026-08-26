#!/usr/bin/env python3
"""Test IMX500 ArUco pose output without PID, Offboard, or PX4 topics."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('px4_uavcup_perception')
    cameras_config = os.path.join(package_share, 'config', 'pi_cameras.yaml')
    aruco_config = os.path.join(package_share, 'config', 'aruco.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('publish_debug_topics', default_value='true'),
        DeclareLaunchArgument('publish_debug_image', default_value='true'),
        Node(
            package='px4_uavcup_perception',
            executable='picamera2_socket_camera_node',
            name='down_picamera',
            output='screen',
            parameters=[cameras_config],
        ),
        Node(
            package='px4_uavcup_perception',
            executable='aruco_detector_node',
            name='aruco_detector_node',
            output='screen',
            parameters=[aruco_config, {
                'publish_debug_topics': ParameterValue(
                    LaunchConfiguration('publish_debug_topics'),
                    value_type=bool),
                'publish_debug_image': ParameterValue(
                    LaunchConfiguration('publish_debug_image'),
                    value_type=bool),
            }],
        ),
    ])
