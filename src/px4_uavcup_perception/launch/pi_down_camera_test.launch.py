#!/usr/bin/env python3
"""Safely test the Picamera2 host bridge without ArUco or flight nodes."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('px4_uavcup_perception')
    default_config = os.path.join(
        package_share, 'config', 'pi_cameras.yaml')
    config = LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        Node(
            package='px4_uavcup_perception',
            executable='picamera2_socket_camera_node',
            name='down_picamera',
            output='screen',
            parameters=[config],
        ),
    ])
