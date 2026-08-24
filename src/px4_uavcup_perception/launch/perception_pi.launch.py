#!/usr/bin/env python3
"""Launch the two-camera ZipDepth and ArUco landing pipeline on Pi 5."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('px4_uavcup_perception')
    default_config = os.path.join(
        package_share, 'config', 'perception_pi.yaml')
    config = LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        # ZipDepth opens /dev/video0 directly by default. Enable this only when
        # camera_device is empty in the YAML and a ROS image topic is desired.
        DeclareLaunchArgument('front_usb_camera', default_value='false'),
        Node(
            package='px4_uavcup_perception',
            executable='v4l2_camera_node',
            name='front_usb_camera',
            output='screen',
            parameters=[config],
            condition=IfCondition(LaunchConfiguration('front_usb_camera')),
        ),
        Node(
            package='px4_uavcup_perception',
            executable='zipdepth_node',
            name='zipdepth_node',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='px4_uavcup_perception',
            executable='aruco_detector_node',
            name='aruco_detector_node',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='px4_uavcup_perception',
            executable='aruco_landing_pid_node',
            name='aruco_landing_pid',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='px4_uavcup_slam',
            executable='cmd_vel_to_px4',
            name='cmd_vel_to_px4',
            output='screen',
            parameters=[config],
        ),
    ])
