#!/usr/bin/env python3
"""Test both Pi perception pipelines without control or PX4 nodes."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    perception_share = get_package_share_directory('px4_uavcup_perception')
    launch_dir = os.path.join(perception_share, 'launch')

    return LaunchDescription([
        DeclareLaunchArgument('publish_depth_visualization',
                              default_value='false'),
        DeclareLaunchArgument('publish_aruco_debug_topics',
                              default_value='true'),
        DeclareLaunchArgument('publish_aruco_debug_image',
                              default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'zipdepth_pi.launch.py')),
            launch_arguments={
                'publish_raw_output': 'false',
                'publish_metric_depth': 'false',
                'publish_visualization': LaunchConfiguration(
                    'publish_depth_visualization'),
                'publish_pointcloud': 'false',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'aruco_pi_test.launch.py')),
            launch_arguments={
                'publish_debug_topics': LaunchConfiguration(
                    'publish_aruco_debug_topics'),
                'publish_debug_image': LaunchConfiguration(
                    'publish_aruco_debug_image'),
            }.items(),
        ),
    ])
