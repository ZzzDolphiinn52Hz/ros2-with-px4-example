#!/usr/bin/env python3
"""Start only the Raspberry Pi ZipDepth node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    perception_share = get_package_share_directory('px4_uavcup_perception')
    return LaunchDescription([
        DeclareLaunchArgument('publish_raw_output', default_value='false'),
        DeclareLaunchArgument('publish_metric_depth', default_value='false'),
        DeclareLaunchArgument('publish_visualization', default_value='false'),
        DeclareLaunchArgument('publish_pointcloud', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                perception_share, 'launch', 'zipdepth_pi.launch.py')),
            launch_arguments={
                'publish_raw_output': LaunchConfiguration(
                    'publish_raw_output'),
                'publish_metric_depth': LaunchConfiguration(
                    'publish_metric_depth'),
                'publish_visualization': LaunchConfiguration(
                    'publish_visualization'),
                'publish_pointcloud': LaunchConfiguration(
                    'publish_pointcloud'),
            }.items(),
        ),
    ])
