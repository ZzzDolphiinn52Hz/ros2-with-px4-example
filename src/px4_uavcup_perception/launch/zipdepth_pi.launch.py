#!/usr/bin/env python3
"""Start only the Raspberry Pi ZipDepth node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('px4_uavcup_perception')
    default_config = os.path.join(package_share, 'config', 'zipdepth.yaml')
    config = LaunchConfiguration('config')
    debug_parameters = {
        'publish_raw_output': ParameterValue(
            LaunchConfiguration('publish_raw_output'), value_type=bool),
        'publish_metric_depth': ParameterValue(
            LaunchConfiguration('publish_metric_depth'), value_type=bool),
        'publish_visualization': ParameterValue(
            LaunchConfiguration('publish_visualization'), value_type=bool),
        'publish_pointcloud': ParameterValue(
            LaunchConfiguration('publish_pointcloud'), value_type=bool),
    }
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('publish_raw_output', default_value='false'),
        DeclareLaunchArgument('publish_metric_depth', default_value='false'),
        DeclareLaunchArgument('publish_visualization', default_value='false'),
        DeclareLaunchArgument('publish_pointcloud', default_value='false'),
        Node(
            package='px4_uavcup_perception',
            executable='zipdepth_node',
            name='zipdepth_node',
            output='screen',
            parameters=[config, debug_parameters],
        ),
    ])
