#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    perception_share = get_package_share_directory('px4_uavcup_perception')
    control_share = get_package_share_directory('px4_uavcup_control')
    perception_launch = os.path.join(
        perception_share, 'launch', 'zipdepth_pi.launch.py')
    perception_config_default = os.path.join(
        perception_share, 'config', 'perception_pi.yaml')
    control_config_default = os.path.join(
        control_share, 'config', 'shadow_controller_relative.yaml')
    perception_config = LaunchConfiguration('perception_config')
    control_config = LaunchConfiguration('control_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'perception_config', default_value=perception_config_default),
        DeclareLaunchArgument(
            'control_config', default_value=control_config_default),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(perception_launch),
            launch_arguments={
                'config': perception_config,
                'publish_raw_output': 'false',
                'publish_metric_depth': 'false',
                'publish_visualization': 'false',
                'publish_pointcloud': 'false',
            }.items(),
        ),
        Node(
            package='px4_uavcup_control',
            executable='local_controller_shadow',
            name='local_controller_shadow',
            output='screen',
            parameters=[control_config],
        ),
    ])
