#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('px4_uavcup_control')
    default_config = os.path.join(
        package_share, 'config', 'shadow_controller.yaml')
    config = LaunchConfiguration('config')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        Node(
            package='px4_uavcup_control',
            executable='local_controller_shadow',
            name='local_controller_shadow',
            output='screen',
            parameters=[config],
        ),
    ])
