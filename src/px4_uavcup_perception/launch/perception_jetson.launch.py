#!/usr/bin/env python3
"""Start the hardware-side consumer of an existing metric-depth pipeline."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('px4_uavcup_perception')
    default_config = os.path.join(
        package_share, 'config', 'perception_jetson.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/uav/depth/image',
            description=(
                'Existing metric depth topic (sensor_msgs/Image, 32FC1, m)'),
        ),
        Node(
            package='px4_uavcup_perception',
            executable='free_space_node',
            name='free_space_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config'),
                {'depth_topic': LaunchConfiguration('depth_topic')},
            ],
        ),
    ])
