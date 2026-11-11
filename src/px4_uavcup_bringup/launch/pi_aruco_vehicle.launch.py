#!/usr/bin/env python3
"""Launch the IMX500 ArUco landing and PX4 adapter stack.

This profile intentionally excludes ZipDepth and the front camera.  The
landing PID and PX4 adapter both remain disabled until their services are
called explicitly.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    perception_share = get_package_share_directory('px4_uavcup_perception')
    control_share = get_package_share_directory('px4_uavcup_control')
    bridge_share = get_package_share_directory('px4_uavcup_px4_bridge')

    aruco_config = os.path.join(perception_share, 'config', 'aruco.yaml')
    landing_config = os.path.join(control_share, 'config', 'landing.yaml')
    bridge_config = os.path.join(bridge_share, 'config', 'px4_bridge.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'publish_aruco_debug_topics', default_value='false'),
        DeclareLaunchArgument(
            'publish_aruco_debug_image', default_value='false'),
        Node(
            package='px4_uavcup_perception',
            executable='aruco_detector_node',
            name='aruco_detector_node',
            output='screen',
            parameters=[aruco_config, {
                'publish_debug_topics': ParameterValue(
                    LaunchConfiguration('publish_aruco_debug_topics'),
                    value_type=bool),
                'publish_debug_image': ParameterValue(
                    LaunchConfiguration('publish_aruco_debug_image'),
                    value_type=bool),
            }],
        ),
        Node(
            package='px4_uavcup_control',
            executable='aruco_landing_pid_node',
            name='aruco_landing_pid',
            output='screen',
            parameters=[landing_config],
        ),
        Node(
            package='px4_uavcup_px4_bridge',
            executable='cmd_vel_to_px4',
            name='cmd_vel_to_px4',
            output='screen',
            parameters=[bridge_config],
        ),
    ])
