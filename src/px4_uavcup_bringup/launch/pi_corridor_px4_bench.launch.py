#!/usr/bin/env python3
"""Bench-only corridor-to-PX4 stack with two explicit enable gates."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    control_share = get_package_share_directory('px4_uavcup_control')
    bringup_share = get_package_share_directory('px4_uavcup_bringup')
    shadow_launch = os.path.join(
        control_share, 'launch', 'pi_zipdepth_shadow.launch.py')
    gate_config = os.path.join(
        control_share, 'config', 'corridor_velocity_gate.yaml')
    bridge_config = os.path.join(
        bringup_share, 'config', 'px4_bridge.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('publish_visualization', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(shadow_launch),
            launch_arguments={
                'publish_visualization': LaunchConfiguration(
                    'publish_visualization'),
            }.items(),
        ),
        Node(
            package='px4_uavcup_control',
            executable='corridor_velocity_gate',
            name='corridor_velocity_gate',
            output='screen',
            parameters=[gate_config],
        ),
        Node(
            package='px4_uavcup_px4_bridge',
            executable='cmd_vel_to_px4',
            name='cmd_vel_to_px4',
            output='screen',
            parameters=[bridge_config, {
                'cmd_vel_topic': '/uav/local_controller/cmd_vel',
                # Bench step: corridor Z must not alter altitude yet.
                'allow_vertical_cmd_vel': False,
                'max_xy_speed_m_s': 0.20,
            }],
        ),
    ])
