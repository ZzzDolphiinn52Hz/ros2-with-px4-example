#!/usr/bin/env python3
"""Full simulated ArUco landing stack with all motion gates disabled.

This launch mirrors the real ArUco vehicle pipeline. It replaces only the
Picamera2 socket camera with the Gazebo Harmonic camera adapter. Neither the
landing PID nor the PX4 adapter is enabled by this launch.
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
    simulation_share = get_package_share_directory('px4_uavcup_simulation')

    aruco_config = os.path.join(perception_share, 'config', 'aruco.yaml')
    landing_config = os.path.join(control_share, 'config', 'landing.yaml')
    px4_bridge_config = os.path.join(
        bridge_share, 'config', 'px4_bridge.yaml')
    camera_bridge_config = os.path.join(
        simulation_share, 'config', 'gz_bridge.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'publish_debug_image', default_value='false'),
        Node(
            package='px4_uavcup_simulation',
            executable='gz_camera_bridge',
            name='gazebo_down_camera_bridge',
            output='screen',
            parameters=[camera_bridge_config, {
                'gz_clock_topic': '/world/aruco_landing/clock',
            }],
        ),
        Node(
            package='px4_uavcup_perception',
            executable='aruco_detector_node',
            name='aruco_detector_node',
            output='screen',
            parameters=[aruco_config, {
                'input_mode': 'ros_topic',
                'image_topic': '/camera/down/image_raw',
                'camera_info_topic': '/camera/down/camera_info',
                'camera_frame_id': 'down_camera_optical_frame',
                'publish_debug_topics': True,
                'publish_debug_image': ParameterValue(
                    LaunchConfiguration('publish_debug_image'),
                    value_type=bool),
            }],
        ),
        Node(
            package='px4_uavcup_control',
            executable='aruco_landing_pid_node',
            name='aruco_landing_pid',
            output='screen',
            parameters=[landing_config, {
                'capture_clock_topic': '/uav/sim/clock',
            }],
        ),
        Node(
            package='px4_uavcup_px4_bridge',
            executable='cmd_vel_to_px4',
            name='cmd_vel_to_px4',
            output='screen',
            parameters=[px4_bridge_config],
        ),
    ])
