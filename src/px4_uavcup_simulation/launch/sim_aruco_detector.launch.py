#!/usr/bin/env python3
"""Bridge the Gazebo down camera and run the production ArUco detector.

This phase-2 profile intentionally contains no landing controller, PX4 bridge
or publisher to /fmu/in/*. It is safe for perception-only simulation checks.
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
    simulation_share = get_package_share_directory('px4_uavcup_simulation')
    aruco_config = os.path.join(perception_share, 'config', 'aruco.yaml')
    bridge_config = os.path.join(simulation_share, 'config', 'gz_bridge.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('publish_debug_image', default_value='true'),
        Node(
            package='px4_uavcup_simulation',
            executable='gz_camera_bridge',
            name='gazebo_down_camera_bridge',
            output='screen',
            parameters=[bridge_config],
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
    ])
