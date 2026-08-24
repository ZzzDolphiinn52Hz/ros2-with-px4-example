#!/usr/bin/env python3

"""Run USB camera capture and the monocular calibration GUI on the Pi."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('px4_uavcup_perception')
    default_config = os.path.join(
        package_share, 'config', 'camera_calibration.yaml')

    config = LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        Node(
            package='px4_uavcup_perception',
            executable='camera_calibration_publisher',
            name='camera_calibration_publisher',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='camera_calibration',
            executable='cameracalibrator',
            name='front_camera_calibrator',
            output='screen',
            arguments=[
                '--size', '9x6',
                '--square', '0.03',
                '--no-service-check',
            ],
            remappings=[
                ('image', '/camera/front/image_raw'),
                ('camera', '/camera/front'),
            ],
        ),
    ])
