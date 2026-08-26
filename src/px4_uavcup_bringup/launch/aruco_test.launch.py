#!/usr/bin/env python3
"""Test IMX500 ArUco pose output without PID, Offboard, or PX4 topics."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    perception_share = get_package_share_directory('px4_uavcup_perception')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                perception_share, 'launch', 'aruco_pi_test.launch.py')),
        ),
    ])
