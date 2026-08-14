#!/usr/bin/env python3
"""Start AMCL + Navigation2 against the saved UAV Cup occupancy map.

Start ``px4_uavcup_slam/uavcup_robot.launch.py`` first. That robot-side launch
provides /clock, /scan, /odom, odom -> base_footprint, and the disabled-by-
default /cmd_vel adapter. This launch owns localization/navigation and AMCL is
the sole publisher of map -> odom.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('px4_uavcup_nav')
    nav2_share = get_package_share_directory('nav2_bringup')

    default_map = os.path.join(
        os.path.expanduser('~'), 'ros2_ws', 'maps', 'uavcup_map.yaml')
    default_params = os.path.join(
        package_share, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(
        package_share, 'rviz', 'uavcup_nav.rviz')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            # Humble bringup uses PythonExpression(['not ', slam]); retain a
            # Python boolean literal rather than lower-case launch syntax.
            'slam': 'False',
            'autostart': 'True',
            # Separate processes give clearer node logs during first bringup.
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_nav',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='Absolute path to the static map YAML'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Navigation2 parameters for the UAV'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        nav2,
        rviz,
    ])
