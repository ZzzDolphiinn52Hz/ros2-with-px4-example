#!/usr/bin/env python3
"""
Mapping bringup: robot interfaces + slam_toolbox + RViz.

Prerequisites:
  1) make px4_sitl gz_x500_lidar_2d_urban_uavcup
  2) MicroXRCEAgent udp4 -p 8888
  3) source ~/ros2_ws/install/setup.bash  (NOT Documents/ros2_ws)

Note: ros_gz_bridge (Humble apt) is built for Fortress/ignition and CANNOT
decode Harmonic (gz-msgs10) LaserScan — use gz_lidar_bridge instead.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    pkg = get_package_share_directory('px4_uavcup_slam')
    use_sim_time = (
        LaunchConfiguration('use_sim_time').perform(context).strip().lower()
        in ('1', 'true', 'yes', 'on')
    )

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'uavcup_robot.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'model': LaunchConfiguration('model'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'cmd_vel_adapter': LaunchConfiguration('cmd_vel_adapter'),
            'target_altitude_m': LaunchConfiguration('target_altitude_m'),
        }.items(),
    )

    slam_params = os.path.join(pkg, 'config', 'mapper_params_online_async.yaml')
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params,
            {'use_sim_time': use_sim_time},
        ],
    )

    rviz_config = os.path.join(pkg, 'rviz', 'uavcup_slam.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return [robot_launch, slam, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='urban_uavcup'),
        DeclareLaunchArgument('model', default_value='x500_lidar_2d_0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('cmd_vel_adapter', default_value='true'),
        DeclareLaunchArgument('target_altitude_m', default_value='0.7'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=_launch_setup),
    ])
