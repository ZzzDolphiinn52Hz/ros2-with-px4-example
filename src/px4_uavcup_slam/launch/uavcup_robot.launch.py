#!/usr/bin/env python3
"""Bring up only the PX4/Gazebo interfaces required by SLAM or Nav2.

This launch deliberately does not start slam_toolbox, AMCL, a map server, or
RViz. It owns the robot-side portion of the TF tree and ROS interfaces:

* Gazebo lidar + clock -> ``/scan`` + ``/clock``
* PX4 state -> ``/odom`` and ``odom -> base_footprint -> base_link -> link``
* Nav2 ``/cmd_vel`` -> explicitly-enabled PX4 Offboard adapter

The localization layer is responsible for ``map -> odom``.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    world = LaunchConfiguration('world').perform(context)
    model = LaunchConfiguration('model').perform(context)
    use_sim_time = _as_bool(
        LaunchConfiguration('use_sim_time').perform(context))
    target_altitude_m = float(
        LaunchConfiguration('target_altitude_m').perform(context))

    scan_gz = (
        f'/world/{world}/model/{model}/link/link/sensor/lidar_2d_v2/scan'
    )

    bridge = Node(
        package='px4_uavcup_slam',
        executable='gz_lidar_bridge',
        name='gz_lidar_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'gz_scan_topic': scan_gz,
            'gz_clock_topic': '/clock',
            'ros_scan_topic': 'scan',
            'frame_id': 'link',
            'publish_rate_hz': 30.0,
            'max_tilt_deg': 5.0,
            'min_mapping_altitude_m': 0.5,
            'attitude_topic': '/fmu/out/vehicle_attitude',
            'local_position_topic': '/fmu/out/vehicle_local_position_v1',
        }],
    )

    odom_tf = Node(
        package='px4_uavcup_slam',
        executable='px4_odom_tf',
        name='px4_odom_tf',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'body_frame': 'base_link',
            'laser_frame': 'link',
            'laser_xyz': [0.12, 0.0, 0.26],
            'publish_rate_hz': 30.0,
        }],
    )

    cmd_vel_adapter = Node(
        package='px4_uavcup_slam',
        executable='cmd_vel_to_px4',
        name='cmd_vel_to_px4',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_altitude_m': target_altitude_m,
            'max_xy_speed_m_s': 0.4,
            'max_yaw_rate_rad_s': 0.3,
            'max_xy_accel_m_s2': 0.3,
            'max_yaw_accel_rad_s2': 0.5,
            'cmd_timeout_s': 0.5,
            'publish_rate_hz': 20.0,
        }],
        condition=IfCondition(LaunchConfiguration('cmd_vel_adapter')),
    )

    return [bridge, odom_tf, cmd_vel_adapter]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='urban_uavcup'),
        DeclareLaunchArgument('model', default_value='x500_lidar_2d_0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('cmd_vel_adapter', default_value='true'),
        DeclareLaunchArgument('target_altitude_m', default_value='0.7'),
        OpaqueFunction(function=_launch_setup),
    ])
