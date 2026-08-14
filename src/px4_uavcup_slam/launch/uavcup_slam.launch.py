#!/usr/bin/env python3
"""
Launch: Harmonic-compatible GZ→ROS lidar/clock bridge + PX4 odom/TF + slam_toolbox.

Prerequisites:
  1) make px4_sitl gz_x500_lidar_2d_urban_uavcup
  2) MicroXRCEAgent udp4 -p 8888
  3) source ~/ros2_ws/install/setup.bash  (NOT Documents/ros2_ws)

Note: ros_gz_bridge (Humble apt) is built for Fortress/ignition and CANNOT
decode Harmonic (gz-msgs10) LaserScan — use gz_lidar_bridge instead.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    model = LaunchConfiguration('model').perform(context)
    pkg = get_package_share_directory('px4_uavcup_slam')

    scan_gz = (
        f'/world/{world}/model/{model}/link/link/sensor/lidar_2d_v2/scan'
    )

    bridge = Node(
        package='px4_uavcup_slam',
        executable='gz_lidar_bridge',
        name='gz_lidar_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': True,
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
            'use_sim_time': True,
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'body_frame': 'base_link',
            'laser_frame': 'link',
            'laser_xyz': [0.12, 0.0, 0.26],
            'publish_rate_hz': 30.0,
        }],
    )

    slam_params = os.path.join(pkg, 'config', 'mapper_params_online_async.yaml')
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params,
            {'use_sim_time': True},
        ],
    )

    rviz_config = os.path.join(pkg, 'rviz', 'uavcup_slam.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return [bridge, odom_tf, slam, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='urban_uavcup'),
        DeclareLaunchArgument('model', default_value='x500_lidar_2d_0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=_launch_setup),
    ])
