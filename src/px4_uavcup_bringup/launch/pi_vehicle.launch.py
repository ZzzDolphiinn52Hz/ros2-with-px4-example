#!/usr/bin/env python3
"""Launch the Pi vehicle stack: cameras, ZipDepth, ArUco, PID and PX4 adapter."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('px4_uavcup_bringup')
    cameras_config = os.path.join(bringup_share, 'config', 'pi_cameras.yaml')
    zipdepth_config = os.path.join(bringup_share, 'config', 'zipdepth.yaml')
    aruco_config = os.path.join(bringup_share, 'config', 'aruco.yaml')
    landing_config = os.path.join(bringup_share, 'config', 'landing.yaml')
    bridge_config = os.path.join(bringup_share, 'config', 'px4_bridge.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('publish_raw_output', default_value='false'),
        DeclareLaunchArgument('publish_metric_depth', default_value='false'),
        DeclareLaunchArgument('publish_visualization', default_value='false'),
        DeclareLaunchArgument('publish_pointcloud', default_value='false'),
        DeclareLaunchArgument(
            'publish_aruco_debug_topics', default_value='false'),
        DeclareLaunchArgument(
            'publish_aruco_debug_image', default_value='false'),
        # ZipDepth opens the persistent USB by-id device directly by default.
        # Enable this only when camera_device is empty in the YAML and a ROS
        # image topic is desired.
        DeclareLaunchArgument('front_usb_camera', default_value='false'),
        DeclareLaunchArgument('down_picamera', default_value='true'),
        Node(
            package='px4_uavcup_perception',
            executable='v4l2_camera_node',
            name='front_usb_camera',
            output='screen',
            parameters=[cameras_config],
            condition=IfCondition(LaunchConfiguration('front_usb_camera')),
        ),
        Node(
            package='px4_uavcup_perception',
            executable='picamera2_socket_camera_node',
            name='down_picamera',
            output='screen',
            parameters=[cameras_config],
            condition=IfCondition(LaunchConfiguration('down_picamera')),
        ),
        Node(
            package='px4_uavcup_perception',
            executable='zipdepth_node',
            name='zipdepth_node',
            output='screen',
            parameters=[zipdepth_config, {
                'publish_raw_output': ParameterValue(
                    LaunchConfiguration('publish_raw_output'),
                    value_type=bool),
                'publish_metric_depth': ParameterValue(
                    LaunchConfiguration('publish_metric_depth'),
                    value_type=bool),
                'publish_visualization': ParameterValue(
                    LaunchConfiguration('publish_visualization'),
                    value_type=bool),
                'publish_pointcloud': ParameterValue(
                    LaunchConfiguration('publish_pointcloud'),
                    value_type=bool),
            }],
        ),
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
