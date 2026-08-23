#!/usr/bin/env python3
"""Start the optimized Jetson TensorRT flight-perception pipeline."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _boolean_argument(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def generate_launch_description():
    package_share = get_package_share_directory('px4_uavcup_perception')
    default_config = os.path.join(
        package_share, 'config', 'perception_jetson.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument(
            'publish_depth_image',
            default_value='false',
            description='Publish the 32FC1 depth image for debugging'),
        DeclareLaunchArgument(
            'publish_visualization',
            default_value='false',
            description='Publish mono8 depth visualization for debugging'),
        DeclareLaunchArgument(
            'publish_pointcloud',
            default_value='false',
            description='Publish PointCloud2 for RViz debugging'),
        Node(
            package='px4_uavcup_perception',
            executable='jetson_depth_node',
            name='jetson_depth_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config'),
                {
                    'publish_depth_image': _boolean_argument(
                        'publish_depth_image'),
                    'publish_visualization': _boolean_argument(
                        'publish_visualization'),
                    'publish_pointcloud': _boolean_argument(
                        'publish_pointcloud'),
                },
            ],
        ),
    ])
