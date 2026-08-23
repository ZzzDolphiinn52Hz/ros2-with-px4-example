#!/usr/bin/env python3
"""Start the optimized Jetson TensorRT flight-perception pipeline."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _boolean_argument(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value in ('true', '1', 'yes', 'on'):
        return True
    if value in ('false', '0', 'no', 'off'):
        return False
    raise ValueError(f'{name} must be true or false, got: {value}')


def _setup(context, *args, **kwargs):
    del args, kwargs
    config = LaunchConfiguration('config').perform(context)
    debug_parameters = {
        'publish_depth_image': _boolean_argument(
            context, 'publish_depth_image'),
        'publish_visualization': _boolean_argument(
            context, 'publish_visualization'),
        'publish_pointcloud': _boolean_argument(
            context, 'publish_pointcloud'),
    }
    return [Node(
        package='px4_uavcup_perception',
        executable='jetson_depth_node',
        name='jetson_depth_node',
        output='screen',
        # Keep debug flags out of the exact-node YAML file. Foxy otherwise
        # gives those YAML values precedence over these launch overrides.
        parameters=[debug_parameters, config],
    )]


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
        OpaqueFunction(function=_setup),
    ])
