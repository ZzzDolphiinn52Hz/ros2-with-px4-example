#!/usr/bin/env python3
"""Start the Urban UAV Cup front-camera perception pipeline."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    del args, kwargs
    world = LaunchConfiguration('world').perform(context)
    model = LaunchConfiguration('model').perform(context)
    config = LaunchConfiguration('config').perform(context)
    depth_python_path = os.path.expanduser(
        LaunchConfiguration('depth_python_path').perform(context))
    inherited_python_path = os.environ.get('PYTHONPATH', '')
    depth_environment = {
        'PYTHONPATH': os.pathsep.join(
            path for path in (depth_python_path, inherited_python_path)
            if path
        ),
    }
    gz_image_topic = (
        f'/world/{world}/model/{model}/link/front_camera_link/'
        'sensor/front_imager/image'
    )

    bridge = Node(
        package='px4_uavcup_perception',
        executable='gz_image_bridge',
        name='gz_image_bridge',
        output='screen',
        parameters=[config, {'gz_image_topic': gz_image_topic}],
    )
    depth = Node(
        package='px4_uavcup_perception',
        executable='depth_anything_node',
        name='depth_anything_node',
        output='screen',
        parameters=[config],
        additional_env=depth_environment,
        condition=IfCondition(LaunchConfiguration('run_depth')),
    )
    free_space = Node(
        package='px4_uavcup_perception',
        executable='free_space_node',
        name='free_space_node',
        output='screen',
        parameters=[config],
        condition=IfCondition(LaunchConfiguration('run_depth')),
    )
    return [bridge, depth, free_space]


def generate_launch_description():
    package_share = get_package_share_directory('px4_uavcup_perception')
    default_config = os.path.join(package_share, 'config', 'perception.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='urban_uavcup'),
        DeclareLaunchArgument('model', default_value='x500_uavcup_0'),
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('run_depth', default_value='true'),
        DeclareLaunchArgument(
            'depth_python_path',
            default_value=(
                '~/ros2_ws/.venv-depth/lib/python3.10/site-packages')),
        OpaqueFunction(function=_setup),
    ])
