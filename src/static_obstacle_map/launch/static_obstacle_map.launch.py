#!/usr/bin/env python3
"""Launch the persistent confirmed-static obstacle map node."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Launch static_obstacle_map with its YAML parameters."""
    package_share = get_package_share_directory('static_obstacle_map')
    default_params = os.path.join(
        package_share, 'config', 'static_obstacle_map.yaml')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to static_obstacle_map parameter YAML',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='true only when a /clock publisher exists',
    )

    node = Node(
        package='static_obstacle_map',
        executable='static_obstacle_map_node',
        name='static_obstacle_map',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([
        params_arg,
        use_sim_time_arg,
        node,
    ])
