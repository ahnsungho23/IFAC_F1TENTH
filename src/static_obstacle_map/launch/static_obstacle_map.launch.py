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
    runtime_profile_arg = DeclareLaunchArgument(
        'runtime_profile',
        default_value=os.path.join(
            get_package_share_directory('f1tenth_control'),
            'config',
            'runtime_visualization.yaml',
        ),
        description='Shared visualization/output profile YAML',
    )

    node = Node(
        package='static_obstacle_map',
        executable='static_obstacle_map_node',
        name='static_obstacle_map',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            LaunchConfiguration('runtime_profile'),
        ],
    )

    return LaunchDescription([
        params_arg,
        use_sim_time_arg,
        runtime_profile_arg,
        node,
    ])
