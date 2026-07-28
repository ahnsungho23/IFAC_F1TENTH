#!/usr/bin/env python3
"""Package entry point for the layered LiDAR obstacle detector (+ optional RViz).

Usage:
  Real car:  ros2 launch obstacle_detector obstacle_detector.launch.py
  Simulator: ros2 launch obstacle_detector obstacle_detector.launch.py simulator:=true
  With RViz: ros2 launch obstacle_detector obstacle_detector.launch.py rviz:=true

This launch starts only ``obstacle_detector_node``. Path planning and state-machine behavior are
outside this package.
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
    pkg_share = get_package_share_directory('obstacle_detector')
    default_config = os.path.join(pkg_share, 'config', 'obstacle_detector.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz', 'obstacle_detector.rviz')

    simulator_arg = DeclareLaunchArgument(
        'simulator',
        default_value='false',
        description='true: /ego_racecar/odom; false: /pf/pose/odom',
    )
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Parameter file for obstacle_detector_node',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='true only when a /clock publisher exists',
    )
    detector_map_arg = DeclareLaunchArgument(
        'detector_map_yaml',
        default_value='',
        description='Optional obstacle-free map YAML for the Layer-1 filter',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2 with the obstacle-detector view',
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz,
        description='RViz configuration file',
    )

    detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'obstacle_detector_node.launch.py')
        ),
        launch_arguments={
            'simulator': LaunchConfiguration('simulator'),
            'config_file': LaunchConfiguration('config_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'detector_map_yaml': LaunchConfiguration('detector_map_yaml'),
        }.items(),
    )

    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        simulator_arg,
        config_arg,
        use_sim_time_arg,
        detector_map_arg,
        rviz_arg,
        rviz_config_arg,
        detector,
        rviz_node,
    ])
