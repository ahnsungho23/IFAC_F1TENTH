#!/usr/bin/env python3
"""Launch file for the opponent_detector node.

Usage:
  Real car:    ros2 launch opponent_detector opponent_detector.launch.py
  Simulator:   ros2 launch opponent_detector opponent_detector.launch.py simulator:=true

The `simulator:=true` argument switches the ego pose source to the f110_gym topic
(`/ego_racecar/odom`) and sets the `simulator` parameter. Everything else (scan, raceline, map,
TF frames) is resolved automatically: laser->map uses TF with the scan's own frame_id, so the
`ego_racecar/laser` vs `laser` difference needs no manual change.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('opponent_detector')
    default_config = os.path.join(pkg_share, 'config', 'opponent_detector.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz', 'opponent_detector.rviz')

    simulator_arg = DeclareLaunchArgument(
        'simulator',
        default_value='false',
        description='true: use f110_gym topics (/ego_racecar/odom); false: real vehicle (/pf/pose/odom)',
    )
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the opponent_detector parameter file',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='true: also launch RViz2 with the opponent_detector view (map/scan/markers)',
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz,
        description='Path to the RViz config used when rviz:=true',
    )
    # NOTE: decoupled from `simulator`. The f1tenth_gym_ros bridge runs on WALL CLOCK
    # (it does not publish /clock), so use_sim_time must stay false there or the node stalls.
    # Only set true for a simulator that actually publishes /clock.
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='true only if the simulator publishes /clock (f1tenth_gym_ros bridge does NOT -> keep false)',
    )

    simulator = LaunchConfiguration('simulator')
    config_file = LaunchConfiguration('config_file')
    rviz_config = LaunchConfiguration('rviz_config')

    common_params = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
    }

    # Vehicle profile (real car): ego pose from MCL (/pf/pose/odom)
    node_real = Node(
        condition=UnlessCondition(simulator),
        package='opponent_detector',
        executable='opponent_detector_node',
        name='opponent_detector',
        output='screen',
        parameters=[
            config_file,
            common_params,
            {'simulator': False, 'ego_odom_topic': '/pf/pose/odom'},
        ],
    )

    # Simulator profile: ego pose from f110_gym (/ego_racecar/odom)
    node_sim = Node(
        condition=IfCondition(simulator),
        package='opponent_detector',
        executable='opponent_detector_node',
        name='opponent_detector',
        output='screen',
        parameters=[
            config_file,
            common_params,
            {'simulator': True, 'ego_odom_topic': '/ego_racecar/odom'},
        ],
    )

    # Optional RViz2 (shares use_sim_time with the detector)
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[common_params],
    )

    return LaunchDescription([
        simulator_arg,
        config_arg,
        rviz_arg,
        rviz_config_arg,
        use_sim_time_arg,
        node_real,
        node_sim,
        rviz_node,
    ])
