#!/usr/bin/env python3
"""Launch file for the layered LiDAR obstacle_detector node.

Usage:
  Real car:    ros2 launch obstacle_detector obstacle_detector_node.launch.py
  Simulator:   ros2 launch obstacle_detector obstacle_detector_node.launch.py simulator:=true

`simulator:=true` switches the ego pose source to the f110_gym topic (`/ego_racecar/odom`).
Scan/raceline/map/TF are resolved automatically (laser->map uses TF with the scan's own frame_id,
so the `ego_racecar/laser` vs `laser` difference needs no change).

Outputs: /static_obs (Layer 2), /opp_obs (Layer 3), /perception/obstacles/markers (RViz).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('obstacle_detector')
    default_config = os.path.join(pkg_share, 'config', 'obstacle_detector.yaml')

    simulator_arg = DeclareLaunchArgument(
        'simulator',
        default_value='false',
        description='true: use f110_gym topics (/ego_racecar/odom); false: real vehicle (/pf/pose/odom)',
    )
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the obstacle_detector parameter file',
    )
    # NOTE: decoupled from `simulator`. The f1tenth_gym_ros bridge runs on WALL CLOCK (no /clock),
    # so use_sim_time must stay false there or the node stalls. Only true if /clock is published.
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='true only if the simulator publishes /clock (f1tenth_gym_ros bridge does NOT -> keep false)',
    )
    # Optional CLEAN map for the Layer-1 filter. In a sim whose /map is an obstacle-BAKED map
    # (e.g. fuck_f1_obs from the obstacle-map GUI) the baked obstacles sit in /map, so filtering
    # against /map erases exactly the obstacles we want to detect. When this arg is set, a private
    # nav2 map_server serves the given (obstacle-free) yaml on /obstacle_detector/map and the
    # detector filters against THAT while the rest of the stack can keep using the live /map.
    detector_map_arg = DeclareLaunchArgument(
        'detector_map_yaml',
        default_value='',
        description='CLEAN map yaml for the Layer-1 filter (empty = subscribe the normal /map)',
    )

    simulator = LaunchConfiguration('simulator')
    config_file = LaunchConfiguration('config_file')
    detector_map_yaml = LaunchConfiguration('detector_map_yaml')
    have_clean_map = PythonExpression(["'", detector_map_yaml, "' != ''"])
    map_topic = PythonExpression(
        ["'/obstacle_detector/map' if '", detector_map_yaml, "' != '' else '/map'"])
    ego_odom_topic = PythonExpression(
        ["'/ego_racecar/odom' if '", simulator,
         "'.lower() == 'true' else '/pf/pose/odom'"])
    common_params = {'use_sim_time': LaunchConfiguration('use_sim_time'),
                     'map_topic': map_topic,
                     'ego_odom_topic': ego_odom_topic}

    detector_node = Node(
        package='obstacle_detector',
        executable='obstacle_detector_node',
        name='obstacle_detector',
        output='screen',
        parameters=[config_file, common_params],
    )

    # private clean-map server (latched /obstacle_detector/map), only when detector_map_yaml is set
    clean_map_server = Node(
        condition=IfCondition(have_clean_map),
        package='nav2_map_server',
        executable='map_server',
        name='detector_map_server',
        output='screen',
        parameters=[{'yaml_filename': detector_map_yaml,
                     'topic_name': '/obstacle_detector/map',
                     'frame_id': 'map',
                     'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    clean_map_lifecycle = Node(
        condition=IfCondition(have_clean_map),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='detector_map_lifecycle_manager',
        output='screen',
        parameters=[{'autostart': True,
                     'node_names': ['detector_map_server'],
                     'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        simulator_arg,
        config_arg,
        use_sim_time_arg,
        detector_map_arg,
        detector_node,
        clean_map_server,
        clean_map_lifecycle,
    ])
