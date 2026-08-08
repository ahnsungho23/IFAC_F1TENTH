# Copyright 2026 2026_IFAC contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    pkg_dir = get_package_share_directory('local_planning')
    default_config = os.path.join(pkg_dir, 'config', 'local_planning.yaml')
    default_reference_map = os.path.join(
        get_package_share_directory('particle_filter_cpp'),
        'maps',
        os.environ.get('F1_MAP', 'ifac_track') + '.yaml',
    )
    obstacle_detector_launch = os.path.join(
        get_package_share_directory('obstacle_detector'),
        'launch',
        'obstacle_detector.launch.py',
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_config,
        description='Full path to the local_planning YAML parameter file'
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
    simulator_arg = DeclareLaunchArgument(
        'simulator',
        default_value='true',
        description='Use gym ego odometry for the perception node',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use ROS simulation time for all nodes (requires a /clock publisher)',
    )
    start_detector_arg = DeclareLaunchArgument(
        'start_obstacle_detector',
        default_value='true',
        description='Start the obstacle_detector that publishes /static_obs',
    )
    planning_map_topic_arg = DeclareLaunchArgument(
        'planning_map_topic',
        default_value='/local_planning/reference_map',
        description=(
            'Wall-only reference map used by perception and local planning. '
            'In gym, /map may be the obstacle-baked physics map.'
        ),
    )
    reference_map_arg = DeclareLaunchArgument(
        'reference_map',
        default_value=default_reference_map,
        description='Wall-only reference-map YAML loaded for local planning',
    )

    reference_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='local_planning_map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('reference_map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[('/map', LaunchConfiguration('planning_map_topic'))],
    )
    reference_map_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='local_planning_map_lifecycle_manager',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['local_planning_map_server'],
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    obstacle_detector = GroupAction(actions=[
        SetRemap(src='/map', dst=LaunchConfiguration('planning_map_topic')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(obstacle_detector_launch),
            condition=IfCondition(LaunchConfiguration('start_obstacle_detector')),
            launch_arguments={
                'simulator': LaunchConfiguration('simulator'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'rviz': 'false',
                'runtime_profile': LaunchConfiguration('runtime_profile'),
            }.items(),
        ),
    ])

    local_planner_node = Node(
        package='local_planning',
        executable='local_planner_node',
        name='local_planner_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            LaunchConfiguration('runtime_profile'),
        ]
    )

    return LaunchDescription([
        params_file_arg,
        runtime_profile_arg,
        simulator_arg,
        use_sim_time_arg,
        start_detector_arg,
        planning_map_topic_arg,
        reference_map_arg,
        reference_map_server,
        reference_map_lifecycle,
        obstacle_detector,
        local_planner_node
    ])
