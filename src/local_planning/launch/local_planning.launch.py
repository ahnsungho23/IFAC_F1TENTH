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


# ============================================================================
# local_planning 통합 launch
# ============================================================================
# 기본 구성:
#   1. 장애물이 포함되지 않은 wall-only 기준 지도를 별도 토픽으로 제공
#   2. obstacle_detector를 함께 실행해 /static_obs 생성
#   3. local_planner_node가 /static_obs를 회피 경로로 변환
#
# 이미 외부 obstacle_detector를 실행했다면 start_obstacle_detector:=false로 중복 실행을 막는다.
def generate_launch_description():
    # 설치된 package share를 기준으로 config와 포함 launch의 절대 경로를 만든다.
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

    # ── 사용자가 명령행에서 덮어쓸 수 있는 launch 인자 ──────────────────────
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_config,
        description='Full path to the local_planning YAML parameter file'
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

    # ── local planning 전용 wall-only map server ─────────────────────────────
    # gym의 /map에는 시뮬레이션 장애물이 구워져 있을 수 있으므로 perception 필터에는 별도의
    # 깨끗한 기준 지도를 공급한다.
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

    # ── obstacle_detector 선택 실행 ──────────────────────────────────────────
    # detector가 보는 /map만 planning_map_topic으로 remap한다. RViz는 이 통합 launch에서
    # 중복으로 띄우지 않는다.
    obstacle_detector = GroupAction(actions=[
        SetRemap(src='/map', dst=LaunchConfiguration('planning_map_topic')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(obstacle_detector_launch),
            condition=IfCondition(LaunchConfiguration('start_obstacle_detector')),
            launch_arguments={
                'simulator': LaunchConfiguration('simulator'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'rviz': 'false',
            }.items(),
        ),
    ])

    # ── 정적 장애물 회피 계획 노드 ──────────────────────────────────────────
    local_planner_node = Node(
        package='local_planning',
        executable='local_planner_node',
        name='local_planner_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # 선언 순서와 무관하게 모든 action이 하나의 LaunchDescription으로 함께 관리된다.
    return LaunchDescription([
        params_file_arg,
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
