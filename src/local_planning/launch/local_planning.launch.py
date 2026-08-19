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

"""
Start the full static-obstacle perception + local planning stack.

Ownership note: local_planner_node itself subscribes to NO occupancy map. The reference map
server started here exists solely for the obstacle_detector that produces /static_obs and
/confirmed_static_obs (the planner consumes the confirmed-only one), so it is
gated by the same start_obstacle_detector condition. With start_obstacle_detector:=false this
launch reduces to local_planner_only.launch.py, which it includes for the planner node itself.
"""

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
        # 기본값 'map' (2026-08-18): ifac_track 맵은 삭제됐다. 세 노드(MCL/global/local)가
        # 같은 맵을 보도록 global_planning·mcl_launch와 기본값을 맞춘다.
        os.environ.get('F1_MAP', 'map') + '.yaml',
    )
    local_planner_only_launch = os.path.join(pkg_dir, 'launch', 'local_planner_only.launch.py')
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
    simulator_arg = DeclareLaunchArgument(
        'simulator',
        # 🔴 기본값은 false 다 (2026-08-20 수정). true 는 obstacle_detector 의 ego odom 을
        # /ego_racecar/odom(gym 전용)으로 바꾸는데, 실차에는 그 토픽이 없다.
        #
        # 기본값이 true 이던 동안 실차 백 10 개 · DIAG 창 4262 개에서 예외 없이
        #   motion(fresh=false),  view 기각 합계 0,  "Ego odometry not received yet" 반복
        # 이 찍혔다. 경고 문구가 전부 "not received yet"(= ego_s_ < 0)이고 "stale" 이 단
        # 한 번도 없었다 — 늦게 온 게 아니라 한 번도 안 온 것이다.
        # 그 결과 detector 의 시야창 게이트(`if (ego_s_ >= 0.0)` 안에 있다)가 통째로 꺼져
        # 있었고, /opp_obs 는 전면 억제됐다.
        #
        # 같은 인자의 기본값이 obstacle_detector 쪽 런치 두 개에서는 이미 false 였다.
        # 이 파일만 true 로 덮어써서 실차 실행을 깨뜨리고 있었다.
        # 시뮬에서는 simulator:=true 를 **명시적으로** 넘긴다 (sim/run.sh, CLAUDE.md 참고).
        default_value='false',
        description=(
            'true: gym ego odometry(/ego_racecar/odom) for the perception node. '
            'false (default): real vehicle(/pf/pose/odom)'
        ),
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use ROS simulation time for all nodes (requires a /clock publisher)',
    )
    start_detector_arg = DeclareLaunchArgument(
        'start_obstacle_detector',
        default_value='true',
        description=(
            'Start the obstacle_detector that publishes /static_obs and '
            '/confirmed_static_obs (the planner input)'
        ),
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
    timing_diagnostics_enable_arg = DeclareLaunchArgument(
        'timing_diagnostics_enable',
        default_value='false',
        description='Publish tuning-only monotonic timing events',
    )
    timing_diagnostics_topic_arg = DeclareLaunchArgument(
        'timing_diagnostics_topic',
        default_value='/cma_timing/events',
        description='Companion timing-event topic',
    )
    replay_diagnostics_enable_arg = DeclareLaunchArgument(
        'replay_diagnostics_enable',
        default_value='false',
        description='Publish tuning-only detector/planner record/replay diagnostics',
    )
    detector_replay_diagnostics_topic_arg = DeclareLaunchArgument(
        'detector_replay_diagnostics_topic',
        default_value='/cma_replay/detector_events',
        description='Detector record/replay companion-event topic',
    )
    planner_replay_diagnostics_topic_arg = DeclareLaunchArgument(
        'planner_replay_diagnostics_topic',
        default_value='/cma_replay/planner_events',
        description='Planner record/replay companion-event topic',
    )
    lockstep_mode_arg = DeclareLaunchArgument(
        'lockstep_mode', default_value='false',
        description='CMA-only deterministic event-driven execution',
    )
    p3_mode_arg = DeclareLaunchArgument(
        'p3_mode', default_value='TEST_ACTIVE',
        description='Production P3/M1 mode: OFF, SHADOW, or bounded TEST_ACTIVE',
    )
    p3_diagnostics_topic_arg = DeclareLaunchArgument(
        'p3_diagnostics_topic', default_value='/local_planning/p3_shadow',
        description='Non-commanding P3/M1 ownership and lifecycle diagnostic topic',
    )
    lockstep_scan_offset_arg = DeclareLaunchArgument(
        'lockstep_scan_offset_x_m', default_value='0.275',
        description='CMA lockstep LiDAR x offset from base_link [m]',
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
        condition=IfCondition(LaunchConfiguration('start_obstacle_detector')),
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
        condition=IfCondition(LaunchConfiguration('start_obstacle_detector')),
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
                'replay_diagnostics_enable': LaunchConfiguration(
                    'replay_diagnostics_enable'),
                'replay_diagnostics_topic': LaunchConfiguration(
                    'detector_replay_diagnostics_topic'),
                'lockstep_mode': LaunchConfiguration('lockstep_mode'),
                'lockstep_scan_offset_x_m': LaunchConfiguration(
                    'lockstep_scan_offset_x_m'),
            }.items(),
        ),
    ])

    local_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(local_planner_only_launch),
        launch_arguments={
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'timing_diagnostics_enable': LaunchConfiguration('timing_diagnostics_enable'),
            'timing_diagnostics_topic': LaunchConfiguration('timing_diagnostics_topic'),
            'replay_diagnostics_enable': LaunchConfiguration('replay_diagnostics_enable'),
            'replay_diagnostics_topic': LaunchConfiguration(
                'planner_replay_diagnostics_topic'),
            'lockstep_mode': LaunchConfiguration('lockstep_mode'),
            'p3_mode': LaunchConfiguration('p3_mode'),
            'p3_diagnostics_topic': LaunchConfiguration('p3_diagnostics_topic'),
        }.items(),
    )

    return LaunchDescription([
        params_file_arg,
        simulator_arg,
        use_sim_time_arg,
        start_detector_arg,
        planning_map_topic_arg,
        reference_map_arg,
        timing_diagnostics_enable_arg,
        timing_diagnostics_topic_arg,
        replay_diagnostics_enable_arg,
        detector_replay_diagnostics_topic_arg,
        planner_replay_diagnostics_topic_arg,
        lockstep_mode_arg,
        p3_mode_arg,
        p3_diagnostics_topic_arg,
        lockstep_scan_offset_arg,
        reference_map_server,
        reference_map_lifecycle,
        obstacle_detector,
        local_planner,
    ])
