#!/usr/bin/env python3
"""계층형 LiDAR obstacle_detector 노드를 실행한다.

사용 예:
  실차:         ros2 launch obstacle_detector obstacle_detector_node.launch.py
  시뮬레이터:   ros2 launch obstacle_detector obstacle_detector_node.launch.py simulator:=true

`simulator:=true`이면 자차 자세 입력을 f110_gym 토픽(`/ego_racecar/odom`)으로 바꾼다.
scan/raceline/map/TF는 자동으로 연결한다. laser->map 변환은 스캔 자체 frame_id를 사용하므로
`ego_racecar/laser`와 `laser` 차이에 별도 설정이 필요하지 않다.

출력은 /static_obs(계층 2), /opp_obs(계층 3), 그리고 각각의 최종 Frenet 경계를 복제한
/static_obs/markers와 /opp_obs/markers이다.
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
    # simulator 인자와 분리되어 있다. f1tenth_gym_ros bridge는 /clock 없이 wall clock으로
    # 동작하므로 그 환경에서 use_sim_time=true이면 노드가 멈춘다. /clock이 있을 때만 켠다.
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='true only if the simulator publishes /clock (f1tenth_gym_ros bridge does NOT -> keep false)',
    )
    # 계층 1 필터용 선택적 CLEAN map이다. 장애물이 구워진 /map을 쓰는 시뮬레이션에서는 검출할
    # 장애물도 지도에 들어 있으므로 /map 필터가 그 장애물을 지워 버린다. 이 인자를 설정하면
    # 전용 nav2 map_server가 장애물 없는 YAML을 /obstacle_detector/map으로 발행한다.
    # 검출기만 이 clean map을 쓰고 나머지 stack은 기존 /map을 계속 사용할 수 있다.
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

    # detector_map_yaml이 있을 때만 latched /obstacle_detector/map을 제공하는 전용 서버
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
