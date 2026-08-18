#!/usr/bin/env python3
"""
Unified MCL Launch File
Supports real hardware, simulation, and bag playback modes

Usage:
  Real car:      ros2 launch particle_filter_cpp mcl_launch.py mod:=real
  Simulation:    ros2 launch particle_filter_cpp mcl_launch.py mod:=sim  
  Bag playback:  ros2 launch particle_filter_cpp mcl_launch.py mod:=bag
  
  # To change map, launch with map_name:='your_map'
  Example:       ros2 launch particle_filter_cpp mcl_launch.py mod:=real map_name:='my_custom_map'
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


# Ctrl+C teardown escalation window (SIGINT -> SIGTERM -> SIGKILL) for the launched processes.
# The terminal sends every Ctrl+C to the whole foreground process group, so mashing it can wedge a
# nav2 node mid-shutdown: rclcpp catches SIGINT/SIGTERM, and a second signal arriving during that
# handler leaves the node stuck. launch ignores the repeated SIGINTs and keeps its own escalation
# timers running, so capping them guarantees an *uncatchable* SIGKILL ~4 s later instead of the
# default 5 + 5 = 10 s that reads as "the node won't die". Values are strings (launch substitutions).
FAST_SHUTDOWN = {'sigterm_timeout': '2.0', 'sigkill_timeout': '2.0'}


def _node_running_in_domain(target_name, settle_sec=1.5):
    """Return True if a node named `target_name` is already visible in this ROS_DOMAIN_ID.

    Stops a second mcl_launch from spawning a duplicate map_server: two nodes sharing the
    fully-qualified name /particle_filter_map_server both publish /particle_filter_map_server/map
    and their two lifecycle managers fight over configure/activate transitions, so RViz and the
    particle filter see a map that flaps or never activates. This happens whenever mcl_launch is
    started twice in one domain -- e.g. real/all.launch.py already includes it and a debug
    `ros2 launch particle_filter_cpp mcl_launch.py` is run alongside, or the launch is restarted
    before the previous particle_filter_map_server is reaped.

    rclpy honours ROS_DOMAIN_ID, so this only ever sees nodes in the *same* domain -- exactly the
    scope the check must cover. Any failure (no rclpy, discovery hiccup) returns False so the
    map_server still starts: never worse than the unconditional launch.
    """
    try:
        import time
        import rclpy
    except Exception:
        return False

    started_here = False
    probe = None
    try:
        if not rclpy.ok():
            rclpy.init()
            started_here = True
        probe = rclpy.create_node('mcl_map_server_probe_%d' % os.getpid())
        deadline = time.monotonic() + settle_sec
        while time.monotonic() < deadline:
            if target_name in probe.get_node_names():
                return True
            rclpy.spin_once(probe, timeout_sec=0.1)
        return target_name in probe.get_node_names()
    except Exception:
        return False
    finally:
        try:
            if probe is not None:
                probe.destroy_node()
        except Exception:
            pass
        if started_here:
            try:
                rclpy.shutdown()
            except Exception:
                pass


def _map_server_actions(map_server_node, lifecycle_manager_node):
    """OpaqueFunction body: launch the map_server pair only when it won't duplicate one already up.

    start_map_server:=auto (default) skips both nodes when a particle_filter_map_server is already
    running in this domain; :=true forces them (old behaviour); :=false never starts them (attach to
    a map_server provided elsewhere).
    """
    def _decide(context):
        from launch.actions import ExecuteProcess
        mode = LaunchConfiguration('start_map_server').perform(context).strip().lower()
        if mode == 'false':
            return [LogInfo(msg='[MCL Launch] start_map_server:=false -> not starting map_server '
                                '(reusing /particle_filter_map_server/map from elsewhere).')]
        if mode != 'true' and _node_running_in_domain('particle_filter_map_server'):
            return [LogInfo(msg="[MCL Launch] 'particle_filter_map_server' already running in this "
                                'ROS_DOMAIN_ID -> reusing it; NOT starting a duplicate '
                                'map_server/lifecycle_manager.')]

        # Check if nav2_lifecycle_manager package exists
        try:
            from ament_index_python.packages import get_package_share_directory
            get_package_share_directory('nav2_lifecycle_manager')
            return [map_server_node, lifecycle_manager_node]
        except Exception:
            # Fallback: launch map_server and configure/activate via CLI commands
            activate_cmd = ExecuteProcess(
                cmd=['bash', '-c', 'sleep 1.0; ros2 lifecycle set /particle_filter_map_server configure; sleep 0.5; ros2 lifecycle set /particle_filter_map_server activate'],
                output='screen'
            )
            return [map_server_node, activate_cmd]
    return OpaqueFunction(function=_decide)


def generate_launch_description():
    # Get package share directory  
    from ament_index_python.packages import get_package_share_directory
    pkg_share_dir = get_package_share_directory('particle_filter_cpp')
    pkg_share = FindPackageShare('particle_filter_cpp')
    
    # === LAUNCH ARGUMENTS ===
    mode_arg = DeclareLaunchArgument(
        'mod',
        default_value='real',
        description='Launch mode: real (real car using /odom), sim (simulation, use sim time, /ego_racecar/odom), bag (bag file play, use sim time, /odom)'
    )
    
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='map',
        description='Map name (without .yaml extension)'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz visualization'
    )

    start_map_server_arg = DeclareLaunchArgument(
        'start_map_server',
        default_value='auto',
        description="Map server policy: 'auto' skips it when a particle_filter_map_server is already "
                    "running in this ROS_DOMAIN_ID (prevents duplicates); 'true' always starts it; "
                    "'false' never starts it (attach to a map_server provided elsewhere)."
    )
    
    # === CONFIGURATION ===
    # Try to find source config first, fallback to install config
    install_config_file = os.path.join(
        get_package_share_directory('particle_filter_cpp'),
        'config',
        'mcl_config.yaml'
    )
    
    # Look for source config relative to install directory
    install_dir = get_package_share_directory('particle_filter_cpp')
    potential_source_config = os.path.join(install_dir, '..', '..', '..', '..', 'src', 'perception_ws', 'monte_carlo_localization', 'config', 'mcl_config.yaml')
    potential_source_config = os.path.abspath(potential_source_config)
    
    # Use source config if it exists, otherwise use install config
    if os.path.exists(potential_source_config):
        default_config_file = potential_source_config
        print(f"[MCL Launch] Using SOURCE config: {default_config_file}")
    else:
        default_config_file = install_config_file
        print(f"[MCL Launch] Using INSTALL config: {default_config_file}")
    
    config_arg = DeclareLaunchArgument(
        'config_file',
        # mod에 따라 기본 설정 파일 선택 (sim → mcl_config_sim.yaml, real/bag → mcl_config.yaml).
        # 튜닝값의 단일 소스는 YAML — launch는 값을 오버라이드하지 않고 파일만 고른다.
        default_value=PythonExpression([
            "'", os.path.join(pkg_share_dir, 'config', 'mcl_config_sim.yaml'),
            "' if '", LaunchConfiguration('mod'), "' == 'sim' else '", default_config_file, "'"
        ]),
        description='Path to MCL configuration file (default: mod-dependent)'
    )
    map_file_path = PathJoinSubstitution([pkg_share, 'maps', [LaunchConfiguration('map_name'), '.yaml']])
    
    # === DYNAMIC PARAMETERS BASED ON MODE ===
    dynamic_params = {
        # Topic names
        'scan_topic': '/scan',  # All modes use /scan
        'odom_topic': PythonExpression([
            "'/ego_racecar/odom' if '", LaunchConfiguration('mod'), "' == 'sim' else '/odom'"
        ]),
        
        # TF frame names
        'odom_frame': PythonExpression([
            "'ego_racecar/odom' if '", LaunchConfiguration('mod'), "' == 'sim' else 'odom'"
        ]),
        'base_frame': PythonExpression([
            "'ego_racecar/base_link' if '", LaunchConfiguration('mod'), "' == 'sim' else 'base_link'"
        ]),
        'laser_frame': PythonExpression([
            "'ego_racecar/laser' if '", LaunchConfiguration('mod'), "' == 'sim' else 'laser'"
        ]),
        
        # TF publishing control
        'publish_map_odom_tf': PythonExpression([
            "'false' if '", LaunchConfiguration('mod'), "' == 'sim' else 'true'"
        ]),
        # 🔴 real 도 false 다 (2026-08-17 수정). 예전엔 real 에서 'true' 로 덮었는데,
        #    real 은 f1tenth_stack 의 vesc_to_odom 이 odom->base_link 를 이미 발행한다
        #    (config/mcl_config.yaml 의 "Let F1Tenth stack handle odom->base_link" 가 그 뜻).
        #    그래서 **같은 변환을 두 노드가 동시에 발행**하고 있었다 — run_0817_165444 실측:
        #      · odom->base_link 발행률 89.5 Hz (= vesc_to_odom 50 + MCL 40)
        #      · 두 발행자가 6 ms 안에 겹친 비율 24.7%
        #      · 그때 위치 불일치 p50 8.5 cm / p90 12.7 / max 18.5, 헤딩 p50 1.01°
        #        (6 ms 에 8.5 cm 는 14 m/s 가 필요 = 물리적으로 이동이 아니라 불일치다)
        #    tf2 버퍼는 (parent,child) 하나에 두 소스를 섞어 넣으므로, 조회 결과가 "어느
        #    발행자가 마지막이었나"에 따라 갈린다 → 로컬플래닝이 장애물을 map 프레임으로
        #    옮길 때마다 90 Hz 톱니 8.5 cm 가 실린다(코너 MCL 오차 13 cm 에 맞먹는다).
        #    ⚠️ map->odom(publish_map_odom_tf)은 MCL 만 내므로 real=true 그대로 둘 것.
        # 🔴 파이썬 bool 이어야 한다 (2026-08-18). 'false' 라는 **문자열**을 주면
        #    launch_ros 가 문자열 파라미터로 그대로 넘기고, 노드는 이 이름을 bool 로
        #    선언해 두었으므로 InvalidParameterTypeException 으로 즉시 죽는다.
        #    위 publish_map_odom_tf 는 PythonExpression(=치환)이라 launch_ros 가 타입을
        #    추론해 주지만, 평범한 파이썬 문자열에는 그 추론이 걸리지 않는다.
        'publish_odom_base_tf': False,

        # ※ 튜닝값(모션 노이즈/스묻싱 등)은 전부 YAML이 단일 소스. launch는
        #    모드 배선(토픽/프레임/TF 플래그)과 설정 파일 선택만 담당한다.
    }
    
    # === COMMON PARAMETERS ===
    common_params = {
        'use_sim_time': PythonExpression([
            "'true' if '", LaunchConfiguration('mod'), "' in ['sim', 'bag'] else 'false'"
        ])
    }
    
    # === MAP SERVER NODE ===
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='particle_filter_map_server',
        output='screen',
        parameters=[
            common_params,
            {'yaml_filename': map_file_path}
        ],
        **FAST_SHUTDOWN
    )
    
    # === LIFECYCLE MANAGER ===
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_particle_filter',
        output='screen',
        parameters=[
            common_params,
            {
                'autostart': True,
                'node_names': ['particle_filter_map_server'],
                # Don't wait on the managed-node heartbeat during teardown: a short bond timeout
                # keeps the manager from blocking on a map_server that is already being killed.
                'bond_timeout': 0.0,
            }
        ],
        **FAST_SHUTDOWN
    )
    
    # === PARTICLE FILTER NODE ===
    # 튜닝값은 전부 config_file(YAML)에서 온다 — launch가 값을 오버라이드하지 않는다.
    # (2026-08-03: 과거 else 분기 0.15/0.25가 real/bag에도 강제 적용돼 YAML이 묵살이던
    #  배선 버그를 제거하고, sim 전용 값은 mcl_config_sim.yaml로 분리)
    particle_filter_node = TimerAction(
        period=3.0,  # Allow map server and simulator to initialize
        actions=[
            Node(
                package='particle_filter_cpp',
                executable='particle_filter_node',
                name='particle_filter',
                output='screen',
                parameters=[
                    LaunchConfiguration('config_file'),
                    common_params,
                    dynamic_params
                ],
                remappings=[
                    ('/map_server/map', '/particle_filter_map_server/map')
                ],
                **FAST_SHUTDOWN
            )
        ]
    )
    
    # === TF TRANSFORMS RESPONSIBILITY ===
    # Real mode: F1Tenth stack provides base_link->laser, MCL provides map->odom->base_link
    # Sim mode:  Simulator provides map->base_link->laser, MCL only does localization
    
    # === RVIZ NODE ===
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'particle_filter.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen',
        parameters=[
            common_params,
            {
                'transform_timeout': 300.0,
                'message_filter_queue_size': 100,
                'tf_buffer_cache_time_s': 300.0,
                'tf_tolerance': 300.0
            }
        ],
        **FAST_SHUTDOWN
    )

    return LaunchDescription([
        # Launch arguments
        mode_arg,
        map_name_arg,
        use_rviz_arg,
        start_map_server_arg,
        config_arg,

        # Nodes
        # map_server + lifecycle_manager are gated so a second mcl_launch in the same
        # ROS_DOMAIN_ID reuses the running particle_filter_map_server instead of duplicating it.
        _map_server_actions(map_server_node, lifecycle_manager_node),
        particle_filter_node,
        rviz_node,
    ])
