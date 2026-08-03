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
        mode = LaunchConfiguration('start_map_server').perform(context).strip().lower()
        if mode == 'false':
            return [LogInfo(msg='[MCL Launch] start_map_server:=false -> not starting map_server '
                                '(reusing /particle_filter_map_server/map from elsewhere).')]
        if mode != 'true' and _node_running_in_domain('particle_filter_map_server'):
            return [LogInfo(msg="[MCL Launch] 'particle_filter_map_server' already running in this "
                                'ROS_DOMAIN_ID -> reusing it; NOT starting a duplicate '
                                'map_server/lifecycle_manager.')]
        return [map_server_node, lifecycle_manager_node]
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
        default_value=default_config_file,
        description='Path to MCL configuration file'
    )
    map_file_path = PathJoinSubstitution([pkg_share, 'maps', [LaunchConfiguration('map_name'), '.yaml']])
    
    # === DYNAMIC PARAMETERS BASED ON MODE ===
    dynamic_params = {
        # Mode configuration
        'sim_mode': PythonExpression([
            "'true' if '", LaunchConfiguration('mod'), "' == 'sim' else 'false'"
        ]),
        
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
        'publish_odom_base_tf': PythonExpression([
            "'false' if '", LaunchConfiguration('mod'), "' == 'sim' else 'true'"
        ]),

        # ※ 모션/스묽싱 오버라이드는 sim에만 적용한다 (아래 _particle_filter_node_action 참고).
        #    과거에는 real/bag에도 0.15/0.25가 강제로 들어가 YAML(0.08/0.08)이
        #    실차에 적용되지 않는 배선 버그가 있었다 (4de8e1a의 튜닝 의도 묵살).
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
    # 모션/스묽싱(sim 전용) 오버라이드: mod=='sim'일 때만 추가한다.
    # real/bag은 config_file(YAML) 값을 그대로 쓴다 — 이전에는 else 분기 0.15/0.25/0.3이
    # real/bag에도 강제 적용돼 YAML 튜닝이 묵살였다 (주석 "real/bag keep YAML values"와 불일치).
    # gym 시뮬 값 근거: 합성 복도 맵이 near-symmetric이라 큰 dispersion은 코너에서 180° 플립 유발,
    # 무거운 스묽싱은 요 지연 유발. (D6 body-frame 수정 후에도 sim A/B로 확인된 값)
    sim_only_params = {
        'motion_dispersion_x': 0.05,
        'motion_dispersion_theta': 0.04,
        'smoothing_alpha': 0.5,
    }

    def _particle_filter_node_action(context):
        def _coerce(v):
            # 수동 평가된 substitution은 str이 되므로 ROS 파라미터 타입에 맞게 복원
            if isinstance(v, str):
                low = v.lower()
                if low == 'true':
                    return True
                if low == 'false':
                    return False
                try:
                    return int(v)
                except ValueError:
                    pass
                try:
                    return float(v)
                except ValueError:
                    pass
            return v

        def _resolved(d):
            return {k: _coerce(v.perform(context) if hasattr(v, 'perform') else v) for k, v in d.items()}

        params = [
            LaunchConfiguration('config_file').perform(context),
            _resolved(common_params),
            _resolved(dynamic_params),
        ]
        if LaunchConfiguration('mod').perform(context) == 'sim':
            params.append(sim_only_params)

        return [Node(
            package='particle_filter_cpp',
            executable='particle_filter_node',
            name='particle_filter',
            output='screen',
            parameters=params,
            remappings=[
                ('/map_server/map', '/particle_filter_map_server/map')
            ],
            **FAST_SHUTDOWN
        )]

    particle_filter_node = TimerAction(
        period=3.0,  # Allow map server and simulator to initialize
        actions=[OpaqueFunction(function=_particle_filter_node_action)]
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
