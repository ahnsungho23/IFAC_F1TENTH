#!/usr/bin/env python3
"""
Launch the unified MCL stack.

Supports real hardware, simulation, and bag playback modes.

Usage:
  Real car:      ros2 launch particle_filter_cpp mcl_launch.py mod:=real
  Simulation:    ros2 launch particle_filter_cpp mcl_launch.py mod:=sim
  Bag playback:  ros2 launch particle_filter_cpp mcl_launch.py mod:=bag

  # To change map, launch with map_name:='your_map'
  Example: ros2 launch particle_filter_cpp mcl_launch.py mod:=real map_name:=ifac_track
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Ctrl+C teardown escalation window (SIGINT -> SIGTERM -> SIGKILL) for the launched processes.
# The terminal sends every Ctrl+C to the whole foreground process group, so mashing it can wedge a
# nav2 node mid-shutdown: rclcpp catches SIGINT/SIGTERM, and a second signal arriving during that
# handler leaves the node stuck. launch ignores the repeated SIGINTs and keeps its own escalation
# timers running, so capping them guarantees an *uncatchable* SIGKILL ~4 s later instead of the
# default 5 + 5 = 10 s. Values are strings because launch resolves substitutions at runtime.
FAST_SHUTDOWN = {'sigterm_timeout': '2.0', 'sigkill_timeout': '2.0'}


def _node_running_in_domain(target_name, settle_sec=1.5):
    """
    Return whether a node is already visible in this ROS domain.

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
    """
    Build an action that avoids launching a duplicate map server.

    start_map_server:=auto (default) skips both nodes when a particle_filter_map_server is already
    running in this domain. true forces both nodes and false attaches to a map server provided
    elsewhere.
    """

    def _decide(context):
        mode = LaunchConfiguration('start_map_server').perform(context).strip().lower()
        if mode == 'false':
            return [
                LogInfo(
                    msg='[MCL Launch] start_map_server:=false -> not starting map_server '
                    '(reusing /particle_filter_map_server/map from elsewhere).'
                )
            ]
        if mode != 'true' and _node_running_in_domain('particle_filter_map_server'):
            return [
                LogInfo(
                    msg="[MCL Launch] 'particle_filter_map_server' already running "
                    'in this ROS_DOMAIN_ID -> reusing it; NOT starting a duplicate '
                    'map_server/lifecycle_manager.'
                )
            ]
        return [map_server_node, lifecycle_manager_node]

    return OpaqueFunction(function=_decide)


def generate_launch_description():
    pkg_share = FindPackageShare('particle_filter_cpp')

    # === LAUNCH ARGUMENTS ===
    mode_arg = DeclareLaunchArgument(
        'mod',
        default_value='real',
        description=(
            'Launch mode: real (/odom), sim (/ego_racecar/odom with sim time), '
            'or bag (/odom with sim time)'
        ),
    )

    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='ifac_track',
        description='Map name (without .yaml extension)',
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz visualization',
    )

    start_map_server_arg = DeclareLaunchArgument(
        'start_map_server',
        default_value='auto',
        description=(
            "Map server policy: 'auto' skips an existing server in this ROS domain; "
            "'true' always starts one; 'false' attaches to a server provided elsewhere."
        ),
    )

    # === CONFIGURATION ===
    default_config_file = os.path.join(
        get_package_share_directory('particle_filter_cpp'),
        'config',
        'mcl_config.yaml',
    )

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to MCL configuration file',
    )
    map_file_path = PathJoinSubstitution(
        [pkg_share, 'maps', [LaunchConfiguration('map_name'), '.yaml']]
    )

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
            "'ego_racecar/base_link' if '",
            LaunchConfiguration('mod'),
            "' == 'sim' else 'base_link'",
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
    }

    # === COMMON PARAMETERS ===
    common_params = {
        'use_sim_time': PythonExpression([
            "'true' if '", LaunchConfiguration('mod'), "' in ['sim', 'bag'] else 'false'"
        ]),
    }

    # === MAP SERVER NODE ===
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='particle_filter_map_server',
        output='screen',
        parameters=[
            common_params,
            {'yaml_filename': map_file_path},
        ],
        **FAST_SHUTDOWN,
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
            },
        ],
        **FAST_SHUTDOWN,
    )

    # === PARTICLE FILTER NODE ===
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
                    dynamic_params,
                ],
                remappings=[
                    (
                        '/map_server/map',
                        PythonExpression([
                            "'/map_server/map' if '",
                            LaunchConfiguration('mod'),
                            "' == 'sim' else '/particle_filter_map_server/map'",
                        ]),
                    ),
                ],
                **FAST_SHUTDOWN,
            ),
        ],
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
                'tf_tolerance': 300.0,
            },
        ],
        **FAST_SHUTDOWN,
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
