from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_cfg = PathJoinSubstitution([FindPackageShare('lap_timer'), 'rviz', 'lap_hud.rviz'])
    runtime_profile = PathJoinSubstitution([
        FindPackageShare('f1tenth_control'), 'config', 'runtime_visualization.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument(
            'runtime_profile',
            default_value=runtime_profile,
            description='Shared visualization/output profile YAML',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch the lap-timer RViz view',
        ),
        Node(
            package='lap_timer',
            executable='lap_timer',
            name='lap_timer',
            parameters=[{'odom_topic': '/car_state/frenet/odom'},
                        {'start_window': 0.5},
                        {'wrap_threshold': 10.0},
                        {'min_lap_time': 3.0},
                        {'frame_id': 'map'},
                        {'use_position_x_as_s': True},
                        {'rviz_text': True},
                        {'exact_zero_mode': False},
                        {'exact_zero_eps': 0.05},
                        LaunchConfiguration('runtime_profile')],
            output='screen'
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg]
        )
    ])
