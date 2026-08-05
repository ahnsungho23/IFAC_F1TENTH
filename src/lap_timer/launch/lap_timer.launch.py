from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    rviz_cfg = PathJoinSubstitution([FindPackageShare('lap_timer'), 'rviz', 'lap_hud.rviz'])
    return LaunchDescription([
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
                        {'exact_zero_eps': 0.05}],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg]
        )
    ])

