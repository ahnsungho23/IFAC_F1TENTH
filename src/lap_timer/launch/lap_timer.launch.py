import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = get_package_share_directory('lap_timer')
    default_params = os.path.join(package_share, 'config', 'params.yaml')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_cfg = PathJoinSubstitution([FindPackageShare('lap_timer'), 'rviz', 'lap_hud.rviz'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to the lap_timer parameter YAML file.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz with the lap HUD configuration.',
        ),
        Node(
            package='lap_timer',
            executable='lap_timer',
            name='lap_timer',
            parameters=[params_file],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            condition=IfCondition(use_rviz),
        ),
    ])
