import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('local_planning')
    default_config = os.path.join(pkg_dir, 'config', 'local_planning.yaml')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_config,
        description='Full path to parameter yaml file for local planning node'
    )

    node = Node(
        package='local_planning',
        executable='static_obstacle_avoidance_node',
        name='static_obstacle_avoidance_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    return LaunchDescription([
        params_arg,
        node
    ])
