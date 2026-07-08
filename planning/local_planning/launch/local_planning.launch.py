import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('local_planning')
    default_config = os.path.join(pkg_dir, 'config', 'local_planning.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_config,
        description='Full path to the local_planning YAML parameter file'
    )

    local_planner_node = Node(
        package='local_planning',
        executable='local_planner_node',
        name='local_planner_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    return LaunchDescription([
        params_file_arg,
        local_planner_node
    ])
