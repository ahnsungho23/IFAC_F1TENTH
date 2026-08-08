import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_param_file = os.path.join(
        get_package_share_directory("global_planning"),
        "config",
        "global_planning.yaml",
    )

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_param_file,
        description="Path to the global_planning parameter YAML",
    )

    lap_counter = Node(
        package="global_planning",
        executable="lap_counter_node",
        name="lap_counter_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([params_arg, lap_counter])
