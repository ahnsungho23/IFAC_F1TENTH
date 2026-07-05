import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_param_file = os.path.join(
        get_package_share_directory("global_planner"),
        "config",
        "global_planning.yaml",
    )
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_param_file,
        description="Path to global planning parameter yaml",
    )

    params = LaunchConfiguration("params_file")

    global_republisher = Node(
        package="global_planner",
        executable="global_trajectory_publisher_node",
        name="global_trajectory_publisher_node",
        output="screen",
        parameters=[params],
    )

    frenet_odom = Node(
        package="global_planner",
        executable="frenet_odom_node",
        name="frenet_odom_node",
        output="screen",
        parameters=[params],
    )

    return LaunchDescription([
        params_arg,
        global_republisher,
        frenet_odom,
    ])
