from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("state_machine"),
            "config",
            "state_machine.yaml",
        ]),
        description="Path to state_machine_node parameter YAML",
    )
    runtime_profile_arg = DeclareLaunchArgument(
        "runtime_profile",
        default_value=PathJoinSubstitution([
            FindPackageShare("f1tenth_control"), "config", "runtime_visualization.yaml"
        ]),
        description="Shared visualization/output profile YAML",
    )

    state_machine_node = Node(
        package="state_machine",
        executable="state_machine_node",
        name="state_machine_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            LaunchConfiguration("runtime_profile"),
        ],
    )

    return LaunchDescription([
        params_file_arg,
        runtime_profile_arg,
        state_machine_node,
    ])
