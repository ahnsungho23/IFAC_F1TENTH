from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("wpnt_publisher"),
                "config",
                "wpnt_publisher.yaml",
            ]),
            description="Path to the wpnt_publisher parameter YAML file.",
        ),
        Node(
            package="wpnt_publisher",
            executable="wpnt_publisher",
            name="wpnt_publisher",
            output="screen",
            parameters=[params_file],
        ),
    ])
