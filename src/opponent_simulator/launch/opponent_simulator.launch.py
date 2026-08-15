#!/usr/bin/env python3
"""Launch the centerline-following simulator opponent controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    waypoints_topic = LaunchConfiguration("waypoints_topic")
    speed_scale = LaunchConfiguration("speed_scale")
    enabled = LaunchConfiguration("enabled")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("opponent_simulator"),
                "config",
                "opponent_simulator.yaml",
            ]),
            description="Opponent controller parameter YAML",
        ),
        DeclareLaunchArgument(
            "waypoints_topic",
            default_value="/centerline_waypoints",
            description="Waypoint topic followed by the simulated opponent",
        ),
        DeclareLaunchArgument(
            "speed_scale",
            default_value="0.8",
            description="Multiplier applied to waypoint speed",
        ),
        DeclareLaunchArgument(
            "enabled",
            default_value="true",
            description="Enable opponent drive publication",
        ),
        Node(
            package="opponent_simulator",
            executable="opponent_drive_controller",
            name="opponent_drive_controller",
            output="screen",
            parameters=[
                params_file,
                {
                    "waypoints_topic": waypoints_topic,
                    "speed_scale": ParameterValue(speed_scale, value_type=float),
                    "enabled": ParameterValue(enabled, value_type=bool),
                },
            ],
        ),
    ])
