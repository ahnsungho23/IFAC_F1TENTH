#!/usr/bin/env python3
"""Launch the opponent_drive_controller node.

Publishes drive commands to f1sim's opponent vehicle (/opp_drive) to follow global waypoints
at a scaled speed (default 0.8x). Works with num_agent: 2 in f1sim.

Usage:
  ros2 launch new_map_con opponent_simulator.launch.py
  ros2 launch new_map_con opponent_simulator.launch.py speed_scale:=0.7
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    speed_scale = LaunchConfiguration("speed_scale")
    enabled = LaunchConfiguration("enabled")

    return LaunchDescription([
        DeclareLaunchArgument(
            "speed_scale",
            default_value="0.3",
            description="Speed multiplier for opponent (0.8 = 80% of waypoint speed)",
        ),
        DeclareLaunchArgument(
            "enabled",
            default_value="true",
            description="Enable/disable opponent drive controller",
        ),
        Node(
            package="new_map_con",
            executable="opponent_drive_controller",
            name="opponent_drive_controller",
            output="screen",
            parameters=[{
                "waypoints_topic": "/global_waypoints",
                "opp_odom_topic": "/opp_racecar/odom",
                "opp_drive_topic": "/opp_drive",
                "speed_scale": speed_scale,
                "lookahead_distance": 2.0,
                "wheelbase": 0.33,
                "control_rate_hz": 40.0,
                "enabled": enabled,
            }],
        ),
    ])
