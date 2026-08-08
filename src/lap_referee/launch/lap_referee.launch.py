from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory("lap_referee")
    default_params = os.path.join(package_share, "config", "lap_referee.yaml")

    params_file = LaunchConfiguration("params_file")
    waypoints_csv = LaunchConfiguration("waypoints_csv")
    output_dir = LaunchConfiguration("output_dir")
    output_prefix = LaunchConfiguration("output_prefix")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to the lap_referee parameter YAML file.",
            ),
            DeclareLaunchArgument(
                "waypoints_csv",
                default_value="",
                description="Reference raceline CSV (overrides params_file value when set).",
            ),
            DeclareLaunchArgument(
                "output_dir",
                default_value="/tmp/lap_referee",
                description="Directory for <prefix>_summary.json and <prefix>_trace.csv.",
            ),
            DeclareLaunchArgument(
                "output_prefix",
                default_value="rollout",
                description="Output file prefix for this rollout.",
            ),
            Node(
                package="lap_referee",
                executable="lap_referee",
                name="lap_referee",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "waypoints_csv": waypoints_csv,
                        "output_dir": output_dir,
                        "output_prefix": output_prefix,
                    },
                ],
            ),
        ]
    )
