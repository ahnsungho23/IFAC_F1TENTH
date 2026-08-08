from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os
from pathlib import Path


def find_source_package(package_share):
    share_path = Path(package_share).resolve()
    for parent in share_path.parents:
        candidate = parent / "src" / "new_map_con"
        if (candidate / "package.xml").is_file():
            return candidate
    return None


def generate_launch_description():
    package_share = get_package_share_directory("new_map_con")
    source_package = find_source_package(package_share)
    if source_package is not None:
        default_params = os.path.join(str(source_package), "config", "config.yaml")
        package_resource_root = str(source_package)
    else:
        default_params = os.path.join(package_share, "config", "config.yaml")
        package_resource_root = package_share

    params_file = LaunchConfiguration("params_file")
    simulator = LaunchConfiguration("simulator")
    runtime_profile = os.path.join(
        get_package_share_directory("f1tenth_control"),
        "config",
        "runtime_visualization.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to the new_map_con parameter YAML file.",
            ),
            DeclareLaunchArgument(
                "simulator",
                default_value="false",
                description="Use simulator topic profile when true.",
            ),
            DeclareLaunchArgument(
                "runtime_profile",
                default_value=runtime_profile,
                description="Shared visualization/output profile YAML.",
            ),
            Node(
                package="new_map_con",
                executable="map_controller",
                name="map_controller",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "simulator": ParameterValue(simulator, value_type=bool),
                        "package_resource_root": package_resource_root,
                    },
                    LaunchConfiguration("runtime_profile"),
                ],
            ),
        ]
    )
