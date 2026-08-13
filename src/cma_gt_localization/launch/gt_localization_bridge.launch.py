import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("cma_gt_localization"),
        "config",
        "gt_localization_bridge.yaml",
    )
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="GT localization bridge parameter YAML",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="f1tenth_gym_ros does not publish /clock, so keep false",
    )
    node = Node(
        package="cma_gt_localization",
        executable="gt_localization_bridge_node",
        name="gt_localization_bridge",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )
    return LaunchDescription([params_arg, use_sim_time_arg, node])

