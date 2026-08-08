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
        description="Path to global planning parameter yaml",
    )
    # 맵 이름 단일화: F1_MAP 환경변수로 MCL/global/local 이 같은 맵을 보게 한다.
    # yaml 의 map_name 을 이 인자로 덮어쓴다(미설정 시 'map').
    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value=os.environ.get("F1_MAP", "map"),
        description="Map name (overrides map_name in yaml). Shared via F1_MAP env var.",
    )

    params = LaunchConfiguration("params_file")

    global_republisher = Node(
        package="global_planning",
        executable="global_trajectory_publisher_node",
        name="global_trajectory_publisher_node",
        output="screen",
        parameters=[params, {"map_name": LaunchConfiguration("map_name")}],
    )

    frenet_odom = Node(
        package="global_planning",
        executable="frenet_odom_node",
        name="frenet_odom_node",
        output="screen",
        parameters=[params],
    )

    return LaunchDescription([
        params_arg,
        map_name_arg,
        global_republisher,
        frenet_odom,
    ])
