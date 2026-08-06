import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


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

    state_machine_node = Node(
        package="state_machine",
        executable="state_machine_node",
        name="state_machine_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    # lap_counter_node(global_planning 패키지)를 상태 머신과 함께 기동한다.
    # 파라미터는 global_planning.yaml의 lap_counter_node 섹션을 그대로 사용.
    lap_counter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("global_planning"),
                "launch",
                "lap_counter.launch.py",
            )
        )
    )

    return LaunchDescription([
        params_file_arg,
        state_machine_node,
        lap_counter_launch,
    ])
