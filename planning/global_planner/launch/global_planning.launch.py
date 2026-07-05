from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    default_param_file = "/home/haejun/2026_IFAC/planning/global_planner/config/global_planning.yaml"
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_param_file,
        description="Path to global planning parameter yaml",
    )

    params = LaunchConfiguration("params_file")
    pythonpath_inject = SetEnvironmentVariable(
        name="PYTHONPATH",
        value=[
            TextSubstitution(
                text="/home/haejum-park/race_stack/system_identification/steering_lookup/src:"
            ),
            TextSubstitution(
                text="/home/haejum-park/race_stack/f110_utils/libs/frenet_conversion/src:"
            ),
            EnvironmentVariable("PYTHONPATH", default_value=""),
        ],
    )

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
        pythonpath_inject,
        global_republisher,
        frenet_odom,
    ])
