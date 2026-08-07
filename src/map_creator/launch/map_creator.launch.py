from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Relative paths in the YAML (driver, gui_params, output dir) resolve from the
    # `ros2 launch` working directory — launch from the workspace root (~/2026_IFAC),
    # same convention as global_planning.
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("map_creator"),
            "config",
            "map_creator.yaml",
        ]),
        description="Path to map_creator_node parameter YAML",
    )
    static_obstacle_map_params_arg = DeclareLaunchArgument(
        "static_obstacle_map_params_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("static_obstacle_map"),
            "config",
            "static_obstacle_map.yaml",
        ]),
        description="Path to static_obstacle_map parameter YAML",
    )

    static_obstacle_map = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("static_obstacle_map"),
            "launch",
            "static_obstacle_map.launch.py",
        ])),
        launch_arguments={
            "params_file": LaunchConfiguration("static_obstacle_map_params_file"),
        }.items(),
    )

    map_creator_node = Node(
        package="map_creator",
        executable="map_creator_node",
        name="map_creator_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([
        params_file_arg,
        static_obstacle_map_params_arg,
        static_obstacle_map,
        map_creator_node,
    ])
