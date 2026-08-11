from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
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

    # scoped=True 필수: IncludeLaunchDescription의 launch_arguments는 스코프가 없어서
    # 여기서 넘긴 params_file 이 부모 스코프의 params_file 을 덮어쓴다. 그러면 뒤에 오는
    # map_creator_node 가 static_obstacle_map.yaml 을 받아 자기 파라미터를 전부 잃고
    # C++ 기본값으로 돈다(base_map_yaml="" -> gui_params 폴백 -> base map load failed).
    static_obstacle_map = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare("static_obstacle_map"),
                    "launch",
                    "static_obstacle_map.launch.py",
                ])),
                launch_arguments={
                    "params_file": LaunchConfiguration("static_obstacle_map_params_file"),
                }.items(),
            ),
        ],
        scoped=True,
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
