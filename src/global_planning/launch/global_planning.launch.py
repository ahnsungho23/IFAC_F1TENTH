import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
    map_creator_params_arg = DeclareLaunchArgument(
        "map_creator_params_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("map_creator"),
            "config",
            "map_creator.yaml",
        ]),
        description="Path to map_creator_node parameter YAML",
    )
    # 맵 이름 단일화: F1_MAP 환경변수로 MCL/global/local 이 같은 맵을 보게 한다.
    # yaml 의 초기 map_name 을 이 인자로 덮어쓴다(미설정 시 'map').
    # map_creator의 생성·검증과 다음 랩 게이트가 끝나면 reload 서비스가 별도
    # obstacle_map 디렉터리로 런타임 참조만 전환한다.
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

    map_creator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("map_creator"),
            "launch",
            "map_creator.launch.py",
        ])),
        launch_arguments={
            "params_file": LaunchConfiguration("map_creator_params_file"),
        }.items(),
    )

    # lap_counter_node는 state_machine.launch.py에서 함께 기동한다
    # (단독 실행은 lap_counter.launch.py 사용).
    return LaunchDescription([
        params_arg,
        map_creator_params_arg,
        map_name_arg,
        global_republisher,
        frenet_odom,
        map_creator,
    ])
