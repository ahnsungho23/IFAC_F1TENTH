from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
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
    lap_counter_params_arg = DeclareLaunchArgument(
        "lap_counter_params_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("global_planning"),
            "config",
            "global_planning.yaml",
        ]),
        description="Path to lap_counter_node parameter YAML",
    )

    state_machine_node = Node(
        package="state_machine",
        executable="state_machine_node",
        name="state_machine_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    # lap_counter_node는 /lap_count를 내는 유일한 발행자이고, map_creator가 그 값으로
    # 랩2 프리즈·스왑을 트리거한다(map_creator.yaml trigger_lap_count). 여기서 같이 띄운다.
    #
    # scoped=True 필수: IncludeLaunchDescription은 부모 스코프를 그대로 물려주고,
    # 자식의 DeclareLaunchArgument는 이미 설정된 값을 덮지 않는다. 두 launch가 똑같이
    # "params_file"을 쓰므로 스코프를 안 끊으면 lap_counter_node가 state_machine.yaml을
    # 받아 자기 블록을 못 찾고 C++ 기본값으로 돈다(map_creator.launch.py에 같은 사례 기록).
    lap_counter = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare("global_planning"),
                    "launch",
                    "lap_counter.launch.py",
                ])),
                launch_arguments={
                    "params_file": LaunchConfiguration("lap_counter_params_file"),
                }.items(),
            ),
        ],
        scoped=True,
    )

    return LaunchDescription([
        params_file_arg,
        lap_counter_params_arg,
        state_machine_node,
        lap_counter,
    ])
