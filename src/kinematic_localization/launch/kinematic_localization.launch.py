import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ⚠️ 절대경로 + 존재 검사. PathJoinSubstitution으로 넘기면 파일이 없어도 launch_ros가
    # **조용히 건너뛰고** 노드는 코드 기본값으로 정상 기동한다 — 2026-08-19에 젯슨에서
    # 이 config가 지워진 채 한 세션을 통째로 날렸다(gate off, ICP 대응점 54->15,
    # smoothing_alpha_rot 0.12->0.5). 여기서 즉시 실패시켜 그 조용한 실패를 없앤다.
    config = os.path.join(
        get_package_share_directory('kinematic_localization'),
        'config',
        'kinematic_localization.yaml',
    )
    if not os.path.isfile(config):
        raise RuntimeError(
            f"파라미터 파일이 없습니다: {config}\n"
            "  이 파일 없이 기동하면 gate_enable/smoothing_alpha_rot/source_voxel_size 등이\n"
            "  전부 코드 기본값이 되어 위치추정 품질이 조용히 무너집니다.\n"
            "  복구: git checkout HEAD -- src/kinematic_localization/config/"
            "kinematic_localization.yaml\n"
            "  (symlink install이면 재빌드 없이 즉시 반영, 노드 재시작만 하면 됩니다)")
    map_name = LaunchConfiguration('map_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_mode = LaunchConfiguration('slam_mode')
    map_output_file = LaunchConfiguration('map_output_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_name',
            default_value='',
            description='Frozen map name: maps/<map_name>.kissmap in the package share '
                        '(an absolute path is used as-is, empty = pure odometry). '
                        'Ignored when slam_mode is true.'),
        DeclareLaunchArgument(
            'slam_mode',
            default_value='false',
            description='Online SLAM mode: no frozen map, auto-init at identity on the '
                        'first scan, save the accumulated map via the ~/save_map service '
                        'or on shutdown.'),
        DeclareLaunchArgument(
            'map_output_file',
            default_value='slam_map.kissmap',
            description='Output .kissmap path for slam_mode (relative paths resolve '
                        'against the process cwd).'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='kinematic_localization',
            executable='localization_node',
            name='kinematic_localization',
            output='screen',
            parameters=[config, {
                'map_name': map_name,
                'slam_mode': slam_mode,
                'map_output_file': map_output_file,
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
