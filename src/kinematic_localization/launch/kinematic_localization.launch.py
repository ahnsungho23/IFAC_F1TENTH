import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # launch_ros는 존재하지 않는 parameter file을 조용히 건너뛸 수 있다. 그러면 노드는
    # gate/source downsample/smoothing이 코드 기본값인 채 정상 기동하므로, launch 단계에서
    # 깨진 symlink까지 포함해 즉시 차단한다. 파일 내용의 세대 일치는 노드의
    # config_schema_version 가드가 한 번 더 확인한다.
    config = os.path.join(
        get_package_share_directory('kinematic_localization'),
        'config',
        'kinematic_localization.yaml',
    )
    if not os.path.isfile(config):
        raise RuntimeError(
            f'필수 파라미터 파일이 없거나 symlink가 깨졌습니다: {config}\n'
            'src/kinematic_localization/config와 install overlay를 복구·재빌드한 뒤 '
            '다시 실행하세요.')
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
