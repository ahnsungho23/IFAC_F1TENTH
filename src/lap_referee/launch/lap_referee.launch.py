import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('lap_referee')
    default_params = os.path.join(package_share, 'config', 'lap_referee.yaml')

    params_file = LaunchConfiguration('params_file')
    waypoints_csv = LaunchConfiguration('waypoints_csv')
    output_dir = LaunchConfiguration('output_dir')
    output_prefix = LaunchConfiguration('output_prefix')
    odom_topic = LaunchConfiguration('odom_topic')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'params_file',
                default_value=default_params,
                description='Path to the lap_referee parameter YAML file.',
            ),
            DeclareLaunchArgument(
                'waypoints_csv',
                default_value='',
                description='Reference raceline CSV (overrides params_file value when set).',
            ),
            DeclareLaunchArgument(
                'output_dir',
                default_value='/tmp/lap_referee',
                description='Directory for <prefix>_summary.json and <prefix>_trace.csv.',
            ),
            DeclareLaunchArgument(
                'output_prefix',
                default_value='rollout',
                description='Output file prefix for this rollout.',
            ),
            # ⚠️ 2026-08-13 실차 사고: 이 인자가 선언되지 않아 runner의
            # odom_topic:=/pf/pose/odom 이 조용히 버려졌고(launch는 미선언 인자를
            # 에러 없이 무시한다), referee가 시뮬 전용 /ego_racecar/odom 을 구독해
            # 실차에서 odom 0건으로 영원히 대기했다. 토픽 인자는 반드시 여기 선언
            # + 아래 Node parameters 전달까지 한 쌍으로 넣을 것.
            DeclareLaunchArgument(
                'odom_topic',
                default_value='/ego_racecar/odom',
                description='Odometry topic (sim: /ego_racecar/odom, real: /pf/pose/odom).',
            ),
            Node(
                package='lap_referee',
                executable='lap_referee',
                name='lap_referee',
                output='screen',
                parameters=[
                    params_file,
                    {
                        'waypoints_csv': waypoints_csv,
                        'output_dir': output_dir,
                        'output_prefix': output_prefix,
                        'odom_topic': odom_topic,
                    },
                ],
            ),
        ]
    )
