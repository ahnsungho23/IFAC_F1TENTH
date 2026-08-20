import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 파일이 없으면 launch_ros가 조용히 건너뛰므로 여기서 즉시 실패시킨다
    # (kinematic_localization.launch.py의 같은 주석 참고).
    config = os.path.join(
        get_package_share_directory('kinematic_localization'), 'config', 'mapping.yaml')
    if not os.path.isfile(config):
        raise RuntimeError(f"파라미터 파일이 없습니다: {config}")
    bag_path = LaunchConfiguration('bag_path')
    output_path = LaunchConfiguration('output_path')
    lidar_topic = LaunchConfiguration('lidar_topic')

    return LaunchDescription([
        DeclareLaunchArgument('bag_path', description='Input rosbag2 directory'),
        DeclareLaunchArgument(
            'output_path', default_value='map.kissmap',
            description='Output .kissmap file path'),
        DeclareLaunchArgument('lidar_topic', default_value='/scan'),
        Node(
            package='kinematic_localization',
            executable='mapping_node',
            name='kinematic_mapping',
            output='screen',
            parameters=[config, {
                'bag_path': bag_path,
                'output_path': output_path,
                'lidar_topic': lidar_topic,
            }],
        ),
    ])
