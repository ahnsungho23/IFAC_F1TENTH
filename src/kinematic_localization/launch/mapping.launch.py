from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('kinematic_localization'),
        'config',
        'mapping.yaml',
    ])
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
