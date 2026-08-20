from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('kinematic_localization'),
        'config',
        'kinematic_localization.yaml',
    ])
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
