from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('amcl_localization')
    config_dir = os.path.join(pkg_share, 'config')

    return LaunchDescription([
        

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[os.path.join(config_dir, 'amcl.yaml')]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[os.path.join(config_dir, 'lifecycle_manager.yaml')]
        ),
    ])

