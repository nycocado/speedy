import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('speedy_vision_pc')
    config_path = os.path.join(pkg_share, 'config', 'vision_pc.yaml')

    return LaunchDescription([
        Node(
            package='speedy_vision_pc',
            executable='obstacle_detector',
            name='obstacle_detector',
            output='screen',
            parameters=[config_path]
        )
    ])
