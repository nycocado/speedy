import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_dir = get_package_share_directory('speedy_bringup')
    teleop_dir = get_package_share_directory('speedy_teleop')

    log_msg = LogInfo(
        msg=f"\n{'*' * 60}\n  [SPEEDY MASTER] Iniciando todos os subsistemas...\n{'*' * 60}\n"
    )

    core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'core.launch.py')
        )
    )

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(teleop_dir, 'launch', 'teleop.launch.py')
        )
    )

    return LaunchDescription([
        log_msg,
        core_launch,
        teleop_launch
    ])
