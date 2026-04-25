import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """
    Script Master de Inicialização do Speedy.
    Lança a infraestrutura central (Core) e o sistema de controlo manual (Teleop) num único comando.
    """
    
    # Obter os caminhos de instalação dos pacotes
    bringup_dir = get_package_share_directory('speedy_bringup')
    teleop_dir = get_package_share_directory('speedy_teleop')

    # Log global via Launch Action
    log_msg = LogInfo(
        msg=f"\n{'*'*60}\n  [SPEEDY MASTER] Iniciando todos os subsistemas...\n{'*'*60}\n"
    )

    # 1. Incluir o Launch do Core (Micro-ROS + Foxglove)
    core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'core.launch.py')
        )
    )

    # 2. Incluir o Launch do Teleop (Joystick)
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(teleop_dir, 'launch', 'teleop.launch.py')
        )
    )

    # Retornar a lista de todos os launches para o ROS 2 orquestrar
    return LaunchDescription([
        log_msg,
        core_launch,
        teleop_launch
    ])