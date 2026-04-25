import os
import glob
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo

def find_first_joystick():
    """Busca o primeiro joystick disponível (/dev/input/js*)."""
    devices = glob.glob('/dev/input/js*')
    if devices:
        devices.sort()
        return devices[0]
    return '/dev/input/js0'

def generate_launch_description():
    """
    Script de inicialização de teleoperação manual.
    Orquestra apenas o Driver do Joystick e eventuais nós de conversão de comandos.
    """
    
    joy_dev = find_first_joystick()

    # Log informativo via Launch Action
    log_msg = LogInfo(
        msg=f"\n{'='*60}\n  [SPEEDY TELEOP] Inicializando Sistema de Controle Manual...\n  [HARDWARE] Controle detectado em: {joy_dev}\n{'='*60}\n"
    )

    # Nó do Joystick usando joy (Pacote antigo/SDL2 padrão)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.05,
            'coalesce_interval': 0.01
        }],
        respawn=True,
        respawn_delay=1.0, 
        output='screen'
    )

    return LaunchDescription([
        log_msg,
        joy_node
    ])