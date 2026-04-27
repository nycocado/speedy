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
    """
    
    joy_dev = find_first_joystick()

    # Log informativo
    log_msg = LogInfo(
        msg=f"\n{'='*60}\n  [SPEEDY TELEOP] Inicializando Sistema de Controle Manual...\n"
    )

    # Nó do Joystick
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_name': 'MACHENIKE G5Pro',
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
            'coalesce_interval_ms': 1,
        }],
        respawn=True,
        respawn_delay=1.0,
        output='screen'
    )

    return LaunchDescription([
        log_msg,
        joy_node
    ])