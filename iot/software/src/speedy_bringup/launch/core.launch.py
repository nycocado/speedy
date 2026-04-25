import os
import glob
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo

def find_first_serial():
    """
    Busca o dispositivo serial (ESP32-S3). 
    Ordena a lista para priorizar índices menores (ex: ACM0 antes de ACM1).
    """
    devices = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
    if devices:
        devices.sort() # Garante que /dev/ttyACM0 venha antes de /dev/ttyACM1
        return devices[0]
    return '/dev/ttyACM0'

def generate_launch_description():
    """
    Script de inicialização da infraestrutura central do Speedy.
    Orquestra o Agente Micro-ROS e o Servidor do Foxglove Studio.
    """
    
    esp_dev = find_first_serial()

    # Log informativo via Launch Action
    log_msg = LogInfo(
        msg=f"\n{'='*60}\n  [SPEEDY BRINGUP] Inicializando Infraestrutura Core...\n  [HARDWARE] ESP32-S3 detectado em: {esp_dev}\n  [TELEMETRIA] Foxglove Bridge: Pronto (Porta 8765)\n{'='*60}\n"
    )

    # Nó do Agente Micro-ROS (Ponte Serial)
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', esp_dev, '-b', '921600'],
        output='screen'
    )

    # Nó do Foxglove Bridge (Servidor para Visualização)
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'tls': False
        }],
        output='screen'
    )

    return LaunchDescription([
        log_msg,
        micro_ros_agent,
        foxglove_bridge
    ])