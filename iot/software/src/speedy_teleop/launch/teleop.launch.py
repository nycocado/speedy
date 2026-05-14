import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    """
    Script de inicialização de teleoperação manual.
    """

    # Obtendo o diretorio de instalacao para carregar o YAML de forma segura
    teleop_dir = get_package_share_directory('speedy_teleop')
    teleop_config_path = os.path.join(teleop_dir, 'config', 'teleop.yaml')

    # Lendo limite de esterçamento dinamicamente do controllers.yaml
    bringup_dir = get_package_share_directory('speedy_bringup')
    config_path = os.path.join(bringup_dir, 'config', 'controllers.yaml')
    
    max_steer = 0.785 # default fallback
    max_linear = 1.0 # default fallback
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
            params = config['bicycle_steering_controller']['ros__parameters']
            max_steer = float(params.get('angular.z.max_position', 0.785))
            max_linear = float(params.get('linear.x.max_velocity', 1.0))
    except Exception as e:
        print(f"[Aviso] Nao foi possivel ler os limites do controllers.yaml. Usando defaults. Erro: {e}")

    # Log informativo
    log_msg = LogInfo(
        msg=f"\n{'='*60}\n  [SPEEDY TELEOP] Initializing Manual Control System...\n"
            f"  - Automatic Steering Limit: {max_steer} rad\n"
            f"  - Velocity Limit: {max_linear} m/s\n"
    )

    # Nó do Joystick
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
            'coalesce_interval_ms': 1,
            'autoreconnect_period': 1.0,
        }],
        respawn=True,
        respawn_delay=1.0,
        output='screen'
    )

    # Teleop: Piloto Humano (Gera comandos para ambos os modos)
    teleop_node = Node(
        package='speedy_teleop',
        executable='racing_teleop',
        name='racing_teleop_node',
        parameters=[
            teleop_config_path, 
            {
                'max_steer_angle': max_steer,
                'max_velocity': max_linear
            }
        ],
        output='screen'
    )

    # Supervisor: Máquina de Estados (Muda o hardware)
    supervisor_node = Node(
        package='speedy_supervisor',
        executable='speedy_supervisor',
        name='speedy_supervisor',
        parameters=[
            teleop_config_path
        ],
        output='screen'
    )

    return LaunchDescription([
        log_msg,
        joy_node,
        teleop_node,
        supervisor_node
    ])
