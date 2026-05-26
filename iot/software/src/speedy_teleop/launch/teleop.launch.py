import os
import math
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    """
    Script de inicialização de teleoperação manual e ferramentas de calibração.
    """

    # Configs
    bringup_dir = get_package_share_directory('speedy_bringup')
    teleop_config_path = os.path.join(bringup_dir, 'config', 'teleop.yaml')
    supervisor_config_path = os.path.join(bringup_dir, 'config', 'supervisor.yaml')
    hardware_config_path = os.path.join(bringup_dir, 'config', 'hardware.yaml')

    # Extração de limites do hardware
    max_steer = 0.785
    max_linear = 1.0
    try:
        with open(hardware_config_path, 'r') as f:
            hw = yaml.safe_load(f)['speedy_hardware']['ros__parameters']
            steer_deg = max(float(hw['max_steering_angle_left_deg']),
                            float(hw['max_steering_angle_right_deg']))
            max_steer = math.radians(steer_deg)
            max_linear = float(hw['max_linear_velocity'])
            wheelbase = float(hw['wheelbase'])
    except Exception as e:
        print(f"[Aviso] Nao foi possivel ler hardware.yaml. Usando defaults. Erro: {e}")

    log_msg = LogInfo(
        msg=f"\n{'='*60}\n  [SPEEDY TELEOP] Initializing Manual Control System...\n"
            f"  - Steering Limit: {max_steer:.4f} rad\n"
            f"  - Velocity Limit: {max_linear} m/s\n"
    )

    # Joystick
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
        output='screen'
    )

    # Teleop
    teleop_node = Node(
        package='speedy_teleop',
        executable='racing_teleop',
        name='racing_teleop_node',
        parameters=[
            teleop_config_path,
            {
                'max_steer_angle': max_steer,
                'max_velocity': max_linear,
            }
        ],
        output='screen'
    )

    # Supervisor
    supervisor_node = Node(
        package='speedy_supervisor',
        executable='speedy_supervisor',
        name='speedy_supervisor',
        parameters=[supervisor_config_path],
        output='screen'
    )

    return LaunchDescription([
        log_msg,
        joy_node,
        teleop_node,
        supervisor_node
    ])
