import os
import glob
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import LogInfo, ExecuteProcess, RegisterEventHandler
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    """
    Script de inicialização da infraestrutura central do Speedy.
    Orquestra ROS2 Control (RPi PWM), Controladores, Foxglove e Sensores.
    """
    
    # Paths
    speedy_description_path = FindPackageShare('speedy_description')
    speedy_bringup_path = FindPackageShare('speedy_bringup')
    
    urdf_path = PathJoinSubstitution([speedy_description_path, 'urdf', 'speedy.urdf.xacro'])
    controller_config_path = PathJoinSubstitution([speedy_bringup_path, 'config', 'controllers.yaml'])
    hardware_config_path = PathJoinSubstitution([speedy_bringup_path, 'config', 'hardware.yaml'])
    dataset_config_path = PathJoinSubstitution([speedy_bringup_path, 'config', 'dataset.yaml'])
    teleop_config_path = PathJoinSubstitution([speedy_bringup_path, 'config', 'teleop.yaml'])
    ekf_config_path = PathJoinSubstitution([speedy_bringup_path, 'config', 'ekf.yaml'])

    # Gera URDF passando pelo Xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', 
        urdf_path, ' ',
        'hardware_config_file:=', hardware_config_path
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Log informativo via Launch Action
    log_msg = LogInfo(
        msg=f"\n{'='*60}\n  [SPEEDY BRINGUP] Initializing Core Infrastructure...\n"
            f"  [ROS2 CONTROL] Mode: Direct RPi PWM (SpeedyHardwareInterface)\n"
            f"  [CONTROL] Bicycle Steering Controller: Enabled\n"
            f"  [TELEMETRY] Foxglove Bridge: Ready (Port 8765)\n{'='*60}\n"
    )

    # 1. Robot State Publisher (publica o URDF)
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    # 2. Controller Manager (O motor do ros2_control)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config_path, hardware_config_path],
        output='both'
    )

    # 3. Spawner dos Controladores
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    bicycle_steering_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['bicycle_steering_controller', '--controller-manager', '/controller_manager', '--inactive'],
    )

    manual_steering_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['manual_steering_controller', '--controller-manager', '/controller_manager'],
    )

    manual_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['manual_drive_controller', '--controller-manager', '/controller_manager'],
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

    # Nó da Câmera (Acesso Nativo Zero-Copy via libcamera)
    camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='speedy_camera',
        parameters=[{
            'width': 640,
            'height': 480,
            'format': 'YUYV',
            'vflip': True,
            'frame_id': 'camera_link',
            'camera_info_url': 'package://speedy_bringup/config/camera_info.yaml'
        }],
        output='screen'
    )

    # Nó do LiDAR D500
    ldlidar_node = Node(
        package='ldlidar_ros2',
        executable='ldlidar_ros2_node',
        name='ldlidar_node',
        output='screen',
        parameters=[{
            'product_name': 'LDLiDAR_LD19',
            'laser_scan_topic_name': 'scan',
            'point_cloud_2d_topic_name': 'pointcloud2d',
            'frame_id': 'laser_link',
            'port_name': '/dev/ldlidar',
            'serial_baudrate': 230400,
            'laser_scan_dir': True,
            'enable_angle_crop_func': False,
            'range_min': 0.02,
            'range_max': 25.0
        }]
    )

    # Coletor de Dataset (Botão X)
    dataset_collector_node = Node(
        package='speedy_dataset',
        executable='collector',
        name='dataset_collector',
        parameters=[dataset_config_path],
        output='screen'
    )

    # Racing Teleop (Controle Manual)
    racing_teleop_node = Node(
        package='speedy_teleop',
        executable='racing_teleop',
        name='racing_teleop_node',
        parameters=[teleop_config_path],
        output='screen'
    )

    # Speedy Supervisor
    speedy_supervisor_node = Node(
        package='speedy_supervisor',
        executable='speedy_supervisor',
        name='speedy_supervisor',
        parameters=[teleop_config_path],
        output='screen'
    )

    return LaunchDescription([
        log_msg,
        robot_state_pub_node,
        controller_manager,
        joint_state_broadcaster_spawner,
        bicycle_steering_controller_spawner,
        manual_steering_controller_spawner,
        manual_drive_controller_spawner,
        foxglove_bridge,
        camera_node,
        ldlidar_node,
        dataset_collector_node,
        racing_teleop_node,
        speedy_supervisor_node
    ])
