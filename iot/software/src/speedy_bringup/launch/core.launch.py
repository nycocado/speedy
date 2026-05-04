import os
import glob
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    """
    Script de inicialização da infraestrutura central do Speedy.
    Orquestra o Agente Micro-ROS, o Foxglove Bridge e a Câmera (libcamera).
    """
    
    esp_dev = '/dev/ttyESP32'

    # Log informativo via Launch Action
    log_msg = LogInfo(
        msg=f"\n{'='*60}\n  [SPEEDY BRINGUP] Inicializando Infraestrutura Core...\n"
            f"  [HARDWARE] ESP32-S3 (udev) em: {esp_dev}\n"
            f"  [CÂMERA] Stream UDP Nativo: 640x480\n"
            f"  [TELEMETRIA] Foxglove Bridge: Pronto (Porta 8765)\n{'='*60}\n"
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

    # Nó da Câmera (Acesso Nativo Zero-Copy via libcamera)
    camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='speedy_camera',
        parameters=[{
            'width': 640,
            'height': 480,
            'format': 'YUYV',
            'frame_id': 'camera_link'
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

    return LaunchDescription([
        log_msg,
        micro_ros_agent,
        foxglove_bridge,
        camera_node,
        ldlidar_node
    ])
