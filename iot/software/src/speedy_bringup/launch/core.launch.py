import os
import math
import yaml
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    speedy_description_path = FindPackageShare('speedy_description')
    speedy_bringup_path = FindPackageShare('speedy_bringup')

    urdf_path = PathJoinSubstitution([speedy_description_path, 'urdf', 'speedy.urdf.xacro'])
    controller_config_path = PathJoinSubstitution(
        [speedy_bringup_path, 'config', 'controllers.yaml'])
    hardware_config_path = PathJoinSubstitution([speedy_bringup_path, 'config', 'hardware.yaml'])
    imu_config_path = PathJoinSubstitution([speedy_bringup_path, 'config', 'imu.yaml'])
    ekf_config_path = PathJoinSubstitution([speedy_bringup_path, 'config', 'ekf.yaml'])
    vision_config_path = PathJoinSubstitution([speedy_bringup_path, 'config', 'vision.yaml'])
    navigation_config_path = PathJoinSubstitution(
        [speedy_bringup_path, 'config', 'navigation.yaml'])
    camera_config_path = PathJoinSubstitution([speedy_bringup_path, 'config', 'camera.yaml'])
    lidar_config_path = PathJoinSubstitution([speedy_bringup_path, 'config', 'lidar.yaml'])
    foxglove_config_path = PathJoinSubstitution([speedy_bringup_path, 'config', 'foxglove.yaml'])

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        urdf_path, ' ',
        'hardware_config_file:=', hardware_config_path
    ])
    robot_description = {
        'robot_description': ParameterValue(
            robot_description_content,
            value_type=str)}

    controller_overrides_path = ''
    reactive_params = {}
    try:
        hw_yaml = os.path.join(get_package_share_directory('speedy_bringup'),
                               'config', 'hardware.yaml')
        with open(hw_yaml, 'r') as f:
            hw = yaml.safe_load(f)['speedy_hardware']['ros__parameters']

        steer_rad = math.radians(max(float(hw['max_steering_angle_left_deg']),
                                     float(hw['max_steering_angle_right_deg'])))
        max_lin = float(hw['max_linear_velocity'])
        wheelbase = float(hw['wheelbase'])
        wheel_radius = float(hw['wheel_radius'])

        overrides_dict = {
            'bicycle_steering_controller': {
                'ros__parameters': {
                    'wheelbase': wheelbase,
                    'traction_wheel_radius': wheel_radius,
                    'linear.x.max_velocity': max_lin,
                    'linear.x.min_velocity': -max_lin,
                    'angular.z.max_position': steer_rad,
                    'angular.z.min_position': -steer_rad,
                }
            }
        }
        tmp_param_file = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
        yaml.dump(overrides_dict, tmp_param_file)
        tmp_param_file.close()
        controller_overrides_path = tmp_param_file.name

        reactive_params = {
            'wheelbase': wheelbase,
            'steer_max': steer_rad,
        }
    except Exception as e:
        print(f"[Aviso] Falha ao injetar params: {e}")

    log_msg = LogInfo(
        msg=f"\n{'=' * 60}\n  [SPEEDY BRINGUP] Initializing Core Infrastructure...\n"
        f"  [ROS2 CONTROL] Mode: Direct RPi PWM\n"
        f"  [TELEMETRY] Foxglove Bridge: Ready\n{'=' * 60}\n"
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    cm_params = [robot_description, controller_config_path]
    if controller_overrides_path:
        cm_params.append(controller_overrides_path)

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=cm_params,
        output='both'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )
    bicycle_steering_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'bicycle_steering_controller',
            '--controller-manager',
            '/controller_manager',
            '--inactive'],
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

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[foxglove_config_path],
        output='screen'
    )

    camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='speedy_camera',
        parameters=[camera_config_path],
        output='screen'
    )

    ldlidar_node = Node(
        package='ldlidar_ros2',
        executable='ldlidar_ros2_node',
        name='ldlidar_node',
        parameters=[lidar_config_path],
        output='screen'
    )

    mpu6050_node = Node(
        package='mpu6050_driver',
        executable='mpu6050_node',
        name='mpu6050_sensor',
        parameters=[imu_config_path],
        output='screen'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config_path],
        output='screen'
    )

    line_detector_node = Node(
        package='speedy_vision',
        executable='line_detector',
        parameters=[vision_config_path],
        output='screen'
    )

    reactive_controller_node = Node(
        package='speedy_navigation',
        executable='reactive_controller',
        parameters=[navigation_config_path, reactive_params],
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
        mpu6050_node,
        ekf_node,
        line_detector_node,
        reactive_controller_node
    ])
