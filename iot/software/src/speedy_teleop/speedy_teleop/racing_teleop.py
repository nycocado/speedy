#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray

class RacingTeleopNode(Node):
    """
    Lê o Joystick e "grita" os comandos para todas as interfaces disponíveis.
    Não sabe e não se importa quem está ouvindo (Desacoplamento).
    """
    def __init__(self):
        super().__init__('racing_teleop')

        self.declare_parameter('axis_steering', 0)
        self.declare_parameter('axis_throttle', 4)
        self.declare_parameter('axis_brake', 5)
        self.declare_parameter('max_steer_angle', 0.785)
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('deadzone', 0.05)
        self.declare_parameter('poc_angular_velocity_multiplier', 1.5)

        self.axis_steering = self.get_parameter('axis_steering').value
        self.axis_throttle = self.get_parameter('axis_throttle').value
        self.axis_brake = self.get_parameter('axis_brake').value
        self.max_steer_angle = self.get_parameter('max_steer_angle').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.deadzone = self.get_parameter('deadzone').value
        self.poc_angular_velocity_multiplier = self.get_parameter('poc_angular_velocity_multiplier').value

        self.lt_touched = False
        self.rt_touched = False

        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Publica direto para o Hardware (Modo MANUAL)
        self.pub_manual_steer = self.create_publisher(Float64MultiArray, '/manual_steering_controller/commands', 10)
        self.pub_manual_drive = self.create_publisher(Float64MultiArray, '/manual_drive_controller/commands', 10)
        
        # Publica como Velocidade (Modo AUTO/Cinemático PoC)
        self.pub_auto_twist = self.create_publisher(TwistStamped, '/bicycle_steering_controller/reference', 10)

        self.get_logger().info('[TELEOP] Node initialized: Reading Joystick and publishing commands.')

    def normalize_trigger(self, raw_value, is_lt):
        touched = self.lt_touched if is_lt else self.rt_touched
        if not touched and raw_value == 0.0:
            return 0.0
        if not touched and raw_value != 0.0:
            if is_lt: self.lt_touched = True
            else: self.rt_touched = True
        val = (1.0 - raw_value) / 2.0
        return max(0.0, min(1.0, val))

    def joy_callback(self, msg: Joy):
        raw_steer = msg.axes[self.axis_steering] if len(msg.axes) > self.axis_steering else 0.0
        raw_lt = msg.axes[self.axis_brake] if len(msg.axes) > self.axis_brake else 1.0
        raw_rt = msg.axes[self.axis_throttle] if len(msg.axes) > self.axis_throttle else 1.0

        steer_input = 0.0 if abs(raw_steer) < self.deadzone else raw_steer
        lt = self.normalize_trigger(raw_lt, True)
        rt = self.normalize_trigger(raw_rt, False)
        
        throttle = rt - lt
        if abs(throttle) < self.deadzone:
            throttle = 0.0

        desired_angle = steer_input * self.max_steer_angle
        linear_vel = throttle * self.max_velocity

        # 1. Envia comando MANUAL direto pro hardware (Ignora a física da bicicleta)
        msg_steer = Float64MultiArray()
        msg_steer.data = [desired_angle]
        self.pub_manual_steer.publish(msg_steer)
        
        msg_drive = Float64MultiArray()
        msg_drive.data = [linear_vel]
        self.pub_manual_drive.publish(msg_drive)

        # 2. Envia comando AUTO (PoC: Você pilota "por velocidade" em vez de "por posição")
        # Se o modo AUTO estiver ligado, ele fará o "pulo do parado", provando a cinemática.
        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = 'base_link'
        ts.twist.linear.x = linear_vel
        ts.twist.angular.z = steer_input * self.poc_angular_velocity_multiplier
        self.pub_auto_twist.publish(ts)

def main(args=None):
    rclpy.init(args=args)
    node = RacingTeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()