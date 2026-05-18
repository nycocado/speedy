#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray, Float64

class RacingTeleopNode(Node):
    """
    Reads Joystick and publishes commands to all available interfaces.
    Uses a fixed-rate timer (50Hz) to stabilize the signal and eliminate jitter.
    """
    def __init__(self):
        super().__init__('racing_teleop')

        self.declare_parameter('axis_steering', 0)
        self.declare_parameter('axis_throttle', 4)
        self.declare_parameter('axis_brake', 5)
        self.declare_parameter('max_steer_angle', 0.785)
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('deadzone', 0.05)
        self.declare_parameter('publish_rate', 50.0)

        self.axis_steering = self.get_parameter('axis_steering').value
        self.axis_throttle = self.get_parameter('axis_throttle').value
        self.axis_brake = self.get_parameter('axis_brake').value
        self.max_steer_angle = self.get_parameter('max_steer_angle').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.deadzone = self.get_parameter('deadzone').value
        self.publish_rate = self.get_parameter('publish_rate').value

        self.lt_touched = False
        self.rt_touched = False

        # D-Pad Axes
        self.dpad_h_axis = 6
        self.dpad_v_axis = 7
        
        # Last Joystick State
        self.latest_steer_input = 0.0
        self.latest_linear_vel = 0.0
        
        # Calibration States
        self.cruise_speed_mps = 0.0
        self.last_dpad_v = 0.0

        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Publishers
        self.pub_manual_steer = self.create_publisher(Float64MultiArray, '/manual_steering_controller/commands', 10)
        self.pub_manual_drive = self.create_publisher(Float64MultiArray, '/manual_drive_controller/commands', 10)
        self.pub_auto_twist = self.create_publisher(TwistStamped, '/bicycle_steering_controller/reference', 10)
        self.pub_current_speed = self.create_publisher(Float64, '/teleop/current_speed_mps', 10)

        # Timer para publicação estabilizada
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(f'[TELEOP] Node initialized. Rate: {self.publish_rate}Hz')

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
        
        dpad_h = msg.axes[self.dpad_h_axis] if len(msg.axes) > self.dpad_h_axis else 0.0
        dpad_v = msg.axes[self.dpad_v_axis] if len(msg.axes) > self.dpad_v_axis else 0.0

        # Cruise Control Step
        if dpad_v == 1.0 and self.last_dpad_v != 1.0:
            self.cruise_speed_mps += 0.05
        elif dpad_v == -1.0 and self.last_dpad_v != -1.0:
            self.cruise_speed_mps -= 0.05
        self.last_dpad_v = dpad_v
        
        self.cruise_speed_mps = max(-self.max_velocity, min(self.max_velocity, self.cruise_speed_mps))

        lt = self.normalize_trigger(raw_lt, True)
        rt = self.normalize_trigger(raw_rt, False)
        trigger_throttle = rt - lt
        
        if abs(trigger_throttle) > self.deadzone:
            self.latest_linear_vel = trigger_throttle * self.max_velocity
            self.cruise_speed_mps = 0.0
        else:
            self.latest_linear_vel = self.cruise_speed_mps

        # Steering
        analog_steer_input = 0.0 if abs(raw_steer) < self.deadzone else raw_steer
        if abs(analog_steer_input) > 0.0:
            self.latest_steer_input = analog_steer_input
        else:
            if dpad_h == 1.0: self.latest_steer_input = 1.0
            elif dpad_h == -1.0: self.latest_steer_input = -1.0
            else: self.latest_steer_input = 0.0

    def timer_callback(self):
        desired_angle = self.latest_steer_input * self.max_steer_angle
        joint_vel_rads = self.latest_linear_vel / self.wheel_radius

        # 1. MANUAL
        msg_steer = Float64MultiArray()
        msg_steer.data = [float(desired_angle)]
        self.pub_manual_steer.publish(msg_steer)
        
        msg_drive = Float64MultiArray()
        msg_drive.data = [float(joint_vel_rads)]
        self.pub_manual_drive.publish(msg_drive)

        # 2. AUTO
        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = 'base_link'
        ts.twist.linear.x = float(self.latest_linear_vel)
        ts.twist.angular.z = float(desired_angle)
        self.pub_auto_twist.publish(ts)

        # 3. Telemetry
        msg_speed = Float64()
        msg_speed.data = float(self.latest_linear_vel)
        self.pub_current_speed.publish(msg_speed)

def main(args=None):
    rclpy.init(args=args)
    node = RacingTeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
