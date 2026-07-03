#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray, String


class RacingTeleopNode(Node):
    def __init__(self):
        super().__init__('racing_teleop')

        self.declare_parameter('axis_steering', 0)
        self.declare_parameter('axis_throttle', 4)
        self.declare_parameter('axis_brake', 5)
        self.declare_parameter('axis_dpad_h', 6)
        self.declare_parameter('axis_dpad_v', 7)
        self.declare_parameter('max_steer_angle', 0.785)
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('deadzone', 0.05)
        self.declare_parameter('poc_angular_velocity_multiplier', 1.5)
        self.declare_parameter('joy_timeout', 0.3)
        self.declare_parameter('cruise_step', 0.1)

        self.axis_steering = self.get_parameter('axis_steering').value
        self.axis_throttle = self.get_parameter('axis_throttle').value
        self.axis_brake = self.get_parameter('axis_brake').value
        self.axis_dpad_h = self.get_parameter('axis_dpad_h').value
        self.axis_dpad_v = self.get_parameter('axis_dpad_v').value
        self.max_steer_angle = self.get_parameter('max_steer_angle').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.deadzone = self.get_parameter('deadzone').value
        self.poc_angular_velocity_multiplier = self.get_parameter(
            'poc_angular_velocity_multiplier').value
        self.joy_timeout = self.get_parameter('joy_timeout').value
        self.cruise_step = self.get_parameter('cruise_step').value

        self.lt_touched = False
        self.rt_touched = False

        self.cruise_speed = 0.0
        self._prev_dpad_v = 0.0

        self.last_joy_time = None
        self.failsafe_active = False
        self.estopped = False
        self.mode = 'MANUAL'

        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.sub_state = self.create_subscription(
            String, '/speedy_supervisor/state', self.state_callback, 10)

        self.pub_manual_steer = self.create_publisher(
            Float64MultiArray, '/manual_steering_controller/commands', 10)
        self.pub_manual_drive = self.create_publisher(
            Float64MultiArray, '/manual_drive_controller/commands', 10)
        self.pub_auto_twist = self.create_publisher(
            TwistStamped, '/bicycle_steering_controller/reference', 10)

        self.create_timer(0.05, self.watchdog_check)

        self.get_logger().info(
            '[TELEOP] Node initialized: Reading Joystick and publishing commands.')

    def watchdog_check(self):
        now = self.get_clock().now()
        stale = (self.last_joy_time is None or
                 (now - self.last_joy_time).nanoseconds / 1e9 > self.joy_timeout)
        if stale:
            if not self.failsafe_active and self.last_joy_time is not None:
                self.get_logger().warn('[TELEOP] /joy obsoleto — failsafe: comandos zerados.')
            self.failsafe_active = True
            self.lt_touched = False
            self.rt_touched = False
            self.cruise_speed = 0.0
            self.publish_commands(0.0, 0.0, 0.0)
        elif self.failsafe_active:
            self.failsafe_active = False
            self.get_logger().info('[TELEOP] /joy restabelecido — controle manual retomado.')

    def publish_commands(self, desired_angle, linear_vel, steer_input):
        wheel_omega = linear_vel / self.wheel_radius if self.wheel_radius > 1e-6 else 0.0

        msg_steer = Float64MultiArray()
        msg_steer.data = [desired_angle]
        self.pub_manual_steer.publish(msg_steer)

        msg_drive = Float64MultiArray()
        msg_drive.data = [wheel_omega]
        self.pub_manual_drive.publish(msg_drive)

        if self.mode != 'AUTO':
            ts = TwistStamped()
            ts.header.stamp = self.get_clock().now().to_msg()
            ts.header.frame_id = 'base_link'
            ts.twist.linear.x = linear_vel
            ts.twist.angular.z = steer_input * self.poc_angular_velocity_multiplier
            self.pub_auto_twist.publish(ts)

    def normalize_trigger(self, raw_value, is_lt):
        touched = self.lt_touched if is_lt else self.rt_touched
        if not touched and raw_value == 0.0:
            return 0.0
        if not touched and raw_value != 0.0:
            if is_lt:
                self.lt_touched = True
            else:
                self.rt_touched = True
        val = (1.0 - raw_value) / 2.0
        return max(0.0, min(1.0, val))

    def state_callback(self, msg: String):
        self.mode = msg.data.upper()
        self.estopped = (self.mode == 'ESTOP')

    def joy_callback(self, msg: Joy):
        self.last_joy_time = self.get_clock().now()

        if self.estopped:
            self.publish_commands(0.0, 0.0, 0.0)
            return

        raw_steer = msg.axes[self.axis_steering] if len(msg.axes) > self.axis_steering else 0.0
        raw_lt = msg.axes[self.axis_brake] if len(msg.axes) > self.axis_brake else 1.0
        raw_rt = msg.axes[self.axis_throttle] if len(msg.axes) > self.axis_throttle else 1.0
        dpad_h = msg.axes[self.axis_dpad_h] if len(msg.axes) > self.axis_dpad_h else 0.0
        dpad_v = msg.axes[self.axis_dpad_v] if len(msg.axes) > self.axis_dpad_v else 0.0

        steer_input = 0.0 if abs(raw_steer) < self.deadzone else raw_steer
        lt = self.normalize_trigger(raw_lt, True)
        rt = self.normalize_trigger(raw_rt, False)

        throttle = rt - lt
        if abs(throttle) >= self.deadzone and self.cruise_speed != 0.0:
            self.cruise_speed = 0.0
            self.get_logger().info('[TELEOP] Cruise cancelado.')

        if dpad_v > 0.5 and self._prev_dpad_v <= 0.5:
            self.cruise_speed = round(
                min(self.cruise_speed + self.cruise_step, self.max_velocity), 3)
            self.get_logger().info(f'[TELEOP] Cruise: {self.cruise_speed:.2f} m/s')
        elif dpad_v < -0.5 and self._prev_dpad_v >= -0.5:
            self.cruise_speed = round(
                max(self.cruise_speed - self.cruise_step, -self.max_velocity), 3)
            if self.cruise_speed == 0.0:
                self.get_logger().info('[TELEOP] Cruise desativado.')
            else:
                self.get_logger().info(f'[TELEOP] Cruise: {self.cruise_speed:.2f} m/s')
        self._prev_dpad_v = dpad_v

        if abs(dpad_h) > 0.5:
            desired_angle = dpad_h * self.max_steer_angle
            steer_input = dpad_h
        else:
            desired_angle = steer_input * self.max_steer_angle

        if self.cruise_speed != 0.0:
            linear_vel = self.cruise_speed
        else:
            linear_vel = (throttle if abs(throttle) >= self.deadzone else 0.0) * self.max_velocity

        self.publish_commands(desired_angle, linear_vel, steer_input)


def main(args=None):
    rclpy.init(args=args)
    node = RacingTeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
