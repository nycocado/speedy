#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from controller_manager_msgs.srv import SwitchController


class SpeedySupervisorNode(Node):
    def __init__(self):
        super().__init__('speedy_supervisor')

        self.declare_parameter('btn_select', 10)
        self.declare_parameter('btn_start', 11)
        self.declare_parameter('btn_estop', 1)
        self.declare_parameter('hold_duration', 2.0)

        self.btn_select = self.get_parameter('btn_select').value
        self.btn_start = self.get_parameter('btn_start').value
        self.btn_estop = self.get_parameter('btn_estop').value
        self.hold_duration = self.get_parameter('hold_duration').value

        self.state = 'MANUAL'
        self.hold_start_time = None
        self.toggling = False

        self.estopped = False
        self.estop_prev_pressed = False

        self.pub_state = self.create_publisher(String, '/speedy_supervisor/state', 10)
        self.sub_joy = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.switch_client = self.create_client(
            SwitchController, '/controller_manager/switch_controller')

        self.create_timer(1.0, self.publish_state)

        self.get_logger().info('[SUPERVISOR] Node initialized.')
        self.get_logger().info(
            '[SUPERVISOR] Estado inicial: MANUAL (controladores manuais já ativos).')

    def publish_state(self):
        msg = String()
        msg.data = 'ESTOP' if self.estopped else self.state
        self.pub_state.publish(msg)

    def switch_to_mode(self, target):
        if not self.switch_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                f'[SUPERVISOR] SwitchController indisponível — modo mantido em {
                    self.state}.')
            return

        req = SwitchController.Request()
        req.strictness = SwitchController.Request.STRICT
        if target == 'AUTO':
            req.activate_controllers = ['bicycle_steering_controller']
            req.deactivate_controllers = ['manual_steering_controller', 'manual_drive_controller']
        else:
            req.activate_controllers = ['manual_steering_controller', 'manual_drive_controller']
            req.deactivate_controllers = ['bicycle_steering_controller']

        future = self.switch_client.call_async(req)
        future.add_done_callback(lambda f: self._switch_done(f, target))

    def _switch_done(self, future, target):
        try:
            ok = future.result().ok
        except Exception as e:
            self.get_logger().error(
                f'[SUPERVISOR] Erro no SwitchController: {e} — modo mantido em {
                    self.state}.')
            return
        if ok:
            self.state = target
            self.get_logger().warn(f'[SUPERVISOR] Hardware em {self.state}.')
        else:
            self.get_logger().error(
                f'[SUPERVISOR] Falha ao trocar p/ {target} — modo mantido em {self.state}. '
                f'Cheque "ros2 control list_controllers".')

    def joy_callback(self, msg: Joy):
        now = self.get_clock().now()

        def pressed(idx):
            return len(msg.buttons) > idx and msg.buttons[idx] == 1

        estop_pressed = pressed(self.btn_estop)
        if estop_pressed and not self.estop_prev_pressed:
            self.estopped = not self.estopped
            if self.estopped:
                self.get_logger().error('[SUPERVISOR] E-STOP ATIVADO — comandos zerados.')
            else:
                self.get_logger().warn('[SUPERVISOR] E-stop liberado.')
            self.publish_state()
        self.estop_prev_pressed = estop_pressed

        if self.estopped:
            self.hold_start_time = None
            self.toggling = False
            return

        select_pressed = pressed(self.btn_select)
        start_pressed = pressed(self.btn_start)

        if select_pressed and start_pressed:
            if self.hold_start_time is None:
                self.hold_start_time = now
            else:
                duration = (now - self.hold_start_time).nanoseconds / 1e9
                if duration >= self.hold_duration and not self.toggling:
                    target = 'AUTO' if self.state == 'MANUAL' else 'MANUAL'
                    self.get_logger().warn(f'[SUPERVISOR] Solicitando troca p/ {target}...')
                    self.switch_to_mode(target)
                    self.toggling = True
        else:
            self.hold_start_time = None
            self.toggling = False


def main(args=None):
    rclpy.init(args=args)
    node = SpeedySupervisorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
