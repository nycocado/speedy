#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from controller_manager_msgs.srv import SwitchController

class SpeedySupervisorNode(Node):
    """
    Máquina de Estados de Alto Nível do Speedy.
    Alterna ativamente entre controladores de hardware (Manual Direto vs Cinemático).
    """
    def __init__(self):
        super().__init__('speedy_supervisor')

        self.declare_parameter('btn_select', 10)
        self.declare_parameter('btn_start', 11)

        self.btn_select = self.get_parameter('btn_select').value
        self.btn_start = self.get_parameter('btn_start').value

        self.state = 'MANUAL'
        self.hold_start_time = None
        self.toggling = False

        # Publishers / Subscriptions
        self.pub_state = self.create_publisher(String, '/speedy_supervisor/state', 10)
        self.sub_joy = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.switch_client = self.create_client(SwitchController, '/controller_manager/switch_controller')

        # Timer para publicar estado periodicamente (1Hz)
        self.create_timer(1.0, self.publish_state)

        self.get_logger().info('[SUPERVISOR] Node initialized.')
        self.get_logger().info('[SUPERVISOR] Initial State: MANUAL (Direct Control)')
        
        # Garante que inicie no estado correto
        self.switch_to_mode('MANUAL')

    def publish_state(self):
        msg = String()
        msg.data = self.state
        self.pub_state.publish(msg)

    def switch_to_mode(self, mode):
        if not self.switch_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('[SUPERVISOR] SwitchController service unavailable. Is Controller Manager running?')
            return
        
        req = SwitchController.Request()
        req.strictness = SwitchController.Request.BEST_EFFORT
        
        if mode == 'AUTO':
            req.activate_controllers = ['bicycle_steering_controller']
            req.deactivate_controllers = ['manual_steering_controller', 'manual_drive_controller']
        else:
            req.activate_controllers = ['manual_steering_controller', 'manual_drive_controller']
            req.deactivate_controllers = ['bicycle_steering_controller']
            
        future = self.switch_client.call_async(req)
        future.add_done_callback(self.switch_callback)

    def switch_callback(self, future):
        try:
            response = future.result()
            if response.ok:
                self.get_logger().info(f'[SUPERVISOR] Hardware transition to {self.state} completed successfully.')
            else:
                self.get_logger().error('[SUPERVISOR] Failed to transition hardware in Controller Manager.')
        except Exception as e:
            self.get_logger().error(f'[SUPERVISOR] Error calling SwitchController: {e}')

    def joy_callback(self, msg: Joy):
        now = self.get_clock().now()
        
        select_pressed = msg.buttons[self.btn_select] == 1
        start_pressed = msg.buttons[self.btn_start] == 1

        if select_pressed and start_pressed:
            if self.hold_start_time is None:
                self.hold_start_time = now
            else:
                duration = (now - self.hold_start_time).nanoseconds / 1e9
                if duration >= 2.0 and not self.toggling:
                    self.state = 'AUTO' if self.state == 'MANUAL' else 'MANUAL'
                    self.get_logger().warn(f'[SUPERVISOR] Switching to MODE {self.state}')
                    self.switch_to_mode(self.state)
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