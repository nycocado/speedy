#!/usr/bin/env python3
import math
import statistics

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                       QoSHistoryPolicy)
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry


class ServoCalibratorNode(Node):
    """
    Aquisição de dados para calibração da direção.

    Relaciona o pulso comandado (PWM) com o ângulo de esterço FÍSICO, estimado
    pelo modelo bicycle: delta = atan(omega * L / v).

    Medir só o yaw rate (omega) não basta: omega escala com a velocidade, e a
    média sobre a janela inteira é diluída por qualquer tempo parado. Aqui só
    entram amostras em regime (v >= min_velocity), e o ângulo é calculado por
    amostra a partir do par (omega, v) sincronizado.
    """

    def __init__(self):
        super().__init__('servo_calibrator')

        self.declare_parameter('select_button', 10)
        self.declare_parameter('imu_topic', '/imu/mpu6050')
        self.declare_parameter('pulse_topic', '/steering/pulse_ms')
        self.declare_parameter('odom_topic', '/front_wheels/odom')
        self.declare_parameter('wheelbase', 0.251)        # m — bate com hardware.yaml
        self.declare_parameter('min_velocity', 0.05)      # m/s — gate de regime

        self._btn_index = self.get_parameter('select_button').value
        self._imu_topic = self.get_parameter('imu_topic').value
        self._pulse_topic = self.get_parameter('pulse_topic').value
        self._odom_topic = self.get_parameter('odom_topic').value
        self._wheelbase = float(self.get_parameter('wheelbase').value)
        self._min_velocity = float(self.get_parameter('min_velocity').value)

        # Estado
        self._is_recording = False
        self._prev_btn_state = False
        self._last_pulse_received = 0.0
        self._last_v = 0.0

        # Amostras aceites (em regime): pares sincronizados omega/v + pulso
        self._omega_samples = []   # rad/s
        self._v_samples = []       # m/s
        self._pulse_samples = []   # ms
        self._dropped_slow = 0     # amostras descartadas por v < min_velocity

        self._cb_group = ReentrantCallbackGroup()

        # Resultados latched (TRANSIENT_LOCAL): persistem para assinantes que
        # chegam atrasados (foxglove_bridge), evitando perder o valor publicado.
        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self._angle_pub = self.create_publisher(Float64, '/servo_calibration/measured_angle_deg', latched)
        self._pulse_pub = self.create_publisher(Float64, '/servo_calibration/output_pulse', latched)
        self._omega_pub = self.create_publisher(Float64, '/servo_calibration/measured_omega_z', latched)
        self._vel_pub = self.create_publisher(Float64, '/servo_calibration/measured_velocity', latched)

        # Subs sempre-ativas. O custo ocioso é só desserialização + early-return
        # (os callbacks de aquisição só acumulam quando _is_recording). Não se
        # destroem subs em runtime: fazê-lo de dentro de um callback corrompe o
        # wait set do executor (InvalidHandle) e mata o nó.
        self.create_subscription(Joy, '/joy', self._joy_callback, 10, callback_group=self._cb_group)
        self.create_subscription(Imu, self._imu_topic, self._imu_callback, 10, callback_group=self._cb_group)
        self.create_subscription(Float64, self._pulse_topic, self._pulse_callback, 10, callback_group=self._cb_group)
        self.create_subscription(Odometry, self._odom_topic, self._odom_callback, 10, callback_group=self._cb_group)

        self.get_logger().info(
            f'CALIBRATION_NODE_READY [SELECT_BTN: {self._btn_index}] '
            f'[L: {self._wheelbase:.3f} m] [V_MIN: {self._min_velocity:.2f} m/s]'
        )

    def _odom_callback(self, msg):
        self._last_v = msg.twist.twist.linear.x

    def _pulse_callback(self, msg):
        self._last_pulse_received = msg.data

    def _imu_callback(self, msg):
        if not self._is_recording:
            return
        v = self._last_v
        # Gate de regime: descarta amostras quase-paradas, onde delta=atan(omega*L/v)
        # amplifica ruído e a média é diluída.
        if v < self._min_velocity:
            self._dropped_slow += 1
            return
        self._omega_samples.append(msg.angular_velocity.z)
        self._v_samples.append(v)
        self._pulse_samples.append(self._last_pulse_received)

    def _joy_callback(self, msg):
        if len(msg.buttons) <= self._btn_index:
            return
        current_pressed = bool(msg.buttons[self._btn_index])
        if current_pressed and not self._prev_btn_state:
            if not self._is_recording:
                self._start_session()
            else:
                self._stop_session()
        self._prev_btn_state = current_pressed

    def _start_session(self):
        self._omega_samples.clear()
        self._v_samples.clear()
        self._pulse_samples.clear()
        self._dropped_slow = 0
        self._is_recording = True
        self.get_logger().info('SESSION_STARTED: ACQUIRING_DATA...')

    def _stop_session(self):
        self._is_recording = False
        self.get_logger().info('SESSION_STOPPED: PROCESSING...')

        n = len(self._omega_samples)
        if n == 0:
            self.get_logger().error(
                f'PROCESSING_FAILED: NO_STEADY_SAMPLES '
                f'(descartadas {self._dropped_slow} por v < {self._min_velocity:.2f} m/s). '
                f'O carro precisa andar > {self._min_velocity:.2f} m/s durante a gravação.'
            )
            return

        mean_v = statistics.fmean(self._v_samples)
        std_v = statistics.pstdev(self._v_samples) if n > 1 else 0.0
        mean_omega = statistics.fmean(self._omega_samples)
        std_omega = statistics.pstdev(self._omega_samples) if n > 1 else 0.0
        mean_pulse = statistics.fmean(self._pulse_samples) if self._pulse_samples else self._last_pulse_received

        # Ângulo por amostra (modelo bicycle) e estatística sobre ele.
        angles_deg = [
            math.degrees(math.atan(w * self._wheelbase / v))
            for w, v in zip(self._omega_samples, self._v_samples)
        ]
        mean_deg = statistics.fmean(angles_deg)
        std_deg = statistics.pstdev(angles_deg) if n > 1 else 0.0

        self._angle_pub.publish(Float64(data=mean_deg))
        self._pulse_pub.publish(Float64(data=mean_pulse))
        self._omega_pub.publish(Float64(data=mean_omega))
        self._vel_pub.publish(Float64(data=mean_v))

        self.get_logger().info(
            f'RESULT: [ANGLE: {mean_deg:+.2f} +/- {std_deg:.2f} deg] '
            f'[PULSE: {mean_pulse:.4f} ms] '
            f'[V: {mean_v:.3f} +/- {std_v:.3f} m/s] '
            f'[OMEGA: {mean_omega:+.4f} +/- {std_omega:.4f} rad/s] '
            f'[N: {n} aceites, {self._dropped_slow} lentas]'
        )

        # Avisos de qualidade: regime instável compromete a estimativa do ângulo.
        if mean_v > 1e-6 and (std_v / mean_v) > 0.30:
            self.get_logger().warn(
                'VELOCIDADE_INSTAVEL: desvio da velocidade > 30%. '
                'Tenta manter velocidade constante durante a gravação.'
            )
        if std_deg > 2.0:
            self.get_logger().warn(
                f'ANGULO_RUIDOSO: desvio do ângulo {std_deg:.2f} deg. '
                'Janela curta ou regime instável — repete com tração mais estável.'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ServoCalibratorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
