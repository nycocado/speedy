import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32, String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


def _yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _ang_diff(ref, cur):
    return (ref - cur + math.pi) % (2.0 * math.pi) - math.pi


class ReactiveController(Node):
    """Navegação com Visão (Desejo) + Mini-DWA (Veto).

    Em vez de lutar com a visão fugindo do obstáculo mais próximo, este algoritmo
    simula fisicamente 21 trajetórias (arcos de direção). Se um arco colidir com o
    LiDAR, é cortado. Dos arcos seguros que sobram, ele escolhe o mais parecido com 
    o que a câmara pediu. Isto permite contornar labirintos/chicanes suavemente.
    """

    def __init__(self):
        super().__init__('reactive_controller')

        # --- Hardware / Geometria ---
        self.declare_parameter('steer_max', 0.4363)
        self.declare_parameter('wheelbase', 0.251)
        self.declare_parameter('steer_rate_max', 3.0)    # rad/s: Suaviza o servo

        # --- PID de guiamento ---
        self.declare_parameter('kp_lat', 0.5)
        self.declare_parameter('ki_lat', 0.1)
        self.declare_parameter('kd_lat', 0.05)
        self.declare_parameter('i_max', 0.3)
        self.declare_parameter('k_heading', 0.3)         # Feedforward ( ROI longe )
        self.declare_parameter('k_reacquire', 0.3)       # Viés para 1 fita

        # --- AVOID -> Mini-DWA ---
        self.declare_parameter('wall_detect_range', 0.80)  # Lookahead do DWA
        self.declare_parameter('dwa_half_width', 0.0725)   # 14.5cm total / 2
        # Mantidos declarados para não quebrar o YAML
        self.declare_parameter('avoid_fov_deg', 100.0)
        self.declare_parameter('k_avoid', 0.8)
        self.declare_parameter('dodge_max_frac', 0.6)
        self.declare_parameter('vision_suppress', 0.7)
        self.declare_parameter('center_threat_deg', 10.0)

        # --- Heading-hold (BLIND / Queda) ---
        self.declare_parameter('kp_yaw', 1.0)
        self.declare_parameter('blind_momentum_timeout', 1.5) # Tempo de "fé" na queda da rampa

        # --- Velocidade ---
        self.declare_parameter('v_base', 0.6)
        self.declare_parameter('v_min', 0.15)
        self.declare_parameter('ramp_speed', 0.8)        # Usada como velocidade de inércia

        # --- Segurança ---
        self.declare_parameter('hard_stop_range', 0.15)
        self.declare_parameter('safety_fov_deg', 60.0)

        self.declare_parameter('control_hz', 20.0)
        self.declare_parameter('navigation_enabled', True)

        # --- Tópicos ---
        self.declare_parameter('cmd_topic', '/bicycle_steering_controller/reference')
        self.declare_parameter('state_topic', '/speedy_supervisor/state')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('lateral_topic', '/line_detector/lateral_error')
        self.declare_parameter('heading_topic', '/line_detector/heading_error')
        self.declare_parameter('side_topic', '/line_detector/visible_side')
        self.declare_parameter('odom_topic', '/odometry/filtered')

        # --- Estado ---
        self._wheelbase = self.get_parameter('wheelbase').value
        self._steer_max = self.get_parameter('steer_max').value

        self._lateral = float('nan')
        self._heading = float('nan')
        self._visible_side = 0.0
        self._angles = None
        self._ranges = None
        self._have_scan = False
        self._yaw = None
        self._yaw_ref = None
        self._mode = 'MANUAL'
        
        self._last_line_time = 0.0

        self._i_lat = 0.0
        self._prev_lat = 0.0
        self._steer_prev = 0.0
        self._last_t = None

        g = self.get_parameter
        self.create_subscription(Float32, g('lateral_topic').value, self._lat_cb, 10)
        self.create_subscription(Float32, g('heading_topic').value, self._head_cb, 10)
        self.create_subscription(Float32, g('side_topic').value, self._side_cb, 10)
        self.create_subscription(LaserScan, g('scan_topic').value, self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, g('odom_topic').value, self._odom_cb, 10)
        self.create_subscription(String, g('state_topic').value, self._state_cb, 10)
        self._cmd_pub = self.create_publisher(TwistStamped, g('cmd_topic').value, 10)

        # Debug
        self._dbg_steer_raw_pub = self.create_publisher(Float32, '/reactive_controller/steer_raw', 1)
        self._dbg_steer_out_pub = self.create_publisher(Float32, '/reactive_controller/steer_out', 1)
        self._dbg_v_pub = self.create_publisher(Float32, '/reactive_controller/v_cmd', 1)
        self._dbg_i_lat_pub = self.create_publisher(Float32, '/reactive_controller/i_lat', 1)
        self._dbg_mode_pub = self.create_publisher(String, '/reactive_controller/mode', 1)
        self._dbg_diag_pub = self.create_publisher(DiagnosticStatus, '/reactive_controller/diag', 1)

        hz = g('control_hz').value
        self.create_timer(1.0 / hz, self._control_loop)

    # ---- Callbacks ----
    def _lat_cb(self, msg):
        self._lateral = msg.data

    def _head_cb(self, msg):
        self._heading = msg.data

    def _side_cb(self, msg):
        self._visible_side = msg.data

    def _scan_cb(self, msg):
        r = np.asarray(msg.ranges, dtype=np.float32)
        if self._angles is None or self._angles.shape[0] != r.shape[0]:
            a = msg.angle_min + np.arange(r.shape[0], dtype=np.float64) * msg.angle_increment
            self._angles = (a + np.pi) % (2.0 * np.pi) - np.pi
        self._ranges = r
        self._have_scan = True

    def _odom_cb(self, msg):
        self._yaw = _yaw_from_quat(msg.pose.pose.orientation)

    def _state_cb(self, msg):
        self._mode = msg.data

    # ---- Helpers ----
    def _min_range(self, lo, hi):
        if not self._have_scan:
            return float('inf')
        a, r = self._angles, self._ranges
        m = ((a >= lo) & (a <= hi) & np.isfinite(r) & (r > 0.05))
        return float(r[m].min()) if m.any() else float('inf')

    def _reset_pid(self):
        self._i_lat = 0.0
        self._prev_lat = 0.0

    def _vision(self, p, steer_max, dt):
        err = self._lateral
        self._i_lat = max(-p('i_max'), min(p('i_max'), self._i_lat + err * dt))
        d = (err - self._prev_lat) / dt if dt > 1e-6 else 0.0
        self._prev_lat = err
        steer = p('kp_lat') * err + p('ki_lat') * self._i_lat + p('kd_lat') * d
        if not math.isnan(self._heading):
            steer += p('k_heading') * self._heading
        v = p('v_base') - (p('v_base') - p('v_min')) * min(1.0, abs(steer) / steer_max)
        return steer, v

    def _mini_dwa(self, p, steer_max, desired_steer, desired_v, L):
        """Avaliação de Arcos (Mini-DWA) para desvio de obstáculos."""
        if not self._have_scan:
            return desired_steer, desired_v, False

        num_arcs = 21
        lookahead = p('wall_detect_range')
        half_width = p('dwa_half_width')

        arcs = np.linspace(-steer_max, steer_max, num_arcs)
        
        a, r = self._angles, self._ranges
        valid = np.isfinite(r) & (r > 0.05) & (r < lookahead * 1.5)
        if not valid.any():
            return desired_steer, desired_v, False
            
        xs = r[valid] * np.cos(a[valid])
        ys = r[valid] * np.sin(a[valid])
        
        best_steer = None
        min_cost = float('inf')
        
        for steer in arcs:
            collision = False
            if abs(steer) < 1e-3:
                # Linha reta
                col_mask = (xs > 0.05) & (xs < lookahead) & (np.abs(ys) < half_width)
                if col_mask.any():
                    collision = True
            else:
                # Curva num arco
                R = L / math.tan(steer)
                dists_to_center = np.sqrt(xs**2 + (ys - R)**2)
                path_err = np.abs(dists_to_center - abs(R))
                
                if R > 0:
                    angles_c = np.arctan2(xs, R - ys)
                else:
                    angles_c = np.arctan2(xs, ys - R)
                    
                arc_dist = np.abs(R * angles_c)
                col_mask = (path_err < half_width) & (xs > 0.05) & (arc_dist < lookahead)
                if col_mask.any():
                    collision = True
                    
            if not collision:
                cost = abs(steer - desired_steer)
                if cost < min_cost:
                    min_cost = cost
                    best_steer = steer
                    
        if best_steer is not None:
            diff = abs(best_steer - desired_steer)
            if diff > 0.05:
                # O DWA forçou desvio -> reduz velocidade
                v = p('v_min') + (desired_v - p('v_min')) * (1.0 - diff / steer_max)
                return best_steer, max(p('v_min'), v), True
            return best_steer, desired_v, False
            
        # Dead end
        return desired_steer, 0.0, True

    def _blind(self, p, steer_max):
        steer = 0.0
        if self._yaw is not None and self._yaw_ref is not None:
            steer = p('kp_yaw') * _ang_diff(self._yaw_ref, self._yaw)
        return steer, p('v_min')

    def _reacquire(self, p, steer_max):
        steer = p('k_reacquire') * self._visible_side
        steer = max(-steer_max, min(steer_max, steer))
        return steer, p('v_min')

    def _rate_limit(self, target, rate_max, dt):
        if rate_max <= 0.0 or dt <= 0.0:
            self._steer_prev = target
            return target
        step = rate_max * dt
        self._steer_prev += max(-step, min(step, target - self._steer_prev))
        return self._steer_prev

    # ---- laço ----
    def _control_loop(self):
        now = self.get_clock().now()
        dt = (now - self._last_t).nanoseconds * 1e-9 if self._last_t else 0.05
        self._last_t = now
        if dt <= 0.0:
            dt = 0.05

        if not self.get_parameter('navigation_enabled').value or self._mode != 'AUTO':
            self._reset_pid()
            self._steer_prev = 0.0
            self._publish(0.0, 0.0)
            self._publish_debug(0.0, 0.0, 0.0, 0.0, 'DISABLED')
            return

        p = lambda n: self.get_parameter(n).value
        steer_max = p('steer_max')
        L = p('wheelbase')

        have_line = not math.isnan(self._lateral)
        one_tape = self._visible_side != 0.0
        
        if have_line or one_tape:
            self._last_line_time = now.nanoseconds * 1e-9
            if self._yaw is not None:
                self._yaw_ref = self._yaw

        in_momentum = (now.nanoseconds * 1e-9 - self._last_line_time) < p('blind_momentum_timeout')

        # 1. VISÃO (O Desejo)
        if one_tape:
            self._reset_pid()
            desired_steer, desired_v = self._reacquire(p, steer_max)
            base_mode = 'REACQ_L' if self._visible_side < 0.0 else 'REACQ_R'
        elif have_line:
            desired_steer, desired_v = self._vision(p, steer_max, dt)
            base_mode = 'VISION'
        elif in_momentum:
            self._reset_pid()
            desired_steer, _ = self._blind(p, steer_max)
            desired_v = p('ramp_speed')
            base_mode = 'MOMENTUM'
        else:
            self._reset_pid()
            desired_steer, desired_v = self._blind(p, steer_max)
            base_mode = 'BLIND'

        # 2. LiDAR (O Veto via Mini-DWA)
        if base_mode != 'MOMENTUM':
            steer_target, v_target, is_dodging = self._mini_dwa(p, steer_max, desired_steer, desired_v, L)
            if is_dodging:
                mode = 'STOP' if v_target == 0.0 else base_mode + '+DWA'
            else:
                mode = base_mode
                
            # Paragem Absoluta por Proximidade frontal pura
            front = self._min_range(-math.radians(p('safety_fov_deg')) / 2.0, math.radians(p('safety_fov_deg')) / 2.0)
            if front <= p('hard_stop_range'):
                v_target = 0.0
                mode = 'STOP'
        else:
            steer_target = desired_steer
            v_target = desired_v
            mode = base_mode

        steer_raw = steer_target

        # 3. Rate Limit e Cinemática Final
        steer_out = self._rate_limit(steer_target, p('steer_rate_max'), dt)
        steer_out = max(-steer_max, min(steer_max, steer_out))

        omega = v_target * math.tan(steer_out) / L if v_target > 1e-3 else 0.0
        self._publish(v_target, omega)
        self._publish_debug(steer_raw, steer_out, v_target, self._i_lat, mode)

    def _publish(self, v, omega):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = float(v)
        cmd.twist.angular.z = float(omega)
        self._cmd_pub.publish(cmd)

    def _publish_debug(self, steer_raw, steer_out, v, i_lat, mode):
        def _f32(val): return Float32(data=float(val))
        self._dbg_steer_raw_pub.publish(_f32(steer_raw))
        self._dbg_steer_out_pub.publish(_f32(steer_out))
        self._dbg_v_pub.publish(_f32(v))
        self._dbg_i_lat_pub.publish(_f32(i_lat))
        s = String()
        s.data = mode
        self._dbg_mode_pub.publish(s)

        d = DiagnosticStatus()
        d.name, d.message = 'reactive_controller', mode
        lat = self._lateral if not math.isnan(self._lateral) else float('nan')
        head = self._heading if not math.isnan(self._heading) else float('nan')
        d.values = [
            KeyValue(key='mode', value=mode),
            KeyValue(key='lateral_err', value=f'{lat:.4f}'),
            KeyValue(key='heading_err', value=f'{head:.4f}'),
            KeyValue(key='steer_raw', value=f'{steer_raw:.4f}'),
            KeyValue(key='steer_out', value=f'{steer_out:.4f}'),
            KeyValue(key='i_lat', value=f'{i_lat:.4f}'),
            KeyValue(key='v_cmd', value=f'{v:.3f}'),
        ]
        self._dbg_diag_pub.publish(d)


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
