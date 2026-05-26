import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32, String
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


def _yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _pitch_from_quat(q):
    s = 2.0 * (q.w * q.y - q.z * q.x)
    s = max(-1.0, min(1.0, s))
    return math.asin(s)


def _ang_diff(ref, cur):
    return (ref - cur + math.pi) % (2.0 * math.pi) - math.pi


class ReactiveController(Node):
    """Navegação reativa para o bicycle_steering_controller (Ackermann).

    A visão é a única fonte que conhece a pista (duas fitas), então é a espinha
    dorsal sempre que há linha. O LiDAR não vê a borda da pista — só os painéis —
    por isso entra apenas como um *desvio limitado* somado ao esterço da visão para
    contornar um painel, nunca como navegação principal (senão fugiria da pista pelo
    chão aberto ao lado da fita).
      RAMP   (pitch do IMU):  segura o heading do EKF, sobe a velocidade.
      VISION (fita visível):  PID de guiamento sobre o erro lateral.
      BLIND  (sem fita):      segura o último heading bom (yaw_ref), devagar.
      +DODGE (painel perto):  desvio lateral limitado do LiDAR somado por cima, com
                              a autoridade da visão reduzida conforme a aproximação.
    Só comanda em 'AUTO' (ver speedy_supervisor); fora disso publica zero.
    """

    def __init__(self):
        super().__init__('reactive_controller')

        # Fonte única (injetados pelo launch a partir do hardware.yaml).
        self.declare_parameter('steer_max', 0.4363)
        self.declare_parameter('wheelbase', 0.251)

        # PID de guiamento (erro lateral -> ângulo de esterço). Inverter sinais se virar
        # para o lado errado (vflip da câmera / sentido do servo).
        self.declare_parameter('kp_lat', 0.5)
        self.declare_parameter('ki_lat', 0.1)
        self.declare_parameter('kd_lat', 0.05)
        self.declare_parameter('i_max', 0.3)             # anti-windup do termo integral
        self.declare_parameter('k_heading', 0.3)         # feedforward da curvatura (ROI longe)
        # Re-aquisição (1 fita): viés de esterço p/ o lado da fita EM FALTA, para a trazer
        # de volta ao FOV (curva apertada de 90°). Sinal ~ -kp_lat (verificar em bancada).
        self.declare_parameter('k_reacquire', 0.3)

        # AVOID: desvio limitado do LiDAR (só contorna painel; não navega)
        self.declare_parameter('wall_detect_range', 0.80)  # painel frontal mais perto que isto -> ativa o desvio
        self.declare_parameter('avoid_fov_deg', 100.0)   # setor frontal considerado p/ o desvio
        self.declare_parameter('k_avoid', 0.8)           # ganho do desvio (rad de esterço no máx. de proximidade)
        self.declare_parameter('dodge_max_frac', 0.6)    # teto do desvio como fração do steer_max
        self.declare_parameter('vision_suppress', 0.7)   # quanto a proximidade reduz a autoridade da visão [0..1]
        self.declare_parameter('center_threat_deg', 10.0)  # painel ~à frente: decide o lado pela metade mais aberta

        # Heading-hold (rampa / cego), referência = yaw do EKF
        self.declare_parameter('kp_yaw', 1.0)

        # Rampa
        self.declare_parameter('ramp_pitch_deg', 8.0)    # pitch acima disto -> modo RAMP
        self.declare_parameter('ramp_speed', 0.8)        # m/s para subir com momento
        self.declare_parameter('ramp_steer_scale', 0.4)  # reduz autoridade do esterço na rampa

        # Velocidade
        self.declare_parameter('v_base', 0.6)            # m/s em pista livre
        self.declare_parameter('v_min', 0.15)            # m/s perto de parede / cego

        # Segurança e suavização
        self.declare_parameter('hard_stop_range', 0.15)  # m: para o motor
        self.declare_parameter('safety_fov_deg', 60.0)   # setor frontal p/ deteção de parede/parada
        self.declare_parameter('steer_rate_max', 3.0)    # rad/s: limita a taxa do esterço

        self.declare_parameter('control_hz', 20.0)       # lido no arranque (não dinâmico)
        self.declare_parameter('navigation_enabled', True)  # False = publica zero sem matar o nó

        # Tópicos
        self.declare_parameter('cmd_topic', '/bicycle_steering_controller/reference')
        self.declare_parameter('state_topic', '/speedy_supervisor/state')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('lateral_topic', '/line_detector/lateral_error')
        self.declare_parameter('heading_topic', '/line_detector/heading_error')
        self.declare_parameter('side_topic', '/line_detector/visible_side')
        self.declare_parameter('imu_topic', '/imu/mpu6050')
        self.declare_parameter('odom_topic', '/odometry/filtered')

        # Constantes físicas (fonte única do hardware.yaml via launch) — lidas uma vez.
        self._wheelbase = self.get_parameter('wheelbase').value
        self._steer_max = self.get_parameter('steer_max').value

        self._lateral = float('nan')
        self._heading = float('nan')
        self._visible_side = 0.0     # -1 = só fita esq., +1 = só fita dir., 0 = ambas/nenhuma
        self._angles = None          # ângulos do scan, pré-computados (0 = frente)
        self._ranges = None
        self._range_min = 0.0
        self._range_max = float('inf')
        self._have_scan = False
        self._yaw = None
        self._pitch = None
        self._yaw_ref = None
        self._mode = 'MANUAL'

        # Estado dos controladores
        self._i_lat = 0.0
        self._prev_lat = 0.0
        self._steer_prev = 0.0
        self._last_t = None

        g = self.get_parameter
        self.create_subscription(Float32, g('lateral_topic').value, self._lat_cb, 10)
        self.create_subscription(Float32, g('heading_topic').value, self._head_cb, 10)
        self.create_subscription(Float32, g('side_topic').value, self._side_cb, 10)
        self.create_subscription(LaserScan, g('scan_topic').value, self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Imu, g('imu_topic').value, self._imu_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, g('odom_topic').value, self._odom_cb, 10)
        self.create_subscription(String, g('state_topic').value, self._state_cb, 10)
        self._cmd_pub = self.create_publisher(TwistStamped, g('cmd_topic').value, 10)

        # Debug — visíveis no Foxglove como plots individuais.
        self._dbg_steer_raw_pub = self.create_publisher(Float32, '/reactive_controller/steer_raw', 1)
        self._dbg_steer_out_pub = self.create_publisher(Float32, '/reactive_controller/steer_out', 1)
        self._dbg_v_pub = self.create_publisher(Float32, '/reactive_controller/v_cmd', 1)
        self._dbg_i_lat_pub = self.create_publisher(Float32, '/reactive_controller/i_lat', 1)
        self._dbg_mode_pub = self.create_publisher(String, '/reactive_controller/mode', 1)
        self._dbg_diag_pub = self.create_publisher(DiagnosticStatus, '/reactive_controller/diag', 1)

        hz = g('control_hz').value
        self.create_timer(1.0 / hz, self._control_loop)

    # ---- callbacks ----
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
            self._angles = (a + np.pi) % (2.0 * np.pi) - np.pi   # normaliza p/ 0 = frente
        self._ranges = r
        self._range_min = msg.range_min
        self._range_max = msg.range_max
        self._have_scan = True

    def _imu_cb(self, msg):
        self._pitch = _pitch_from_quat(msg.orientation)

    def _odom_cb(self, msg):
        self._yaw = _yaw_from_quat(msg.pose.pose.orientation)

    def _state_cb(self, msg):
        self._mode = msg.data

    # ---- helpers ----
    def _min_range(self, lo, hi):
        """Menor distância finita no setor angular [lo, hi] (rad, 0 = frente)."""
        if not self._have_scan:
            return float('inf')
        a, r = self._angles, self._ranges
        m = ((a >= lo) & (a <= hi) & np.isfinite(r)
             & (r > self._range_min) & (r < self._range_max))
        return float(r[m].min()) if m.any() else float('inf')

    def _reset_pid(self):
        """Zera só o estado do PID de guiamento (fora do modo VISION)."""
        self._i_lat = 0.0
        self._prev_lat = 0.0

    # ---- modos ----
    def _vision(self, p, steer_max, dt):
        err = self._lateral
        self._i_lat += err * dt
        i_max = p('i_max')
        self._i_lat = max(-i_max, min(i_max, self._i_lat))
        d = (err - self._prev_lat) / dt if dt > 1e-6 else 0.0
        self._prev_lat = err
        steer = p('kp_lat') * err + p('ki_lat') * self._i_lat + p('kd_lat') * d
        if not math.isnan(self._heading):
            steer += p('k_heading') * self._heading
        frac = min(1.0, abs(steer) / steer_max) if steer_max > 1e-6 else 0.0
        v = p('v_base') - (p('v_base') - p('v_min')) * frac
        return steer, v

    def _proximity(self, p, front):
        """0 (longe, no wall_detect_range) -> 1 (no hard_stop_range)."""
        lo = p('hard_stop_range')
        hi = p('wall_detect_range')
        if hi <= lo:
            return 1.0
        return max(0.0, min(1.0, (hi - front) / (hi - lo)))

    def _dodge(self, p, steer_max, front):
        """Desvio lateral limitado p/ contornar um painel (NÃO navega).

        Considera só pontos próximos (< wall_detect_range) no setor frontal: foge do
        ponto mais perto. Painel ~à frente: decide o lado pela metade mais aberta.
        A magnitude escala com a proximidade e satura em dodge_max_frac*steer_max.
        Não persegue espaço livre (evita ser atraído p/ fora da pista).
        """
        if not self._have_scan:
            return 0.0, p('v_base')
        fov = math.radians(p('avoid_fov_deg'))
        a, r = self._angles, self._ranges
        sel = ((a >= -fov / 2.0) & (a <= fov / 2.0) & np.isfinite(r)
               & (r > self._range_min) & (r < self._range_max)
               & (r < p('wall_detect_range')))
        if not sel.any():
            return 0.0, p('v_base')
        asel, rsel = a[sel], r[sel]
        prox = self._proximity(p, front)
        k = p('k_avoid')
        threat = float(asel[int(np.argmin(rsel))])
        if abs(threat) < math.radians(p('center_threat_deg')):
            # Painel ~à frente: vai p/ a metade com mais alcance médio.
            left = rsel[asel > 0.0]
            right = rsel[asel < 0.0]
            lr = (float(left.mean()) if left.size else 0.0) - (float(right.mean()) if right.size else 0.0)
            dodge = math.copysign(k * prox, lr)
        else:
            # Foge do obstáculo mais próximo (+ang = esquerda -> esterça p/ direita).
            dodge = -math.copysign(k * prox, threat)
        dmax = p('dodge_max_frac') * steer_max
        dodge = max(-dmax, min(dmax, dodge))
        v = p('v_min') + (p('v_base') - p('v_min')) * (1.0 - prox)
        return dodge, v

    def _ramp(self, p, steer_max):
        steer = 0.0
        scale = p('ramp_steer_scale')
        if self._yaw is not None and self._yaw_ref is not None:
            steer = p('kp_yaw') * _ang_diff(self._yaw_ref, self._yaw) * scale
            lim = steer_max * scale
            steer = max(-lim, min(lim, steer))
        return steer, p('ramp_speed')

    def _blind(self, p, steer_max):
        steer = 0.0
        if self._yaw is not None and self._yaw_ref is not None:
            steer = p('kp_yaw') * _ang_diff(self._yaw_ref, self._yaw)
        return steer, p('v_min')

    def _reacquire(self, p, steer_max):
        """Só 1 fita visível: esterça p/ o lado OPOSTO da fita p/ trazer a perdida ao FOV.

        _visible_side: -1 = só esquerda visível (curva à direita; falta a direita),
                       +1 = só direita visível (curva à esquerda; falta a esquerda).
        O esterço é um viés fixo no sentido da fita em falta; o sinal de k_reacquire
        ajusta-se em bancada (mesmo critério que kp_lat).
        """
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
        # Sempre lê os parâmetros reais para garantir sincronia com hardware.yaml
        steer_max = p('steer_max')
        L = p('wheelbase')

        front = self._min_range(-math.radians(p('safety_fov_deg')) / 2.0,
                                math.radians(p('safety_fov_deg')) / 2.0)
        on_ramp = self._pitch is not None and abs(self._pitch) > math.radians(p('ramp_pitch_deg'))
        have_line = not math.isnan(self._lateral)
        one_tape = self._visible_side != 0.0
        obstacle = front < p('wall_detect_range')

        # Atualiza yaw_ref sempre que houver referência de pista (2 fitas ou 1).
        # Assim, ao sair de REACQUIRE (curva) o BLIND não segura o heading reto anterior.
        if (have_line or one_tape) and self._yaw is not None:
            self._yaw_ref = self._yaw

        # Esterço base:
        #   VISION    — 2 fitas; PID lateral + heading feedforward
        #   REACQUIRE — 1 fita; viés p/ o lado oposto da visível (re-apanha a perdida)
        #   BLIND     — sem fita; segura o último yaw_ref do EKF
        # REACQUIRE tem prioridade sobre VISION: com 1 fita a estimativa lateral não é
        # fiável numa curva apertada — vale mais virar p/ recuperar as duas fitas.
        if on_ramp:
            self._reset_pid()
            steer, v = self._ramp(p, steer_max)
            mode = 'RAMP'
        elif one_tape:
            self._reset_pid()
            steer, v = self._reacquire(p, steer_max)
            mode = 'REACQ_L' if self._visible_side < 0.0 else 'REACQ_R'
        elif have_line:
            steer, v = self._vision(p, steer_max, dt)
            mode = 'VISION'
        else:
            self._reset_pid()
            steer, v = self._blind(p, steer_max)
            mode = 'BLIND'

        steer_raw = steer

        # Painel perto: soma um desvio limitado do LiDAR e reduz a autoridade da
        # visão conforme a aproximação, p/ o desvio conseguir contornar.
        if obstacle and not on_ramp:
            dodge, v_cap = self._dodge(p, steer_max, front)
            steer = (1.0 - p('vision_suppress') * self._proximity(p, front)) * steer + dodge
            v = min(v, v_cap)
            mode = mode + '+DODGE'

        if front <= p('hard_stop_range'):
            v = 0.0
            mode = 'STOP'

        steer = self._rate_limit(steer, p('steer_rate_max'), dt)
        steer = max(-steer_max, min(steer_max, steer))

        omega = v * math.tan(steer) / L if v > 1e-3 else 0.0
        self._publish(v, omega)
        self._publish_debug(steer_raw, steer, v, self._i_lat, mode)

    def _publish(self, v, omega):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = float(v)
        cmd.twist.angular.z = float(omega)
        self._cmd_pub.publish(cmd)

    def _publish_debug(self, steer_raw, steer_out, v, i_lat, mode):
        def _f32(val):
            m = Float32()
            m.data = float(val)
            return m
        self._dbg_steer_raw_pub.publish(_f32(steer_raw))
        self._dbg_steer_out_pub.publish(_f32(steer_out))
        self._dbg_v_pub.publish(_f32(v))
        self._dbg_i_lat_pub.publish(_f32(i_lat))
        s = String()
        s.data = mode
        self._dbg_mode_pub.publish(s)

        # DiagnosticStatus agrupa tudo para uma vista rápida no Raw Messages.
        d = DiagnosticStatus()
        d.name = 'reactive_controller'
        d.message = mode
        lat = self._lateral if not math.isnan(self._lateral) else float('nan')
        head = self._heading if not math.isnan(self._heading) else float('nan')
        d.values = [
            KeyValue(key='mode',        value=mode),
            KeyValue(key='lateral_err', value=f'{lat:.4f}'),
            KeyValue(key='heading_err', value=f'{head:.4f}'),
            KeyValue(key='steer_raw',   value=f'{steer_raw:.4f}'),
            KeyValue(key='steer_out',   value=f'{steer_out:.4f}'),
            KeyValue(key='i_lat',       value=f'{i_lat:.4f}'),
            KeyValue(key='v_cmd',       value=f'{v:.3f}'),
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
