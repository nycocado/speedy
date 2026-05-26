import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32, String
from cv_bridge import CvBridge
from foxglove_msgs.msg import (
    ImageAnnotations, PointsAnnotation, CircleAnnotation, TextAnnotation,
    Point2, Color,
)


def _color(r, g, b, a=1.0):
    c = Color()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
    return c


def _pt(x, y):
    p = Point2()
    p.x, p.y = float(x), float(y)
    return p


class LineDetectorNode(Node):
    """Seguimento de lane (duas fitas escuras em chão claro).

    Varredura por linhas horizontais: em cada linha acha os trechos escuros *finos*
    (a fita) e descarta os *largos* (painel, sombra). Em vez de colapsar tudo num
    "centro" por linha, **rastreia as duas fitas como duas retas separadas** (x = m·y + c),
    associando cada candidato à fita esquerda ou direita pela predição do frame anterior
    (tracking temporal). Daí saem:
      - 2 fitas: centro = média das duas retas -> lateral + heading.
      - 1 fita:  heading vem da reta real dessa fita (fiável, sem zig-zag); lateral só
                 se a ID for confiante (a fita já tinha track), senão NaN (degradação segura).
      - 0 fitas: NaN nos dois -> o controlador cai para heading-hold do EKF.
    Um obstáculo entre as fitas não pertence a nenhuma das duas (fora do gate de associação)
    -> excluído por construção.
    """

    def __init__(self):
        super().__init__('line_detector')

        # Banda de análise (faixa inferior da imagem = chão à frente).
        self.declare_parameter('band_bottom_frac', 0.0)   # ignora estes % de baixo (nariz do carro)
        self.declare_parameter('band_top_frac', 0.50)      # sobe até estes % da altura
        self.declare_parameter('n_rows', 12)               # linhas de varredura na banda
        self.declare_parameter('blur_kernel', 5)           # blur pré-threshold (ímpar)

        # Gate de largura do trecho escuro (fração da largura da imagem).
        # Fita "meio grossa" -> max folgado; painel de 60cm fica muito acima disto.
        self.declare_parameter('min_edge_w_frac', 0.008)
        self.declare_parameter('max_edge_w_frac', 0.18)

        # Validade e estimativa de largura da lane (para fallback de uma borda só).
        self.declare_parameter('min_valid_rows', 4)
        self.declare_parameter('lane_half_init_frac', 0.22)  # meia-largura inicial da lane
        self.declare_parameter('lane_width_min_frac', 0.12)  # sanidade p/ aceitar par de bordas
        self.declare_parameter('lane_width_max_frac', 0.95)
        self.declare_parameter('lane_ema_alpha', 0.1)        # suavização da meia-largura
        self.declare_parameter('max_rate_hz', 15.0)          # teto de frames processados/s (câmera segue a 30; 0 = sem teto)
        self.declare_parameter('enabled', True)               # False = congela processamento sem matar o nó

        # Rejeição de outliers no ajuste de cada fita (marcas/obstáculos colineares).
        self.declare_parameter('outlier_reject', True)
        self.declare_parameter('outlier_max_frac', 0.06)     # ponto a > esta fração da largura, da reta = outlier

        # Tracking temporal das duas fitas (associação por predição do frame anterior).
        self.declare_parameter('assoc_gate_frac', 0.12)     # candidato a > esta fração da fita prevista = não associa
        self.declare_parameter('track_max_miss', 5)         # frames sem ver a fita até largar o track

        self._bridge = CvBridge()
        self._undist_maps = None  # (map1, map2) — precomputados uma vez de /camera_info
        self._auto = False
        self._enabled = self.get_parameter('enabled').value
        self._last_proc = 0.0
        self._lane_half = None    # meia-largura da lane em px (lazy-init com a 1ª imagem)
        self._left = None         # track da fita esquerda: {'m','c','miss'} (x = m*y + c)
        self._right = None        # track da fita direita

        # Cache para o overlay (preenchido a cada frame processado).
        self._ov_left = []
        self._ov_right = []
        self._ov_fit = None
        self._ov_msg = None

        self._sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(CameraInfo, '/speedy_camera/camera_info', self._info_cb, self._sensor_qos)
        self._image_sub = None  # Subscrição dinâmica (lazy)
        self.create_subscription(String, '/speedy_supervisor/state', self._state_cb, 10)
        self._lateral_pub = self.create_publisher(Float32, '/line_detector/lateral_error', 10)
        self._heading_pub = self.create_publisher(Float32, '/line_detector/heading_error', 10)
        # Qual fita está visível quando só há uma: -1 = só esquerda, +1 = só direita,
        # 0 = ambas ou nenhuma. O controller usa isto p/ re-aquirir a fita perdida.
        self._side_pub = self.create_publisher(Float32, '/line_detector/visible_side', 10)
        # Overlay leve: o Foxglove desenha estas anotações por cima do /image_raw.
        self._annot_pub = self.create_publisher(ImageAnnotations, '/line_detector/annotations', 1)

        # Reage a `enabled` em runtime: desligar larga a subscrição do /image_raw
        # (o transporte da imagem crua a 30fps é o custo no Pi4, não o processamento).
        self.add_on_set_parameters_callback(self._on_params)

    def _state_cb(self, msg: String):
        self._auto = (msg.data == 'AUTO')
        self._update_image_sub()

    def _on_params(self, params):
        for p in params:
            if p.name == 'enabled':
                self._enabled = bool(p.value)
        self._update_image_sub()
        return SetParametersResult(successful=True)

    def _update_image_sub(self):
        """Subscreve a imagem só quando AUTO e enabled; senão larga-a (poupa CPU/banda)."""
        want = self._auto and self._enabled
        if want and self._image_sub is None:
            self._image_sub = self.create_subscription(Image, '/speedy_camera/image_raw', self._image_cb, self._sensor_qos)
            self.get_logger().info('[line_detector] Subscrito no /image_raw.')
        elif not want and self._image_sub is not None:
            self.destroy_subscription(self._image_sub)
            self._image_sub = None
            self.get_logger().info('[line_detector] Subscrição do /image_raw cancelada (poupar CPU/banda).')

    def _info_cb(self, msg: CameraInfo):
        if self._undist_maps is not None:
            return
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        D = np.array(msg.d, dtype=np.float64)
        # P[:3,:3] é a câmara matrix ótima pós-retificação (sem bordas pretas, FOV reduzido).
        new_K = np.array(msg.p, dtype=np.float64).reshape(3, 4)[:, :3]
        self._undist_maps = cv2.initUndistortRectifyMap(
            K, D, None, new_K, (msg.width, msg.height), cv2.CV_16SC2
        )
        self.get_logger().info('[line_detector] Mapa de undistorção inicializado.')

    def _segments_in_row(self, bin_row, min_w, max_w):
        """Centros de TODOS os trechos-fita numa linha binária, filtrados por largura.

        Trecho fino demais = ruído; largo demais = painel/sombra. Devolve lista de x (px).
        """
        m = (bin_row > 0).astype(np.int8)
        d = np.diff(np.concatenate(([0], m, [0])))
        starts = np.flatnonzero(d == 1)
        ends = np.flatnonzero(d == -1)        # índice exclusivo do fim
        if starts.size == 0:
            return []
        widths = ends - starts
        keep = (widths >= min_w) & (widths <= max_w)
        if not keep.any():
            return []
        centers = (starts + ends - 1) * 0.5
        return centers[keep].tolist()

    def _robust_fit(self, ys, cs, min_valid, thr_px):
        """Ajuste linear com rejeição de outliers.

        Mínimos quadrados é sensível: um centro falso (obstáculo/marca que passou o
        filtro de largura) torce a reta. Ajusta, mede o resíduo de cada ponto à reta,
        descarta os que estão a mais de thr_px e reajusta (até 2 passagens). Nunca
        desce abaixo de min_valid pontos — se cortar demais, mantém o ajuste anterior.
        Devolve (slope, intercept, ys_inliers, cs_inliers).
        """
        slope, intercept = np.polyfit(ys, cs, 1)
        if thr_px <= 0.0:
            return slope, intercept, ys, cs
        for _ in range(2):
            resid = np.abs(cs - (slope * ys + intercept))
            keep = resid <= thr_px
            if keep.all() or int(keep.sum()) < min_valid:
                break
            ys, cs = ys[keep], cs[keep]
            slope, intercept = np.polyfit(ys, cs, 1)
        return slope, intercept, ys, cs

    def _image_cb(self, msg: Image):
        if not self._auto or not self._enabled:
            return
        # Câmera publica a 30fps; processar a lane a essa taxa não acrescenta nada e
        # come CPU. (Não evita a desserialização do rclpy, mas corta todo o resto.)
        max_rate = self.get_parameter('max_rate_hz').value
        if max_rate > 0.0:
            now = time.monotonic()
            if (now - self._last_proc) < (1.0 / max_rate):
                return
            self._last_proc = now

        gray = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        if self._undist_maps is not None:
            gray = cv2.remap(gray, self._undist_maps[0], self._undist_maps[1], cv2.INTER_LINEAR)
        h, w = gray.shape[:2]
        cx = w * 0.5

        if self._lane_half is None:
            self._lane_half = self.get_parameter('lane_half_init_frac').value * w

        blur = self.get_parameter('blur_kernel').value
        if blur % 2 == 0:
            blur += 1
        n_rows = self.get_parameter('n_rows').value
        min_w = max(1, int(self.get_parameter('min_edge_w_frac').value * w))
        max_w = int(self.get_parameter('max_edge_w_frac').value * w)
        min_valid = self.get_parameter('min_valid_rows').value
        lane_w_min = self.get_parameter('lane_width_min_frac').value * w
        lane_w_max = self.get_parameter('lane_width_max_frac').value * w
        alpha = self.get_parameter('lane_ema_alpha').value

        y0 = int(h * (1.0 - self.get_parameter('band_top_frac').value))
        y1 = int(h * (1.0 - self.get_parameter('band_bottom_frac').value))
        band = gray[y0:y1, :]

        blurred = cv2.GaussianBlur(band, (blur, blur), 0)
        # Threshold global (Otsu): o miolo do painel fica escuro -> trecho largo -> rejeitado.
        # (O adaptativo marcaria só as bordas finas do painel, fingindo serem fitas.)
        _, binary = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)

        rows = np.linspace(0, band.shape[0] - 1, n_rows).astype(int)
        # Candidatos (todos os trechos-fita) por linha de varredura.
        row_cands = []
        for ry in rows:
            cs = self._segments_in_row(binary[ry], min_w, max_w)
            if cs:
                row_cands.append((y0 + ry, cs))

        # Estado dos tracks ANTES de atualizar (decide a confiança da ID com 1 fita).
        had_left = self._left is not None
        had_right = self._right is not None
        any_track = had_left or had_right

        # Associa cada candidato à fita esquerda ou direita.
        gate = self.get_parameter('assoc_gate_frac').value * w
        left_pts, right_pts = [], []
        for (y, cs) in row_cands:
            if any_track:
                # Referência = reta prevista de cada track; sem track de um lado, a
                # posição esperada de uma lane centrada (cx ± meia-largura).
                xL = self._left['m'] * y + self._left['c'] if had_left else (cx - self._lane_half)
                xR = self._right['m'] * y + self._right['c'] if had_right else (cx + self._lane_half)
                bestL = bestR = None
                bdL = bdR = gate
                for x in cs:
                    dL, dR = abs(x - xL), abs(x - xR)
                    if dL <= dR:
                        if dL < bdL:
                            bdL, bestL = dL, x
                    elif dR < bdR:
                        bdR, bestR = dR, x
                if bestL is not None:
                    left_pts.append((y, bestL))
                if bestR is not None:
                    right_pts.append((y, bestR))
            else:
                # Bootstrap (sem tracks): par mais interno à volta do centro.
                cs_arr = np.asarray(cs)
                left = cs_arr[cs_arr < cx]
                right = cs_arr[cs_arr > cx]
                if left.size:
                    left_pts.append((y, float(left.max())))
                if right.size:
                    right_pts.append((y, float(right.min())))

        thr_px = self.get_parameter('outlier_max_frac').value * w
        if not self.get_parameter('outlier_reject').value:
            thr_px = 0.0
        max_miss = self.get_parameter('track_max_miss').value

        # Ajusta cada fita (x = m*y + c) e atualiza/expira o track.
        mL = cL = mR = cR = None
        ov_left, ov_right = [], []
        if len(left_pts) >= min_valid:
            ys_l = np.array([p[0] for p in left_pts], dtype=np.float64)
            xs_l = np.array([p[1] for p in left_pts], dtype=np.float64)
            mL, cL, ys_lf, xs_lf = self._robust_fit(ys_l, xs_l, min_valid, thr_px)
            self._left = {'m': mL, 'c': cL, 'miss': 0}
            ov_left = list(zip(xs_lf.tolist(), ys_lf.tolist()))
        elif self._left is not None:
            self._left['miss'] += 1
            if self._left['miss'] > max_miss:
                self._left = None

        if len(right_pts) >= min_valid:
            ys_r = np.array([p[0] for p in right_pts], dtype=np.float64)
            xs_r = np.array([p[1] for p in right_pts], dtype=np.float64)
            mR, cR, ys_rf, xs_rf = self._robust_fit(ys_r, xs_r, min_valid, thr_px)
            self._right = {'m': mR, 'c': cR, 'miss': 0}
            ov_right = list(zip(xs_rf.tolist(), ys_rf.tolist()))
        elif self._right is not None:
            self._right['miss'] += 1
            if self._right['miss'] > max_miss:
                self._right = None

        # Base fixa (bordas da banda) -> escala de heading consistente entre frames.
        near_y = float(y1 - 1)   # base = mais perto do carro
        far_y = float(y0)        # topo = lookahead
        have_left = mL is not None
        have_right = mR is not None

        lateral_msg = Float32()
        heading_msg = Float32()
        side_msg = Float32()
        side_msg.data = 0.0      # ambas ou nenhuma fita -> sem re-aquisição
        self._ov_fit = None
        self._ov_msg = None

        if have_left and have_right:
            xL_near, xL_far = mL * near_y + cL, mL * far_y + cL
            xR_near, xR_far = mR * near_y + cR, mR * far_y + cR
            sep = xR_near - xL_near
            if lane_w_min <= sep <= lane_w_max:
                self._lane_half = (1.0 - alpha) * self._lane_half + alpha * (sep * 0.5)
            c_near = 0.5 * (xL_near + xR_near)
            c_far = 0.5 * (xL_far + xR_far)
            lateral_msg.data = float((c_near - cx) / cx)
            heading_msg.data = float((c_far - c_near) / cx)
            self._ov_fit = (c_near, near_y, c_far, far_y)
        elif have_left or have_right:
            # Uma fita: heading vem da reta real dela (fiável). Lateral só com ID confiante
            # (a fita já tinha track antes deste frame), senão suprime (NaN).
            if have_left:
                m, c, off, confident = mL, cL, self._lane_half, had_left
                side_msg.data = -1.0   # só a esquerda visível -> a direita perdeu-se
            else:
                m, c, off, confident = mR, cR, -self._lane_half, had_right
                side_msg.data = 1.0    # só a direita visível -> a esquerda perdeu-se
            x_near, x_far = m * near_y + c, m * far_y + c
            heading_msg.data = float((x_far - x_near) / cx)
            if confident:
                c_near, c_far = x_near + off, x_far + off
                lateral_msg.data = float((c_near - cx) / cx)
                self._ov_fit = (c_near, near_y, c_far, far_y)
            else:
                lateral_msg.data = float('nan')
                self._ov_fit = (x_near, near_y, x_far, far_y)
            self._ov_msg = 'SEE L' if have_left else 'SEE R'
        else:
            lateral_msg.data = float('nan')
            heading_msg.data = float('nan')
            self._ov_msg = 'NO LANE'

        self._ov_left = ov_left
        self._ov_right = ov_right
        self._lateral_pub.publish(lateral_msg)
        self._heading_pub.publish(heading_msg)
        self._side_pub.publish(side_msg)

        if self._annot_pub.get_subscription_count() > 0:
            self._publish_overlay(msg.header.stamp, w, h, y0, y1)

    def _publish_overlay(self, stamp, w, h, y0, y1):
        ann = ImageAnnotations()

        band = PointsAnnotation()
        band.timestamp = stamp
        band.type = PointsAnnotation.LINE_LOOP
        band.points = [_pt(0, y0), _pt(w, y0), _pt(w, y1), _pt(0, y1)]
        band.outline_color = _color(0, 0.6, 0.8)
        band.thickness = 2.0
        ann.points.append(band)

        center = PointsAnnotation()
        center.timestamp = stamp
        center.type = PointsAnnotation.LINE_LIST
        center.points = [_pt(w / 2.0, 0), _pt(w / 2.0, h)]
        center.outline_color = _color(0.7, 0.7, 0.7)
        center.thickness = 1.0
        ann.points.append(center)

        for (x, y) in self._ov_left:
            ci = CircleAnnotation()
            ci.timestamp = stamp
            ci.position = _pt(x, y)
            ci.diameter = 8.0
            ci.thickness = 1.5
            ci.outline_color = _color(1, 0, 0)
            ann.circles.append(ci)
        for (x, y) in self._ov_right:
            ci = CircleAnnotation()
            ci.timestamp = stamp
            ci.position = _pt(x, y)
            ci.diameter = 8.0
            ci.thickness = 1.5
            ci.outline_color = _color(1, 0.4, 0)
            ann.circles.append(ci)

        if self._ov_fit is not None:
            c_near, near_y, c_far, far_y = self._ov_fit
            fit = PointsAnnotation()
            fit.timestamp = stamp
            fit.type = PointsAnnotation.LINE_STRIP
            fit.points = [_pt(c_near, near_y), _pt(c_far, far_y)]
            fit.outline_color = _color(1, 1, 0)
            fit.thickness = 2.0
            ann.points.append(fit)

        if self._ov_msg is not None:
            tx = TextAnnotation()
            tx.timestamp = stamp
            tx.position = _pt(10, y0 - 10)
            tx.text = self._ov_msg
            tx.font_size = 18.0
            tx.text_color = _color(1, 0, 0)
            tx.background_color = _color(0, 0, 0, 0.5)
            ann.texts.append(tx)

        self._annot_pub.publish(ann)


def main(args=None):
    rclpy.init(args=args)
    node = LineDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
