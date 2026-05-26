#!/usr/bin/env python3
"""Deteção de obstáculos por YOLOv8 (NCNN). Só ativo em AUTO; publica telemetria.

A inferência (pesada no Pi4) corre numa thread dedicada que consome sempre o frame
mais recente — descarta frames atrasados em vez de os enfileirar, para a latência não
crescer. Não toca no controlo: por agora só publica Detection2DArray + overlay Foxglove.
"""
import os
import time
import threading

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rcl_interfaces.msg import SetParametersResult
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from vision_msgs.msg import (
    Detection2D, Detection2DArray, ObjectHypothesisWithPose,
)
from foxglove_msgs.msg import (
    ImageAnnotations, PointsAnnotation, TextAnnotation, Point2, Color,
)

from speedy_vision.yolo_ncnn import YoloNcnn


def _color(r, g, b, a=1.0):
    c = Color()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
    return c


def _pt(x, y):
    p = Point2()
    p.x, p.y = float(x), float(y)
    return p


class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        self.declare_parameter('model_dir', '')       # vazio -> share/speedy_vision/models/obstacle_detector
        self.declare_parameter('param_file', 'model.ncnn.param')
        self.declare_parameter('bin_file', 'model.ncnn.bin')
        self.declare_parameter('input_size', 0)        # 0 -> usa imgsz do metadata.yaml
        self.declare_parameter('num_threads', 2)
        self.declare_parameter('conf_threshold', 0.35)
        self.declare_parameter('nms_threshold', 0.45)
        self.declare_parameter('class_names', [''])    # vazio -> usa names do metadata.yaml
        self.declare_parameter('max_rate_hz', 0.0)     # 0 = sem teto (deixa a inferência ditar)
        self.declare_parameter('image_topic', '/speedy_camera/image_raw')
        self.declare_parameter('state_topic', '/speedy_supervisor/state')
        self.declare_parameter('publish_overlay', True)
        self.declare_parameter('undistort', True)
        self.declare_parameter('grayscale', True)
        self.declare_parameter('enabled', True)               # False = congela inferência sem matar o nó

        model_dir = self.get_parameter('model_dir').value or os.path.join(
            get_package_share_directory('speedy_vision'), 'models', 'obstacle_detector')
        param_path = os.path.join(model_dir, self.get_parameter('param_file').value)
        bin_path = os.path.join(model_dir, self.get_parameter('bin_file').value)

        input_size = self.get_parameter('input_size').value
        names_param = [n for n in self.get_parameter('class_names').value if n]
        meta = self._load_metadata(model_dir)
        if input_size <= 0:
            imgsz = meta.get('imgsz', [320, 320])
            input_size = int(imgsz[0] if isinstance(imgsz, (list, tuple)) else imgsz)
        if names_param:
            self._names = {i: n for i, n in enumerate(names_param)}
        else:
            self._names = {int(k): v for k, v in meta.get('names', {}).items()}

        self._conf = self.get_parameter('conf_threshold').value
        self._nms = self.get_parameter('nms_threshold').value
        self._max_rate = self.get_parameter('max_rate_hz').value
        self._overlay = self.get_parameter('publish_overlay').value
        self._undistort = self.get_parameter('undistort').value
        self._grayscale = self.get_parameter('grayscale').value
        self._undist_maps = None

        self._model = YoloNcnn(
            param_path, bin_path, input_size=input_size,
            num_threads=self.get_parameter('num_threads').value,
        )
        self.get_logger().info(
            f'[obstacle] Modelo carregado ({input_size}px, classes={list(self._names.values())}).')

        self._bridge = CvBridge()
        self._auto = False
        self._enabled = self.get_parameter('enabled').value
        self._lock = threading.Lock()
        self._latest = None
        self._seq = 0

        self._sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._image_sub = None
        self.create_subscription(CameraInfo, '/speedy_camera/camera_info', self._info_cb, self._sensor_qos)
        self.create_subscription(String, self.get_parameter('state_topic').value,
                                 self._state_cb, 10)
        self._det_pub = self.create_publisher(Detection2DArray, '/obstacle_detector/detections', 10)
        self._annot_pub = self.create_publisher(ImageAnnotations, '/obstacle_detector/annotations', 1)

        # Reage a `enabled` em runtime: desligar larga a subscrição da imagem crua
        # (não basta saltar a inferência — o transporte da imagem é o custo no Pi4).
        self.add_on_set_parameters_callback(self._on_params)

        self._stop = False
        self._worker = threading.Thread(target=self._infer_loop, daemon=True)
        self._worker.start()

    def _load_metadata(self, model_dir):
        path = os.path.join(model_dir, 'metadata.yaml')
        try:
            with open(path, 'r') as f:
                return yaml.safe_load(f) or {}
        except FileNotFoundError:
            self.get_logger().warn(f'[obstacle] metadata.yaml não encontrado em {model_dir}.')
            return {}

    def _info_cb(self, msg: CameraInfo):
        if self._undist_maps is not None or not self._undistort:
            return
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        D = np.array(msg.d, dtype=np.float64)
        new_K = np.array(msg.p, dtype=np.float64).reshape(3, 4)[:, :3]
        self._undist_maps = cv2.initUndistortRectifyMap(
            K, D, None, new_K, (msg.width, msg.height), cv2.CV_16SC2
        )
        self.get_logger().info('[obstacle] Mapa de undistorção inicializado.')

    def _state_cb(self, msg: String):
        self._auto = (msg.data.upper() == 'AUTO')
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
            self._image_sub = self.create_subscription(
                Image, self.get_parameter('image_topic').value,
                self._image_cb, self._sensor_qos)
            self.get_logger().info('[obstacle] Subscrito no tópico de imagem.')
        elif not want and self._image_sub is not None:
            self.destroy_subscription(self._image_sub)
            self._image_sub = None
            with self._lock:
                self._latest = None
            self.get_logger().info('[obstacle] Subscrição de imagem cancelada (poupar CPU/banda).')

    def _image_cb(self, msg: Image):
        with self._lock:
            self._latest = msg
            self._seq += 1

    def _infer_loop(self):
        last_seq = -1
        last_proc = 0.0
        n_inf = 0
        t_acc = 0.0
        t_log = time.time()
        while not self._stop:
            if not self._auto or not self._enabled:
                time.sleep(0.1)
                continue
            with self._lock:
                msg = self._latest
                seq = self._seq
            if msg is None or seq == last_seq:
                time.sleep(0.005)
                continue
            now = time.time()
            if self._max_rate > 0.0 and (now - last_proc) < (1.0 / self._max_rate):
                time.sleep(0.005)
                continue
            last_seq = seq
            last_proc = now

            try:
                bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as e:
                self.get_logger().error(f'[obstacle] cv_bridge: {e}')
                continue

            if self._undistort and self._undist_maps is not None:
                bgr = cv2.remap(bgr, self._undist_maps[0], self._undist_maps[1], cv2.INTER_LINEAR)

            if self._grayscale:
                gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
                bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

            t0 = time.time()
            dets = self._model.infer(bgr, self._conf, self._nms)
            dt = time.time() - t0

            self._publish(msg.header, dets)
            if self._overlay and self._annot_pub.get_subscription_count() > 0:
                self._publish_overlay(msg.header.stamp, dets)

            n_inf += 1
            t_acc += dt
            if now - t_log >= 2.0:
                fps = n_inf / (now - t_log)
                self.get_logger().info(
                    f'[obstacle] {fps:.1f} FPS, {1000.0 * t_acc / max(1, n_inf):.0f} ms/inf, '
                    f'{len(dets)} obj')
                n_inf = 0
                t_acc = 0.0
                t_log = now

    def _publish(self, header, dets):
        arr = Detection2DArray()
        arr.header = header
        for d in dets:
            det = Detection2D()
            det.header = header
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = self._names.get(d.cls, str(d.cls))
            hyp.hypothesis.score = d.score
            det.results.append(hyp)
            det.bbox.center.position.x = (d.x1 + d.x2) * 0.5
            det.bbox.center.position.y = (d.y1 + d.y2) * 0.5
            det.bbox.center.theta = 0.0
            det.bbox.size_x = abs(d.x2 - d.x1)
            det.bbox.size_y = abs(d.y2 - d.y1)
            arr.detections.append(det)
        self._det_pub.publish(arr)

    def _publish_overlay(self, stamp, dets):
        ann = ImageAnnotations()
        for d in dets:
            box = PointsAnnotation()
            box.timestamp = stamp
            box.type = PointsAnnotation.LINE_LOOP
            box.points = [_pt(d.x1, d.y1), _pt(d.x2, d.y1), _pt(d.x2, d.y2), _pt(d.x1, d.y2)]
            box.outline_color = _color(0, 1, 0)
            box.thickness = 2.0
            ann.points.append(box)

            tx = TextAnnotation()
            tx.timestamp = stamp
            tx.position = _pt(d.x1, max(0.0, d.y1 - 4.0))
            tx.text = f'{self._names.get(d.cls, d.cls)} {d.score:.2f}'
            tx.font_size = 16.0
            tx.text_color = _color(1, 1, 1)
            tx.background_color = _color(0, 0.5, 0, 0.6)
            ann.texts.append(tx)
        self._annot_pub.publish(ann)

    def destroy_node(self):
        self._stop = True
        if self._worker.is_alive():
            self._worker.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
