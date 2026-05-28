#!/usr/bin/env python3
"""Deteção de obstáculos por YOLOv8 (ONNX/GPU) no PC. 

Consome imagens (preferencialmente comprimidas) do robô via Wi-Fi e realiza
a inferência usando a RTX 4060 através da biblioteca ultralytics.
Publica Detection2DArray + overlay Foxglove.
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
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from std_msgs.msg import String
from vision_msgs.msg import (
    Detection2D, Detection2DArray, ObjectHypothesisWithPose,
)
from foxglove_msgs.msg import (
    ImageAnnotations, PointsAnnotation, TextAnnotation, Point2, Color,
)

from ultralytics import YOLO


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

        self.declare_parameter('model_dir', '')       # vazio -> share/speedy_vision_pc/models/obstacle_detector
        self.declare_parameter('model_file', 'model.onnx') # ou best.pt
        self.declare_parameter('input_size', 640)      # 640 é o ideal para o PC
        self.declare_parameter('conf_threshold', 0.35)
        self.declare_parameter('nms_threshold', 0.45)
        self.declare_parameter('max_rate_hz', 0.0)     # 0 = sem teto
        self.declare_parameter('image_topic', '/speedy_camera/image_raw')
        self.declare_parameter('state_topic', '/speedy_supervisor/state')
        self.declare_parameter('publish_overlay', True)
        self.declare_parameter('undistort', True)
        self.declare_parameter('use_compressed', True) # No PC, via Wi-Fi, sempre True
        self.declare_parameter('enabled', True)

        model_dir = self.get_parameter('model_dir').value or os.path.join(
            get_package_share_directory('speedy_vision_pc'), 'models', 'obstacle_detector')
        model_path = os.path.join(model_dir, self.get_parameter('model_file').value)

        # Carrega o modelo via Ultralytics (detecta CUDA automaticamente)
        self._model = YOLO(model_path, task='detect')
        self.get_logger().info(f'[obstacle] Modelo ONNX carregado: {model_path}')

        self._input_size = self.get_parameter('input_size').value
        self._conf = self.get_parameter('conf_threshold').value
        self._nms = self.get_parameter('nms_threshold').value
        self._max_rate = self.get_parameter('max_rate_hz').value
        self._overlay = self.get_parameter('publish_overlay').value
        self._undistort = self.get_parameter('undistort').value
        self._undist_maps = None

        self._bridge = CvBridge()
        self._auto = False
        self._enabled = self.get_parameter('enabled').value
        self._lock = threading.Lock()
        self._latest = None
        self._seq = 0
        self._frame_event = threading.Event()

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
        self._img_pub = self.create_publisher(CompressedImage, '/obstacle_detector/image/compressed', 1)

        self.add_on_set_parameters_callback(self._on_params)

        self._stop = False
        self._worker = threading.Thread(target=self._infer_loop, daemon=True)
        self._worker.start()

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
        want = self._auto and self._enabled
        if want and self._image_sub is None:
            topic = self.get_parameter('image_topic').value
            if self.get_parameter('use_compressed').value:
                if not topic.endswith('/compressed'):
                    topic += '/compressed'
                self._image_sub = self.create_subscription(
                    CompressedImage, topic, self._compressed_image_cb, self._sensor_qos)
            else:
                self._image_sub = self.create_subscription(
                    Image, topic, self._image_cb, self._sensor_qos)
            self.get_logger().info(f'[obstacle] Subscrito no tópico: {topic}')
        elif not want and self._image_sub is not None:
            self.destroy_subscription(self._image_sub)
            self._image_sub = None
            with self._lock:
                self._latest = None
            self.get_logger().info('[obstacle] Subscrição de imagem cancelada.')

    def _image_cb(self, msg: Image):
        with self._lock:
            self._latest = msg
            self._seq += 1
        self._frame_event.set()

    def _compressed_image_cb(self, msg: CompressedImage):
        with self._lock:
            self._latest = msg
            self._seq += 1
        self._frame_event.set()

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

            now = time.time()
            if self._max_rate > 0.0:
                remaining = (1.0 / self._max_rate) - (now - last_proc)
                if remaining > 0.0:
                    time.sleep(remaining)
                    continue

            if not self._frame_event.wait(timeout=0.05):
                continue
            self._frame_event.clear()

            with self._lock:
                msg = self._latest
                seq = self._seq
            if msg is None or seq == last_seq:
                continue

            last_seq = seq
            last_proc = now = time.time()

            try:
                if isinstance(msg, CompressedImage):
                    frame = self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
                else:
                    frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as e:
                self.get_logger().error(f'[obstacle] cv_bridge: {e}')
                continue

            if self._undistort and self._undist_maps is not None:
                frame = cv2.remap(frame, self._undist_maps[0], self._undist_maps[1], cv2.INTER_LINEAR)

            t0 = time.time()
            # Inferência via Ultralytics (usa GPU se disponível)
            results = self._model.predict(
                source=frame,
                imgsz=self._input_size,
                conf=self._conf,
                iou=self._nms,
                device=None, # Usa default (CUDA:0 se disponível)
                verbose=False
            )
            dt = time.time() - t0

            if results:
                res = results[0]
                self._publish(msg.header, res)
                if self._overlay and self._annot_pub.get_subscription_count() > 0:
                    self._publish_overlay(msg.header.stamp, res)

            n_inf += 1
            t_acc += dt
            if now - t_log >= 2.0:
                fps = n_inf / (now - t_log)
                self.get_logger().info(
                    f'[obstacle] {fps:.1f} FPS, {1000.0 * t_acc / max(1, n_inf):.1f} ms/inf')
                n_inf = 0
                t_acc = 0.0
                t_log = now

    def _publish(self, header, res):
        arr = Detection2DArray()
        arr.header = header
        
        boxes = res.boxes
        if boxes is not None:
            for i in range(len(boxes)):
                box = boxes[i]
                d = box.xyxy[0].tolist() # x1, y1, x2, y2
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                name = res.names[cls]

                det = Detection2D()
                det.header = header
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = name
                hyp.hypothesis.score = conf
                det.results.append(hyp)
                det.bbox.center.position.x = (d[0] + d[2]) * 0.5
                det.bbox.center.position.y = (d[1] + d[3]) * 0.5
                det.bbox.size_x = abs(d[2] - d[0])
                det.bbox.size_y = abs(d[3] - d[1])
                arr.detections.append(det)
        self._det_pub.publish(arr)

    def _publish_overlay(self, stamp, res):
        ann = ImageAnnotations()
        boxes = res.boxes
        if boxes is not None:
            for i in range(len(boxes)):
                box = boxes[i]
                d = box.xyxy[0].tolist()
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                name = res.names[cls]

                b = PointsAnnotation()
                b.timestamp = stamp
                b.type = PointsAnnotation.LINE_LOOP
                b.points = [_pt(d[0], d[1]), _pt(d[2], d[1]), _pt(d[2], d[3]), _pt(d[0], d[3])]
                b.outline_color = _color(0, 1, 0)
                b.thickness = 2.0
                ann.points.append(b)

                tx = TextAnnotation()
                tx.timestamp = stamp
                tx.position = _pt(d[0], max(0.0, d[1] - 4.0))
                tx.text = f'{name} {conf:.2f}'
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
