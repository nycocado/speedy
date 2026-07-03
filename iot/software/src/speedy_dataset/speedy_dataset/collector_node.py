#!/usr/bin/env python3
import os
import time
import queue
import threading
import cv2
import json
import numpy as np
from datetime import datetime
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Joy, CameraInfo
from std_msgs.msg import String


class DatasetCollector(Node):
    def __init__(self):
        super().__init__('dataset_collector')

        self.declare_parameter('save_path', '~/dataset')
        self.declare_parameter('burst_period', 0.5)
        self.declare_parameter('button_index', 3)
        self.declare_parameter('undistort', True)
        self.declare_parameter('grayscale', False)

        self.save_path = os.path.expanduser(self.get_parameter('save_path').value)
        self.burst_period = self.get_parameter('burst_period').value
        self.button_index = self.get_parameter('button_index').value
        self.should_undistort = self.get_parameter('undistort').value
        self.should_grayscale = self.get_parameter('grayscale').value

        self.latest_image = None
        self.camera_info = None
        self.button_pressed = False
        self.last_burst_time = 0
        self.is_manual_mode = False

        self.bridge = CvBridge()
        self.map1, self.map2 = None, None

        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)

        self.photo_count = len([f for f in os.listdir(self.save_path) if f.endswith('.jpg')])

        self.save_queue = queue.Queue(maxsize=100)
        self.stop_worker = False
        self.worker = threading.Thread(target=self._writer_loop, daemon=True)
        self.worker.start()

        self.info_pub = self.create_publisher(String, '/speedy_dataset/info', 10)

        self._image_sub = None
        self.create_subscription(CameraInfo, '/speedy_camera/camera_info', self.info_callback, 10)
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.create_subscription(String, '/speedy_supervisor/state', self.state_callback, 10)

        self.get_logger().info("[DATASET] Node initialized. Mode: Manual Only.")

    def state_callback(self, msg):
        was_manual = self.is_manual_mode
        self.is_manual_mode = (msg.data.upper() == "MANUAL")

        if self.is_manual_mode and not was_manual:
            self._image_sub = self.create_subscription(
                Image, '/speedy_camera/image_raw', self.image_callback, 10)
            self.get_logger().info('[DATASET] Modo MANUAL: subscrito no /image_raw.')
        elif not self.is_manual_mode and was_manual:
            if self._image_sub is not None:
                self.destroy_subscription(self._image_sub)
                self._image_sub = None
                self.get_logger().info(
                    '[DATASET] Modo AUTO: cancelando subscrição do /image_raw para poupar CPU.')

    def info_callback(self, msg):
        if self.camera_info is None and self.should_undistort:
            self.camera_info = msg
            k = np.array(msg.k).reshape((3, 3))
            d = np.array(msg.d)
            w, h = msg.width, msg.height
            new_k, _ = cv2.getOptimalNewCameraMatrix(k, d, (w, h), 1, (w, h))
            self.map1, self.map2 = cv2.initUndistortRectifyMap(
                k, d, None, new_k, (w, h), cv2.CV_32FC1)
            self.get_logger().info("[DATASET] Rectification maps calculated successfully.")

    def image_callback(self, msg):
        self.latest_image = msg

    def joy_callback(self, msg):
        if not self.is_manual_mode:
            return

        if len(msg.buttons) <= self.button_index:
            return

        is_now_pressed = msg.buttons[self.button_index] == 1

        if is_now_pressed and not self.button_pressed:
            self.save_photo("single")

        if is_now_pressed:
            now = time.time()
            if (now - self.last_burst_time) >= self.burst_period:
                if self.button_pressed:
                    self.save_photo("burst")
                self.last_burst_time = now

        self.button_pressed = is_now_pressed

    def save_photo(self, mode):
        if self.latest_image is None:
            return
        try:
            self.save_queue.put_nowait((mode, self.latest_image))
        except queue.Full:
            self.get_logger().warn("[DATASET] Fila de escrita cheia — frame descartado.")

    def _writer_loop(self):
        while not self.stop_worker:
            try:
                mode, img_msg = self.save_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

                if self.should_undistort and self.map1 is not None:
                    cv_img = cv2.remap(cv_img, self.map1, self.map2, cv2.INTER_LINEAR)
                if self.should_grayscale:
                    cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                filename = f"speedy_{mode}_{timestamp}.jpg"
                cv2.imwrite(os.path.join(self.save_path, filename), cv_img)
                self.photo_count += 1
                self.get_logger().info(f"[DATASET] Photo saved: {filename}")

                info_msg = String()
                info_msg.data = json.dumps({
                    "latest_photo": filename,
                    "timestamp": datetime.now().strftime("%d/%m/%Y %H:%M:%S"),
                    "total_photos": self.photo_count
                })
                self.info_pub.publish(info_msg)
            except Exception as e:
                self.get_logger().error(f"[DATASET] Error saving image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DatasetCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_worker = True
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
