#!/usr/bin/env python3
import os
import time
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

        # Parâmetros
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

        # Estado
        self.latest_image = None
        self.camera_info = None
        self.button_pressed = False
        self.last_burst_time = 0
        self.is_manual_mode = False

        # OpenCV
        self.bridge = CvBridge()
        self.map1, self.map2 = None, None

        # Garante que a pasta existe
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)

        # Publishers
        self.info_pub = self.create_publisher(String, '/speedy_dataset/info', 10)

        # Subscriptions
        self.create_subscription(Image, '/speedy_camera/image_raw', self.image_callback, 10)
        self.create_subscription(CameraInfo, '/speedy_camera/camera_info', self.info_callback, 10)
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Monitor de estado do robô (para saber se está em Manual)
        self.create_subscription(String, '/speedy_supervisor/state', self.state_callback, 10)

        self.get_logger().info("[DATASET] Node initialized. Mode: Manual Only.")

    def state_callback(self, msg):
        self.is_manual_mode = (msg.data.upper() == "MANUAL")

    def info_callback(self, msg):
        if self.camera_info is None and self.should_undistort:
            self.camera_info = msg
            # Pré-calcula os mapas de retificação para economizar CPU
            k = np.array(msg.k).reshape((3, 3))
            d = np.array(msg.d)
            w, h = msg.width, msg.height
            new_k, _ = cv2.getOptimalNewCameraMatrix(k, d, (w, h), 1, (w, h))
            self.map1, self.map2 = cv2.initUndistortRectifyMap(k, d, None, new_k, (w, h), cv2.CV_32FC1)
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
            cv_img = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
            
            # Aplica retificação se os mapas estiverem prontos
            if self.should_undistort and self.map1 is not None:
                cv_img = cv2.remap(cv_img, self.map1, self.map2, cv2.INTER_LINEAR)

            # Tons de Cinza (BW)
            if self.should_grayscale:
                cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            filename = f"speedy_{mode}_{timestamp}.jpg"
            full_path = os.path.join(self.save_path, filename)

            cv2.imwrite(full_path, cv_img)
            self.get_logger().info(f"[DATASET] Photo saved (Rectified): {filename}")

            # Contar fotos atuais na pasta (apenas .jpg)
            num_photos = len([f for f in os.listdir(self.save_path) if f.endswith('.jpg')])
            
            # Publicar info
            info_msg = String()
            info_dict = {
                "latest_photo": filename,
                "timestamp": datetime.now().strftime("%d/%m/%Y %H:%M:%S"),
                "total_photos": num_photos
            }
            info_msg.data = json.dumps(info_dict)
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
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
