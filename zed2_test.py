#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


# ZED2 LAUNCH KODU : ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i

class Zed2ImageViewer(Node):
    def __init__(self):
        super().__init__('zed2_image_viewer')
        
        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None

        # Subscribe to RGB image
        self.rgb_sub = self.create_subscription(
            Image,
            '/zed/zed_node/rgb_raw/image_raw_color',
            self.rgb_callback,
            10)
        
        # Subscribe to Depth image
        self.depth_sub = self.create_subscription(
            Image,
            '/zed/zed_node/depth/depth_registered',
            self.depth_callback,
            10)

        cv2.namedWindow('ZED2 Camera', cv2.WINDOW_NORMAL)
        self.get_logger().info('ZED2 RGB and Depth Viewer Node Started')

    def rgb_callback(self, msg):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.display_images()
        except Exception as e:
            self.get_logger().error(f"RGB image conversion failed: {e}")

    def depth_callback(self, msg):
        try:
            # Depth image is in 32FC1 encoding (float32, 1 channel)
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f"Depth image conversion failed: {e}")

    def display_images(self):
        if self.rgb_image is None or self.depth_image is None:
            return
        
        height, width, _ = self.rgb_image.shape
        center_x, center_y = width // 2, height // 2
        
        # Get depth at the center pixel
        depth = self.depth_image[center_y, center_x]
        self.get_logger().info(f"Noktanın kameraya uzaklığı = {depth:.2f}")
        # Show depth value on RGB image
        display_image = self.rgb_image.copy()
        cv2.circle(display_image, (center_x, center_y), 5, (0, 255, 0), -1)
        cv2.putText(display_image, f"Depth: {depth:.2f} m", (center_x - 100, center_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # Display RGB image with overlay
        cv2.imshow('ZED2 Camera', display_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = Zed2ImageViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.get_logger().info('Shutting down ZED2 Image Viewer')
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
