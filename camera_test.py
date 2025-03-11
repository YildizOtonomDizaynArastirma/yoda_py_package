#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraTest(Node):
    def __init__(self):
        super().__init__('camera_test_node')
        self.subscription = self.create_subscription(
            Image,
            '/yoda/camera/image_raw',  # Kamera verisinin topic adı
            self.callback,
            10
        )
        self.bridge = CvBridge()  # ROS görüntü mesajlarını OpenCV formatına çevirmek için
        self.get_logger().info('Kamera Test Düğümü Başlatıldı: Görüntü alınıyor...')

        # Tek bir pencere oluştur
        cv2.namedWindow('Kamera Görüntüsü', cv2.WINDOW_NORMAL)

    def callback(self, msg):
        try:
            # ROS Image mesajını OpenCV formatına çevir
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Görüntüyü tek pencerede güncelle
            cv2.imshow('Kamera Görüntüsü', cv_image)
            
            # 1 ms bekleyerek pencereyi güncelle (ESC tuşuna basıldığında pencereyi kapatır)
            if cv2.waitKey(1) & 0xFF == 27:  # ESC tuşu
                self.get_logger().info('Kamera Test Düğümü Kapatılıyor...')
                rclpy.shutdown()
        except Exception as e:
            self.get_logger().error(f'Görüntü işleme hatası: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CameraTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Düğüm kapatıldı.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()  # OpenCV penceresini düzgünce kapat

if __name__ == '__main__':
    main()
