#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class LidarTest(Node):
    def __init__(self):
        super().__init__('lidar_detection_node')
        self.subscription = self.create_subscription(
            LaserScan, 
            '/yoda/lidar/scan',  # Lidar verisi genellikle '/scan' olarak gelir
            self.callback, 
            10
        )
        self.subscription  # Alınan veriyi saklamaya gerek yok, ROS kendisi halleder.
        self.get_logger().info('Lidar Test Düğümü Başlatıldı: Engel verisi dinleniyor...')

    def callback(self, msg):
        min_distance = min(msg.ranges)  # En yakın mesafeyi bul
        threshold = 4.0  # Engel algılama mesafesi eşiği (4 metre)
        # Mesafe verilerini al
        if min_distance < threshold and min_distance > 0:  # 0 değeri sensör hatasıdır
            self.get_logger().info('Engel tespit edildi! Mesafe: {:.2f} m'.format(min_distance))
        else:
            self.get_logger().info('Engel yok. En yakın mesafe: {:.2f} m'.format(min_distance))

def main(args=None):
    rclpy.init(args=args)
    node = LidarTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
