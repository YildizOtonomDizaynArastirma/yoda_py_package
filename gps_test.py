#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class GpsTest(Node):
    def __init__(self):
        super().__init__('gps_test_node')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/yoda/gps/fix',  # GPS verisinin topic adı
            self.callback,
            10
        )
        self.get_logger().info('GPS Test Düğümü Başlatıldı: GPS verisi dinleniyor...')

    def callback(self, msg):
        # Enlem, boylam ve yükseklik bilgilerini al
        latitude = msg.latitude
        longitude = msg.longitude
        altitude = msg.altitude
        
        # Mesajı loglayarak çıktı al
        self.get_logger().info(
            f'GPS Konumu: Enlem={latitude:.6f}, Boylam={longitude:.6f}, Yükseklik={altitude:.2f} m'
        )

def main(args=None):
    rclpy.init(args=args)
    node = GpsTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Düğüm kapatıldı.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
