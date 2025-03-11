#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuTest(Node):
    def __init__(self):
        super().__init__('imu_test_node')
        self.subscription = self.create_subscription(
            Imu,
            '/yoda/imu/data',  # IMU verisinin topic adı
            self.callback,
            10
        )
        self.get_logger().info('IMU Test Düğümü Başlatıldı: IMU verisi dinleniyor...')

    def callback(self, msg):
        # İvme verilerini al
        accel_x = msg.linear_acceleration.x
        accel_y = msg.linear_acceleration.y
        accel_z = msg.linear_acceleration.z
        
        # Jiroskop verilerini al
        gyro_x = msg.angular_velocity.x
        gyro_y = msg.angular_velocity.y
        gyro_z = msg.angular_velocity.z
        
        # Mesajı loglayarak çıktı al
        self.get_logger().info(
            f'İvme: x={accel_x:.2f}, y={accel_y:.2f}, z={accel_z:.2f} | '
            f'Jiroskop: x={gyro_x:.2f}, y={gyro_y:.2f}, z={gyro_z:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = ImuTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Düğüm kapatıldı.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
