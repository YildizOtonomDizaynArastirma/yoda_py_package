#!/usr/bin/env python3
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

class VehicleSpeed(Node):
    def __init__(self):
        super().__init__('vehicle_speed_node')
        self.publisher = self.create_publisher(Twist, '/yoda/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.send_velocity_commands)  # 0.1 saniyelik aralıkla hız komutları

    def send_velocity_commands(self):
        # Basit bir hız komutu
        twist = Twist()
        twist.linear.x = 0.1  # İleri hız (m/s)
        twist.angular.z = 10.0  # Sağ dönüş hızı (rad/s)
        self.publisher.publish(twist)
        self.get_logger().info('Hareket komutu gönderildi!!')

def main(args=None):
    rclpy.init(args=args)
    node = VehicleSpeed()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
