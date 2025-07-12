import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class EngelZamanKontrol(Node):
    def __init__(self):
        super().__init__('engel_zaman_kontrol')
        self.cmd_vel_pub = self.create_publisher(Twist, '/engel/cmd_vel', 10)
        self.timer_period = 0.1  # 10 Hz
        self.move_duration = 10.0  # saniye, 4m / 0.5 m/s
        self.elapsed_time = 0.0

        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info("Engel hareketine başladı")

    def timer_callback(self):
        if self.elapsed_time < self.move_duration:
            twist = Twist()
            twist.linear.x = 0.5
            self.cmd_vel_pub.publish(twist)
            self.elapsed_time += self.timer_period
        else:
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info("Engel durdu.")
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = EngelZamanKontrol()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
