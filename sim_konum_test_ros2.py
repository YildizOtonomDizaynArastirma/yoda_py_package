import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf_transformations
import matplotlib.pyplot as plt
import math

class OdomPlotter(Node):
    def __init__(self):
        super().__init__('odom_plotter')

        self.subscription = self.create_subscription(
            Odometry,
            '/yoda/odom',
            self.odom_callback,
            10)

        self.x_positions = []
        self.y_positions = []

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion([qx, qy, qz, qw])

        self.get_logger().info(f"X: {x:.2f}, Y: {y:.2f}, Yaw: {math.degrees(yaw):.2f} derece")

        self.x_positions.append(x)
        self.y_positions.append(y)

    def plot_trajectory(self):
        if len(self.x_positions) == 0 or len(self.y_positions) == 0:
            self.get_logger().warn('Grafik çizilecek veri bulunamadı!')
            return
        plt.figure()
        plt.plot(self.x_positions, self.y_positions)
        plt.xlabel('X Koordinatı')
        plt.ylabel('Y Koordinatı')
        plt.title('-0.5 Radyan Girişi')
        plt.grid(True)
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = OdomPlotter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.plot_trajectory()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

