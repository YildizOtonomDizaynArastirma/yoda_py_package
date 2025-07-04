import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomCovFixer(Node):
    def __init__(self):
        super().__init__('odom_cov_fixer')
        self.sub = self.create_subscription(Odometry, '/yoda/odom', self.odom_callback, 10)
        self.pub = self.create_publisher(Odometry, '/yoda/odom_fixed', 10)
        self.z_cov = 1.0  # Pose Z covariance
        self.rpy_cov = 1.0  # Roll/Pitch covariance
        self.twist_cov = 1.0  # Twist Z and angular

    def odom_callback(self, msg):

        msg.pose.pose.position.z = 0.0

        # Pose covariance
        msg.pose.covariance[14] = self.z_cov  # Z
        msg.pose.covariance[21] = self.rpy_cov  # Roll
        msg.pose.covariance[28] = self.rpy_cov  # Pitch

        # Twist covariance
        msg.twist.covariance[14] = self.twist_cov  # Linear Z
        msg.twist.covariance[21] = self.twist_cov  # Roll velocity
        msg.twist.covariance[28] = self.twist_cov  # Pitch velocity

        msg.header.frame_id = "odom"

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = OdomCovFixer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
