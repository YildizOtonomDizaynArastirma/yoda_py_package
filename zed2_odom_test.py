import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math


RAD2DEG = 57.295779513


class MinimalPoseOdomSubscriber(Node):

    def __init__(self):
        super().__init__('zed_odom_pose_tutorial')

        # QoS: reliable + volatile to match default ZED QoS settings
        qos_profile = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/zed/zed_node/pose',
            self.pose_callback,
            qos_profile
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/zed/zed_node/odom',
            self.odom_callback,
            qos_profile
        )
    def euler_from_quaternion(self,quat):
        x, y, z, w = quat
        t0 = +2.0*(w*x + y*z)
        t1 = +1.0 - 2.0*(x*x + y*y)
        roll = math.atan2(t0, t1)
        t2 = +2.0*(w*y - z*x)
        t2 = max(-1.0, min(1.0, t2))
        pitch = math.asin(t2)
        t3 = +2.0*(w*z + x*y)
        t4 = +1.0 - 2.0*(y*y + z*z)
        yaw = math.atan2(t3, t4)
        return roll, pitch, yaw


    def pose_callback(self, msg: PoseStamped):
        tx = msg.pose.position.x
        ty = msg.pose.position.y
        tz = msg.pose.position.z

        q = msg.pose.orientation
        orientation_q = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = self.euler_from_quaternion(orientation_q)
        """
        self.get_logger().info(
            f"Received pose in '{msg.header.frame_id}' frame : "
            f"X: {tx:.2f} Y: {ty:.2f} Z: {tz:.2f} - "
            f"R: {roll * RAD2DEG:.2f} P: {pitch * RAD2DEG:.2f} Y: {yaw * RAD2DEG:.2f} - "
            f"Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec} sec"
        )
        """
    def odom_callback(self, msg: Odometry):
        tx = msg.pose.pose.position.x
        ty = msg.pose.pose.position.y
        tz = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        orientation_q = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = self.euler_from_quaternion(orientation_q)

        self.get_logger().info(
            f"Received odometry in '{msg.header.frame_id}' frame : "
            f"X: {tx:.2f} Y: {ty:.2f} Z: {tz:.2f} - "
            f"R: {roll * RAD2DEG:.2f} P: {pitch * RAD2DEG:.2f} Y: {yaw * RAD2DEG:.2f} - "
            f"Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec} sec"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MinimalPoseOdomSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
