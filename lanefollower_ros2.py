#!/usr/bin/env python3

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

def map_value(x, in_min, in_max, out_min, out_max):
    # Clamp x to the range [in_min, in_max]
    if x < in_min:
        x = in_min
    elif x > in_max:
        x = in_max
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

class LaneFollowerNode(Node):
    def __init__(self):
        super().__init__('lane_follower')

        self.subscription = self.create_subscription(
            Float32,
            '/lane_offset',  # metre cinsinden: +sol, -sağ
            self.lane_center_callback,
            10)

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
    
    def lane_center_callback(self, msg):
        offset_meters = msg.data  # örn: -0.20 → sağda, +0.15 → solda

        twist = Twist()
        twist.linear.x = 80.0

        # Açısal hız aralığına (örneğin 100–440) map et
        angular_z = map_value(offset_meters, -0.36, 0.32, 100.0, 440.0)

        twist.angular.z = angular_z

        self.get_logger().info(
            f"Offset: {offset_meters:.3f} m → Angular Z: {angular_z:.2f}"
        )

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

