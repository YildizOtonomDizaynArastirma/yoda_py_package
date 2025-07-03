#!/usr/bin/env python3


from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException


class LaneFollowerNode(Node):
    def __init__(self):
        super().__init__('lane_follower')

        self.subscription = self.create_subscription(
            Float32,
            '/lane_offset',  
            self.lane_center_callback,
            10)
        """self.subscription2 = self.create_subscription(
            Float32,
            '/lane_curve',  
            self.lane_curve_callback,
            10)
        """
        self.publisher = self.create_publisher(Twist, '/yoda/cmd_vel', 10)
    
    def lane_center_callback(self, msg):
        lane_center = msg.data
        #self.get_logger().info(f"Received lane center: {lane_center}")

        twist = Twist()

        twist.linear.x = 1.0

        twist.angular.z = -lane_center 

        self.publisher.publish(twist)
    
    """
    def lane_curve_callback(self, msg):
        lane_curve = msg.data
        lane_curve = lane_curve*30/1080
        linear_speed = 1.0
        angular_speed = 0.0
        if lane_curve == 0:
            angular_speed = 0.0
        else:
            angular_speed = -5.0*(linear_speed / lane_curve)
        self.get_logger().info(f"Received lane curve: {lane_curve}")
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self.publisher.publish(twist)
    """
       
    
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