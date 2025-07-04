#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np
import math
from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class PointCloud2ToLaserScan(Node):
    """
    ROS2 Node that converts PointCloud2 messages to LaserScan messages.
    """
    def __init__(self):
        super().__init__('pointcloud_to_laserscan')
                
        # Declare parameters with descriptions
        self.declare_parameter(
            'angle_min', -math.pi,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Minimum angle of the LaserScan [rad]')
        )
        self.declare_parameter(
            'angle_max', math.pi,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Maximum angle of the LaserScan [rad]')
        )
        self.declare_parameter(
            'angle_increment', 0.005,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Angular resolution of the LaserScan [rad]')
        )
        self.declare_parameter(
            'range_min', 0.1,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Minimum range of the LaserScan [m]')
        )
        self.declare_parameter(
            'range_max', 50.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Maximum range of the LaserScan [m]')
        )
        self.declare_parameter(
            'z_min', -3,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Minimum z value to include in scan [m]')
        )
        self.declare_parameter(
            'z_max', 3,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Maximum z value to include in scan [m]')
        )
        self.declare_parameter(
            'min_height', -1.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Minimum height of points to include [m]')
        )
        self.declare_parameter(
            'max_height', 1.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Maximum height of points to include [m]')
        )
        self.declare_parameter(
            'pointcloud_topic', '/yoda/lidar/points',
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Topic name for the input PointCloud2')
        )
        self.declare_parameter(
            'laserscan_topic', '/yoda/lidar/scan',
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Topic name for the output LaserScan')
        )
        self.declare_parameter(
            'target_frame', 'lidar_link',
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Frame ID for the LaserScan message')
        )
        
        # Get parameters
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.angle_increment = self.get_parameter('angle_increment').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.z_min = self.get_parameter('z_min').value
        self.z_max = self.get_parameter('z_max').value
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        pointcloud_topic = self.get_parameter('pointcloud_topic').value
        laserscan_topic = self.get_parameter('laserscan_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.use_sim_time = True
        
        # Calculate number of points in the scan
        self.num_points = int(round((self.angle_max - self.angle_min) / self.angle_increment) + 1)
        
        # Create publisher and subscriber
        self.laserscan_pub = self.create_publisher(LaserScan, laserscan_topic, 10)
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            pointcloud_topic,
            self.pointcloud_callback,
            10
        )
        
        self.get_logger().info(f'PointCloud2 to LaserScan node initialized')
        self.get_logger().info(f'Subscribed to: {pointcloud_topic}')
        self.get_logger().info(f'Publishing to: {laserscan_topic}')
        self.get_logger().info(f'Field of view: {self.angle_min:.2f} to {self.angle_max:.2f} rad')
        self.get_logger().info(f'Number of points in scan: {self.num_points}')
    
    def pointcloud_callback(self, msg):
        """
        Callback function for PointCloud2 messages.
        Converts the pointcloud to a LaserScan message.
        
        :param msg: The PointCloud2 message
        """
        try:
            self.get_logger().debug(f'Received PointCloud2 with {msg.width * msg.height} points')
            # Initialize LaserScan message
            scan_msg = LaserScan()
            scan_msg.header = msg.header
            scan_msg.header.frame_id = self.target_frame
            scan_msg.angle_min = self.angle_min
            scan_msg.angle_max = self.angle_max
            scan_msg.angle_increment = self.angle_increment
            scan_msg.time_increment = 0.0
            scan_msg.scan_time = 0.1
            scan_msg.range_min = self.range_min
            scan_msg.range_max = self.range_max
            
            # Initialize ranges with infinity
            scan_msg.ranges = [float('inf')] * self.num_points
            # Initialize intensities with zeros (optional)
            scan_msg.intensities = [0.0] * self.num_points
            
            # Process pointcloud data
            # Check which fields are available in the point cloud
            available_fields = [field.name for field in msg.fields]
            field_names = ["x", "y", "z"]
            if "intensity" in available_fields:
                field_names.append("intensity")
            
            self.get_logger().debug(f'Available fields: {available_fields}, using: {field_names}')
            
            # Read points with only available fields
            points = list(pc2.read_points(msg, field_names=field_names, skip_nans=True))
            
            for point in points:
                # Handle the point data based on what's available
                x = point[0]
                y = point[1]
                z = point[2]
                
                # Skip points outside the height range
                if z < self.min_height or z > self.max_height:
                    continue
                
                # Calculate distance and angle in the horizontal plane
                distance = math.sqrt(x*x + y*y)
                
                # Skip points outside the range limits
                if distance < self.range_min or distance > self.range_max:
                    continue
                
                angle = math.atan2(y, x)
                
                # Skip points outside the angle range
                if angle < self.angle_min or angle > self.angle_max:
                    continue
                
                # Find the index in the LaserScan ranges array
                index = int((angle - self.angle_min) / self.angle_increment)
                
                # Ensure the index is valid
                if 0 <= index < self.num_points:
                    # Only update if the new range is smaller
                    if distance < scan_msg.ranges[index]:
                        scan_msg.ranges[index] = distance
                        # If intensity is available, use it
                        if len(field_names) > 3 and len(point) > 3:
                            scan_msg.intensities[index] = float(point[3])
            
            # Publish the LaserScan message
            self.laserscan_pub.publish(scan_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing pointcloud: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = PointCloud2ToLaserScan()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
