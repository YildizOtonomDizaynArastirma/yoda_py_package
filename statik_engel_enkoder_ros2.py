#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import Imu
import numpy as np
import math
from scipy.spatial.transform import Rotation as R


class IMUEncoderFusionNavigator(Node):
    def __init__(self):
        super().__init__('imu_encoder_fusion_navigator')

        # Tuning parameters
        self.max_speed         = 0.5   # m/s
        self.min_speed         = 0.3   # m/s
        self.speed_gain        = 1.0    # v = gain * distance
        self.angular_gain      = 0.75    # w = gain * angular error
        self.turn_slowdown     = 3*math.pi/4  # rad at which linear=0
        self.reach_tolerance   = 0.4    # meters
        self.angular_tolerance = 0.05   # radians

        # Motor command value ranges
        self.angular_center = 270.0      # Center value (straight)
        self.angular_range = 170.0       # ±170 → 100 to 440 range
        self.linear_scale = 150.0         # Linear speed multiplier
        self.linear_offset = 0.0         # Linear speed offset

        # Odometry fusion parameters
        self.imu_weight = 1.0  # Use IMU fully for angular velocity since we have single encoder
        self.velocity_smoothing = 0.8  # Smoothing factor for velocity estimation

        # Current pose (fused from IMU + encoders)
        self.current_x   = 0.0
        self.current_y   = 0.0
        self.current_yaw = 0.0
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0

        # Previous values for integration
        self.prev_time = None
        self.prev_encoder_distance = 0.0
        self.smoothed_linear_vel = 0.0

        # IMU data
        self.imu_angular_vel_z = 0.0
        self.imu_linear_acc_x = 0.0
        self.imu_linear_acc_y = 0.0

        # Encoder data
        self.encoder_distance = 0.0  # Total distance traveled

        # Left lane obstacle avoidance maneuver
        x_add = [0.5, 1.0, 1.7, 2.4, 3.0, 5.0, 6.0, 6.7, 7.4, 8.1, 8.8, 9.5, 10.2, 10.8, 11.5, 12.0, 13.0, 14.0]  # Forward progression
        y_add = [ 0.05, 0.3, 0.8, 1.4, 1.7, 2.0, 2.2, 1.9, 1.7, 1.4, 1.0, 0.6, 0.3, 0.05, 0.0, 0.0,   0.0,  0.0]      # Left offset, then return to center
        
        
        self.wp_offsets = [(dx, dy) for dx, dy in zip(x_add, y_add)]

        # Waypoint state
        self.waypoints = []
        self.wp_index  = 0
        self.wp_active = False

        # QoS profile
        qos_profile = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.wp_pub      = self.create_publisher(Bool,  '/engel/waypoint', 10)
        self.odom_pub    = self.create_publisher(Odometry, '/fused_odom', 10)

        # Subscribers
        self.create_subscription(Imu, '/zed/zed_node/imu/data', self.imu_callback, qos_profile)
        
        # Single encoder subscriber
        self.create_subscription(Float64, '/encoder_pub', self.encoder_callback, 10)

        # Timer for odometry fusion and control
        self.timer=self.create_timer(0.02, self.fusion_timer_callback)  # 50 Hz

        self.get_logger().info("IMU-Encoder fusion navigation node started.")
        
        # Initialize pose after a short delay to ensure subscriptions are active
        self.create_timer(1.0, self.initialize_waypoints)

    def imu_callback(self, msg: Imu):
        """Process IMU data"""
        self.imu_angular_vel_z = msg.angular_velocity.z
        self.imu_linear_acc_x = msg.linear_acceleration.x
        self.imu_linear_acc_y = msg.linear_acceleration.y

    def encoder_callback(self, msg: Float64):
        """Process single encoder data (distance traveled)"""
        self.encoder_distance = msg.data

    def fusion_timer_callback(self):
        """Main fusion and control loop"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_encoder_distance = self.encoder_distance
            return

        dt = current_time - self.prev_time
        if dt <= 0:
            return

        # Calculate linear velocity from encoder distance change
        distance_delta = self.encoder_distance - self.prev_encoder_distance
        raw_linear_vel = distance_delta / dt
        
        # Apply smoothing to reduce noise
        self.smoothed_linear_vel = (self.velocity_smoothing * self.smoothed_linear_vel + 
                                   (1 - self.velocity_smoothing) * raw_linear_vel)

        # Use IMU for angular velocity (more reliable for turning)
        fused_angular_vel = self.imu_angular_vel_z
        
        # Use smoothed encoder velocity for linear motion
        fused_linear_vel = self.smoothed_linear_vel

        # Update current velocities
        self.current_linear_vel = fused_linear_vel
        self.current_angular_vel = fused_angular_vel

        # Integrate to get pose
        self.current_yaw += fused_angular_vel * dt
        self.current_yaw = self.normalize_angle(self.current_yaw)

        # Update position using current heading
        dx = fused_linear_vel * math.cos(self.current_yaw) * dt
        dy = fused_linear_vel * math.sin(self.current_yaw) * dt
        self.current_x += dx
        self.current_y += dy

        # Publish fused odometry
        self.publish_fused_odometry(current_time)

        # Update previous values
        self.prev_time = current_time
        self.prev_encoder_distance = self.encoder_distance

        # Control logic
        if self.wp_active:
            self.move_toward_waypoint()

    def publish_fused_odometry(self, timestamp):
        """Publish the fused odometry data"""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Position
        odom_msg.pose.pose.position.x = self.current_x
        odom_msg.pose.pose.position.y = self.current_y
        odom_msg.pose.pose.position.z = 0.0

        # Orientation (quaternion from yaw)
        quat = self.yaw_to_quaternion(self.current_yaw)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        # Velocities
        odom_msg.twist.twist.linear.x = self.current_linear_vel
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = self.current_angular_vel

        self.odom_pub.publish(odom_msg)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion [x, y, z, w]"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return [0.0, 0.0, sy, cy]

    def initialize_waypoints(self):
        """Initialize waypoints after sensors are ready"""
        self.waypoint_create()
        self.destroy_timer(self.timer)  # Remove the initialization timer

    def waypoint_create(self):
        """Create waypoints based on current pose"""
        yaw = self.current_yaw
        self.waypoints = []
        for dx_local, dy_local in self.wp_offsets:
            # Rotate local offset by current yaw
            global_dx = dx_local*math.cos(yaw) - dy_local*math.sin(yaw)
            global_dy = dx_local*math.sin(yaw) + dy_local*math.cos(yaw)
            wx = self.current_x + global_dx
            wy = self.current_y + global_dy
            self.waypoints.append((wx, wy))

        self.wp_index  = 0
        self.wp_active = True
        tx, ty = self.waypoints[0]
        self.get_logger().info(f"Current position: ({self.current_x:.2f}, {self.current_y:.2f})")
        self.get_logger().info(f"Target waypoint 0: ({tx:.2f}, {ty:.2f})")
        self.wp_pub.publish(Bool(data=True))

    def move_toward_waypoint(self):
        """Move toward current target waypoint"""
        if not self.waypoints or self.wp_index >= len(self.waypoints):
            return

        tx, ty = self.waypoints[self.wp_index]
        dx, dy = tx - self.current_x, ty - self.current_y
        dist = math.hypot(dx, dy)

        # Check if waypoint is reached
        if dist < self.reach_tolerance:
            self.get_logger().info(f"Waypoint {self.wp_index} reached.")
            self.wp_index += 1
            if self.wp_index < len(self.waypoints):
                nx, ny = self.waypoints[self.wp_index]
                self.get_logger().info(f"Target waypoint {self.wp_index}: ({nx:.2f}, {ny:.2f})")
            else:
                self.get_logger().info("All waypoints completed. Stopping.")
                self.wp_active = False
                self.wp_pub.publish(Bool(data=False))
                self.stop_robot()
                return

        # Calculate heading error
        desired_yaw = math.atan2(dy, dx)
        err = self.normalize_angle(desired_yaw - self.current_yaw)
        if abs(err) < self.angular_tolerance:
            err = 0.0

        # Calculate speed commands
        v_base   = min(self.speed_gain * dist, self.max_speed)
        slow_fac = max(0.0, 1 - abs(err)/self.turn_slowdown)
        linear   = max(self.min_speed, v_base * slow_fac)
        angular  = self.angular_gain * err

        # Convert to motor commands
        angular_normalized = max(-1.0, min(1.0, angular))
        angular_command = self.angular_center + (angular_normalized * self.angular_range)
        angular_command = max(100.0, min(440.0, angular_command))

        linear_command = linear * self.linear_scale + self.linear_offset

        # Debug output
        self.get_logger().info(f"Dist: {dist:.2f}m, Err: {err:.3f}rad, "
                             f"Angular: {angular:.3f} → {angular_command:.1f}, "
                             f"Linear: {linear:.3f} → {linear_command:.1f}")

        # Publish command
        twist = Twist()
        twist.linear.x  = linear_command
        twist.angular.z = angular_command
        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        """Stop the robot"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.angular_center
        self.cmd_vel_pub.publish(twist)

    def destroy_node(self):
        """Clean shutdown"""
        self.stop_robot()
        super().destroy_node()


def main():
    rclpy.init()
    node = IMUEncoderFusionNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()