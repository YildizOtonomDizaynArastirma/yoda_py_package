#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np
import math



class MultiWaypointNavigator(Node):
    def __init__(self):
        super().__init__('multi_waypoint_navigator')

        # tuning parameters
        self.max_speed         = 0.4   # m/s
        self.min_speed         = 0.3   # m/s
        self.speed_gain        = 1.0    # v = gain * distance
        self.angular_gain      = 1.0    # w = gain * angular error
        self.turn_slowdown     = 3*math.pi/4  # rad at which linear=0
        self.reach_tolerance   = 0.5    # meters
        self.angular_tolerance = 0.005   # radians

        # Motor komut değer aralıkları
        self.angular_center = 270.0      # Merkez değer (düz)
        self.angular_range = 170.0       # ±170 → 100 ile 440 arası
        self.linear_scale = 150.0         # Lineer hız çarpanı
        self.linear_offset = 0.0         # Lineer hız offset

        # current pose
        self.current_x   = 0.0
        self.current_y   = 0.0
        self.current_yaw = 0.0

        # Left lane obstacle avoidance maneuver
        # x = forward distance, y = left distance (positive y = left)
        # Maneuver: go left first, then forward around obstacle, then back to center
        x_add = [0.0, 0.5, 1.0, 1.7, 2.4, 3.0,4.0 ,5.0, 6.0, 6.7, 7.4, 8.1, 8.8, 9.5, 10.2, 10.8, 11.5, 12.0, 13.0, 14.0, 15.0, 16.0]  # Forward progression
        y_add = [0.0, 0.05, 0.3, 0.8, 1.4, 1.7,2.0,2.5, 2.5, 2.2, 2.0, 1.7, 1.4, 1.0, 0.8, 0.5, 0.3, 0.005,   0.0,  0.0,  0.0,   0.0]      # Left offset, then return to center
        
        # Create waypoint offsets in vehicle frame
        self.wp_offsets = [(dx, dy) for dx, dy in zip(x_add, y_add)]

        # waypoint state
        self.waypoints = []
        self.wp_index  = 0
        self.wp_active = False

        # QoS profile for ZED2 camera
        qos_profile = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE
        )

        # publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.wp_pub      = self.create_publisher(Bool,  '/engel/waypoint', 10)
        self.create_subscription(Odometry,  '/zed/zed_node/odom',  self.odom_callback, qos_profile)
        self.get_logger().info("Engel manevra düğümü başlatıldı.")
        self.waypoint_create()
        
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
    def odom_callback(self, msg: Odometry):
        # ZED2 odom formatına göre pozisyon ve orientasyon al
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.current_x, self.current_y = p.x, p.y
        _, _, self.current_yaw = self.euler_from_quaternion([o.x, o.y, o.z, o.w])

        if self.wp_active:
            self.move_toward_waypoint()

    def waypoint_create(self):
        yaw = self.current_yaw
        self.waypoints = []
        for dx_local, dy_local in self.wp_offsets:
            # rotate local offset by current yaw
            global_dx = dx_local*math.cos(yaw) - dy_local*math.sin(yaw)
            global_dy = dx_local*math.sin(yaw) + dy_local*math.cos(yaw)
            wx = self.current_x + global_dx
            wy = self.current_y + global_dy
            self.waypoints.append((wx, wy))

        self.wp_index  = 0
        self.wp_active = True
        tx, ty = self.waypoints[0]
        print(f'suanki konum: {self.current_x}, {self.current_y}')
        self.get_logger().info(f"Hedef waypoint 0: ({tx:.2f}, {ty:.2f})")
        self.wp_pub.publish(Bool(data=True))

 

    def move_toward_waypoint(self):
        tx, ty = self.waypoints[self.wp_index]
        dx, dy = tx - self.current_x, ty - self.current_y
        dist    = math.hypot(dx, dy)

        # check reach
        if dist < self.reach_tolerance:
            print(f'suanki konum1: {self.current_x}, {self.current_y}')
            self.get_logger().info(f"Waypoint {self.wp_index} ulaşıldı.")
            self.wp_index += 1
            if self.wp_index < len(self.waypoints):
                nx, ny = self.waypoints[self.wp_index]
                self.get_logger().info(f"Hedef waypoint {self.wp_index}: ({nx:.2f}, {ny:.2f})")
            else:
                self.get_logger().info("Tüm waypoint'ler tamamlandı. Shutting down.")
                self.wp_active = False
                self.wp_pub.publish(Bool(data=False))
                self.stop_robot()
                self.destroy_node()
                return

        # heading error
        desired_yaw = math.atan2(dy, dx)
        err = (desired_yaw - self.current_yaw + math.pi) % (2*math.pi) - math.pi
        if abs(err) < self.angular_tolerance:
            err = 0.0

        # speed commands (orijinal değerler)
        v_base   = min(self.speed_gain * dist, self.max_speed)
        slow_fac = max(0.0, 1 - abs(err)/self.turn_slowdown)
        linear   = max(self.min_speed, v_base * slow_fac)
        angular  = self.angular_gain * err

        # Motor komutlarına dönüştürme
        # Açısal hız: radyan/s → 110-440 aralığı
        # Negatif angular = sola dönüş = daha düşük değer
        # Pozitif angular = sağa dönüş = daha yüksek değer
        angular_normalized = max(-1.0, min(1.0, angular))  # -1 ile 1 arası sınırla
        angular_command = self.angular_center + (angular_normalized * self.angular_range)
        angular_command = max(100.0, min(440.0, angular_command))  # 100-440 arası sınırla

        # Lineer hız: m/s → ~70 civarı değerler
        linear_command = linear * self.linear_scale + self.linear_offset

        # Debug çıktısı
        self.get_logger().info(f"Angular: {angular:.3f} rad/s → {angular_command:.1f}, Linear: {linear:.3f} m/s → {linear_command:.1f}")

        twist = Twist()
        twist.linear.x  = linear_command
        twist.angular.z = angular_command
        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.angular_center  # Düz konuma getir
        self.cmd_vel_pub.publish(twist)


def main():
    rclpy.init()
    node = MultiWaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
