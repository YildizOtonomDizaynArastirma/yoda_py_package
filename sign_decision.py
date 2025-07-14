#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool 
from std_msgs.msg import Float32
from std_msgs.msg import Int32 
import time
import math
import numpy as np
import cvxpy as cp

class SignDecision(Node):   
    def __init__(self):    
        super().__init__('sign_decision_node')

        self.sign_ID = 'id_topic'           
        self.imu_rate = '/yoda/imu/data'    
        
        self.decision = None
        self.initial_linear_velocity = 0.5
        self.turning_linear_velocity = 0.5
        self.max_turn_angle = 90
        self.yaw_angle = 0.0
        self.initial_yaw = None
        self.is_turning = False
        self.turn_complete = False
        self.traffic_light_id = None

        self.N = 10
        self.dt = 0.5
        self.max_steering_angle = math.radians(30)

        self.sign_subscription = self.create_subscription(
            Float32,
            self.sign_ID,
            self.sign_ID_listener_callback,
            10)

        self.create_subscription(
            Int32, 
            '/traffic_light_id', 
            self.traffic_light_callback, 
            10)

        self.publisher_ = self.create_publisher(Twist, '/yoda/cmd_vel', 10)
        self.maneuver_pub = self.create_publisher(Bool, '/start_maneuver', 10)
        self.park_pub = self.create_publisher(Bool, '/start_parking', 10)

        self.imu_subscription = self.create_subscription(
            Imu,                   
            self.imu_rate,
            self.imu_listener_callback,
            10)

        self.timer = self.create_timer(0.5, self.timer_callback)
        self.start_moving()

    def maneuver_for_station(self):
        msg = Bool()
        msg.data = True
        self.maneuver_pub.publish(msg)

    def start_parking(self):
        msg = Bool()
        msg.data = True
        self.park_pub.publish(msg)

    def traffic_light_callback(self, msg):
        self.traffic_light_id = msg.data
        self.get_logger().info(f"Trafik ışığı algılandı: {self.traffic_light_id}")

    def start_moving(self):
        start_msg = Twist()
        start_msg.linear.x = self.initial_linear_velocity  
        start_msg.angular.z = 0.0
        self.publisher_.publish(start_msg)
        self.get_logger().info(f"Araç {self.initial_linear_velocity} hızla ileri gidiyor")

    def sign_ID_listener_callback(self, msg):
        self.get_logger().info(f'Alinan veri: {msg.data}')     
        self.decision = msg.data
        self.decision_part()

    def imu_listener_callback(self, msg):
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        current_yaw = math.atan2(siny_cosp, cosy_cosp) * (180 / math.pi)
        self.yaw_angle = current_yaw

    def timer_callback(self):
        if self.is_turning and self.initial_yaw is not None:
            angle_diff = self.calculate_angle_diff(self.initial_yaw, self.yaw_angle)
            self.get_logger().info(f"Dönüş: {angle_diff:.2f} derece, Hedef: 90 derece")
            
            if angle_diff >= 85:
                self.get_logger().info("Dönüş tamamlandı! Düz harekete geçiliyor.")
                self.is_turning = False
                self.turn_complete = True

                straight_msg = Twist()
                straight_msg.linear.x = self.initial_linear_velocity
                straight_msg.angular.z = 0.0
                self.publisher_.publish(straight_msg)
            else:
                error = 90 - angle_diff
                u = cp.Variable()
                if self.turn_direction == 'left':
                    k_mpc = 0.003 if error > 55 else 0.03   # Sola dönüş
                elif self.turn_direction == 'right':
                    k_mpc = 0.003 if error > 75 else 0.03   # Sağa dönüş
                cost = cp.square(u - error * k_mpc)
                constraints = [cp.abs(u) <= self.max_steering_angle]
                prob = cp.Problem(cp.Minimize(cost), constraints)
                prob.solve()

                angular_z = float(u.value) if self.turn_direction == 'left' else -float(u.value)
                # Direksiyon açısını ±30 derece (0.523 rad) ile sınırla
                angular_z = max(min(angular_z, self.max_steering_angle), -self.max_steering_angle)

                twist = Twist()
                twist.linear.x = self.turning_linear_velocity
                twist.angular.z = angular_z
                self.publisher_.publish(twist)

                self.get_logger().info(f"{self.turn_direction.upper()} | k_mpc: {k_mpc}, angular_z: {angular_z:.2f}")

    def calculate_angle_diff(self, start_angle, current_angle):
        diff = current_angle - start_angle
        if diff < -180:
            diff += 360
        elif diff > 180:
            diff -= 360
        return abs(diff)

    def decision_part(self):
        if self.decision == 0.0:
            self.get_logger().info("duz devam et")
            self.start_moving()
        elif self.decision == 1.0:
            self.get_logger().info("dur")
            self.stop_moving()
        elif self.decision == 2.0:
            self.get_logger().info("Durak")
            self.maneuver_for_station()
        elif self.decision == 3.0:
            self.get_logger().info("Girilmez")
            self.stop_moving()
        elif self.decision == 4.0:
            self.get_logger().info("Kavşak")
        elif self.decision == 5.0:
            if not self.is_turning or self.turn_direction != 'left':
                self.get_logger().info("sola mecburi yon")
                self.turn_left()
        elif self.decision == 6.0:
            if not self.is_turning or self.turn_direction != 'right':
                self.get_logger().info("saga mecburi yon")
                self.turn_right()
        elif self.decision == 7.0:
            call = 1
            if call == 1:
                self.turn_left()
            else:
                self.get_logger().info("duz devam et")
        elif self.decision == 8.0:
            call = 1
            if call == 1:
                self.turn_right()
            else:
                self.get_logger().info("duz devam et")
        elif self.decision == 9.0:
            if self.traffic_light_id == 0:
                self.get_logger().info("Kırmızı ışık – DUR")
                self.stop_moving()
            elif self.traffic_light_id == 1:
                self.get_logger().info("Yeşil ışık – HAREKET ET")
                self.start_moving()          
        elif self.decision == 10.0:
            self.get_logger().info("park et")
            self.start_parking()
        else:
            self.get_logger().info(f"Tanımlanmamış karar kodu: {self.decision}")

    def turn_left(self):
        if self.is_turning:
            self.get_logger().info("Zaten dönüş yapılıyor.")
            return
        self.get_logger().info("MPC ile sola dönüş başlatıldı.")
        self.initial_yaw = self.yaw_angle
        self.is_turning = True
        self.turn_complete = False
        self.turn_direction = 'left'

    def turn_right(self):
        if self.is_turning:
            self.get_logger().info("Zaten dönüş yapılıyor.")
            return
        self.get_logger().info("MPC ile sağa dönüş başlatıldı.")
        self.initial_yaw = self.yaw_angle
        self.is_turning = True
        self.turn_complete = False
        self.turn_direction = 'right'

    def stop_moving(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.publisher_.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args) 
    node = SignDecision()   
    rclpy.spin(node) 
    node.destroy_node()
    rclpy.shutdown() 

if __name__ == "__main__":
    main()