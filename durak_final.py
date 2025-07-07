#!/usr/bin/env python3

#22.03.2025
#dt eklendi
#self.yaw_integral ve self.prev_yaw_error eklendi (PID için)
#self.pid() fonksiyonu eklendi
#twist.angular.z pid() fonksiyonuyla hesaplandı

# 30.05.2025
# imu enkoder sub olundu
# callbackleri eklendi
# calculate_orientation() eklendi
# x,y ve yaw fuzyon olmadan trigonometriyle hesaplandi
# odom yoruma alindi
# simulasyonda enkoder olmadigi icin arac surekli sabit kaliyor.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool 
import math
import time

def euclidean_distance(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

class ManeuverNode(Node):
    def __init__(self):
        super().__init__('maneuver_node')
        self.dt = 0.1
        self.start_maneuver = False     #Durak tabelası algılanmadığı sürece bu node çalışmasın diye eklendi.

        # Subscribers and Publishers
        # self.odom_sub = self.create_subscription(Odometry, '/yoda/odom', self.odom_callback, 10)
        self.create_subscription(Float32, '/encoder_pub', self.encoder_callback, 10)
        self.create_subscription(Imu, '/bno_data', self.imu_callback, 10)
        self.create_subscription(Bool, '/start_maneuver', self.start_callback, 10)  #Durak algılandığında çalıştırmak için eklendi
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(self.dt, self.control_loop)  # 10 Hz

        # State machine variables
        self.state = "MOVE_4M"  # Initial state: move 2 meters
        self.state_start_pose = None
        self.state_start_time = None

        # Global starting heading (recorded on first odom reception)
        self.initial_heading = None

        # Latest pose and yaw from odometry
        self.current_pose = None  # (x, y)
        self.current_yaw = 0  # in radians

        self.Kp = 1
        self.Ki = 0.01
        self.Kd = 0.1

        self.yaw_integral = 0
        self.prev_yaw_error = 0

        # Tuning parameters
        self.linear_speed = 80         # m/s for straight movement
        self.turn_linear_speed = 70    # m/s for turning states (forward speed during turns)
        self.max_angular_speed = 1.0    # maximum angular speed (rad/s) for turning
        self.yaw_threshold = 0.09       # threshold for finishing a turn (radians)
        # New parameter: while turning, if yaw error is too high, reduce forward speed
        self.turning_yaw_error_threshold = 0.2  # rad

    # def odom_callback(self, msg: Odometry):
    #     # Update current position and yaw from odometry
    #     x = msg.pose.pose.position.x
    #     y = msg.pose.pose.position.y
    #     self.current_pose = (x, y)

    #     # Convert quaternion to yaw (assuming small roll and pitch)
    #     q = msg.pose.pose.orientation
    #     siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    #     cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    #     self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    #     # Record the global starting heading once
    #      if self.initial_heading is None:
    #          self.initial_heading = self.current_yaw
    #          self.get_logger().info(f"Initial heading: {self.initial_heading:.2f} rad")

    def encoder_callback(self, msg):
        """Encoder verisi alındığında çağrılır."""
        self.encoder_speed = msg.data
        self.get_logger().info(f"Encoder verisi alındı: Speed={msg.data:.2f} m/s")

#    def imu_callback(self, msg): # bu calismazsa yorumdaki ikinci imu_callback() kullanilabilir
#        """IMU verisi alındığında çağrılır."""
#        q = msg.orientation
#        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
#        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
#        self.get_logger().info(f"IMU verisi alındı: Yaw (z yönelimi)={msg.orientation.z:2f} rad")
#        
#        # Record the global starting heading once
#        if self.initial_heading is None:
#            self.initial_heading = self.current_yaw
#            self.get_logger().info(f"Initial heading: {self.initial_heading:.2f} rad")

    def imu_callback(self, msg):
         """IMU verisi alındığında çağrılır."""
         self.current_yaw = math.radians(msg.x)
         self.get_logger().info(f"IMU verisi alındı: Yaw (z yönelimi)={self.current_yaw:.2f} rad")

        
    def calculate_orientation(self):
        x = self.encoder_speed * math.cos(self.current_yaw) #radyan 
        y = self.encoder_speed * math.sin(self.current_yaw) #radyan
        self.current_pose = (x,y)

    #durak manevrasının kontrollü olarak başlatılması için eklendi
    def start_callback(self, msg):
        if msg.data:
            self.get_logger().info("Manevra başlatılıyor.")
            self.start_maneuver = True

    def control_loop(self):

        #Durak algılanmadığı sürece bu node'un çalışmasını önler
        if not self.start_maneuver:
            return  # Eğer durak sinyali gelmediyse hiçbir şey yapma
        
        # Update pose and yaw from IMU + encoder before control logic
        if hasattr(self, 'encoder_speed') and self.encoder_speed is not None and \
            hasattr(self, 'current_yaw') and self.current_yaw is not None:
            self.calculate_orientation()
        else:
            self.get_logger().warn("Encoder ya da IMU verisi bekleniyor...")
            return

            
            twist = Twist()

            # Ensure we have valid odometry data
            if self.current_pose is None or self.current_yaw is None:
                return

        # Initialize state start variables when entering a new state
        if self.state_start_pose is None:
            self.state_start_pose = self.current_pose
        # For any waiting state, initialize the start time if not set.
        if self.state in ["WAIT_40", "WAIT_5"] and self.state_start_time is None:
            self.state_start_time = time.time()

        # --- State Machine ---
        if self.state == "MOVE_4M":
            # Move straight until 2.0 meters are traveled.
            dist = euclidean_distance(self.state_start_pose, self.current_pose)
            self.get_logger().info(f"MOVE_4M: distance traveled = {dist:.2f} m")
            if dist < 4.0:
                twist.linear.x = self.linear_speed
                twist.angular.z = 270.0
            else:
                self.get_logger().info("Moved 4m. Transitioning to TURN_RIGHT_45.")
                self.transition_state("TURN_RIGHT_45")

        elif self.state == "TURN_RIGHT_45":
            # Turn right 45° relative to the initial heading.
            target_yaw = self.normalize_angle(self.initial_heading - math.radians(45))
            yaw_error = self.normalize_angle(target_yaw - self.current_yaw)
            if abs(yaw_error) > self.yaw_threshold:
                twist.linear.x = self.turn_linear_speed
                twist.angular.z = self.clamp(self.pid(yaw_error), -self.max_angular_speed, self.max_angular_speed)
            else:
                self.get_logger().info("Turned right 45°. Transitioning to MOVE_50CM_A.")
                self.transition_state("MOVE_2M_A")

        elif self.state == "MOVE_2M_A":
            # Move forward 2 meters.
            dist = euclidean_distance(self.state_start_pose, self.current_pose)
            if dist < 2.0:
                twist.linear.x = self.linear_speed
                twist.angular.z = 270.0
            else:
                self.get_logger().info("Moved 2m. Transitioning to TURN_LEFT_45.")
                self.transition_state("TURN_LEFT_45")

        elif self.state == "TURN_LEFT_45":
            # Turn left 45° to return to the initial heading.
            target_yaw = self.initial_heading
            yaw_error = self.normalize_angle(target_yaw - self.current_yaw)
            # Use a minimum forward speed so the car moves along an arc
            min_forward_speed = 0.3
            if abs(yaw_error) > self.turning_yaw_error_threshold:
                twist.linear.x = min_forward_speed
            else:
                twist.linear.x = self.turn_linear_speed
            twist.angular.z = self.clamp(self.pid(yaw_error), -self.max_angular_speed, self.max_angular_speed)
            if abs(yaw_error) <= self.yaw_threshold:
                self.get_logger().info("Returned to original heading. Transitioning to WAIT_5.")
                self.transition_state("WAIT_5")

        elif self.state == "WAIT_5":
            # Wait for 5 seconds.
            twist.linear.x = 0.0
            twist.angular.z = 270.0
            elapsed = time.time() - self.state_start_time
            if elapsed >= 5.0:
                self.get_logger().info("Wait complete. Transitioning to MOVE_1.5M_B.")
                self.transition_state("MOVE_1.5M_B")

        elif self.state == "MOVE_1.5M_B":
            # Move forward 1.5 meters.
            dist = euclidean_distance(self.state_start_pose, self.current_pose)
            if dist < 1.5:
                twist.linear.x = self.linear_speed
                twist.angular.z = 270.0
            else:
                self.get_logger().info("Moved 1.5m. Transitioning to TURN_LEFT_45_SECOND_AND_MOVE_2CM_C.")
                self.transition_state("TURN_LEFT_45_SECOND_AND_MOVE_2CM_C")

        elif self.state == "TURN_LEFT_45_SECOND_AND_MOVE_2CM_C":
            # Combined state: Turn left 45° relative to the original heading while moving forward 2m.
            target_yaw = self.normalize_angle(self.initial_heading + math.radians(45))
            yaw_error = self.normalize_angle(target_yaw - self.current_yaw)
            dist = euclidean_distance(self.state_start_pose, self.current_pose)
            # Ensure some forward motion. If yaw error is large, use a reduced speed.
            min_forward_speed = 0.2
            if abs(yaw_error) > self.turning_yaw_error_threshold:
                twist.linear.x = min_forward_speed
            else:
                twist.linear.x = self.turn_linear_speed
            twist.angular.z = self.clamp(self.pid(yaw_error), -self.max_angular_speed, self.max_angular_speed)
            self.get_logger().info(f"Combined State: linear.x = {twist.linear.x:.2f}, angular.z = {twist.angular.z:.2f}, yaw_error = {yaw_error:.2f}, dist = {dist:.2f}")
            if abs(yaw_error) <= self.yaw_threshold and dist >= 4.0:
                self.get_logger().info("Combined left turn and 2m forward achieved. Transitioning to TURN_RIGHT_45_SECOND.")
                self.transition_state("TURN_RIGHT_45_SECOND")
            self.cmd_pub.publish(twist)
            return  # Exit control_loop to avoid publishing again below

        elif self.state == "TURN_RIGHT_45_SECOND":
            # Turn right 45° to return to the original heading.
            target_yaw = self.normalize_angle(self.initial_heading)
            yaw_error = self.normalize_angle(target_yaw - self.current_yaw)
            if abs(yaw_error) > self.yaw_threshold:
                twist.linear.x = self.turn_linear_speed
                twist.angular.z = self.clamp(self.pid(yaw_error), -self.max_angular_speed, self.max_angular_speed)
            else:
                self.get_logger().info("Final alignment reached. Maneuver complete.")
                self.transition_state("DONE")

        elif self.state == "DONE":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("Maneuver complete. Shutting down control loop.")
            self.timer.cancel()

        # Publish the command velocity.
        self.cmd_pub.publish(twist)

    def pid(self,yaw_error):
            self.yaw_integral += yaw_error * self.dt
            P = self.Kp * yaw_error
            I = self.Ki * self.yaw_integral
            D = self.Kd * (yaw_error - self.prev_yaw_error) / self.dt
            self.prev_yaw_error = yaw_error
            output = P + I + D
            return output

    def transition_state(self, new_state):
        """Transition to a new state and reset state-related variables."""
        self.state = new_state
        self.state_start_pose = self.current_pose  # reset the distance reference
        self.state_start_time = None  # reset time for wait states

    def clamp(self, value, min_val, max_val):
        return max(min_val, min(value, max_val))

    def normalize_angle(self, angle):
        """Normalize angle to the range [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    

def main(args=None):
    rclpy.init(args=args)
    node = ManeuverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

