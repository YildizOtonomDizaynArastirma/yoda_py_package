#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from std_msgs.msg import Bool
import cv2
import numpy as np
import math
import random
from enum import Enum

class ParkingState(Enum):
    """Parking state machine states"""
    BLIND_DRIVING = 1    # Driving forward when no lines detected
    SLOT_SELECTION = 2   # Selecting target parking slot
    STEERING = 3         # Steering towards target slot
    ALIGNED = 4          # Aligned and stopped

class ParkingLineDetector(Node):
    """
    ROS2 Node for parking line detection and autonomous parking.
    
    This node subscribes to camera images, detects white parking lines,
    and controls the robot to park in a randomly selected slot.
    """
    
    def __init__(self):
        super().__init__('parking_line_detector')

        self.start_parking = False      #Park alanı algılanmadığı sürece çalışmasın diye eklendi
        
        # ROS2 Publishers and Subscribers
        self.image_subscriber = self.create_subscription(
            Image, '/yoda/camera/image_raw', self.image_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/yoda/cmd_vel', 10)

        self.create_subscription(Bool, '/start_parking', self.start_parking_callback, 10)       #Park algılandığında çalıştırmak için eklendi.
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # State machine
        self.current_state = ParkingState.BLIND_DRIVING
        
        # Image processing parameters
        self.canny_low = 50
        self.canny_high = 150
        self.hough_threshold = 100
        self.hough_min_line_length = 50
        self.hough_max_line_gap = 20
        
        # Vertical line detection parameters
        self.vertical_angle_tolerance = 15  # degrees
        self.min_vertical_lines = 2
        
        # Robot control parameters
        self.blind_drive_speed = 0.3  # m/s
        self.steering_kp = 0.01  # Proportional gain for steering
        self.max_angular_velocity = 0.5  # rad/s
        self.alignment_tolerance = 20  # pixels
        
        # Parking slot data
        self.vertical_lines = []
        self.target_slot_center = None
        self.image_width = 0
        self.image_height = 0
        
        # Control timer (10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Parking Line Detector Node Started")
        self.get_logger().info("State: BLIND_DRIVING - Looking for parking lines...")


    #Park algoritmasını kontrollü olarak başlatmak için eklendi.
    def start_parking_callback(self, msg):
        if msg.data:
            self.get_logger().info("Park etme tetiklendi!")
            self.start_parking = True



    
    def image_callback(self, msg):
        """
        Callback function for camera image topic.
        Processes the image to detect parking lines.
        """
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.image_height, self.image_width = cv_image.shape[:2]
            
            # Detect white parking lines
            self.detect_parking_lines(cv_image)
            
            # Update state machine based on detection results
            self.update_state_machine()
            
            # Display visual feedback
            self.show_visual_feedback(cv_image)
            
        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")
    
    def detect_parking_lines(self, image):
        """
        Detect white parking lines using Canny edge detection and HoughLinesP.
        
        Args:
            image: Input BGR image from camera
        """
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Apply Canny edge detection
        edges = cv2.Canny(blurred, self.canny_low, self.canny_high)
        
        # Detect lines using HoughLinesP
        lines = cv2.HoughLinesP(
            edges, 
            rho=1, 
            theta=np.pi/180,
            threshold=self.hough_threshold,
            minLineLength=self.hough_min_line_length,
            maxLineGap=self.hough_max_line_gap
        )
        
        # Filter for vertical lines
        self.vertical_lines = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                
                # Calculate line angle
                angle = math.degrees(math.atan2(abs(y2 - y1), abs(x2 - x1)))
                
                # Check if line is approximately vertical
                if angle > (90 - self.vertical_angle_tolerance):
                    self.vertical_lines.append({
                        'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                        'center_x': (x1 + x2) // 2,
                        'angle': angle
                    })
        
        # Sort vertical lines by x-coordinate
        self.vertical_lines.sort(key=lambda line: line['center_x'])
        
        self.get_logger().debug(f"Detected {len(self.vertical_lines)} vertical lines")
    
    def update_state_machine(self):
        """
        Update the state machine based on detected lines.
        """
        num_lines = len(self.vertical_lines)
        
        if self.current_state == ParkingState.BLIND_DRIVING:
            if num_lines >= self.min_vertical_lines:
                self.current_state = ParkingState.SLOT_SELECTION
                self.select_random_parking_slot()
                self.get_logger().info(f"State: SLOT_SELECTION - Found {num_lines} vertical lines")
        
        elif self.current_state == ParkingState.SLOT_SELECTION:
            if self.target_slot_center is not None:
                self.current_state = ParkingState.STEERING
                self.get_logger().info(f"State: STEERING - Target slot center: {self.target_slot_center}")
        
        elif self.current_state == ParkingState.STEERING:
            if self.is_aligned():
                self.current_state = ParkingState.ALIGNED
                self.get_logger().info("State: ALIGNED - Parking completed!")
        
        # Reset to blind driving if we lose the lines
        if num_lines < self.min_vertical_lines and self.current_state != ParkingState.ALIGNED:
            if self.current_state != ParkingState.BLIND_DRIVING:
                self.current_state = ParkingState.BLIND_DRIVING
                self.target_slot_center = None
                self.get_logger().info("State: BLIND_DRIVING - Lost parking lines")
    
    def select_random_parking_slot(self):
        """
        Select a random parking slot between detected vertical lines.
        """
        if len(self.vertical_lines) < 2:
            self.target_slot_center = None
            return
        
        # Create parking slots between adjacent vertical lines
        slots = []
        for i in range(len(self.vertical_lines) - 1):
            left_line = self.vertical_lines[i]
            right_line = self.vertical_lines[i + 1]
            
            # Calculate slot center
            slot_center_x = (left_line['center_x'] + right_line['center_x']) // 2
            slot_center_y = self.image_height // 2  # Middle of image height
            
            slots.append({
                'center_x': slot_center_x,
                'center_y': slot_center_y,
                'left_line': left_line,
                'right_line': right_line
            })
        
        # Select random slot
        if slots:
            selected_slot = random.choice(slots)
            self.target_slot_center = (selected_slot['center_x'], selected_slot['center_y'])
            self.get_logger().info(f"Selected random parking slot at x={selected_slot['center_x']}")
    
    def is_aligned(self):
        """
        Check if the robot is aligned with the target parking slot.
        
        Returns:
            bool: True if aligned within tolerance
        """
        if self.target_slot_center is None:
            return False
        
        image_center_x = self.image_width // 2
        target_x = self.target_slot_center[0]
        
        error = abs(target_x - image_center_x)
        return error < self.alignment_tolerance
    
    def control_loop(self):
        """
        Main control loop for robot movement.
        Called at 10 Hz by the timer.
        """
        cmd_vel = Twist()

        #Park tabelası algılanmadığı sürece bu node.un çalışmasını önler.
        if not self.start_parking:
            return  # Park etme sinyali gelmediyse hiçbir şey yapma
        
        if self.current_state == ParkingState.BLIND_DRIVING:
            # Drive forward at fixed speed
            cmd_vel.linear.x = self.blind_drive_speed
            cmd_vel.angular.z = 0.0
        
        elif self.current_state == ParkingState.STEERING:
            # Proportional control for steering towards target
            if self.target_slot_center is not None:
                image_center_x = self.image_width // 2
                target_x = self.target_slot_center[0]
                
                # Calculate steering error
                error = target_x - image_center_x
                
                # Apply proportional control
                angular_velocity = self.steering_kp * error
                
                # Limit angular velocity
                angular_velocity = max(-self.max_angular_velocity, 
                                     min(self.max_angular_velocity, angular_velocity))
                
                # Continue moving forward while steering
                cmd_vel.linear.x = self.blind_drive_speed * 0.5  # Slower while steering
                cmd_vel.angular.z = angular_velocity
                
                self.get_logger().debug(f"Steering error: {error}, Angular vel: {angular_velocity:.3f}")
        
        elif self.current_state == ParkingState.ALIGNED:
            # Stop the robot
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
        
        # Publish velocity command
        self.cmd_vel_publisher.publish(cmd_vel)
    
    def show_visual_feedback(self, image):
        """
        Display visual feedback using OpenCV imshow.
        
        Args:
            image: Original BGR image
        """
        # Create a copy for visualization
        vis_image = image.copy()
        
        # Draw detected vertical lines
        for line in self.vertical_lines:
            cv2.line(vis_image, 
                    (line['x1'], line['y1']), 
                    (line['x2'], line['y2']), 
                    (0, 255, 0), 3)  # Green lines
            
            # Draw line center point
            cv2.circle(vis_image, 
                      (line['center_x'], line['y1']), 
                      5, (0, 255, 0), -1)
        
        # Draw target slot center
        if self.target_slot_center is not None:
            cv2.circle(vis_image, 
                      self.target_slot_center, 
                      10, (0, 0, 255), -1)  # Red circle
            
            # Draw target slot indicator
            cv2.putText(vis_image, 
                       "TARGET SLOT", 
                       (self.target_slot_center[0] - 50, self.target_slot_center[1] - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        # Draw image center line (robot's heading)
        image_center_x = self.image_width // 2
        cv2.line(vis_image, 
                (image_center_x, 0), 
                (image_center_x, self.image_height), 
                (255, 0, 0), 2)  # Blue line
        
        # Display state information
        state_text = f"State: {self.current_state.name}"
        lines_text = f"Lines: {len(self.vertical_lines)}"
        
        cv2.putText(vis_image, state_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(vis_image, lines_text, (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        # Show alignment status
        if self.current_state == ParkingState.STEERING and self.target_slot_center:
            alignment_error = abs(self.target_slot_center[0] - image_center_x)
            alignment_text = f"Alignment Error: {alignment_error} px"
            cv2.putText(vis_image, alignment_text, (10, 90), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Display the image
        cv2.imshow('Parking Line Detection', vis_image)
        cv2.waitKey(1)  # Required for OpenCV window updates
    
    def destroy_node(self):
        """
        Clean up resources when node is destroyed.
        """
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    """
    Main function to run the parking line detector node.
    """
    rclpy.init(args=args)
    
    try:
        node = ParkingLineDetector()
        
        # Log startup information
        node.get_logger().info("=== Parking Line Detector Node ===")
        node.get_logger().info("Subscribing to: /yoda/camera/image_raw")
        node.get_logger().info("Publishing to: /cmd_vel")
        node.get_logger().info("Press Ctrl+C to stop")
        
        # Spin the node
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\nShutting down parking line detector...")
    
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()