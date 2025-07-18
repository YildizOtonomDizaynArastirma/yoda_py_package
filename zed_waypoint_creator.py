import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
import math
import json
import threading
import sys
import select
import tty
import termios
import numpy as np
from collections import deque


RAD2DEG = 57.295779513


class WaypointTracker(Node):

    def __init__(self):
        super().__init__('zed_waypoint_tracker')

        # QoS: reliable + volatile to match default ZED QoS settings
        qos_profile = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/zed/zed_node/odom',
            self.odom_callback,
            qos_profile
        )

        # Service client to reset ZED odometry
        self.reset_odom_client = self.create_client(
            Trigger,
            '/zed/zed_node/reset_odometry'
        )

        # Waypoint tracking variables
        self.waypoints = []
        self.last_position = None
        self.last_yaw = None
        self.waypoint_counter = 0
        
        # Thresholds for creating waypoints
        self.distance_threshold = 0.5  # meters
        self.angle_threshold = 0.2     # radians (~11 degrees)
        self.straight_distance_threshold = 2.0  # meters for straight line detection
        
        # Recording state
        self.is_recording = False
        self.recording_start_position = None
        
        # Straight line detection
        self.position_history = deque(maxlen=20)  # Keep last 20 positions
        self.in_straight_line = False
        self.straight_line_start = None
        self.straight_deviation_threshold = 0.1  # meters
        
        # Setup keyboard input for controls
        self.setup_keyboard_input()
        
        self.get_logger().info("ZED 2D Waypoint Tracker initialized!")
        self.get_logger().info("Commands:")
        self.get_logger().info("  's' - Start recording (resets odometry) / Stop recording")
        self.get_logger().info("  'r' - Reset ZED odometry manually")
        self.get_logger().info("  'w' - Save current position as waypoint")
        self.get_logger().info("  'p' - Print all waypoints")
        self.get_logger().info("  'c' - Clear all waypoints")
        self.get_logger().info("  'e' - Export waypoints to JSON file")
        self.get_logger().info("  'q' - Quit")

    def setup_keyboard_input(self):
        """Setup non-blocking keyboard input"""
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        
        # Start keyboard monitoring thread
        self.keyboard_thread = threading.Thread(target=self.keyboard_monitor, daemon=True)
        self.keyboard_thread.start()

    def keyboard_monitor(self):
        """Monitor keyboard input in separate thread"""
        while True:
            try:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1).lower()
                    self.handle_keyboard_input(key)
            except:
                break

    def reset_zed_odometry(self):
        """Reset ZED odometry using service call"""
        if not self.reset_odom_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Reset odometry service not available")
            return False
            
        request = Trigger.Request()
        future = self.reset_odom_client.call_async(request)
        
        # Wait for the service call to complete
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is not None:
            if future.result().success:
                self.get_logger().info("ZED odometry reset successfully!")
                return True
            else:
                self.get_logger().warn(f"Failed to reset odometry: {future.result().message}")
                return False
        else:
            self.get_logger().warn("Service call failed")
            return False

    def handle_keyboard_input(self, key):
        """Handle keyboard commands"""
        if key == 's':
            if not self.is_recording:
                # Start recording - reset odometry first
                self.get_logger().info("Starting recording session...")
                if self.reset_zed_odometry():
                    self.is_recording = True
                    self.recording_start_position = None
                    self.position_history.clear()
                    self.in_straight_line = False
                    self.get_logger().info("Recording STARTED - Odometry reset!")
                else:
                    self.get_logger().warn("Failed to reset odometry - recording not started")
            else:
                # Stop recording
                self.is_recording = False
                self.get_logger().info("Recording STOPPED")
                
        elif key == 'r':
            self.reset_zed_odometry()
            
        elif key == 'w':
            if self.last_position is not None:
                self.add_waypoint(force=True, waypoint_type="manual")
                self.get_logger().info("Manual waypoint added!")
            else:
                self.get_logger().warn("No position data available yet")
                
        elif key == 'p':
            self.print_waypoints()
            
        elif key == 'c':
            self.waypoints.clear()
            self.waypoint_counter = 0
            self.position_history.clear()
            self.in_straight_line = False
            self.get_logger().info("All waypoints cleared!")
            
        elif key == 'e':
            self.export_waypoints()
            
        elif key == 'q':
            self.get_logger().info("Shutting down...")
            self.cleanup()
            rclpy.shutdown()

    def yaw_from_quaternion(self, quat):
        """Extract yaw angle from quaternion"""
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        t3 = +2.0*(w*z + x*y)
        t4 = +1.0 - 2.0*(y*y + z*z)
        yaw = math.atan2(t3, t4)
        return yaw

    def calculate_2d_distance(self, pos1, pos2):
        """Calculate 2D distance between two positions"""
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

    def calculate_yaw_difference(self, yaw1, yaw2):
        """Calculate the difference between two yaw angles"""
        diff = abs(yaw1 - yaw2)
        # Handle wraparound
        if diff > math.pi:
            diff = 2*math.pi - diff
        return diff

    def is_straight_line(self, positions):
        """Check if the given 2D positions form a straight line"""
        if len(positions) < 3:
            return False
            
        # Convert to numpy array
        points = np.array(positions)
        
        # Calculate the line from first to last point
        start_point = points[0]
        end_point = points[-1]
        
        # Vector from start to end
        line_vector = end_point - start_point
        line_length = np.linalg.norm(line_vector)
        
        if line_length < 0.1:  # Too short to determine
            return False
            
        # Normalize the line vector
        line_unit = line_vector / line_length
        
        # Check deviation of each point from the line
        max_deviation = 0
        for point in points[1:-1]:  # Skip first and last points
            # Vector from start to current point
            point_vector = point - start_point
            
            # Project onto line direction
            projection_length = np.dot(point_vector, line_unit)
            projection_point = start_point + projection_length * line_unit
            
            # Calculate perpendicular distance
            deviation = np.linalg.norm(point - projection_point)
            max_deviation = max(max_deviation, deviation)
        
        return max_deviation < self.straight_deviation_threshold

    def detect_straight_line_segments(self, current_position):
        """Detect straight line segments and create waypoints accordingly"""
        if len(self.position_history) < 5:
            return
            
        # Check if we're currently in a straight line
        recent_positions = list(self.position_history)[-10:]  # Last 10 positions
        is_currently_straight = self.is_straight_line(recent_positions)
        
        if is_currently_straight and not self.in_straight_line:
            # Starting a straight line segment
            self.in_straight_line = True
            self.straight_line_start = current_position.copy()
            self.get_logger().info("Detected start of straight line segment")
            
        elif not is_currently_straight and self.in_straight_line:
            # Ending a straight line segment
            self.in_straight_line = False
            if self.straight_line_start is not None:
                # Calculate distance of straight segment
                distance = self.calculate_2d_distance(current_position, self.straight_line_start)
                if distance >= self.straight_distance_threshold:
                    # Add waypoint for straight line segment
                    self.add_waypoint(force=True, waypoint_type="straight_end", 
                                    note=f"End of {distance:.2f}m straight segment")
                    self.get_logger().info(f"Added waypoint for straight segment: {distance:.2f}m")
            self.straight_line_start = None

    def add_waypoint(self, position=None, yaw=None, force=False, waypoint_type="auto", note=""):
        """Add a waypoint if thresholds are met or forced"""
        if position is None:
            position = self.last_position
        if yaw is None:
            yaw = self.last_yaw
            
        if position is None or yaw is None:
            return
            
        current_pos = position if isinstance(position, list) else [position[0], position[1]]
        
        should_add = force
        
        if not force and self.last_position is not None:
            # Check distance threshold
            distance = self.calculate_2d_distance(current_pos, self.last_position)
            angle_diff = self.calculate_yaw_difference(yaw, self.last_yaw)
            
            if distance >= self.distance_threshold or angle_diff >= self.angle_threshold:
                should_add = True
        elif self.last_position is None:
            # First waypoint
            should_add = True
            waypoint_type = "start"
            
        if should_add:
            self.waypoint_counter += 1
            waypoint = {
                'id': self.waypoint_counter,
                'type': waypoint_type,
                'x': current_pos[0],
                'y': current_pos[1],
                'yaw': yaw,
                'yaw_degrees': yaw * RAD2DEG,
                'timestamp': {
                    'sec': rclpy.clock.Clock().now().seconds_nanoseconds()[0],
                    'nanosec': rclpy.clock.Clock().now().seconds_nanoseconds()[1]
                },
                'note': note
            }
            
            self.waypoints.append(waypoint)
            self.last_position = current_pos
            self.last_yaw = yaw
            
            type_str = f"[{waypoint_type.upper()}] " if waypoint_type != "auto" else ""
            self.get_logger().info(
                f"{type_str}Waypoint #{self.waypoint_counter} added: "
                f"X: {current_pos[0]:.2f} Y: {current_pos[1]:.2f} "
                f"Yaw: {yaw*RAD2DEG:.1f}°"
                f"{' - ' + note if note else ''}"
            )

    def odom_callback(self, msg: Odometry):
        """Process odometry messages"""
        # Extract 2D position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Extract yaw angle
        yaw = self.yaw_from_quaternion(msg.pose.pose.orientation)

        current_pos = [x, y]

        # Update position history for straight line detection
        if self.is_recording:
            self.position_history.append(current_pos)
            
            # Set recording start position if not set
            if self.recording_start_position is None:
                self.recording_start_position = current_pos.copy()
                self.add_waypoint(current_pos, yaw, force=True, waypoint_type="start", note="Recording start")
            
            # Detect straight line segments
            self.detect_straight_line_segments(current_pos)
            
            # Add regular waypoints
            self.add_waypoint(current_pos, yaw)

        # Display current position (less frequently to avoid spam)
        if hasattr(self, '_display_counter'):
            self._display_counter += 1
        else:
            self._display_counter = 0
            
        if self._display_counter % 20 == 0:  # Display every 20 messages
            straight_status = "[STRAIGHT]" if self.in_straight_line else ""
            self.get_logger().info(
                f"Position: X: {x:.2f} Y: {y:.2f} Yaw: {yaw*RAD2DEG:.1f}° "
                f"[Recording: {'ON' if self.is_recording else 'OFF'}] {straight_status}"
            )

    def print_waypoints(self):
        """Print all recorded waypoints"""
        if not self.waypoints:
            self.get_logger().info("No waypoints recorded yet")
            return
            
        self.get_logger().info(f"=== {len(self.waypoints)} Recorded Waypoints ===")
        for wp in self.waypoints:
            type_str = f"[{wp['type'].upper()}] " if wp['type'] != "auto" else ""
            note_str = f" - {wp['note']}" if wp.get('note') else ""
            self.get_logger().info(
                f"{type_str}WP#{wp['id']}: "
                f"X: {wp['x']:.2f} Y: {wp['y']:.2f} "
                f"Yaw: {wp['yaw_degrees']:.1f}°{note_str}"
            )

    def export_waypoints(self):
        """Export waypoints to JSON file"""
        if not self.waypoints:
            self.get_logger().warn("No waypoints to export")
            return
            
        filename = f"waypoints_2d_{rclpy.clock.Clock().now().seconds_nanoseconds()[0]}.json"
        
        try:
            with open(filename, 'w') as f:
                json.dump({
                    'waypoints': self.waypoints,
                    'metadata': {
                        'format': '2D waypoints (x, y, yaw)',
                        'total_waypoints': len(self.waypoints),
                        'distance_threshold': self.distance_threshold,
                        'angle_threshold': self.angle_threshold,
                        'straight_distance_threshold': self.straight_distance_threshold,
                        'straight_deviation_threshold': self.straight_deviation_threshold,
                        'export_time': rclpy.clock.Clock().now().seconds_nanoseconds()[0]
                    }
                }, f, indent=2)
            
            self.get_logger().info(f"2D waypoints exported to {filename}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to export waypoints: {e}")

    def cleanup(self):
        """Clean up resources"""
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        except:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = WaypointTracker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()