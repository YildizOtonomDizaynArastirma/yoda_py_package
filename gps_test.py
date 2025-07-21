#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import math

# User-defined origin parameters (set to None to get initial position of the vehicle as reference position)
ref_lat = None
ref_lon = None

# User-defined waypoints (add as many [lat, lon] as needed)
waypoint_list = [
    [0.000135, 0], # baslangica gore 15 metre +X
    [-0.000135, 0], # baslangica gore 15 metre -X
    [0, 0.000135], # baslangica gore 15 metre +Y
    [0, -0.000135], # baslangica gore 15 metre -Y
]

class GpsTest(Node):
    def __init__(self):
        super().__init__('gps_test_node')

        # Validate reference
        if self.is_valid_latlon(ref_lat, ref_lon):
            self.origin_lat = ref_lat
            self.origin_lon = ref_lon
            self.origin_set = True
            self.get_logger().info(f'Using provided reference: lat={ref_lat:.6f}, lon={ref_lon:.6f}')
        else:
            self.origin_lat = 0.0
            self.origin_lon = 0.0
            self.origin_set = False
            self.get_logger().info('No valid reference provided. Waiting for first GPS callback.')

        # Store waypoints (only valid ones)
        self.waypoints = [
            (lat, lon) for lat, lon in waypoint_list
            if self.is_valid_latlon(lat, lon)
        ]
        self.get_logger().info(f'{len(self.waypoints)} valid waypoints loaded.')

        # Subscribe to GPS topic
        self.subscription = self.create_subscription(NavSatFix, '/yoda/gps/fix', self.callback, 10)

    def is_valid_latlon(self, lat, lon):
        return (
            isinstance(lat, (float, int)) and isinstance(lon, (float, int)) and
            not math.isnan(lat) and not math.isnan(lon) and
            -90.0 <= lat <= 90.0 and
            -180.0 <= lon <= 180.0
        )


    def callback(self, msg):
        lat = msg.latitude
        lon = msg.longitude

        if not self.origin_set:
            self.origin_lat = lat
            self.origin_lon = lon
            self.origin_set = True
            self.get_logger().info(f'Origin set from first GPS callback. ({lat:.6f},{lon:.6f})')

        # Calculate relative position to origin (flat earth)
        delta_lat = lat - self.origin_lat
        delta_lon = lon - self.origin_lon

        meters_per_deg_lat = 111_320
        meters_per_deg_lon = 111_320 * math.cos(math.radians(self.origin_lat))

        dx = delta_lon * meters_per_deg_lon  # East-West
        dy = delta_lat * meters_per_deg_lat  # North-South

        self.get_logger().info(
            f'Current GPS: Lat={lat:.6f}, Lon={lon:.6f} '
        )

        # Calculate and print distance to origin (reference)
        self.get_logger().info(
            f'Origin: {self.origin_lat:.6f}, {self.origin_lon:.6f} Distance: ΔX={dx:.2f}, ΔY={dy:.2f} meters'
        )

        # --- Distance to each waypoint
        for i, (wp_lat, wp_lon) in enumerate(self.waypoints):
            distx, disty = self.distance_to_waypoint(lat, lon, wp_lat, wp_lon)
            self.get_logger().info(
                f'  -> Waypoint {i+1}: {wp_lat:.6f}, lon={wp_lon:.6f}) Distance: ΔX = {distx:.2f}, ΔY = {disty:.2f} meters'
            )

    def distance_to_waypoint(self, current_lat, current_lon, wp_lat, wp_lon):
        """Calculate flat-earth distance in meters between two lat/lon points"""
        dlat = wp_lat - current_lat
        dlon = wp_lon - current_lon

        meters_per_deg_lat = 111_320
        meters_per_deg_lon = 111_320 * math.cos(math.radians(current_lat))

        dx = dlon * meters_per_deg_lon
        dy = dlat * meters_per_deg_lat

        return dx, dy

def main(args=None):
    rclpy.init(args=args)
    node = GpsTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
