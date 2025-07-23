#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import math
import json
import os

# --- User-defined origin parameters (set to None to get initial position of the vehicle as reference position)
ref_lat = None
ref_lon = None

# JSON input file
WAYPOINT_FILE = 'waypoints.json'

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

        # Load waypoints from JSON file
        self.waypoints = self.load_waypoints_from_json(WAYPOINT_FILE)
        self.get_logger().info(f'{len(self.waypoints)} valid waypoints loaded from JSON.')

        # Subscribe to GPS topic
        self.subscription = self.create_subscription(NavSatFix, '/yoda/gps/fix', self.callback, 10)

    def is_valid_latlon(self, lat, lon):
        return (
            isinstance(lat, (float, int)) and isinstance(lon, (float, int)) and
            not math.isnan(lat) and not math.isnan(lon) and
            -90.0 <= lat <= 90.0 and
            -180.0 <= lon <= 180.0
        )

    def load_waypoints_from_json(self, filename):
        waypoints = []
        try:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            filepath = os.path.join(current_dir, filename)
            with open(filepath, 'r', encoding='utf-8') as f:
                data = json.load(f)

            if data.get('type') == 'FeatureCollection':
                for feature in data.get('features', []):
                    geometry = feature.get('geometry', {})
                    coords = geometry.get('coordinates', [])
                    if (
                        isinstance(coords, list) and len(coords) == 2 and
                        self.is_valid_latlon(coords[1], coords[0])  # Note: GeoJSON uses [lon, lat]
                    ):
                        lat = coords[1]
                        lon = coords[0]
                        waypoints.append((lat, lon))
            else:
                self.get_logger().warn('Invalid GeoJSON structure in waypoint file.')
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints from JSON: {e}')
        return waypoints

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
        self.get_logger().info(
            f'Origin: {self.origin_lat:.6f}, {self.origin_lon:.6f} Distance: ΔX={dx:.2f}, ΔY={dy:.2f} meters'
        )

        # --- Distance to each waypoint
        for i, (wp_lat, wp_lon) in enumerate(self.waypoints):
            distx, disty = self.distance_to_waypoint(lat, lon, wp_lat, wp_lon)
            self.get_logger().info(
                f'  -> Waypoint {i+1}: {wp_lat:.6f}, lon={wp_lon:.6f} Distance: ΔX = {distx:.2f}, ΔY = {disty:.2f} meters'
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
