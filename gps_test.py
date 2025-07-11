#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import math

# User-defined origin parameters (set to None to get initial position of the vehicle as reference position)
ref_lat = None
ref_lon = None

class GpsTest(Node):
    def __init__(self):
        super().__init__('gps_test_node')

        # set reference position to ref_lat and ref_lon
        if self.is_valid_latlon(ref_lat, ref_lon):
            self.origin_lat = ref_lat
            self.origin_lon = ref_lon
            self.origin_set = True
            self.get_logger().info(f'Using provided reference: lat={ref_lat:.6f}, lon={ref_lon:.6f}')
        # wait for GPS callback and set reference position to first provided value 
        else:
            self.origin_lat = 0.0
            self.origin_lon = 0.0
            self.origin_set = False
            self.get_logger().info('No valid reference provided. Waiting for first GPS callback.')

        # Subscribe to GPS topic
        self.subscription = self.create_subscription(NavSatFix, '/yoda/gps/fix', self.callback, 10)

    def is_valid_latlon(self, lat, lon):
        return (
            isinstance(lat, float) and isinstance(lon, float) and
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
            self.get_logger().info('Origin set from first GPS callback.')

        # Calculate relative position (assumes flat earth, ignores altitude)
        delta_lat = lat - self.origin_lat
        delta_lon = lon - self.origin_lon

        meters_per_deg_lat = 111320 # traveling east/west by 1 equals to constant 111,320 meters in latitude 0
        meters_per_deg_lon = 111320 * math.cos(math.radians(self.origin_lat)) # traveling north/south equals to decreasing distance over farther latitudes

        dx = delta_lon * meters_per_deg_lon  # East-West
        dy = delta_lat * meters_per_deg_lat  # North-South

        self.get_logger().info(
            f'GPS: Lat={lat:.6f}, Lon={lon:.6f} | ΔX={dx:.2f} m, ΔY={dy:.2f} m'
        )

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
