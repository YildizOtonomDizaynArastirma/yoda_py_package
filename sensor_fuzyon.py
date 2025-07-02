import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32
import numpy as np
from kalman import ExtendedKalmanFilter, VehicleState, SimulationParams
from geometry_msgs.msg import PoseStamped

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Kalman Filtresi ve Parametreler
        self.dt = 0.1
        self.params = SimulationParams(dt=self.dt)
        self.ekf = ExtendedKalmanFilter(VehicleState(), self.params)

        # Sensör verilerini dinlemek için subscriber'lar
        self.create_subscription(NavSatFix, '/yoda/gps/fix', self.gps_callback, 10)
        self.create_subscription(Float32, '/encoder/speed', self.encoder_callback, 10)
        self.create_subscription(Imu, '/yoda/imu/data', self.imu_callback, 10)

        # Füzyon sonucu yayımlamak için bir publisher
        self.pose_publisher = self.create_publisher(PoseStamped, '/fused_pose', 10)

        # Sensör verilerini saklama
        self.gps_data = None
        self.encoder_speed = None
        self.imu_yaw = None

        self.get_logger().info("✅ Sensor Fusion Node başlatıldı ve sensör verilerini dinliyor...")

    def gps_callback(self, msg):
        """GPS verisi alındığında çağrılır."""
        self.gps_data = np.array([msg.latitude, msg.longitude])
        self.get_logger().info(f"🌍 GPS verisi alındı: Latitude={msg.latitude}, Longitude={msg.longitude}")
        self.run_fusion()

    def encoder_callback(self, msg):
        """Encoder verisi alındığında çağrılır."""
        self.encoder_speed = msg.data
        self.get_logger().info(f"⚙️ Encoder verisi alındı: Speed={msg.data} m/s")
        self.run_fusion()

    def imu_callback(self, msg):
        """IMU verisi alındığında çağrılır."""
        self.imu_yaw = msg.orientation.z
        self.get_logger().info(f"🧭 IMU verisi alındı: Yaw (z yönelimi)={msg.orientation.z} rad")
        self.run_fusion()

    def run_fusion(self):
        """Tüm sensör verileri geldiğinde füzyon işlemini başlatır."""
        if self.gps_data is not None and self.encoder_speed is not None and self.imu_yaw is not None:
            self.get_logger().info(f"🔄 Füzyon başlatılıyor...")

            # EKF tahmini
            self.ekf.predict()

            # GPS verisini EKF'ye güncelle
            H_gps = np.array([[1, 0, 0, 0, 0], [0, 1, 0, 0, 0]])
            gps_measurement = np.array([self.gps_data[0], self.gps_data[1]])
            self.ekf.update(gps_measurement, H_gps, self.ekf.R_gps)

            # Encoder hızını EKF'ye güncelle
            H_encoder = np.array([[0, 0, 1, 0, 0]])
            encoder_measurement = np.array([self.encoder_speed])
            self.ekf.update(encoder_measurement, H_encoder, self.ekf.R_encoder)

            # IMU'dan gelen yönelimi EKF'ye güncelle
            H_imu = np.array([[0, 0, 0, 0, 1]])
            imu_measurement = np.array([self.imu_yaw])
            self.ekf.update(imu_measurement, H_imu, self.ekf.R_imu)

            # Füzyon edilen konumu yayımla
            fused_pose = PoseStamped()
            fused_pose.header.stamp = self.get_clock().now().to_msg()
            fused_pose.header.frame_id = "map"
            fused_pose.pose.position.x = self.ekf.state[0]
            fused_pose.pose.position.y = self.ekf.state[1]
            fused_pose.pose.orientation.z = self.ekf.state[4]

            self.pose_publisher.publish(fused_pose)

            self.get_logger().info(f"✅ Füzyon başarılı! X={self.ekf.state[0]:.2f}, Y={self.ekf.state[1]:.2f}, Theta={self.ekf.state[4]:.2f}")

            # Sensör verilerini sıfırla
            self.gps_data = None
            self.encoder_speed = None
            self.imu_yaw = None

def main(args=None):
    rclpy.init(args=args)
    node = Node("sensor_fuzyon_node")
    node = SensorFusionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("⚠️ Sensor Fusion Node kapatılıyor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
