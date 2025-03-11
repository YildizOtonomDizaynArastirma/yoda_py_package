#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class EncoderTest(Node):
    def __init__(self):
        super().__init__('encoder_test_node')
        self.subscription = self.create_subscription(
            JointState,
            '/yoda/joint_states',  # Encoder verisinin topic adı
            self.callback,
            10
        )
        self.get_logger().info('Encoder Test Düğümü Başlatıldı: Encoder verisi dinleniyor...')

    def callback(self, msg):
        # Encoder verilerini al
        joint_names = msg.name
        positions = msg.position
        velocities = msg.velocity
        efforts = msg.effort

        # Loglama için çıktı oluştur
        for i, name in enumerate(joint_names):
            position = positions[i] if i < len(positions) else "N/A"
            velocity = velocities[i] if i < len(velocities) else "N/A"
            effort = efforts[i] if i < len(efforts) else "N/A"

            self.get_logger().info(
                f'{name}: Pozisyon={position}, Hız={velocity}, Tork/Efor={effort}'
            )

def main(args=None):
    rclpy.init(args=args)
    node = EncoderTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Düğüm kapatıldı.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
