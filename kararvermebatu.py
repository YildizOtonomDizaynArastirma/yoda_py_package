#!/usr/bin/env python3
import os
import subprocess
import psutil
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from vision_msgs.msg import Detection2DArray
import time

class AutonomousVehicleNode(Node):
    def __init__(self):
        super().__init__('autonomous_vehicle_node')
        self.home = os.path.expanduser("~")
        self.pkg_dir = os.path.dirname(os.path.abspath(__file__))

        # Subscriptions
        self.create_subscription(LaserScan, '/yoda/lidar/scan', self.obstacle_callback, 10)
        self.create_subscription(Float32, '/lane_offset', self.lane_info_callback, 10)
        self.create_subscription(Bool, '/engel/waypoint', self.waypoint_callback, 10)
        self.create_subscription(Detection2DArray, '/detections', self.sign_callback, 10)
        

        self.sign_det_proc = subprocess.Popen(
            ['ros2', 'run', 'yoda_py_package', 'sign_detection'],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True
        )
        self.get_logger().info("Sign detection başlatıldı.")

        self.sign = None
        self.last_sign_time = 0  # Track when last sign was received

        # State flags
        self.obstacle = False
        self.waypoint_active = False

        # Subprocess handles
        self.engel_proc = None
        self.lane_det_proc = None
        self.lane_follow_proc = None
        self.sign_proc = None
        self.sign_detect_proc = None
        self.decision_lock = False  # Tabela manevrası sırasında yeni karar alınmasın


        # 1 Hz control loop
        self.timer = self.create_timer(1.0, self.control_loop)
        self.get_logger().info("Karar verme düğümü başlatıldı. Şerit takibi modu aktif.")
        self.create_timer(0.5, self._check_sign_det)
        self.create_timer(0.1, self._pump_sign_det_log)


    def terminate_process(self, proc, identifier):
        """Attempt a graceful shutdown, then pkill if still alive."""
        if not proc:
            return
        try:
            if proc.poll() is None:
                psutil.Process(proc.pid).terminate()
                proc.wait(timeout=2)
        except Exception:
            pass
        if proc.poll() is None:
            subprocess.run(['pkill', '-f', identifier])

    def waypoint_callback(self, msg: Bool):
        self.waypoint_active = msg.data
        self.get_logger().info(f"Waypoint durumu: {self.waypoint_active}")
        if not self.waypoint_active and self.engel_proc:
            self.terminate_process(self.engel_proc, 'sabit_engel_final.py')
            self.engel_proc = None

    def obstacle_callback(self, msg: LaserScan):
        ranges = msg.ranges
        inc = msg.angle_increment
        span = int((np.pi / 3) / inc)
        mid = len(ranges) // 2
        front = ranges[mid - span // 2: mid + span // 2]
        try:
            d = np.nanmin(front)
        except ValueError:
            return

        prev = self.obstacle
        self.obstacle = (d < 6.0 and not self.waypoint_active)
        if self.obstacle and not prev:
            self.get_logger().info("Engel algılandı → Engel modu")
        elif not self.obstacle and prev:
            self.get_logger().info("Engel kalktı → Şerit takibe dönülüyor")

    def sign_callback(self, msg: Detection2DArray):
        if self.decision_lock:
            self.get_logger().info("Manevra kilidi aktif → yeni tabela yoksayılıyor.")
            return
        if len(msg.detections) == 0:
            return

        detection = msg.detections[0]
        result = detection.results[0]
        label = result.hypothesis.class_id
        confidence = result.hypothesis.score

        # Threshold değeri (örnek: %80)
        if confidence < 0.9:
            self.get_logger().info(f"Tabela {label} algılandı ama güven düşük: {confidence:.2f} < 0.9 → yoksayılıyor.")
            return

        label_to_id = {
            'dur': 1.0,
            'durak': 2.0,
            'girilmez': 3.0,
            'kavsak': 4.0
        }

        if label in label_to_id:
            self.sign = label_to_id[label]
            self.last_sign_time = time.time()
            self.get_logger().info(f"Tabela algılandı: {label} → ID: {self.sign}, Güven: {confidence:.2f}")
        else:
            self.get_logger().warn(f"Bilinmeyen tabela etiketi: {label}")



    def lane_info_callback(self, msg: Float32):
        # Not using offset, but available
        pass

    def _check_sign_det(self):
        if self.sign_det_proc and self.sign_det_proc.poll() is not None:
            # ❶ çıkış kodunu al
            exit_code = self.sign_det_proc.returncode

            # ❷ tek satırlık log – f-string kullan
            self.get_logger().error(
                f"sign_detection.py öldü ↯↯ (exit {exit_code})"
            )

            # ❸ script’in toplu çıktısını al ve satır satır yaz
            for line in self.sign_det_proc.stdout.readlines():
                self.get_logger().error("[sign_det] " + line.rstrip())


    def _pump_sign_det_log(self):
        if self.sign_det_proc and self.sign_det_proc.stdout:
            line = self.sign_det_proc.stdout.readline()
            if line:
                self.get_logger().info("[sign_det] " + line.rstrip())



    def control_loop(self):
        """
        Stay in ENGEL mode while obstacle OR waypoint is active.
        Only switch back to lane mode once both are False.
        """
        

        if not self.sign_detect_proc:
            detect_script = os.path.join(self.pkg_dir, 'sign_detection.py')
            self.sign_detect_proc = subprocess.Popen(['python3', detect_script], cwd=self.pkg_dir)
            self.get_logger().info(f"SignDetection script’i çalıştırıldı: {detect_script}")
        # --- ENGEL MODU ---
        if self.obstacle or self.waypoint_active:
            self.terminate_process(self.lane_det_proc, 'lane.py')
            self.lane_det_proc = None
            self.terminate_process(self.lane_follow_proc, 'lanefollower.py')
            self.lane_follow_proc = None

            if not self.engel_proc:
                engel_script = os.path.join(self.pkg_dir, 'sabit_engel_final.py')
                self.engel_proc = subprocess.Popen(
                    ['python3', engel_script],
                    cwd=self.pkg_dir
                )
                self.get_logger().info(f"Engel manevra script’i çalıştırıldı: {engel_script}")

        # --- SIGN MODE ---
        if self.sign_proc and self.sign_proc.poll() is not None:
            self.get_logger().info("Sign manevrası tamamlandı → kilit kaldırılıyor.")
            self.sign_proc = None
            self.decision_lock = False

        elif self.sign is not None:

            self.terminate_process(self.lane_det_proc, 'lane.py')
            self.lane_det_proc = None
            self.terminate_process(self.lane_follow_proc, 'lanefollower.py')
            self.lane_follow_proc = None

            if not self.sign_proc or self.sign_proc.poll() is not None:
                sign_script = os.path.join(self.pkg_dir, 'sign_decision.py')
                self.sign_proc = subprocess.Popen(
                    ['python3', sign_script],
                    cwd=self.pkg_dir
                )
                self.get_logger().info(f"SignDecision başlatıldı: {sign_script}")


        # --- LANE FOLLOW MODE ---
        else:
            self.terminate_process(self.engel_proc, 'sabit_engel_final.py')
            self.engel_proc = None
            self.terminate_process(self.sign_proc, 'sign_decision.py')
            self.sign_proc = None

            if not self.lane_det_proc:
                lane_script = os.path.join(self.pkg_dir, 'lane.py')
                self.lane_det_proc = subprocess.Popen(
                    ['python3', lane_script],
                    cwd=self.pkg_dir
                )
                self.get_logger().info(f"Şerit tespiti script’i çalıştırıldı: {lane_script}")

            if not self.lane_follow_proc:
                follow_script = os.path.join(self.pkg_dir, 'lanefollower.py')
                self.lane_follow_proc = subprocess.Popen(
                    ['python3', follow_script],
                    cwd=self.pkg_dir
                )
                self.get_logger().info(f"Şerit takip script’i çalıştırıldı: {follow_script}")
    
    def shutdown(self):
        # Kill all subprocesses on shutdown
        for proc, name in [
            (self.engel_proc, 'sabit_engel_final.py'),
            (self.lane_det_proc, 'lane.py'),
            (self.lane_follow_proc, 'lanefollower.py'),
            (self.sign_proc, 'sign_decision.py'),
            (self.sign_detect_proc, 'sign_detection.py')

        ]:
            self.terminate_process(proc, name)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousVehicleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Düğüm sonlandırılıyor, subprocess’lar kapatılıyor...")
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        

if __name__ == '__main__':
    main()
