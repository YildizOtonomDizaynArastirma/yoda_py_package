#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np
import math
from rclpy.executors import ExternalShutdownException
from collections import deque

try:
    from tf_transformations import euler_from_quaternion
except ImportError:
    def euler_from_quaternion(quat):
        x, y, z, w = quat
        t0 = +2.0*(w*x + y*z)
        t1 = +1.0 - 2.0*(x*x + y*y)
        roll = math.atan2(t0, t1)
        t2 = +2.0*(w*y - z*x)
        t2 = max(-1.0, min(1.0, t2))
        pitch = math.asin(t2)
        t3 = +2.0*(w*z + x*y)
        t4 = +1.0 - 2.0*(y*y + z*z)
        yaw = math.atan2(t3, t4)
        return roll, pitch, yaw

class ContinuousMovementObstacleDetector(Node):
    def __init__(self):
        super().__init__('continuous_movement_obstacle_detector')

        # Temel parametreler
        self.min_obstacle_distance = 5.0  # meters
        self.analysis_duration = 2.5  # saniye - analiz süresi
        self.crossing_threshold = 0.3  # metre - yan hareket eşiği
        self.min_readings = 8  # minimum okuma sayısı
        self.linear_speed = 1.0  # m/s - normal sürüş hızı
        
        # Durum değişkenleri
        self.obstacle_detected = False
        self.analysis_mode = False
        self.analysis_start_time = None
        self.analysis_readings = 0
        self.maneuvering = False
        
        # Sadece mevcut engel verisi
        self.current_positions = deque(maxlen=15)
        self.initial_position = None
        self.last_position = None
        
        # Odometry verisi
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Manevra parametreleri
        self.max_speed = 1.0
        self.min_speed = 0.05
        self.speed_gain = 1.0
        self.angular_gain = 1.5
        self.turn_slowdown = math.pi/2
        self.reach_tolerance = 0.4
        self.angular_tolerance = 0.05
        
        # Waypoint sistemi
        self.waypoints = []
        self.wp_index = 0
        self.wp_active = False
        
        # Waypoint offsets - engelahmet.py'den
        x_add = [1.0, 2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0]
        y_add = [0.0, -1.2, -2.8, -3.6, -3.6, -2.8, -1.2, 0.0, 0.0, 0.0]
        self.wp_offsets = list(zip(x_add, y_add))
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/yoda/cmd_vel', 10)
        self.obst_pub = self.create_publisher(Bool, '/lidar/obstacle', 10)
        self.wp_pub = self.create_publisher(Bool, '/engel/waypoint', 10)

        # Subscribers
        self.create_subscription(LaserScan, '/yoda/lidar/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/yoda/odom', self.odom_callback, 10)

        # Timer - sürekli hareket ve analiz kontrolü
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        # Başlangıç hareketi
        self.move_forward()
        self.get_logger().info("Sürekli hareket ve engel algılama düğümü başlatıldı.")

    def odom_callback(self, msg: Odometry):
        """Odometry callback - pozisyon güncelleme"""
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.current_x, self.current_y = p.x, p.y
        _, _, self.current_yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])

        if self.wp_active:
            self.move_toward_waypoint()

    def timer_callback(self):
        """Timer callback - analiz kontrolü ve sürekli hareket"""
        if self.analysis_mode:
            current_time = self.get_clock().now()
            elapsed_time = (current_time - self.analysis_start_time).nanoseconds / 1e9
            
            if elapsed_time >= self.analysis_duration:
                self.complete_analysis()
        elif not self.obstacle_detected and not self.maneuvering:
            # Engel yoksa ve manevra yapmıyorsa ileri git
            self.move_forward()

    def scan_callback(self, msg: LaserScan):
        """Lidar verilerini işle"""
        ranges = np.array(msg.ranges)
        
        # Engeli tespit et
        obstacle_info = self.detect_crossing_obstacle(ranges, msg)
        
        if obstacle_info:
            if not self.obstacle_detected:
                self.get_logger().warn(f"Engel algılandı! Pozisyon: ({obstacle_info['x']:.2f}, {obstacle_info['y']:.2f})")
                self.stop_robot()
                self.obstacle_detected = True
                self.start_analysis()
            
            if self.analysis_mode:
                self.track_obstacle_movement(obstacle_info)
        else:
            if self.obstacle_detected and not self.maneuvering:
                self.get_logger().info("Engel geçti. Normal harekete devam.")
                self.reset_detection()
                self.move_forward()

    def detect_crossing_obstacle(self, ranges, msg):
        """Ön bölgede geçiş yapan engeli tespit et"""
        obstacles = []
        
        # Sadece ön bölgeyi tara (±28 derece)
        mid = len(ranges) // 2
        span = int(math.atan(1.85/5) / msg.angle_increment)  # 28 derece
        
        for i in range(mid - span, mid + span):
            if i < 0 or i >= len(ranges):
                continue
                
            distance = ranges[i]
            if not np.isnan(distance) and distance < self.min_obstacle_distance:
                angle = (i - mid) * msg.angle_increment
                obstacles.append({
                    'distance': distance,
                    'angle': angle,
                    'x': distance * math.cos(angle),
                    'y': distance * math.sin(angle)
                })
        
        if not obstacles:
            return None
        
        # En yakın engeli seç
        closest_obstacle = min(obstacles, key=lambda obs: obs['distance'])
        return closest_obstacle

    def start_analysis(self):
        """Engel analizi başlat"""
        self.analysis_mode = True
        self.analysis_start_time = self.get_clock().now()
        self.analysis_readings = 0
        self.current_positions.clear()
        self.initial_position = None
        self.last_position = None
        self.get_logger().info("Engel analizi başlatılıyor...")

    def track_obstacle_movement(self, obstacle_info):
        """Engelin hareketini takip et"""
        position = (obstacle_info['x'], obstacle_info['y'])
        
        # İlk pozisyonu kaydet
        if self.initial_position is None:
            self.initial_position = position
        
        # Mevcut pozisyonu ekle
        self.current_positions.append(position)
        self.last_position = position
        self.analysis_readings += 1
        
        # İlerleme bilgisi
        if self.analysis_readings % 5 == 0:
            current_time = self.get_clock().now()
            elapsed_time = (current_time - self.analysis_start_time).nanoseconds / 1e9
            remaining_time = self.analysis_duration - elapsed_time
            self.get_logger().info(f"Analiz: {self.analysis_readings} okuma, Kalan: {remaining_time:.1f}s")

    def complete_analysis(self):
        """Analizi tamamla ve geçiş yönünü belirle"""
        self.analysis_mode = False
        
        if self.analysis_readings < self.min_readings:
            self.get_logger().warn(f"Yeterli veri yok, analiz tekrarlanacak.")
            self.start_analysis()
            return
        
        # Geçiş analizi yap
        crossing_analysis = self.analyze_crossing_movement()
        
        # Sonuçları raporla
        self.report_crossing_results(crossing_analysis)
        
        # Geçiş stratejisi uygula
        self.apply_crossing_strategy(crossing_analysis)

    def analyze_crossing_movement(self):
        """Yan geçiş hareketini analiz et"""
        if not self.initial_position or not self.last_position:
            return None
        
        positions = list(self.current_positions)
        
        # Başlangıç ve bitiş pozisyonları
        start_pos = self.initial_position
        end_pos = self.last_position
        
        # Toplam hareket
        total_movement_x = end_pos[0] - start_pos[0]
        total_movement_y = end_pos[1] - start_pos[1]
        total_distance = math.sqrt(total_movement_x**2 + total_movement_y**2)
        
        # Hareket yönü
        movement_angle = math.atan2(total_movement_y, total_movement_x)
        
        # Y ekseni hareketi (yan hareket)
        lateral_movement = abs(total_movement_y)
        
        # X ekseni hareketi (öne/arkaya hareket)
        longitudinal_movement = total_movement_x
        
        # Ortalama pozisyon
        avg_x = np.mean([pos[0] for pos in positions])
        avg_y = np.mean([pos[1] for pos in positions])
        
        # Geçiş yönü belirleme
        if lateral_movement > self.crossing_threshold:
            if total_movement_y < 0:
                crossing_direction = "sol_to_sag"
                direction_text = "soldan sağa"
            else:
                crossing_direction = "sag_to_sol"
                direction_text = "sağdan sola"
            is_crossing = True
        else:
            crossing_direction = "statik"
            direction_text = "statik"
            is_crossing = False
        
        # Geçiş hızı
        crossing_speed = lateral_movement / self.analysis_duration
        
        # Güvenlik durumu
        if avg_x < 3.0:
            safety_status = "tehlikeli"
        elif avg_x < 5.0:
            safety_status = "dikkatli"
        else:
            safety_status = "güvenli"
        
        return {
            'is_crossing': is_crossing,
            'crossing_direction': crossing_direction,
            'direction_text': direction_text,
            'lateral_movement': lateral_movement,
            'longitudinal_movement': longitudinal_movement,
            'total_distance': total_distance,
            'movement_angle': movement_angle,
            'crossing_speed': crossing_speed,
            'avg_position': (avg_x, avg_y),
            'start_position': start_pos,
            'end_position': end_pos,
            'safety_status': safety_status,
            'data_points': len(positions)
        }

    def report_crossing_results(self, analysis):
        """Geçiş analizi sonuçlarını raporla"""
        if not analysis:
            self.get_logger().error("Analiz verisi yetersiz!")
            return
        
        self.get_logger().info("=" * 40)
        self.get_logger().info("ENGEL ANALİZİ SONUÇLARI")
        self.get_logger().info("=" * 40)
        
        # Temel bilgiler
        start = analysis['start_position']
        end = analysis['end_position']
        avg = analysis['avg_position']
        
        self.get_logger().info(f"Başlangıç: ({start[0]:.2f}, {start[1]:.2f}) m")
        self.get_logger().info(f"Bitiş: ({end[0]:.2f}, {end[1]:.2f}) m")
        self.get_logger().info(f"Ortalama: ({avg[0]:.2f}, {avg[1]:.2f}) m")
        
        # Hareket analizi
        self.get_logger().info(f"Yan Hareket: {analysis['lateral_movement']:.2f} m")
        self.get_logger().info(f"Geçiş Hızı: {analysis['crossing_speed']:.2f} m/s")
        
        # Sonuç
        if analysis['is_crossing']:
            self.get_logger().warn(f"SONUÇ: {analysis['direction_text']} geçiş")
        else:
            self.get_logger().info("SONUÇ: Statik engel")
        
        self.get_logger().info(f"Güvenlik: {analysis['safety_status']}")
        self.get_logger().info("=" * 40)

    def apply_crossing_strategy(self, analysis):
        """Geçiş stratejisini uygula"""
        if not analysis:
            return
        
        if analysis['is_crossing']:
            if analysis['crossing_direction'] == "sol_to_sag":
                self.get_logger().info("Strateji: Soldan sağa geçiş - Bekle")
            elif analysis['crossing_direction'] == "sag_to_sol":
                self.get_logger().info("Strateji: Sağdan sola geçiş - Bekle")
            
            if analysis['safety_status'] == "tehlikeli":
                self.get_logger().error("⚠️ Engel çok yakın!")
            
            self.get_logger().info("Engelin geçmesi bekleniyor...")
        else:
            self.get_logger().info("Strateji: Statik engel - Manevra başlatılıyor")
            # Statik engelse manevra başlat
            self.start_maneuver()

    def start_maneuver(self):
        """Manevra başlat - engelahmet.py'den alınan mantık"""
        if self.maneuvering or self.wp_active:
            return
        
        self.maneuvering = True
        yaw = self.current_yaw
        self.waypoints = []
        
        for dx_local, dy_local in self.wp_offsets:
            # Yerel offset'leri mevcut yaw'a göre döndür
            global_dx = dx_local * math.cos(yaw) + dy_local * math.sin(yaw)
            global_dy = dx_local * math.sin(yaw) - dy_local * math.cos(yaw)
            wx = self.current_x + global_dx
            wy = self.current_y + global_dy
            self.waypoints.append((wx, wy))

        self.wp_index = 0
        self.wp_active = True
        tx, ty = self.waypoints[0]
        self.get_logger().info(f"Manevra başlatıldı! Şu anki konum: ({self.current_x:.2f}, {self.current_y:.2f})")
        self.get_logger().info(f"Hedef waypoint 0: ({tx:.2f}, {ty:.2f})")
        self.wp_pub.publish(Bool(data=True))

    def move_toward_waypoint(self):
        """Waypoint'e doğru hareket et"""
        if not self.wp_active or self.wp_index >= len(self.waypoints):
            return
        
        tx, ty = self.waypoints[self.wp_index]
        dx, dy = tx - self.current_x, ty - self.current_y
        dist = math.hypot(dx, dy)

        # Hedefe ulaşma kontrolü
        if dist < self.reach_tolerance:
            self.get_logger().info(f"Waypoint {self.wp_index} ulaşıldı.")
            self.wp_index += 1
            if self.wp_index < len(self.waypoints):
                nx, ny = self.waypoints[self.wp_index]
                self.get_logger().info(f"Hedef waypoint {self.wp_index}: ({nx:.2f}, {ny:.2f})")
            else:
                self.get_logger().info("Tüm waypoint'ler tamamlandı. Normal harekete devam.")
                self.wp_active = False
                self.maneuvering = False
                self.wp_pub.publish(Bool(data=False))
                self.reset_detection()
                self.move_forward()
                return

        # Yön hatası
        desired_yaw = math.atan2(dy, dx)
        err = (desired_yaw - self.current_yaw + math.pi) % (2*math.pi) - math.pi
        if abs(err) < self.angular_tolerance:
            err = 0.0

        # Hız komutları
        v_base = min(self.speed_gain * dist, self.max_speed)
        slow_fac = max(0.0, 1 - abs(err)/self.turn_slowdown)
        linear = max(self.min_speed, v_base * slow_fac)
        angular = self.angular_gain * err

        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)

    def move_forward(self):
        """Aracı ileri hareket ettir"""
        if not self.wp_active:
            twist = Twist()
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        """Robotu durdur"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def reset_detection(self):
        """Engel algılama durumunu sıfırla"""
        self.obstacle_detected = False
        self.analysis_mode = False
        self.analysis_start_time = None
        self.analysis_readings = 0
        self.current_positions.clear()
        self.initial_position = None
        self.last_position = None

def main(args=None):
    rclpy.init(args=args)
    node = ContinuousMovementObstacleDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.get_logger().info("Düğüm kapatılıyor...")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()