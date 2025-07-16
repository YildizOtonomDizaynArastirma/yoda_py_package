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
class ContinuousMovementObstacleDetector(Node):
    def __init__(self):
        super().__init__('continuous_movement_obstacle_detector')

        # Temel parametreler
        self.min_obstacle_distance = 5.0  # meters
        self.analysis_duration = 2.5  # saniye - analiz süresi
        self.crossing_threshold = 0.3  # metre - yan hareket eşiği
        self.min_readings = 8  # minimum okuma sayısı
        self.grouping_threshold = 0.5  # metre - aynı engelin parçası kabul edilecek mesafe
        self.linear_speed=80.0
        # Durum değişkenleri
        self.obstacle_detected = False
        self.analysis_mode = False
        self.analysis_start_time = None
        self.analysis_readings = 0
        
        # Sadece mevcut engel verisi
        self.current_positions = deque(maxlen=15)
        self.initial_position = None
        self.last_position = None
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/yoda/cmd_vel', 10)
        self.obst_pub = self.create_publisher(Bool, '/lidar/obstacle', 10)

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Timer - sürekli hareket ve analiz kontrolü
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        # Başlangıç hareketi
        self.move_forward()
        self.get_logger().info("Sürekli hareket ve engel algılama düğümü başlatıldı.")

    def timer_callback(self):
        """Timer callback - analiz kontrolü ve sürekli hareket"""
        if self.analysis_mode:
            current_time = self.get_clock().now()
            elapsed_time = (current_time - self.analysis_start_time).nanoseconds / 1e9
            
            if elapsed_time >= self.analysis_duration:
                self.complete_analysis()
        elif not self.obstacle_detected:
            self.move_forward()

    def scan_callback(self, msg: LaserScan):
        """Lidar verilerini işle"""
        ranges = np.array(msg.ranges)
        
        # Engeli tespit et
        obstacle_info = self.detect_crossing_obstacle(ranges, msg)
        
        if obstacle_info:
            if not self.obstacle_detected:
                self.get_logger().warn(f"Engel algılandı! Ortalama pozisyon: ({obstacle_info['x']:.2f}, {obstacle_info['y']:.2f})")
                self.stop_robot()
                self.obstacle_detected = True
                self.start_analysis()
            
            if self.analysis_mode:
                self.track_obstacle_movement(obstacle_info)
        else:
            if self.obstacle_detected:
                self.get_logger().info("Engel geçti. Normal harekete devam.")
                self.reset_detection()
                self.move_forward()

    def detect_crossing_obstacle(self, ranges, msg):
        """Ön bölgede geçiş yapan engeli tespit et - Ortalama pozisyon analizi"""
        obstacles = []
        
        # Sadece ön bölgeyi tara (±28 derece)
        mid = len(ranges) // 2
        span = int(math.pi/6 / msg.angle_increment)  # 30 derece
        
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
                    'y': distance * math.sin(angle),
                    'index': i
                })
        
        if not obstacles:
            return None
        
        # Engel gruplaması ve ortalama pozisyon hesaplama
        obstacle_groups = self.group_obstacles(obstacles)
        
        if not obstacle_groups:
            return None
        
        # En büyük grubu seç (en fazla noktaya sahip grup)
        main_group = max(obstacle_groups, key=len)
        
        # Grubun ortalama pozisyonunu hesapla
        avg_x = np.mean([obs['x'] for obs in main_group])
        avg_y = np.mean([obs['y'] for obs in main_group])
        avg_distance = np.mean([obs['distance'] for obs in main_group])
        avg_angle = np.mean([obs['angle'] for obs in main_group])
        
        # Grubun genişliği (lateral spread)
        y_positions = [obs['y'] for obs in main_group]
        lateral_spread = max(y_positions) - min(y_positions)
        
        # Grup büyüklüğü
        group_size = len(main_group)
        
        return {
            'x': avg_x,
            'y': avg_y,
            'distance': avg_distance,
            'angle': avg_angle,
            'lateral_spread': lateral_spread,
            'group_size': group_size,
            'raw_obstacles': main_group
        }

    def group_obstacles(self, obstacles):
        """Engel noktalarını grupla - yakın noktalar aynı engeli temsil eder"""
        if not obstacles:
            return []
        
        # Engelleri x koordinatına göre sırala
        obstacles.sort(key=lambda obs: obs['x'])
        
        groups = []
        current_group = [obstacles[0]]
        
        for i in range(1, len(obstacles)):
            current_obs = obstacles[i]
            last_in_group = current_group[-1]
            
            # Mesafe kontrolü (hem x hem y ekseni)
            distance_to_group = math.sqrt(
                (current_obs['x'] - last_in_group['x'])**2 + 
                (current_obs['y'] - last_in_group['y'])**2
            )
            
            if distance_to_group <= self.grouping_threshold:
                # Aynı gruba ekle
                current_group.append(current_obs)
            else:
                # Yeni grup başlat
                if len(current_group) >= 2:  # En az 2 nokta olmalı
                    groups.append(current_group)
                current_group = [current_obs]
        
        # Son grubu ekle
        if len(current_group) >= 2:
            groups.append(current_group)
        
        return groups

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
        """Engelin hareketini takip et - Geliştirilmiş versiyon"""
        # Ortalama pozisyon
        avg_position = (obstacle_info['x'], obstacle_info['y'])
        
        # İlk pozisyonu kaydet
        if self.initial_position is None:
            self.initial_position = avg_position
            self.get_logger().info(f"İlk engel pozisyonu: ({avg_position[0]:.2f}, {avg_position[1]:.2f})")
            self.get_logger().info(f"Engel genişliği: {obstacle_info['lateral_spread']:.2f}m, "
                                  f"Nokta sayısı: {obstacle_info['group_size']}")
        
        # Mevcut pozisyonu ekle
        self.current_positions.append(avg_position)
        self.last_position = avg_position
        self.analysis_readings += 1
        
        # İlerleme bilgisi (daha detaylı)
        if self.analysis_readings % 5 == 0:
            current_time = self.get_clock().now()
            elapsed_time = (current_time - self.analysis_start_time).nanoseconds / 1e9
            remaining_time = self.analysis_duration - elapsed_time
            
            # Anlık hareket hızı hesapla
            if len(self.current_positions) >= 2:
                last_pos = self.current_positions[-2]
                current_pos = self.current_positions[-1]
                instant_speed = math.sqrt(
                    (current_pos[0] - last_pos[0])**2 + 
                    (current_pos[1] - last_pos[1])**2
                ) / 0.1  # 0.1 saniye timer aralığı
                
                self.get_logger().info(f"Analiz: {self.analysis_readings} okuma, "
                                      f"Kalan: {remaining_time:.1f}s, "
                                      f"Anlık hız: {instant_speed:.2f}m/s")

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
        """Yan geçiş hareketini analiz et - Geliştirilmiş versiyon"""
        if not self.initial_position or not self.last_position:
            return None
        
        positions = list(self.current_positions)
        
        if len(positions) < 3:
            return None
        
        # Başlangıç ve bitiş pozisyonları
        start_pos = self.initial_position
        end_pos = self.last_position
        
        # Toplam hareket
        total_movement_x = end_pos[0] - start_pos[0]
        total_movement_y = end_pos[1] - start_pos[1]
        total_distance = math.sqrt(total_movement_x**2 + total_movement_y**2)
        
        # Hareket vektörü analizi
        movement_angle = math.atan2(total_movement_y, total_movement_x)
        
        # Pozisyon istatistikleri
        x_positions = [pos[0] for pos in positions]
        y_positions = [pos[1] for pos in positions]
        
        # Ortalama pozisyon
        avg_x = np.mean(x_positions)
        avg_y = np.mean(y_positions)
        
        # Standart sapma (hareket tutarlılığı)
        std_x = np.std(x_positions)
        std_y = np.std(y_positions)
        
        # Y ekseni hareketi (yan hareket)
        lateral_movement = abs(total_movement_y)
        lateral_range = max(y_positions) - min(y_positions)
        
        # X ekseni hareketi (öne/arkaya hareket)
        longitudinal_movement = total_movement_x
        longitudinal_range = max(x_positions) - min(x_positions)
        
        # Hareket tutarlılığı (düz hat mı yoksa zikzak mı?)
        movement_consistency = self.calculate_movement_consistency(positions)
        
        # Geçiş yönü belirleme (daha hassas)
        crossing_threshold_adaptive = max(self.crossing_threshold, std_y * 2)
        
        if lateral_movement > crossing_threshold_adaptive:
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
        
        # Geçiş hızı (ortalama ve maksimum)
        crossing_speed_avg = lateral_movement / self.analysis_duration
        crossing_speed_max = lateral_range / self.analysis_duration
        
        # Güvenlik durumu (daha detaylı)
        min_distance = min([pos[0] for pos in positions])
        if min_distance < 2.0:
            safety_status = "çok_tehlikeli"
        elif min_distance < 3.0:
            safety_status = "tehlikeli"
        elif min_distance < 5.0:
            safety_status = "dikkatli"
        else:
            safety_status = "güvenli"
        
        return {
            'is_crossing': is_crossing,
            'crossing_direction': crossing_direction,
            'direction_text': direction_text,
            'lateral_movement': lateral_movement,
            'lateral_range': lateral_range,
            'longitudinal_movement': longitudinal_movement,
            'longitudinal_range': longitudinal_range,
            'total_distance': total_distance,
            'movement_angle': movement_angle,
            'crossing_speed_avg': crossing_speed_avg,
            'crossing_speed_max': crossing_speed_max,
            'avg_position': (avg_x, avg_y),
            'position_std': (std_x, std_y),
            'start_position': start_pos,
            'end_position': end_pos,
            'safety_status': safety_status,
            'data_points': len(positions),
            'movement_consistency': movement_consistency,
            'min_distance': min_distance,
            'crossing_threshold_used': crossing_threshold_adaptive
        }

    def calculate_movement_consistency(self, positions):
        """Hareket tutarlılığını hesapla (0-1 arası, 1 = çok tutarlı)"""
        if len(positions) < 3:
            return 0.0
        
        # Ardışık pozisyonlar arası açı değişimlerini hesapla
        angle_changes = []
        
        for i in range(1, len(positions) - 1):
            p1 = positions[i-1]
            p2 = positions[i]
            p3 = positions[i+1]
            
            # İki vektörün açısını hesapla
            v1 = (p2[0] - p1[0], p2[1] - p1[1])
            v2 = (p3[0] - p2[0], p3[1] - p2[1])
            
            # Açı farkını hesapla
            angle1 = math.atan2(v1[1], v1[0])
            angle2 = math.atan2(v2[1], v2[0])
            
            angle_diff = abs((angle2 - angle1 + math.pi) % (2*math.pi) - math.pi)
            angle_changes.append(angle_diff)
        
        if not angle_changes:
            return 1.0
        
        # Ortalama açı değişimi (radyan)
        avg_angle_change = np.mean(angle_changes)
        
        # Tutarlılık skoru (düşük açı değişimi = yüksek tutarlılık)
        consistency = max(0.0, 1.0 - (avg_angle_change / (math.pi/4)))
        
        return consistency

    def report_crossing_results(self, analysis):
        """Geçiş analizi sonuçlarını raporla - Geliştirilmiş versiyon"""
        if not analysis:
            self.get_logger().error("Analiz verisi yetersiz!")
            return
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("DETAYLI ENGEL ANALİZİ SONUÇLARI")
        self.get_logger().info("=" * 50)
        
        # Temel pozisyon bilgileri
        start = analysis['start_position']
        end = analysis['end_position']
        avg = analysis['avg_position']
        pos_std = analysis['position_std']
        
        self.get_logger().info(f"Başlangıç pozisyonu: ({start[0]:.2f}, {start[1]:.2f}) m")
        self.get_logger().info(f"Bitiş pozisyonu: ({end[0]:.2f}, {end[1]:.2f}) m")
        self.get_logger().info(f"Ortalama pozisyon: ({avg[0]:.2f}, {avg[1]:.2f}) m")
        self.get_logger().info(f"Pozisyon std sapması: ({pos_std[0]:.2f}, {pos_std[1]:.2f}) m")
        
        # Hareket analizi
        self.get_logger().info("-" * 30)
        self.get_logger().info("HAREKET ANALİZİ:")
        self.get_logger().info(f"Yan hareket (toplam): {analysis['lateral_movement']:.2f} m")
        self.get_logger().info(f"Yan hareket (aralık): {analysis['lateral_range']:.2f} m")
        self.get_logger().info(f"Boyuna hareket: {analysis['longitudinal_movement']:.2f} m")
        self.get_logger().info(f"Toplam mesafe: {analysis['total_distance']:.2f} m")
        
        # Hız analizi
        self.get_logger().info("-" * 30)
        self.get_logger().info("HIZ ANALİZİ:")
        self.get_logger().info(f"Ortalama geçiş hızı: {analysis['crossing_speed_avg']:.2f} m/s")
        self.get_logger().info(f"Maksimum geçiş hızı: {analysis['crossing_speed_max']:.2f} m/s")
        
        # Tutarlılık analizi
        consistency_percent = analysis['movement_consistency'] * 100
        self.get_logger().info(f"Hareket tutarlılığı: {consistency_percent:.1f}%")
        
        # Güvenlik analizi
        self.get_logger().info("-" * 30)
        self.get_logger().info("GÜVENLİK ANALİZİ:")
        self.get_logger().info(f"Minimum mesafe: {analysis['min_distance']:.2f} m")
        self.get_logger().info(f"Güvenlik durumu: {analysis['safety_status']}")
        self.get_logger().info(f"Kullanılan eşik: {analysis['crossing_threshold_used']:.2f} m")
        
        # Sonuç
        self.get_logger().info("-" * 30)
        if analysis['is_crossing']:
            self.get_logger().warn(f"🚨 SONUÇ: {analysis['direction_text']} geçiş tespit edildi!")
        else:
            self.get_logger().info("✅ SONUÇ: Statik engel")
        
        self.get_logger().info(f"Veri noktası sayısı: {analysis['data_points']}")
        self.get_logger().info("=" * 50)

    def apply_crossing_strategy(self, analysis):
        """Geçiş stratejisini uygula"""
        if not analysis:
            return
        
        if analysis['is_crossing']:
            if analysis['crossing_direction'] == "sol_to_sag":
                self.get_logger().info("Strateji: Soldan sağa geçiş - Bekle")
            elif analysis['crossing_direction'] == "sag_to_sol":
                self.get_logger().info("Strateji: Sağdan sola geçiş - Bekle")
            
            if analysis['safety_status'] == "çok_tehlikeli":
                self.get_logger().error("⚠️ Engel çok yakın!")
            elif analysis['safety_status'] == "tehlikeli":
                self.get_logger().error("⚠️ Engel tehlikeli mesafede!")
            
            self.get_logger().info("Engelin geçmesi bekleniyor...")
        else:
            self.get_logger().info("Strateji: Statik engel - Manevra başlatılıyor")
    def move_forward(self):
        """Aracı ileri hareket ettir"""
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