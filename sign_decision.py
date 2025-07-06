#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool 
from std_msgs.msg import Float32
from std_msgs.msg import Int32 
import time
import math

class SignDecision(Node):   

    def __init__(self):    
        super().__init__("sign_decision_node")
        
        self.sign_ID = 'id_topic'  # Yolodan gelecek olan tabela verileri için topic adı.
        self.imu_rate = '/yoda/imu/data' #imu verilerinin topiği.
        #self.traffic_sign = '' #trafik işaretinin topiği
        self.decision = None  # Tabela kararını tutmak için
        self.initial_linear_velocity = 0.5  # Başlangıç hızı
        self.angular_velocity = 0.3  # Açısal hız
        self.turning_linear_velocity = 0.5  # Dönüş sırasındaki lineer hız artırıldı
        self.max_turn_angle = 90  # Maksimum dönüş açısı
        self.yaw_angle = 0.0  # Mevcut yaw açısı
        self.initial_yaw = None  # Dönüşe başlamadan önceki açı
        self.is_turning = False  # Dönüş durumunu takip etmek için flag
        self.turn_complete = False  # Dönüşün tamamlandığını takip etmek için flag
        
        self.traffic_light_id = None


        # Subscription'lar
        self.sign_subscription = self.create_subscription(
            Float32,
            self.sign_ID,
            self.sign_ID_listener_callback,
            10)
        
        #Bu kısım düzeltilecek!!!!!
        self.create_subscription(
            Int32, 
            '/traffic_light_id', 
            self.traffic_light_callback, 
            10)

        self.publisher_ = self.create_publisher(Twist, '/yoda/cmd_vel', 10)


        self.maneuver_pub = self.create_publisher(Bool, '/start_maneuver', 10)    #Durak ile ilgili
        self.park_pub = self.create_publisher(Bool, '/start_parking', 10)       #park ile ilgili
        

        self.imu_subscription = self.create_subscription(
            Imu,                   
            self.imu_rate,
            self.imu_listener_callback,
            10)
        
        
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        # Başlangıçta hareket et
        self.start_moving()
    
    #Durak için
    def maneuver_for_station(self):
        msg = Bool()
        msg.data = True
        self.maneuver_pub.publish(msg)
        
    #Park için
    def start_parking(self):
        msg = Bool()
        msg.data = True
        self.park_pub.publish(msg)


    #Trafik işareti için
    def traffic_light_callback(self, msg):
        self.traffic_light_id = msg.data
        self.get_logger().info(f"Trafik ışığı algılandı: {self.traffic_light_id}")


    def start_moving(self):
        """Başlangıçta aracı ileri hareket ettirir."""
        start_msg = Twist()
        start_msg.linear.x = self.initial_linear_velocity  
        start_msg.angular.z = 0.0
        self.publisher_.publish(start_msg)
        self.get_logger().info(f"Araç {self.initial_linear_velocity} hızla ileri gidiyor")

    def sign_ID_listener_callback(self, msg):
        """Tabela ID verisi geldiğinde çağrılır."""
        self.get_logger().info(f'Alinan veri: {msg.data}')     
        self.decision = msg.data
        # Karar verme işlemini başlat
        self.decision_part()

    def imu_listener_callback(self, msg):
        """IMU'dan gelen açısal verileri işler."""
        # imudan alınan Quaternion değerleri kullanarak yaw açısını hesapladık.
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        current_yaw = math.atan2(siny_cosp, cosy_cosp) * (180 / math.pi)  # Dereceye çevir
        
        # Açıyı -180 ile 180 arasında tutarak hatalı dönüşler yapmasını önledik. Bunu yukarıdak hesaplamalarla yaptık
        self.yaw_angle = current_yaw
        
        # IMU verisi debug için loglama
        if self.is_turning:
            self.get_logger().info(f"Yaw: {self.yaw_angle:.2f}, Başlangıç: {self.initial_yaw:.2f}")



    def timer_callback(self):
        """Dönüş durumunu düzenli olarak kontrol eder."""
        #initial_yaw imu verisi geldiğinde none olmaktan çıkar. Yani imu_listener_callback.te değişir.
        if self.is_turning and self.initial_yaw is not None: 
            # Başlangıç açısından farkı hesapla
            angle_diff = self.calculate_angle_diff(self.initial_yaw, self.yaw_angle)
            
            # dönüş açısı farkını logluyoruz.
            self.get_logger().info(f"Dönüş: {angle_diff:.2f} derece, Hedef: 90 derece")
            
            if angle_diff >= 85:  # 90 derece yerine biraz tolerans bıraktık
                # Dönüşü durdur ve düz hareket etmeye başla
                self.get_logger().info("Dönüş tamamlandı! Düz harekete geçiliyor.")
                self.is_turning = False
                self.turn_complete = True
                
                # Düz harekete geçir
                straight_msg = Twist()
                straight_msg.linear.x = self.initial_linear_velocity
                straight_msg.angular.z = 0.0
                self.publisher_.publish(straight_msg)
    
    def calculate_angle_diff(self, start_angle, current_angle):
        """İki açı arasındaki farkı hesaplar (sola dönüş için pozitif)."""
        diff = current_angle - start_angle
        
        # 180 derece sınırını aşma durumlarını düzelt
        if diff < -180:
            diff += 360
        elif diff > 180:
            diff -= 360
            
        # Sola dönüş için mutlak değeri al (pozitif olmalı) 
        #Bunu pozitif olarak alıyoruz sonrasında sağa dönüş için direkt eksi koyarak ayarladık.
        #Daha sonrasında bu farkı timer_callback'e gönderdik ve angle_diff.e atadık.
        return abs(diff)

    def decision_part(self):
        """Tabela ID'sine göre karar verir ve uygun eylemi başlatır."""
        if self.decision == 0.0:
            self.get_logger().info("duz devam et")
            self.start_moving()


        elif self.decision == 1.0:
            self.get_logger().info("dur")
            self.stop_moving()

            
        elif self.decision == 2.0:
            self.get_logger().info("sola mecburi yon")
            self.turn_left()

            
        elif self.decision == 3.0:
            self.get_logger().info("saga mecburi yon")
            self.turn_right()
        
        
        elif self.decision == 4.0:
            self.get_logger().info("ileri ve sola mecburi yon")
            call = 1   # Sola dönüş için 1, düz devam etmek için 0 girilecek.

            if call == 1:
                self.turn_left()
            elif call == 0:
                self.get_logger().info("duz devam et")


        elif self.decision == 5.0:
            self.get_logger().info("ileri ve saga mecburi yon")
            call = 1  # Sağa dönüş için 1, düz devam etmek için 0 girilecek.

            if call == 1:
                self.turn_right()
            elif call == 0:
                self.get_logger().info("duz devam et")


        elif self.decision == 6.0:
            if self.traffic_light_id == 0:
                self.get_logger().info("🔴 Kırmızı ışık – DUR")
                self.stop_moving()
            elif self.traffic_light_id == 1:
                self.get_logger().info("🟢 Yeşil ışık – HAREKET ET")
                self.start_moving()          
                
        elif self.decision == 7.0:
            self.get_logger().info("Durak")
            self.maneuver_for_station()

        elif self.decision == 8.0:
            self.get_logger().info("park et")
            self.start_parking()
    

        else:
            self.get_logger().info(f"Tanımlanmamış karar kodu: {self.decision}")



    def turn_left(self):
        """Aracı sola 90 derece döndürür."""
        if self.is_turning:
            self.get_logger().info("Zaten dönüş yapılıyor, bu komut atlandı.")
            return
            
        self.get_logger().info("Araç sola dönmeye başlıyor...")
        
        # Dönüş öncesi mevcut açıyı kaydet
        self.initial_yaw = self.yaw_angle
        self.is_turning = True
        self.turn_complete = False
        
        # ÖNEMLI: Net ve güçlü bir dönüş komutu gönder
        turn_msg = Twist()
        turn_msg.linear.x = self.turning_linear_velocity  # Dönüş sırasında 0.5 hızla ilerle
        turn_msg.angular.z = self.angular_velocity  # Pozitif değer (sola dönüş)
        
        # Komutu birkaç kez gönder (güvence için)
        for _ in range(3):
            self.publisher_.publish(turn_msg)
            time.sleep(0.1)
        
        self.get_logger().info(f"Dönüş komutu gönderildi! Linear.x: {turn_msg.linear.x}, Angular.z: {turn_msg.angular.z}")
        

    def turn_right(self):
        """Aracı sola 90 derece döndürür."""
        if self.is_turning:
            self.get_logger().info("Zaten dönüş yapılıyor, bu komut atlandı.")
            return
            
        self.get_logger().info("Araç sağa dönmeye başlıyor...")
        
        # Dönüş öncesi mevcut açıyı kaydet
        self.initial_yaw = self.yaw_angle
        self.is_turning = True
        self.turn_complete = False
        
        # ÖNEMLI: Net ve güçlü bir dönüş komutu gönder
        turn_msg = Twist()
        turn_msg.linear.x = self.turning_linear_velocity  # Dönüş sırasında 0.5 hızla ilerle
        turn_msg.angular.z = -self.angular_velocity  # negatif değer (sağa dönüş)
        
        # Komutu birkaç kez gönder (güvence için)
        for _ in range(3):
            self.publisher_.publish(turn_msg)
            time.sleep(0.1)
        
        self.get_logger().info(f"Dönüş komutu gönderildi! Linear.x: {turn_msg.linear.x}, Angular.z: {turn_msg.angular.z}")


    def stop_moving(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.publisher_.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args) 
    node = SignDecision()   
    rclpy.spin(node) 
    node.destroy_node()
    rclpy.shutdown() 


if __name__ == "__main__":
    main()
