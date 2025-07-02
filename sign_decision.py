#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
#from example_interfaces.srv import Trigger     # durak noduyla aynı servis tipi ve yolu girilecek
#from example_interfaces.srv import Trigger     # park noduyla aynı servis tipi ve yolu girilecek
#from example_interfaces.srv import Trigger     # dönel kavşak noduyla aynı servis tipi ve yolu girilecek
from std_msgs.msg import Float32
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
        
        # Subscription'lar
        self.sign_subscription = self.create_subscription(
            Float32,
            self.sign_ID,
            self.sign_ID_listener_callback,
            10)

        self.publisher_ = self.create_publisher(Twist, '/yoda/cmd_vel', 10)

        self.imu_subscription = self.create_subscription(
            Imu,                   
            self.imu_rate,
            self.imu_listener_callback,
            10)
        
        #self.traffic_sign_subscription = self.create_subscription(
            #<Gelecek veri tipi>,                       verinin sınıfını ekle
            #self.traffic_sign,                         sadece yorum satırını sil
            #self.traffic_sign_listener_callback,       sadece yorum satırını sil
            #10                                         sadece yorum satırını sil
        #    )
        
        
        # self.station_client = self.create_client(Trigger, 'station_service')     #servis tipi ve servis adı girilecek
        # while not self.station_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Waiting for Station Node (Service)...')

        # self.park_client = self.create_client(Trigger, 'park_service')     #servis tipi ve servis adı girilecek
        # while not self.park_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Waiting for Park Node (Service)...')

        # self.traffic_circle_client = self.create_client(Trigger, 'traffic_circle_service')     #servis tipi ve servis adı girilecek
        # while not self.traffic_circle_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Waiting for traffic circle Node (Service)...')

        # Düzenli kontroler için timer eklendi. 0.5 saniyede bir timer_callback fonksiyonunu çalıştıracak.
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        # Başlangıçta hareket et
        self.start_moving()
    
    #def send_station_request(self):
    #    station_request = Trigger.Request()             #bu kısmı düzenle, servis tipi
    #    station_future = self.station_client.call_async(station_request)    
    #    station_future.add_done_callback(self.station_callback)
    #def station_callback(self, station_future):
    #    response = station_future.result()   
    #    self.get_logger().info(f'Received from station node: {response.message}')     #tekrar doldurulacak
    #    rclpy.shutdown()
        

    # def send_park_request(self):
    #     park_request = Trigger.Request()             #bu kısmı düzenle, servis tipi
    #     park_future = self.park_client.call_async(park_request)    
    #     park_future.add_done_callback(self.park_callback)
    # def park_callback(self, park_future):
    #     response = park_future.result()   
    #     self.get_logger().info(f'Received from park node: {response.message}')     #tekrar doldurulacak
    #     rclpy.shutdown()


    # def send_traffic_circle_request(self):
    #     traffic_circle_request = Trigger.Request()             #bu kısmı düzenle, servis tipi
    #     traffic_circle_future = self.traffic_circle_client.call_async(traffic_circle_request)    
    #     traffic_circle_future.add_done_callback(self.traffic_circle_callback)
    # def traffic_circle_callback(self, traffic_circle_future):
    #     response = traffic_circle_future.result()   
    #     self.get_logger().info(f'Received from traffic circle node: {response.message}')     #tekrar doldurulacak
    #     rclpy.shutdown()



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

    #def traffic_sign_listener_callback(self, msg):
        #self.get_logger().info(f'Alinan veri: {msg.<data gibi bir veri gelecek artık ne çağırıyorsa>}')
        #self.light = msg.<data gibi bir veri gelecek artık ne çağırıyorsa>

        #if self.light == <kırmızı>:
            #self.light_id = 0
        #elif self.light == <yeşil>:
            #self.light_id = 1



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
            #call = self.light_id  #Aşağıdaki call silinecek ve bu satırın yorumu açılacak.
            call = 0        #Buraya trafik ışığı verisi gelecek. 0 ise kırmızı, dur. 1 ise yeşil, geç. 
            if call == 0:
                self.get_logger().info("dur")
                self.stop_moving()
            elif call == 1:
                self.get_logger().info("duz devam et")
                self.start_moving()            
                
        elif self.decision == 7.0:
            self.get_logger().info("Calling the station service...")
            self.send_station_request()

        elif self.decision == 8.0:
            self.get_logger().info("Calling the park service...")
            self.send_park_request()
        
        elif self.decision == 9.0:
            self.get_logger().info("Calling the traffic circle service...")
            self.send_traffic_circle_request()

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
