import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import random

class CarBrakingSimulation(Node):
    def __init__(self):
        super().__init__('car_braking_simulation')
        self.min_speed = 0.0
        self.nominal_start_speed = 10.0  # Ortalama başlangıç hızı (km/h)
        self.speeds = {0.4: '25%', 0.8: '50%', 1.6: '100%'}
        self.tests = {}

    def run_test(self, base_deceleration):
        speeds = []
        times = []
        # Başlangıç hızı küçük bir rastgele sapmayla
        start_speed = self.nominal_start_speed + random.uniform(-0.5, 0.5)
        speed = start_speed
        time = 0.0

        while speed > self.min_speed:
            speeds.append(speed)
            times.append(time)

            # Her adımda yavaşlamaya küçük rastgele bir değişim ekleyelim
            deceleration_variation = random.uniform(-0.05, 0.05)
            effective_deceleration = base_deceleration + deceleration_variation
            speed -= effective_deceleration * time

            if speed < 0:
                speed = 0

            time += 0.1  # 100ms

        speeds.append(self.min_speed)
        times.append(time)
        return times, speeds

    def execute_tests(self):
        for deceleration, pedal_state in self.speeds.items():
            times, speeds = self.run_test(deceleration)
            self.tests[pedal_state] = (times, speeds)

    def plot_results(self):
        plt.figure()
        for pedal_state, (times, speeds) in self.tests.items():
            plt.plot(times, speeds, label=f'{pedal_state} Fren Pedalı')
        plt.xlabel('Zaman (s)')
        plt.ylabel('Hız (km/h)')
        plt.title('Hız-Zaman Grafiği')
        plt.legend()
        plt.grid(True)
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    car_braking_simulation = CarBrakingSimulation()

    car_braking_simulation.execute_tests()
    car_braking_simulation.plot_results()

    car_braking_simulation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

