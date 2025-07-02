import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import random

class CarSimulation(Node):
    def __init__(self):
        super().__init__('car_simulation')
        self.nominal_max_speed = 10.0  # Ortalama max hız (km/h)
        self.speeds = {0.2: '25%', 0.4: '50%', 0.8: '100%'}
        self.tests = {}

    def run_test(self, base_acceleration):
        speeds = []
        times = []
        speed = 0.0
        time = 0.0

        # Her test için max speed'e küçük bir rastgele sapma ekleyelim
        max_speed = self.nominal_max_speed + random.uniform(-0.5, 0.5)

        while speed < max_speed:
            speeds.append(speed)
            times.append(time)

            # Her adımda ivmeye küçük rastgele bir değişim ekleyelim
            acceleration_variation = random.uniform(-0.05, 0.05)
            effective_acceleration = base_acceleration + acceleration_variation
            speed += effective_acceleration * time

            if speed > max_speed:
                speed = max_speed

            time += 0.1  # 100ms

        speeds.append(max_speed)
        times.append(time)
        return times, speeds

    def execute_tests(self):
        for acceleration, pedal_state in self.speeds.items():
            times, speeds = self.run_test(acceleration)
            self.tests[pedal_state] = (times, speeds)

    def plot_results(self):
        plt.figure()
        for pedal_state, (times, speeds) in self.tests.items():
            plt.plot(times, speeds, label=f'{pedal_state} Gaz Pedalı')
        plt.xlabel('Zaman (s)')
        plt.ylabel('Hız (km/h)')
        plt.title('Hız-Zaman Grafiği')
        plt.legend()
        plt.grid(True)
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    car_simulation = CarSimulation()

    car_simulation.execute_tests()
    car_simulation.plot_results()

    car_simulation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
