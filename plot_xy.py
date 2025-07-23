#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class PositionLogger(Node):
    def __init__(self):
        super().__init__('position_logger')

        # Live trajectory points (updating)
        self.x_points = []
        self.y_points = []

        # Initial trajectory points (static, for comparison)
        self.x_points_initial = [0, 1, 2, 3, 4, 5]                  #buraya hedef waypointler girilecek, baslangic cizgisi cizilecek
        self.y_points_initial = [0, 1, 1, 2, 2, 3]

        self.current_x = 0.0
        self.current_y = 0.0

        self.lock = threading.Lock()  # To safely access lists from multiple threads

        # Subscribe to odometry topic
        self.create_subscription(Odometry, '/zed/zed_node/odom', self.odom_callback, 10)

        # Timer to append position every 0.5 seconds
        self.create_timer(0.5, self.log_position)                           #zaman degistirilebilir, 0.5s dedim

        # Start matplotlib plotting in a separate thread
        plot_thread = threading.Thread(target=self.plot_positions)
        plot_thread.daemon = True
        plot_thread.start()

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        with self.lock:
            self.current_x = p.x
            self.current_y = p.y

    def log_position(self):
        with self.lock:
            self.x_points.append(self.current_x)
            self.y_points.append(self.current_y)

    def plot_positions(self):
        plt.style.use('seaborn-darkgrid')
        fig, ax = plt.subplots()
        # Two lines: one for initial trajectory, one for live trajectory
        line_initial, = ax.plot([], [], 'y--', label='Initial Trajectory')
        line_live, = ax.plot([], [], 'g-', label='Live Trajectory')

        ax.set_xlabel('X position')
        ax.set_ylabel('Y position')
        ax.set_title('Live Position Plot with Initial Trajectory')
        ax.legend()
        ax.grid(True)

        def init():
            ax.set_xlim(-20, 20)  # Fixed x limits                  #grafigin x ve y boyutunu -20,20 olarak sinirlar
            ax.set_ylim(-20, 20)  # Fixed y limits
            line_initial.set_data([], [])
            line_live.set_data([], [])
            return line_initial, line_live

        def update(frame):
            with self.lock:
                xdata_live = self.x_points.copy()
                ydata_live = self.y_points.copy()

                xdata_init = self.x_points_initial
                ydata_init = self.y_points_initial

            # Set data for initial trajectory (static)
            if xdata_init and ydata_init:
                line_initial.set_data(xdata_init, ydata_init)

            # Set data for live trajectory (updating)
            if xdata_live and ydata_live:
                line_live.set_data(xdata_live, ydata_live)

            return line_initial, line_live

        ani = FuncAnimation(fig, update, init_func=init, blit=True, interval=500)
        plt.show()


def main():
    rclpy.init()
    node = PositionLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
