#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import os
import time

if os.name == 'nt':
    import msvcrt
else:
    import tty
    import termios

BURGER_MAX_LIN_VEL = 10.00
BURGER_MAX_ANG_VEL = 10.00

WAFFLE_MAX_LIN_VEL = 10.00
WAFFLE_MAX_ANG_VEL = 10.00

LIN_VEL_STEP_SIZE = 0.1
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your Automobile
---------------------------
Moving around:
        w
   a    s    d
        x

space key, s : force stop

CTRL-C to quit
"""

class TeleopKey(Node):
    def __init__(self):
        super().__init__('teleop_key')
        self.publisher_ = self.create_publisher(Twist, '/yoda/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.turtlebot3_model = self.declare_parameter('model', 'burger').value

        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.control_linear_vel = 0.0
        self.control_angular_vel = 0.0

        if os.name != 'nt':
            self.settings = termios.tcgetattr(sys.stdin)
        
        print(msg)

    def getKey(self):
        if os.name == 'nt':
            return msvcrt.getch().decode() if msvcrt.kbhit() else ''

        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def timer_callback(self):
        key = self.getKey()
        if key:
            self.process_key(key)
        self.publish_velocity()

    def process_key(self, key):
        if key == 'w':
            self.target_linear_vel = self.checkLinearLimitVelocity(self.target_linear_vel + LIN_VEL_STEP_SIZE)
        elif key == 'x':
            self.target_linear_vel = self.checkLinearLimitVelocity(self.target_linear_vel - LIN_VEL_STEP_SIZE)
        elif key == 'a':
            self.target_angular_vel = self.checkAngularLimitVelocity(self.target_angular_vel + ANG_VEL_STEP_SIZE)
        elif key == 'd':
            self.target_angular_vel = self.checkAngularLimitVelocity(self.target_angular_vel - ANG_VEL_STEP_SIZE)
        elif key == ' ' or key == 's':
            self.target_linear_vel = 0.0
            self.target_angular_vel = 0.0
        elif key == '\x03':
            self.destroy_node()
            rclpy.shutdown()

        print(self.vels(self.target_linear_vel, self.target_angular_vel))

    def publish_velocity(self):
        twist = Twist()
        self.control_linear_vel = self.makeSimpleProfile(
            self.control_linear_vel,
            self.target_linear_vel,
            (LIN_VEL_STEP_SIZE / 2.0)
        )
        self.control_angular_vel = self.makeSimpleProfile(
            self.control_angular_vel,
            self.target_angular_vel,
            (ANG_VEL_STEP_SIZE / 2.0)
        )

        twist.linear.x = self.control_linear_vel
        twist.angular.z = self.control_angular_vel

        self.publisher_.publish(twist)

    def vels(self, target_linear_vel, target_angular_vel):
        return "Hız Durumu:\n\tLineer hız: %.2f m/s\n\tAçısal hız: %.2f rad/s" % (target_linear_vel, target_angular_vel)

    def makeSimpleProfile(self, output, input, slop):
        if input > output:
            output = min(input, output + slop)
        elif input < output:
            output = max(input, output - slop)
        else:
            output = input
        return output

    def constrain(self, input, low, high):
        return max(low, min(high, input))

    def checkLinearLimitVelocity(self, vel):
        if self.turtlebot3_model == "burger":
            return self.constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
        elif self.turtlebot3_model == "waffle" or self.turtlebot3_model == "waffle_pi":
            return self.constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
        else:
            return self.constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

    def checkAngularLimitVelocity(self, vel):
        if self.turtlebot3_model == "burger":
            return self.constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
        elif self.turtlebot3_model == "waffle" or self.turtlebot3_model == "waffle_pi":
            return self.constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
        else:
            return self.constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

def main(args=None):
    rclpy.init(args=args)
    teleop_key = TeleopKey()
    
    try:
        rclpy.spin(teleop_key)
    except Exception as e:
        print(e)
    finally:
        teleop_key.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()