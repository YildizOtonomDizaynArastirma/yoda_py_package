#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class TemplateNode(Node):
    def __init__(self):
        super().__init__("template_node")
        
def main(args=None):
    rclpy.init(args = args) #ilk yapacağımız şey
    node = TemplateNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()