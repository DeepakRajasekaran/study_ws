#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class battery_node(Node): 
    def __init__(self):
        super().__init__("battery_node")

        self.client_ = self.create_client(SetBool, 'set_led', self.request)

    def request():
        pass


def main(args=None):
    rclpy.init(args=args)
    node = battery_node() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
