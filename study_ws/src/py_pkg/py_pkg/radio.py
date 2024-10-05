#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class radio(Node): 
    def __init__(self):
        super().__init__("radio") 
        #self.counter = 0
        self.subscriber_ = self.create_subscription(String, "news", self.listener_callback, 10)
        self.get_logger().info("Listening..")

    def listener_callback(self, msg):
        #self.counter += 1
        self.get_logger().info("I heard: " + msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = radio() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
