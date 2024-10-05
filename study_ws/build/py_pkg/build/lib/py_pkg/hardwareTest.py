#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from irobot_interfaces.msg import HardwareStatus

class HardwareStatusPublisherNode(Node): 
    def __init__(self):
        super().__init__("HardwareStatusPublisher") 
        self.hw_status_publisher_ = self.create_publisher(HardwareStatus, "hardware_status", 10)
        self.timer = self.create_timer(1, self.publish_hw_status)
        self.get_logger().info("hw_Publisher Initiated...")

    def publish_hw_status(self):
        msg = HardwareStatus()
        msg.temperature = 45
        msg.motor_status = True
        msg.debug_message = "Nothing to report"
        self.hw_status_publisher_ .publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HardwareStatusPublisherNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
