#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int8MultiArray

class activity005_node(Node): 
    def __init__(self):
        super().__init__("act005_node")
        self.declare_parameter('led_states_', [0, 0, 0, 0])
        self.led_states_ = self.get_parameter('led_states_').value
        self.publisher_ = self.create_publisher(Int8MultiArray, 'integer_array', 10)
        self.timer_ = self.create_timer(1.0, self.callback_)

    def callback_(self):
        msg = Int8MultiArray()
        msg.data = self.led_states_
        self.get_logger().info('Publishing message')
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = activity005_node() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
