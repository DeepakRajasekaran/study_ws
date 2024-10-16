#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64

# Python Specifics
import random
from functools import partial

NodeName = "PublisherNode"

class numPub(Node): 
    def __init__(self):
        super().__init__(NodeName)
        self.get_logger().info('NumberPublisher Has been initiated')
        self.number_publisher_ = self.create_publisher(Int64, 'number', 10)
        self.timer_callback = self.create_timer(1.0, self.publishNumber)
    
    def publishNumber(self):
        msg = Int64()
        msg.data = random.randint(1, 100)
        self.get_logger().info('Publishing number :' + str(msg.data))
        self.number_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = numPub() 
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
