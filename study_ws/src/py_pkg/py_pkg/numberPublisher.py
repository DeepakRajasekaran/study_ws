#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

NodeName = "numPub"

class numPub(Node): 
    def __init__(self):
        super().__init__(NodeName)
        self.get_logger().info('NumberPublisher Has been initiated')
        self.publishNumber()
    
    def publishNumber():

        self.numpub = self.create_publisher(/* msg_type */, '/* topic_name */', 10)

        msg = /* msg_type */()
        self.get_logger().info('Publishing message')
        self./* pub_name */.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = numPub() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
