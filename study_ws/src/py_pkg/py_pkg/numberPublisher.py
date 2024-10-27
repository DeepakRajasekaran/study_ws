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
        # Tried ros2Parameters in this region..
        self.declare_parameter('number_to_publish', 10)
        self.declare_parameter('publishFrequency', 1)
        self.period = 1/self.get_parameter('publishFrequency').value
        #*********************************************
        self.number_publisher_ = self.create_publisher(Int64, 'number', 10)
        self.timer_callback = self.create_timer(self.period, self.publishNumber)
    
    def publishNumber(self):
        msg = Int64()
        #random.randint(1, 100)
        try:
            msg.data = self.get_parameter('number_to_publish').value
            self.get_logger().info('Publishing number :' + str(msg.data))
            self.number_publisher_.publish(msg)            
        except Exception as e:
            self.get_logger().error(f'Ran into an error : {e}')

def main(args=None):
    rclpy.init(args=args)
    node = numPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Detected KeyboardInterrupt. Exiting...')
    finally:
        # Clean up the node and shutdown rclpy
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
