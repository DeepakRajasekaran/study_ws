#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from rectangle_interfaces.srv import ComputeRectangleArea

# Python Specifics
import random
from functools import partial

class ClientNode(Node):

    def __init__(self):
        super().__init__('Client_')
        self.get_logger().info('Client node has been Initialized....')
        request = self.create_client(ComputeRectangleArea, 'compute_rectangle_area')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for Server to start....')

    def GenerateRequest(self):
        request.length = int(random.randint(1, 100))
        request.width = int(random.randint(1, 100))
        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback, request)

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Area: {response.area}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')



def main(args=None):
    rclpy.init(args=args)
    node = ClientNode()
    rclpy.spin(node())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
