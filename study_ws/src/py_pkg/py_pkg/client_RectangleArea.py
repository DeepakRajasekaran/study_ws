#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from irobot_interfaces.srv import RectangleParameters

# Python Specifics
import random
from functools import partial
import time

class ClientNode(Node):

    def __init__(self):
        super().__init__('client_node')
        self.get_logger().info('Client node has been Initialized....')
        self.client = self.create_client(RectangleParameters, 'compute_rectangle_area')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for Server to start....')
        self.request_timer = self.create_timer(1.0, self.generate_request)
        
    def generate_request(self):
        request = RectangleParameters.Request()
        request.length = float(random.randint(1, 100))
        request.width = float(random.randint(1, 100))
        self.get_logger().info(f"Request  | length: {request.length}, width: {request.width}")
        future = self.client.call_async(request)
        future.add_done_callback(partial(self.response_callback, request))

    def response_callback(self, request, future):
        try:
            response = future.result()
            self.get_logger().info(f"Response | Area = {response.area}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ClientNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
