#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from irobot_interfaces.srv import ComputeRectangleArea


class FindRectangleAreaNode(Node): 
    def __init__(self):
        super().__init__("find_rectangle_area_server")

        self.get_logger().info('findRectangleArea_server node has been Initialized....')
        self.server_ = self.create_service(ComputeRectangleArea, "compute_rectangle_area", self.find_area)

    def find_area(self, request, response):
        response.area = request.length * request.width
        self.get_logger().info(f"Area: {response.area}")
        self.get_logger().debug(f'Length: {request.length}, Width: {request.width}, Area: {response.area}')
        return response 


def main(args=None):
    rclpy.init(args=args)
    node = FindRectangleAreaNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
