#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from rectangle_interfaces.srv import ComputeRectangleArea


class findRectangleAreaNode(Node): 
    def __init__(self):
        super().__init__("findRectangleArea")

        self.get_logger().info('findRectangleArea_server node has been Initialized....')
        self.server_ = self.create_service(FindRectangleArea, "find_rectangle_area", self.findArea)

    def findArea(self, request, response):
        response.area = request.length * request.breadth
        self.get_logger().info(f"Area: {response.area}")
        self.get_logger().debug('Length: %f, Breath = %f, Area = %f', request.length)
        return response 


def main(args=None):
    rclpy.init(args=args)
    node = findRectangleAreaNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
