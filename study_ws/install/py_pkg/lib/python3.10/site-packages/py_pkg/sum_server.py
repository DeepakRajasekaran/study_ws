#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts

class sum_serverNode(Node): 
    def __init__(self):
        super().__init__("sumServer") 
        self.server_ = self.create_service(AddTwoInts, "add_two_ints", self.callback_add_two_ints)
        self.get_logger().info("Add_Two_Ints Server Initiated...")
    
    def callback_add_two_ints(self, request, response):
        response.sum = int(request.a) + int(request.b)
        self.get_logger().info(str(request.a) + " + " + str(request.b) + " = " + str(response.sum))
        return response


def main(args=None):
    rclpy.init(args=args)
    node = sum_serverNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 

if __name__ == "__main__":
    main()
