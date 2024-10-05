#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
# Python Specifics
import random
from functools import partial

class client_sumServerNode(Node): 
    def __init__(self):
        super().__init__("client_sumServer")
        self.get_logger().info('Request Client Has Initiated...')
        while rclpy.ok():
           self.f1(int(random.randint(1, 100)), int(random.uniform(1, 100)))

    def f1(self, a, b):
        client = self.create_client(AddTwoInts, "add_two_ints")

        while not client.wait_for_service(1.0):
            self.get_logger().warn("Server is not up yet...")
        
        
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_f1, a=a, b=b))

    def callback_f1(self, future, a, b):
        try:
            response = future.result()
            self.get_logger().info(str(a) + " + " + str(b) + " = " + str(response.sum))
        except Exception as e:
            self.get_logger().error("Service call failed due to %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = client_sumServerNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
