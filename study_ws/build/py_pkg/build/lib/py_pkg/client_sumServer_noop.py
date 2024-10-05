#!/usr/bin/env python3
import rclpy
import random
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts

def main(args=None):
    rclpy.init(args=args)
    node = Node("addTwoInt_noop") 

    client = node.create_client(AddTwoInts, "add_two_ints")

    while not client.wait_for_service(1.0):
        node.get_logger().warn("Server is not up yet...")
    
    request = AddTwoInts.Request()


    request.a = int(random.randint(1, 100))
    request.b = int(random.uniform(1, 100))

    future = client.call_async(request)
    
    rclpy.spin_until_future_complete(node, future)

    try:
        response = future.result()
        node.get_logger().info(str(request.a) + " + " + str(request.b) + " = " + str(response.sum))
    except Exception as e:
        node.get_logger().error("Service call failed due to %r" % (e,))

        
    # rclpy.spin(node) 
    rclpy.shutdown()


if __name__ == "__main__":
    main()
