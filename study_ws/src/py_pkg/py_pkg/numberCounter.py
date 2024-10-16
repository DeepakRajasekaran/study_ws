#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from std_srvs.srv import SetBool

class CounterNode(Node): 
    def __init__(self):
        super().__init__("numberCount")
        self.get_logger().info('Number Counter Has Initiated...')
        self.counter, self.history = 0, 0
        self.counter_reset_service = self.create_service(SetBool, 'reset_counter', self.counter_reset)
        self.subscription = self.create_subscription(Int64, "number", self.subscriber_callback, 10)

    def subscriber_callback(self, msg):
        if self.history != msg.data:
            self.counter += 1
            self.history = msg.data
        self.get_logger().info("Data: " + str(msg.data) + " | Instance: " + str(self.counter))

    def counter_reset(self, request, response):
        if request.data:
            self.counter = 0
            response.success = True
            response.message = "Counter_has_been_reset"
        else:
            response.success = False
            response.message = "Counter_has_not_reset"
        return response 

def main(args=None):
    rclpy.init(args=args)
    node = CounterNode() 
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
