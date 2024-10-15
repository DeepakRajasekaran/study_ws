#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64

class counterNode(Node): 
    def __init__(self):
        super().__init__("numberCount")
        self.get_logger().info('Number Counter Has Initiated...')
        self.counter = 0
        self.subscription = self.create_subscription(Int64, "number", self.callback, 10)
        self.timer_event = self.create_timer(1.0, self.callback)

    def callback(self, msg):
        self.get_logger().info("Data" + str(msg.data) + " | Instance"+ str(self.counter))
        self.counter += 1

    def counter_reset():
        pass

def main(args=None):
    rclpy.init(args=args)
    node = counterNode() 
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
