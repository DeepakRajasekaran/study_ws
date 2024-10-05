#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class news_station(Node): 
    def __init__(self):
        super().__init__("news_station")
        self.counter = 0
        self.publisher_ = self.create_publisher(String, "news", 10)
        self.get_logger().info("Broadcasting Starts..")
        self.timer_ = self.create_timer(0.5, self.publish_news)

    def publish_news(self):
        msg = String()
        
        self.counter += 1
        msg.data = "Test_Msg" + str(self.counter)
        self.publisher_.publish(msg)
        #self.get_logger().info(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = news_station() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
