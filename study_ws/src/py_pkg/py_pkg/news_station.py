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
        self.declare_parameter('publishFrequency', 1.0)
        self.delay_ = self.get_parameter('publishFrequency').get_parameter_value().double_value
        self.timer_ = self.create_timer(self.delay_, self.publish_news)
        self.declare_parameter('robot_name', "R2D2")
        self.show_runner_ = self.get_parameter('robot_name').get_parameter_value().string_value

    def publish_news(self):
        msg = String()
        
        #self.counter += 1
        msg.data = "Hi, I'm " + self.show_runner_ + ", This is the Robot News Station.."
        self.publisher_.publish(msg)
        self.get_logger().info(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = news_station() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
