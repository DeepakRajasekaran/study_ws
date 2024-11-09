#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class Hunter_Node(Node): 
    def __init__(self):
        super().__init__("hunter")
        
        self.cmd_vel_publish_ = self.create_publisher(/* msg_type */, '/* topic_name */', 10)



def main(args=None):
    rclpy.init(args=args)
    node = Hunter_Node() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
