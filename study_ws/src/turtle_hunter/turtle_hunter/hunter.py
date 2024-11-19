#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from math import sqrt
from math import pi
from math import atan2

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
# from irobot_interfaces.msg import Turtleinfo
from irobot_interfaces.msg import TurtleArray

class Hunter_Node(Node): 
    def __init__(self):
        super().__init__("hunter")

        self.pose_ = None
        self.target= None

        self.pose_subscriber_ = self.create_subscription(
            Pose, 'turtle1/pose', self.pose_subscriber_callback, 10)

        self.cmd_vel_publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        self.target_subscriber_ = self.create_subscription(
            TurtleArray, "alive_turtles", self.find_target_pray, 10)

        # need to fix the publish_frequency with ros2_Parameters
        self.publish_frequency = 100 # unit Hz
        self.timer_period = 1/self.publish_frequency
        self.hunt_pray = self.create_timer(self.timer_period, self.hunt_pray_callback)

    def pose_subscriber_callback(self, msg):
        self.pose_  = msg

    def find_target_pray(self, msg):
        if len(msg.turtles) > 0:
            self.target = msg.turtles[0]

    def hunt_pray_callback(self):
        
        if self.pose_ == None or self.target == None:
            return

        dist_x = self.target.x - self.pose_.x
        dist_y = self.target.y - self.pose_.y

        # Applying Pythogorean theorem.. (nerds also call it euclidean distance measurement)
        distance = sqrt((dist_x * dist_x)+(dist_y * dist_y))

        msg = Twist()

        # Control Loop using P Controller..

        # Propotional Value for Linear 
        # {u(t) = kp * e(t)} | kp = proportional gain, u(t) = control signal, e(t) = error signal
        # for the human understanding multiply the distance the robot has to move with the gain value to achieve velocity
        kp_linear = 2
        kp_angular = 6

        if distance > 0.5:

            # Position
            msg.linear.x = kp_linear * distance

            # Orientation
            goal_theta = atan2(dist_y, dist_x)
            # diff = goal_theta - self.pose_.theta
            # if diff > pi:
            #     diff -= 2*pi
            # elif diff < -pi:
            #     diff += 2*pi
            # According to GPT, The above commented shit can be achieved with the following equation 
            # Execution will be according to BODMAS
            diff = (goal_theta - self.pose_.theta + pi) % (2 * pi) - pi

            msg.angular.z = kp_angular * diff

        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.cmd_vel_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Hunter_Node() 
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
