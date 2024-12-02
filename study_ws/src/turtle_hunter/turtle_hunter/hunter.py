#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from math import sqrt, pi, atan2

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from irobot_interfaces.msg import TurtleArray
from irobot_interfaces.srv import KillSwitch

class Hunter_Node(Node):
    def __init__(self):
        super().__init__("hunter")
        self.get_logger().info("Hunter Node has been initiated...")

        self.pose_ = None
        self.target = None
        self.target_distance = None
        self.goal_theta = None
        self.target_locked = False # By Default Target is not Locked

        self.declare_parameter('kill_closest_turtle_first', False)
        self.kill_closest_turtle_first = self.get_parameter('kill_closest_turtle_first').get_parameter_value().bool_value
        self.declare_parameter('hunter_freq', 80)
        self.hunter_freq = 1/self.get_parameter('hunter_freq').get_parameter_value().integer_value

        self.pose_subscriber_ = self.create_subscription(Pose, "turtle1/pose", self.pose_subscriber_callback, 10)
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.target_subscriber_ = self.create_subscription(TurtleArray, "alive_turtles", self.find_target_pray, 10)

        self.kill_request_ = self.create_client(KillSwitch, "killer")
        self.timer_ = self.create_timer(self.hunter_freq, self.hunt_pray_callback)

    def pose_subscriber_callback(self, msg):
        self.pose_ = msg

    def find_target_pray(self, msg):
        if self.kill_closest_turtle_first:
            closest_turtle = self.target
            closest_turtle_distance = self.target_distance

            if not self.target_locked:
                for turtle in msg.turtles:
                    dist_x = turtle.x - self.pose_.x
                    dist_y = turtle.y - self.pose_.y
                    distance = sqrt((dist_x * dist_x) + (dist_y * dist_y))
                    if closest_turtle_distance is None or distance < closest_turtle_distance:
                        closest_turtle_distance = distance
                        closest_turtle = turtle
            else:
                dist_x = self.target.x - self.pose_.x 
                dist_y = self.target.y - self.pose_.y
                closest_turtle_distance = sqrt((dist_x * dist_x) + (dist_y * dist_y))

            self.target = closest_turtle
            self.target_distance = closest_turtle_distance
            self.goal_theta = atan2(dist_y, dist_x) 
            self.target_locked = True
            self.get_logger().info('\033[92mtarget_locked..\033[0m')

        else:
            self.target = msg.turtles[0]
            dist_x = self.target.x - self.pose_.x
            dist_y = self.target.y - self.pose_.y
            self.target_distance = sqrt((dist_x * dist_x) + (dist_y * dist_y))
            self.goal_theta = atan2(dist_y, dist_x)

    def hunt_pray_callback(self):
        if self.pose_ is None or self.target is None:
            return

        msg = Twist()
        kp_linear = 0.8
        kp_angular = 6.0

        if self.target_distance > 0.5:
            msg.linear.x = kp_linear * self.target_distance
            diff = (self.goal_theta - self.pose_.theta + pi) % (2 * pi) - pi
            msg.angular.z = kp_angular * diff
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.send_kill_request(self.target.name)

        self.cmd_vel_publisher_.publish(msg)

    def send_kill_request(self, turtle_name):
        if not self.kill_request_.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("KillSwitch service unavailable. Retrying...")
            return

        request = KillSwitch.Request()
        request.name = turtle_name
        future = self.kill_request_.call_async(request)

        # Reset target-related data
        if self.target_locked:
            self.target = None
            self.target_distance = None
            self.target_locked = False
            self.goal_theta = None
            self.get_logger().info('\033[92mtarget_resetted..\033[0m')
        
        future.add_done_callback(self.kill_request_response_callback_)

    def kill_request_response_callback_(self, future):
        try:
            response = future.result()
            if not response.killed:
                self.get_logger().info("Pray escaped due to unknown reasons.")
        except Exception as e:
            self.get_logger().warn(f"Error occurred during kill request: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = Hunter_Node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
