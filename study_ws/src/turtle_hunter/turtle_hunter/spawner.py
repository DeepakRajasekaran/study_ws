#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from math import pi

from turtlesim.srv import Spawn
from irobot_interfaces.msg import Turtleinfo
from irobot_interfaces.msg import TurtleArray

# Python Specifics
import random
from functools import partial
import time

class spawner_node(Node):

    def __init__(self):
        super().__init__("spawner")
        self.get_logger().info('Spawner Node has been initiated...')
        self.aliveTurtles_publisher_ = self.create_publisher(TurtleArray, 'alive_turtles', 10)
        self.spawner_ = self.create_client(Spawn, 'spawn')
        self.turtle_count = 0
        self.alive_turtles = []
        # need to revisit for having common frequency in all nodes using ros2_parameters
        self.timer_ = self.create_timer(1.0, self.timer_callback_)
    
    def timer_callback_(self):
        while not self.spawner_.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for Server to start....')
        self.spawn_turtle()
        self.get_logger().info('new turtle is in labor..')
        self.publish_alive_turtles()

    def publish_alive_turtles(self):
        msg = TurtleArray()
        msg.turtles = self.alive_turtles
        self.aliveTurtles_publisher_.publish(msg)

    def spawn_turtle(self):
        turtle = Spawn.Request()
        turtle.x = float(random.randint(1, 10))
        turtle.y = float(random.randint(1, 10))
        turtle.theta = random.uniform(-pi, pi)
        self.turtle_count += 1
        turtle.name = "Pray_" + str(self.turtle_count) # here the pray_ is the prefix..
        future = self.spawner_.call_async(turtle)
        future.add_done_callback(partial(self.log_turtle, turtle))
        
    def log_turtle(self, request, future):
        # append the data in self.alive_turtles array
        try:
            out = future.result()
            self.get_logger().info(f'Turtle {out.name} born at x: {request.x}, y: {request.y}')
            alive_turtle = Turtleinfo()
            alive_turtle.x = request.x
            alive_turtle.y = request.y
            alive_turtle.theta = request.theta
            alive_turtle.name = request.name
            self.alive_turtles.append(alive_turtle)

        except Exception as e:
            self.get_logger().error(f"Turtle died During birth: {e}")        

def main(args=None):
    rclpy.init(args=args)
    node = spawner_node() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
