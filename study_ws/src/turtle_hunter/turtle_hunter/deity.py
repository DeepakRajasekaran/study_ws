#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from math import pi

from turtlesim.srv import Spawn, Kill
from irobot_interfaces.msg import Turtleinfo, TurtleArray
from irobot_interfaces.srv import KillSwitch

import random
from functools import partial

class deity_node(Node):
    def __init__(self):
        super().__init__("deity")
        self.get_logger().info("Deity Node has been initiated...")

        self.declare_parameter('spawn_freq', 1.0)
        self.spawn_frequency_ = self.get_parameter('spawn_freq').value

        self.aliveTurtles_publisher_ = self.create_publisher(TurtleArray, "alive_turtles", 10)

        self.spawner_ = self.create_client(Spawn, "spawn")
        self.killer_ = self.create_client(Kill, "kill")
        self.kill_command_ = self.create_service(KillSwitch, "killer", self.kill_command_callback)

        self.turtle_count = 0
        self.alive_turtles = []

        # Timer for spawning turtles at regular intervals
        self.timer_ = self.create_timer(1.0, self.timer_callback_)

    def timer_callback_(self):
        if not self.spawner_.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Spawn service is unavailable. Retrying...")
            return
        self.spawn_turtle()

    def spawn_turtle(self):
        turtle = Spawn.Request()
        turtle.x = float(random.randint(1, 10))
        turtle.y = float(random.randint(1, 10))
        turtle.theta = random.uniform(-pi, pi)
        self.turtle_count += 1
        turtle.name = f"Pray_{self.turtle_count}"

        future = self.spawner_.call_async(turtle)
        future.add_done_callback(partial(self.log_spawned_turtle, turtle))

    def log_spawned_turtle(self, request, future):
        try:
            response = future.result()
            self.get_logger().info(f"Turtle {response.name} born at x: {request.x}, y: {request.y}")
            born_turtle = Turtleinfo(x=request.x, y=request.y, theta=request.theta, name=response.name)
            self.alive_turtles.append(born_turtle)
            self.publish_alive_turtles()
        except Exception as e:
            self.get_logger().error(f"Turtle died during birth: {e}")

    def kill_command_callback(self, request, response):
        self.kill_turtle(request.name)
        response.killed = True
        return response

    def kill_turtle(self, turtle_ID):
        if not self.killer_.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Kill service is unavailable. Retrying...")
            return

        kill_request = Kill.Request()
        kill_request.name = turtle_ID
        future = self.killer_.call_async(kill_request)
        future.add_done_callback(partial(self.log_killed_turtle, turtle_ID))

    def log_killed_turtle(self, turtle_ID, future):
        try:
            response = future.result()
            for i, turtle in enumerate(self.alive_turtles):
                if turtle.name == turtle_ID:
                    del self.alive_turtles[i]
                    self.get_logger().warn(f"{turtle_ID} is killed by hunter.")
                    self.publish_alive_turtles()
                    break
        except Exception as e:
            self.get_logger().error(f"{turtle_ID} escaped from hunter due to: {e}")

    def publish_alive_turtles(self):
        msg = TurtleArray(turtles=self.alive_turtles)
        self.aliveTurtles_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = deity_node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
