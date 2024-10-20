#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from irobot_interfaces.srv import SetLed

# Python Specifics..
import random
from functools import partial


class BatteryNode(Node): 
    def __init__(self):
        super().__init__("battery_node")
        self.get_logger().info('Client has initiated...')
        self.client_ = self.create_client(SetLed, 'set_led')
        self.request_timer = self.create_timer(1.0, self.request_)

    def battery_percent(self):
        return random.randint(1, 100)

    def process_led_states(self, array, charge):
        match charge:
            case x if x < 25:
                array = [False] * len(array)
            case x if x < 50:
                array[0] = True
                array[1:] = [False] * (len(array) - 1)
            case x if x < 75:
                array[0] = True
                array[1] = True
                array[2:] = [False] * (len(array) - 2)
            case x if 75 < x < 95:
                array[0] = True
                array[1] = True
                array[2] = True
                array[3:] = [False] * (len(array) - 3)
            case x if x >= 95:
                array = [True] * len(array)
        return array

    def request_(self):
        request_call = SetLed.Request()
        self.charge = self.battery_percent()
        request_call.input_array = [False] * len(request_call.input_array)
        request_call.input_array = self.process_led_states(request_call.input_array, self.charge)

        self.get_logger().info(f'Determined LED States: {request_call.input_array}')
        if not self.client_.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Waiting for Server to start....')
        self.future = self.client_.call_async(request_call)
        self.future.add_done_callback(partial(self.response_callback, request_call))

    def response_callback(self, request, future):
        green = '\033[92m'
        endc = '\033[0m'
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'{green}LED Array Updated | Battery Percent: {self.charge}%{endc}')
            else:
                self.get_logger().warn('LED Array Update Failed.')
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode() 
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
