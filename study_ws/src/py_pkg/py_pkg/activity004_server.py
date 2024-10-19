#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from irobot_interfaces.srv import SetLed
from irobot_interfaces.msg import LEDStates
# Python Specifics..

class LED_Panel_Node(Node): 
    def __init__(self):
        super().__init__("LED_Panel_Node")
        self.get_logger().info('LED Panel Node Server has initiated...')
        self.server_ = self.create_service(SetLed, 'set_led', self.callback_)

    def callback_(self, request, response):
        try:
            led_states = LEDStates()
            led_states.led_states = request.input_array
            response.success = True
            self.get_logger().info(f'LED States Updated: {led_states.led_states}')
        except Exception as e:
            response.success = False
            self.get_logger().error(f'LED States Update Failed: {e}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LED_Panel_Node() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
