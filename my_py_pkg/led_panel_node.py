#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from my_robot_interfaces.srv import SetLed
from my_robot_interfaces.msg import LedStates


class LedPanelNode(Node):
    def __init__(self):
        super().__init__("led_panel")

        self.led_states = [False, False, False]
        self.service = self.create_service(SetLed, "set_led", self.set_led_callback)
        self.publisher = self.create_publisher(LedStates, "led_panel_state", 10)
        led_states_descriptor = ParameterDescriptor(
            description="States of LEDs (0 for off, 1 for on)")
        self.declare_parameter("led_states", [0, 0, 0, 0], led_states_descriptor)

        self_timer_ = self.create_timer(2.0, self.check_led_states)
        self.create_timer(0.1, self.publish_led_states)
        self.get_logger().info("Led Panel node initialized")

    def check_led_states(self):
        led_states = self.get_parameter("led_states").value
        state_str = ""
        for i, state in enumerate(led_states):
            state_str += f"LED {i}: {'ON' if state == 1 else 'OFF'},"
        self.get_logger().info(f"Current LED states: {state_str[:-2]}")

    def set_led_callback(self, request, response):
        if request.led_number < 1 or request.led_number > 3:
            self.get_logger().error(f"Invalid LED number: {request.led_number}")
            response.success = Fase
            return response
        led_index = request.led_number - 1
        self.led_states[led_index] = request.state
        state_str = "on" if request.state else "off"
        self.get_logger().info(f"LED {request.led_number} turned {state_str}")
        response.success = True
        return response

    def publish_led_states(self):
        msg = LedStates()
        msg.states = self.led_states
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LedPanelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
