#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLed
from my_robot_interfaces.msg import LedStates


class LedPanelNode(Node):
    def __init__(self):
        super().__init__("led_panel_node")
        self.led_states = [False, False, False]
        self.service = self.create_service(
            SetLed,
            "set_led",
            self.set_led_callback
        )
        self.publisher = self.create_publisher(
            LedStates,
            "led_panel_state",
            10
        )
        self.create_timer(0.1, self.publish_led_states)
        self.get_logger().info("Battery node initialized")

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
