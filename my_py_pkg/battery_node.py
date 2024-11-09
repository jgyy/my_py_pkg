#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLed
import time


class BatteryNode(Node):
    def __init__(self):
        super().__init__("battery_node")
        self.client = self.create_client(SetLed, "set_led")
        self.battery_state = True
        self.create_timer(0.1, self.battery_state_callback)
        self.start_time = time.time()
        self.get_logger().info("Battery node initialized")

    def battery_state_callback(self):
        current_time = time.time() - self.start_time
        cycle_time = current_time % 10.0
        if cycle_time >= 4.0 and self.battery_state:
            self.battery_state = False
            self.get_logger().info("Battery empty! Turning on LED...")
            self.send_led_request(3, True)
        elif cycle_time < 4.0 and not self.battery_state:
            self.battery_state = True
            self.get_logger().info("Battery full! Turning off LED...")
            self.send_led_request(3, False)

    def send_led_request(self, led_number, state):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")
        request = SetLed.Request()
        request.led_number = led_number
        request.state = state
        future = self.client.call_async(request)
        future.add_done_callback(self.led_response_callback)

    def led_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Successfully changed LED state")
            else:
                self.get_logger().warning("Failed to change LED state")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
