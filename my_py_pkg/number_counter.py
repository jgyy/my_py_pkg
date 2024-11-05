#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from example_interfaces.srv import SetBool


class NumberCounterNode(Node):
    def __init__(self):
        super().__init__("number_counter")
        self.number_subscriber = self.create_subscription(
            Int64, "/number", self.callback_number, 10)
        self.counter_publisher_ = self.create_publisher(
            Int64, "/number_count", 10)
        self.reset_service = self.create_service(
            SetBool, "/reset_number_count", self.callback_reset_counter)
        self.get_logger().info("Number Counter Node has been started.")

    def callback_number(self, msg):
        self.counter += msg.data
        msg_count = Int64()
        msg_count.data = self.counter
        self.counter_publisher.publish(msg_count)

    def callback_reset_counter(self, request, response):
        if request.data:
            self.counter = 0
            response.success = True
            response.message = "Counter has been reset to 0"
        else:
            response.success = False
            response.message = "Counter was not reset"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
