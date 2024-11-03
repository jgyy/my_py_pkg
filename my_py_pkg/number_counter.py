#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64


class NumberCounter(Node):
    def __init__(self):
        super().__init__("number_counter")
        self.subscription = self.create_subscription(
                Int64, "number", self.number_callback, 10)
        self.publisher_ = self.create_publisher(Int64, "number_count", 10)
        self.counter = 0
        self.get_logger().info("Number Counter has been started")

    def number_callback(self, msg):
        self.counter += msg.data
        count_msg = Int64()
        count_msg.data = self.counter
        self.publisher_.publish(count_msg)
        self.get_logger().info(f"Counter updated: {self.counter}")


def main(args=None):
    rclpy.init(args=args)
    node = NumberCounter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
