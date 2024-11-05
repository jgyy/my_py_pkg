#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class ResetCounterClient(Node):
    def __init__(self):
        super().__init__("reset_counter_client")
        self.client = self.create_client(SetBool, "/reset_number_count")
        self.get_logger().info("Reset Counter Client has been started.")

    def send_request(self):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        request = SetBool.Request()
        request.data = True

        future = self.client.call_async(request)
        return future

def main(args=None):
    rclpy.init(args=args)
    client_node = ResetCounterClient()

    future = client_node.send_request()
    rclpy.spin_until_future_complete(client_node, future)

    if future.result() is not None:
        client_node.get_logger().info(
            f'Result: {future.result().message}')
    else:
        client_node.get_logger().error(
            'Service call failed')

    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
