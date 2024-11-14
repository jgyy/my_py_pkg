#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
import random
import math


class TurtleSpawner(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.spawn_client = self.create_client(Spawn, "spawn")
        self.spawn_frequency = self.declare_parameter("spawn_frequency", 5.0).value
        self.create_timer(self.spawn_frequency, self.spawn_turtle)
        self.turtle_counter = 2
        self.get_logger().info("Turtle Spawner node has started")

    def spawn_turtle(self):
        if not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Spawn service not available")
            return
        request = Spawn.Request()
        request.x = random.uniform(1.0, 10.0)
        request.y = random.uniform(1.0, 10.0)
        request.theta = random.uniform(0, 2 * math.pi)
        request.name = f"turtle{self.turtle_counter}"
        future = self.spawn_client.call_async(request)
        future.add_done_callback(self.spawn_callback)

    def spawn_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Spawned turtle: {response.name}")
            self.turtle_counter += 1
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    spawner = TurtleSpawner()
    rclpy.spin(spawner)
    spawner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
