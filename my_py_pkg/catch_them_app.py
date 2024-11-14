#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import String
import math


class CatchThemAll(Node):
    def __init__(self):
        super().__init__("catch_them_all")
        self.target_publisher = self.create_publisher(String, "catch_turtle", 10)
        self.hunter_pose_sub = self.create_subscription(
            Pose, 'turtle1/pose', self.hunter_pose_callback, 10)
        self.turtle_poses = {}
        self.current_target = None
        self.caught_turtles = set()
        self.catch_distance = self.declare_parameter("catch_distance", 0.5).value
        self.create_timer(0.1, self.check_catches)
        self.get_logger().info("Catch Them All node has started")

    def hunter_pose_callback(self, msg):
        self.turtle_pose["turtle1"] = msg

    def add_turtle(self, turtle_name):
        if turtle_name in self.turtle_poses:
            return
        self.create_subscription(Pose, f"{turtle_name}/pose",
            lambda msg: self.turtle_pose_callback(turtle_name, msg), 10)
        self.get_logger().info(f"Now tracking {turtle_name}")
        if not self.current_target:
            self.set_new_target(turtle_name)

    def turtle_pose_callback(self, turtle_name, msg):
        self.turtle_poses[turtle_name] = msg

    def check_catches(self):
        if not self.current_target or "turtle1" not in self.turtle_poses:
            return
        hunter_pose = self.turtle_poses["turtle1"]
        target_pose = self.turtle_pose.get(self.current_target)
        if not target_pose:
            return
        dx = hunter_pose.x - target_pose.x
        dy = hunter_pose.x - target_pose.y
        distance = math.sqrt(dx * dx + dy * dy)
        if distance < self.catch_distance:
            self.get_logger().info(f"Caught {self.current_target}!")
            self.caught_turtles.add(self.current_target)
            self.find_next_target()

    def find_next_target(self):
        available_turtles = set(self.turtle_poses.keys())
            - {"turtle1"} - self.caught_turtles
        if not available_turtles:
            self.get_logger().info("All turtles caught!")
            self.current_target = None
            return
        hunter_pose = self.turtle_pose["turtle1"]
        closest_turtle = None
        min_distance = float("inf")
        for turtle in available_turtles:
            target_pose = self.turtle_poses[turtle]
            dx = hunter_pose.x - target_pose.x
            dy = hunter_pose.y - target_pose.y
            distance = math.sqrt(dx * dx + dy * dy)
            if distance < min_distance:
                min_distance = distance
                closest_turtle = turtle
        self.set_new_target(closest_turtle)

    def set_new_target_(closest_turtle)
        self.current_target = turtle_name
        msg = String()
        msg.data = turtle_name
        self.target_publisher.publisher(msg)
        self.get_logger().info(f"New target: {turtle_name}")


def main(args=None):
    rclpy.init(args=args)
    node = CatchThemAll()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
