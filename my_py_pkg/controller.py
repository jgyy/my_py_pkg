#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, SetPen
from std_msgs.msg import String
import math


class TurtleController(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.vel_publisher = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.pose_subscription = self.create_subscription(
            Pose, "turtle1/pose", self.pose_callback, 10)
        self.target_subscription = self.create_subscription(
            String, "catch_turtle", self.target_callback, 10)
        self.teleport_client = self.create_client(
            TeleportAbsolute, "turtle1/teleport_absolute")
        self.set_pen_client = self.create_client(SetPen, "turtle1/set_pen")
        self.current_pose = None
        self.target_turtle = None
        self.target_pose = None
        self.target_pose_subscription = None
        self.catch_distance_threshold = self.declare_parameter(
            "catch_distance_threshold", 0.5).value
        self.get_logger().info("Turtle Controller node has started")

    def pose_callback(self, msg):
        self.current_pose = msg
        if self.target_turtle and self.target_pose:
            self.move_to_target()

    def target_callback(self, msg):
        self.target_turtle = msg.data
        self.get_logger().info(f"New target: {self.target_turtle}")
        if self.target_pose_subscription:
            self.destroy_subscription(self.target_pose_subscription)
        self.target_pose_subscription = self.create_subscription(
            Pose, f"{self.target_turtle}/pose", self.target_pose_callback, 10)

    def target_pose_callback(self, msg):
        self.target_pose = msg
        if self.current_pose:
            self.move_to_target()

    def move_to_target(self):
        if not self.current_pose or not self.target_turtle:
            return
        dx = self.target_pose.x - self.current_pose.x
        dy = self.target_pose.y - self.current_pose.y
        distance = math.sqrt(dx * dx + dy * dy)
        angle = math.atan2(dy, dx)
        vel_msg = Twist()
        angle_diff = angle - self.current_pose.theta
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        elif angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        vel_msg.angular.z = 4.0 * angle_diff
        vel_msg.linear.x = min(2.0 * distance, 2.0)
        self.vel_publisher.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    controller = TurtleController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
