#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor


class RobotNewsStationNode(Node):
    def __init__(self):
        super().__init__("robot_news_station")

        robot_name_descriptor = ParameterDescriptor(
            description="Name of the robot for the news station")
        self.declare_parameter("robot_name", "R2D2", robot_name_descriptor)

        self.publisher_ = self.create_publisher(String, "robot_news", 10)
        self.timer_ = self.create_timer(2.0, self.publish_news)
        self.get_logger().info("Robot New Station has been started")

    def publish_news(self):
        msg = String()
        robot_name = self.get_parameter("robot_name").value
        msg.data = f"Hi, this is {robot_name} from the robot news station."
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
