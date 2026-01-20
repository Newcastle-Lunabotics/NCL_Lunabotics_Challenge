# talker.py
# Publishes String messages to /chatter every 1 second.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    def __init__(self):
        # Node name (shows up in ros2 node list)
        super().__init__('talker')

        # Create a publisher:
        # - message type: String
        # - topic name: /chatter
        # - queue size: 10
        self.pub = self.create_publisher(String, '/chatter', 10)

        # Create a timer that runs once per second
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.counter = 0
        self.get_logger().info("Talker started. Publishing on /chatter")

    def timer_callback(self):
        msg = String()
        msg.data = f"hello #{self.counter}"
        self.pub.publish(msg)

        self.get_logger().info(f"Published: {msg.data}")
        self.counter += 1


def main(args=None):
    # Start ROS
    rclpy.init(args=args)

    # Create node object
    node = Talker()

    # Keep it running
    rclpy.spin(node)

    # Cleanup (runs when you Ctrl+C)
    node.destroy_node()
    rclpy.shutdown()
