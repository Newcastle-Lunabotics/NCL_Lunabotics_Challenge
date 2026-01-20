# listener.py
# Subscribes to /chatter and prints every message received.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Listener(Node):
    def __init__(self):
        super().__init__('listener')

        # Create a subscription:
        # - message type: String
        # - topic name: /chatter
        # - callback: what to do when message arrives
        # - queue size: 10
        self.sub = self.create_subscription(
            String,
            '/chatter',
            self.callback,
            10
        )

        self.get_logger().info("Listener started. Subscribed to /chatter")

    def callback(self, msg: String):
        # This runs every time a message is received
        self.get_logger().info(f"Received: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = Listener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
