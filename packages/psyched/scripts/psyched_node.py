#!/usr/bin/env python3
"""
Example ROS2 Python node for the psyched framework
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PsychedNode(Node):
    """
    Example psyched framework node
    """

    def __init__(self):
        super().__init__('psyched_node')

        # Create publisher
        self.publisher_ = self.create_publisher(String, 'psyched_topic', 10)

        # Create timer
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0

        self.get_logger().info('Psyched node has been started')

    def timer_callback(self):
        """
        Timer callback to publish messages
        """
        msg = String()
        msg.data = f'Psyched framework message {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    psyched_node = PsychedNode()

    try:
        rclpy.spin(psyched_node)
    except KeyboardInterrupt:
        pass
    finally:
        psyched_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
