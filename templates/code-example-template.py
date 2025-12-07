#!/usr/bin/env python3
"""
Example [N].[M]: [Example Title]

Description:
    [Brief description of what this example demonstrates]

Learning Objectives:
    - [Objective 1]
    - [Objective 2]
    - [Objective 3]

Prerequisites:
    - ROS 2 Humble installed
    - [Additional prerequisites]

Usage:
    python3 [filename].py [args]

Author: Physical AI & Humanoid Robotics Textbook
Date: [YYYY-MM-DD]
"""

import rclpy
from rclpy.node import Node
# [Additional imports as needed]


class ExampleNode(Node):
    """
    [Brief class description]

    This node demonstrates [key concept being taught].
    """

    def __init__(self):
        """Initialize the ExampleNode."""
        super().__init__('example_node')

        # [Initialize publishers, subscribers, services, actions, etc.]

        # [Initialize class variables]

        self.get_logger().info('[Example Node] has been started.')

    def callback_function(self, msg):
        """
        [Callback description]

        Args:
            msg: [Message type and description]
        """
        # [Callback implementation]
        pass


def main(args=None):
    """
    Main entry point for the example node.

    Args:
        args: Command-line arguments (default: None)
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create the node
    node = ExampleNode()

    try:
        # Spin the node to execute callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Example node stopped by user.')
    finally:
        # Clean shutdown
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
