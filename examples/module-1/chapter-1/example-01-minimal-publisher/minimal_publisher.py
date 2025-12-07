#!/usr/bin/env python3
"""
Minimal Publisher Node - ROS 2 Example

This node demonstrates the basic publish-subscribe pattern in ROS 2.
It publishes string messages to the '/chatter' topic at 2 Hz.

Learning Objectives:
- Create a ROS 2 node using rclpy
- Set up a publisher for a topic
- Use timers for periodic callbacks
- Publish messages with proper message types

Author: ROS 2 Fundamentals Module
License: Apache 2.0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    """
    A minimal publisher node that sends string messages periodically.

    This class inherits from rclpy.node.Node and creates a publisher
    that sends messages to the '/chatter' topic every 0.5 seconds.
    """

    def __init__(self):
        """
        Initialize the MinimalPublisher node.

        Creates a publisher on '/chatter' topic and sets up a timer
        for periodic message publication.
        """
        # Call the parent class constructor with node name
        super().__init__('minimal_publisher')

        # Create a publisher
        # Parameters:
        #   - msg_type: The message type (String from std_msgs)
        #   - topic: The topic name ('/chatter')
        #   - qos_profile: Queue size (10) - buffers last 10 messages
        self.publisher_ = self.create_publisher(String, '/chatter', 10)

        # Create a timer that calls timer_callback every 0.5 seconds (2 Hz)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize message counter
        self.i = 0

        self.get_logger().info('Minimal Publisher node has been started!')
        self.get_logger().info(f'Publishing to /chatter at {1/timer_period} Hz')

    def timer_callback(self):
        """
        Timer callback function - called every 0.5 seconds.

        Creates and publishes a string message with an incrementing counter.
        """
        # Create a String message
        msg = String()
        msg.data = f'Hello World: {self.i}'

        # Publish the message
        self.publisher_.publish(msg)

        # Log the published message
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment the counter
        self.i += 1


def main(args=None):
    """
    Main function to initialize and run the node.

    Args:
        args: Command-line arguments (default: None)
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of the MinimalPublisher node
    minimal_publisher = MinimalPublisher()

    try:
        # Spin the node to process callbacks
        # This keeps the node running and processing timer callbacks
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        minimal_publisher.get_logger().info('Keyboard interrupt detected, shutting down...')
    finally:
        # Clean up: destroy the node and shutdown rclpy
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
