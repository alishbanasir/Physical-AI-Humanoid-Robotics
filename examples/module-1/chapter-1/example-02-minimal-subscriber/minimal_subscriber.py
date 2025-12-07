#!/usr/bin/env python3
"""
Minimal Subscriber Node - ROS 2 Example

This node demonstrates the subscriber side of the publish-subscribe pattern.
It subscribes to the '/chatter' topic and receives string messages.

Learning Objectives:
- Create a ROS 2 subscriber node
- Process incoming messages with callbacks
- Understand topic-based communication
- Handle message reception

Author: ROS 2 Fundamentals Module
License: Apache 2.0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    """
    A minimal subscriber node that receives string messages.

    This class inherits from rclpy.node.Node and creates a subscriber
    that listens to the '/chatter' topic and processes incoming messages.
    """

    def __init__(self):
        """
        Initialize the MinimalSubscriber node.

        Creates a subscriber on '/chatter' topic that calls listener_callback
        whenever a message is received.
        """
        # Call the parent class constructor with node name
        super().__init__('minimal_subscriber')

        # Create a subscriber
        # Parameters:
        #   - msg_type: The message type (String from std_msgs)
        #   - topic: The topic name ('/chatter')
        #   - callback: Function to call when message received
        #   - qos_profile: Queue size (10) - buffers last 10 messages
        self.subscription = self.create_subscription(
            String,
            '/chatter',
            self.listener_callback,
            10
        )

        # Prevent unused variable warning
        self.subscription

        # Initialize message counter
        self.message_count = 0

        self.get_logger().info('Minimal Subscriber node has been started!')
        self.get_logger().info('Listening on /chatter topic...')

    def listener_callback(self, msg):
        """
        Callback function - called whenever a message is received.

        This function is invoked automatically by rclpy.spin() when a new
        message arrives on the '/chatter' topic.

        Args:
            msg (String): The received message from the topic
        """
        # Increment message counter
        self.message_count += 1

        # Log the received message
        self.get_logger().info(f'I heard: "{msg.data}" (Message #{self.message_count})')


def main(args=None):
    """
    Main function to initialize and run the node.

    Args:
        args: Command-line arguments (default: None)
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of the MinimalSubscriber node
    minimal_subscriber = MinimalSubscriber()

    try:
        # Spin the node to process callbacks
        # This keeps the node running and processes incoming messages
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        minimal_subscriber.get_logger().info(
            f'Keyboard interrupt detected. Total messages received: {minimal_subscriber.message_count}'
        )
    finally:
        # Clean up: destroy the node and shutdown rclpy
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
