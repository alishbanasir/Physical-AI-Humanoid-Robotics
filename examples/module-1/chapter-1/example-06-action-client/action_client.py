#!/usr/bin/env python3
"""
Action Client Node - ROS 2 Example

Demonstrates sending goals to an action server and receiving feedback.

Author: ROS 2 Fundamentals Module
License: Apache 2.0
"""

import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci


class CountdownActionClient(Node):
    """Action client that sends countdown goals."""

    def __init__(self):
        super().__init__('countdown_action_client')

        self._action_client = ActionClient(
            self,
            Fibonacci,
            'countdown'
        )

        self.get_logger().info('Action Client started!')

    def send_goal(self, order):
        """Send a goal to the action server."""
        self.get_logger().info(f'Waiting for action server...')

        self._action_client.wait_for_server()

        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self.get_logger().info(f'Sending goal: countdown from {order}')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted!')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Process feedback from server."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.sequence}')

    def get_result_callback(self, future):
        """Process final result."""
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        self.get_logger().info('Goal completed!')


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print('Usage: action_client.py <countdown_from>')
        print('Example: action_client.py 5')
        sys.exit(1)

    try:
        count_from = int(sys.argv[1])
    except ValueError:
        print('ERROR: Argument must be an integer')
        sys.exit(1)

    action_client = CountdownActionClient()
    action_client.send_goal(count_from)

    rclpy.spin(action_client)

    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
