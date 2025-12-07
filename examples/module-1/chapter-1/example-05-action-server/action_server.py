#!/usr/bin/env python3
"""
Action Server Node - ROS 2 Example

Demonstrates action-based communication with goal, feedback, and result.
Simulates a countdown task that provides periodic feedback.

Author: ROS 2 Fundamentals Module
License: Apache 2.0
"""

import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci


class CountdownActionServer(Node):
    """
    Action server that counts down from a goal number to zero.
    Provides feedback on remaining count and returns final count.
    """

    def __init__(self):
        super().__init__('countdown_action_server')

        # Create action server
        self._action_server = ActionServer(
            self,
            Fibonacci,  # Using Fibonacci action as example
            'countdown',
            self.execute_callback
        )

        self.get_logger().info('Action Server started!')
        self.get_logger().info('Waiting for goals on "countdown" action...')

    def execute_callback(self, goal_handle):
        """
        Execute the countdown action.

        Args:
            goal_handle: The goal handle for this action execution

        Returns:
            Fibonacci.Result: The final result
        """
        self.get_logger().info(f'Executing goal: count down from {goal_handle.request.order}')

        # Initialize feedback
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = []

        # Count down
        for i in range(goal_handle.request.order, -1, -1):
            # Check if goal was cancelled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            # Add to sequence (simulate countdown)
            feedback_msg.sequence.append(i)

            # Publish feedback
            self.get_logger().info(f'Feedback: {i} remaining...')
            goal_handle.publish_feedback(feedback_msg)

            # Simulate work
            time.sleep(1)

        # Goal succeeded
        goal_handle.succeed()

        # Prepare result
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence

        self.get_logger().info('Goal succeeded!')
        return result


def main(args=None):
    rclpy.init(args=args)
    action_server = CountdownActionServer()

    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        pass

    action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
