#!/usr/bin/env python3
"""
Service Client Node - ROS 2 Example

This node demonstrates calling a ROS 2 service from a client.
It sends requests to the 'add_two_ints' service and displays responses.

Author: ROS 2 Fundamentals Module
License: Apache 2.0
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class ServiceClient(Node):
    """A service client node that calls the addition service."""

    def __init__(self):
        super().__init__('service_client')

        # Create a client for the add_two_ints service
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to become available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.get_logger().info('Service client ready!')

    def send_request(self, a, b):
        """
        Send a request to add two integers.

        Args:
            a (int): First integer
            b (int): Second integer

        Returns:
            Future: A future object that will contain the response
        """
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        self.get_logger().info(f'Sending request: {a} + {b}')

        # Call the service asynchronously
        future = self.client.call_async(request)
        return future


def main(args=None):
    rclpy.init(args=args)

    # Get arguments from command line
    if len(sys.argv) < 3:
        print('Usage: service_client.py <a> <b>')
        print('Example: service_client.py 5 3')
        sys.exit(1)

    try:
        a = int(sys.argv[1])
        b = int(sys.argv[2])
    except ValueError:
        print('ERROR: Arguments must be integers')
        sys.exit(1)

    # Create client node
    client_node = ServiceClient()

    # Send request
    future = client_node.send_request(a, b)

    # Wait for response
    rclpy.spin_until_future_complete(client_node, future)

    if future.result() is not None:
        response = future.result()
        client_node.get_logger().info(
            f'Result: {a} + {b} = {response.sum}'
        )
    else:
        client_node.get_logger().error('Service call failed')

    # Clean up
    client_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
