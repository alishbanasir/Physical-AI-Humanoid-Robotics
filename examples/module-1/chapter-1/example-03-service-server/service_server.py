#!/usr/bin/env python3
"""
Service Server Node - ROS 2 Example

This node demonstrates the server side of service-based communication.
It provides an 'add_two_ints' service that adds two integers and returns the sum.

Learning Objectives:
- Create a ROS 2 service server
- Handle service requests synchronously
- Return responses to clients
- Understand request-response communication patterns

Author: ROS 2 Fundamentals Module
License: Apache 2.0
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class ServiceServer(Node):
    """
    A service server node that provides an addition service.

    This class inherits from rclpy.node.Node and creates a service
    that accepts two integers and returns their sum.
    """

    def __init__(self):
        """
        Initialize the ServiceServer node.

        Creates a service '/add_two_ints' that processes addition requests.
        """
        # Call the parent class constructor with node name
        super().__init__('service_server')

        # Create a service
        # Parameters:
        #   - srv_type: The service type (AddTwoInts)
        #   - service_name: The service name ('/add_two_ints')
        #   - callback: Function to call when request received
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

        # Track number of requests processed
        self.request_count = 0

        self.get_logger().info('Service Server node has been started!')
        self.get_logger().info('Service "/add_two_ints" is ready.')
        self.get_logger().info('Waiting for client requests...')

    def add_two_ints_callback(self, request, response):
        """
        Service callback function - processes addition requests.

        This function is called when a client sends a request to the service.
        It calculates the sum and returns it in the response.

        Args:
            request (AddTwoInts.Request): Contains 'a' and 'b' integers
            response (AddTwoInts.Response): Will contain 'sum' result

        Returns:
            AddTwoInts.Response: The response with the calculated sum
        """
        # Increment request counter
        self.request_count += 1

        # Calculate the sum
        response.sum = request.a + request.b

        # Log the request and response
        self.get_logger().info(
            f'Request #{self.request_count}: {request.a} + {request.b} = {response.sum}'
        )

        # Return the response to the client
        return response


def main(args=None):
    """
    Main function to initialize and run the node.

    Args:
        args: Command-line arguments (default: None)
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of the ServiceServer node
    service_server = ServiceServer()

    try:
        # Spin the node to process service requests
        # This keeps the node running and handles incoming requests
        rclpy.spin(service_server)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        service_server.get_logger().info(
            f'Shutting down. Total requests processed: {service_server.request_count}'
        )
    finally:
        # Clean up: destroy the node and shutdown rclpy
        service_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
