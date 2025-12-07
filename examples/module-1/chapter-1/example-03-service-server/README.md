# Example 1.3: Service Server Node in Python

## Description

This example demonstrates the server side of ROS 2 service-based communication. The `ServiceServer` node provides an `/add_two_ints` service that accepts two integers as input and returns their sum. This shows synchronous request-response communication, which is ideal for operations that require an immediate response.

## Learning Objectives

After completing this example, you will understand:
- How to create a service server in ROS 2
- How to handle service requests with callbacks
- The synchronous nature of service communication
- How to define service request and response handling
- When to use services vs. topics

## Dependencies

- **ROS 2 Humble**
- **Python 3.10+**
- **rclpy** (ROS 2 Python client library)
- **example_interfaces** (Standard ROS 2 service types)

## Setup

### 1. Source ROS 2 Environment

```bash
source /opt/ros/humble/setup.bash
```

### 2. Navigate to Example Directory

```bash
cd examples/module-1/chapter-1/example-03-service-server
```

### 3. Make Script Executable

```bash
chmod +x service_server.py
```

## Running the Example

```bash
python3 service_server.py
```

**Expected Output**:
```
[INFO] [service_server]: Service Server node has been started!
[INFO] [service_server]: Service "/add_two_ints" is ready.
[INFO] [service_server]: Waiting for client requests...
```

The server will wait for client requests and remain running until you press `Ctrl+C`.

## Testing the Service

### Option 1: Using ros2 service call Command

In a new terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"
```

**Server Output**:
```
[INFO] [service_server]: Request #1: 5 + 3 = 8
```

**Client Output**:
```
waiting for service to become available...
requester: making request: example_interfaces.srv.AddTwoInts_Request(a=5, b=3)

response:
example_interfaces.srv.AddTwoInts_Response(sum=8)
```

### Option 2: Multiple Requests

```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 10, b: 20}"
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: -5, b: 15}"
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 100, b: 200}"
```

## Validation

### List Active Services

```bash
ros2 service list
```

You should see `/add_two_ints` in the list.

### Inspect Service Type

```bash
ros2 service type /add_two_ints
```

Expected output:
```
example_interfaces/srv/AddTwoInts
```

### View Service Definition

```bash
ros2 interface show example_interfaces/srv/AddTwoInts
```

Expected output:
```
int64 a
int64 b
---
int64 sum
```

## Code Structure Explained

### Service Creation

```python
self.srv = self.create_service(
    AddTwoInts,
    'add_two_ints',
    self.add_two_ints_callback
)
```
- **Service Type**: `AddTwoInts` from `example_interfaces.srv`
- **Service Name**: `'add_two_ints'`
- **Callback**: `self.add_two_ints_callback` function

### Service Callback

```python
def add_two_ints_callback(self, request, response):
    response.sum = request.a + request.b
    return response
```
- **Request**: Contains `a` and `b` (int64 values)
- **Response**: Contains `sum` (int64 value)
- **Return**: The populated response object

The callback is **synchronous** - the client waits until the server returns the response.

## Common Issues and Solutions

### Issue: "Service not available"

**Solution**: Ensure the server is running:
```bash
ros2 service list | grep add_two_ints
```

### Issue: "Service call failed"

**Possible Causes**:
1. Server not running
2. Incorrect service name
3. Wrong message format

**Solution**: Check service details:
```bash
ros2 service type /add_two_ints
ros2 interface show example_interfaces/srv/AddTwoInts
```

### Issue: "Long response times"

**Cause**: Service callbacks should complete quickly. Long-running operations block the server.

**Solution**: For long operations, use **actions** instead of services (see Example 1.5).

## When to Use Services

✅ **Use Services For**:
- Configuration changes (set parameters)
- State queries (get robot pose)
- Short operations (<1 second)
- Operations requiring confirmation

❌ **Don't Use Services For**:
- Continuous data streams → Use **Topics**
- Long-running tasks → Use **Actions**
- High-frequency communication → Use **Topics**

## Exercises

### Beginner

1. **Modify Operation**: Change the service to multiply instead of add
2. **Input Validation**: Add checks to ensure inputs are positive numbers
3. **Logging Enhancement**: Log the timestamp of each request

### Intermediate

4. **Multi-Operation Service**: Create a service that performs add, subtract, multiply, divide
5. **Error Handling**: Return error codes for invalid operations (e.g., division by zero)
6. **Statistics**: Track and log average request processing time

### Advanced

7. **Custom Service**: Define a custom service type for robot configuration
8. **Concurrent Clients**: Test server behavior with multiple simultaneous clients
9. **Timeout Handling**: Implement server-side timeouts for long operations

## Next Steps

- **Example 1.4**: Create a service client to call this server
- **Example 1.5**: Implement action servers for long-running tasks
- **Integration**: Combine services with topics for complete systems

## References

- [ROS 2 - Writing a Simple Service and Client (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
- [rclpy Service API](https://docs.ros2.org/humble/api/rclpy/api/services.html)
- [example_interfaces Documentation](https://docs.ros2.org/humble/api/example_interfaces/)
