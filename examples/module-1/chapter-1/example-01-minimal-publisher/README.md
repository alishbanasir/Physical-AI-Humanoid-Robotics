# Example 1.1: Minimal Publisher Node in Python

## Description

This example demonstrates the fundamental publish-subscribe pattern in ROS 2. The `MinimalPublisher` node creates a publisher that sends string messages to the `/chatter` topic at 2 Hz (every 0.5 seconds). This is the simplest possible ROS 2 publisher and serves as the foundation for understanding topic-based communication.

## Learning Objectives

After completing this example, you will understand:
- How to create a basic ROS 2 node using `rclpy`
- How to set up a publisher for a specific topic and message type
- How to use timers for periodic callbacks
- How to properly initialize and shut down a ROS 2 node
- The basic structure of a ROS 2 Python node

## Dependencies

- **ROS 2 Humble** (`ros-humble-desktop` or `ros-humble-ros-base`)
- **Python 3.10+** (included with Ubuntu 22.04)
- **rclpy** (ROS 2 Python client library - included in ROS 2 installation)
- **std_msgs** (Standard ROS 2 message types - included in ROS 2 installation)

## Setup

### 1. Source ROS 2 Environment

```bash
source /opt/ros/humble/setup.bash
```

### 2. Navigate to Example Directory

```bash
cd examples/module-1/chapter-1/example-01-minimal-publisher
```

### 3. Make Script Executable (Optional)

```bash
chmod +x minimal_publisher.py
```

## Running the Example

### Option 1: Using Python Directly

```bash
python3 minimal_publisher.py
```

### Option 2: Using ROS 2 Run (if installed as package)

```bash
ros2 run chapter1_examples minimal_publisher
```

## Expected Output

When you run the publisher, you should see output similar to:

```
[INFO] [1701234567.123456789] [minimal_publisher]: Minimal Publisher node has been started!
[INFO] [1701234567.123456790] [minimal_publisher]: Publishing to /chatter at 2.0 Hz
[INFO] [1701234567.623456789] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [1701234568.123456789] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [1701234568.623456789] [minimal_publisher]: Publishing: "Hello World: 2"
[INFO] [1701234569.123456789] [minimal_publisher]: Publishing: "Hello World: 3"
...
```

The node will continue publishing until you press `Ctrl+C`.

## Validation

### Check Active Topics

In a new terminal (while the publisher is running):

```bash
source /opt/ros/humble/setup.bash
ros2 topic list
```

You should see `/chatter` in the list of active topics.

### Inspect Topic Details

```bash
ros2 topic info /chatter
```

Expected output:
```
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 0
```

### Echo Messages

To see the messages being published in real-time:

```bash
ros2 topic echo /chatter
```

You should see:
```
data: Hello World: 5
---
data: Hello World: 6
---
data: Hello World: 7
---
...
```

### Measure Publication Rate

```bash
ros2 topic hz /chatter
```

Expected output (approximately):
```
average rate: 2.000
  min: 0.499s max: 0.501s std dev: 0.00050s window: 10
```

## Testing

Run the automated test script:

```bash
./test.sh
```

This script will:
1. Start the publisher node in the background
2. Wait for the node to initialize
3. Verify the `/chatter` topic exists
4. Check that messages are being published
5. Measure the publication rate
6. Clean up and report results

Expected output: `All tests passed ✓`

## Code Structure Explained

### Node Initialization

```python
super().__init__('minimal_publisher')
```
- Creates a ROS 2 node with the name `'minimal_publisher'`
- Node names must be unique within a namespace

### Publisher Creation

```python
self.publisher_ = self.create_publisher(String, '/chatter', 10)
```
- **Message Type**: `String` from `std_msgs.msg`
- **Topic Name**: `'/chatter'`
- **Queue Size**: `10` (buffers last 10 messages if subscriber is slow)

### Timer Setup

```python
self.timer = self.create_timer(0.5, self.timer_callback)
```
- **Period**: `0.5` seconds (2 Hz publication rate)
- **Callback**: `self.timer_callback` function called every period

### Message Publication

```python
msg = String()
msg.data = f'Hello World: {self.i}'
self.publisher_.publish(msg)
```
- Create a `String` message
- Set the `data` field
- Publish using the publisher instance

## Common Issues and Solutions

### Issue: "ros2: command not found"

**Solution**: Source the ROS 2 environment:
```bash
source /opt/ros/humble/setup.bash
```

Add to `~/.bashrc` for automatic sourcing:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Issue: "ModuleNotFoundError: No module named 'rclpy'"

**Solution**: Ensure ROS 2 is properly installed and sourced:
```bash
sudo apt update
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash
```

### Issue: Node doesn't publish anything

**Solution**: Check that the timer is being called:
- Add debug logging in `timer_callback()`
- Verify `rclpy.spin()` is being called (it processes callbacks)
- Ensure no exceptions are being silently caught

## Exercises

### Beginner

1. **Change Publication Rate**: Modify the timer period to publish at 5 Hz instead of 2 Hz
2. **Custom Messages**: Change the message content to include a timestamp
3. **Add Counter**: Log the total number of messages published when shutting down

### Intermediate

4. **QoS Configuration**: Implement custom QoS settings using `QoSProfile`
5. **Parameter-Based Rate**: Make the publication rate configurable via ROS 2 parameters
6. **Multiple Publishers**: Create a node with multiple publishers on different topics

### Advanced

7. **Performance Analysis**: Measure actual vs. expected publication timing and calculate jitter
8. **Graceful Shutdown**: Implement signal handling for clean shutdown and publish a final "goodbye" message

## Next Steps

- **Example 1.2**: Create a subscriber node to receive these messages
- **Example 1.3**: Implement service-based communication
- **Example 1.4**: Explore action servers and clients

## References

- [ROS 2 Humble - Writing a Simple Publisher and Subscriber (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [rclpy API Documentation](https://docs.ros2.org/humble/api/rclpy/)
- [std_msgs Documentation](https://docs.ros2.org/humble/api/std_msgs/)
