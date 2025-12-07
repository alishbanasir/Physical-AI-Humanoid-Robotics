# Example 1.2: Minimal Subscriber Node in Python

## Description

This example demonstrates the subscriber side of the publish-subscribe pattern in ROS 2. The `MinimalSubscriber` node creates a subscriber that listens to the `/chatter` topic and processes incoming string messages. This complements Example 1.1 (Minimal Publisher) to show bidirectional topic-based communication.

## Learning Objectives

After completing this example, you will understand:
- How to create a subscriber in ROS 2 using `rclpy`
- How to handle incoming messages with callback functions
- How callbacks are processed by the ROS 2 executor
- The asynchronous nature of topic-based communication
- Message counting and basic state management in nodes

## Dependencies

- **ROS 2 Humble** (`ros-humble-desktop` or `ros-humble-ros-base`)
- **Python 3.10+** (included with Ubuntu 22.04)
- **rclpy** (ROS 2 Python client library)
- **std_msgs** (Standard ROS 2 message types)

## Setup

### 1. Source ROS 2 Environment

```bash
source /opt/ros/humble/setup.bash
```

### 2. Navigate to Example Directory

```bash
cd examples/module-1/chapter-1/example-02-minimal-subscriber
```

### 3. Make Script Executable (Optional)

```bash
chmod +x minimal_subscriber.py
```

## Running the Example

### Standalone (Without Publisher)

```bash
python3 minimal_subscriber.py
```

**Expected Output**:
```
[INFO] [minimal_subscriber]: Minimal Subscriber node has been started!
[INFO] [minimal_subscriber]: Listening on /chatter topic...
```

The node will wait for messages. Nothing will be printed until a publisher starts sending data.

### With Publisher (Complete System)

**Terminal 1** - Start the subscriber:
```bash
cd examples/module-1/chapter-1/example-02-minimal-subscriber
python3 minimal_subscriber.py
```

**Terminal 2** - Start the publisher from Example 1.1:
```bash
cd examples/module-1/chapter-1/example-01-minimal-publisher
python3 minimal_publisher.py
```

## Expected Output

When both nodes are running, the subscriber should display:

```
[INFO] [minimal_subscriber]: Minimal Subscriber node has been started!
[INFO] [minimal_subscriber]: Listening on /chatter topic...
[INFO] [minimal_subscriber]: I heard: "Hello World: 0" (Message #1)
[INFO] [minimal_subscriber]: I heard: "Hello World: 1" (Message #2)
[INFO] [minimal_subscriber]: I heard: "Hello World: 2" (Message #3)
[INFO] [minimal_subscriber]: I heard: "Hello World: 3" (Message #4)
...
```

When you press `Ctrl+C`:
```
[INFO] [minimal_subscriber]: Keyboard interrupt detected. Total messages received: 15
```

## Validation

### Check Active Topics

In a new terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 topic list
```

You should see `/chatter` in the list.

### Inspect Topic Details

```bash
ros2 topic info /chatter
```

Expected output (with both publisher and subscriber running):
```
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 1
```

### Monitor Topic Data

```bash
ros2 topic echo /chatter
```

You should see the messages being published.

## Testing

Run the automated test script:

```bash
./test.sh
```

This script will:
1. Start both publisher and subscriber in background
2. Wait for nodes to initialize
3. Verify the `/chatter` topic exists
4. Check that subscriber is receiving messages
5. Validate message reception rate
6. Clean up and report results

Expected output: `All tests passed ✓`

## Code Structure Explained

### Node Initialization

```python
super().__init__('minimal_subscriber')
```
- Creates a ROS 2 node with the name `'minimal_subscriber'`
- Node names must be unique within a namespace

### Subscriber Creation

```python
self.subscription = self.create_subscription(
    String,
    '/chatter',
    self.listener_callback,
    10
)
```
- **Message Type**: `String` from `std_msgs.msg`
- **Topic Name**: `'/chatter'` (matches publisher)
- **Callback**: `self.listener_callback` function
- **Queue Size**: `10` (buffers last 10 messages if processing is slow)

### Callback Function

```python
def listener_callback(self, msg):
    self.message_count += 1
    self.get_logger().info(f'I heard: "{msg.data}"')
```
- Called automatically when a message arrives
- Receives the message as a parameter
- Can access and modify node state (`self.message_count`)
- Non-blocking: returns quickly to process next message

### Message Processing

The callback is invoked by `rclpy.spin()`, which:
1. Monitors all subscriptions
2. Calls appropriate callbacks when messages arrive
3. Handles multiple subscriptions concurrently

## Common Issues and Solutions

### Issue: "No messages received"

**Solution**: Ensure a publisher is running on `/chatter`:
```bash
# In another terminal
ros2 topic list | grep chatter
ros2 topic info /chatter  # Should show Publisher count: 1
```

### Issue: "Messages arrive slowly or out of order"

**Solution**: This is expected behavior for asynchronous communication:
- Network delays can cause latency
- QoS settings affect reliability vs. speed
- For ordered delivery, use appropriate QoS profiles

### Issue: "Subscriber misses some messages"

**Possible Causes**:
1. **Late Joiner**: Subscriber started after publisher already sent messages
   - Solution: Use `TRANSIENT_LOCAL` durability in QoS
2. **Queue Overflow**: Processing too slow, queue size exceeded
   - Solution: Increase queue size or optimize callback
3. **Best Effort QoS**: Some messages dropped intentionally
   - Solution: Use `RELIABLE` QoS for critical data

## Exercises

### Beginner

1. **Message Filtering**: Modify the callback to only log messages containing the word "World"
2. **Statistics**: Add logging for average messages per second
3. **Multiple Subscriptions**: Create a node that subscribes to two different topics

### Intermediate

4. **QoS Configuration**: Implement custom QoS with `RELIABLE` reliability
5. **Data Processing**: Calculate and log a running average if messages contain numbers
6. **Selective Logging**: Only log every 10th message to reduce console output

### Advanced

7. **Late Joiner Recovery**: Use `TRANSIENT_LOCAL` durability to receive the last message even if subscriber starts late
8. **Multi-Threaded Callback**: Implement a multi-threaded executor for concurrent message processing
9. **Performance Monitoring**: Measure callback execution time and detect processing bottlenecks

## Next Steps

- **Example 1.3**: Implement a service server for request-response communication
- **Example 1.4**: Create a service client to call the server
- **Combine Examples**: Create a system with multiple publishers and subscribers

## References

- [ROS 2 Humble - Writing a Simple Publisher and Subscriber (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [rclpy Subscription API](https://docs.ros2.org/humble/api/rclpy/api/node.html#rclpy.node.Node.create_subscription)
- [Understanding ROS 2 Callbacks](https://docs.ros.org/en/humble/Concepts/About-Executors.html)
