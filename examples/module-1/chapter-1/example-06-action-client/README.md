# Example 1.6: Action Client Node in Python

## Description

Demonstrates sending goals to an action server and receiving feedback.

## Running

**Terminal 1** - Start server:
```bash
cd examples/module-1/chapter-1/example-05-action-server
python3 action_server.py
```

**Terminal 2** - Send goal:
```bash
cd examples/module-1/chapter-1/example-06-action-client
python3 action_client.py 5
```

**Expected Output**:
```
[INFO] [countdown_action_client]: Action Client started!
[INFO] [countdown_action_client]: Sending goal: countdown from 5
[INFO] [countdown_action_client]: Goal accepted!
[INFO] [countdown_action_client]: Received feedback: [5]
[INFO] [countdown_action_client]: Received feedback: [5, 4]
...
[INFO] [countdown_action_client]: Result: [5, 4, 3, 2, 1, 0]
[INFO] [countdown_action_client]: Goal completed!
```

## Testing

```bash
./test.sh
```

## References

- [ROS 2 - Writing an Action Client (Python)](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)
