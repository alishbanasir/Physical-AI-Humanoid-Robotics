# Example 1.5: Action Server Node in Python

## Description

Demonstrates ROS 2 action server for long-running, goal-oriented tasks with feedback.

## Running

```bash
python3 action_server.py
```

**Test with CLI**:
```bash
ros2 action send_goal /countdown example_interfaces/action/Fibonacci "{order: 5}"
```

**Expected Output**:
```
[INFO] [countdown_action_server]: Executing goal: count down from 5
[INFO] [countdown_action_server]: Feedback: 5 remaining...
[INFO] [countdown_action_server]: Feedback: 4 remaining...
[INFO] [countdown_action_server]: Feedback: 3 remaining...
[INFO] [countdown_action_server]: Feedback: 2 remaining...
[INFO] [countdown_action_server]: Feedback: 1 remaining...
[INFO] [countdown_action_server]: Feedback: 0 remaining...
[INFO] [countdown_action_server]: Goal succeeded!
```

## Testing

```bash
./test.sh
```

## Key Features

- ✅ Goal-based execution
- ✅ Periodic feedback
- ✅ Cancellation support
- ✅ Result reporting

## References

- [ROS 2 - Writing an Action Server (Python)](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)
