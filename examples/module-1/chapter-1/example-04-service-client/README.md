# Example 1.4: Service Client Node in Python

## Description

This example demonstrates calling a ROS 2 service from a client. Works with Example 1.3 (Service Server).

## Running

**Terminal 1** - Start server:
```bash
cd examples/module-1/chapter-1/example-03-service-server
python3 service_server.py
```

**Terminal 2** - Call service:
```bash
cd examples/module-1/chapter-1/example-04-service-client
python3 service_client.py 5 3
```

**Expected Output**:
```
[INFO] [service_client]: Service client ready!
[INFO] [service_client]: Sending request: 5 + 3
[INFO] [service_client]: Result: 5 + 3 = 8
```

## Testing

```bash
./test.sh
```

## References

- [ROS 2 - Writing a Service Client (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
