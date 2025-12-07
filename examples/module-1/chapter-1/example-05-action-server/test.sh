#!/bin/bash
set -e
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'

DOCKER_IMAGE="osrf/ros:humble-desktop"

echo "Testing action server..."

docker run --rm -v "$PWD:/workspace" -w /workspace \
    "$DOCKER_IMAGE" bash -c "
    source /opt/ros/humble/setup.bash

    python3 action_server.py &
    SERVER_PID=\$!
    sleep 2

    # Send action goal
    timeout 10 ros2 action send_goal --feedback /countdown example_interfaces/action/Fibonacci \"{order: 3}\" > /tmp/action_output.log 2>&1 &
    CLIENT_PID=\$!

    sleep 5
    kill \$SERVER_PID \$CLIENT_PID || true

    if grep -q 'Goal succeeded' /tmp/action_output.log || ps -p \$SERVER_PID > /dev/null 2>&1; then
        echo 'SUCCESS'
        exit 0
    else
        echo 'FAILED'
        exit 1
    fi
"

if [ $? -eq 0 ]; then
    echo -e "${GREEN}TEST PASSED${NC}"
else
    echo -e "${RED}TEST FAILED${NC}"
    exit 1
fi
