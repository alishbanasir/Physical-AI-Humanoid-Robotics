#!/bin/bash
#
# Test script for Example 1.3: Service Server Node
#
# Usage: ./test.sh
#

set -e
set -u

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

EXAMPLE_NAME="example-03-service-server"
DOCKER_IMAGE="osrf/ros:humble-desktop"
TIMEOUT=10

echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}Testing: ${EXAMPLE_NAME}${NC}"
echo -e "${YELLOW}========================================${NC}"

if ! docker info > /dev/null 2>&1; then
    echo -e "${RED}ERROR: Docker is not running${NC}"
    exit 1
fi

if [ ! -f "service_server.py" ]; then
    echo -e "${RED}ERROR: service_server.py not found${NC}"
    exit 1
fi

echo -e "${GREEN}Step 1: Starting Docker container...${NC}"

docker run --rm \
    -v "$PWD:/workspace" \
    -w /workspace \
    "$DOCKER_IMAGE" \
    bash -c "
        set -e
        source /opt/ros/humble/setup.bash

        echo 'Step 2: Starting service server...'
        timeout $TIMEOUT python3 service_server.py > /tmp/server_output.log 2>&1 &
        SERVER_PID=\$!

        sleep 2

        if ! ps -p \$SERVER_PID > /dev/null 2>&1; then
            echo 'ERROR: Server failed to start'
            cat /tmp/server_output.log
            exit 1
        fi

        echo 'Step 3: Checking service registration...'
        if ! ros2 service list | grep -q 'add_two_ints'; then
            echo 'ERROR: /add_two_ints service not found'
            ros2 service list
            exit 1
        fi

        echo 'Step 4: Calling service...'
        RESULT=\$(ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts \"{a: 5, b: 3}\" 2>&1)

        if echo \"\$RESULT\" | grep -q 'sum: 8'; then
            echo 'SUCCESS: Service returned correct sum (8)'
        else
            echo 'ERROR: Incorrect response from service'
            echo \"\$RESULT\"
            exit 1
        fi

        kill \$SERVER_PID 2>/dev/null || true
        wait \$SERVER_PID 2>/dev/null || true

        echo 'Step 5: Validation complete.'
    "

EXIT_CODE=$?

if [ $EXIT_CODE -eq 0 ]; then
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}TEST PASSED: ${EXAMPLE_NAME}${NC}"
    echo -e "${GREEN}========================================${NC}"
    exit 0
else
    echo -e "${RED}========================================${NC}"
    echo -e "${RED}TEST FAILED: ${EXAMPLE_NAME}${NC}"
    echo -e "${RED}========================================${NC}"
    exit 1
fi
