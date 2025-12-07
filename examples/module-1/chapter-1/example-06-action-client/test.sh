#!/bin/bash
set -e
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'

DOCKER_IMAGE="osrf/ros:humble-desktop"

echo "Testing action client..."

docker run --rm -v "$PWD/../:/workspace" -w /workspace/example-06-action-client \
    "$DOCKER_IMAGE" bash -c "
    source /opt/ros/humble/setup.bash

    # Start server
    cd ../example-05-action-server
    python3 action_server.py &
    SERVER_PID=\$!
    sleep 2

    # Start client
    cd ../example-06-action-client
    timeout 10 python3 action_client.py 3 > /tmp/client_output.log 2>&1 &
    CLIENT_PID=\$!

    sleep 5
    kill \$SERVER_PID \$CLIENT_PID || true

    if grep -q 'Goal accepted' /tmp/client_output.log; then
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
