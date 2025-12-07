#!/bin/bash
set -e
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'

DOCKER_IMAGE="osrf/ros:humble-desktop"

echo "Testing service client..."

docker run --rm -v "$PWD/../:/workspace" -w /workspace/example-04-service-client \
    "$DOCKER_IMAGE" bash -c "
    source /opt/ros/humble/setup.bash

    # Start server
    cd ../example-03-service-server
    python3 service_server.py &
    SERVER_PID=\$!
    sleep 2

    # Call client
    cd ../example-04-service-client
    RESULT=\$(python3 service_client.py 10 20 2>&1)

    kill \$SERVER_PID || true

    if echo \"\$RESULT\" | grep -q 'Result: 10 + 20 = 30'; then
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
