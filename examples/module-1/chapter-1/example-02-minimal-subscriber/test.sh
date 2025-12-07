#!/bin/bash
#
# Test script for Example 1.2: Minimal Subscriber Node
#
# This script validates that the subscriber receives messages from a publisher
# in a Docker container with ROS 2 Humble.
#
# Usage: ./test.sh
#
# Exit codes:
#   0 - Test passed
#   1 - Test failed

set -e  # Exit on error
set -u  # Exit on undefined variable

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test configuration
EXAMPLE_NAME="example-02-minimal-subscriber"
DOCKER_IMAGE="osrf/ros:humble-desktop"
TIMEOUT=15  # Timeout in seconds

echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}Testing: ${EXAMPLE_NAME}${NC}"
echo -e "${YELLOW}========================================${NC}"

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    echo -e "${RED}ERROR: Docker is not running${NC}"
    exit 1
fi

# Check if example files exist
if [ ! -f "minimal_subscriber.py" ]; then
    echo -e "${RED}ERROR: Main file not found: minimal_subscriber.py${NC}"
    exit 1
fi

# Check if publisher exists (needed for complete test)
PUBLISHER_PATH="../example-01-minimal-publisher/minimal_publisher.py"
if [ ! -f "$PUBLISHER_PATH" ]; then
    echo -e "${YELLOW}WARNING: Publisher not found at $PUBLISHER_PATH${NC}"
    echo -e "${YELLOW}Will test subscriber in isolation (will wait for messages)${NC}"
fi

echo -e "${GREEN}Step 1: Starting Docker container...${NC}"

# Create a temporary directory for test outputs
TEMP_DIR=$(mktemp -d)
echo "Using temp directory: $TEMP_DIR"

# Run the test in Docker
docker run --rm \
    -v "$PWD/../:/workspace" \
    -w /workspace/example-02-minimal-subscriber \
    "$DOCKER_IMAGE" \
    bash -c "
        set -e
        source /opt/ros/humble/setup.bash

        echo 'Step 2: Testing subscriber node startup...'

        # Start subscriber in background
        timeout $TIMEOUT python3 minimal_subscriber.py > /tmp/subscriber_output.log 2>&1 &
        SUBSCRIBER_PID=\$!

        # Wait for subscriber to initialize
        sleep 2

        # Check if subscriber is running
        if ! ps -p \$SUBSCRIBER_PID > /dev/null 2>&1; then
            echo 'ERROR: Subscriber failed to start'
            cat /tmp/subscriber_output.log
            exit 1
        fi

        echo 'Step 3: Checking topic registration...'

        # Verify topic exists (subscriber should create it)
        if ! ros2 topic list | grep -q '/chatter'; then
            echo 'ERROR: /chatter topic not found'
            ros2 topic list
            exit 1
        fi

        echo 'Step 4: Starting publisher for integration test...'

        # Start publisher in background
        cd ../example-01-minimal-publisher
        timeout $TIMEOUT python3 minimal_publisher.py > /tmp/publisher_output.log 2>&1 &
        PUBLISHER_PID=\$!

        # Wait for publisher to start
        sleep 2

        # Give time for message exchange
        sleep 3

        echo 'Step 5: Validating message reception...'

        # Check subscriber output for received messages
        cd ../example-02-minimal-subscriber
        if grep -q 'I heard:' /tmp/subscriber_output.log; then
            echo 'SUCCESS: Subscriber received messages!'
            grep 'I heard:' /tmp/subscriber_output.log | head -5
        else
            echo 'ERROR: No messages received by subscriber'
            echo '--- Subscriber Log ---'
            cat /tmp/subscriber_output.log
            echo '--- Publisher Log ---'
            cat /tmp/publisher_output.log
            exit 1
        fi

        # Clean up processes
        kill \$SUBSCRIBER_PID 2>/dev/null || true
        kill \$PUBLISHER_PID 2>/dev/null || true
        wait \$SUBSCRIBER_PID 2>/dev/null || true
        wait \$PUBLISHER_PID 2>/dev/null || true

        echo 'Step 6: Validation complete.'
    "

EXIT_CODE=$?

# Clean up temp directory
rm -rf "$TEMP_DIR"

if [ $EXIT_CODE -eq 0 ]; then
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}TEST PASSED: ${EXAMPLE_NAME}${NC}"
    echo -e "${GREEN}========================================${NC}"
    exit 0
else
    echo -e "${RED}========================================${NC}"
    echo -e "${RED}TEST FAILED: ${EXAMPLE_NAME}${NC}"
    echo -e "${RED}Exit code: ${EXIT_CODE}${NC}"
    echo -e "${RED}========================================${NC}"
    exit 1
fi
