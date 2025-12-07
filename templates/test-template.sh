#!/bin/bash
#
# Test script for Example [N].[M]: [Example Title]
#
# This script validates that the example code executes successfully
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
EXAMPLE_NAME="[example-slug]"
DOCKER_IMAGE="osrf/ros:humble-desktop"
TIMEOUT=30  # Timeout in seconds

echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}Testing: ${EXAMPLE_NAME}${NC}"
echo -e "${YELLOW}========================================${NC}"

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    echo -e "${RED}ERROR: Docker is not running${NC}"
    exit 1
fi

# Check if example files exist
if [ ! -f "[main_file].py" ]; then
    echo -e "${RED}ERROR: Main file not found: [main_file].py${NC}"
    exit 1
fi

echo -e "${GREEN}Step 1: Starting Docker container...${NC}"

# Run the example in Docker
docker run --rm \
    -v "$PWD:/workspace" \
    -w /workspace \
    "$DOCKER_IMAGE" \
    bash -c "
        set -e
        source /opt/ros/humble/setup.bash

        echo 'Step 2: Installing dependencies...'
        # [Add any additional pip install or apt-get commands here]

        echo 'Step 3: Running example...'
        timeout $TIMEOUT python3 [main_file].py &
        EXAMPLE_PID=\$!

        # Wait for node to start
        sleep 2

        # [Add validation checks here]
        # Example: Check if ROS 2 node is running
        # ros2 node list | grep '[node_name]' || exit 1

        # Stop the example
        kill \$EXAMPLE_PID 2>/dev/null || true
        wait \$EXAMPLE_PID 2>/dev/null || true

        echo 'Step 4: Validation complete.'
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
    echo -e "${RED}Exit code: ${EXIT_CODE}${NC}"
    echo -e "${RED}========================================${NC}"
    exit 1
fi
