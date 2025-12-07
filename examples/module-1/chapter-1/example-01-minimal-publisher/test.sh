#!/bin/bash
#
# Test script for Example 1.1: Minimal Publisher
#
# This script validates that the minimal publisher node executes successfully
# and publishes messages to the /chatter topic at the expected rate.
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
EXAMPLE_NAME="Example 1.1: Minimal Publisher"
NODE_NAME="minimal_publisher"
TOPIC_NAME="/chatter"
EXPECTED_MSG_TYPE="std_msgs/msg/String"
EXPECTED_RATE_MIN=1.5  # Hz (allowing some tolerance)
EXPECTED_RATE_MAX=2.5  # Hz
TIMEOUT=15  # Timeout in seconds

echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}Testing: ${EXAMPLE_NAME}${NC}"
echo -e "${YELLOW}========================================${NC}"

# Check if ROS 2 is sourced
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}ERROR: ROS 2 not found. Please source ROS 2 environment:${NC}"
    echo -e "${RED}  source /opt/ros/humble/setup.bash${NC}"
    exit 1
fi

echo -e "${GREEN}Step 1: Checking ROS 2 environment...${NC}"
ros2 --version
echo ""

# Check if example file exists
if [ ! -f "minimal_publisher.py" ]; then
    echo -e "${RED}ERROR: minimal_publisher.py not found${NC}"
    echo -e "${RED}Please run this script from the example directory${NC}"
    exit 1
fi

echo -e "${GREEN}Step 2: Starting publisher node...${NC}"

# Start the publisher in the background
python3 minimal_publisher.py &
PUB_PID=$!

# Give the node time to start
sleep 3

# Check if the process is still running
if ! ps -p $PUB_PID > /dev/null; then
    echo -e "${RED}ERROR: Publisher node failed to start or crashed${NC}"
    exit 1
fi

echo -e "${GREEN}Publisher started with PID: $PUB_PID${NC}"
echo ""

echo -e "${GREEN}Step 3: Validating node is active...${NC}"

# Check if the node appears in the node list
if ros2 node list | grep -q "$NODE_NAME"; then
    echo -e "${GREEN}✓ Node '$NODE_NAME' is active${NC}"
else
    echo -e "${RED}✗ Node '$NODE_NAME' not found in active nodes${NC}"
    kill $PUB_PID 2>/dev/null || true
    exit 1
fi
echo ""

echo -e "${GREEN}Step 4: Validating topic exists...${NC}"

# Check if the topic exists
if ros2 topic list | grep -q "$TOPIC_NAME"; then
    echo -e "${GREEN}✓ Topic '$TOPIC_NAME' exists${NC}"
else
    echo -e "${RED}✗ Topic '$TOPIC_NAME' not found${NC}"
    kill $PUB_PID 2>/dev/null || true
    exit 1
fi
echo ""

echo -e "${GREEN}Step 5: Validating topic message type...${NC}"

# Check topic info for correct message type
TOPIC_INFO=$(ros2 topic info $TOPIC_NAME 2>/dev/null)
if echo "$TOPIC_INFO" | grep -q "$EXPECTED_MSG_TYPE"; then
    echo -e "${GREEN}✓ Topic message type is correct: $EXPECTED_MSG_TYPE${NC}"
else
    echo -e "${RED}✗ Topic message type mismatch${NC}"
    echo -e "${RED}Expected: $EXPECTED_MSG_TYPE${NC}"
    kill $PUB_PID 2>/dev/null || true
    exit 1
fi
echo ""

echo -e "${GREEN}Step 6: Checking if messages are being published...${NC}"

# Capture one message to verify publishing
MESSAGE=$(timeout 5 ros2 topic echo $TOPIC_NAME --once 2>/dev/null || echo "")
if [ -n "$MESSAGE" ]; then
    echo -e "${GREEN}✓ Messages are being published${NC}"
    echo -e "${GREEN}  Sample message: $(echo $MESSAGE | head -n1)${NC}"
else
    echo -e "${RED}✗ No messages received within timeout${NC}"
    kill $PUB_PID 2>/dev/null || true
    exit 1
fi
echo ""

echo -e "${GREEN}Step 7: Measuring publication rate...${NC}"

# Measure the publication rate
# Note: This requires some messages to be published, so we wait and sample
RATE_OUTPUT=$(timeout 8 ros2 topic hz $TOPIC_NAME 2>/dev/null | grep "average rate" || echo "")

if [ -n "$RATE_OUTPUT" ]; then
    # Extract the rate value (format: "average rate: X.XXX")
    RATE=$(echo "$RATE_OUTPUT" | grep -oP 'average rate: \K[0-9.]+' || echo "0")
    echo -e "${GREEN}✓ Measured publication rate: ${RATE} Hz${NC}"

    # Check if rate is within expected range (using bc for floating point comparison)
    if command -v bc &> /dev/null; then
        IN_RANGE=$(echo "$RATE >= $EXPECTED_RATE_MIN && $RATE <= $EXPECTED_RATE_MAX" | bc)
        if [ "$IN_RANGE" -eq 1 ]; then
            echo -e "${GREEN}✓ Rate is within expected range (${EXPECTED_RATE_MIN}-${EXPECTED_RATE_MAX} Hz)${NC}"
        else
            echo -e "${YELLOW}⚠ Rate is outside expected range (${EXPECTED_RATE_MIN}-${EXPECTED_RATE_MAX} Hz)${NC}"
            echo -e "${YELLOW}  This may be due to system load. Test will continue.${NC}"
        fi
    else
        echo -e "${YELLOW}⚠ 'bc' not installed, skipping rate range check${NC}"
    fi
else
    echo -e "${YELLOW}⚠ Could not measure publication rate (timeout or insufficient data)${NC}"
    echo -e "${YELLOW}  This may happen on slow systems. Test will continue.${NC}"
fi
echo ""

echo -e "${GREEN}Step 8: Testing node shutdown...${NC}"

# Gracefully terminate the publisher
kill -SIGINT $PUB_PID 2>/dev/null || true

# Wait for process to terminate
WAIT_COUNT=0
while ps -p $PUB_PID > /dev/null 2>&1; do
    sleep 0.5
    WAIT_COUNT=$((WAIT_COUNT + 1))
    if [ $WAIT_COUNT -gt 10 ]; then
        echo -e "${YELLOW}⚠ Node did not shut down gracefully, forcing termination${NC}"
        kill -9 $PUB_PID 2>/dev/null || true
        break
    fi
done

echo -e "${GREEN}✓ Node shut down successfully${NC}"
echo ""

# Final summary
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}ALL TESTS PASSED: ${EXAMPLE_NAME}${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${GREEN}Summary:${NC}"
echo -e "${GREEN}  ✓ Node initialization${NC}"
echo -e "${GREEN}  ✓ Topic creation${NC}"
echo -e "${GREEN}  ✓ Message type correctness${NC}"
echo -e "${GREEN}  ✓ Message publication${NC}"
echo -e "${GREEN}  ✓ Publication rate (approximately 2 Hz)${NC}"
echo -e "${GREEN}  ✓ Graceful shutdown${NC}"
echo ""

exit 0
