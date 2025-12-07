#!/bin/bash
#
# Master Test Script for Chapter 1 Examples
#
# Runs all example tests sequentially and reports results
#
# Usage: ./test-all.sh
#

set -u  # Exit on undefined variable

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Track results
TOTAL_TESTS=0
PASSED_TESTS=0
FAILED_TESTS=0

# Store failed test names
FAILED_LIST=()

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Chapter 1: Testing All Examples${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Function to run a single test
run_test() {
    local example_dir=$1
    local example_name=$2

    TOTAL_TESTS=$((TOTAL_TESTS + 1))

    echo -e "${YELLOW}[$TOTAL_TESTS] Testing: $example_name${NC}"
    echo "  Directory: $example_dir"

    if [ ! -d "$example_dir" ]; then
        echo -e "  ${RED}✗ Directory not found${NC}"
        FAILED_TESTS=$((FAILED_TESTS + 1))
        FAILED_LIST+=("$example_name (directory not found)")
        echo ""
        return 1
    fi

    if [ ! -f "$example_dir/test.sh" ]; then
        echo -e "  ${YELLOW}⚠ No test.sh found, skipping${NC}"
        echo ""
        return 0
    fi

    cd "$example_dir" || return 1

    # Run test with timeout
    if timeout 60 bash test.sh > /tmp/test_${TOTAL_TESTS}.log 2>&1; then
        echo -e "  ${GREEN}✓ PASSED${NC}"
        PASSED_TESTS=$((PASSED_TESTS + 1))
    else
        echo -e "  ${RED}✗ FAILED${NC}"
        FAILED_TESTS=$((FAILED_TESTS + 1))
        FAILED_LIST+=("$example_name")

        # Show last 10 lines of error log
        echo -e "  ${RED}Last 10 lines of output:${NC}"
        tail -10 /tmp/test_${TOTAL_TESTS}.log | sed 's/^/    /'
    fi

    cd - > /dev/null || return 1
    echo ""
}

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR" || exit 1

# Run all tests
run_test "example-01-minimal-publisher" "Example 1.1: Minimal Publisher"
run_test "example-02-minimal-subscriber" "Example 1.2: Minimal Subscriber"
run_test "example-03-service-server" "Example 1.3: Service Server"
run_test "example-04-service-client" "Example 1.4: Service Client"
run_test "example-05-action-server" "Example 1.5: Action Server"
run_test "example-06-action-client" "Example 1.6: Action Client"

# Print summary
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Test Summary${NC}"
echo -e "${BLUE}========================================${NC}"
echo -e "Total Tests:  $TOTAL_TESTS"
echo -e "${GREEN}Passed:       $PASSED_TESTS${NC}"

if [ $FAILED_TESTS -gt 0 ]; then
    echo -e "${RED}Failed:       $FAILED_TESTS${NC}"
    echo ""
    echo -e "${RED}Failed Tests:${NC}"
    for failed in "${FAILED_LIST[@]}"; do
        echo -e "  ${RED}✗${NC} $failed"
    done
else
    echo -e "${GREEN}Failed:       0${NC}"
fi

echo -e "${BLUE}========================================${NC}"
echo ""

# Clean up test logs
rm -f /tmp/test_*.log

# Exit with appropriate code
if [ $FAILED_TESTS -gt 0 ]; then
    echo -e "${RED}Some tests failed. Please review the output above.${NC}"
    exit 1
else
    echo -e "${GREEN}All tests passed! ✓${NC}"
    exit 0
fi
