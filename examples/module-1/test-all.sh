#!/bin/bash
#
# Master test script for Module 1 - ROS 2 Fundamentals
#
# This script runs all chapter test-all.sh scripts sequentially.
#
# Usage: ./test-all.sh
#
# Exit codes:
#   0 - All tests passed
#   1 - One or more tests failed

set -u  # Exit on undefined variable

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================"
echo -e "Module 1: ROS 2 Fundamentals"
echo -e "Master Test Suite"
echo -e "========================================${NC}"

# Test results tracking
TOTAL_CHAPTERS=0
PASSED_CHAPTERS=0
FAILED_CHAPTERS=0

# Array to store failed chapters
declare -a FAILED_CHAPTER_LIST

# Test each chapter
for CHAPTER_DIR in chapter-1 chapter-2 chapter-3 chapter-4; do
    if [ -d "$CHAPTER_DIR" ] && [ -f "$CHAPTER_DIR/test-all.sh" ]; then
        TOTAL_CHAPTERS=$((TOTAL_CHAPTERS + 1))

        echo ""
        echo -e "${YELLOW}========================================${NC}"
        echo -e "${YELLOW}Testing: $CHAPTER_DIR${NC}"
        echo -e "${YELLOW}========================================${NC}"

        # Run chapter test suite
        cd "$CHAPTER_DIR"
        if bash test-all.sh; then
            PASSED_CHAPTERS=$((PASSED_CHAPTERS + 1))
            echo -e "${GREEN}✓ $CHAPTER_DIR tests PASSED${NC}"
        else
            FAILED_CHAPTERS=$((FAILED_CHAPTERS + 1))
            FAILED_CHAPTER_LIST+=("$CHAPTER_DIR")
            echo -e "${RED}✗ $CHAPTER_DIR tests FAILED${NC}"
        fi
        cd ..
    fi
done

# Print summary
echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Module 1 Test Summary${NC}"
echo -e "${BLUE}========================================${NC}"
echo -e "Total chapters tested: ${TOTAL_CHAPTERS}"
echo -e "${GREEN}Passed: ${PASSED_CHAPTERS}${NC}"

if [ $FAILED_CHAPTERS -eq 0 ]; then
    echo -e "${RED}Failed: ${FAILED_CHAPTERS}${NC}"
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}ALL MODULE 1 TESTS PASSED!${NC}"
    echo -e "${GREEN}========================================${NC}"
    exit 0
else
    echo -e "${RED}Failed: ${FAILED_CHAPTERS}${NC}"
    echo ""
    echo -e "${RED}Failed chapters:${NC}"
    for chapter in "${FAILED_CHAPTER_LIST[@]}"; do
        echo -e "${RED}  - $chapter${NC}"
    done
    echo ""
    echo -e "${RED}========================================${NC}"
    echo -e "${RED}SOME MODULE 1 TESTS FAILED${NC}"
    echo -e "${RED}========================================${NC}"
    exit 1
fi
