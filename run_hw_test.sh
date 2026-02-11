#!/usr/bin/env bash
# Hardware Test Runner with proper ROS 2 environment and sudo permissions
#
# Usage: ./run_hw_test.sh [args passed to test_runner.py]
# Example: ./run_hw_test.sh --category gantry --subtest limits

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "Sourcing ROS 2 Jazzy underlay..."
source /opt/ros/jazzy/setup.bash

if [ -f install/setup.bash ]; then
    echo "Sourcing workspace overlay..."
    source install/setup.bash
else
    echo "WARNING: install/setup.bash not found. Did you run colcon build?"
    echo "Continuing with underlay only..."
fi

echo "Running hardware tests with sudo (preserving environment)..."
echo ""

# Pass both PYTHONPATH and LD_LIBRARY_PATH explicitly
# sudo doesn't preserve these even with -E, so we set them explicitly
# Must run as root for /dev/mem access
sudo PYTHONPATH="$PYTHONPATH" LD_LIBRARY_PATH="$LD_LIBRARY_PATH" \
    python3 -m chess_hw_interface.testing.test_runner "$@"

exit_code=$?
if [ $exit_code -eq 0 ]; then
    echo ""
    echo "✓ Tests completed successfully"
else
    echo ""
    echo "✗ Tests failed with exit code $exit_code"
fi

exit $exit_code
