#!/usr/bin/env bash
# Hardware Test Runner with proper ROS 2 environment and sudo permissions
#
# Usage: ./run_hw_test.sh [args passed to test_runner.py]
# Example: ./run_hw_test.sh --category gantry --subtest limits
#
# Requires passwordless sudo (recommended, e.g. via a sudoers NOPASSWD rule for
# this command) or CHESS_SUDO_PASS set in the environment, since this needs to
# run non-interactively when launched from chess_os.py's subprocess:
#   export CHESS_SUDO_PASS=yourpassword

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

run_with_sudo() {
    if [ -n "${CHESS_SUDO_PASS:-}" ]; then
        echo "$CHESS_SUDO_PASS" | sudo -S "$@"
    elif sudo -n true 2>/dev/null; then
        sudo "$@"
    else
        echo "ERROR: sudo requires a password and none is available non-interactively." >&2
        echo "Set CHESS_SUDO_PASS in the environment, or configure passwordless sudo" >&2
        echo "for this command (see setup/sudoers)." >&2
        exit 1
    fi
}

echo "Sourcing ROS 2 humble underlay..."
source /opt/ros/humble/setup.bash

if [ -f install/setup.bash ]; then
    echo "Sourcing workspace overlay..."
    source install/setup.bash
else
    echo "WARNING: install/setup.bash not found. Did you run colcon build?"
    echo "Continuing with underlay only..."
fi

echo "Running hardware tests with sudo (preserving environment)..."
echo ""

# Pass PYTHONPATH, LD_LIBRARY_PATH, and the cancel-file killswitch path
# (set by test_runner_node.py so a cancel request can reach this
# root-owned subprocess despite sudo blocking a direct SIGTERM) explicitly.
run_with_sudo \
    PYTHONPATH="$PYTHONPATH" LD_LIBRARY_PATH="$LD_LIBRARY_PATH" \
    CHESS_HW_TEST_CANCEL_FILE="${CHESS_HW_TEST_CANCEL_FILE:-}" \
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
