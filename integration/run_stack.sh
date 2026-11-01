#!/usr/bin/env bash
set -euo pipefail
TASK_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# Separate from the running scan-nav domain 61. Use the SAME domain for chassis,
# camera and this stack when actually deploying; do not connect both stacks.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-76}"
set +u
source /opt/ros/humble/setup.bash
source "$TASK_ROOT/install/setup.bash"
set -u
exec ros2 launch "$TASK_ROOT/integration/door_stack.launch.py" "$@"
