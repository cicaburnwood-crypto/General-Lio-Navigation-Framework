#!/usr/bin/env bash
set -euo pipefail
TASK_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-76}"
set +u
source /opt/ros/humble/setup.bash
source "$TASK_ROOT/install/setup.bash"
set -u
exec python3 "$TASK_ROOT/integration/web_server.py" --ros "$@"
