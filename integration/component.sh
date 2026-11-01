#!/usr/bin/env bash
set -eo pipefail
AL_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$AL_ROOT"
export ROS_DOMAIN_ID=76
export ROS_HOME="$AL_ROOT/.ros"
export ROS_LOG_DIR="$AL_ROOT/logs/ros"
export PYTHONDONTWRITEBYTECODE=1
source /opt/ros/humble/setup.bash
source "$AL_ROOT/install/setup.bash"
export PYTHONPATH="$AL_ROOT/.venv/lib/python3.10/site-packages:${PYTHONPATH:-}"
case "${1:-}" in
  stack) exec ros2 launch "$AL_ROOT/integration/door_stack.launch.py" ;;
  camera) exec /usr/bin/python3 "$AL_ROOT/integration/d435i_rgb.py" ;;
  web) exec /usr/bin/python3 "$AL_ROOT/integration/web_server.py" --ros --host 0.0.0.0 --port 8092 ;;
  manager) exec ros2 run patrol_manager door_patrol_manager --ros-args -p mission_file:="$AL_ROOT/src/patrol_manager/config/door_patrol.yaml" ;;
  *) echo 'Use stack, camera, web or manager (manager-only hot replacement)' >&2; exit 2 ;;
esac
