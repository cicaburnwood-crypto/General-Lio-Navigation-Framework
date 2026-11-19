#!/usr/bin/env bash
set -eo pipefail
AL_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$AL_ROOT"
source "$AL_ROOT/integration/robot.env"
export ROS_DOMAIN_ID="${AL_ROS_DOMAIN_ID:-76}"
export ROS_LOCALHOST_ONLY="${AL_ROS_LOCALHOST_ONLY:-1}"
export ROS_HOME="$AL_ROOT/.ros"
export ROS_LOG_DIR="$AL_ROOT/logs/ros"
export PYTHONDONTWRITEBYTECODE=1
source /opt/ros/humble/setup.bash
source "$AL_ROOT/install/local_setup.bash"
export PYTHONPATH="$AL_ROOT/.venv/lib/python3.10/site-packages:${PYTHONPATH:-}"
if [[ -d "$AL_ROOT/.deps/livox/lib" ]]; then
  export LD_LIBRARY_PATH="$AL_ROOT/.deps/livox/lib:${LD_LIBRARY_PATH:-}"
fi
robot_config="$AL_ROOT/integration/robots/$AL_LIDAR_PROFILE"
case "${1:-}" in
  stack) exec ros2 launch "$AL_ROOT/integration/door_stack.launch.py" \
    "lidar:=$AL_LIDAR_PROFILE" "profile_file:=$robot_config/lidar_profiles.yaml" \
    "nav_params:=$robot_config/nav2_mppi.yaml" "enable_lidar:=${AL_ENABLE_LIDAR:-true}" "${@:2}" ;;
  web) exec /usr/bin/python3 "$AL_ROOT/integration/web_server.py" --ros \
    --host "$AL_WEB_HOST" --port 8092 --nav-params "$robot_config/nav2_mppi.yaml" "${@:2}" ;;
  manager) exec ros2 run patrol_manager door_patrol_manager --ros-args \
    -p mission_file:="$AL_ROOT/src/patrol_manager/config/door_patrol.yaml" ;;
  *) echo 'Use stack, web or manager' >&2; exit 2 ;;
esac
