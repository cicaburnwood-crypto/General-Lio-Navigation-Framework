#!/usr/bin/env bash
set -euo pipefail
AL_ROOT="$(cd "$(dirname "$0")" && pwd)"
test -f "$AL_ROOT/install/local_setup.bash"
source "$AL_ROOT/integration/robot.env"
if tmux has-session -t '=AL-dual' 2>/dev/null; then
  echo 'AL already running. Inspect with: tmux attach -t AL-dual'
  exit 0
fi
mkdir -p "$AL_ROOT/logs/ros" "$AL_ROOT/.ros"
enable_lidar=true
if ! ip -o -4 addr show | awk '{print $4}' | grep -Eq '^192\.168\.1\.50/'; then
  enable_lidar=false
fi
for component in stack web; do
  printf -v command 'exec env -i HOME=%q USER=%q PATH=/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 AL_ROS_DOMAIN_ID=%q AL_ROS_LOCALHOST_ONLY=%q AL_ENABLE_LIDAR=%q bash %q %q >>%q 2>&1' \
    "$HOME" "$USER" "${AL_ROS_DOMAIN_ID:-76}" "${AL_ROS_LOCALHOST_ONLY:-1}" "$enable_lidar" \
    "$AL_ROOT/integration/component.sh" "$component" "$AL_ROOT/logs/$component.log"
  if [[ "$component" == stack ]]; then
    tmux new-session -d -s AL-dual -n stack "$command"
  else
    tmux new-window -d -t AL-dual -n "$component" "$command"
  fi
done
echo "AL started: http://$AL_WEB_HOST:8092/#patrol (lidar=$enable_lidar)"
