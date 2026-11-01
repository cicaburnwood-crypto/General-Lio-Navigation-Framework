#!/usr/bin/env bash
set -euo pipefail
AL_ROOT="$(cd "$(dirname "$0")" && pwd)"
test -f "$AL_ROOT/install/setup.bash"
if tmux has-session -t '=AL' 2>/dev/null; then
  echo 'AL already running. Inspect with: tmux attach -t AL'
  exit 0
fi
mkdir -p "$AL_ROOT/logs/ros" "$AL_ROOT/.ros"
for component in stack camera web; do
  printf -v command 'exec env -i HOME=%q USER=%q PATH=/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 bash %q %q >>%q 2>&1' \
    "$HOME" "$USER" "$AL_ROOT/integration/component.sh" "$component" "$AL_ROOT/logs/$component.log"
  if [[ "$component" == stack ]]; then
    tmux new-session -d -s AL -n stack "$command"
  else
    tmux new-window -d -t AL -n "$component" "$command"
  fi
done
echo 'AL started: http://10.30.3.234:8092/#patrol'
echo 'No patrol started. BL was not sourced, started, stopped or changed.'
