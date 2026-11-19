#!/usr/bin/env bash
set -euo pipefail
# This exact session belongs to AL only. Never stop nav_3d_in_map or agv_control.
if tmux has-session -t '=AL-dual' 2>/dev/null; then
  for pane in $(tmux list-panes -a -F '#{session_name} #{pane_id}' | awk '$1=="AL-dual" {print $2}'); do
    tmux send-keys -t "$pane" C-c
  done
  for attempt in {1..10}; do
    tmux has-session -t '=AL-dual' 2>/dev/null || exit 0
    sleep 1
  done
  # Close only the remaining PTYs in this AL session.
  tmux kill-session -t '=AL-dual'
fi
