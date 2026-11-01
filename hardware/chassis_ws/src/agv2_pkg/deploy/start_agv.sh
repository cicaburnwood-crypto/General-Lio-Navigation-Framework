#!/usr/bin/env bash
set -euo pipefail

if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
  echo "Run with sudo because configuring SocketCAN requires root." >&2
  exit 2
fi
if [[ $# -ne 1 ]]; then
  echo "usage: sudo $0 /absolute/path/to/chassis.yaml" >&2
  exit 2
fi

interface="${CAN_INTERFACE:-can0}"
bitrate=1000000
txqueuelen=4096
session_name=agv_control
real_user="${SUDO_USER:-$(id -un)}"
real_home="$(getent passwd "$real_user" | cut -d: -f6)"
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ws_dir="$(cd "$repo_dir/../.." && pwd)"
setup_file="$repo_dir/deploy/setup_agv_env.bash"
params_file="$(realpath "$1")"

if [[ ! -r "$params_file" ]]; then
  echo "Unreadable parameter file: $params_file" >&2
  exit 2
fi
if [[ ! -r "$ws_dir/install_release_humble/setup.bash" ]]; then
  echo "Workspace is not built. Run: $repo_dir/deploy/build_humble.sh" >&2
  exit 2
fi
if [[ ! -x /usr/bin/tmux ]]; then
  echo "Missing /usr/bin/tmux; run deploy/install_dependencies_ubuntu22_humble.sh" >&2
  exit 2
fi

run_as_user=(sudo -u "$real_user" env HOME="$real_home")
setup_q="$(printf '%q' "$setup_file")"
domain_file="$real_home/.config/hkclr/ros_domain.sh"
echo "[robot234] host=$(hostname) ROS_DOMAIN_ID=76 config=$domain_file"
if ! "${run_as_user[@]}" /bin/bash -c "source $setup_q && [[ \${ROS_DOMAIN_ID:-} == 76 ]]"; then
  echo "ROS environment preflight failed; CAN and robot processes were not started." >&2
  exit 1
fi
if "${run_as_user[@]}" /usr/bin/tmux has-session -t "$session_name" 2>/dev/null; then
  echo "Session '$session_name' already exists. Safely stop the existing stack first; no CAN changes made." >&2
  exit 1
fi

logger_cli="$repo_dir/scripts/chassis_fault_log.py"
logger_root="$ws_dir/data/chassis_faults/continuous"
logctl() { "${run_as_user[@]}" timeout 10s /usr/bin/python3 -B "$logger_cli" "$@" --root "$logger_root"; }
logger_started=0
stack_requested=0
logger_start_exit() {
  local rc=$?
  if [[ $rc -ne 0 && $logger_started -eq 1 ]]; then
    logctl event --reason "start_failed_exit_$rc" || true
    logctl pause --reason startup_failed || true
    # Never tear down partially started hardware from the logging wrapper.
    if [[ $stack_requested -eq 0 ]]; then logctl stop || true; fi
  fi
}
trap logger_start_exit EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP
/usr/bin/python3 -B "$repo_dir/deploy/fault_logger_start.py" --user "$real_user" --params "$params_file" --interface "$interface"
logger_started=1
logctl event --reason can_initialization_begin
echo "Keep the physical E-stop PRESSED while the stack starts."
while [[ ! -d "/sys/class/net/$interface" ]]; do
  sleep 1
done
sleep 3

ip link set "$interface" down
ip link set "$interface" type can bitrate "$bitrate"
ip link set "$interface" txqueuelen "$txqueuelen"
ip link set "$interface" up
logctl event --reason can_initialization_success
logctl wait --field can_bound --timeout 10
logctl activate
stack_requested=1

"${run_as_user[@]}" /usr/bin/tmux new-session -d -s "$session_name" /bin/bash
"${run_as_user[@]}" /usr/bin/tmux set-option -t "$session_name" @hkclr_ros_domain_id 76
"${run_as_user[@]}" /usr/bin/tmux set-environment -t "$session_name" ROS_DOMAIN_ID 76

params_q="$(printf '%q' "$params_file")"
interface_q="$(printf '%q' "$interface")"
init_cmd="source $setup_q"

"${run_as_user[@]}" /usr/bin/tmux send-keys -t "$session_name" \
  "$init_cmd && ros2 launch ros2_socketcan socket_can_receiver.launch.py interface:=$interface_q" C-m

"${run_as_user[@]}" /usr/bin/tmux split-window -v -t "$session_name" /bin/bash
"${run_as_user[@]}" /usr/bin/tmux send-keys -t "$session_name" \
  "$init_cmd && ros2 launch ros2_socketcan socket_can_sender.launch.py interface:=$interface_q enable_frame_loopback:=false" C-m

"${run_as_user[@]}" /usr/bin/tmux select-pane -t 0
"${run_as_user[@]}" /usr/bin/tmux split-window -h -t "$session_name" /bin/bash
"${run_as_user[@]}" /usr/bin/tmux send-keys -t "$session_name" \
  "$init_cmd && ros2 run joy joy_node" C-m

"${run_as_user[@]}" /usr/bin/tmux select-pane -t 2
"${run_as_user[@]}" /usr/bin/tmux split-window -h -t "$session_name" /bin/bash
"${run_as_user[@]}" /usr/bin/tmux send-keys -t "$session_name" \
  "$init_cmd && ros2 launch agv2_pkg agv2_control.launch.py params_file:=$params_q enable_fault_logging:=false" C-m

"${run_as_user[@]}" /usr/bin/tmux select-layout -t "$session_name" tiled

echo "AGV stack launch requested in ROS_DOMAIN_ID=76 with explicit parameters: $params_file"
echo "View: tmux attach -t $session_name"
logctl event --reason stack_launch_requested
logctl status || true
echo "Stop unchanged: sudo ./deploy/stop_agv.sh"
