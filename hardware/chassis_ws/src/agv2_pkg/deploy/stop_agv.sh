#!/usr/bin/env bash
set -euo pipefail

if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
  echo "Run with sudo so can0 can be placed DOWN after shutdown." >&2
  exit 2
fi

interface="${CAN_INTERFACE:-can0}"
session_name=agv_control
real_user="${SUDO_USER:-$(id -un)}"
real_home="$(getent passwd "$real_user" | cut -d: -f6)"
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
setup_file="$repo_dir/deploy/setup_agv_env.bash"
setup_q="$(printf '%q' "$setup_file")"
run_as_user=(sudo -u "$real_user" env HOME="$real_home")
ws_dir="$(cd "$repo_dir/../.." && pwd)"
logger_cli="$repo_dir/scripts/chassis_fault_log.py"
logger_root="$ws_dir/data/chassis_faults/continuous"
logctl() {
  local budget=0.5s
  [[ "$1" == stop ]] && budget=10s
  "${run_as_user[@]}" timeout "$budget" /usr/bin/python3 -B "$logger_cli" "$@" --root "$logger_root"
}

echo "[robot234] host=$(hostname) ROS_DOMAIN_ID=76 config=$real_home/.config/hkclr/ros_domain.sh"
if ! "${run_as_user[@]}" /bin/bash -c "source $setup_q && [[ \${ROS_DOMAIN_ID:-} == 76 ]]"; then
  echo "ROS environment preflight failed; no shutdown service or CAN changes made." >&2
  exit 1
fi
echo "The physical E-stop must be PRESSED before shutdown."
if "${run_as_user[@]}" /usr/bin/tmux has-session -t "$session_name" 2>/dev/null; then
  session_domain="$("${run_as_user[@]}" /usr/bin/tmux show-option -qv -t "$session_name" @hkclr_ros_domain_id)"
  if [[ "$session_domain" != "76" ]]; then
    echo "Existing session domain is '${session_domain:-unknown}', not verified 76. Use its original-domain shutdown procedure; no service/CAN changes made." >&2
    exit 1
  fi
  logctl pause --reason shutdown_begin || echo "Logger unavailable; hardware shutdown continues." >&2
  if ! "${run_as_user[@]}" timeout 15s bash -lc \
    "source $setup_q && ros2 service call /agv2/prepare_shutdown std_srvs/srv/Trigger '{}'"; then
    echo "Graceful shutdown handshake failed; tmux and CAN were left running." >&2
    logctl event --reason hardware_shutdown_failed_passive_recording_continues || true
    exit 1
  fi
  "${run_as_user[@]}" /usr/bin/tmux kill-session -t "$session_name"
fi

logctl pause --reason hardware_stopped || true
if ! ip link set "$interface" down; then
  logctl event --reason can_shutdown_failed_passive_recording_continues || true
  exit 1
fi
ip -details link show "$interface"
echo "Shutdown complete. Confirm state DOWN and CAN state STOPPED above."
logctl event --reason hardware_shutdown_success || true
logctl stop || echo "Logger close incomplete; hardware shutdown has already completed." >&2
