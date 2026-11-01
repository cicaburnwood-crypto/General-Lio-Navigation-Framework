#!/usr/bin/env bash

# Source this file. It intentionally preserves the caller's nounset setting
# because ROS 2 Humble setup scripts may inspect unset AMENT variables.
case $- in
  *u*) _agv_restore_nounset=1 ;;
  *) _agv_restore_nounset=0 ;;
esac
set +u

_agv_repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
_agv_ws_dir="$(cd "$_agv_repo_dir/../.." && pwd)"

# Fail closed even in `source ... && ros2 ...`, where errexit is suppressed.
if [[ -r "$HOME/.config/hkclr/ros_domain.sh" ]] &&
   source /opt/ros/humble/setup.bash &&
   source "$_agv_ws_dir/install_release_humble/setup.bash" &&
   source "$HOME/.config/hkclr/ros_domain.sh" &&
   [[ "${ROS_DOMAIN_ID:-}" == "76" ]]; then
  :
else
  printf '[robot234] Environment/domain setup failed; robot launch is blocked.\n' >&2
  if [[ $_agv_restore_nounset -eq 1 ]]; then
    set -u
  fi
  unset _agv_restore_nounset _agv_repo_dir _agv_ws_dir
  return 1
fi

if [[ $_agv_restore_nounset -eq 1 ]]; then
  set -u
fi
unset _agv_restore_nounset _agv_repo_dir _agv_ws_dir
