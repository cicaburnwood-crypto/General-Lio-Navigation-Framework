#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"
python3 tools/fetch_assets.py
source /opt/ros/humble/setup.bash
# Livox ROS2 package.xml and launch/ are already materialized in this snapshot.
# Do not call upstream build.sh: it deletes the workspace build/install trees.
colcon build --base-paths src --symlink-install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble
