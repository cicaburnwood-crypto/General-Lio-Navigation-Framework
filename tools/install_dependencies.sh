#!/usr/bin/env bash
# Dependency setup only; never starts ROS nodes or a chassis driver.
set -eo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
test -f /opt/ros/humble/setup.bash || { echo 'Install ROS 2 Humble on Ubuntu 22.04 first.' >&2; exit 1; }
sudo apt-get update
sudo apt-get install -y build-essential cmake git tmux python3-venv python3-colcon-common-extensions \
  python3-rosdep python3-opencv python3-aiohttp python3-yaml python3-pytest python3-pil \
  libeigen3-dev libpcl-dev libasio-dev ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-spatio-temporal-voxel-layer ros-humble-tf2-ros-py \
  ros-humble-tf2-sensor-msgs ros-humble-pcl-ros ros-humble-pcl-conversions
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then sudo rosdep init; fi
rosdep update
source /opt/ros/humble/setup.bash
rosdep install --from-paths "$root/src" --ignore-src --rosdistro humble -y
/usr/bin/python3 -m venv --system-site-packages "$root/.venv"
"$root/.venv/bin/python" -m pip install -r "$root/requirements-runtime.txt"
# External SDK revision is pinned for source reproducibility, not claimed to be
# the unknown revision of the previously installed onboard shared library.
sdk="$root/.dependencies/Livox-SDK2"
mkdir -p "$root/.dependencies"
if [[ ! -d "$sdk/.git" ]]; then git clone https://github.com/Livox-SDK/Livox-SDK2.git "$sdk"; fi
git -C "$sdk" checkout --detach c0796f04c143143899c87a773d9f6b7136453c0b
cmake -S "$sdk" -B "$sdk/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$sdk/build" -j "${BUILD_JOBS:-2}"
sudo cmake --install "$sdk/build"
sudo ldconfig
