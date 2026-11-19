#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"
mkdir -p src/FAST_LIO/Log src/FAST_LIO/PCD
source /opt/ros/humble/setup.bash
if [[ -d "$root/.deps/livox/lib" ]]; then
  export LD_LIBRARY_PATH="$root/.deps/livox/lib:${LD_LIBRARY_PATH:-}"
  export CMAKE_PREFIX_PATH="$root/.deps/livox:${CMAKE_PREFIX_PATH:-}"
fi
if ! /usr/bin/python3 -c 'import aiohttp,PIL,yaml' 2>/dev/null; then
  if compgen -G 'wheels/aiohttp-*.whl' >/dev/null; then
    /usr/bin/python3 -m pip install --no-index --find-links wheels \
      --target .venv/lib/python3.10/site-packages aiohttp Pillow
  else
    printf '%s\n' 'Install python3-aiohttp, python3-pil and python3-yaml from the Ubuntu packages, or supply offline wheels in wheels/.' >&2
    exit 1
  fi
fi
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
export PYTHONNOUSERSITE=1
# Humble systems can have setuptools versions that reject colcon's editable install.
# Install the pure Python manager normally; retain symbolic links for the C++ workspace.
/usr/bin/colcon build --base-paths "$root/src" --packages-select patrol_manager
boost_config=(/usr/lib/x86_64-linux-gnu/cmake/Boost-*/BoostConfig.cmake)
boost_args=()
if [[ -f "${boost_config[0]}" ]]; then
  boost_args+=("-DBoost_DIR=$(dirname "${boost_config[0]}")")
fi
/usr/bin/colcon build --base-paths "$root/src" --symlink-install --parallel-workers 3 --cmake-clean-cache \
  --packages-up-to bringup nav2_mppi_controller \
  --cmake-args -DDISTRO_ROS=humble -DROS_EDITION=ROS2 -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release "${boost_args[@]}"
