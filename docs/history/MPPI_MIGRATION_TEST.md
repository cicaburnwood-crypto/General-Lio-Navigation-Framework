# MPPI Controller Migration and Test Notes

This workspace now keeps the custom `agv_mpc_controller` as a fallback, but the default known-map navigation controller is the source-overlay Nav2 MPPI controller.

## What Changed

- Added source package: `src/nav2_mppi_controller`
  - Source baseline: official Navigation2 `humble` branch, `nav2_mppi_controller`
  - License: MIT, kept in `src/nav2_mppi_controller/LICENSE.md`
- Changed default Nav2 params in `src/navigation/launch/nav2.launch.py`
  - Old default: `src/navigation/config/nav2.yaml`
  - New default: `src/navigation/config/nav2_mppi.yaml`
- Added MPPI params:
  - `src/navigation/config/nav2_mppi.yaml`
  - `src/navigation/config/nav2_mppi_loose_yaw.yaml`
- Kept fallback custom MPC params:
  - `src/navigation/config/nav2_mpc.yaml`
  - `src/agv_mpc_controller`

## Controller Modes

Default startup now uses MPPI:

```bash
ros2 launch navigation nav2.launch.py
```

Expected controller plugin:

```bash
ros2 param get /controller_server FollowPath.plugin
```

Expected output:

```text
String value is: nav2_mppi_controller::MPPIController
```

Strict yaw MPPI config:

```bash
src/navigation/config/nav2_mppi.yaml
```

Loose yaw MPPI config:

```bash
ros2 launch navigation nav2.launch.py \
  params_file:=$(ros2 pkg prefix navigation)/share/navigation/config/nav2_mppi_loose_yaw.yaml
```

Fallback to the custom MPC:

```bash
ros2 launch navigation nav2.launch.py \
  params_file:=$(ros2 pkg prefix navigation)/share/navigation/config/nav2_mpc.yaml
```

## Build on the Robot

After pulling these changes on the robot:

```bash
cd ~/code/chassis_nav_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select nav2_mppi_controller navigation agv_mpc_controller
source install/setup.bash
```

If dependency resolution fails, install the likely MPPI build dependencies:

```bash
sudo apt install ros-humble-xtensor ros-humble-xsimd libbenchmark-dev
```

Then rebuild.

## Runtime Test Flow

Start the known-map navigation stack as usual:

```bash
cd ~/code/chassis_nav_ws
./nav_in_map.sh
```

Confirm lifecycle state:

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /velocity_smoother
```

All should be `active`.

Confirm the controller:

```bash
ros2 param get /controller_server FollowPath.plugin
```

Confirm velocity chain:

```bash
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel
```

Confirm MPPI visualization topics:

```bash
ros2 topic list | grep -E "trajectories|transformed_global_plan"
```

Typical MPPI topics:

```text
/trajectories
/transformed_global_plan
```

## Field Scenarios

Run these in RViz after AMCL has converged:

1. Straight target
   - Expected: smooth forward motion, no repeated stop-start behavior.
2. Corner target
   - Expected: path should keep more clearance than the earlier custom MPC baseline.
3. Narrow passage
   - Expected: no aggressive wall hugging; watch local costmap and `/trajectories`.
4. Strict final yaw target
   - Use `nav2_mppi.yaml`.
   - Expected: final heading should approach RViz goal yaw.
5. Loose final yaw target
   - Use `nav2_mppi_loose_yaw.yaml`.
   - Expected: robot should prioritize final position and avoid unnecessary final rotation.

## Useful Debug Commands

Check selected global path and final orientation:

```bash
ros2 topic echo /plan --once
ros2 topic echo /goal_pose --once
```

Check TF:

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo odom base_link
```

Check costmaps:

```bash
ros2 topic hz /local_costmap/costmap
ros2 topic hz /global_costmap/costmap
```

Check local planner output:

```bash
ros2 topic echo /cmd_vel_nav
```

Check chassis command:

```bash
ros2 topic echo /cmd_vel
```

If `/cmd_vel_nav` is nonzero but `/cmd_vel` is zero, inspect `velocity_smoother`.
If `/cmd_vel` is nonzero but the robot does not move, inspect the chassis driver mode, joystick Auto/AD enable, joy heartbeat, and CAN bridge.

## Tuning Notes

Main MPPI parameters are in `controller_server.ros__parameters.FollowPath`:

- Speed limits: `vx_max`, `vx_min`, `vy_max`, `wz_max`
- Sampling noise: `vx_std`, `vy_std`, `wz_std`
- Compute load: `batch_size`, `time_steps`, `model_dt`
- Obstacle behavior: `CostCritic.cost_weight`, `CostCritic.critical_cost`
- Final yaw behavior: `GoalAngleCritic.cost_weight`, `GoalAngleCritic.threshold_to_consider`
- Path tracking behavior: `PathAlignCritic.cost_weight`, `PathFollowCritic.cost_weight`, `PathAngleCritic.cost_weight`

Costmap clearance is mainly controlled by:

- `global_costmap.global_costmap.ros__parameters.inflation_layer.inflation_radius`
- `global_costmap.global_costmap.ros__parameters.inflation_layer.cost_scaling_factor`
- `local_costmap.local_costmap.ros__parameters.inflation_layer.inflation_radius`
- The shared rectangular `footprint`

The MPPI `CostCritic.consider_footprint` is enabled. Keep the footprint configured in both local and global costmaps; using only `robot_radius` with `consider_footprint: true` can make the MPPI controller fail during startup.

