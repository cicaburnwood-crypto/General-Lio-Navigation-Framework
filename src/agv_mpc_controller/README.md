# agv_mpc_controller

Nav2 local controller plugin using a sampling MPC strategy.

The package is split into two layers:

- `sampling_mpc`: ROS-free core logic. It rolls out candidate `vx/vy/omega`
  commands, scores predicted trajectories, and returns the first command of the
  best trajectory.
- `MpcController`: Nav2 `nav2_core::Controller` plugin. It adapts Nav2 path,
  pose, velocity, and costmap data to the core MPC API and returns `/cmd_vel`.

This split keeps the optimizer independent from Nav2 so later work can extend
the state and command vectors toward mobile manipulation or whole-body planning.

## Current State Model

```text
state   = [x, y, yaw]
command = [vx, vy, omega]
```

The rollout model assumes body-frame `vx/vy/omega` commands, matching the
`agv2_pkg` dual-steer driver input.

## Debug Output

When `debug_trajectory: true`, the plugin publishes the selected rollout path:

```text
/FollowPath/selected_trajectory
```

Visualize this path in RViz together with the local costmap to tune weights and
sampling ranges.
