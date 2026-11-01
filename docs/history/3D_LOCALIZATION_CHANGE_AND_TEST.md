# 3D VGICP Localization Change and Test

## 1. What Changed

This workspace now contains a 3D localization path under `chassis_nav_ws`:

```text
FAST-LIO /Odometry + /cloud_registered_body
  -> nav_interface_adapter
  -> /odom + /sensors/cloud + TF odom->base_link
  -> chassis_3d_localization
  -> TF map->odom + /localizer/localized_result
```

Added packages:

- `chassis_localization_msgs`
  - `chassis_localization_msgs/srv/Relocalize`
  - `chassis_localization_msgs/srv/IsValid`
- `scan_registration`
  - Shared ICP/GICP/NDT/VGICP registration library.
  - Uses migrated `fast_gicp` and `pclomp` sources.
- `chassis_3d_localization`
  - `chassis_3d_localizer_node`
  - Loads a prior PCD map through `/localizer/relocalize`.
  - Runs local submap crop + rough/refine VGICP scan-to-map.
  - Publishes `map -> odom` without modifying FAST-LIO state.

Added launch/config files:

- `src/nav_interface_adapter/config/adapter_fastlio_body.yaml`
- `src/chassis_3d_localization/config/vgicp_localizer.yaml`
- `src/bringup/launch/adapter_fastlio_body.launch.py`
- `src/bringup/launch/localization_3d.launch.py`
- `src/bringup/launch/nav_known_map.launch.py`

The AMCL path is unchanged. `navigation.launch.py` and `full_stack.launch.py` still use the existing 2D localization flow.
The 3D path uses FAST-LIO body-frame data through the FAST-LIO body adapter.
The adapter still reads the selected LiDAR profile to publish the
`base_link -> fastlio_body` extrinsic, but it must not be routed through the raw
Livox CustomMsg adapter.

## 2. Build

Minimal build for the new 3D localization packages plus the updated launch/config packages:

```bash
cd ~/chassis_nav_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  chassis_localization_msgs scan_registration chassis_3d_localization \
  nav_interface_adapter bringup
source install/setup.bash
```

Full workspace build on the vehicle:

```bash
cd ~/chassis_nav_ws
source /opt/ros/humble/setup.bash

# Only needed if FAST-LIO/livox_ros_driver2 are not already present.
vcs import src < chassis_nav.repos

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 3. Runtime Test

Start the hardware driver and FAST-LIO first:

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

Start the 3D localization stack:

```bash
cd ~/chassis_nav_ws
source install/setup.bash
ros2 launch bringup localization_3d.launch.py
```

Or start the full known-map stack in 3D mode:

```bash
ros2 launch bringup nav_known_map.launch.py localization:=3d
```

Initialize localization with a known rough pose:

```bash
ros2 service call /localizer/relocalize chassis_localization_msgs/srv/Relocalize \
"{pcd_path: '/home/hkclr/maps/market_test/map.pcd', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"
```

The rough pose is interpreted as `T_map_base_link`. For the first test, keep the rough pose within about `1 m` and `10 deg` of the real pose.

## 4. Checks

Topic checks:

```bash
ros2 topic hz /Odometry
ros2 topic hz /cloud_registered_body
ros2 topic hz /odom
ros2 topic hz /sensors/cloud
ros2 topic hz /localizer/localized_result
```

TF checks:

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo base_link fastlio_body
```

RViz checks:

- Fixed Frame: `map`
- Add `/localizer/map_cloud`
- Add `/sensors/cloud`
- Add `/localizer/localized_result`
- The current scan should align with the local part of the PCD map after relocalization.

Expected behavior:

- `/odom` is continuous and follows FAST-LIO.
- `map->odom` changes slowly as VGICP corrects drift.
- `/localizer/localized_result` is stable in `map`.
- No large pose jump during slow vehicle motion.

## 5. Common Failures

No `/odom`:

- Check FAST-LIO `/Odometry`.
- Check `nav_interface_adapter` is running through `bringup adapter_fastlio_body.launch.py`
  with `adapter_fastlio_body.yaml`.
- Confirm the adapter is not launched through `adapter_raw_livox.launch.py`.

No `/sensors/cloud`:

- Check FAST-LIO `/cloud_registered_body`.
- Confirm `cloud_input_type: pointcloud2` in `adapter_fastlio_body.yaml`.

`/localizer/relocalize` returns `pcd file not found`:

- Use an absolute path on the vehicle.
- Confirm the PCD exists and is readable by the current user.

VGICP reports submap too small:

- Initial pose may be outside the map.
- Increase `submap_radius_xy`.
- Check map frame and initial pose convention.

VGICP score too high or does not converge:

- Initial pose may be outside the convergence basin.
- Try a better rough pose.
- Increase rough correspondence distance temporarily.
- Check that `/cloud_registered_body` is in the expected body frame, and that
  `base_link -> fastlio_body` matches the selected LiDAR profile.

Pose jumps are rejected:

- Current `max_translation_jump` is `0.15 m`.
- Current `max_yaw_jump_deg` is `3 deg`.
- For debugging only, loosen these gates and restore them after confirming map alignment.

## 6. Parameters to Tune First

File:

```text
src/chassis_3d_localization/config/vgicp_localizer.yaml
```

Recommended first tuning knobs:

- `submap_radius_xy`: increase if initial pose is less accurate.
- `rough_max_correspondence_distance`: increase for rougher initial poses.
- `refine_max_correspondence_distance`: decrease after stable convergence.
- `rough_score_thresh` / `refine_score_thresh`: loosen for first bring-up, tighten after validation.
- `update_hz`: start at `0.5`; increase only after CPU load is acceptable.
- `num_threads`: keep bounded so VGICP does not starve FAST-LIO or Nav2 MPPI.
