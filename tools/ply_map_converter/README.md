# ply_map_converter

Offline converter from a PLY point cloud map to the files used by the chassis navigation stack:

```text
src/bringup/maps/<map_name>/3D/map.pcd
src/bringup/maps/<map_name>/2D/<map_name>_2d.pgm
src/bringup/maps/<map_name>/2D/<map_name>_2d.yaml
```

Build:

```bash
cd chassis_nav_ws
colcon build --packages-select ply_map_converter
source install/setup.bash
```

Convert:

```bash
ros2 run ply_map_converter ply_map_converter \
  --input /path/to/map.ply \
  --map-name meeting_room \
  --overwrite
```

Use the generated map:

```bash
./nav_3d_in_map.sh \
  src/bringup/maps/meeting_room/3D/map.pcd \
  src/bringup/maps/meeting_room/2D/meeting_room_2d.yaml
```

Defaults assume the PLY is in meter-scale map coordinates, with `x/y` horizontal and `z` upward. The 2D occupancy projection marks obstacles from `0.15m` to `1.70m` high, matching the chassis plus dual-arm robot height envelope.

## Head-camera-centered point clouds

Some prebuilt point clouds are centered on a wall-mounted head camera rather
than a floor-based map frame. Do not pass those files directly to
`ply_map_converter`: camera-forward depth would be treated as map height and
the 2D occupancy map would be wrong.

First align the fused centered cloud to a floor frame:

```bash
python3 tools/ply_map_converter/scripts/align_centered_ply_to_floor.py \
  --input /home/hkclr/maps/meetint_room2/hkclr_meetingroom/hkclr/other_views_fused_centered_pointcloud.ply \
  --output build/meeting_room2_aligned.ply
```

Then convert the aligned PLY:

```bash
ros2 run ply_map_converter ply_map_converter \
  --input build/meeting_room2_aligned.ply \
  --map-name meeting_room2 \
  --overwrite
```

For the HKCLR meeting-room data, the fitted floor plane should put the camera
about `2.0m` above the floor and produce aligned bounds close to
`x[-0.25, 4.92] y[-3.00, 3.12] z[-0.05, 2.71]`.
