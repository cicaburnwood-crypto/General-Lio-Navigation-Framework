# 2D/3D 定位跳变对比测试

这个测试用于判断导航不顺畅是否由 3D 定位跳变导致。方法是分别启动 2D AMCL 和 3D VGICP 定位链路，在相同 `/cmd_vel` 输入下让机器人低速移动同一段时间，然后对比 TF 和定位话题输出。

## 1. 准备

所有终端先 source：

```bash
cd ~/code/chassis_nav_ws
source install/setup.bash
```

绘图依赖 `matplotlib`：

```bash
sudo apt install python3-matplotlib
```

安全要求：

- 第一次测试建议低速、短时间、空旷区域。
- 测试期间不要发送 Nav2 goal，避免 Nav2 controller 同时发布 `/cmd_vel`。
- 2D AMCL 和 3D localizer 不要同时启动，因为它们都会发布 `map->odom`。
- `record_trial.py` 默认只记录，不会让车动；必须加 `--drive` 才会发布非零 `/cmd_vel`。
- Ctrl+C 或脚本异常退出时，脚本会连续发布 0 速度停车。

## 2. 2D AMCL 测试

启动底盘：

```bash
cd ~/code/agv_ws
sudo ./start_agv.sh
```

启动 2D 定位和导航链路：

```bash
cd ~/code/chassis_nav_ws
./nav_in_map.sh
```

在 RViz 中使用 `2D Pose Estimate` 给 AMCL 初始位姿。确认：

```bash
ros2 topic hz /amcl_pose
ros2 run tf2_ros tf2_echo map base_link
```

开始采集。下面命令会以 `0.10 m/s` 前进 `5 s`，理论位移约 `0.5 m`：

```bash
cd ~/code/chassis_nav_ws
python3 tools/localization_compare/record_trial.py \
  --label 2d \
  --drive \
  --linear-x 0.10 \
  --drive-duration 5.0
```

脚本结束后会打印输出目录，例如：

```text
trial output: /home/hkclr/loc_compare/20260529_153000_2d
```

记录这个目录，后面绘图要用。

## 3. 3D VGICP 测试

停止 2D 测试相关窗口，确保 AMCL 不再发布 `map->odom`。然后启动 3D 定位链路：

```bash
cd ~/code/chassis_nav_ws
./nav_3d_in_map.sh
```

在 RViz 中同样使用 `2D Pose Estimate` 给 3D localizer 初始位姿。确认：

```bash
ros2 topic hz /localizer/localized_result
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo map odom
```

用同样速度和时长采集：

```bash
cd ~/code/chassis_nav_ws
python3 tools/localization_compare/record_trial.py \
  --label 3d \
  --drive \
  --linear-x 0.10 \
  --drive-duration 5.0
```

脚本结束后会打印输出目录，例如：

```text
trial output: /home/hkclr/loc_compare/20260529_154000_3d
```

## 4. 生成对比图

把下面两个目录替换成你实际打印出来的目录：

```bash
cd ~/code/chassis_nav_ws
python3 tools/localization_compare/plot_compare.py \
  --run-2d ~/loc_compare/20260529_153000_2d \
  --run-3d ~/loc_compare/20260529_154000_3d \
  --out ~/loc_compare/localization_compare.png
```

输出文件：

```text
~/loc_compare/localization_compare.png
~/loc_compare/compare_summary.csv
~/loc_compare/compare_summary.json
```

默认跳变阈值：

```bash
--jump-m 0.15
--jump-yaw-deg 5.0
```

如果你的定位频率较低，或者机器人速度更高，可以适当放宽阈值：

```bash
python3 tools/localization_compare/plot_compare.py \
  --run-2d ~/loc_compare/<2d_run_dir> \
  --run-3d ~/loc_compare/<3d_run_dir> \
  --jump-m 0.25 \
  --jump-yaw-deg 8.0 \
  --out ~/loc_compare/localization_compare.png
```

## 5. 输出内容

每次采集目录包含：

```text
tf_samples.csv
raw_pose_samples.csv
cmd_vel_samples.csv
summary.json
```

`tf_samples.csv` 记录：

- `map -> base_link`：Nav2 最终看到的机器人全局位姿。
- `map -> odom`：定位模块对里程计系的全局修正。
- `odom -> base_link`：FAST-LIO/adapter 提供的连续里程计位姿。

`raw_pose_samples.csv` 记录：

- `/amcl_pose`：2D AMCL 原始定位结果。
- `/localizer/localized_result`：3D VGICP localizer 原始定位结果。
- `/odom`：adapter 输出的里程计。

`cmd_vel_samples.csv` 记录脚本实际发布的速度指令。

## 6. 如何判断问题来源

优先看 `localization_compare.png`：

- 如果 3D 的 `map->base_link translation step` 或 `yaw step` 有明显尖峰，而 2D 没有，说明 Nav2 接收到的 3D 定位位姿存在跳变。
- 如果 3D 的 `map->odom correction` 有突然变化，同时 `odom->base_link` 平滑，说明问题主要在 3D scan-to-map 纠偏。
- 如果 3D 的 `/localizer/localized_result` 原始话题也跳，说明 VGICP 匹配结果本身不稳定。
- 如果 `odom->base_link` 也跳，说明问题更可能在 FAST-LIO、adapter 或传感器输入，而不是 VGICP 纠偏。
- 如果 2D 和 3D 定位曲线都平滑，但 3D 导航动作仍然不自然，下一步应重点检查 Nav2 controller、costmap、footprint、速度限制和 MPPI 参数。

也可以直接看 `compare_summary.csv`：

- `max_translation_step_m`
- `translation_jump_count`
- `max_yaw_step_deg`
- `yaw_jump_count`

正常情况下，低速直线测试时这些值应该比较小，并且 3D 不应明显大于 2D。

## 7. 常用变体

只记录不动车，用于观察静止定位抖动：

```bash
python3 tools/localization_compare/record_trial.py --label 3d_static
```

延长直线距离：

```bash
python3 tools/localization_compare/record_trial.py \
  --label 3d \
  --drive \
  --linear-x 0.10 \
  --drive-duration 10.0
```

原地旋转测试 yaw 稳定性：

```bash
python3 tools/localization_compare/record_trial.py \
  --label 3d_turn \
  --drive \
  --linear-x 0.0 \
  --angular-z 0.20 \
  --drive-duration 5.0
```

指定输出目录根路径：

```bash
python3 tools/localization_compare/record_trial.py \
  --label 2d \
  --drive \
  --output-root ~/loc_compare/office_test
```
