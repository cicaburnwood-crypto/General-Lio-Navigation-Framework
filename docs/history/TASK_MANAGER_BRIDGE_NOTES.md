# 3D 导航栈近期改动说明（Task Manager 对接 / 一键启动）

覆盖提交范围：`439826e`（2026-06-29，Add task navigation bridge）～ `24ab089`（2026-07-20，change param），即最近一批围绕「接入 Task Manager」与「3D 定位收敛/平滑/分数门」的改动。给接手/协作的同学做改动说明和上手参考。

## 1. 这批改动做了什么

三条主线：

1. **接入 Task Manager**：新增 `task_navigation_bridge_node`（接收任务点、校验/投影、转发给 Nav2）和 `arrival_monitor_node`（监听 Nav2 执行结果、回executor `/navigation/arrived`），让另一台机器上的 task manager 能直接指挥这台车导航，不用人工在 RViz 点目标。
2. **3D（VGICP）定位收敛/漂移修复**：拆分 rough/refine 两级收敛阈值、跳变门（AlignmentGate）、map→odom 平滑（EMA blend）、自适应分数门（ScoreGate，基线相对离群判据代替绝对阈值），解决了长时间运行 yaw 漂移和死锁的问题。详见 `3D_LOCALIZATION_CHANGE_AND_TEST.md`。
3. **参数调优**：`clearance_radius`（目标点离障碍物的安全半径）从 0.33 → 0.22 → 0.25 → 0.27 来回调；`xy_goal_tolerance` 0.10→0.05；MPPI `progress_checker.required_movement_radius` 0.5→0.25（放宽"多久没往前挪就判定卡住"的距离阈值）。

## 2. 系统架构总览

```text
传感器与定位链路：
  MID360 -> livox_self_filter -> FAST-LIO -> nav_interface_adapter
         -> chassis_3d_localization (两级 VGICP) -> TF map -> odom -> base_link
                                                  -> Nav2 (SmacPlanner2D + MPPI Omni)

Task Manager 目标链路（本次新增）：
  task_manager (另一台机器)
    --/navigation/target_pose(PoseStamped)-->
      task_navigation_bridge_node  -- 变换到 map 系 + 用 /map 投影校验 -->
      NavigateToPose action --> Nav2
                                  |
                                  v (action status)
  task_manager <--/navigation/arrived(Bool)-- arrival_monitor_node

RViz 手动目标链路（保留，互不干扰）：
  RViz "2D Goal Pose" --/goal_pose--> goal_regenerator_node --/validated_goal_pose--> Nav2
```

`task_navigation_bridge_node` 和 `goal_regenerator_node` 代码结构几乎一样（都基于 `GoalProjector` 做投影校验），但订阅的输入 topic 不同——前者服务 task manager，后者服务 RViz，两者独立运行、互不影响。

## 3. 一键启动方式

a. 启动底盘驱动
```bash
cd agv_ws
sudo ./start_agv.sh /home/hkclr/agv_ws/src/agv2_pkg/config/chassis_robot2.yaml
```
b. 启动导航
```bash
# 默认启动（使用左前方的lidar）。
cd code/chassis_nav_ws
./nav_3d_in_map.sh
```

脚本做的事（`nav_3d_in_map.sh`）：起一个 tmux session（默认名 `nav_3d_in_map`），按依赖顺序在独立 pane 里依次拉起：

| pane | 内容 |
| --- | --- |
| sensor_stack | Livox 驱动 + self_filter |
| adapter | FAST-LIO body → nav 接口 adapter |
| localizer_3d | chassis_3d_localization（VGICP，含新的收敛/平滑/分数门逻辑） |
| map_server | 2D 栅格图（供 costmap 与目标投影用） |
| nav2 | Nav2（SmacPlanner2D + MPPI）+ `goal_regenerator` + `task_navigation_bridge` + `arrival_monitor`（三者随 nav2.launch.py 一起起，`enable_task_bridge` 默认 true） |
| rviz | 可选，`RUN_RVIZ=0` 关闭 |

常用环境变量：`LIDAR_PROFILE`（默认 `left_front_119`）、`NAV_LOG_DIR`（每个 pane 落盘日志，默认 `logs/<session>_<时间戳>`，设 `0` 关闭）、`INITIAL_POSE_DELAY`/`INITIAL_POSE_TIMEOUT`。不传 `initial_pose` 时，启动后需在 RViz 用 "2D Pose Estimate" 手动发一次 `/initialpose`，定位收敛稳定后 Task Manager 或 RViz 目标才建议发送。

关闭：

```bash
tmux kill-session -t nav_3d_in_map
```

## 4. 与 Task Manager 的通信方式（核心）

### 4.1 输入：目标点

- Topic：`/navigation/target_pose`，类型 `geometry_msgs/msg/PoseStamped`，由 task manager（另一台机器，节点 `/task_manager`、`/conference_room_pose_server`）发布。
- `frame_id` 固定为 `meeting_room2_source_headcam`——地图里头顶相机的光学系静态 frame，`translation=[0,0,~2.0]`，由 `map_server.launch.py` 从地图目录下的 `static_tfs.yaml` 加载为 `map -> meeting_room2_source_headcam` 的静态 TF（相机朝下俯视，非水平安装）。**这个静态 TF 是每张地图自带的，换地图要重新标定/写这个文件。**
- `task_navigation_bridge_node` 用 tf2 把收到的目标从相机系变换到 `map` 系（若 `frame_id` 已经是 `map` 则直接用，为空则按 `map` 处理并告警）。

### 4.2 校验与投影

- 订阅 `/map`（`nav_msgs/OccupancyGrid`，transient_local QoS），用 `GoalProjector` 检查目标点是否落在自由空间、离障碍物是否有足够安全间隙；不满足则在目标点周围按 `max_search_radius` 找最近的合法落点并重投影（并重新计算朝向），完全找不到就拒绝该目标（不会发给 Nav2）。
- 相关参数（`nav2.launch.py` 里传给 `task_navigation_bridge` 节点）：`clearance_radius`（当前 **0.27m**，即目标点周围至少留这么大的安全半径，是本批改动里改得最多的参数）；`max_search_radius`（3.0m）、`free_threshold`（25）、`allow_unknown`（false）为代码默认值，本批未单独调。
- 没收到过 `/map`，或投影失败时默认**拒绝**该目标（`fallback_to_raw` 默认 false，不会裸发原始坐标给 Nav2）。

### 4.3 转发给 Nav2

- 校验通过后，通过 action client 直接给 `/navigate_to_pose`（`nav2_msgs/action/NavigateToPose`）发目标，**不经过** `/goal_pose` topic（那是 RViz 手动目标走的路，见 §2 图）。
- 每来一个新目标会先取消上一个还在跑的 goal（用递增序号防止旧目标的回调污染新状态）。

### 4.4 到达回执

- `arrival_monitor_node` 订阅 Nav2 的 `/navigate_to_pose/_action/status`，以及 `/navigation/target_pose`、`/goal_pose` 两个"目标输入"topic（用于在新目标到来时立刻把 `arrived` 清成待定状态）。
- Nav2 goal 结束（成功/失败/取消）后发布 `/navigation/arrived`（`std_msgs/Bool`，true=到达成功，false=失败/取消），并**连续发 2 次、间隔 0.1s**（`repeat_count`/`repeat_interval`）以防对端订阅晚了漏收。

### 4.5 已知限制：跨机通信

单机（这台车载机内部各节点互相通信）已验证端到端可用。**如果 task manager 部署在另一台物理机上，需要先打通两台机器之间的 ROS 2 DDS discovery（同网段组播，或配置 Fast-DDS 的 initial peers / discovery server）**，否则会出现"task manager 发了目标车没反应"的现象——这不是这批代码的 bug，是网络层没打通。跨机部署前务必先用 `ros2 topic echo /navigation/target_pose`（在车上）确认能收到对端发布的消息。

## 5. 逐提交变更清单

| commit | 日期 | 说明 |
| --- | --- | --- |
| `439826e` | 06-29 | 新增 `task_navigation_bridge_node`：接收 `/navigation/target_pose`，变换+投影校验后转发到 Nav2 action |
| `8181d90` | 06-29 | 3D 导航默认参数微调 + 子模块指针更新（FAST_LIO / livox_ros_driver2） |
| `46bd0c8` | 07-06 | 重写 `nav_3d_in_map.sh` 启动流程与目标点安全间隙逻辑；精简 README |
| `4390446` | 07-07 | 新增 `arrival_monitor_node`：监听 Nav2 action status，发布 `/navigation/arrived` 回执给 task manager |
| `0db3b11` | 07-07 | 修 3D 定位漂移：`AlignmentGate` 跳变门 + 启动日志落盘（`NAV_LOG_DIR`、tmux pipe-pane、`[align]` 结构化日志） |
| `6307959` | 07-07 | 补 `.gitignore` |
| `8e305d6` | 07-07 | VGICP 参数：`refine_score_thresh` 0.15→0.12，`transformation_epsilon` 1e-6→1e-4（原值导致约 80% 尝试误判 not_converged） |
| `5db5d13` | 07-08 | `xy_goal_tolerance` 0.10→0.05；`clearance_radius` 0.33→0.22 |
| `96286d4` | 07-08 | 定位收敛/平滑/自适应分数门大改：rough/refine 分级停机条件、`correction_smoothing`（EMA α=0.4）、`ScoreGate`（基线×1.2 相对离群判据代替绝对阈值），配套 gtest |
| `58e3484` | 07-08 | `clearance_radius` 0.22→0.25 |
| `24ab089` | 07-20（今天） | `clearance_radius` 0.25→0.27；MPPI `progress_checker.required_movement_radius` 0.5→0.25；`nav.rviz` 视图调整 |

## 6. 已知问题 / 上车验证前请注意

以下几点是读代码逐行核实过的真实行为，**不是猜测**，接手前建议留意：

1. **重定位期间 TF 完全停播，不是发布旧值。** `initialpose_callback`（`chassis_3d_localizer_node.cpp:559-598`）收到 `/initialpose` 后立即置 `initialized_=false`；`tf_timer_callback`（同文件 `:600-621`）在 `!initialized_` 时直接 `return`，即整个重定位窗口内 `map -> odom` 完全不发布（不是卡在旧值），Nav2 会因为 TF 数据太旧而中断当前 goal。人工点 `2D Pose Estimate` 之后到重新收敛之间这几秒，机器人侧会经历一次 abort + 恢复行为，属预期但比较突兀。
2. **重定位后"首次接受"的解会跳过跳变门，可能锁进对称误解。** `align_timer_callback` 里 `bypass_jump_check = has_pending_initial_guess || !initialized`（`:711`），一旦这次解通过分数门，`full_jump = bypass_jump_check || jump_overridden`（`:727`）会无条件采信，哪怕它和历史朝向差出一个房间对称角度（比如 90°）也不会被跳变门拦。矩形/对称房间里手点初始位姿如果点偏了角度，有小概率直接锁死在错误解上，之后长期定位失败。
3. **`rough_score_thresh` 拒绝不计入任何"连续拒绝后放行"的逃生机制。** 该拒绝发生在 `align_scan_to_map`（`:857-860`），比 `ScoreGate`/`AlignmentGate` 的求值点（`:691` 起）更早返回，两个 gate 各自的"连续 N 次拒绝后放行"计数器完全看不到这类拒绝。如果场景变化导致 rough 阶段系统性拒绝，会卡死且没有自动恢复路径，需要人工重新点位姿。

## 7. 相关文档

- [3D VGICP 定位变更与测试](3D_LOCALIZATION_CHANGE_AND_TEST.md)
- [README](README.md)：整体构建/启动/常用命令
