# Patrol Manager

读取 YAML 巡检任务，依次通过 `/navigation/navigate_to` Action 执行各点导航。到达普通巡检点后等待配置的检查时间，再执行下一点；配置了 home 时，最后导航回家。检查目前只是明确标记的占位等待，不执行相机或传感器检查。

## 代码分工

- `mission.py`：读取并校验任务；将可选 home 追加到点队列末尾。
- `start()`：加载任务、重置字段，进入 `WAITING_FOR_SERVER`。
- `build_goal()`、`send_goal()`：构造并异步发送目标。目标 Z 已按 `/Odometry_base.z - 0.3` 准备并原样传给全局规划器；manager 不再降低目标高度。反馈位姿为实际 base 位姿，Z 距离按 `abs((odom_base.z - 0.3) - target.z)` 计算。
- `feedback()`：保存导航反馈，供状态话题显示。
- `handle_result()`：读取 ROS Action 终态和 NavigateTo.Result；普通点成功进入检查，home 成功结束，失败停止队列，停止未确认进入 BLOCKED。
- `inspect()`：记录单调时钟截止时间，不阻塞。等待使用实际经过的秒数，不随 rosbag 回放倍率变化。
- `cancel()`：停止队列推进；已有导航时等待最终结果确认结束。
- `tick()`：每 0.2 秒检查状态和 Future，串联以上函数。

## 状态流程

```text
WAITING_FOR_SERVER -> WAITING_FOR_ACCEPTANCE -> NAVIGATING -> INSPECTING
       ^                                                       |
       +---------------------- 下一点 --------------------------+
最后普通点检查完成，或 home 导航成功 -> SUCCEEDED
目标拒绝或导航失败 -> FAILED
执行中取消 -> CANCELING -> 等待导航最终结果 -> CANCELED
停止确认失败 -> BLOCKED（禁止启动新任务）
```

取消等待接受中的目标时，先等接受答复；若接受，再请求取消。取消回复只表示请求是否被接受，不表示导航已经停止。若取消被拒绝，仍等待导航最终结果，期间不发送下一点。等待服务器及 Action 答复目前没有独立超时；导航执行超时由 NavigateTo.Goal 的 `timeout_sec` 和 navigation_manager 处理。

## 启动

先启动 Navigation Action Server 及底层导航组件，再启动巡检节点：

SCAN-NAV 托管模式的完整构建与五终端启动说明见 [PATROL_INTEGRATION.md](../../scan-nav/PATROL_INTEGRATION.md)。下面使用本次适配构建输出；该构建只纳入 scan-nav 内的一份 scan_planner_msgs。

```bash
source /home/kun/scan_nav_patrol_build/install/setup.bash
ros2 run patrol_manager patrol_manager --ros-args -p mission_file:=/home/kun/下载/patrol_align/patrol_manager/config/patrol.example.yaml
```

示例坐标在实际使用前需要核对地图。节点启动后为 IDLE，需要调用 start 才执行任务。

各终端使用同一 ROS Domain：

```bash
ros2 topic echo /patrol/status --qos-durability transient_local
ros2 service call /patrol/start std_srvs/srv/Trigger '{}'
ros2 service call /patrol/cancel std_srvs/srv/Trigger '{}'
```

start 和 cancel 都使用 `std_srvs/srv/Trigger`。cancel 返回 success=true 表示已受理取消，请通过状态确认最终 CANCELED 或 BLOCKED。活动任务或 BLOCKED 状态不允许再次 start。

`/patrol/status` 使用 `std_msgs/msg/String`，内容是 JSON：`state`、`reason`、`point_index`（从零开始）及 `navigation`。navigation 包含任务 UUID、阶段、距离、路径进度有效性和诊断；收到最终结果后记录 result_status、error_code、error_reason。未知距离转为 JSON null。

结果处理接收完整的 Action 响应：

```python
self.handle_result(self.result_future.result())
# response.status：ROS Action 终态
# response.result：NavigateTo.Result
```

