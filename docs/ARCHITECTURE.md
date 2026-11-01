# 架构与接口

| 层 | 实现 | 职责 |
|---|---|---|
| 雷达 | livox_ros_driver2、livox_self_filter | 原始点云/IMU、本体过滤 |
| 里程计 | FAST_LIO | /Odometry、/cloud_registered_body |
| 导航适配 | nav_interface_adapter | /odom、/scan、/sensors/cloud、TF |
| 全局定位 | chassis_3d_localization / scan_registration | 3D PCD VGICP、map→odom |
| 地图 | Nav2 map_server | 静态 2D occupancy grid |
| 规划/跟踪 | SmacPlanner2D / MPPI Omni | 原生 Nav2 路径、控制与恢复 |
| 速度 | Nav2 velocity_smoother + 外部 agv2_pkg | /cmd_vel_nav → /cmd_vel → CAN |
| 任务 | patrol_manager | 红蓝点队列、Nav2 action、门检测联动 |
| 门模型 | door_opening_server | RGB 单次采样、ONNX、返回 0/100 |
| GUI | integration/web_server.py、ros_bridge.py、web/ | 状态、2D地图、点编辑、既有接口调用 |

## 使用的接口

| 接口 | 类型 | 说明 |
|---|---|---|
| `/navigate_to_pose` | nav2_msgs/action/NavigateToPose | manager 独占顺序目标派发 |
| `/patrol/start` | std_srvs/srv/Trigger | 启动已配置任务 |
| `/patrol/cancel` | std_srvs/srv/Trigger | 取消任务 |
| `/patrol/status` | std_msgs/msg/String | JSON 任务状态 |
| `/patrol_manager/set_parameters_atomically` | rcl_interfaces/srv/SetParametersAtomically | `runtime_mission_json` 一次性实时队列 |
| `/controller_server/set_parameters_atomically` | 同上 | manager 根据红蓝点设到达容差 |
| `/inspection/detection_door_opening` | scan_planner_msgs/srv/DetectDoorOpening | 单次门检测 |
| `/localizer/relocalize` | 见 chassis_localization_msgs/srv | 参考初值启动原生 VGICP 匹配 |
| `/localizer/relocalize_check` | 见 chassis_localization_msgs/srv | 原生定位状态确认 |

ROS 命令前先 source Humble 和本工作区，设置 `ROS_DOMAIN_ID=76`。

```bash
ros2 service call /patrol/start std_srvs/srv/Trigger '{}'
ros2 service call /patrol/cancel std_srvs/srv/Trigger '{}'
ros2 topic echo /patrol/status
ros2 topic hz /sensors/cloud
ros2 topic hz /camera/d435i/color/image_raw
```

第一条命令会真正开始导航；不是健康检查。

## GUI 是否改变导航

GUI 不发布 `/cmd_vel`，不自行规划，不用浏览器定时器逐点调度；导航由 onboard ROS 进程完成。但配置/启动/取消/重定位按钮会通过既有服务影响任务。Web 后端的 RGB JPEG 编码和点云可视化会消耗 CPU/DDS 资源，不能据此声称 GUI 对实时性绝对零影响。

AL 和 BL 的核心 Nav2/MPPI/VGICP 源码及默认 MPPI YAML 保持原底座设计；完整流程不同：AL 关闭 table bridge，由门巡检 manager 接管队列；蓝点容差翻倍；GUI 重定位重启 FAST-LIO；静态地图是用户修改后的版本。AL 关闭 FAST-LIO `/Laser_map` 累积展示发布，保留内部地图与导航点云，并启用 AL 标记的 FAST-LIO 自动重启。

本仓库只是整理这些已有行为，没有增加新的运行服务、控制器、碰撞处理或任务规则。
