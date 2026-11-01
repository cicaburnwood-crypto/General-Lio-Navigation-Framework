# 配置表

| 文件 | 内容 |
|---|---|
| `src/bringup/config/lidar_profiles.yaml` | 单雷达 profile、外参、自过滤 |
| `src/bringup/config/livox/*.json` | 雷达与主机 UDP 地址/端口 |
| `integration/fastlio_navigation.yaml` | AL 实际使用的 FAST-LIO 参数 |
| `src/nav_interface_adapter/config/adapter_fastlio_body.yaml` | 点云/里程计标准化 |
| `src/chassis_3d_localization/config/vgicp_localizer.yaml` | 定位频率和匹配参数 |
| `src/navigation/config/nav2_mppi.yaml` | 默认实际 Nav2 参数 |
| `src/patrol_manager/config/door_patrol.yaml` | GUI 保存的巡检任务与门前停稳设置 |
| `src/door_opening_server/config/door_opening.yaml` | 门检测运行阈值 0.38 |
| `integration/door_stack.launch.py` | 顶层地图、模型与任务接入 |
| `src/bringup/maps/hkclr_office/` | 当前快照地图：2D PGM/YAML 与 3D PCD |

## 当前参数的不同层次

- Nav2 MPPI：20 Hz，Omni，vx_max 0.45 m/s、vx_min -0.15 m/s、vy_max 0.25 m/s、wz_max 0.70 rad/s。
- 规划车体：0.56 × 0.36 m；完整 footprint 和 inflation 以 Nav2 YAML 为准。
- manager 红点 Nav2 到达容差：0.05 m / 0.087 rad；蓝点为其 2 倍：0.10 m / 0.174 rad。
- manager 门前确认的 XY/Yaw 参数独立于上述 Nav2 goal checker。发布模板根据最后记录设为 0.1 m / 0.1 rad，位置在 Nav2 success 后锁存。
- 用户最后指定的停稳线速度阈值 **1.0 m/s**、角速度阈值 0.03 rad/s、连续确认 1 秒。1.0 是原任务配置，不能把该值解释为“物理上完全静止”。
- 每次门检测采样 1 秒，1–5 个有效 RGB 帧均可推理；平均 fully-open 概率严格大于 **0.38** 返回 **100**，否则返回 0。100 是放行码，不是 100% 置信度。模型元数据的 0.47 为训练选择记录，运行值由 ROS YAML 覆盖。
- 成功的门服务响应可覆盖推理期间停稳/朝向失效；取消任务不会因此恢复执行。尚未开始检测时的门前确认仍然存在。
- D435i：1920×1080、YUYV、请求 30 FPS；ROS 最新帧目标发布 15 Hz；Web 预览降采样，原生录像不使用预览画面。实际设备协商值见 camera.log。

默认点保留 null，明确要求 GUI 配置，避免新克隆自动携带旧场地可执行任务。需要部署旧任务时，应从车端取回实际 YAML；本次未能连接车端核对该文件。

## 地图

此版来自 `office_without_door` 处理后、截至 2026-10-29 的静态清理地图，不是 BL 初始地图。包含此前用户指定范围的 2D 障碍清理和相应 3D 点云裁剪；实时点云层仍独立生效。分辨率 0.05 m。

`docs/map_snapshot.json` 记录最后一次裁剪、边界和文件校验；`SOURCE_MANIFEST.sha256` 校验最终归档。地图名称 `hkclr_office` 是兼容路径，不代表内容与 BL 相同。换地图应同时更新 2D map_server、3D localizer 和 WebUI 的 map/pcd-map 参数。

## 修改范围

本次只在独立归档中补齐依赖/文档，沿用现有 AL manager/WebUI。任务模板的停稳值按已确认历史配置整理；没有更改运行中的 AL 或任何 BL 文件。导航备选 `nav2.yaml`、`nav2_mppi_loose_yaw.yaml` 是本机底座的备选配置，和历史车端 BL 对应文件有差异，默认 launch 不使用它们。
