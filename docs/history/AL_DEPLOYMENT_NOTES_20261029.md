# AL：门检测 + Nav2 巡检

部署主机：`hkclr@10.30.3.234`。独立根目录：`/home/hkclr/AL`。
WebUI：<http://10.30.3.234:8092/#patrol>。
开发本本次已打开 SSH 转发入口：<http://127.0.0.1:18925/#patrol>（转发断开后使用上面的局域网地址）。

## AL / BL 边界

- BL：`/home/hkclr/chassis_nav_ws`。只读配置来源，不允许本部署启动、停止或修改 BL。
- AL 的导航、Livox、FAST-LIO、VGICP、MPPI 都由 AL 源码重新编译；不 source BL/install，不执行 BL 二进制。
- BL 的导航 YAML、雷达外参/过滤、FAST-LIO、adapter、定位 YAML 和 hkclr_office 地图复制后保持原样。所有日志、构建、虚拟环境和运行配置写入 AL。
- `evidence/BL_before.sha256` 记录部署前 4,798 个 BL 文件指纹，部署后再次校验。
- 现有 `agv_control` 是独立硬件驱动，保持原状。AL 使用同一 ROS Domain 76 和原 `/cmd_vel` 接口，不修改驱动、手柄或底盘模式。
- AL 不自启巡检。没有任务时不发送导航目标；本次验收不做实车运动测试。

## 启停

### 原生 RGB 录像（2026-10-29）

页面右侧顶部“原生 RGB 录像”提供手动开始、停止按钮，独立于所有巡检 / 定位操作。录像直接接入 D435i 原生采集帧，1920×1080，不使用 960×540 网页预览，不叠加检测框或文字。PyAV 17.1.0 / H.264 CRF 20 编码为 MP4；采用采集时间戳保留实际时长。队列有界，编码跟不上时丢帧计数会显示，避免阻塞相机和门检测。

录像在 `recordings/rgb` 下先写 fragmented MP4，停止并完成封装后才开放下载。任务完成、取消、定位及仅重启 web 不会停止录像；重启相机 / 关闭整机则会中断采集。异常中断留下的 `.partial.mp4` 不冒充完整录像。

开发本运行 `AL_deployment_20261028/start_rgb_receiver.sh` 启动用户级 `al-rgb-receiver.service`（本次已启动，不新增开机自启）。接收程序通过主控局域网 `http://10.30.3.234:8092` 自动拉取已完成视频，不依赖浏览器保持打开或 SSH 页面转发。保存于 `/home/wolfie/Documents/ChatGPT/Campus Library Logistics/recordings/AL_RGB`；SHA-256 和长度匹配后才向页面回报“已传回本机”。断网时保留小车端原片并自动重试，文件不会自动删除。开发本重启后须重新运行接收脚本。

本次 4 秒端到端验证：1920×1080，115 帧，编码丢帧 0，MP4 约 4.24 MB，已完成自动回传及校验。录像控制只使用已有 web HTTP 与相机进程内文件通信，不增加 ROS 导航 / 底盘控制接口。

2026-10-28 更新：GUI 的“以参考位姿开始定位”先重启 AL FAST-LIO，等待新进程发布的新时间戳 `/Odometry` 与 `/sensors/cloud`，再调用原 `/localizer/relocalize`。参考位姿仍只是 VGICP 搜索初值。启动中的巡检须先用原取消功能结束，重定位不派发导航目标。
AL 的 FAST-LIO launch 增加 `respawn=True` 和进程标记；GUI 仅向该 AL 可执行文件、Domain 76、带标记的唯一进程发 SIGINT，由现有 ROS launch 重启，不新增 ROS 服务。每次重启前将 FAST-LIO 状态日志归档至 `logs/fastlio_sessions/`。重启清空滤波、偏置和内部地图状态；无法保证消除导致发散的原始输入问题。BL 不变。

```bash
bash /home/hkclr/AL/start_al.sh
tmux attach -t AL
bash /home/hkclr/AL/stop_al.sh
```

AL tmux 含 stack、camera、web 三个窗口；日志在 `logs/stack.log`、`logs/camera.log`、`logs/web.log`。
停止脚本只关闭 AL；先发 SIGINT，最多等 10 秒，再关闭仍被 MJPEG 客户端占用的 AL 会话。
不新增开机自启，也不要运行 BL 的 nav_3d_in_map.sh。

## 原生流程

GUI 提交红/蓝点队列 → 原 manager `/patrol/start` → Nav2 顺序目标。蓝点到达后立即进入下一点；红点到达且正对门 → 实测持续停稳 → 原门检测服务 → 全开 100 后 manager 发下一点。最后一点为红点时，也必须完成门检测才结束任务。
GUI 不发布速度，不替 manager 派发顺序目标。保留昨天已验证的 manager 状态机、取消处理、停稳和朝向确认。
停稳速度估计使用 `/odom` 最近 1 秒的 XY / 解缠 Yaw 最小二乘趋势，不再将两帧雷达定位抖动直接除以约 0.1 秒。仍取该估计与消息中实测 twist 的较大值，仍要求 ≤0.02 m/s、≤0.03 rad/s 连续 1 秒；不把零指令或 FAST-LIO 的零填充 twist 单独当作已经停稳。完整窗口不足、换坐标系、时间回跳或数据间断会重新采样。状态中的 `door.settling` 提供实际判定速度，`door.position_latched` 显示到达锁存。
不启动原桌边插入/后退桥，避免它干预门前巡检（与昨天集成一致）。

门检测沿用 epoch-11 ONNX、原 RGB 预处理；采用今天确认的规则：采样 1 秒，最多 5 个有效帧，不足 5 帧也推理；平均全开概率严格 >0.47 返回 100，否则返回 0。
只使用 D435i RGB，不启用深度。`/camera/d435i/color/image_raw`，640×480，最新帧发布目标 15 Hz。
相机当前稳定路径：`/dev/v4l/by-path/pci-0000:00:14.0-usb-0:1.1:1.3-video-index0`；更换 USB 插孔时需只修改 AL 的相机参数。

## BL 配置复用

- MID360：192.168.1.118；接收主机：192.168.1.50。
- 雷达 profile：front_118；外参/自过滤保留 BL 原文件。
- 地图：`src/bringup/maps/hkclr_office/{2D,3D}`。
- MPPI：`src/navigation/config/nav2_mppi.yaml`，完整原样复制，不用昨天小车演示副本的 nav2_door.yaml。
- MPPI 上限：vx 0.45 m/s、vy 0.25 m/s、wz 0.70 rad/s；下游底盘原有限制仍独立生效。
- 速度拓扑保持本机 Humble 原生配置：controller → `/cmd_vel_nav` → smoother → `/cmd_vel` → 底盘；behavior_server 的原生 `/cmd_vel` 输出也保留。AL 外层不再添加会覆盖原生拓扑的重映射。
- 规划 footprint：前后总长 0.56 m，左右总宽 0.36 m；其余参数见原 YAML。
- 初始定位本次由用户确认：x=-6.968、y=-37.587、z=0.05、yaw=0。不是永久默认位置；重新搬动车辆后应在 GUI 手动设置。

门前和门后任务点尚未配置，保留 null，不把定位初值或历史示例当作真实目标。

## 依赖与测试

Ubuntu 22.04 / ROS2 Humble；AL `.venv` 只覆盖自己的 Python 依赖，复用系统 ROS 模块。
ONNX Runtime 1.23.2、NumPy 1.26.4、Pillow 12.2.0、aiohttp 3.8.1。
CPU 推理，不依赖开发本或 GPU 服务器；运行中不需要开发本的 SSH 图像转发。

隔离回归使用 Domain 192 / localhost；实际运行 Domain 76。回归包含模拟 Nav2 Action、原 manager 和真实门模型，不驱动物理底盘。
验收报告：`evidence/regression.xml`、`evidence/live_topics.json`、`evidence/live_status.json`、`evidence/BL_after_check.txt`。

验收通过仅说明系统与传感器链路正常，不代表已经完成真实过门、动态避障或地图定位精度验证。

## 2026-10-28 部署验收

- 51 项隔离回归全部通过，包括真实门模型和模拟 Nav2 下的 manager 下一路径点派发。
- AL 重启后实测：雷达 9.99 Hz、里程计 9.99 Hz、D435i RGB 12.15 Hz。
- 六个导航 lifecycle 组件均为 active；原生速度链路审计通过。
- GUI 静止诊断调用真实门服务成功，5 帧平均概率 98.41%，返回 100；该操作没有让 manager 派发路径点。
- BL 的 4,798 个已记录文件 SHA-256 全部一致，未启动 BL。
- manager 为 IDLE；本次未发实际导航目标，12 秒采样窗口内未观察到 `/cmd_vel` 消息。
- 后续真实巡检前，需在 GUI 配置门前点、正对门 Yaw 和门后点；当前目标坐标有意保留为空。

## AL 2D 地图界面（2026-10-28）

主页已改为 2D 俯视地图，原 3D 页面文件仍保留。更新前完整 GUI 备份在
`backups/web_before_2d_20261028.tar.gz`。只重启过 AL 的 web 窗口；stack、camera 和原底盘驱动未重启。

- 底图直接读取 AL 的 hkclr_office PGM/YAML，5 cm/格，尊重原点与旋转；黑=占据、白=空闲、灰=未知。
- 蓝色箭头表示 `map → base_link` 当前位置与朝向，绿色轮廓来自原 Nav2 footprint；蓝色全局路径与绿色 MPPI 最优局部轨迹仅作可视化。
- 先在巡检页选中一个路径点，再按“选取路径点”。单击地图设 XY/Z=0；拖动同时设 Yaw。只修改浏览器草稿，**不会自动保存或派发目标**。
- 直接点击地图上的编号可选中对应路径点。编号保留原顺序，不因前面的空坐标点被隐藏而重排。
- 滚轮缩放、拖动平移；右键拖动始终平移；“当前位置”回到机器人，“跟随”随位置更新，“整张地图”显示全图。
- “测距”在地图点击两点，显示米制距离；“选取定位参考点”只填入定位表单，仍须手动点击“以参考位姿开始定位”。
- 参考 BL `src/bringup/rviz/nav.rviz` 的占据地图、机器人轮廓、路径与取点工具；没有添加 ROS 服务、速度发布器或规划逻辑。
- 5 项 JS 坐标几何测试、17 项 HTTP/原 manager gate 测试通过。浏览器实测拖动产生 +90° Yaw，测距正常，无前端错误；测试草稿已丢弃，没有保存任务或发送导航目标。

本次界面验收另观察到 `/livox/lidar` 仍为约 10 Hz，但 `/sensors/cloud` 暂无新消息；RGB 与 TF 位姿仍更新。
该点云链路问题不靠 GUI 伪造数据掩盖，未在此次界面修改中重启或改变导航链路。

## 红 / 蓝点与实时模式（2026-10-28）

- 巡检点增加 `type: blue | red`。蓝点无 manager 额外等待、停稳检查或门检测；仍使用 Nav2 原生 NavigateToPose 到达判定，未修改 MPPI、Nav2 goal checker 或速度参数。
- 红点采用位置到达锁存：收到该点 Nav2 `SUCCEEDED` 后继承其 XY 到达结果，不因随后地图定位漂移反复检查 5 cm；每个新点使用独立锁存。仍按该点 Yaw 和实测速度确认连续停稳，然后调用原 `/inspection/detection_door_opening`，间隔至少 1 秒、无并发；仅返回 100 放行。不限红点数量和位置，红点不必排第一。
- 旧 YAML 没有 type 时兼容原语义：第一点红、其余蓝；GUI 读取后明确显示类型，保存时写入 type。有效队列至少 1 点，不再要求至少两个点。
- 每个点支持地图点击 / 拖动设朝向、数值编辑、红蓝切换、上移 / 下移和删除；地图用红蓝圆点、编号和朝向箭头显示，选中点加亮边框，不改变其类型颜色。
- “连续地图选点”每次点击添加一点，点类型由旁边下拉框决定；“在地图替换第 N 点”修改已选点。选点本身不启动。
- 巡检页：保存 YAML 或保存并启动。实时页：草稿仅在浏览器内，点击“确认并立即顺序执行”提交完整内存快照；不调用 settings 保存、不产生 YAML / 备份 / 临时任务文件。
- 实时任务要求当前 map → base_link 定位可用。机器人已在执行任务时不能同时启动另一队列，可使用原“取消任务”；编辑草稿不会改变已经接收的队列。
- 两种模式均使用同一个 AL manager。无新增 ROS 服务定义、调度器或速度发布器：实时快照经节点原生 `/patrol_manager/set_parameters_atomically` 设置 `runtime_mission_json`，再调用现有 `/patrol/start`。参数为一次性输入，启动尝试时立即消费 / 清空；普通巡检启动显式清空临时输入、读取原 YAML。
- `/patrol/status` 增加 `task_mode`、点类型与队列快照；页面刷新可以显示 manager 正在执行的任务信息，但不会自动重启任务。未提交的实时草稿不保存。
- 更新前备份：`backups/before_red_blue_realtime_20261028.tar.gz`。
- 本次只热替换 AL manager 与 web：临时 `AL:manager` 窗口替代 stack 启动的旧 manager，不与其并存；下一次正常 stop/start 仍由原 stack 启动唯一 manager。相机、定位、Nav2、底盘驱动不重启。
- 测试：Domain 192 / localhost 下 60 项 Python 回归（含真实门模型、模拟 Nav2、混合多红点、取消、实时内存任务及 YAML 不变检查）和 7 项 JS 几何 / 点编辑测试。结果见 `evidence/red_blue_tests.xml`。未发送实车导航目标。
- 浏览器实测连续添加红/蓝点、拖动设置 +90° 朝向、类型切换、顺序调整和两页草稿隔离；测试草稿已清空，manager 保持 IDLE。原任务 YAML SHA-256 未变，BL 的 4,798 文件校验全部通过。
- 本次验收 RGB 与位姿持续更新；Web 重启后 `/sensors/cloud` 已有消息，但仍间歇超过 1 秒（末次采样约 2.95 秒）。未在此功能修改中调整雷达 / FAST-LIO / 导航链路，不将其表述为全链路实时性已通过。

## 定位参考与匹配结果（2026-10-28）

- GUI 的 XYZ / Yaw 仅为搜索参考，不直接写机器人最终位姿或 `map → odom`。调用已有 `/localizer/relocalize`，使用 AL 当前 hkclr_office 3D PCD；自定义地图时须同步设置 web `--pcd-map`，与定位节点地图保持一致。
- 原模块按参考附近子地图进行 VGICP 粗配准、精配准，成功后才更新 TF。接口返回 `success` 仅表示接收初值；GUI 用原 `/localizer/relocalize_check` 的 `code=0` 查询是否已建立定位，绝不使用 `code=1` 的强制有效路径。
- GUI 显示参考已提交 / 等待匹配 / 定位已建立，并读取原 `/rosout` 显示最近粗精配准 RMS 和采用或拒绝原因。已建立定位后单次匹配被拒绝，并不表示未曾初始化；原模块可能仍使用上次配准与里程计传播，页面分别展示这些信息。
- 无新鲜 TF 或原模块尚未确认定位时，不再把旧缓存箭头显示成当前位置。此为显示修复，不新增导航门控，不改变 manager、Nav2 或定位参数。
- 保留原限制：执行中的导航任务需先取消，再提交新的定位参考。启动定位本身不发布导航目标、不会让车辆移动。
