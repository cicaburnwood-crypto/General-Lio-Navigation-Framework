# 234 整车 ROS DOMAIN 隔离

适用：10.30.3.234 / hkclr；部署日期：2026-09-30。

## 配置与覆盖范围

本车固定使用 `ROS_DOMAIN_ID=76`。唯一配置文件为
`/home/hkclr/.config/hkclr/ros_domain.sh`；正式入口在 ROS/工作区 setup 之后加载。
遇到外部不同域值时打印覆盖提示。配置缺失、加载失败或值不是 76 时，正式入口拒绝启动。
175 未修改，不得同时设置为 76。域隔离是 DDS 通信分组，不是安全认证或网络防火墙。

- 底盘 CAN 接收、CAN 发送、joy、agv2 控制、故障 rosbag、快照及 respawn 子进程均继承 76。
- 底盘停机服务客户端也使用 76。
- 2D/3D 导航每个 tmux 窗格明确加载配置，包括雷达、FAST-LIO、定位、地图、Nav2、RViz、初始位姿。
- hkclr 的新交互 Bash 终端默认加载配置；旧终端、已运行节点、已有 tmux server 不会被自动迁移。
- 非交互 SSH、独立桥接程序、手动启动命令应显式 source 本车环境，不能仅依赖 `.bashrc`。
- 未修改运动控制、CAN ID、底盘参数、地图、雷达 IP、RMW、localhost 设置、ROS 接口或日志录制行为；无需重新编译。

## 原有启停命令不变

先确认现场安全条件。启动期间物理急停保持按下：

```bash
cd /home/hkclr/agv/agv_ws/src/agv2_pkg
sudo ./deploy/start_agv.sh "$PWD/deploy/chassis_full_manual_test.yaml"
```

查看底盘：`tmux attach -t agv_control`。启动脚本打印域和配置来源。
已有同名会话时现在会拒绝覆盖，不再杀掉旧会话后重建。

停止前按下物理急停，保留原有顺序：停机服务成功 → 关闭底盘会话 → CAN DOWN。

```bash
cd /home/hkclr/agv/agv_ws/src/agv2_pkg
sudo ./deploy/stop_agv.sh
```

配置预检失败、会话缺少域标识/不是 76、停机服务失败时，不强制关闭底盘或 CAN。
这不是软件自动安全停车；出现拒绝提示时保持物理急停并由现场人员核查，不要反复强启。

导航仍使用原来的命令、地图与雷达参数：

```bash
cd /home/hkclr/chassis_nav_ws
./nav_in_map.sh
# 或原有 3D 导航命令及参数；不要同时启动两套控制链
./nav_3d_in_map.sh
```

仅允许复用带有 76 标识的新导航会话。未知域/旧域会话不会自动关闭或迁移。
会话标识用于防止误复用，不替代真实进程环境检查。

## 手动程序、旧终端与外部电脑

底盘 CLI / 独立自动控制桥：

```bash
source /home/hkclr/agv/agv_ws/src/agv2_pkg/deploy/setup_agv_env.bash
echo "$ROS_DOMAIN_ID"  # 应为 76
```

独立导航/雷达工具：

```bash
source /home/hkclr/chassis_nav_ws/install/setup.bash
source /home/hkclr/.config/hkclr/ros_domain.sh
```

本机已有速度测试程序保持不变；只有需要运动测试且现场确认安全时才运行。此次验证不会运行它。
外部电脑启动对应 234 的 RViz、遥控、导航或日志进程前，在其 ROS setup 后执行：

```bash
export ROS_DOMAIN_ID=76
```

两端还需保持网络与 ROS QoS 兼容。不依赖 `sudo -E`；本车入口在切回 hkclr 用户后显式加载。
未来新增 systemd/Docker 启动必须在进程环境中明确设置 76，不能假设其读取 `.bashrc`。
自定义 UDP/HTTP 与 Livox 原始网络通信不受 DOMAIN 隔离；只隔离其 ROS 侧。

## 只读检查

```bash
source /home/hkclr/agv/agv_ws/src/agv2_pkg/deploy/setup_agv_env.bash
echo "$ROS_DOMAIN_ID"
ros2 node list
ros2 topic info /to_can_bus -v
ros2 topic info /from_can_bus -v
```

检查真实 PID（替换为已确认的数字）：

```bash
tr '\0' '\n' < /proc/<PID>/environ | grep '^ROS_DOMAIN_ID='
```

底盘、CAN 收发、日志、雷达、导航及 RViz 都应显示 76。
本车日志录制会增加订阅者，因此不能以“必须只有一个 CAN 订阅者”验收。
检查节点名称和端点归属，域 76 中不得出现 175 的底盘节点。
CLI daemon 按域分别运行；在正确域查询即可，不需要杀死所有 ROS daemon。
不要为了检查而订阅/发布控制话题或调用停机、锁车等控制服务。

## 无硬件回归测试

```bash
source /opt/ros/humble/setup.bash
python3 /home/hkclr/agv/agv_ws/src/agv2_pkg/test/test_domain_isolation.py --tmux --ros
```

启动/停止脚本仅在临时副本中测试：替代 sudo、ip、tmux、ros2 等命令；不触碰真实 CAN。
实际 tmux 测试使用随机独立 socket，仅启动环境探针，结束后关闭该测试 server。
实际 ROS 测试限定 localhost，使用独立 String 测试话题和域 0/76，不使用机器人话题或服务。
测试 rosbag/重启子进程的是环境继承，不会真实启动驱动或 recorder。

## 备份与回滚

备份：`/home/hkclr/domain234_backups/20260930_164957`。
包含原文件、权限/哈希清单、修改前 Git 状态、源码与配置校验以及部署校验结果。
导航原有未提交修改按现场内容备份，不以 Git HEAD 覆盖。

若需要回滚，先在现场安全停机（仍使用当前域 76），关闭相关导航进程，再执行：

```bash
python3 /home/hkclr/domain234_backups/20260930_164957/deploy.py rollback
```

回滚工具遇到运行中的相关程序或部署后另有文件修改会拒绝继续，避免覆盖新工作。
仅恢复此次修改文件；新增文件移入备份内保留，不做整仓库 reset，不删除地图、日志或构建。
旧终端仍保留旧环境；回滚后需重新建立一致环境再启动。
恢复默认域 0 后禁止两车同网同时运行，直到重新建立隔离。

## 验收边界

部署及自动化测试不自动启动真实底盘/整机、不发送速度/CAN 指令、不调用真实控制服务。
现场人员确认安全后再做两车同时在线的静止验收，并核对正常停机。
域隔离针对已发现的 ROS 串扰风险；尚不能保证所有舵轮抖动均由串扰导致。
若域隔离后仍抖动，应保留日志继续检查控制输入、CAN 反馈、电气与机械因素。

依据：[ROS 2 Humble Domain ID](https://github.com/ros2/ros2_documentation/blob/humble/source/Concepts/Intermediate/About-Domain-ID.rst)
及 [CLI daemon 分域说明](https://github.com/ros2/ros2_documentation/blob/humble/source/Concepts/Basic/About-Command-Line-Tools.rst)。
