# General Lio Navigation Framework

基于 **ROS 2 Humble、Livox MID360、FAST-LIO、VGICP、Nav2 / MPPI** 的车载导航系统，包含 RGB 门检测、顺序巡检 manager、2D WebUI 与原生视频录制/回传。

这是现有 **AL 系统的独立源码整理版**，不是 BEV/VGGT 系统，也不会启动或修改 BL。导航和门检测均可在小车内置电脑运行，无需开发本 GPU。开发本仅用于浏览器、SSH 转发和可选的视频回传。

## 快速入口

- [安装和启动](docs/DEPLOYMENT.md)
- [架构、接口与 GUI 职责](docs/ARCHITECTURE.md)
- [配置与操作说明](docs/CONFIGURATION.md)
- [源码来源、AL/BL 差异与复现边界](docs/PROVENANCE.md)
- [第三方版权说明](THIRD_PARTY_NOTICES.md)
- [发布验证记录](docs/VALIDATION.md)

## 完整链路

```text
MID360 → 本体点云过滤 → FAST-LIO → 标准 /odom、/scan、/sensors/cloud
                              ↓
                      VGICP 对齐 3D 地图 → map→odom
                              ↓
2D 静态地图 → Nav2 SmacPlanner2D → MPPI → velocity_smoother → /cmd_vel
                                                                  ↓
                                                        原底盘 CAN 驱动

WebUI → patrol_manager → Nav2 NavigateToPose
          ├─ 蓝点：到达后进入下一点
          └─ 红点：到达锁存 → 朝向/停稳确认 → 门检测 → 成功后下一点

D435i 原生 RGB → 按需门检测 + Web 预览 + 手动 MP4 录像
                                            └→ 可选开发本校验回传
```

GUI 支持地图定位参考、红蓝点编辑/排序、保存巡检、实时内存队列、地图测距、实时位置/路径/点云、门检测状态、RGB 预览和手动录像。浏览器不发布速度、不计算路径，也不负责逐点派发。

## 获取

```bash
gh repo clone cicaburnwood-crypto/General-Lio-Navigation-Framework ~/AL-framework
cd ~/AL-framework
python3 tools/fetch_assets.py
```

门检测 ONNX 约 109 MiB，存放于同仓库 `v2026.11.01` Release；下载脚本会验证 SHA-256。私有仓库需要先 `gh auth login`。Git 源码包含最新保存的 2D/3D 地图，未包含实验视频、运行日志、密码、构建缓存或 SSH 密钥。

## 当前快照说明

整理日期 **2026-11-01**。车端 `10.30.3.234` 在整理时无法 SSH 连接，因此本版使用开发本保存的 AL 部署源码、2026-10-29 最新地图及已记录的配置；**不是当日车端文件的完整镜像**。当前车端是否有未回传修改和最新任务点，尚未核对。

默认任务点保留为空，必须在 GUI 配置；启动系统不会自动执行历史任务。已保存的 AL 算法和执行逻辑保持原样。安装脚本、资产下载和文档是本次整理新增的部署辅助，不新增运动控制服务。

运行时继续使用 AL tmux 名称与接口，不能和已有 AL 实例或 BL 同时占用同一底盘/ROS Domain。底盘驱动保持单独启动，根启动脚本不会接管现有驱动。
