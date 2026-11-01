# 安装与启动

## 环境

Ubuntu 22.04 x86_64、ROS 2 Humble、Python 3.10；MID360 通过网线连接，D435i RGB 通过 UVC/V4L2。门模型使用 ONNX Runtime CPU，默认不启用深度，不依赖 GPU。开发本当前 ROS Jazzy 不能直接替代这套 Humble 环境。

```bash
cd ~/AL-framework
python3 tools/fetch_assets.py  # 需要 gh 登录仓库所属账号
bash tools/install_dependencies.sh
bash tools/build.sh
```

安装脚本只安装依赖、SDK、Python 环境；构建脚本不启动硬件。原有 ROS 工作区不要 source 到构建终端中，尤其不要 source BL。源码已展开，包括 FAST-LIO 的 ikd-Tree，不需要再次 `vcs import` 覆盖本版文件。

Livox SDK2 使用固定 commit，见脚本；历史车端 SDK 二进制版本未取回，本版没有宣称完整二进制复现。`rosdep` / apt 仍跟随 Humble 软件源，不是全系统依赖锁文件。

## 配置硬件

1. 根据实际网卡配置雷达网段；当前 profile 的主机接收地址是 `192.168.1.50`，雷达为 `192.168.1.118`。不要只改雷达 IP 忽略 host_net_info。
2. 检查 `src/bringup/config/lidar_profiles.yaml`、`src/bringup/config/livox/*.json`。默认选单个 `front_118`，不是自动双雷达融合。保留的外参来自原部署，换硬件需使用实际安装值。
3. 相机路径在 `integration/d435i_rgb.py` 的 `device` 参数。通过 `ls -l /dev/v4l/by-path/` 查找实际 RGB 设备；换 USB 插孔后路径可能变化。相机进程支持 ROS 参数覆盖。
4. 根启动链路固定 ROS Domain **76**。底盘必须处于同一 domain。

## 系统启停

```bash
bash start_al.sh
tmux attach -t AL
# 浏览器访问：http://<小车主机IP>:8092/#patrol
bash stop_al.sh
```

AL 会话包含 `stack`、`camera`、`web`；日志为 `logs/stack.log`、`camera.log`、`web.log`。启动不发送巡检目标；停止脚本只关闭 AL，会等待 ROS 退出，最后关闭剩余 AL PTY。不会停止外部 `agv_control` 或 BL。

若已有 AL 同名会话，启动脚本不会创建第二套。不要同时运行原 AL/BL 与本仓库副本，否则雷达端口、ROS topic 和速度接口冲突。

开发本访问可用局域网直连，或：

```bash
ssh -N -L 18925:127.0.0.1:8092 hkclr@10.30.3.234
# http://127.0.0.1:18925/#patrol
```

不要用 `file://.../web/index_2d.html` 打开；WebUI 依赖 HTTP API 和 `/static` 路径。

## 底盘链路

根启动脚本有意不启动 CAN 底盘。已存在的车载原生底盘服务继续作为 `/cmd_vel` 的接收端。`hardware/chassis_ws/src/agv2_pkg` 是独立恢复源码（基线 + 2026-10-28 payload），不属于 Nav2 工作区构建，不会由 AL 自动启动。

需要在新主机重建该驱动时，参考该包的 `DEPLOYMENT.md`、`DOMAIN_ISOLATION.md`：

```bash
bash hardware/chassis_ws/src/agv2_pkg/deploy/install_dependencies_ubuntu22_humble.sh
mkdir -p ~/.config/hkclr
printf 'export ROS_DOMAIN_ID=76\n' > ~/.config/hkclr/ros_domain.sh
bash hardware/chassis_ws/src/agv2_pkg/deploy/build_humble.sh
# 以下命令会初始化 CAN 并启动硬件驱动；仅在确实要接通底盘时运行：
sudo bash hardware/chassis_ws/src/agv2_pkg/deploy/start_agv.sh \
  "$(pwd)/hardware/chassis_ws/src/agv2_pkg/config/chassis.yaml"
```

此硬件副本没有从当前车端重新验证，不能把其配置默认值当作今日运行参数。原有底盘已经在运行时，不要再执行这组启动命令。

## GUI 操作

1. 打开页面，等待雷达、RGB、ROS 状态。
2. 在地图选“定位参考点”，填写位置/朝向，再点开始定位。参考位姿只是匹配初值；程序重启 AL FAST-LIO，等待新里程计/点云，再调用 VGICP 定位服务。
3. 巡检页配置红/蓝点，保存后手动启动；实时页配置内存队列，确认后直接执行，不写 YAML。
4. 红点到达后 manager 等待朝向/停稳，再每次间隔至少一秒调用门检测，返回成功后发下一点。
5. 原生 RGB 录像需手动开始/停止；巡检完成、取消或重定位不会自动停止录像。停止相机/整套系统会终止采集。

## 录像回传

```bash
# 在开发本运行；不需要 ROS
python3 tools/desktop/rgb_receiver.py \
  --url http://10.30.3.234:8092 --output "$HOME/Downloads/AL_RGB"
```

完成录像存于车端 `recordings/rgb/`；接收程序只下载已封装 MP4，验证长度和 SHA-256 后确认回传。使用上述前台命令可显式选择目的目录。`start_rgb_receiver.sh` 保留原用户级 systemd 辅助方式，未自动安装或启用。
