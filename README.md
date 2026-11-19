# AL-2

AL-2 是 2026-11-19 部署到 `.175` 的 ROS 2 Humble 激光导航版本。它用 MID360 和 FAST-LIO 提供点云及里程计，用 VGICP 配准 3D 地图，再由 Nav2 的 SmacPlanner2D、全向 MPPI 和速度平滑器控制底盘。WebUI 提供 2D 地图、定位、巡检编辑和实时任务。

| 项目 | `.175` 部署值 |
| --- | --- |
| MID360 | `192.168.1.119`，配置 `left_front_119` |
| 主控雷达网口 | `enp87s0`，`192.168.1.50/24` |
| 雷达相对 `base_link` | 前 0.20 m、左 0.16 m、上 0.10 m，RPY 为零 |
| 底盘 | 全向，`can_chassis`，1 Mbit/s |
| MPPI 速度上限 | 前向 0.35 m/s、横向 0.15 m/s、转向 0.45 rad/s |
| ROS 域及速度入口 | 域 `0`，`/cmd_vel`，`geometry_msgs/msg/Twist` |
| 地图 | `hkclr_office`，2D 分辨率 0.05 m，另有同版本 3D PCD |

## 安装

预计环境为 x86-64、Ubuntu 22.04、ROS 2 Humble。已安装在实验设备上的原生底盘 SDK 由 AL-2 复用。默认路径为 `~/wg/hkclr_mobile_robot_sdk`，需要其中已有可用的 `hkclr_mobile_base` ROS 包和 `chassis.yaml`。Livox-SDK2 的头文件和共享库可通过系统路径提供，也可分别放到 AL-2 的 `.deps/livox/include` 和 `.deps/livox/lib`。这两项依赖须在主控单独安装。

在机器人主控上执行以下命令。先确认下列克隆目录可用。在已部署的车辆上，可将 Git 副本克隆到新目录并用 `AL_ROOT` 指向它，保留运行中的部署目录。

```bash
git clone --branch AL-2 --single-branch \
  https://github.com/cicaburnwood-crypto/General-Lio-Navigation-Framework.git \
  "$HOME/AL_20261119_dual"
cd "$HOME/AL_20261119_dual"
cp -n integration/robot.env.example integration/robot.env

sudo apt update
sudo apt install tmux python3-colcon-common-extensions python3-rosdep \
  python3-aiohttp python3-pil python3-yaml libpcl-dev libapr1-dev \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-spatio-temporal-voxel-layer ros-humble-ros2-socketcan
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
bash integration/build_onboard.sh
```

在首次使用 `rosdep` 的镜像上先执行 `sudo rosdep init`。构建脚本分别安装 Python manager 和 C++ 导航工作区。离线镜像可以提前安装依赖，或将兼容 Ubuntu 22.04/Python 3.10 的 Python wheels 放入 `wheels/`。

`integration/robot.env` 选择雷达 profile、WebUI 绑定地址和 ROS 域。更换主控 IP 时同时更新其中的 `AL_WEB_HOST`，启动时设置 `AL_EXPECTED_HOST` 为同一个 IP。底盘已有其他本机 ROS 域或 CAN 参数时，应先核对现有底盘部署参数。仓库也保留 `.155` 的 `hkclr_mr_2` 标定及全向参数，`start_al_175.sh` 的网口、雷达 IP 和底盘 SDK 默认值具体对应 `.175`。

## 一键启动与远端访问

在机器人主控上执行：

```bash
cd "$HOME/AL_20261119_dual"
./start_al_175.sh --check
./start_al_175.sh
```

若项目克隆在其他位置，运行时可用 `AL_ROOT="$PWD" ./start_al_175.sh`。可用 `SDK_ROOT` 指定现场原生底盘 SDK 的安装位置。在已经部署的 `.175` 上也可直接运行 `~/start_al_175.sh`。

脚本检查雷达网口、专用 IP 和路由、底盘 CAN 及波特率，再启动原生底盘和 AL。如果底盘驱动已经运行，则复用它。直接接入实机前，结束同时持续向 `/cmd_vel` 发指令的 VR 或手动遥控会话。随后脚本检查实际雷达、IMU、里程计、轮驱反馈、速度话题和 WebUI。启动时巡检保持空闲。

- 同一局域网的操作电脑打开 [AL-2 操作页面](http://10.30.3.175:8092/#patrol)。
- 如果浏览器不能直接访问，在**运行浏览器的电脑**执行下面的 SSH 命令，保留终端连接，再打开[本机转发页面](http://127.0.0.1:18930/#patrol)。

```bash
ssh -NT -L 127.0.0.1:18930:10.30.3.175:8092 hkclr@10.30.3.175
```

绑定地址是本机回环接口。如果本地端口已被占用，可将命令中的第一个端口换为 `18931`，浏览器地址也使用该端口。

## 定位与运行巡检

1. 打开「定位」，在地图上选机器人当前大致位置，拖动设朝向，或填入 X、Y、Z、Yaw。平面测试 Z 可填 0，Yaw 单位是弧度。
2. 点击「以参考位姿开始定位」。AL 重启 Fast-LIO，等待新进程送来里程计和点云，再用参考位姿进行 VGICP 粗配准和精配准。等页面显示实时地图位姿，再启动任务。已知参考位姿时也可在主控执行 `./start_al_175.sh --pose X Y YAW`。
3. 在「巡检任务」里依次在图上选点，设置点类型、朝向和顺序。发布分支的巡检队列初始为空。点击「保存任务」会写入 `src/patrol_manager/config/door_patrol.yaml`，点击「保存并启动巡检」才会派发目标。
4. 蓝点收到 Nav2 到达结果后直接派发下一点。红点到达后固定等待 10 秒，界面会显示倒计时，等待期间取消将清掉后续派发。当前版本使用雷达导航，红点无需相机或门检测。
5. 若只想临时走一组点，使用「实时模式」，定位成功后选点并确认。该队列仅保存在本次运行的内存中。需要中断时点「取消 manager 任务」。

## 停止与故障排查

停止 AL 的雷达、导航、巡检 manager 和 WebUI：

```bash
cd "$HOME/AL_20261119_dual"
bash stop_al.sh
```

如果原生底盘是由一键脚本启动，可在 AL 停止后用 `tmux send-keys -t AL-175-chassis:0.0 C-c` 关闭该会话。由其他系统启动的底盘使用对应的原有停止流程。定位重启只影响 Fast-LIO，整套 AL 重启后须重新确认地图定位。

| 现象 | 检查方法 |
| --- | --- |
| MID360 无载波 | 用 `ip -br link show enp87s0` 查看网口，检查网线两端和雷达供电。网口有载波后确认 `ip route get 192.168.1.119` 指向 `enp87s0`，源地址为 `192.168.1.50`。 |
| 网口已连但缺点云或 IMU | 查 `logs/stack.log` 的 Livox SDK 日志，确认驱动选的是 `left_front_119`。驱动配置含点云与 IMU 的 UDP 端口。 |
| 缺少 `can_chassis` 或轮驱未就绪 | 检查底盘 USB-CAN、供电和 `ip -details link show can_chassis`。正常总线应为 `ERROR-ACTIVE`，再查 `logs/chassis.log`。 |
| 里程计正常，Nav2 仍是 `inactive` | 尚缺 `map → base_link` 或定位没有收敛。先检查 WebUI 的定位状态及日志中的配准结果。 |
| Nav2 报 `Starting point in lethal space` | 核对初始位姿、实时点云和车体附近是否有障碍，检查地图与现场是否对齐。 |
| 车上网页正常，本机页面掉线 | 测 `curl --noproxy '*' http://10.30.3.175:8092/api/status`。如果有返回，恢复浏览器电脑上的 SSH 转发即可。 |

`./start_al_175.sh --check` 只做硬件预检。缺设备或雷达网络配置异常会逐条打印具体项目并返回 2。运行阶段如果点云、里程计、轮驱反馈或 WebUI 缺失，脚本会打印缺失项并返回 3。有明确参考位姿时，定位步骤失败会返回 4。

## 地图与源码

默认地图位于 `src/bringup/maps/hkclr_office/`。2D YAML 引用同目录的 PGM，VGICP 使用 `3D/map.pcd`。两张地图与车上验证过的 AL 地图校验值相同，见 [地图校验文件](MAPS.sha256)。更换场地时要同时准备同一坐标系下的 2D 栅格和 3D PCD，并同步更新导航启动与 WebUI 的默认地图设置。

该分支包含 AL-2 源码、当前 WebUI、雷达配置、原有 AL 地图及部署脚本。现场巡检记录、运行日志、凭据、视频、构建产物和底盘 SDK 外部依赖由现场保管。许可证保留原组件声明，见 [第三方与许可](THIRD_PARTY_NOTICES.md)。
