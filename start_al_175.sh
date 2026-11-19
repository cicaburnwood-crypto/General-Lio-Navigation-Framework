#!/usr/bin/env bash
# Start the .175 hardware driver and the existing AL launch.
set -eo pipefail

AL_ROOT="${AL_ROOT:-$HOME/AL_20261119_dual}"
SDK_ROOT="${SDK_ROOT:-$HOME/wg/hkclr_mobile_robot_sdk}"
BASE_CONFIG="$SDK_ROOT/ros/src/hkclr_mobile_base/config/chassis.yaml"
BASE_SESSION=AL-175-chassis
CAN_IF=can_chassis
LIDAR_IF=enp87s0
LIDAR_IP=192.168.1.119
LIDAR_HOST=192.168.1.50
WEB_HOST="${AL_EXPECTED_HOST:-10.30.3.175}"

fail() { printf '[错误] %s\n' "$*" >&2; exit 1; }

check_only=false
case "${1:-}" in
  '') ;;
  --check)
    [[ $# == 1 ]] || fail '用法：start_al_175.sh --check'
    check_only=true
    shift ;;
  --pose)
    [[ $# == 4 ]] || fail '用法：start_al_175.sh --pose X Y YAW（米、弧度）'
    shift
    /usr/bin/python3 -c 'import math,sys; sys.exit(not all(math.isfinite(float(x)) for x in sys.argv[1:]))' "$@" 2>/dev/null ||
      fail '定位参考位姿须为有限数值。'
    ;;
  *) fail '用法：start_al_175.sh [--check | --pose X Y YAW]' ;;
esac

[[ -f "$AL_ROOT/start_al.sh" && -r "$AL_ROOT/integration/robot.env" ]] ||
  fail "找不到 AL 或 integration/robot.env：$AL_ROOT"
grep -Fxq "AL_WEB_HOST=$WEB_HOST" "$AL_ROOT/integration/robot.env" ||
  fail 'AL 的机器人配置与 .175 不匹配。'
grep -Fxq 'AL_ROS_DOMAIN_ID=0' "$AL_ROOT/integration/robot.env" ||
  fail '.175 AL 必须与原生底盘使用同一 ROS 域 0。'

missing=0
report() { printf '[硬件或连接异常] %s\n' "$*" >&2; missing=1; }
if ! ip link show dev "$LIDAR_IF" >/dev/null 2>&1; then
  report "MID360：主控未识别雷达网口 $LIDAR_IF。"
elif [[ "$(cat "/sys/class/net/$LIDAR_IF/carrier" 2>/dev/null)" != 1 ]]; then
  report "MID360：$LIDAR_IF 无物理链路，请检查雷达供电和以太网线两端。"
elif ! ip -o -4 addr show dev "$LIDAR_IF" | grep -qF "$LIDAR_HOST/24"; then
  report "MID360：$LIDAR_IF 缺少 $LIDAR_HOST/24，请检查已有的 Wired connection 1。"
elif ! ip -4 route get "$LIDAR_IP" 2>/dev/null | grep -qF "dev $LIDAR_IF src $LIDAR_HOST"; then
  report "MID360：$LIDAR_IP 没有走 $LIDAR_IF，检查雷达专用网络配置。"
fi
if ! ip link show dev "$CAN_IF" >/dev/null 2>&1; then
  report "底盘：未识别 $CAN_IF USB-CAN 适配器。"
fi
(( missing == 0 )) || exit 2

can_info() { ip -details link show dev "$CAN_IF"; }
info="$(can_info)"
if grep -q 'can state STOPPED' <<<"$info"; then
  if $check_only; then
    fail "底盘 $CAN_IF 已识别但未启用。"
  fi
  if sudo -n true 2>/dev/null; then
    sudo -n /usr/sbin/ip link set dev "$CAN_IF" up type can bitrate 1000000 ||
      fail "无法启用 $CAN_IF。"
  elif [[ -t 0 ]]; then
    sudo /usr/sbin/ip link set dev "$CAN_IF" up type can bitrate 1000000 ||
      fail "无法启用 $CAN_IF。"
  else
    fail "底盘 $CAN_IF 已识别但未启用，请从本机终端运行脚本以完成 sudo。"
  fi
  info="$(can_info)"
fi
grep -q 'can state ERROR-ACTIVE' <<<"$info" ||
  fail "底盘 $CAN_IF 总线异常：$(awk '/can state/ {print $3}' <<<"$info")。"
grep -q 'bitrate 1000000' <<<"$info" || fail "底盘 $CAN_IF 波特率应为 1 Mbit/s。"
printf '[通过] 雷达网口 %s 有载波，底盘 %s 为 ERROR-ACTIVE。\n' "$LIDAR_IF" "$CAN_IF"
$check_only && exit 0

[[ -r "$SDK_ROOT/ros/install/local_setup.bash" && -r "$BASE_CONFIG" ]] ||
  fail "缺少原生底盘安装或配置：$SDK_ROOT"
[[ -r "$AL_ROOT/install/local_setup.bash" ]] || fail 'AL 的 ROS 安装未找到。'
mkdir -p "$AL_ROOT/logs/ros" "$AL_ROOT/.ros"
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
export ROS_HOME="$AL_ROOT/.ros" ROS_LOG_DIR="$AL_ROOT/logs/ros"

if ! tmux has-session -t "=$BASE_SESSION" 2>/dev/null; then
  if pgrep -u "$(id -u)" -f '[/]opt/ros/humble/bin/ros2 launch hkclr_mobile_base mobile_base.launch.py' >/dev/null; then
    printf '[启动] 已有原生底盘启动进程，复用。\n'
  else
    printf -v base_command 'cd %q; source /opt/ros/humble/setup.bash; source ros/install/local_setup.bash; export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 ROS_HOME=%q ROS_LOG_DIR=%q; exec ros2 launch hkclr_mobile_base mobile_base.launch.py interface:=%q params_file:=%q >>%q 2>&1' \
      "$SDK_ROOT" "$ROS_HOME" "$ROS_LOG_DIR" "$CAN_IF" "$BASE_CONFIG" "$AL_ROOT/logs/chassis.log"
    tmux new-session -d -s "$BASE_SESSION" "$base_command" || fail '无法启动原生底盘。'
  fi
fi
bash "$AL_ROOT/start_al.sh" || fail 'AL 启动失败，请查看 logs/stack.log 和 logs/web.log。'
printf '[访问] 在同一局域网的电脑浏览器打开 http://%s:8092/#patrol\n' "$WEB_HOST"
printf '[访问] 如需 SSH 转发，在浏览器所在电脑运行：\n  ssh -NT -L 127.0.0.1:18930:%s:8092 %s@%s\n' "$WEB_HOST" "$(id -un)" "$WEB_HOST"
printf '[访问] 保持 SSH 命令运行，然后打开 http://127.0.0.1:18930/#patrol。若本机已在使用该转发，可直接打开。\n'
source /opt/ros/humble/setup.bash
source "$AL_ROOT/install/local_setup.bash"

# Wait for actual sensor data and driver feedback before reporting the chain ready.
/usr/bin/python3 - "http://$WEB_HOST:8092" "$@" <<'PY'
import collections
import json
import sys
import time
import urllib.error
import urllib.request

import rclpy
from lifecycle_msgs.srv import GetState
from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String

url, *seed = sys.argv[1:]
opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
points = collections.Counter()
diag = {}
topics = [
    ('/livox/lidar', CustomMsg, 'MID360 没有点云'),
    ('/livox/imu', Imu, 'MID360 没有 IMU 数据'),
    ('/sensors/cloud', PointCloud2, 'AL 没有 FAST-LIO 转换后的点云'),
    ('/odom', Odometry, 'FAST-LIO 没有底盘坐标系里程计'),
]

def read_status():
    try:
        with opener.open(url + '/api/status', timeout=.8) as response:
            status = json.load(response)
        if status.get('patrol_status') is not None:
            return status
    except (OSError, ValueError):
        pass
    return {}

def record_diag(message):
    diag.update(json.loads(message.data))

def motors_ready():
    return all(diag.get(k) for k in ('front_ready', 'rear_ready', 'steer_fb'))

def nav_lifecycle(node):
    client = node.create_client(GetState, '/bt_navigator/get_state')
    if not client.wait_for_service(timeout_sec=1):
        return 'unavailable'
    future = client.call_async(GetState.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=2)
    return future.result().current_state.label if future.done() else 'unavailable'

def verify(node):
    for topic, msg_type, _ in topics:
        node.create_subscription(msg_type, topic, lambda _, t=topic: points.update([t]),
                                 qos_profile_sensor_data)
    node.create_subscription(String, '/mobile_base/diagnostics', record_diag, 10)
    deadline = time.monotonic() + 32
    status = {}
    next_poll = 0.
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.10)
        now = time.monotonic()
        if now >= next_poll:
            status = read_status()
            next_poll = now + .7
        if status and motors_ready() and all(points[topic] >= 3 for topic, _, _ in topics):
            break
    missing = [f'{error}（{topic}）' for topic, _, error in topics if points[topic] < 3]
    if not diag:
        missing.append('底盘没有驱动诊断（/mobile_base/diagnostics），检查 CAN 和原生底盘进程')
    elif not motors_ready():
        for key, part in [('front_ready', '前轮'), ('rear_ready', '后轮'), ('steer_fb', '转向')]:
            if not diag.get(key):
                missing.append(f'底盘 {part}未就绪，检查 CAN 反馈和电机供电')
    if not status:
        missing.append('AL WebUI 或巡检 manager 没有响应')
    connected = [(p.node_name, p.topic_type)
                 for p in node.get_subscriptions_info_by_topic('/cmd_vel')]
    if ('mobile_base_node', 'geometry_msgs/msg/Twist') not in connected:
        missing.append('原生底盘未订阅 ROS 域 0 的 /cmd_vel')
    publishers = {p.node_name for p in node.get_publishers_info_by_topic('/cmd_vel')}
    if 'velocity_smoother' not in publishers:
        missing.append('Nav2 velocity_smoother 未发布 /cmd_vel')
    if missing:
        for error in missing:
            print('[启动错误]', error, file=sys.stderr)
        return 3

    print('[通过] MID360 点云、IMU、FAST-LIO 点云和里程计均有数据。')
    print('[通过] 前后轮和转向反馈就绪，Nav2 到底盘的 /cmd_vel 已接通。')
    print('[通过] WebUI', url + '/#patrol', '巡检状态', status['patrol_status']['state'])
    if seed:
        body = dict(zip(('x','y','yaw'), map(float, seed)), z=0., action='relocalize')
        req = urllib.request.Request(url + '/api/action', data=json.dumps(body).encode(),
                headers={'Content-Type':'application/json', 'X-Scan-Nav':'operator'})
        try:
            with opener.open(req, timeout=68) as response:
                print('[定位]', json.load(response)['result']['message'])
        except urllib.error.HTTPError as error:
            print('[定位错误]', error.read().decode(errors='replace'), file=sys.stderr)
            return 4
        except OSError as error:
            print('[定位错误]', error, file=sys.stderr)
            return 4
        deadline = time.monotonic() + 25
        while time.monotonic() < deadline:
            status = read_status()
            if status.get('pose') is not None and nav_lifecycle(node) == 'active':
                break
            time.sleep(.5)
    if status.get('pose') is not None and nav_lifecycle(node) == 'active':
        print('[通过] 地图定位有效，Nav2 已激活。')
        return 0
    print('[待定位] Nav2 当前状态：', nav_lifecycle(node))
    if seed:
        print('[定位错误] 未在时限内得到有效地图位姿和激活的 Nav2。', file=sys.stderr)
        return 4
    print('[待定位] 请在 WebUI 的定位页给出参考位姿，或以 --pose X Y YAW 启动。')
    return 0

rclpy.init(args=[])
node = rclpy.create_node('al_175_startup_check')
try:
    result = verify(node)
finally:
    node.destroy_node()
    rclpy.shutdown()
sys.exit(result)
PY
