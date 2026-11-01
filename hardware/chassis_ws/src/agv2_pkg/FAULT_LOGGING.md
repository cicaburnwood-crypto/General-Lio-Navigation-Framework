# 234 底盘故障记录器

## 生效范围与安全边界

仅 deploy 整机入口启用。运动学、使能、速度控制、故障复位、底盘参数和 DOMAIN 76 不变。
直接 ros2 run/launch 控制节点不自动获得本机制；原有快照功能仍保留在 launch 中，deploy 明确传 enable_fault_logging:=false，避免两个 /agv2/save_fault_log 服务。

本次安装不停止或重启真实机器人，不替换原有控制可执行文件。下次现场人员按原流程正常停止、再启动时启用新机制。先确认安全区域、物理急停及动力状态。日志并非安全控制器，不会自动清错、使能或恢复运动。

## 原有启停命令保持不变

在 /home/hkclr/agv/agv_ws/src/agv2_pkg 目录：

```bash
sudo ./deploy/start_agv.sh "$PWD/deploy/chassis_full_manual_test.yaml"
sudo ./deploy/stop_agv.sh
```

原启动脚本要求 sudo；本次没有增加额外 sudo 要求。启动脚本返回后，记录器仍独立运行，不以启动 shell 的 PID 判断结束。
不要用 kill 启动脚本代替硬件停止：底盘使用 tmux/ROS launch 运行，安全停止仍需 stop_agv.sh。
停机先暂停新加的查询，再执行原 /agv2/prepare_shutdown、tmux 关闭和 CAN down；失败则维持被动记录，不擅自停硬件。
正常停止保留两秒尾部，日志关闭最多等待十秒。日志调用异常不会阻止硬件停止，停机前日志IPC调用最多等待0.5秒。

## 日志位置与保留

根目录 /home/hkclr/agv/agv_ws/data/chassis_faults：

- continuous：24段，每段600秒或1GiB，总上限12GiB，磁盘最低剩余10GiB。
- incidents：前60秒、后30秒；重叠故障合并，最晚延长到首次触发后120秒。
- 故障包最多20个、7天、4GiB；单包最多1GiB。达到任一限制清理最早的已关闭包。
- 历史bag目录保持原样，旧bag分析脚本仍可用。
- 四小时只是正常流量时的持续日志窗口，不是空间不足或频繁重启时的保证。
- 仅清理经过模块标记、名称、普通文件校验的关闭段；不跟随符号链接，不删除当前段、被读取/导出的段或无关文件。
- 每200ms刷新、每1秒fsync；kill -9、掉电、磁盘故障不保证零丢失。再次启动标记异常段，但不修补原始JSONL。
- 空间不足或写盘失败暂停写盘，5秒重试；恢复记录缺失区间。运行中日志故障只告警，不改变机器人控制。

每段四文件：manifest.json（配置、完整性、覆盖范围）、state.jsonl、can.jsonl、events.jsonl。
目录使用 segment_时间_会话ID_序号 命名；故障包也沿用该命名，manifest.incident.id 是包ID。
记录有会话、序号、系统时间和单调时钟。采集时间、内核时间另保留，不能把旧反馈当当前状态。
故障包可能早停、缺前置、丢记录或截断，请检查 manifest.incident 和 queue_statistics，不能只看存在目录就断言完整。
发生瞬时写入故障时，无法确认落盘的批次会计入可能缺失范围，不会悄悄重复写入。

## 查询、标记、汇总与导出

```bash
python3 scripts/chassis_fault_log.py status
python3 scripts/chassis_fault_log.py save --reason "遥控变化但舵轮停止"
source deploy/setup_agv_env.bash
ros2 service call /agv2/save_fault_log std_srvs/srv/Trigger '{}'
python3 scripts/chassis_fault_log.py summarize --path /完整路径/故障包目录
python3 scripts/chassis_fault_log.py export --path /完整路径/已关闭故障包目录 --destination /home/hkclr/incident_export_唯一名称.tar.gz
journalctl -t agv234-faultlog --since "10 minutes ago"
```

status / summarize 只读，不启动查询、不发CAN。export 只复制已关闭且验证通过的目录，目标必须在 chassis_faults 管理目录之外且不存在。
save/Trigger 成功表示请求时可用证据和事件清单已经持久化；返回 post_capturing 表示仍在收集后置窗口，不是完整结束。
无法及时确认持久化时返回失败/待确认，可能仍会产生包，避免误称成功。离线汇总容忍并统计末尾残缺行。
run 是底层监督进程接口，需要 --config、--params、--root、--interface、--collector；正常现场应使用 deploy 入口，以获得完整的权限和生命周期保障。

## 采集和权限

独立C++节点订阅 /joy、/cmd_vel、/wheel_command、/agv2/chassis_telemetry、/Battery_SOC_STATE、/agv2/log_event，以及过滤的 /rosout 和 /parameter_events。
逐消息记录发布者GID和RMW时间戳，周期保存端点名称、数量及QoS。GID用于区分发布端，不据节点名猜测远端IP；主机位置不确定则 unknown。
不订阅 /to_can_bus；仅查询其发布端图信息。采集节点没有控制发布者。

辅助进程单独临时获取 CAP_NET_RAW；Python系统文件不设置永久capability，没有 CAP_NET_ADMIN，不配置接口。
SocketCAN记录RX/错误/本机回显；AF_PACKET记录PACKET_OUTGOING，以覆盖当前发送器关闭回显的情况。
tx_observed 仅表示本机发送路径可见，不代表电机已收到或执行。local_tx_echo 不当电机回复，两个观察通道不去重伪装成一次总线交互。

默认读取实际启动YAML的can.steer_front_id / steer_rear_id，仅发送8字节9A00000000000000。
两路各1Hz、错开500ms、等待200ms，无立即重试。查询日志包括本地send结果、TX路径观察、回复、超时、失败累计及最后有效回复。
协议没有事务号：回复按CAN ID和时间窗关联，标记 time_and_id_only_no_wire_transaction_id；无法绝对排除另一个诊断客户端同ID同指令的回复。
迟到/无对应请求的回复单独标记，不假装及时成功。温度、电压、电流、开启状态和错误原值均保存；bit0低压、bit3过温，其余保留。
轮毂603F的0006仍是轮毂CANopen错误码，不套用舵机错误位。
接口不可用、日志不健康、采集节点失效、暂停或停机时不发查询。
CAN辅助进程失效不会自动以提权方式重启；监督器/监视器告警，现场下次安全启停恢复。
内核CAN/USB事件通过当前用户的journal权限读取；没有权限时明确记录不可用。USB日志标记为系统观察，须结合接口映射判断是否对应底盘。

## 自动故障判定

保存轮毂fault/非零错误、曾使能后掉使能、已见反馈后轮毂>500ms/舵机>300ms过期、启动10秒缺初始反馈、活跃模式手柄>500ms无消息、
CAN不可用/错误计数新增、USB断连、控制进程退出、tmux消失、关键控制话题多发布者持续1秒、诊断查询连续3次超时。
同一条件只触发一次，恢复稳定5秒后重新布防。普通CAN周期发送和rosout多个发布者不是控制多源冲突证据。
进程退出无法获取退出码时只记录身份消失，不虚构退出码；ROS采集进程是监督器子进程，其退出码可直接记录。

## 验证与现场验收

自动化测试只使用临时目录、替代进程、localhost DOMAIN 93及独立网络命名空间中的vcan_test。
不得直接把测试接口改为真实can0。可参考 test/test_fault_logger*.py、test/test_domain_isolation.py 和原有gtest。
本次构建、具体测试结果、保护校验及回退备份见 FAULT_LOG_VALIDATION.md。

现场确认后，保持车体安全静止，按原命令启动；检查打印的记录器PID、目录、can_bound和健康状态。
静止检查舵机9A回复/超时、手柄零值与GID、轮毂反馈、保存服务、正常停止包的覆盖范围及CPU/磁盘负载。
不要为了测试主动触发运动、拔动力线、清错或用kill -9验证实车安全停止。

