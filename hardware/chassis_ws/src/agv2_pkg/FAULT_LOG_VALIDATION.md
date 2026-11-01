# 实施与验证记录（234，2026-10-07）

## 备份

位置：/home/hkclr/agv/backups/fault_logger_20261007_162639

- source.tar：完整源码目录，含 .git、未提交改动和原域隔离文件。SHA256 6bb472822dd4784ce2305606d829cc6d14d2c1ef37c978ea9c59d8f38eb1b4a8。
- install.tar：原 install_release_humble。SHA256 654afa36cc31c88ae54809357ac5f76b17895a1cce842c2d282e10dc3fb619bb。
- before.sha256、git_status.txt、working.diff、HEAD、ros_domain.sh。
- HEAD保持0f562453dcce7f6631b02d89f0316eced2f4fd44，不升级develop。
- installation_receipt.json：实际安装文件及前后校验值，安装完成时生成。
- 旧bag未迁移、未删除。实施期间用户仍在运行/启停旧驱动，历史bag数量可自然增加。

## 不变项

控制源码、运动配置、域隔离setup/主机域文件、原安装控制二进制均保持不变。
原二进制SHA256：2c3dd82499cc6dbab3e5e20695ea10aca4830b8d3719691051a3d69489d87f70。
只安装新采集节点及日志脚本/配置；不覆盖在用的agv2_control_node、原ROS消息库或整份安装目录。
未自动调用真实底盘启动/停止、复位、使能、速度/角度控制，也未向真实CAN发送测试查询。
安装后核验：35个安装文件与安装清单哈希一致；非本次变更的源文件和原安装文件全部与备份哈希一致；DOMAIN=76；新采集二进制链接正式install_release_humble消息库；正式continuous目录尚未创建（未启动新记录器）。

## 构建和离线验证

候选源码/构建位于原工作区内 .faultlog_stage_20261007、.faultlog_build_20261007、.faultlog_install_20261007。
候选源码为临时验证副本，不是新的运行工作区。正式代码仍定向更新 src/agv2_pkg。
C++ Humble Release构建成功。原5组gtest：kinematics、can_codec、rate_limiter、teleop、wheel_bringup通过。

新增测试：

- test_fault_logger.py：轮转超过24段、跨段连续序号、符号链接/活动段/引用段保护、容量/磁盘空间、互斥锁、异常尾部保留、两根目录隔离、队列优先级与丢失统计；真实故障组合不误判ROS多源；多源持续/恢复消抖；初始化未收与运行过期区分；0x9A零值/符号/错误位/限频/错峰/发送失败/超时/迟到/回显排除；故障包跨段与持久化确认。
- test_fault_logger_lifecycle.py：INT/TERM/HUP只收尾一次，KILL后保留原始文件并识别异常段，注入写盘错误后暂停、恢复和缺口记录。
- test_fault_logger_ros.py：localhost DOMAIN 93；两个发布者的逐消息GID不同，图端点数量正确；/to_can_bus订阅数为零；Trigger持久化应答和后置采集中语义；TERM完整关闭。
- test_fault_logger_vcan.py：sudo unshare --net隔离命名空间，硬性检查只有lo才创建vcan_test。验证辅助进程CapEff/CapBnd/CapAmb仅0x2000（CAP_NET_RAW）、发送关闭回显仍可捕获TX、回显不算回复、查询频率、暂停、接口断开/恢复。
- 域隔离回归21项（含 --ros、--tmux）：DOMAIN保持76、跨域隔离、四窗格来源、导航入口、错误域拒绝启动、日志预检失败不碰硬件、CAN绑定失败不启动ROS、日志故障不阻止既有硬件停止。
- 同时执行bash语法检查、Python语法检查、安装路径和关键文件哈希检查。

完整命令输出保存于 /home/hkclr/agv/agv_ws/.faultlog_validation_20261007.txt。
虚拟CAN验证输出单独保存于 /home/hkclr/agv/agv_ws/.faultlog_vcan_validation_20261007.txt。
测试未覆盖真实USB物理拔插、真实磁盘耗尽、真实电机反馈恢复或真实整机停机；这些不得通过危险操作自动完成。
磁盘满/错误帧/迟到回复等采用软件注入或模拟。实机静止验收及长时间负载观察仍待现场人员确认安全后完成，不将模拟结果当作实机通过。

## 回退（本次没有执行）

1. 现场确认安全，按原 sudo ./deploy/stop_agv.sh 停止底盘；确认控制节点和记录器均已退出。硬件停机失败时先处理现场安全，不强制杀控制进程来满足回退。
2. 从源码目录执行：
   python3 deploy/rollback_fault_logger.py --confirm-robot-stopped
3. 脚本验证安装后文件未被再次编辑，才恢复原文件；新增文件移除前另备份到原备份目录的rollback_saved_时间。
4. 源.git、所有历史bag、continuous及incidents均不清理。原安装产物未被覆盖，旧入口恢复后仍使用原控制二进制。
5. 若提示文件发生新修改，停止自动回退并人工合并，不运行git reset或整目录覆盖。

回退是需要现场明确执行的操作，不会自动发生。source.tar/install.tar提供完整兜底，不建议在运行中直接解压覆盖。
