# 来源与复现边界

整理日期：2026-11-01。SSH 连接车载 `10.30.3.234:22` 超时；本次没有启动、停止、修改车端 AL/BL，也没有取回当日车端修改。

| 内容 | 来源 |
|---|---|
| 导航底座 | 开发本 `Project/chassis_nav_ws`，commit 记录于 source_audit.json |
| AL manager / 门模型 / WebUI / integration | `AL_deployment_20261028/overlay`，包含后续至 10-29 的本机补丁 |
| Livox 驱动 | 官方 `21445540f0d100dc86a7e6df312dd70bbdb4afdf`，展开 ROS2 package.xml/launch |
| FAST-LIO | `2fffc570a25d0df172720bac034fbdb6a13d2162` + AL launch 补丁；ikd-Tree 已展开 |
| 地图 | `map_clear_eastlower_20261029/hkclr_office` |
| 底盘恢复副本 | 10-28 chassis_recovery baseline/source + payload，放在隔离 hardware 工作区 |
| 视频回传 | AL 部署目录 rgb_receiver.py / start_rgb_receiver.sh |
| ONNX | epoch11 原文件，SHA-256 见 assets.json；Release 原样上传 |

`source_audit.json` 对归档中与 10-28 onboard BL 校验记录重叠的文件做逐字节比较。该比较不等于“今日车端已验证”。默认使用的 Nav2 MPPI、定位算法、adapter 等核心源文件可比对；差异列表包含 AL launch、地图、fallback FAST-LIO 参数文件和未使用的备选 Nav2 配置。

FAST-LIO fallback `mid360_filtered.yaml` 由保存的 AL override 恢复 map_en=true，参数含义沿用基线，注释/格式不是 BL 原字节。AL 实际使用 integration/fastlio_navigation.yaml。Livox 通用 MID360_config.json 来自官方源码；AL 使用 bringup 下特定 IP profile，默认不使用此通用文件。

底盘源码不自动运行。当前完整任务 YAML 和当日车端二进制/系统依赖版本没有取回，不能承诺这是可逐字节复现的整机镜像。默认任务模板为空、仅恢复已知参数值；没有伪造车端最新坐标。

## 本次整理排除

密码、SSH 密钥、Git 凭据、运行日志、录像、rosbag、build/install、Python 环境、历史备份、演示 GIF/PDF。第三方许可证和代码版权保留。不包含 BL 启停脚本；历史文档中的 BL 操作仅作原理参考，不是本版启动入口。

`docs/history/` 是历史说明，会包含被后续修改替代的数值（例如 0.47、640×480、0.02 m/s）。以根 README、当前配置和本目录中的 CONFIGURATION.md 为准。
