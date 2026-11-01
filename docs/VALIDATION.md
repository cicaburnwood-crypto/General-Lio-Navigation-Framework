# 发布验证：2026-11-01

本次只验证归档副本，不启动小车、雷达、相机、定位或导航服务。

- 107 个 Python 文件通过 AST 语法检查。
- 所有归档 shell 脚本通过 `bash -n`。
- 35 项 Python 离线回归通过：门放行逻辑、红蓝点容差、里程计停稳估计、HTTP 配置/地图接口、AL FAST-LIO 进程识别。
- 7 项 JavaScript 测试通过：地图坐标转换/旋转/测距基础、footprint 变换、红蓝点编辑。
- 模型原文件 SHA-256 与训练元数据及 assets.json 一致。
- 地图与源码使用 SOURCE_MANIFEST.sha256 记录文件摘要。
- 发布前扫描未发现密码、GitHub token 或私钥；第三方源码中的 `getent passwd` 是用户信息查询，不是凭据。

Python 离线测试在临时环境运行，未写入车载环境。Pillow 有两条未来 API 弃用警告，不影响当前测试。

完整初次导入的 `git diff --check` 会报告上游源码/历史文档已有的尾部空白；没有为消除这些提示重写第三方源码。此次新写的发布说明、安装脚本和下载脚本通过该检查。

未完成：当前车端源码回读、全工作区 Humble clean build、当日硬件连接/定位/运动实测。这些不能以离线单元测试替代。历史实机验收记录位于 docs/history，只代表当时测试。

## 重跑离线测试

在装好 pytest、aiohttp、Pillow、PyYAML、numpy 的 Python 环境中，从仓库根目录执行：

```bash
export PYTHONPATH="$PWD/src/patrol_manager:$PWD/src/door_opening_server:$PWD/integration:${PYTHONPATH:-}"
python3 -m pytest -q integration/tests/test_gate.py \
  integration/tests/test_blue_tolerance.py integration/tests/test_odom_motion.py \
  integration/tests/test_http.py integration/tests/test_fastlio_restart.py
node integration/tests/test_map_2d.mjs
node integration/tests/test_waypoint_model.mjs
sha256sum -c SOURCE_MANIFEST.sha256
```

其余 `test_ros_*` 依赖 ROS 2 以及隔离模拟服务，不属于本次运行范围。历史 Dockerfile.test 依赖已有私有测试基础镜像；部署请使用 tools/install_dependencies.sh 与 tools/build.sh，不把该测试 Dockerfile 当作可直接拉取的通用镜像。
