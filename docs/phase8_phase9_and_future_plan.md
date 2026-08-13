# Phase 8、Phase 9 实施方案与后续开发计划

日期：2026-08-12

适用环境：

- Ubuntu 22.04
- ROS 2 Humble
- 工作空间：`/home/nvidia/fast_lio_ouster_ws`
- 管理包：`slam_system_manager`
- 当前开发设备用户：`nvidia`

本文档用于下一次继续开发。实施时仍按阶段进行：先完成并验收 Phase 8，确认后再进入 Phase 9，不将两个阶段混在一次修改中。

## 1. 当前基线

Phase 1 至 Phase 7 已完成，当前自动化测试基线为：

```text
28 tests, 0 errors, 0 failures, 0 skipped
```

当前已提供：

- `/system/status`
- 地图创建、删除、加载和列表 Service
- Mapping 启动、停止和保存 Service
- Localization 启动和停止 Service
- `/system/set_initial_pose`
- `/initialpose`
- `/localization/pose`
- `/localization/odometry`

当前 rosbridge 已在 `config/process.yaml` 中配置为自动启动，默认监听 TCP 9090。

当前尚缺少原始需求中的两个统一接口：

- `/system/get_status`
- `/system/recover`

Phase 8 应先补齐这两个接口，再做完整 rosbridge 验证。

## 2. 明天开始前的基线检查

### 2.1 登录并加载环境

```bash
ssh nvidia@<DEVICE_IP>
source /opt/ros/humble/setup.bash
source /home/nvidia/fast_lio_ouster_ws/install/setup.bash
cd /home/nvidia/fast_lio_ouster_ws
```

设备 IP 可能由 DHCP 改变，应先确认当前地址。

### 2.2 检查代码和测试基线

```bash
colcon build --packages-select slam_system_manager --symlink-install
colcon test --packages-select slam_system_manager --event-handlers console_direct+
colcon test-result --verbose
```

预期仍为 28 项测试通过。若基线失败，应先处理基线问题，不直接开始 Phase 8。

### 2.3 检查残留进程和 9090 端口

```bash
ps -eo pid,ppid,pgid,stat,cmd | \
  grep -E '(system_manager|rosbridge|fast_lio|localization)' | grep -v grep
```

```bash
ss -ltnp | grep ':9090' || true
```

用途：避免手动 rosbridge、旧 SystemManager 和新 SystemManager 同时运行。一个系统只能保留一个 rosbridge 监听 9090。

### 2.4 检查 Ouster 数据

```bash
ros2 topic hz /ouster/points
ros2 topic hz /ouster/imu
```

Phase 8 的正常业务测试需要两路数据健康。异常恢复测试会单独模拟掉线，不应一开始就关闭雷达。

# Phase 8：rosbridge 接口补齐与全链路验证

## 3. Phase 8 目标

1. 补齐 `/system/get_status`。
2. 补齐 `/system/recover`。
3. 保证 Web/Qt 只使用 `/system/status` 和 `/system/*`，不直接执行 launch、kill 或参数修改。
4. 验证 rosbridge 能订阅状态、调用全部系统 Service，并正确返回异步受理结果。
5. 验证断线重连、非法请求、模式互斥、ERROR 恢复和 Service 超时行为。
6. 输出可重复执行的 rosbridge 冒烟测试脚本和 Phase 8 命令文档。

## 4. Phase 8 预计修改文件

新增：

```text
slam_system_manager/srv/GetSystemStatus.srv
slam_system_manager/test/test_system_manager_api.cpp
slam_system_manager/tools/rosbridge_smoke_test.py
slam_system_manager/docs/phase8_rosbridge_commands.md
```

修改：

```text
slam_system_manager/CMakeLists.txt
slam_system_manager/package.xml
slam_system_manager/include/slam_system_manager/system_manager.hpp
slam_system_manager/src/system_manager.cpp
slam_system_manager/config/process.yaml
```

如冒烟测试脚本需要安装，还应在 `CMakeLists.txt` 中增加 `install(PROGRAMS ...)`。

## 5. Phase 8.1：实现 `/system/get_status`

建议新增：

```srv
---
bool success
slam_system_manager/SystemStatus status
string message
```

实现要求：

- 新增一个构造状态快照的公共内部方法，例如 `buildStatusMessage()`。
- `/system/status` 定时发布和 `/system/get_status` 必须复用同一份状态构造逻辑。
- 获取 `current_state`、`current_map`、错误和传感器状态时使用同一把互斥锁保护。
- Service 只读取状态，不执行耗时任务。
- rosbridge 返回的字段必须与同一时刻 `/system/status` 字段一致。

命令行测试：

```bash
ros2 service call /system/get_status \
  slam_system_manager/srv/GetSystemStatus "{}"
```

预期：返回 `success: true`，并包含完整 `SystemStatus`。

## 6. Phase 8.2：实现 `/system/recover`

第一版使用标准接口即可：

```text
std_srvs/srv/Trigger
```

推荐恢复语义：

1. 只允许从 `ERROR` 接受恢复请求。
2. `ERROR -> RECOVERING`。
3. 在后台线程中停止可能残留的 Mapping 或 Localization 进程组。
4. 清理当前 SLAM 会话状态，但不删除地图文件。
5. 不自动重启之前的 Mapping 或 Localization。
6. 重新读取当前传感器健康状态。
7. 雷达健康时：`RECOVERING -> WAIT_MODE`。
8. 雷达未健康时：`RECOVERING -> SENSOR_STARTING`，等待数据恢复。
9. 恢复成功后清空 `error_code` 和 `error_message`。
10. 恢复失败时重新进入 `ERROR`，使用 `RECOVERY_FAILED`。

当前 FSM 已允许：

```text
ERROR -> RECOVERING
RECOVERING -> WAIT_MODE
RECOVERING -> SENSOR_STARTING
RECOVERING -> ERROR
```

Service 回调不得等待进程停止完成。它只返回是否接受，实际完成状态通过 `/system/status` 观察。

命令行测试：

```bash
ros2 service call /system/recover std_srvs/srv/Trigger "{}"
```

正常状态下调用应拒绝，不改变状态。`ERROR` 下调用应被接受，随后观察：

```text
ERROR -> RECOVERING -> WAIT_MODE
```

或在雷达仍离线时：

```text
ERROR -> RECOVERING -> SENSOR_STARTING
```

## 7. Phase 8.3：rosbridge 启动与配置检查

启动管理器：

```bash
ros2 launch slam_system_manager system_bringup.launch.py
```

检查节点和端口：

```bash
ros2 node list | grep -E '(system_manager|rosbridge|rosapi)'
ss -ltnp | grep ':9090'
```

预期：

- 只有一个 `system_manager`。
- 只有一个 rosbridge WebSocket Server。
- 9090 只有一个监听者。
- 日志包含 `Rosbridge WebSocket server started on port 9090`。
- 不再出现 `Address already in use`。

`process.yaml` 中继续保留：

```yaml
call_services_in_new_thread:=true
default_call_service_timeout:=60.0
send_action_goals_in_new_thread:=true
```

## 8. Phase 8.4：编写 rosbridge 冒烟测试脚本

建议使用 Python `websocket-client` 直接发送 rosbridge JSON 协议，不依赖 Web 页面。

先检查依赖：

```bash
python3 -c 'import websocket; print(websocket.__version__)'
```

如果未安装：

```bash
sudo apt update
sudo apt install python3-websocket
```

脚本建议参数：

```text
--url ws://127.0.0.1:9090
--timeout 10
--map-name test_map2
```

脚本至少验证：

1. WebSocket 握手成功。
2. 订阅 `/system/status` 并在 3 秒内收到消息。
3. 状态消息包含全部必需字段。
4. 调用 `/system/get_status`。
5. 调用 `/system/get_map_list`。
6. 比较 Topic 状态和 Service 状态的关键字段。
7. 断开连接后重新连接，再次收到状态。
8. 每次 Service 使用唯一 `id`，并匹配对应 `service_response`。
9. 超时后明确失败，不无限等待。

rosbridge 订阅消息示例：

```json
{
  "op": "subscribe",
  "id": "status-sub-1",
  "topic": "/system/status",
  "type": "slam_system_manager/msg/SystemStatus"
}
```

Service 调用示例：

```json
{
  "op": "call_service",
  "id": "map-list-1",
  "service": "/system/get_map_list",
  "args": {}
}
```

运行方式：

```bash
python3 /home/nvidia/fast_lio_ouster_ws/src/slam_system_manager/tools/rosbridge_smoke_test.py \
  --url ws://127.0.0.1:9090
```

远程 Web/Qt 主机测试时，把 `127.0.0.1` 改成设备当前 IP。

## 9. Phase 8.5：业务接口测试矩阵

### 9.1 只读接口

- 订阅 `/system/status`，频率约 2 Hz。
- 调用 `/system/get_status`。
- 调用 `/system/get_map_list`。
- 检查 JSON 中布尔值、浮点值、空字符串和数组类型正确。

### 9.2 地图接口

- 创建合法测试地图。
- 重复创建同名地图应失败。
- `../bad_map` 等非法名称应失败。
- 加载存在且 PCD 有效的地图应成功。
- 删除测试地图后，地图列表应同步更新。
- 不删除正式地图和正在使用的地图。

### 9.3 Mapping 接口

- `WAIT_MODE` 且雷达健康时启动 Mapping。
- 重复启动 Mapping 应拒绝。
- Mapping 运行时启动 Localization 应拒绝。
- 测试停止但不保存。
- 单独创建一张测试地图验证保存，确认非空 `map.pcd` 后再删除测试数据。

### 9.4 Localization 接口

- 使用已有地图启动 Localization。
- 重复启动应拒绝。
- Localization 运行时启动 Mapping 应拒绝。
- 通过 rosbridge 调用 `/system/set_initial_pose`。
- 确认 `RELOCALIZING -> LOCALIZED`。
- 检查 `/localization/pose` 和 `/localization/odometry`。
- 停止定位后确认回到 `WAIT_MODE` 且无残留进程。

### 9.5 ERROR 与 Recover

- 在测试模式下停止传感器数据或终止受管理的测试进程，触发明确 `ERROR`。
- 确认 WebSocket 状态中包含正确 `error_code` 和 `error_message`。
- 恢复传感器或修复故障条件后，通过 rosbridge 调用 `/system/recover`。
- 确认不会自动恢复到 Mapping 或 Localization。
- 确认最终进入 `WAIT_MODE` 或 `SENSOR_STARTING`。

## 10. Phase 8 自动化测试要求

新增单元或组件测试应覆盖：

- `get_status` 和发布 Topic 共用同一状态快照逻辑。
- 非 `ERROR` 状态拒绝 Recover。
- `ERROR -> RECOVERING` 合法。
- 恢复时有残留进程能够异步停止。
- 传感器健康和不健康两种恢复结果。
- 并发请求不会同时启动 Mapping 与 Localization。
- Recover 期间其他模式切换请求被拒绝。

执行：

```bash
cd /home/nvidia/fast_lio_ouster_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select slam_system_manager --symlink-install
source install/setup.bash
colcon test --packages-select slam_system_manager --event-handlers console_direct+
colcon test-result --verbose
```

要求：原有 28 项测试继续通过，新增测试全部通过。

## 11. Phase 8 验收标准

满足以下条件才算 Phase 8 完成：

- `/system/get_status` 和 `/system/recover` 已实现并记录接口定义。
- rosbridge 只启动一份，9090 无冲突。
- 本机和局域网远端均能连接 `ws://<DEVICE_IP>:9090`。
- WebSocket 能持续接收 `/system/status`。
- 全部 `/system/*` Service 可通过 rosbridge 正确调用。
- 异步请求不会阻塞 rosbridge 主线程。
- 非法地图、重复启动和模式冲突能返回清晰错误。
- ERROR 恢复流程验证通过，且不会自动疯狂重启算法。
- rosbridge 断线重连后无需重启 SystemManager。
- 自动化测试全部通过。
- 新增 `docs/phase8_rosbridge_commands.md`，记录功能、命令和预期结果。

# Phase 9：systemd 开机自启动

## 12. Phase 9 前置问题

当前 `process.yaml` 中 Ouster 配置为：

```yaml
command: "ros2 launch ouster_ros sensor.launch.xml sensor_hostname:={sensor_hostname} viz:=false"
auto_start: false
```

这适合当前人工单独启动 Ouster 的开发方式，但不满足最终“开机后由系统管理器启动 Ouster”的目标。启用 systemd 前必须选择并实现以下一种方式：

### 推荐方式：SystemManager 管理 Ouster

- 在 YAML 中配置实际 `sensor_hostname`。
- 将该值作为 ProcessManager 命令模板替换变量。
- 部署配置中把 Ouster `auto_start` 设为 `true`。
- 开发配置仍可设为 `false`，便于人工启动驱动。
- SystemManager 启动时防止与外部 Ouster 重复启动。

### 备选方式：Ouster 使用独立 systemd 服务

- `slam_system.service` 只检测 Topic，不管理 Ouster 进程。
- 必须为两个 systemd Unit 声明正确依赖关系。
- 这种方式会把进程控制拆散，不是当前首选。

明天进入 Phase 9 前，应先从现有 Ouster 配置确认真实 hostname，禁止猜测或写入示例地址。

## 13. Phase 9 目标

1. 提供非交互启动脚本 `start_slam_system.sh`。
2. 提供 `slam_system.service`。
3. systemd 启动时正确加载 ROS 2 和工作空间环境。
4. 开机后自动启动 SystemManager、rosbridge，并按部署配置启动 Ouster。
5. 支持优雅停止和 `Restart=on-failure`。
6. 日志统一进入 journald。
7. 验证正常重启、异常退出、网络延迟和传感器未就绪场景。

## 14. Phase 9 预计修改文件

建议在源码中保存可追踪模板：

```text
slam_system_manager/scripts/start_slam_system.sh
slam_system_manager/systemd/slam_system.service
slam_system_manager/docs/phase9_systemd_commands.md
```

根据 Ouster 管理方式，可能还需修改：

```text
slam_system_manager/config/system.yaml
slam_system_manager/config/process.yaml
slam_system_manager/include/slam_system_manager/config_manager.hpp
slam_system_manager/src/config_manager.cpp
slam_system_manager/src/system_manager.cpp
slam_system_manager/CMakeLists.txt
```

## 15. 启动脚本设计

建议源码模板：

```bash
#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash
source /home/nvidia/fast_lio_ouster_ws/install/setup.bash

cd /home/nvidia/fast_lio_ouster_ws
exec ros2 launch slam_system_manager system_bringup.launch.py
```

要求：

- 使用 Bash，不依赖 `.bashrc`。
- `source` 写在脚本中，不直接写进 systemd 的 `ExecStart`。
- 最后一行使用 `exec`，让 systemd 正确跟踪主进程。
- 不在脚本中使用 `sudo`。
- 不后台运行命令，不使用 `&`。
- 配置路径如需覆盖，应通过 launch 参数传入绝对路径。

部署脚本：

```bash
sudo install -Dm755 \
  /home/nvidia/fast_lio_ouster_ws/src/slam_system_manager/scripts/start_slam_system.sh \
  /home/nvidia/scripts/start_slam_system.sh
```

## 16. systemd Unit 设计

建议内容：

```ini
[Unit]
Description=ROS 2 SLAM System Manager
Wants=network-online.target
After=network-online.target

[Service]
Type=simple
User=nvidia
Group=nvidia
WorkingDirectory=/home/nvidia/fast_lio_ouster_ws
Environment=HOME=/home/nvidia
Environment=ROS_DOMAIN_ID=0
ExecStart=/home/nvidia/scripts/start_slam_system.sh
Restart=on-failure
RestartSec=5
KillSignal=SIGINT
KillMode=control-group
TimeoutStopSec=30

[Install]
WantedBy=multi-user.target
```

设计说明：

- `network-online.target` 只表示系统网络基本就绪，不能保证 Ouster 已开始发包，因此 Sensor HealthMonitor 仍必须处理等待状态。
- `KillSignal=SIGINT` 让 ROS 2 launch 和子节点优雅退出。
- `KillMode=control-group` 确保 systemd 最终能处理同一服务 cgroup 内的残留子进程。
- `TimeoutStopSec=30` 到期后由 systemd 执行最终终止。
- `Restart=on-failure` 只对异常退出重启；执行 `systemctl stop` 不应自动拉起。
- 不设置无限高速重启，应保留 `RestartSec`，必要时再增加 StartLimit。

安装 Unit：

```bash
sudo install -Dm644 \
  /home/nvidia/fast_lio_ouster_ws/src/slam_system_manager/systemd/slam_system.service \
  /etc/systemd/system/slam_system.service
```

## 17. systemd 操作命令

```bash
sudo systemctl daemon-reload
sudo systemctl enable slam_system.service
sudo systemctl start slam_system.service
```

查看状态：

```bash
systemctl status slam_system.service --no-pager
```

持续查看日志：

```bash
journalctl -u slam_system.service -f
```

查看本次开机日志：

```bash
journalctl -u slam_system.service -b --no-pager
```

停止和重启：

```bash
sudo systemctl stop slam_system.service
sudo systemctl restart slam_system.service
```

禁止开机启动：

```bash
sudo systemctl disable slam_system.service
```

## 18. Phase 9 测试矩阵

### 18.1 非交互环境测试

```bash
sudo -u nvidia env -i \
  HOME=/home/nvidia PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  /home/nvidia/scripts/start_slam_system.sh
```

用途：验证脚本不依赖交互终端和用户 `.bashrc`。

### 18.2 启动测试

- 启动 Unit 后 `systemctl is-active` 返回 `active`。
- `/system/status` 约 2 Hz。
- rosbridge 9090 正常监听且只有一个实例。
- 部署配置启用 Ouster 管理时，驱动只启动一个实例。
- 雷达数据正常后状态进入 `WAIT_MODE`。

检查命令：

```bash
systemctl is-active slam_system.service
ros2 topic hz /system/status
ros2 topic echo /system/status --once
ss -ltnp | grep ':9090'
```

### 18.3 优雅停止测试

```bash
sudo systemctl stop slam_system.service
```

随后检查：

```bash
ps -eo pid,ppid,pgid,stat,cmd | \
  grep -E '(system_manager|rosbridge|fast_lio|localization|os_driver)' | grep -v grep
```

预期：由该 Unit 启动的进程全部退出，不残留 rosbridge、Mapping 或 Localization。

### 18.4 异常重启测试

- 记录 SystemManager 主进程 PID。
- 只在测试窗口内模拟主进程异常退出。
- 约 5 秒后确认 systemd 拉起新进程。
- 确认不会留下旧 rosbridge 造成 9090 冲突。
- 检查重启后地图文件没有被修改或删除。

不要对生产中的建图保存过程直接执行强制终止测试。

### 18.5 传感器未就绪测试

- 启动服务时让 Ouster 暂时不可用。
- Unit 应保持运行，系统状态应为 `SENSOR_STARTING`，而不是由 systemd 无限重启。
- Ouster 恢复后应自动进入 `WAIT_MODE`。
- 该场景不应被错误判断为 SystemManager 进程启动失败。

### 18.6 重启设备验收

```bash
sudo reboot
```

设备重新上线后：

```bash
systemctl is-enabled slam_system.service
systemctl is-active slam_system.service
journalctl -u slam_system.service -b --no-pager
```

从上位机检查：

- 9090 可以连接。
- `/system/status` 可以订阅。
- Ouster 健康后进入 `WAIT_MODE`。
- 地图列表与重启前一致。
- Last Pose 文件仍存在。

## 19. Phase 9 验收标准

- 启动脚本可在干净非交互环境运行。
- Unit 可启动、停止、重启和 enable。
- `ExecStart` 中没有直接使用 `source`。
- 开机后 SystemManager 和 rosbridge 自动启动。
- 最终部署模式下 Ouster 启动策略明确且无重复实例。
- 正常停止无残留子进程。
- 主进程异常退出后按 5 秒间隔恢复。
- 雷达离线不会触发 systemd 高频重启。
- journald 中能看到状态切换和错误日志。
- 完成真实 reboot 验证。
- 新增 `docs/phase9_systemd_commands.md`。

# 后续开发计划

## 20. Phase 10：恢复策略和故障诊断增强

目标：在保持第一版保守策略的前提下，提高现场可维护性。

- 细化 ERROR 分类和恢复前置条件。
- 增加错误发生时间、进程退出码和恢复次数。
- 对 Ouster、Mapping、Localization 设置独立且有上限的重启策略。
- 使用退避时间，禁止无限快速重启。
- 增加诊断 Topic，可选适配 `diagnostic_msgs`。
- 保存最近一次关键错误，但不让高频日志刷盘。

## 21. Phase 11：Localization 质量状态

目标：不能只根据进程存在和 `/map_to_odom` 判断定位长期可信。

- 定义 `/localization/status`。
- 增加最后一次有效定位时间、定位频率和数据新鲜度。
- 从现有算法可用输出中提取匹配分数或收敛标志。
- 设置质量阈值和持续时间，避免瞬时抖动切换状态。
- 定位失效时停止报告 `LOCALIZED` 并给出明确原因。
- 不在该阶段重新实现全局定位算法。

## 22. Phase 12：平台适配和 TF 验证

目标：正式支持不同 AGV/AMR 的部署参数。

- 使用每台设备的实测标定值配置 `lidar_to_base`。
- 验证 `map -> odom -> base_link` TF 链。
- 检查 TF 时间戳、重复发布者和 Frame 冲突。
- 为不同平台建立独立部署 YAML，不修改通用默认配置。
- 增加启动时 Frame 和外参配置检查。
- 验证不依赖特定底盘驱动和 `cmd_vel`。

## 23. Phase 13：Web/Qt 上位机

目标：基于稳定接口实现实际操作界面。

- 系统状态和错误展示。
- 地图列表、创建、删除和选择。
- 建图启动、保存和停止。
- 定位启动、停止和人工初始位姿。
- 操作中状态、Service 异步受理和最终状态分离展示。
- WebSocket 断线自动重连和请求超时。
- 前端不得直接执行 shell、launch、kill 或 `ros2 param set`。

## 24. Phase 14：安全和网络部署

目标：避免 rosbridge 9090 在非可信网络中裸露。

- 限制 rosbridge 监听地址或使用防火墙限制来源。
- 跨网络访问时增加反向代理、TLS 和认证。
- 为上位机定义允许调用的接口白名单。
- 检查地图名、字符串长度和异常大消息。
- 规划 API 版本兼容策略。
- 不直接把 9090 暴露到公网。

## 25. Phase 15：长期运行和交付验证

目标：达到可部署、可复现和可维护状态。

- 8 至 24 小时 Ouster 与状态监控稳定性测试。
- Mapping、Localization 多轮切换测试。
- 断网、雷达重连、算法崩溃和磁盘空间不足测试。
- Last Pose 长期保存和掉电一致性测试。
- CPU、内存、磁盘写入和温度监控。
- RK3588 目标机重新编译和性能验证。
- 固化依赖版本、部署清单、配置备份和回滚步骤。

## 26. 推荐执行顺序

```text
Phase 8  接口补齐和 rosbridge 联调
  -> Phase 8 单独验收
  -> Phase 9 systemd 和 Ouster 开机启动策略
  -> Phase 9 重启设备验收
  -> Phase 10 故障恢复增强
  -> Phase 11 定位质量状态
  -> Phase 12 多平台与 TF 验证
  -> Phase 13 Web/Qt 上位机
  -> Phase 14 网络安全
  -> Phase 15 长期运行和 RK3588 交付测试
```

每个 Phase 完成后继续记录：

1. 修改文件。
2. 完成功能。
3. 编译命令。
4. 运行命令。
5. 测试命令。
6. 预期输出。
7. 实际测试结果和未解决问题。

未经当前 Phase 验收，不直接进入下一 Phase。
