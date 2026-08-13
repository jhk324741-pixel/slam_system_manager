# Phase 8：统一状态、故障恢复与 rosbridge 验证

日期：2026-08-13

适用环境：

- Ubuntu 22.04
- ROS 2 Humble
- 工作空间：`/home/nvidia/fast_lio_ouster_ws`
- 管理包：`slam_system_manager`
- rosbridge WebSocket：TCP 9090

## 1. Phase 8 新增功能

- 新增 `/system/get_status`，同步返回完整 `SystemStatus` 快照。
- `/system/status` 和 `/system/get_status` 复用同一个状态构造函数。
- 新增 `/system/recover`，只允许在 `ERROR` 状态调用。
- Recover 在后台停止残留 Mapping/Localization，不阻塞 Service 回调。
- Recover 不删除地图，不自动重启故障前的 Mapping 或 Localization。
- 传感器健康时恢复到 `WAIT_MODE`；不健康时恢复到 `SENSOR_STARTING`。
- 新增只读 rosbridge 冒烟测试，覆盖断线重连、请求 ID 和超时。
- 新增 SystemManager API 组件测试，自动测试恢复状态和并发请求。

## 2. 编译和自动化测试

```bash
cd /home/nvidia/fast_lio_ouster_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select slam_system_manager --symlink-install
source install/setup.bash
colcon test --packages-select slam_system_manager --event-handlers console_direct+
colcon test-result --verbose
```

Phase 8 开发后的测试基线：

```text
35 tests, 0 errors, 0 failures, 0 skipped
```

其中新增测试覆盖：

- Topic 和 Service 状态关键字段一致。
- 非 `ERROR` 状态拒绝 Recover。
- Recover 停止残留 Mapping 进程。
- 传感器不健康时进入 `SENSOR_STARTING`。
- 传感器健康时进入 `WAIT_MODE`。
- 两个并发 Recover 请求只能接受一个。

## 3. 启动系统

启动前确认没有残留实例：

```bash
ps -eo pid,ppid,pgid,stat,cmd | \
  grep -E '(system_manager|rosbridge|fast_lio|localization)' | grep -v grep
ss -ltnp | grep ':9090' || true
```

启动：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/fast_lio_ouster_ws/install/setup.bash
ros2 launch slam_system_manager system_bringup.launch.py
```

另一个终端检查：

```bash
ros2 node list | grep -E '(system_manager|rosbridge|rosapi)'
ss -ltnp | grep ':9090'
ros2 service list | grep '^/system/' | sort
```

预期只有一个 SystemManager 和一个 rosbridge，9090 只有一个监听者。

## 4. `/system/get_status`

接口：

```text
slam_system_manager/srv/GetSystemStatus
```

定义：

```srv
---
bool success
slam_system_manager/SystemStatus status
string message
```

调用：

```bash
ros2 service call /system/get_status \
  slam_system_manager/srv/GetSystemStatus "{}"
```

预期 `success: true`，`status` 中包含传感器、Mapping、Localization、当前地图和错误字段。

同时读取 Topic：

```bash
ros2 topic echo /system/status --once
```

状态可能随时间变化，但同一稳定状态下的关键字段应一致。

## 5. `/system/recover`

接口：

```text
std_srvs/srv/Trigger
```

调用：

```bash
ros2 service call /system/recover std_srvs/srv/Trigger "{}"
```

非 `ERROR` 状态预期：

```text
success=false
message='Recovery is only valid in ERROR state'
```

`ERROR` 状态调用成功只表示恢复请求已受理：

```text
ERROR -> RECOVERING -> WAIT_MODE
```

传感器仍不健康时：

```text
ERROR -> RECOVERING -> SENSOR_STARTING
```

恢复完成后：

- `current_map` 被清空。
- `error_code` 和 `error_message` 被清空。
- 不自动重新进入 Mapping 或 Localization。
- 地图目录、`map.pcd`、元数据和 Last Pose 不删除。

如果残留进程无法停止，状态返回 `ERROR`，错误码为 `RECOVERY_FAILED`。

## 6. rosbridge 冒烟测试

检查依赖：

```bash
python3 -c 'import websocket; print(websocket.__version__)'
```

如果缺少模块：

```bash
sudo apt update
sudo apt install python3-websocket
```

本机运行源码脚本：

```bash
python3 /home/nvidia/fast_lio_ouster_ws/src/slam_system_manager/tools/rosbridge_smoke_test.py \
  --url ws://127.0.0.1:9090 \
  --timeout 10
```

编译安装后也可运行：

```bash
ros2 run slam_system_manager rosbridge_smoke_test.py \
  --url ws://127.0.0.1:9090
```

局域网另一台主机运行时使用：

```bash
python3 rosbridge_smoke_test.py --url ws://192.168.1.199:9090
```

预期：

```text
[1/6] Connecting to ws://127.0.0.1:9090
[2/6] Subscribing to /system/status
[3/6] Calling /system/get_status
[4/6] Calling /system/get_map_list
[5/6] Disconnecting and reconnecting
[6/6] Confirming status after reconnect
PASS: rosbridge Phase 8 read-only smoke test completed
```

该脚本只读，不创建、删除、启动或停止业务数据。

## 7. rosbridge 业务接口人工测试矩阵

以下测试应在 Ouster 点云和 IMU 健康、系统处于 `WAIT_MODE` 时执行。

### 7.1 地图管理

- 通过 rosbridge 调用 `/system/get_map_list`。
- 创建独立测试地图，重复创建应失败。
- `../bad_map` 等非法名称应失败。
- 不删除正式地图和当前使用地图。
- 测试结束后只删除本次创建的测试地图。

### 7.2 Mapping

- 启动测试地图 Mapping。
- 重复启动必须拒绝。
- Mapping 期间启动 Localization 必须拒绝。
- 分别验证停止不保存和保存地图两条流程。
- 保存成功后检查 `map.pcd` 存在且非空。

### 7.3 Localization

- 使用已确认有效的 PCD 地图启动定位。
- 重复启动必须拒绝。
- Localization 期间启动 Mapping 必须拒绝。
- 通过 rosbridge 调用 `/system/set_initial_pose`。
- 验证 `RELOCALIZING -> LOCALIZED`。
- 验证 `/localization/pose` 和 `/localization/odometry`。
- 停止后确认回到 `WAIT_MODE` 且进程无残留。

### 7.4 ERROR 和 Recover

- 仅在明确测试窗口中触发受控故障。
- 确认 WebSocket 收到正确 `error_code` 和 `error_message`。
- 修复故障条件后通过 rosbridge 调用 `/system/recover`。
- 确认最终状态为 `WAIT_MODE` 或 `SENSOR_STARTING`。
- 确认不会自动重启故障前的算法。

## 8. 停止和残留检查

启动终端按 `Ctrl+C`，随后执行：

```bash
ps -eo pid,ppid,pgid,stat,cmd | \
  grep -E '(system_manager|rosbridge|fast_lio|localization)' | grep -v grep
ss -ltnp | grep ':9090' || true
```

预期由 SystemManager 启动的 rosbridge、Mapping 和 Localization 全部退出。

## 9. Phase 8 验收记录

截至 2026-08-13：

- 编译通过。
- 自动化测试 `35 tests, 0 errors, 0 failures, 0 skipped`。
- `/system/get_status` 和 `/system/recover` 组件测试通过。
- rosbridge 本机 `ws://127.0.0.1:9090` 冒烟测试通过。
- 局域网控制机 `ws://192.168.1.199:9090` 冒烟测试通过。
- 断开并重连后可以继续收到 `/system/status`。
- 实测发现一个 SystemManager、一个 rosbridge 进程，`/system/status` 只有一个发布者。
- SIGINT 停止测试实例后，SystemManager、rosbridge 和 TCP 9090 均无残留。
- Mapping、Localization 和受控 ERROR 全链路测试应在传感器可用的测试窗口执行。
