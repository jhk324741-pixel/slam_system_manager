# Phase 5 Mapping 控制测试命令

本文档仅适用于 Phase 5。所有 service 都采用异步受理语义：service 返回 `accepted: true` 表示请求已经进入后台执行，最终结果应通过 `/system/status`、进程状态和地图文件确认。

## 1. 编译

```bash
cd /home/nvidia/fast_lio_ouster_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select slam_system_manager --symlink-install
source install/setup.bash
```

用途：生成 Phase 5 的 Mapping service 接口并编译 MappingAdapter。

## 2. 启动 SystemManager 和 Ouster

终端 1：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/fast_lio_ouster_ws/install/setup.bash
ros2 launch slam_system_manager system_bringup.launch.py
```

终端 2 使用已经验证的 Ouster 启动命令。确认状态：

```bash
ros2 topic echo /system/status --once
```

启动 Mapping 前必须看到：

```yaml
state: WAIT_MODE
pointcloud_alive: true
imu_alive: true
```

## 3. 启动建图

```bash
ros2 service call /system/start_mapping \
  slam_system_manager/srv/StartMapping \
  "{map_name: phase5_test}"
```

用途：创建地图目录、生成 FAST-LIO 运行时配置并启动 Mapping。

预期立即返回：

```text
accepted=True
message='Mapping start accepted'
```

观察状态：

```bash
ros2 topic echo /system/status
```

预期状态切换：

```text
WAIT_MODE -> MAPPING_STARTING -> MAPPING
```

预期字段：

```yaml
state: MAPPING
mapping_running: true
localization_running: false
current_map: phase5_test
```

## 4. 检查运行时配置

```bash
cat /home/nvidia/slam_maps/phase5_test/mapping_runtime.yaml
```

用途：确认 Adapter 为当前地图生成了独立参数，而没有修改 FAST-LIO 原始 YAML。

重点字段应为：

```yaml
map_file_path: /home/nvidia/slam_maps/phase5_test/map.pcd
pcd_save:
  pcd_save_en: true
```

检查 FAST-LIO 保存服务：

```bash
ros2 service type /map_save
```

预期：

```text
std_srvs/srv/Trigger
```

## 5. 保存并结束建图

```bash
ros2 service call /system/save_map \
  slam_system_manager/srv/SaveMap '{}'
```

用途：进入 `MAP_SAVING`，由 MappingAdapter 调用 `/map_save`，验证非空 `map.pcd`，更新时间戳，随后停止 Mapping。

预期立即返回：

```text
accepted=True
message='Map save accepted'
```

预期状态切换：

```text
MAPPING -> MAP_SAVING -> WAIT_MODE
```

检查结果：

```bash
ls -lh /home/nvidia/slam_maps/phase5_test/map.pcd
cat /home/nvidia/slam_maps/phase5_test/metadata.yaml
ros2 topic echo /system/status --once
```

预期：`map.pcd` 存在且大小大于 0，`metadata.yaml` 的 `updated_at` 已更新，状态为 `WAIT_MODE`、`mapping_running: false`。

## 6. 停止但不保存

先用另一个地图名重新启动 Mapping：

```bash
ros2 service call /system/start_mapping \
  slam_system_manager/srv/StartMapping \
  "{map_name: phase5_discard_test}"
```

进入 `MAPPING` 后执行：

```bash
ros2 service call /system/stop_mapping \
  slam_system_manager/srv/StopMapping '{}'
```

用途：停止 Mapping，但不调用 `/map_save`。该接口用于放弃本次建图。

预期状态：

```yaml
state: WAIT_MODE
mapping_running: false
current_map: ''
```

未保存目录可以删除：

```bash
ros2 service call /system/delete_map \
  slam_system_manager/srv/DeleteMap \
  "{map_name: phase5_discard_test}"
```

## 7. 预期拒绝场景

没有健康 Ouster 数据时调用 `start_mapping`：

```text
accepted=False
message='Mapping requires WAIT_MODE and healthy Ouster data'
```

Mapping 已运行时再次启动：

```text
accepted=False
message='Mapping is already running or an external mapping service exists'
```

使用已有 `map.pcd` 的地图名启动：

```text
accepted=False
message='Refusing to overwrite an existing map.pcd'
```

Mapping 期间调用地图加载、删除或创建接口会被拒绝。

## 8. 异常检查

传感器订阅使用独立的 `Reentrant` callback group。PointCloud2 和 IMU 超时会立即反映到 `/system/status` 的 alive 字段；活跃建图时，只有异常持续超过 `sensor.yaml` 中的 `active_failure_grace_sec` 才进入 `ERROR`。当前默认值为 5 秒，用于过滤短暂调度停顿，同时保留真实断流保护。

```bash
cat /home/nvidia/fast_lio_ouster_ws/src/slam_system_manager/config/sensor.yaml
```

Mapping 运行时停止其外部进程：

```bash
ps -eo pid,ppid,pgid,cmd | grep fastlio_mapping | grep -v grep
```

如果 Mapping 意外退出，SystemManager 在约 0.5 秒内进入：

```yaml
state: ERROR
error_code: MAPPING_PROCESS_EXITED
error_message: Mapping process exited unexpectedly...
```

Mapping 运行时停止 Ouster并持续超过确认时间，系统进入 `ERROR` 并后台停止 Mapping。

第一版尚未提供自动恢复；异常测试后重新启动 SystemManager。

## 9. 单元测试

```bash
cd /home/nvidia/fast_lio_ouster_ws
source /opt/ros/humble/setup.bash
colcon test --packages-select slam_system_manager --event-handlers console_direct+
colcon test-result --verbose
```

用途：运行 FSM、ProcessManager、MapManager 和 MappingAdapter 测试。

预期：4 个测试程序、16 个测试用例全部通过。
