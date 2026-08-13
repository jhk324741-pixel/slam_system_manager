# Phase 6：Localization 定位控制功能与测试命令

适用环境：

- Ubuntu 22.04
- ROS 2 Humble
- 工作空间：`/home/nvidia/fast_lio_ouster_ws`
- 管理包：`slam_system_manager`
- 地图根目录：`/home/nvidia/slam_maps`

本阶段已于 2026-08-12 使用 Ouster 实机数据和已有 PCD 地图验证通过。

## 1. 已完成功能

1. 提供 `/system/start_localization`，根据地图名启动定位。
2. 验证地图名称、地图元数据和非空 `map.pcd`。
3. 从 `process.yaml` 读取定位命令并替换 `{map_name}`、`{map_path}`。
4. 等待定位进程稳定以及 `/pcd_map`、`/Odometry` 发布端就绪。
5. 提供 `/system/stop_localization`，停止完整定位进程组。
6. Mapping 与 Localization 双向互斥，禁止同时运行。
7. 定位进程异常退出时进入 `ERROR`，错误码为
   `LOCALIZATION_PROCESS_EXITED` 或 `LOCALIZATION_START_FAILED`。
8. Ouster 持续掉线时停止定位并进入 `ERROR`。
9. `/system/status` 输出定位运行状态和当前地图。

正常状态流转：

```text
WAIT_MODE -> LOCALIZATION_STARTING -> RELOCALIZING
RELOCALIZING -> stop_localization -> WAIT_MODE
```

Phase 6 不处理初始位姿，因此定位正常启动后停在 `RELOCALIZING`。初始位姿、
`LOCALIZED` 状态和 Last Pose 属于 Phase 7。

## 2. 加载环境

每个新终端都需要执行：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/fast_lio_ouster_ws/install/setup.bash
```

用途：加载 ROS 2 Humble 和当前工作空间中的消息、服务及可执行程序。

## 3. 启动前检查

确认 Ouster 数据正常：

```bash
ros2 topic hz /ouster/points
ros2 topic hz /ouster/imu
```

确认已有地图及 PCD 非空：

```bash
find /home/nvidia/slam_maps -maxdepth 2 -name map.pcd -type f -printf '%p %s bytes\n'
```

检查定位命令配置：

```bash
sed -n '/localization:/,/rosbridge:/p' \
  /home/nvidia/fast_lio_ouster_ws/src/slam_system_manager/config/process.yaml
```

当前定位命令：

```text
ros2 launch fast_lio_localization localization_ouster64.launch.py map:={map_path} rviz:=false
```

## 4. 启动 SystemManager

```bash
ros2 launch slam_system_manager system_bringup.launch.py
```

另开终端检查状态：

```bash
ros2 topic echo /system/status --once
```

启动定位前预期：

```yaml
state: WAIT_MODE
pointcloud_alive: true
imu_alive: true
mapping_running: false
localization_running: false
error_code: ''
```

## 5. 查看可用地图

```bash
ros2 service call /system/get_map_list \
  slam_system_manager/srv/GetMapList "{}"
```

用途：获取可供 Web、Qt 或命令行选择的地图名称。

## 6. 启动定位

将 `test_map2` 替换为实际地图名：

```bash
ros2 service call /system/start_localization \
  slam_system_manager/srv/StartLocalization \
  "{map_name: test_map2}"
```

预期 Service 响应：

```text
accepted=True
message='Localization start accepted'
```

Service 使用异步受理语义。等待数秒后检查最终状态：

```bash
ros2 topic echo /system/status --once
```

预期：

```yaml
state: RELOCALIZING
pointcloud_alive: true
imu_alive: true
mapping_running: false
localization_running: true
current_map: test_map2
error_code: ''
```

## 7. 检查定位输出

```bash
ros2 topic info /pcd_map
ros2 topic hz /Odometry
ros2 topic hz /cloud_registered
```

实机验证结果：

- `/pcd_map` 有 1 个 Publisher，地图成功加载。
- `/Odometry` 持续发布，实测约 `9.5~10 Hz`。
- `/cloud_registered` 持续发布，实测约 `6~7 Hz`。
- 定位日志显示 `Waiting for initial pose from RViz`，Phase 6 中属于正常现象。

查看定位相关进程：

```bash
ps -eo pid,ppid,pgid,stat,cmd | \
  grep -E '(localization_ouster64|global_localization|map_to_odom|pcd_to_pointcloud)' | \
  grep -v grep
```

## 8. 验证互斥与重复启动保护

定位运行期间再次启动定位：

```bash
ros2 service call /system/start_localization \
  slam_system_manager/srv/StartLocalization \
  "{map_name: test_map3}"
```

预期：

```text
accepted=False
message='Localization is already running'
```

定位运行期间启动建图：

```bash
ros2 service call /system/start_mapping \
  slam_system_manager/srv/StartMapping \
  "{map_name: localization_conflict_test}"
```

预期：

```text
accepted=False
message='Localization process is running'
```

## 9. 停止定位

```bash
ros2 service call /system/stop_localization \
  slam_system_manager/srv/StopLocalization "{}"
```

预期立即返回：

```text
accepted=True
message='Localization stop accepted'
```

等待数秒后检查：

```bash
ros2 topic echo /system/status --once
```

预期：

```yaml
state: WAIT_MODE
mapping_running: false
localization_running: false
current_map: ''
error_code: ''
```

确认没有定位残留进程：

```bash
ps -eo pid,ppid,pgid,stat,cmd | \
  grep -E '(localization_ouster64|global_localization|map_to_odom|pcd_to_pointcloud)' | \
  grep -v grep
```

预期没有输出。

## 10. 无效请求测试

地图不存在：

```bash
ros2 service call /system/start_localization \
  slam_system_manager/srv/StartLocalization \
  "{map_name: missing_map}"
```

非法路径：

```bash
ros2 service call /system/start_localization \
  slam_system_manager/srv/StartLocalization \
  "{map_name: ../../etc/passwd}"
```

两者都应返回 `accepted=False`，且系统保持 `WAIT_MODE`。

## 11. 单元测试

```bash
cd /home/nvidia/fast_lio_ouster_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select slam_system_manager --event-handlers console_direct+
colcon test-result --verbose
```

已验证结果：

```text
5 个测试程序全部通过
24 tests, 0 errors, 0 failures, 0 skipped
```

其中包含 19 个核心 GTest 用例，其余为测试包装统计项。

## 12. Python 节点执行权限故障

检查：

```bash
ros2 pkg executables fast_lio_localization
ls -l /home/nvidia/fast_lio_ouster_ws/src/FAST_LIO_LOCALIZATION-ROS-NOETIC/scripts/*.py
```

如果 launch 报 `global_localization.py not found on the libexec directory`，执行：

```bash
chmod 755 \
  /home/nvidia/fast_lio_ouster_ws/src/FAST_LIO_LOCALIZATION-ROS-NOETIC/scripts/global_localization.py \
  /home/nvidia/fast_lio_ouster_ws/src/FAST_LIO_LOCALIZATION-ROS-NOETIC/scripts/map_to_odom_tf.py \
  /home/nvidia/fast_lio_ouster_ws/src/FAST_LIO_LOCALIZATION-ROS-NOETIC/scripts/publish_initial_pose.py \
  /home/nvidia/fast_lio_ouster_ws/src/FAST_LIO_LOCALIZATION-ROS-NOETIC/scripts/transform_fusion.py
```

该命令只修改节点文件的执行权限，不修改定位算法内容。
