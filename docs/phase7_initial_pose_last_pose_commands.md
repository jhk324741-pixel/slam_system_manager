# Phase 7：Initial Pose 与 Last Pose 功能及测试命令

适用环境：ROS 2 Humble，工作空间 `/home/nvidia/fast_lio_ouster_ws`。

本阶段于 2026-08-12 使用 Ouster 实机数据和 `test_map2` 验证通过。

## 1. 已完成功能

1. 新增 `/system/set_initial_pose` 服务，接收
   `geometry_msgs/PoseWithCovarianceStamped`。
2. 保留 `/initialpose`，兼容 RViz 和 Nav2 常用初始化方式。
3. 校验坐标系、有限数值和四元数，并自动归一化有效四元数。
4. 收到初始位姿后保持 `RELOCALIZING`，不直接报告定位成功。
5. 只有定位算法成功发布 `/map_to_odom` 后，才进入 `LOCALIZED`。
6. 融合 `/map_to_odom` 和 `/Odometry`，统一发布：
   - `/localization/pose`
   - `/localization/odometry`
7. 仅在 `LOCALIZED` 状态下，每 5 秒异步保存一次 `last_pose.yaml`。
8. 再次启动同一地图时自动读取 Last Pose，并尝试恢复定位。
9. Last Pose 使用临时文件加原子重命名，避免写入中断留下半个 YAML。
10. 自动初始化具有稳定等待和有限次数重试，不会无限重复初始化。

状态流转：

```text
RELOCALIZING -> LOCALIZED
LOCALIZED -> 收到新初始位姿 -> RELOCALIZING -> LOCALIZED
```

## 2. 配置

```bash
cat /home/nvidia/fast_lio_ouster_ws/src/slam_system_manager/config/system.yaml
```

Phase 7 相关配置：

```yaml
topics:
  initial_pose: "/initialpose"
  localization_raw_odometry: "/Odometry"
  localization_correction: "/map_to_odom"
  localization_odometry: "/localization/odometry"
  localization_pose: "/localization/pose"

last_pose:
  enable: true
  save_interval_sec: 5.0

relocalization:
  initial_pose_wait_timeout_sec: 5.0
  initial_pose_settle_sec: 6.0
  last_pose_retry_count: 3
  last_pose_retry_interval_sec: 2.0
  min_initial_pose_subscribers: 2
```

## 3. 编译

```bash
cd /home/nvidia/fast_lio_ouster_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select slam_system_manager --symlink-install
source install/setup.bash
```

预期：

```text
Summary: 1 package finished
```

## 4. 启动

确认 Ouster 已发布数据：

```bash
ros2 topic hz /ouster/points
ros2 topic hz /ouster/imu
```

启动管理系统：

```bash
ros2 launch slam_system_manager system_bringup.launch.py
```

另开终端加载环境：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/fast_lio_ouster_ws/install/setup.bash
```

确认系统处于 `WAIT_MODE`：

```bash
ros2 topic echo /system/status --once
```

## 5. 启动定位

```bash
ros2 service call /system/start_localization \
  slam_system_manager/srv/StartLocalization \
  "{map_name: test_map2}"
```

没有 Last Pose 时，等待数秒后预期：

```yaml
state: RELOCALIZING
localization_running: true
current_map: test_map2
```

## 6. 使用 Service 设置初始位姿

下面示例设置地图原点，实际部署时替换位置和四元数：

```bash
ros2 service call /system/set_initial_pose \
  slam_system_manager/srv/SetInitialPose \
  "{pose: {header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}}"
```

预期立即返回：

```text
accepted=True
message='Initial pose published; waiting for localization confirmation'
```

该响应只表示初始位姿已送达算法。ICP 成功后检查：

```bash
ros2 topic echo /system/status --once
```

预期：

```yaml
state: LOCALIZED
localization_running: true
error_code: ''
```

如果 ICP 不匹配，状态继续保持 `RELOCALIZING`，需要提供更准确的初始位姿。

## 7. 使用 RViz 兼容 Topic

RViz 的 `2D Pose Estimate` 会发布 `/initialpose`。命令行等效测试：

```bash
ros2 topic pub -r 1 -t 2 -w 2 \
  /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

`-w 2` 表示等待 SystemManager 和定位算法两个订阅端，避免一次性发布过早退出。

已定位时重新发布，预期日志：

```text
[SYSTEM] LOCALIZED -> RELOCALIZING
[LOCALIZATION] Initial pose received; waiting for scan matching confirmation
[SYSTEM] RELOCALIZING -> LOCALIZED
```

## 8. 检查统一定位输出

```bash
ros2 topic info /localization/pose
ros2 topic info /localization/odometry
ros2 topic echo /localization/pose --once
ros2 topic echo /localization/odometry --once
```

预期：

```text
/localization/pose      geometry_msgs/msg/PoseWithCovarianceStamped
/localization/odometry  nav_msgs/msg/Odometry
```

输出坐标系：

```yaml
header.frame_id: map
child_frame_id: base_link  # Odometry 中
```

`map` 和 `base_link` 名称来自 `system.yaml`，不依赖特定底盘。

## 9. 检查 Last Pose 保存

进入 `LOCALIZED` 后等待超过 `save_interval_sec`：

```bash
sleep 6
cat /home/nvidia/slam_maps/test_map2/last_pose.yaml
```

预期格式：

```yaml
frame_id: map
position:
  x: 0.0
  y: 0.0
  z: 0.0
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0
```

数值以实际定位结果为准。`RELOCALIZING`、`MAPPING` 和 `WAIT_MODE` 不会周期写入。

## 10. 验证 Last Pose 自动恢复

停止定位：

```bash
ros2 service call /system/stop_localization \
  slam_system_manager/srv/StopLocalization "{}"
```

重新启动同一地图：

```bash
ros2 service call /system/start_localization \
  slam_system_manager/srv/StartLocalization \
  "{map_name: test_map2}"
```

不再调用 `set_initial_pose`。等待约 10 至 15 秒：

```bash
ros2 topic echo /system/status --once
```

环境与地图匹配时预期自动进入：

```yaml
state: LOCALIZED
current_map: test_map2
```

日志预期包含：

```text
[LOCALIZATION] Last Pose published for test_map2 (attempt 1/3)
[SYSTEM] RELOCALIZING -> LOCALIZED
```

如果 Last Pose 已失效，有限次数尝试后保持 `RELOCALIZING`，等待人工初始化。

## 11. 错误请求测试

在 `WAIT_MODE` 调用初始位姿服务应被拒绝：

```text
accepted=False
message='Initial pose requires RELOCALIZING or LOCALIZED state'
```

错误 frame 测试：

```bash
ros2 service call /system/set_initial_pose \
  slam_system_manager/srv/SetInitialPose \
  "{pose: {header: {frame_id: odom}, pose: {pose: {orientation: {w: 1.0}}}}}"
```

预期 `accepted=False`，提示初始位姿必须使用 `map` frame。

## 12. 单元测试

```bash
cd /home/nvidia/fast_lio_ouster_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select slam_system_manager --event-handlers console_direct+
colcon test-result --verbose
```

本阶段已验证：

```text
6 个测试程序全部通过
28 tests, 0 errors, 0 failures, 0 skipped
```

新增的 `RelocalizationAdapter` 测试覆盖：

- frame、有限数值和四元数校验
- 非法 Last Pose 路径拒绝
- `map_to_odom × odom_to_base` 位姿融合
- `last_pose.yaml` 原子写入和重新读取
