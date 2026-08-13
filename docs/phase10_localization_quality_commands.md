# Phase 10：定位质量评估与定位失效检测

## 1. 现有 Localization 分析结论

- 使用包：`fast_lio_localization`，源码位于
  `/home/nvidia/fast_lio_ouster_ws/src/FAST_LIO_LOCALIZATION-ROS-NOETIC`。
- `/Odometry`：`nav_msgs/msg/Odometry`，实测约 10 Hz。pose covariance 来自 FAST-LIO
  EKF；当前 twist 字段为 0，因此质量监控不把 twist 当作真实车速。
- `/map_to_odom`：`nav_msgs/msg/Odometry`，仅在 scan-to-map ICP 被接受后发布，实测约
  0.5 Hz；不是固定高频 topic。
- `/localization/odometry`：`nav_msgs/msg/Odometry`，实测约 10 Hz。
- `/localization/pose`：`geometry_msgs/msg/PoseWithCovarianceStamped`，实测约 10 Hz。
- `global_localization.py` 已计算 ICP fitness、有效 correspondence 数量、source 点数和
  mean residual。Phase 10 只暴露这些已有结果，不改变 ICP 数学逻辑。
- 当前算法没有可直接使用的 map overlap 指标。因此 `map_overlap_ratio` 始终为 NaN，
  overlap 权重不会参与本次置信度归一化。

新增轻量内部 topic：

```text
/localization/registration_quality
slam_system_manager/msg/RegistrationQuality
```

字段来源：`fitness = correspondence_count / source_point_count`；`matching_score` 和
`inlier_ratio` 都明确映射该已有 fitness 值；`registration_residual` 是有效对应点的平均
欧氏距离。没有可用结果时发布 invalid/NaN，不补造数值。

## 2. 输出与状态语义

统一质量 topic：

```bash
ros2 topic info /localization/status -v
ros2 topic hz /localization/status
ros2 topic echo /localization/status --once
```

预期类型为 `slam_system_manager/msg/LocalizationStatus`，频率约 5 Hz。

- `UNKNOWN`：定位未运行、未初始化或数据还不足。
- `LOCALIZED`：连续满足质量条件，位姿可信。
- `DEGRADED`：质量已连续下降，但尚未达到 LOST 条件。
- `LOST`：当前位姿不可信，`pose_valid=false`。

`/system/status` 新增：

```text
localization_quality_state
localization_confidence
localization_pose_valid
localization_quality_reason
```

LOST 不会把整个系统 FSM 置为 ERROR。Localization 进程继续运行，`state` 仍为
`LOCALIZED`，便于 Phase 11 从质量 LOST 进入自动重定位。兼容 Phase 1-9，原有
`/localization/pose` 和 `/localization/odometry` 仍发布；新业务应以 `pose_valid` 作为
可信门控。LOST 期间 Last Pose 不会被保存。

## 3. 判断与置信度

监控器使用以下数据：

1. `/localization/pose` 与 `/localization/odometry` 新鲜度；
2. 连续里程计位置跳变和正确处理 ±pi 的 yaw 跳变；
3. 用 `delta_pose / delta_time` 计算的瞬时线速度、角速度；
4. `/map_to_odom` 更新年龄与修正跳变；
5. Localization 进程、Ouster 点云和 IMU 健康；
6. ICP fitness/inlier ratio、correspondence 数和 mean residual。

置信度对当前存在的 update、motion、registration 指标做加权平均。缺失数据不按 0
处理，而是从分母中排除。硬故障（超时、NaN、时间戳倒退、非物理运动、配准拒绝、
进程或传感器异常）会把当前置信度置 0。

生产初始参数在 `config/localization_quality.yaml`：

```yaml
publish_hz: 5.0
pose_timeout_sec: 0.6
correction_timeout_sec: 5.0
registration_timeout_sec: 5.0
max_position_jump_m: 0.6
max_yaw_jump_deg: 25.0
max_linear_velocity_mps: 3.0
max_angular_velocity_dps: 180.0
max_correction_position_jump_m: 0.75
max_correction_yaw_jump_deg: 30.0
min_registration_fitness: 0.70
max_registration_residual_m: 0.60
bad_frames_to_degraded: 3
bad_frames_to_lost: 10
good_frames_to_recover: 10
localized_threshold: 0.75
degraded_threshold: 0.40
weights: {update: 0.30, motion: 0.25, registration: 0.30, overlap: 0.15}
```

阈值基于实测约 10 Hz 位姿、约 0.5 Hz 配准频率设置。5 Hz 评估下，连续 3 个坏帧进入
DEGRADED，连续 10 个 critical 坏帧进入 LOST；LOST 后连续 10 个好帧才能恢复。配准
本身耗时会增加实际恢复时间。

## 4. 编译与自动化测试

```bash
cd /home/nvidia/fast_lio_ouster_ws
source /opt/ros/humble/setup.bash

python3 -m py_compile \
  src/FAST_LIO_LOCALIZATION-ROS-NOETIC/scripts/global_localization.py

colcon build \
  --packages-select slam_system_manager fast_lio_localization \
  --symlink-install \
  --event-handlers console_direct+

source install/setup.bash
export ROS_DOMAIN_ID=42
colcon test \
  --packages-select slam_system_manager \
  --event-handlers console_direct+ \
  --return-code-on-test-failure
colcon test-result --verbose
unset ROS_DOMAIN_ID
```

预期：8 个测试程序全部通过，51 个测试用例 0 error、0 failure。质量测试覆盖正常连续
位姿、单帧异常、DEGRADED、LOST、迟滞恢复、位置/yaw 跳变、超时、非物理速度、NaN、
时间戳倒退、进程仍活着但位姿不更新、配准拒绝、残差超限和独立故障原因保持。

## 5. 实机基本检查

每个终端先执行：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/fast_lio_ouster_ws/install/setup.bash
```

启动定位并观察：

```bash
ros2 service call /system/start_localization \
  slam_system_manager/srv/StartLocalization \
  "{map_name: test_map}"

ros2 topic echo /localization/status
ros2 topic echo /system/status
```

正常静止实测值：ICP fitness 约 0.996、mean residual 约 0.32 m、confidence 约 0.94，
最终 `LOCALIZED`、`pose_valid=true`。

## 6. 受控 LOST 与恢复测试

此测试只暂停配准 Python 节点，不停止 FAST-LIO，因此能够复现“进程仍在、Odometry
仍发布，但配准结果不再更新”。先解析并核对唯一 PID：

```bash
QUALITY_PID=$(pgrep -f \
  '^python3 /home/nvidia/fast_lio_ouster_ws/install/fast_lio_localization/lib/fast_lio_localization/global_localization.py')
ps -o pid,state,cmd -p "$QUALITY_PID"
```

暂停：

```bash
kill -STOP "$QUALITY_PID"
ros2 topic hz /localization/odometry
ros2 topic echo /localization/status
ros2 topic echo /system/status
```

约 5 至 8 秒后预期：

```yaml
state: LOST
confidence: 0.0
pose_valid: false
reason: Registration quality update timed out
```

同时 `/localization/odometry` 应继续约 10 Hz，`/system/status.state` 仍为
`LOCALIZED`，其 `localization_quality_state` 为 `LOST`。恢复命令：

```bash
kill -CONT "$QUALITY_PID"
ros2 topic echo /localization/status
```

获得新配准结果并连续满足 `good_frames_to_recover` 后，预期恢复到 `LOCALIZED`、
`pose_valid=true`。无论测试是否中途退出，都必须执行 `kill -CONT`。

查看只在状态变化时产生的日志：

```bash
journalctl -u slam_system.service --since "10 minutes ago" --no-pager \
  | grep LOCALIZATION_QUALITY
```

预期出现：

```text
LOCALIZED -> DEGRADED
DEGRADED -> LOST
LOST -> LOCALIZED
```

## 7. 场景 A-H

- A AGV 静止：运行 2 分钟，确认 update_age 稳定小于 0.6 s，无假 LOST。
- B 正常直线：低速直行，确认 linear_velocity 小于平台上限且保持 LOCALIZED。
- C 正常转弯：正常转弯，确认 yaw_jump 使用连续小角度并保持 LOCALIZED。
- D 快速旋转：在安全区域逐步提高角速度，记录真实峰值；超过 180 deg/s 时应先
  DEGRADED，持续异常再 LOST。根据 AGV 实际机械上限调参，不为通过测试而放宽。
- E 部分遮挡 LiDAR：逐步遮挡而非完全断开，预期 fitness 下降，先 DEGRADED；持续低于
  0.70 或配准被拒绝时进入 LOST。
- F 临时停止 Localization 配准：使用第 6 节 SIGSTOP/SIGCONT，验证 Odometry 仍发布。
- G 异常 initial pose：在安全静止状态提交明显偏离地图的位姿：

```bash
ros2 service call /system/set_initial_pose \
  slam_system_manager/srv/SetInitialPose \
  "{pose: {header: {frame_id: map}, pose: {pose: {position: {x: 50.0, y: 50.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}}"
```

  预期 registration `accepted=false`，经过迟滞进入 DEGRADED/LOST；随后提交正确初始
  位姿恢复。
- H 弱结构区：驶入长走廊、空旷区等已知弱结构区域，记录 fitness、残差和置信度，确认
  先 DEGRADED 再 LOST，并据实测样本微调阈值。

测试结束：

```bash
ros2 service call /system/stop_localization \
  slam_system_manager/srv/StopLocalization "{}"
```

预期系统回到 `WAIT_MODE`，质量状态重置为 `UNKNOWN`。

## 8. 外部 Localization 最小改动

`fast_lio_localization` 当前目录不是 Git 仓库，因此重建或替换该源码包时必须保留以下
两处集成：

1. `package.xml` 增加 `<exec_depend>slam_system_manager</exec_depend>`；
2. `scripts/global_localization.py` 导入 `RegistrationQuality`，让
   `registration_at_scale()` 返回已有的 fitness、mean residual、correspondence/source
   数量，并在每次 fine ICP 后发布 `/localization/registration_quality`。

仓库内保留了可复现补丁 `docs/fast_lio_localization_phase10.patch`。在外部包恢复到 Phase 10
实施前版本时，可在该包根目录先执行 `patch --dry-run -p1` 检查，再执行 `patch -p1`。

不得改变 correspondence 判定、ICP 迭代、fitness 阈值或位姿变换计算。构建顺序应同时
选择 `slam_system_manager` 和 `fast_lio_localization`，确保自定义消息在 Python 节点运行
前已安装。

## 9. Phase 11 接入点

Phase 11 直接订阅 `/localization/status`：只在状态稳定为 LOST 且 Localization 进程、
Ouster 健康时触发 `AUTO_RELOCALIZING`。恢复成功必须继续沿用
`good_frames_to_recover`，不能因一次 ICP 成功立刻恢复业务位姿。Phase 10 不实现 Scan
Context、全局重定位或自动重启。
