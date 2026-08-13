# 通用三维激光建图定位控制器项目总结

日期：2026-08-12

## 1. 项目目的

本项目用于开发一套运行在 Ubuntu 22.04 + ROS 2 Humble 平台上的“通用三维激光建图定位控制器”。当前使用 Jetson 和 Ouster 三维激光雷达进行开发验证，后续目标平台包括 RK3588，以及不同型号的 AGV、AMR 和移动机器人。

系统不重新实现 Ouster 驱动、FAST-LIO 建图或现有定位算法，而是在它们之上增加统一的系统管理层 `slam_system_manager`，负责：

- 统一启动和停止 Ouster、Mapping、Localization 与 rosbridge。
- 检测点云和 IMU 数据是否正常，并统计近似频率。
- 管理建图、地图保存、地图列表和已有地图定位流程。
- 保证 Mapping 和 Localization 互斥运行。
- 管理初始位姿、Last Pose 和统一定位输出。
- 通过 `/system/status` 和 `/system/*` Service 为 Web/Qt 上位机提供稳定接口。
- 通过 YAML 适配不同雷达、算法启动命令、Topic、Frame 和机器人平台。

系统不依赖特定底盘，不直接发布 `cmd_vel`，也不把轮速里程计作为必要输入。平台相关配置和算法差异由配置文件及 Adapter 层隔离。

## 2. 当前工程环境

- 工作空间：`/home/nvidia/fast_lio_ouster_ws`
- 系统管理包：`slam_system_manager`
- Ouster 驱动包：`ouster_ros`
- 建图包：`fast_lio`
- 定位包：`fast_lio_localization`
- Web 通信：`rosbridge_server`
- 地图根目录：`/home/nvidia/slam_maps`
- Ouster 数据：`/ouster/points`、`/ouster/imu`

现有 Ouster、FAST-LIO 建图和定位核心算法代码没有修改。`fast_lio_incremental` 包按要求保留，没有删除。

## 3. 总体运行流程

```text
系统启动
  -> 配置与地图目录检查
  -> Ouster 数据健康检测
  -> SENSOR_READY
  -> WAIT_MODE
       |-> 启动 Mapping -> 建图 -> 保存地图 -> WAIT_MODE
       `-> 选择地图 -> 启动 Localization -> RELOCALIZING
             |-> Last Pose 初始化
             `-> 人工 /initialpose 初始化
                  -> 定位算法确认 -> LOCALIZED
```

Mapping 和 Localization 不允许同时运行。所有启动、停止及模式切换请求先经过状态机合法性检查。

## 4. 今天完成的工作

### Phase 1：系统基础、状态机和配置

- 创建 ROS 2 Humble C++ 包 `slam_system_manager`。
- 实现强类型 `SystemState` 状态枚举和合法状态转换检查。
- 实现 `ConfigManager`，读取 `system.yaml`、`sensor.yaml` 和 `process.yaml`。
- 新增 `SystemStatus.msg`。
- 以约 2 Hz 发布 `/system/status`。
- 状态中统一提供传感器、建图、定位、当前地图和错误信息。

### Phase 2：外部进程管理

- 实现 `ProcessManager`。
- 使用 `fork`、`exec`、`waitpid` 和进程组管理外部 ROS 2 程序。
- 支持启动、停止、重启、运行检测和退出码获取。
- 防止同一程序重复启动。
- 停止时按 `SIGINT -> SIGTERM -> SIGKILL` 分级处理。
- 所有启动命令和停止超时均来自 `process.yaml`，未写死在 C++ 中。
- 修正 rosbridge 重复启动和 9090 端口占用问题，并配置异步 Service/Action 处理参数。

### Phase 3：Ouster 健康检测

- 订阅 `/ouster/points` 和 `/ouster/imu`。
- 检测点云、IMU 是否超时并统计近似频率。
- 阈值和 Topic 均从 YAML 读取。
- 只有点云与 IMU 同时健康时才进入 `SENSOR_READY` 和 `WAIT_MODE`。
- 支持识别由用户单独启动、并非 ProcessManager 启动的 Ouster 驱动。
- 建图或定位过程中雷达持续掉线时进入 `ERROR`，不继续报告系统正常。

### Phase 4：地图管理

- 实现统一地图目录和每张地图独立目录结构。
- 实现地图创建、删除、加载、存在检查和列表查询。
- 新增 `/system/get_map_list`、`/system/create_map`、`/system/delete_map` 和 `/system/load_map`。
- 创建和更新 `metadata.yaml`。
- 限制地图名只能使用安全字符，防止 `../../` 等路径穿越。
- 元数据采用安全写入方式，避免产生不完整文件。

### Phase 5：Mapping 建图控制

- 新增 `/system/start_mapping`、`/system/stop_mapping` 和 `/system/save_map`。
- 实现 `MappingAdapter`，隔离 FAST-LIO 特有配置和地图保存接口。
- 启动建图前检查状态、传感器健康、地图名和 Localization 互斥条件。
- 为每张地图生成独立运行配置，不修改 FAST-LIO 原始配置。
- 使用 YAML 配置的 `/map_save` Service 保存 `map.pcd`。
- 保存后检查 PCD 是否存在且非空，并更新地图元数据。
- 监控 Mapping 意外退出并报告明确错误码。
- 已验证建图期间 `/cloud_registered` 正常发布。

### Phase 6：Localization 定位控制

- 新增 `/system/start_localization` 和 `/system/stop_localization`。
- 启动前检查地图、非空 `map.pcd`、传感器状态和 Mapping 互斥条件。
- 支持在进程命令中替换 `{map_name}` 和 `{map_path}`。
- 启动后检查 `/pcd_map` 和 `/Odometry` 发布端是否就绪。
- 监控定位进程意外退出、启动失败和传感器掉线。
- 已使用真实 Ouster 数据及现有 PCD 地图验证定位启动和停止。
- 实测 `/Odometry` 约 9.5 至 10 Hz，`/cloud_registered` 约 6 至 7 Hz。

### Phase 7：Initial Pose、统一输出和 Last Pose

- 新增 `/system/set_initial_pose`，请求类型包含 `PoseWithCovarianceStamped`。
- 保留 `/initialpose` Topic，兼容 RViz 的 `2D Pose Estimate`。
- 实现 `RelocalizationAdapter`。
- 校验初始位姿 Frame、有限数值和四元数，并对有效四元数归一化。
- 发送初始位姿后保持 `RELOCALIZING`，只有定位算法产生新的 `/map_to_odom` 后才进入 `LOCALIZED`。
- 融合 `/map_to_odom` 与 `/Odometry`，统一发布：
  - `/localization/pose`
  - `/localization/odometry`
- 输出 Frame 来自配置，当前为 `map` 和 `base_link`。
- 仅在 `LOCALIZED` 状态下定时保存当前地图的 `last_pose.yaml`。
- Last Pose 使用后台写入、临时文件和原子重命名。
- 再次启动同一地图时自动读取 Last Pose，并在定位节点稳定后有限次数重试初始化。
- Last Pose 不可用时保持 `RELOCALIZING`，等待人工初始化，不无限重试。

## 5. 当前 ROS 2 对外接口

主要状态 Topic：

- `/system/status`

地图与模式控制 Service：

- `/system/get_map_list`
- `/system/create_map`
- `/system/delete_map`
- `/system/load_map`
- `/system/start_mapping`
- `/system/stop_mapping`
- `/system/save_map`
- `/system/start_localization`
- `/system/stop_localization`
- `/system/set_initial_pose`

定位接口：

- 输入：`/initialpose`
- 输出：`/localization/pose`
- 输出：`/localization/odometry`

## 6. 配置职责

- `config/system.yaml`：地图根目录、Frame、Topic、状态频率、Last Pose 和重定位参数。
- `config/sensor.yaml`：点云/IMU 超时、最低频率、启动宽限时间和检测周期。
- `config/process.yaml`：Ouster、Mapping、Localization、rosbridge 命令及 Adapter 参数。

更换机器人平台或算法时，应优先修改 YAML 或新增 Adapter，不应把平台参数写入 `SystemManager`。

## 7. 编译与验证结果

编译命令：

```bash
cd /home/nvidia/fast_lio_ouster_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select slam_system_manager --symlink-install
source install/setup.bash
```

自动化测试结果：

```text
28 tests, 0 errors, 0 failures, 0 skipped
```

今天完成的实机验证包括：

- 独立启动的 Ouster 能被管理器识别。
- 点云、IMU 存活状态及频率能正常显示。
- 地图创建、列表、加载和非法地图名拒绝正常。
- FAST-LIO 建图可启动、停止并保存非空 PCD。
- 定位可加载已有地图并稳定发布相关 Topic。
- Mapping 与 Localization 互斥有效。
- Service 和 RViz 两种初始位姿方式有效。
- 定位确认后能进入 `LOCALIZED`。
- 统一定位 Topic 正常发布。
- Last Pose 能保存，并可在重新启动定位时自动恢复。

## 8. 当前边界和后续工作

当前已经完成 Phase 1 至 Phase 7。以下内容尚未作为完成项：

- Phase 8：通过 rosbridge 对全部 `/system/*` 接口进行 Web 侧联调和异常验证。
- Phase 9：增加启动脚本和 `slam_system.service`，完成 systemd 开机自启动、重启策略及日志验证。
- 后续增强：`/system/recover`、自动恢复策略、定位质量指标、全局自动重定位、Localization Adapter 的更多算法适配。
- 上车前必须使用实测标定值替换 `lidar_to_base` 外参占位配置。

## 9. 相关文档

- `docs/phase1_to_phase6_commands.md`
- `docs/phase5_mapping_commands.md`
- `docs/phase6_localization_commands.md`
- `docs/phase7_initial_pose_last_pose_commands.md`

这些文档包含各阶段的详细运行命令、测试方法和预期结果。
