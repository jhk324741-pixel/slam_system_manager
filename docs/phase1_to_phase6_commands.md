# slam_system_manager Phase 1-6 功能与命令手册（截至 Phase 6）

本文档适用于以下环境：

- 设备：Jetson / RK3588
- 系统：Ubuntu 22.04
- ROS 2：Humble
- 工作空间：`/home/nvidia/fast_lio_ouster_ws`
- 管理包：`slam_system_manager`

当前实现范围：

- Phase 1：系统状态机、配置读取、`/system/status`
- Phase 2：外部进程管理
- Phase 3：Ouster 点云和 IMU 健康检测
- Phase 4：地图目录、元数据和地图管理服务
- Phase 5：FAST-LIO 建图启动、停止、保存和异常监控
- Phase 6：已有 PCD 地图定位启动、停止、互斥和异常监控

## 1. 登录设备

```bash
ssh nvidia@<DEVICE_IP>
```

用途：登录运行 ROS 2 工作空间的设备。登录用户名为 `nvidia`。

最初配置的设备地址是 `192.168.1.40`，2026-08-12 验证 Phase 5 时 DHCP 地址变为
`192.168.1.199`。应使用设备当前 IP；如果固定部署，建议在路由器中为设备 MAC
`84:fc:14:80:00:b6` 配置 DHCP 地址保留。

## 2. 每个终端都要加载的环境

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/fast_lio_ouster_ws/install/setup.bash
```

用途：第一条加载 ROS 2 Humble，第二条加载当前工作空间编译生成的包和接口。每打开一个新终端都需要执行。

如果工作空间还没有成功编译，只执行第一条，然后先完成第 4 节的编译。

## 3. 工作空间检查命令

### 3.1 查看当前位置

```bash
pwd
```

用途：确认当前终端所在目录，避免在错误工作空间执行编译或文件操作。

### 3.2 查看工作空间源码

```bash
find /home/nvidia/fast_lio_ouster_ws/src -maxdepth 3 -type f | sort
```

用途：查看 `src` 下的源码、配置和 launch 文件。

### 3.3 查看 colcon 包

```bash
cd /home/nvidia/fast_lio_ouster_ws
colcon list
```

用途：确认 colcon 能识别当前工作空间中的包。应能看到：

```text
slam_system_manager
ouster_ros
fast_lio
fast_lio_localization
```

具体输出还会包含这些包依赖的其他 ROS 2 package。

### 3.4 查看当前 ROS 2 可用包

```bash
ros2 pkg list | sort
```

用途：检查系统安装和当前工作空间中已经可用的 ROS 2 包。

只检查本项目相关包：

```bash
ros2 pkg list | grep -E '^(slam_system_manager|ouster_ros|fast_lio|fast_lio_localization|rosbridge_server)$'
```

### 3.5 查找 launch 文件

```bash
find /home/nvidia/fast_lio_ouster_ws/src \
  -type f \( -name '*.launch.py' -o -name '*.launch.xml' \) | sort
```

用途：确认 Ouster、建图、定位和系统管理程序实际使用的 launch 文件，不依赖猜测包名。

### 3.6 查找 YAML 配置

```bash
find /home/nvidia/fast_lio_ouster_ws/src \
  -type f \( -name '*.yaml' -o -name '*.yml' \) | sort
```

用途：查看工作空间中现有算法和系统管理程序的配置文件位置。

## 4. 编译 slam_system_manager

```bash
cd /home/nvidia/fast_lio_ouster_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select slam_system_manager --symlink-install
source install/setup.bash
```

用途：只编译 `slam_system_manager`，不重新编译、不修改 Ouster、FAST-LIO 建图和定位算法包。

预期结果：

```text
Summary: 1 package finished
```

如果修改了 `.msg`、`.srv`、`CMakeLists.txt` 或 `package.xml`，必须重新执行该命令。

## 5. 运行系统管理节点

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/fast_lio_ouster_ws/install/setup.bash
ros2 launch slam_system_manager system_bringup.launch.py
```

用途：启动 `system_manager`。根据 `process.yaml` 当前配置，程序还会自动启动 rosbridge；Ouster、Mapping 和 Localization 默认不自动启动。

没有雷达数据时，预期关键日志：

```text
[SYSTEM] BOOT -> SYSTEM_CHECK
[MAP] Found 0 map(s) under /home/nvidia/slam_maps
[SYSTEM] SYSTEM_CHECK -> SENSOR_STARTING
```

终止程序：

```text
Ctrl+C
```

用途：向 launch 和受管理的子进程发送退出信号，正常停止本次运行。

## 6. Phase 1：状态机和状态输出

### 6.1 查看系统状态

```bash
ros2 topic echo /system/status
```

用途：持续查看状态机、传感器、建图、定位、当前地图和错误信息。

只读取一帧：

```bash
ros2 topic echo /system/status --once
```

没有 Ouster 数据时预期：

```yaml
state: SENSOR_STARTING
ouster_running: false
pointcloud_alive: false
imu_alive: false
pointcloud_hz: 0.0
imu_hz: 0.0
mapping_running: false
localization_running: false
current_map: ''
error_code: SENSOR_NO_POINTCLOUD
error_message: Waiting for Ouster PointCloud2 data
```

### 6.2 检查状态发布频率

```bash
ros2 topic hz /system/status
```

用途：验证状态按照 `system.yaml` 中的 `status_publish_hz` 发布。

预期：约 `2 Hz`。

### 6.3 查看状态消息定义

```bash
ros2 interface show slam_system_manager/msg/SystemStatus
```

用途：查看 Web、Qt 或其他 ROS 2 节点能够获取的状态字段。

### 6.4 查看系统配置

```bash
cat /home/nvidia/fast_lio_ouster_ws/src/slam_system_manager/config/system.yaml
```

用途：检查地图根目录、TF frame、topic、状态频率和 Last Pose 配置。修改后需要重新编译，或者启动时传入其他配置文件。

## 7. Phase 2：外部进程管理

### 7.1 查看进程配置

```bash
cat /home/nvidia/fast_lio_ouster_ws/src/slam_system_manager/config/process.yaml
```

用途：查看 Ouster、Mapping、Localization 和 rosbridge 的启动命令、工作目录、是否自动启动及停止超时。

所有实际启动命令均从该 YAML 读取，没有写死在 C++ 中。

### 7.2 查看 ROS 节点

```bash
ros2 node list
```

用途：确认 `system_manager` 和自动启动的 rosbridge 节点是否存在。正常情况下可以看到：

```text
/system_manager
/rosbridge_websocket
/rosapi
```

### 7.3 查看 Linux 进程

```bash
ps -eo pid,ppid,pgid,stat,cmd | \
  grep -E '(system_manager|rosbridge|ouster|fast_lio)' | grep -v grep
```

用途：检查进程 PID、父进程 PID 和进程组。ProcessManager 使用独立进程组管理子程序，并按 SIGINT、SIGTERM、SIGKILL 顺序停止。

### 7.4 运行 ProcessManager 单元测试

```bash
cd /home/nvidia/fast_lio_ouster_ws
source /opt/ros/humble/setup.bash
colcon test --packages-select slam_system_manager
colcon test-result --verbose
```

用途：验证防止重复启动、退出码获取、命令模板替换和停止超时后的信号升级。

Phase 2 暂未向上位机暴露任意进程启动 service。这样可以避免 Web 端绕过系统状态机直接启动 Mapping 或 Localization。

## 8. Phase 3：Ouster 健康检测

### 8.1 查看传感器检测配置

```bash
cat /home/nvidia/fast_lio_ouster_ws/src/slam_system_manager/config/sensor.yaml
```

用途：查看点云/IMU 超时阈值、最低频率、频率统计窗口和启动宽限时间。

### 8.2 单独启动 Ouster

使用已经验证可正常运行的 Ouster 命令。当前工作空间对应的命令形式为：

```bash
ros2 launch ouster_ros sensor.launch.xml \
  sensor_hostname:=<OUSTER_IP_OR_HOSTNAME> viz:=false
```

用途：验证 Ouster 即使不是由 ProcessManager 启动，SystemManager 仍能根据 `/ouster/points` 和 `/ouster/imu` 数据检测到它。

必须把 `<OUSTER_IP_OR_HOSTNAME>` 替换为实际雷达地址。

### 8.3 检查 Topic 是否存在

```bash
ros2 topic list | grep '^/ouster/'
ros2 topic info /ouster/points --verbose
ros2 topic info /ouster/imu --verbose
```

用途：确认点云和 IMU topic 已发布，并检查 publisher/subscriber 与 QoS。

### 8.4 检查数据频率

```bash
ros2 topic hz /ouster/points
```

另开一个终端：

```bash
ros2 topic hz /ouster/imu
```

用途：比较实际频率和 `sensor.yaml` 中的 `min_pointcloud_hz`、`min_imu_hz`。

### 8.5 检查健康状态

```bash
ros2 topic echo /system/status
```

当点云和 IMU 都存活且频率达标后，预期状态切换：

```text
SENSOR_STARTING -> SENSOR_READY -> WAIT_MODE
```

预期字段：

```yaml
state: WAIT_MODE
ouster_running: true
pointcloud_alive: true
imu_alive: true
pointcloud_hz: 10.0   # 示例，以实际雷达输出为准
imu_hz: 100.0         # 示例，以实际雷达输出为准
```

停止 Ouster 后，超过配置的 timeout，预期回到：

```yaml
state: SENSOR_STARTING
pointcloud_alive: false
imu_alive: false
```

## 9. Phase 4：地图管理

### 9.1 查看地图服务

```bash
ros2 service list | grep '^/system/'
```

用途：确认地图服务已经注册。当前应包含：

```text
/system/create_map
/system/delete_map
/system/get_map_list
/system/load_map
```

查看接口定义：

```bash
ros2 interface show slam_system_manager/srv/CreateMap
ros2 interface show slam_system_manager/srv/DeleteMap
ros2 interface show slam_system_manager/srv/GetMapList
ros2 interface show slam_system_manager/srv/LoadMap
```

### 9.2 创建测试地图

```bash
ros2 service call /system/create_map \
  slam_system_manager/srv/CreateMap \
  "{map_name: phase4_test}"
```

用途：创建统一地图目录和 `metadata.yaml`。

预期：

```text
success=True
map_path='/home/nvidia/slam_maps/phase4_test/map.pcd'
message='Map directory created'
```

### 9.3 检查地图文件

```bash
find /home/nvidia/slam_maps/phase4_test -maxdepth 1 -type f -print
cat /home/nvidia/slam_maps/phase4_test/metadata.yaml
```

用途：确认目录和元数据已生成。预期元数据格式：

```yaml
name: phase4_test
created_at: 2026-01-01T00:00:00Z
updated_at: 2026-01-01T00:00:00Z
map_file: map.pcd
frame_id: map
```

时间以实际创建时间为准。Phase 4 不会生成真实 `map.pcd`，真实地图由 Phase 5 的建图保存流程生成。

### 9.4 获取地图列表

```bash
ros2 service call /system/get_map_list \
  slam_system_manager/srv/GetMapList '{}'
```

用途：获取 Web/Qt 可显示的本地地图列表。

预期：

```text
success=True
map_names=['phase4_test']
message='Found 1 map(s)'
```

### 9.5 验证非法路径保护

```bash
ros2 service call /system/create_map \
  slam_system_manager/srv/CreateMap \
  "{map_name: ../../etc/passwd}"
```

用途：验证地图名检查和目录穿越保护。

预期：

```text
success=False
message='Invalid map name...'
```

系统不会在 `/home/nvidia/slam_maps` 外创建文件。

### 9.6 测试缺少 PCD 时的加载结果

```bash
ros2 service call /system/load_map \
  slam_system_manager/srv/LoadMap \
  "{map_name: phase4_test}"
```

用途：确认系统不会把只有元数据、没有 PCD 的目录当成可定位地图。

预期：

```text
success=False
message='Map PCD file not found: /home/nvidia/slam_maps/phase4_test/map.pcd'
```

### 9.7 仅测试加载接口成功路径

下面的空文件不是真实地图，只用于测试 Phase 4 接口：

```bash
touch /home/nvidia/slam_maps/phase4_test/map.pcd

ros2 service call /system/load_map \
  slam_system_manager/srv/LoadMap \
  "{map_name: phase4_test}"
```

预期：

```text
success=True
map_path='/home/nvidia/slam_maps/phase4_test/map.pcd'
message='Map selected'
```

再检查状态：

```bash
ros2 topic echo /system/status --once
```

预期：

```yaml
current_map: phase4_test
```

### 9.8 删除测试地图

```bash
ros2 service call /system/delete_map \
  slam_system_manager/srv/DeleteMap \
  "{map_name: phase4_test}"
```

用途：删除地图目录及其文件。

如果 `phase4_test` 已通过 `load_map` 设为当前地图，系统会拒绝删除：

```text
success=False
message='Cannot delete the currently selected map'
```

这是防止运行中误删当前地图的保护。停止并重新启动 SystemManager 后，当前选择会清空，再调用删除服务即可。

## 10. Phase 5：Mapping 建图控制

### 10.1 本阶段完成的功能

Phase 5 在不修改 FAST-LIO 和 Ouster 核心算法的前提下完成以下能力：

1. 通过 `/system/start_mapping` 启动建图，参数为地图名称。
2. 自动创建地图目录并为每次建图生成独立的 `mapping_runtime.yaml`。
3. 从 `process.yaml` 读取 FAST-LIO 启动命令，不在 C++ 中写死包名和 launch 参数。
4. 通过 `/system/stop_mapping` 停止并放弃当前未保存地图。
5. 通过 `/system/save_map` 调用 MappingAdapter，再由 Adapter 调用 FAST-LIO `/map_save`。
6. 验证 `map.pcd` 存在且非空，更新 `metadata.yaml`，最后返回 `WAIT_MODE`。
7. 防止重复启动 Mapping、防止覆盖已有 PCD，并阻止 Mapping 和 Localization 同时运行。
8. 监控 Mapping 子进程，进程意外退出时进入 `ERROR`。
9. Ouster 持续异常时停止 Mapping，避免继续报告建图正常。

正常状态流转：

```text
WAIT_MODE -> MAPPING_STARTING -> MAPPING
MAPPING -> MAP_SAVING -> WAIT_MODE
MAPPING -> stop_mapping -> WAIT_MODE
```

所有启动、停止和保存 service 使用异步受理语义。`accepted: true` 表示操作已经提交到后台线程，最终结果必须通过 `/system/status` 和地图文件确认。

### 10.2 本阶段可靠性修复

ProcessManager 使用 `fork()` 和 `exec()` 启动外部 ROS 2 程序。在 `exec()` 前会关闭继承自 SystemManager 的 DDS socket 和 Fast DDS 共享内存描述符，只保留标准输入、输出和错误流。这可防止 FAST-LIO 话题端点已经创建、但点云和 IMU 回调始终为零的问题。

HealthMonitor 将点云和 IMU 订阅放在独立的 `Reentrant` callback group 中，避免 service 和 timer 阻塞高频传感器回调。当前判断规则：

- PointCloud2 超过 `pointcloud_timeout_sec` 未到达，立即上报 `pointcloud_alive: false`。
- IMU 超过 `imu_timeout_sec` 未到达，立即上报 `imu_alive: false`。
- 频率阈值继续计算并发布，但短窗口频率抖动不再直接中断建图。
- 活跃建图期间，传感器持续异常超过 `active_failure_grace_sec` 才进入 `ERROR`。

当前配置：

```yaml
pointcloud_timeout_sec: 1.0
imu_timeout_sec: 0.5
min_pointcloud_hz: 5.0
min_imu_hz: 20.0
frequency_window_sec: 2.0
active_failure_grace_sec: 5.0
```

配置文件：

```bash
cat /home/nvidia/fast_lio_ouster_ws/src/slam_system_manager/config/sensor.yaml
```

### 10.3 查看 Mapping 接口

```bash
ros2 interface show slam_system_manager/srv/StartMapping
ros2 interface show slam_system_manager/srv/StopMapping
ros2 interface show slam_system_manager/srv/SaveMap
ros2 service type /map_save
```

预期 `/map_save` 类型：

```text
std_srvs/srv/Trigger
```

### 10.4 启动并检查建图

启动前确认：

```bash
ros2 topic echo --once /system/status
```

必须看到 `state: WAIT_MODE`、`pointcloud_alive: true` 和 `imu_alive: true`。

启动：

```bash
ros2 service call /system/start_mapping \
  slam_system_manager/srv/StartMapping \
  "{map_name: phase5_test}"
```

预期立即返回：

```text
accepted=True
message='Mapping start accepted'
```

确认算法实际处理数据，而不只是进程存在：

```bash
ros2 topic hz /cloud_registered
ros2 topic hz /Odometry
ros2 topic echo --once /system/status
```

预期状态为 `MAPPING`，`mapping_running: true`，两个 FAST-LIO 输出话题持续发布。

检查 Adapter 生成的配置：

```bash
cat /home/nvidia/slam_maps/phase5_test/mapping_runtime.yaml
```

其中 `map_file_path` 必须指向：

```text
/home/nvidia/slam_maps/phase5_test/map.pcd
```

### 10.5 保存地图

```bash
ros2 service call /system/save_map \
  slam_system_manager/srv/SaveMap \
  "{}"
```

等待状态返回 `WAIT_MODE` 后检查：

```bash
ros2 topic echo --once /system/status
ls -lh /home/nvidia/slam_maps/phase5_test/map.pcd
cat /home/nvidia/slam_maps/phase5_test/metadata.yaml
```

预期 `map.pcd` 存在且大于 0 字节，Mapping 已停止，`error_code` 为空。

### 10.6 停止但不保存

```bash
ros2 service call /system/start_mapping \
  slam_system_manager/srv/StartMapping \
  "{map_name: phase5_discard_test}"

ros2 service call /system/stop_mapping \
  slam_system_manager/srv/StopMapping \
  "{}"

ros2 service call /system/delete_map \
  slam_system_manager/srv/DeleteMap \
  "{map_name: phase5_discard_test}"
```

用途：验证放弃建图流程，预期最终返回 `WAIT_MODE` 且不生成 `map.pcd`。

### 10.7 拒绝和异常场景

Mapping 运行期间再次启动应返回 `accepted=False`：

```bash
ros2 service call /system/start_mapping \
  slam_system_manager/srv/StartMapping \
  "{map_name: duplicate_test}"
```

非法地图名应被拒绝：

```bash
ros2 service call /system/start_mapping \
  slam_system_manager/srv/StartMapping \
  "{map_name: ../../etc/passwd}"
```

使用已有非空 `map.pcd` 的地图名应返回：

```text
Refusing to overwrite an existing map.pcd
```

可选破坏性测试：在测试地图建图期间结束 FAST-LIO。

```bash
mapping_pid=$(pgrep -x fastlio_mapping)
kill -KILL "${mapping_pid}"
sleep 1
ros2 topic echo --once /system/status
```

预期进入 `ERROR`，`error_code: MAPPING_PROCESS_EXITED`。当前版本的 `ERROR` 是粘滞状态，破坏性测试后需要重新启动 SystemManager。

## 11. Phase 6：Localization 定位控制

### 11.1 本阶段完成的功能

Phase 6 不修改 `fast_lio_localization` 算法源码，新增以下管理能力：

1. `/system/start_localization` 接收 `map_name`，验证地图目录和非空 `map.pcd`。
2. 从 `process.yaml` 读取定位启动命令，并替换 `{map_name}`、`{map_path}`。
3. 等待定位进程稳定且 `/pcd_map`、`/Odometry` 发布端就绪后进入 `RELOCALIZING`。
4. `/system/stop_localization` 停止完整进程组并返回 `WAIT_MODE`。
5. Mapping 与 Localization 双向互斥，定位期间不能启动建图或修改地图目录。
6. 定位进程意外退出、Ouster 持续掉线时进入 `ERROR` 并报告错误码。
7. `/system/status` 输出 `localization_running` 和当前定位地图。

正常状态流转：

```text
WAIT_MODE -> LOCALIZATION_STARTING -> RELOCALIZING
RELOCALIZING -> stop_localization -> WAIT_MODE
```

Phase 6 尚不发送初始位姿，所以不会进入 `LOCALIZED`。`/initialpose`、
`/system/set_initial_pose`、Last Pose 读取与定时保存属于 Phase 7。

### 11.2 查看定位配置和接口

```bash
cat /home/nvidia/fast_lio_ouster_ws/src/slam_system_manager/config/process.yaml
ros2 interface show slam_system_manager/srv/StartLocalization
ros2 interface show slam_system_manager/srv/StopLocalization
```

用途：检查实际定位 launch 命令、启动超时和就绪 topic。当前实际命令为：

```text
ros2 launch fast_lio_localization localization_ouster64.launch.py map:={map_path} rviz:=false
```

集成其他定位算法时，只需修改 `processes.localization.command` 和
`localization_adapter.required_topics`，不需要修改 SystemManager。

定位 launch 中使用的 Python 节点必须具有执行权限。检查命令：

```bash
ls -l /home/nvidia/fast_lio_ouster_ws/src/FAST_LIO_LOCALIZATION-ROS-NOETIC/scripts/*.py
ros2 pkg executables fast_lio_localization
```

如果 launch 报 `executable 'global_localization.py' not found on the libexec directory`，
修复脚本权限后重新启动 SystemManager：

```bash
chmod 755 \
  /home/nvidia/fast_lio_ouster_ws/src/FAST_LIO_LOCALIZATION-ROS-NOETIC/scripts/global_localization.py \
  /home/nvidia/fast_lio_ouster_ws/src/FAST_LIO_LOCALIZATION-ROS-NOETIC/scripts/map_to_odom_tf.py \
  /home/nvidia/fast_lio_ouster_ws/src/FAST_LIO_LOCALIZATION-ROS-NOETIC/scripts/publish_initial_pose.py \
  /home/nvidia/fast_lio_ouster_ws/src/FAST_LIO_LOCALIZATION-ROS-NOETIC/scripts/transform_fusion.py
```

这只修正 ROS 2 节点的文件执行权限，不修改定位算法内容。使用
`--symlink-install` 时，安装目录会直接继承源脚本权限。

### 11.3 启动定位

先确认 Ouster 健康、系统空闲且地图 PCD 非空：

```bash
ros2 topic echo /system/status --once
ls -lh /home/nvidia/slam_maps/test_map/map.pcd
```

启动：

```bash
ros2 service call /system/start_localization \
  slam_system_manager/srv/StartLocalization \
  "{map_name: test_map}"
```

Service 采用异步受理语义，预期立即返回：

```text
accepted=True
message='Localization start accepted'
```

等待数秒后检查：

```bash
ros2 topic echo /system/status --once
ros2 topic info /pcd_map --verbose
ros2 topic hz /Odometry
ros2 topic hz /cloud_registered
```

预期状态：

```yaml
state: RELOCALIZING
pointcloud_alive: true
imu_alive: true
mapping_running: false
localization_running: true
current_map: test_map
error_code: ''
```

`/pcd_map` 应有地图发布端，`/Odometry` 和 `/cloud_registered` 应持续发布。
定位算法此时等待 `/initialpose`，属于正常状态。

### 11.4 验证 Mapping/Localization 互斥

定位运行期间调用建图：

```bash
ros2 service call /system/start_mapping \
  slam_system_manager/srv/StartMapping \
  "{map_name: conflict_test}"
```

预期 `accepted=False`，提示 Localization 正在运行。再次调用
`start_localization` 也应返回 `accepted=False`，不会产生重复进程。

### 11.5 停止定位

```bash
ros2 service call /system/stop_localization \
  slam_system_manager/srv/StopLocalization \
  "{}"
```

预期立即受理；等待进程退出后：

```bash
ros2 topic echo /system/status --once
ps -eo pid,ppid,pgid,stat,cmd | \
  grep -E '(localization_ouster64|global_localization|map_to_odom|pcd_to_pointcloud)' | \
  grep -v grep
```

预期状态回到 `WAIT_MODE`、`localization_running: false`、`current_map` 为空，
第二条命令没有定位进程输出。

### 11.6 拒绝和异常场景

地图不存在：

```bash
ros2 service call /system/start_localization \
  slam_system_manager/srv/StartLocalization \
  "{map_name: missing_map}"
```

预期 `accepted=False`，返回地图不存在或 PCD 缺失原因。非法地图名：

```bash
ros2 service call /system/start_localization \
  slam_system_manager/srv/StartLocalization \
  "{map_name: ../../etc/passwd}"
```

预期 `accepted=False` 和 `Invalid map name`。

可选破坏性测试：定位进入 `RELOCALIZING` 后先找到受管理的 launch 主进程：

```bash
pgrep -af 'ros2 launch fast_lio_localization localization_ouster64.launch.py'
kill -KILL <上一步确认的定位 launch PID>
sleep 1
ros2 topic echo /system/status --once
```

预期进入 `ERROR`，`error_code: LOCALIZATION_PROCESS_EXITED`。当前 `ERROR` 为粘滞状态，
测试后需要重新启动 SystemManager。

## 12. 执行全部 Phase 1-6 测试

```bash
cd /home/nvidia/fast_lio_ouster_ws
source /opt/ros/humble/setup.bash
colcon test --packages-select slam_system_manager --event-handlers console_direct+
colcon test-result --verbose
```

用途：执行状态机、ProcessManager、MapManager、MappingAdapter 和
LocalizationAdapter 的全部单元测试。

当前预期结果：

```text
100% tests passed, 0 tests failed
```

当前共有 5 个测试程序、19 个核心 GTest 用例：

- SystemState：5 个
- ProcessManager：4 个
- MapManager：5 个
- MappingAdapter：2 个
- LocalizationAdapter：3 个

## 13. 常用故障检查

### 找不到包

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/fast_lio_ouster_ws/install/setup.bash
ros2 pkg prefix slam_system_manager
```

用途：确认终端已经加载正确工作空间。

### 找不到服务

```bash
ros2 node list
ros2 service list | grep '^/system/'
```

用途：确认 manager 正在运行且 service 已注册。

### Topic 存在但状态仍不健康

```bash
ros2 topic hz /ouster/points
ros2 topic hz /ouster/imu
cat /home/nvidia/fast_lio_ouster_ws/src/slam_system_manager/config/sensor.yaml
```

用途：检查实际数据频率是否达到最低阈值，以及 topic 是否持续输出而非仅被创建。

### rosbridge 端口被占用

```bash
ss -lntp | grep ':9090'
```

用途：检查是否已有 rosbridge 占用默认 `9090` 端口。不要同时手动启动多个 rosbridge 实例。

### 查看关键进程

```bash
ps -eo pid,ppid,pgid,stat,cmd | \
  grep -E '(system_manager|rosbridge|ouster|fast_lio)' | grep -v grep
```

用途：排查重复启动和残留进程。

## 14. 当前阶段可见效果

Phase 1-6 完成后，系统具备以下行为：

1. 启动后加载系统、进程、传感器和地图配置。
2. 持续以约 2 Hz 发布 `/system/status`。
3. 能识别由系统内或外部单独启动的 Ouster。
4. 同时检查点云/IMU 存活状态和近似频率。
5. 传感器健康后进入 `WAIT_MODE`，等待后续模式选择。
6. 通过统一 service 创建、列出、校验、加载和删除地图目录。
7. 拒绝非法地图名、目录穿越、缺失 PCD 的地图以及删除当前地图。
8. 通过统一 service 启动、停止和保存 FAST-LIO 地图。
9. 为每张地图生成独立运行时配置，不修改 FAST-LIO 原始 YAML。
10. 监控 Mapping 进程和 Ouster 数据，异常时停止 Mapping 并报告明确错误码。
11. 建图时发布 `/cloud_registered` 和 `/Odometry`，保存后生成非空 `map.pcd`。
12. 通过统一 service 加载指定 PCD 并启动、停止 Localization。
13. 定位启动后验证 `/pcd_map` 和 `/Odometry` 发布端，再进入 `RELOCALIZING`。
14. Mapping 与 Localization 严格互斥，定位异常退出时报告明确错误码。

当前尚未实现初始位姿和 Last Pose；这些属于 Phase 7。
