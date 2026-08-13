# Phase 9：systemd 开机自启动部署与测试

日期：2026-08-13

适用环境：

- Ubuntu 22.04
- ROS 2 Humble
- 用户：`nvidia`
- 工作空间：`/home/nvidia/fast_lio_ouster_ws`
- Ouster 地址：`172.168.1.3`
- Unit：`slam_system.service`

## 1. 设计结果

- 开发配置 `config/process.yaml` 保持 Ouster `auto_start: false`。
- 部署配置 `config/process_deployment.yaml` 自动启动 Ouster 和 rosbridge。
- Ouster hostname 由 YAML 的 `substitutions.sensor_hostname` 提供，不写死在 C++。
- systemd 使用非交互 Bash 脚本，不依赖 `.bashrc`。
- 启动时如已发现外部 `/ouster/points` 或 `/ouster/imu` 发布者，跳过托管 Ouster，防止双实例。
- Unit 使用 `Restart=on-failure` 和 5 秒延迟，不因传感器暂时无数据而重启。
- 停止时使用 SIGINT，并由 systemd control group 兜底清理子进程。
- 日志进入 journald。

## 2. 编译与测试

```bash
cd /home/nvidia/fast_lio_ouster_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select slam_system_manager --symlink-install
source install/setup.bash
ROS_DOMAIN_ID=42 colcon test \
  --packages-select slam_system_manager --event-handlers console_direct+
colcon test-result --verbose
```

独立的 `ROS_DOMAIN_ID` 防止已运行的生产节点响应测试使用的同名服务。也可以先停止
`slam_system.service` 再测试。要求：Phase 8 的 35 项测试继续通过，Phase 9 新增测试也全部通过。

## 3. 检查部署配置

```bash
grep -A10 '^  ouster:' \
  /home/nvidia/fast_lio_ouster_ws/src/slam_system_manager/config/process_deployment.yaml
```

预期包含：

```yaml
substitutions:
  sensor_hostname: "172.168.1.3"
auto_start: true
```

开发配置仍应为：

```bash
grep -A5 '^  ouster:' \
  /home/nvidia/fast_lio_ouster_ws/src/slam_system_manager/config/process.yaml
```

```yaml
auto_start: false
```

## 4. 安装启动脚本和 Unit

```bash
sudo install -Dm755 \
  /home/nvidia/fast_lio_ouster_ws/src/slam_system_manager/scripts/start_slam_system.sh \
  /home/nvidia/scripts/start_slam_system.sh

sudo install -Dm644 \
  /home/nvidia/fast_lio_ouster_ws/src/slam_system_manager/systemd/slam_system.service \
  /etc/systemd/system/slam_system.service

sudo systemctl daemon-reload
```

检查 Unit：

```bash
systemd-analyze verify /etc/systemd/system/slam_system.service
systemctl cat slam_system.service
```

## 5. 非交互启动脚本测试

执行前确保没有手动 SystemManager、rosbridge 或 Ouster 实例：

```bash
ps -eo pid,ppid,pgid,stat,cmd | \
  grep -E '(system_manager|rosbridge|ouster|os_driver)' | grep -v grep
```

测试：

```bash
sudo -u nvidia env -i \
  HOME=/home/nvidia \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  timeout --signal=INT 20 /home/nvidia/scripts/start_slam_system.sh
```

预期脚本能加载 ROS 2 和工作空间，启动 SystemManager、Ouster 和 rosbridge，不读取 `.bashrc`。

## 6. 启用和启动服务

```bash
sudo systemctl enable slam_system.service
sudo systemctl start slam_system.service
```

检查：

```bash
systemctl is-enabled slam_system.service
systemctl is-active slam_system.service
systemctl status slam_system.service --no-pager
```

预期：

```text
enabled
active
```

检查节点、状态和端口：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/fast_lio_ouster_ws/install/setup.bash
ros2 node list | sort
ros2 topic echo /system/status --once
ss -ltnp | grep ':9090'
```

传感器健康时预期 `WAIT_MODE`。Ouster 暂不可用时预期 `SENSOR_STARTING`，Unit 仍保持 `active`。

## 7. 重复 Ouster 防护测试

先人工启动 Ouster并确认 Topic 健康，再启动 Unit。日志应出现：

```text
[PROCESS] External Ouster publishers detected; skipping managed Ouster auto-start
```

检查：

```bash
ps -eo pid,ppid,pgid,stat,cmd | \
  grep -E '(sensor.launch.xml|os_driver)' | grep -v grep
```

预期只有原人工 Ouster 实例。此时停止 Unit 不会停止外部 Ouster。

## 8. 优雅停止测试

```bash
sudo systemctl stop slam_system.service
```

检查：

```bash
systemctl is-active slam_system.service
ps -eo pid,ppid,pgid,stat,cmd | \
  grep -E '(system_manager|rosbridge|ouster|os_driver|fast_lio|localization)' | grep -v grep
ss -ltnp | grep ':9090' || true
```

如果 Ouster 由 Unit 启动，预期相关进程全部退出且 9090 释放。手动外部 Ouster 不属于 Unit，不应被停止。

## 9. 异常重启测试

只在测试窗口执行：

```bash
main_pid=$(systemctl show -p MainPID --value slam_system.service)
test "${main_pid}" -gt 1
sudo kill -KILL "${main_pid}"
```

约 5 至 10 秒后：

```bash
systemctl is-active slam_system.service
systemctl show -p MainPID -p NRestarts slam_system.service
ss -ltnp | grep ':9090'
```

预期 MainPID 改变、`NRestarts` 增加、服务重新 active，9090 不冲突。

## 10. 日志

```bash
journalctl -u slam_system.service -n 100 --no-pager
journalctl -u slam_system.service -b --no-pager
```

实时日志：

```bash
journalctl -u slam_system.service -f
```

## 11. 重启设备验收

该步骤会中断 SSH，执行前确认没有建图保存或其他重要任务：

```bash
sudo reboot
```

重新上线后：

```bash
systemctl is-enabled slam_system.service
systemctl is-active slam_system.service
journalctl -u slam_system.service -b --no-pager
```

从控制机检查：

```bash
python3 rosbridge_smoke_test.py --url ws://192.168.1.199:9090 --timeout 10
```

预期开机后服务自动 active、Ouster 健康后进入 `WAIT_MODE`、地图及 Last Pose 保持不变。

## 12. 停用与回滚

仅停止：

```bash
sudo systemctl stop slam_system.service
```

取消开机启动：

```bash
sudo systemctl disable slam_system.service
```

恢复人工开发方式时，继续使用默认启动命令：

```bash
ros2 launch slam_system_manager system_bringup.launch.py
```

默认启动使用 `process.yaml`，不会自动启动 Ouster。

## 13. Phase 9 实际验收记录

2026-08-13 实机验收结果：

- 编译成功；自动化测试 `36 tests, 0 errors, 0 failures, 0 skipped`。
- 空环境非交互启动成功，不依赖 `.bashrc`。
- 外部 Ouster 点云约 7--10 Hz、IMU 约 100 Hz 时，管理器识别发布者并跳过托管 Ouster，未产生双实例。
- Unit 已安装并设为 `enabled`；正常启动为 `active`，传感器健康后进入 `WAIT_MODE`。
- 本机 `ws://127.0.0.1:9090` 和控制机 `ws://192.168.1.199:9090` 的 rosbridge 六步冒烟测试均通过。
- `systemctl stop` 后 SystemManager、Ouster 和 rosbridge 均退出，9090 释放。
- 暂停 Ouster 驱动 3 秒期间，Unit 保持 `active`、`NRestarts=0`，状态进入 `SENSOR_STARTING / SENSOR_NO_POINTCLOUD`；恢复后自动回到 `WAIT_MODE`。
- 强制终止 MainPID 后，MainPID 从 `21919` 变为 `22213`，`NRestarts` 从 0 增至 1；服务、Ouster 和 9090 均正常恢复，状态回到 `WAIT_MODE`。
- 日志已写入 journald，可通过第 10 节命令查询。
- 真实重启验收尚未执行，须先确认设备当前没有不可中断任务。
- 真实 reboot：待最终确认后执行。
