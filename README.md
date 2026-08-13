# slam_system_manager

Phases 1 through 9 of the ROS 2 Humble SLAM system manager.

## Current workspace integration

The existing workspace was inspected before this package was added:

- Ouster driver package: `ouster_ros`
- Mapping package: `fast_lio`, launch file `mapping.launch.py`
- Localization package: `fast_lio_localization`, Ouster launch file `localization_ouster64.launch.py`
- Ouster topics used by both FAST-LIO configurations: `/ouster/points` and `/ouster/imu`
- Current localization odometry output: `/Odometry`

No existing package is modified by Phase 1.

## Process configuration

Edit `config/process.yaml` to change all external commands and stop timeouts. The manager never
embeds an Ouster, mapping, localization, or rosbridge launch command in C++. The current commands
match the packages and launch files found in this workspace.

`rosbridge` is auto-started. Ouster, mapping, and localization are disabled for auto-start until
their required runtime substitutions are supplied by later control phases. In particular, Ouster
requires `{sensor_hostname}`, while mapping and localization use `{map_path}`.

Processes are placed in dedicated process groups and managed using `fork`, `exec`, `waitpid`, and
signals. Stop escalation is SIGINT, then SIGTERM, then SIGKILL using YAML-configured timeouts.

## Sensor health configuration

`config/sensor.yaml` defines PointCloud2/IMU timeouts, minimum frequencies, the rolling frequency
window, startup grace period, and health-check rate. Topic names remain in `config/system.yaml`.

The health monitor uses ROS sensor-data QoS. An Ouster driver started outside ProcessManager is still
detected from its data streams; in that case `ouster_running` is inferred from recent sensor data.

## Configuration

Edit `config/system.yaml` to set map storage, frames, topics, status rate, last-pose interval,
and the platform-specific LiDAR-to-base transform. The transform values are placeholders until
they are replaced with calibration results for the target platform.

## Build

```bash
cd /home/nvidia/fast_lio_ouster_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select slam_system_manager
source install/setup.bash
```

## Run

```bash
ros2 launch slam_system_manager system_bringup.launch.py
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/fast_lio_ouster_ws/install/setup.bash
ros2 topic hz /system/status
ros2 topic echo /system/status --once
```

The manager starts in `SENSOR_STARTING`, enters `SENSOR_READY` when both streams are alive and above
their configured minimum frequencies, then enters `WAIT_MODE`. It returns to `SENSOR_STARTING` if an
idle sensor becomes unhealthy.

## Map management

The manager creates the `map_root` configured in `config/system.yaml` and stores every managed map in
its own directory. New maps receive an atomic `metadata.yaml`; `map.pcd` is written by the mapping
adapter in Phase 5. Map names are restricted to 1-64 ASCII letters, digits, underscores, and hyphens,
starting with a letter or digit.

Services:

- `/system/get_map_list` (`slam_system_manager/srv/GetMapList`)
- `/system/create_map` (`slam_system_manager/srv/CreateMap`)
- `/system/delete_map` (`slam_system_manager/srv/DeleteMap`)
- `/system/load_map` (`slam_system_manager/srv/LoadMap`)

Once sensors are healthy and the map root has been scanned, the manager enters `WAIT_MODE`.

## Mapping control

Phase 5 adds asynchronous `/system/start_mapping`, `/system/stop_mapping`, and `/system/save_map`
services. Their response reports whether an operation was accepted; completion and failures are
reported through `/system/status`.

`MappingAdapter` copies the configured FAST-LIO template into the selected map directory, changes
only `map_file_path` and `pcd_save.pcd_save_en` in that runtime copy, and starts FAST-LIO with the
copy. Saving calls the YAML-configured `/map_save` Trigger service, verifies a non-empty `map.pcd`,
updates metadata, and stops Mapping. `stop_mapping` intentionally stops without saving.

See `docs/phase5_mapping_commands.md` for complete test commands and expected state transitions.

## Phase 8 system API and rosbridge

Phase 8 adds a synchronous `/system/get_status` snapshot service and an asynchronous
`/system/recover` Trigger service. Status publication and status queries use the same snapshot
builder. Recovery is accepted only from `ERROR`, stops residual Mapping and Localization process
groups, clears the failed SLAM session without deleting maps, and returns to `WAIT_MODE` when the
sensors are healthy or `SENSOR_STARTING` while they are unavailable.

The installed `rosbridge_smoke_test.py` verifies WebSocket connection, `/system/status`,
`/system/get_status`, `/system/get_map_list`, JSON field types, request IDs, timeout handling, and
disconnect/reconnect behavior. See `docs/phase8_rosbridge_commands.md` for the complete commands and
manual business-interface test matrix.

## Phase 9 systemd deployment

Phase 9 adds a non-interactive startup script, a tracked `slam_system.service` template, and a
separate `process_deployment.yaml`. The normal `process.yaml` remains safe for manual development;
the deployment profile auto-starts the Ouster at `172.168.1.3` and rosbridge. Process commands can
now define default YAML substitutions, and SystemManager skips managed Ouster auto-start when it
discovers an existing external point-cloud or IMU publisher.

The service uses `Restart=on-failure`, a five-second restart delay, SIGINT shutdown, systemd cgroup
cleanup, and journald logging. See `docs/phase9_systemd_commands.md` for installation, rollback, and
acceptance commands.
