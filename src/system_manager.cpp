#include "slam_system_manager/system_manager.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace slam_system_manager
{
namespace
{

constexpr auto kExternalSensorDiscoveryTimeout = std::chrono::seconds(2);
constexpr auto kExternalSensorDiscoveryPoll = std::chrono::milliseconds(100);

bool isMappingState(const SystemState state)
{
  return state == SystemState::MAPPING_STARTING || state == SystemState::MAPPING ||
         state == SystemState::MAP_SAVING;
}

bool isLocalizationState(const SystemState state)
{
  return state == SystemState::LOCALIZATION_STARTING ||
         state == SystemState::RELOCALIZING || state == SystemState::LOCALIZED;
}

bool isActiveSlamState(const SystemState state)
{
  return isMappingState(state) || isLocalizationState(state);
}

}  // namespace

SystemManager::SystemManager(const rclcpp::NodeOptions & options)
: Node("system_manager", options)
{
  const auto package_share =
    ament_index_cpp::get_package_share_directory("slam_system_manager");
  const auto default_config = package_share + "/config/system.yaml";
  const auto default_process_config = package_share + "/config/process.yaml";
  const auto default_sensor_config = package_share + "/config/sensor.yaml";
  const auto config_file = declare_parameter<std::string>("config_file", default_config);
  const auto process_config_file =
    declare_parameter<std::string>("process_config_file", default_process_config);
  const auto sensor_config_file =
    declare_parameter<std::string>("sensor_config_file", default_sensor_config);

  if (!transitionTo(SystemState::SYSTEM_CHECK)) {
    throw std::logic_error("Failed to enter SYSTEM_CHECK during startup");
  }

  try {
    config_manager_.load(config_file);
  } catch (const std::exception & exception) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      error_code_ = "CONFIG_LOAD_FAILED";
      error_message_ = exception.what();
    }
    transitionTo(SystemState::ERROR);
    RCLCPP_FATAL(get_logger(), "[CONFIG] %s", exception.what());
    throw;
  }

  const auto & config = config_manager_.get();
  status_publisher_ = create_publisher<msg::SystemStatus>(config.topics.system_status, 10);
  get_status_service_ = create_service<srv::GetSystemStatus>(
    "/system/get_status",
    std::bind(
      &SystemManager::handleGetStatus, this,
      std::placeholders::_1, std::placeholders::_2));
  recover_service_ = create_service<std_srvs::srv::Trigger>(
    "/system/recover",
    std::bind(
      &SystemManager::handleRecover, this,
      std::placeholders::_1, std::placeholders::_2));

  try {
    map_manager_ = std::make_unique<MapManager>(config.map_root, config.frames.map);
    const auto maps = map_manager_->listMaps();
    const auto map_root = map_manager_->mapRoot().string();
    RCLCPP_INFO(
      get_logger(), "[MAP] Found %zu map(s) under %s",
      maps.size(), map_root.c_str());
  } catch (const std::exception & exception) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      error_code_ = "MAP_ROOT_INIT_FAILED";
      error_message_ = exception.what();
    }
    transitionTo(SystemState::ERROR);
    RCLCPP_FATAL(get_logger(), "[MAP] %s", exception.what());
    throw;
  }

  get_map_list_service_ = create_service<srv::GetMapList>(
    "/system/get_map_list",
    std::bind(
      &SystemManager::handleGetMapList, this,
      std::placeholders::_1, std::placeholders::_2));
  create_map_service_ = create_service<srv::CreateMap>(
    "/system/create_map",
    std::bind(
      &SystemManager::handleCreateMap, this,
      std::placeholders::_1, std::placeholders::_2));
  delete_map_service_ = create_service<srv::DeleteMap>(
    "/system/delete_map",
    std::bind(
      &SystemManager::handleDeleteMap, this,
      std::placeholders::_1, std::placeholders::_2));
  load_map_service_ = create_service<srv::LoadMap>(
    "/system/load_map",
    std::bind(
      &SystemManager::handleLoadMap, this,
      std::placeholders::_1, std::placeholders::_2));

  try {
    process_manager_ = std::make_unique<ProcessManager>(get_logger());
    process_manager_->loadConfig(process_config_file);
    mapping_adapter_ = std::make_unique<MappingAdapter>(
      *this, *process_manager_, process_config_file);
    localization_adapter_ = std::make_unique<LocalizationAdapter>(
      *this, *process_manager_, process_config_file);
    relocalization_adapter_ = std::make_unique<RelocalizationAdapter>(
      *this, config,
      std::bind(
        &SystemManager::handleInitialPoseObserved, this, std::placeholders::_1),
      std::bind(&SystemManager::handleLocalizationConfirmed, this));
  } catch (const std::exception & exception) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      error_code_ = "PROCESS_CONFIG_LOAD_FAILED";
      error_message_ = exception.what();
    }
    transitionTo(SystemState::ERROR);
    RCLCPP_FATAL(get_logger(), "[PROCESS] %s", exception.what());
    throw;
  }

  start_mapping_service_ = create_service<srv::StartMapping>(
    "/system/start_mapping",
    std::bind(
      &SystemManager::handleStartMapping, this,
      std::placeholders::_1, std::placeholders::_2));
  stop_mapping_service_ = create_service<srv::StopMapping>(
    "/system/stop_mapping",
    std::bind(
      &SystemManager::handleStopMapping, this,
      std::placeholders::_1, std::placeholders::_2));
  save_map_service_ = create_service<srv::SaveMap>(
    "/system/save_map",
    std::bind(
      &SystemManager::handleSaveMap, this,
      std::placeholders::_1, std::placeholders::_2));
  start_localization_service_ = create_service<srv::StartLocalization>(
    "/system/start_localization",
    std::bind(
      &SystemManager::handleStartLocalization, this,
      std::placeholders::_1, std::placeholders::_2));
  stop_localization_service_ = create_service<srv::StopLocalization>(
    "/system/stop_localization",
    std::bind(
      &SystemManager::handleStopLocalization, this,
      std::placeholders::_1, std::placeholders::_2));
  set_initial_pose_service_ = create_service<srv::SetInitialPose>(
    "/system/set_initial_pose",
    std::bind(
      &SystemManager::handleSetInitialPose, this,
      std::placeholders::_1, std::placeholders::_2));

  std::string auto_start_error;
  std::vector<std::string> excluded_auto_start_processes;
  const auto auto_start_names = process_manager_->autoStartProcessNames();
  if (std::find(auto_start_names.begin(), auto_start_names.end(), "ouster") !=
    auto_start_names.end())
  {
    const auto discovery_deadline =
      std::chrono::steady_clock::now() + kExternalSensorDiscoveryTimeout;
    while (std::chrono::steady_clock::now() < discovery_deadline) {
      if (count_publishers(config.topics.pointcloud) > 0U ||
        count_publishers(config.topics.imu) > 0U)
      {
        excluded_auto_start_processes.push_back("ouster");
        RCLCPP_WARN(
          get_logger(),
          "[PROCESS] External Ouster publishers detected; skipping managed Ouster auto-start");
        break;
      }
      std::this_thread::sleep_for(kExternalSensorDiscoveryPoll);
    }
  }
  if (!process_manager_->startAutoStartProcesses(
      &auto_start_error, excluded_auto_start_processes))
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      error_code_ = "PROCESS_AUTO_START_FAILED";
      error_message_ = auto_start_error;
    }
    transitionTo(SystemState::ERROR);
    RCLCPP_ERROR(get_logger(), "[PROCESS] Auto-start failed: %s", auto_start_error.c_str());
  }

  try {
    health_monitor_ = std::make_unique<HealthMonitor>(
      *this, config.topics.pointcloud, config.topics.imu, sensor_config_file);
  } catch (const std::exception & exception) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      error_code_ = "SENSOR_CONFIG_LOAD_FAILED";
      error_message_ = exception.what();
    }
    transitionTo(SystemState::ERROR);
    RCLCPP_FATAL(get_logger(), "[SENSOR] %s", exception.what());
    throw;
  }

  bool system_check_passed = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (current_state_ == SystemState::SYSTEM_CHECK) {
      error_code_.clear();
      error_message_.clear();
      system_check_passed = true;
    }
  }
  if (system_check_passed) {
    transitionTo(SystemState::SENSOR_STARTING);
  }

  const auto sensor_period =
    std::chrono::duration<double>(1.0 / health_monitor_->config().health_check_hz);
  sensor_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(sensor_period),
    std::bind(&SystemManager::updateSensorState, this));
  process_timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&SystemManager::monitorProcesses, this));

  const auto period = std::chrono::duration<double>(1.0 / config.status_publish_hz);
  status_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&SystemManager::publishStatus, this));
  if (config.last_pose.enable) {
    const auto last_pose_period = std::chrono::duration<double>(
      config.last_pose.save_interval_sec);
    last_pose_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(last_pose_period),
      std::bind(&SystemManager::saveLastPose, this));
  }

  RCLCPP_INFO(
    get_logger(),
    "[CONFIG] Loaded %s; status topic=%s, rate=%.2f Hz",
    config_file.c_str(), config.topics.system_status.c_str(), config.status_publish_hz);
  RCLCPP_INFO(
    get_logger(),
    "[SYSTEM] System API ready; waiting for healthy sensors and a mode selection");
}

SystemManager::~SystemManager()
{
  shutting_down_.store(true);
  if (operation_thread_.joinable()) {
    operation_thread_.join();
  }
}

bool SystemManager::transitionTo(const SystemState next)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return transitionToLocked(next);
}

bool SystemManager::transitionToLocked(const SystemState next)
{
  if (!canTransition(current_state_, next)) {
    RCLCPP_ERROR(
      get_logger(), "[SYSTEM] Invalid state transition: %s -> %s",
      toString(current_state_).data(), toString(next).data());
    return false;
  }

  const auto previous = current_state_;
  current_state_ = next;
  RCLCPP_INFO(
    get_logger(), "[SYSTEM] %s -> %s",
    toString(previous).data(), toString(current_state_).data());
  return true;
}

msg::SystemStatus SystemManager::buildStatusMessage()
{
  msg::SystemStatus status;
  SensorStatus sensor_status;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status.state = std::string(toString(current_state_));
    status.current_map = current_map_;
    status.error_code = error_code_;
    status.error_message = error_message_;
    sensor_status = sensor_status_;
  }

  const bool managed_ouster_running =
    process_manager_ && process_manager_->isProcessRunning("ouster");
  status.ouster_running = managed_ouster_running ||
    sensor_status.pointcloud_alive || sensor_status.imu_alive;
  status.pointcloud_alive = sensor_status.pointcloud_alive;
  status.imu_alive = sensor_status.imu_alive;
  status.pointcloud_hz = static_cast<float>(sensor_status.pointcloud_hz);
  status.imu_hz = static_cast<float>(sensor_status.imu_hz);
  status.mapping_running = process_manager_ && process_manager_->isProcessRunning("mapping");
  status.localization_running = localization_adapter_ && localization_adapter_->isRunning();
  return status;
}

void SystemManager::launchOperation(std::function<void()> operation)
{
  if (operation_thread_.joinable()) {
    operation_thread_.join();
  }
  operation_thread_ = std::thread(
    [this, operation = std::move(operation)]() mutable {
      try {
        operation();
      } catch (const std::exception & exception) {
        setSystemError("SYSTEM_OPERATION_EXCEPTION", exception.what());
        RCLCPP_ERROR(get_logger(), "[SYSTEM] Background operation failed: %s", exception.what());
        if (!shutting_down_.load()) {
          transitionTo(SystemState::ERROR);
        }
      } catch (...) {
        setSystemError("SYSTEM_OPERATION_EXCEPTION", "Unknown background operation failure");
        RCLCPP_ERROR(get_logger(), "[SYSTEM] Background operation failed with an unknown exception");
        if (!shutting_down_.load()) {
          transitionTo(SystemState::ERROR);
        }
      }
      finishOperation();
    });
}

void SystemManager::finishOperation()
{
  std::lock_guard<std::mutex> lock(mutex_);
  operation_in_progress_ = false;
}

void SystemManager::setSystemError(const std::string & code, const std::string & message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  error_code_ = code;
  error_message_ = message;
}

void SystemManager::handleGetStatus(
  const std::shared_ptr<srv::GetSystemStatus::Request> request,
  std::shared_ptr<srv::GetSystemStatus::Response> response)
{
  (void)request;
  response->status = buildStatusMessage();
  response->success = true;
  response->message = "System status snapshot returned";
}

void SystemManager::handleRecover(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (current_state_ != SystemState::ERROR) {
      response->success = false;
      response->message = "Recovery is only valid in ERROR state";
      return;
    }
    if (operation_in_progress_) {
      response->success = false;
      response->message = "Another system operation is in progress";
      return;
    }
    if (!transitionToLocked(SystemState::RECOVERING)) {
      response->success = false;
      response->message = "Unable to enter RECOVERING state";
      return;
    }
    operation_in_progress_ = true;
  }

  response->success = true;
  response->message = "Recovery accepted; observe /system/status for completion";
  launchOperation(std::bind(&SystemManager::performRecovery, this));
}

void SystemManager::performRecovery()
{
  try {
    std::vector<std::string> stop_errors;
    if (mapping_adapter_ && mapping_adapter_->isRunning()) {
      std::string error;
      if (!mapping_adapter_->stop(&error) && mapping_adapter_->isRunning()) {
        stop_errors.push_back("mapping: " + error);
      }
    }
    if (localization_adapter_ && localization_adapter_->isRunning()) {
      std::string error;
      if (!localization_adapter_->stop(&error) && localization_adapter_->isRunning()) {
        stop_errors.push_back("localization: " + error);
      }
    }

    if (!stop_errors.empty()) {
      std::ostringstream message;
      for (std::size_t index = 0; index < stop_errors.size(); ++index) {
        if (index != 0U) {
          message << "; ";
        }
        message << stop_errors[index];
      }
      setSystemError("RECOVERY_FAILED", message.str());
      transitionTo(SystemState::ERROR);
      return;
    }

    if (relocalization_adapter_) {
      relocalization_adapter_->resetSession();
    }
    const auto sensor_status = health_monitor_ ? health_monitor_->getStatus() : SensorStatus{};
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (current_state_ != SystemState::RECOVERING) {
        return;
      }
      current_map_.clear();
      sensor_status_ = sensor_status;
      sensor_failure_started_at_.reset();
      error_code_.clear();
      error_message_.clear();
    }

    const auto target = sensor_status.sensor_ready ?
      SystemState::WAIT_MODE : SystemState::SENSOR_STARTING;
    if (!transitionTo(target)) {
      setSystemError("RECOVERY_FAILED", "State changed before recovery completed");
      transitionTo(SystemState::ERROR);
      return;
    }
    RCLCPP_INFO(
      get_logger(), "[SYSTEM] Recovery completed; sensor_ready=%s",
      sensor_status.sensor_ready ? "true" : "false");
  } catch (const std::exception & exception) {
    setSystemError("RECOVERY_FAILED", exception.what());
    transitionTo(SystemState::ERROR);
  } catch (...) {
    setSystemError("RECOVERY_FAILED", "Unknown recovery failure");
    transitionTo(SystemState::ERROR);
  }
}

void SystemManager::updateSensorState()
{
  if (shutting_down_.load() || !health_monitor_) {
    return;
  }
  const auto status = health_monitor_->getStatus();
  const auto now = std::chrono::steady_clock::now();
  const bool sensor_became_not_ready = previous_sensor_ready_ && !status.sensor_ready;

  if (status.pointcloud_alive != previous_pointcloud_alive_) {
    RCLCPP_INFO(
      get_logger(), "[SENSOR] Ouster PointCloud %s",
      status.pointcloud_alive ? "recovered" : "lost");
  }
  if (status.imu_alive != previous_imu_alive_) {
    RCLCPP_INFO(
      get_logger(), "[SENSOR] Ouster IMU %s",
      status.imu_alive ? "recovered" : "lost");
  }
  if (status.sensor_ready != previous_sensor_ready_) {
    RCLCPP_INFO(
      get_logger(), "[SENSOR] Sensor readiness changed to %s (cloud=%.1fHz, imu=%.1fHz)",
      status.sensor_ready ? "READY" : "NOT_READY",
      status.pointcloud_hz, status.imu_hz);
  }

  previous_pointcloud_alive_ = status.pointcloud_alive;
  previous_imu_alive_ = status.imu_alive;
  previous_sensor_ready_ = status.sensor_ready;

  SystemState state;
  double sensor_failure_duration_sec = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    sensor_status_ = status;
    state = current_state_;
    if (status.sensor_ready) {
      sensor_failure_started_at_.reset();
    } else {
      if (!sensor_failure_started_at_) {
        sensor_failure_started_at_ = now;
      }
      sensor_failure_duration_sec =
        std::chrono::duration<double>(now - *sensor_failure_started_at_).count();
    }
  }

  if (status.sensor_ready) {
    if (state == SystemState::SENSOR_STARTING) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        error_code_.clear();
        error_message_.clear();
      }
      transitionTo(SystemState::SENSOR_READY);
      state = SystemState::SENSOR_READY;
    }
    if (state == SystemState::SENSOR_READY && map_manager_) {
      transitionTo(SystemState::WAIT_MODE);
    }
    return;
  }

  const bool active_slam =
    state == SystemState::MAPPING_STARTING || state == SystemState::MAPPING ||
    state == SystemState::MAP_SAVING || state == SystemState::LOCALIZATION_STARTING ||
    state == SystemState::RELOCALIZING || state == SystemState::LOCALIZED;

  if (active_slam) {
    const double failure_grace_sec = health_monitor_->config().active_failure_grace_sec;
    if (sensor_failure_duration_sec < failure_grace_sec) {
      if (sensor_became_not_ready) {
        RCLCPP_WARN(
          get_logger(),
          "[SENSOR] Data timeout during active SLAM; waiting %.1fs before entering ERROR",
          failure_grace_sec);
      }
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!status.pointcloud_alive) {
        error_code_ = "SENSOR_NO_POINTCLOUD";
        error_message_ = "Ouster PointCloud2 stream timed out";
      } else if (!status.imu_alive) {
        error_code_ = "SENSOR_NO_IMU";
        error_message_ = "Ouster IMU stream timed out";
      } else {
        error_code_ = "SENSOR_RATE_LOW";
        error_message_ = "Ouster data rate is below the configured minimum";
      }
    }
    transitionTo(SystemState::ERROR);
    const bool mapping_running = mapping_adapter_ && mapping_adapter_->isRunning();
    const bool localization_running =
      localization_adapter_ && localization_adapter_->isRunning();
    bool stop_active_slam = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!operation_in_progress_ && (mapping_running || localization_running)) {
        operation_in_progress_ = true;
        stop_active_slam = true;
      }
    }
    if (stop_active_slam) {
      launchOperation(
        [this, mapping_running, localization_running]() {
          std::string stop_error;
          if (mapping_running && !mapping_adapter_->stop(&stop_error)) {
            RCLCPP_ERROR(
              get_logger(), "[MAPPING] Unable to stop after sensor failure: %s",
              stop_error.c_str());
          }
          if (localization_running && !localization_adapter_->stop(&stop_error)) {
            RCLCPP_ERROR(
              get_logger(), "[LOCALIZATION] Unable to stop after sensor failure: %s",
              stop_error.c_str());
          }
        });
    }
    return;
  }

  if (state == SystemState::SENSOR_READY || state == SystemState::WAIT_MODE) {
    transitionTo(SystemState::SENSOR_STARTING);
    state = SystemState::SENSOR_STARTING;
  }

  if (state == SystemState::SENSOR_STARTING && status.startup_grace_elapsed) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!status.pointcloud_alive) {
      error_code_ = "SENSOR_NO_POINTCLOUD";
      error_message_ = "Waiting for Ouster PointCloud2 data";
    } else if (!status.imu_alive) {
      error_code_ = "SENSOR_NO_IMU";
      error_message_ = "Waiting for Ouster IMU data";
    } else if (!status.pointcloud_rate_ok) {
      error_code_ = "SENSOR_POINTCLOUD_RATE_LOW";
      error_message_ = "PointCloud2 frequency is below the configured minimum";
    } else if (!status.imu_rate_ok) {
      error_code_ = "SENSOR_IMU_RATE_LOW";
      error_message_ = "IMU frequency is below the configured minimum";
    }
  }
}

void SystemManager::monitorProcesses()
{
  if (shutting_down_.load() || !mapping_adapter_ || !localization_adapter_) {
    return;
  }

  SystemState state;
  bool operation_in_progress = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state = current_state_;
    operation_in_progress = operation_in_progress_;
  }
  if (operation_in_progress) {
    return;
  }

  if (state == SystemState::MAPPING && !mapping_adapter_->isRunning()) {
    const auto exit_code = process_manager_->lastExitCode("mapping");
    const auto message = "Mapping process exited unexpectedly (exit_code=" +
      (exit_code ? std::to_string(*exit_code) : std::string("unknown")) + ")";
    setSystemError("MAPPING_PROCESS_EXITED", message);
    RCLCPP_ERROR(get_logger(), "[MAPPING] %s", message.c_str());
    transitionTo(SystemState::ERROR);
    return;
  }

  if ((state == SystemState::RELOCALIZING || state == SystemState::LOCALIZED) &&
    !localization_adapter_->isRunning())
  {
    const auto exit_code = process_manager_->lastExitCode("localization");
    const auto message = "Localization process exited unexpectedly (exit_code=" +
      (exit_code ? std::to_string(*exit_code) : std::string("unknown")) + ")";
    setSystemError("LOCALIZATION_PROCESS_EXITED", message);
    RCLCPP_ERROR(get_logger(), "[LOCALIZATION] %s", message.c_str());
    transitionTo(SystemState::ERROR);
  }
}

void SystemManager::handleInitialPoseObserved(
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose)
{
  (void)pose;
  SystemState state;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state = current_state_;
  }
  if (state != SystemState::RELOCALIZING && state != SystemState::LOCALIZED) {
    return;
  }

  relocalization_adapter_->resetSession();
  if (state == SystemState::LOCALIZED) {
    transitionTo(SystemState::RELOCALIZING);
  }
  RCLCPP_INFO(
    get_logger(), "[LOCALIZATION] Initial pose received; waiting for scan matching confirmation");
}

void SystemManager::handleLocalizationConfirmed()
{
  bool should_transition = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    should_transition = current_state_ == SystemState::RELOCALIZING;
  }
  if (!should_transition || !localization_adapter_->isRunning()) {
    return;
  }
  if (transitionTo(SystemState::LOCALIZED)) {
    relocalization_adapter_->setLocalized(true);
    RCLCPP_INFO(get_logger(), "[LOCALIZATION] Scan matching confirmed; localization is active");
  }
}

void SystemManager::saveLastPose()
{
  if (shutting_down_.load() || !relocalization_adapter_ || !map_manager_) {
    return;
  }

  std::string map_name;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (current_state_ != SystemState::LOCALIZED || current_map_.empty()) {
      return;
    }
    map_name = current_map_;
  }

  std::string error;
  const auto directory = map_manager_->getMapPath(map_name).parent_path();
  if (!relocalization_adapter_->requestLastPoseSave(directory, &error) &&
    error != "No trusted localized pose is available")
  {
    RCLCPP_ERROR(get_logger(), "[LOCALIZATION] Unable to queue Last Pose: %s", error.c_str());
  }
}

void SystemManager::handleGetMapList(
  const std::shared_ptr<srv::GetMapList::Request> request,
  std::shared_ptr<srv::GetMapList::Response> response)
{
  (void)request;
  try {
    const auto maps = map_manager_->listMaps();
    response->map_names.reserve(maps.size());
    for (const auto & map : maps) {
      response->map_names.push_back(map.name);
    }
    response->success = true;
    response->message = "Found " + std::to_string(maps.size()) + " map(s)";
  } catch (const std::exception & exception) {
    response->success = false;
    response->message = exception.what();
    RCLCPP_ERROR(get_logger(), "[MAP] Unable to list maps: %s", exception.what());
  }
}

void SystemManager::handleCreateMap(
  const std::shared_ptr<srv::CreateMap::Request> request,
  std::shared_ptr<srv::CreateMap::Response> response)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (isActiveSlamState(current_state_) || operation_in_progress_) {
      response->success = false;
      response->message = "Map directories cannot be changed while SLAM is active";
      return;
    }
    operation_in_progress_ = true;
  }
  try {
    map_manager_->createMap(request->map_name);
    response->success = true;
    response->map_path = map_manager_->getMapPath(request->map_name).string();
    response->message = "Map directory created";
    RCLCPP_INFO(get_logger(), "[MAP] Created %s", request->map_name.c_str());
  } catch (const std::invalid_argument & exception) {
    response->success = false;
    response->message = exception.what();
    RCLCPP_ERROR(get_logger(), "[MAP] Invalid map name '%s'", request->map_name.c_str());
  } catch (const std::exception & exception) {
    response->success = false;
    response->message = exception.what();
    RCLCPP_ERROR(get_logger(), "[MAP] Unable to create map: %s", exception.what());
  }
  finishOperation();
}

void SystemManager::handleDeleteMap(
  const std::shared_ptr<srv::DeleteMap::Request> request,
  std::shared_ptr<srv::DeleteMap::Response> response)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (isActiveSlamState(current_state_) || operation_in_progress_) {
      response->success = false;
      response->message = "Maps cannot be deleted while SLAM is active";
      return;
    }
    if (request->map_name == current_map_) {
      response->success = false;
      response->message = "Cannot delete the currently selected map";
      return;
    }
    operation_in_progress_ = true;
  }
  try {
    map_manager_->deleteMap(request->map_name);
    response->success = true;
    response->message = "Map deleted";
    RCLCPP_INFO(get_logger(), "[MAP] Deleted %s", request->map_name.c_str());
  } catch (const std::exception & exception) {
    response->success = false;
    response->message = exception.what();
    RCLCPP_ERROR(get_logger(), "[MAP] Unable to delete map: %s", exception.what());
  }
  finishOperation();
}

void SystemManager::handleLoadMap(
  const std::shared_ptr<srv::LoadMap::Request> request,
  std::shared_ptr<srv::LoadMap::Response> response)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (isActiveSlamState(current_state_) || operation_in_progress_) {
      response->success = false;
      response->message = "Maps cannot be loaded while SLAM is active";
      return;
    }
    operation_in_progress_ = true;
  }
  try {
    const auto map_path = map_manager_->loadMap(request->map_name);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      current_map_ = request->map_name;
    }
    response->success = true;
    response->map_path = map_path.string();
    response->message = "Map selected";
    RCLCPP_INFO(get_logger(), "[MAP] Loaded %s", request->map_name.c_str());
  } catch (const std::exception & exception) {
    response->success = false;
    response->message = exception.what();
    RCLCPP_ERROR(get_logger(), "[MAP] Unable to load map: %s", exception.what());
  }
  finishOperation();
}

void SystemManager::handleStartMapping(
  const std::shared_ptr<srv::StartMapping::Request> request,
  std::shared_ptr<srv::StartMapping::Response> response)
{
  if (!MapManager::isValidMapName(request->map_name)) {
    response->accepted = false;
    response->message = "Invalid map name";
    return;
  }
  if (localization_adapter_->isRunning()) {
    response->accepted = false;
    response->message = "Localization process is running";
    return;
  }
  if (mapping_adapter_->isRunning() || mapping_adapter_->saveServiceAvailable()) {
    response->accepted = false;
    response->message = "Mapping is already running or an external mapping service exists";
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if ((current_state_ != SystemState::WAIT_MODE &&
      current_state_ != SystemState::SENSOR_READY) || !sensor_status_.sensor_ready)
    {
      std::ostringstream message;
      message << "Mapping requires WAIT_MODE and healthy Ouster data"
              << " (state=" << toString(current_state_)
              << ", pointcloud_alive=" << std::boolalpha << sensor_status_.pointcloud_alive
              << ", imu_alive=" << sensor_status_.imu_alive
              << ", pointcloud_hz=" << sensor_status_.pointcloud_hz
              << ", imu_hz=" << sensor_status_.imu_hz << ")";
      response->accepted = false;
      response->message = message.str();
      return;
    }
    if (operation_in_progress_) {
      response->accepted = false;
      response->message = "Another system operation is in progress";
      return;
    }
    operation_in_progress_ = true;
  }

  bool created = false;
  std::filesystem::path map_path;
  try {
    if (!map_manager_->mapExists(request->map_name)) {
      map_manager_->createMap(request->map_name);
      created = true;
    }
    map_path = map_manager_->getMapPath(request->map_name);
    if (std::filesystem::is_regular_file(map_path)) {
      throw std::runtime_error("Refusing to overwrite an existing map.pcd");
    }
  } catch (const std::exception & exception) {
    finishOperation();
    response->accepted = false;
    response->message = exception.what();
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    current_map_ = request->map_name;
    error_code_.clear();
    error_message_.clear();
  }
  if (!transitionTo(SystemState::MAPPING_STARTING)) {
    if (created) {
      try {
        map_manager_->deleteMap(request->map_name);
      } catch (const std::exception &) {
      }
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      current_map_.clear();
    }
    finishOperation();
    response->accepted = false;
    response->message = "State changed before mapping could start";
    return;
  }

  response->accepted = true;
  response->message = "Mapping start accepted";
  const auto map_name = request->map_name;
  launchOperation(
    [this, map_name, map_path, created]() {
      RCLCPP_INFO(get_logger(), "[MAPPING] Starting map %s", map_name.c_str());
      std::string error;
      if (!mapping_adapter_->start(map_name, map_path, &error)) {
        setSystemError("MAPPING_START_FAILED", error);
        RCLCPP_ERROR(get_logger(), "[MAPPING] Start failed: %s", error.c_str());
        bool should_transition = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          should_transition = current_state_ == SystemState::MAPPING_STARTING;
          current_map_.clear();
        }
        if (created) {
          try {
            map_manager_->deleteMap(map_name);
          } catch (const std::exception & cleanup_error) {
            RCLCPP_ERROR(
              get_logger(), "[MAP] Unable to clean failed map directory: %s",
              cleanup_error.what());
          }
        }
        if (should_transition) {
          transitionTo(SystemState::ERROR);
        }
        return;
      }

      if (!transitionTo(SystemState::MAPPING)) {
        std::string stop_error;
        mapping_adapter_->stop(&stop_error);
        RCLCPP_ERROR(get_logger(), "[MAPPING] State changed while mapping started; process stopped");
        return;
      }
      RCLCPP_INFO(get_logger(), "[MAPPING] Mapping active for %s", map_name.c_str());
    });
}

void SystemManager::handleStopMapping(
  const std::shared_ptr<srv::StopMapping::Request> request,
  std::shared_ptr<srv::StopMapping::Response> response)
{
  (void)request;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (current_state_ != SystemState::MAPPING) {
      response->accepted = false;
      response->message = "Stop mapping is only valid in MAPPING state";
      return;
    }
    if (operation_in_progress_) {
      response->accepted = false;
      response->message = "Another system operation is in progress";
      return;
    }
    operation_in_progress_ = true;
  }

  response->accepted = true;
  response->message = "Mapping stop accepted; unsaved map data will be discarded";
  launchOperation(
    [this]() {
      RCLCPP_INFO(get_logger(), "[MAPPING] Stopping without saving");
      std::string error;
      if (!mapping_adapter_->stop(&error)) {
        setSystemError("MAPPING_STOP_FAILED", error);
        RCLCPP_ERROR(get_logger(), "[MAPPING] Stop failed: %s", error.c_str());
        transitionTo(SystemState::ERROR);
        return;
      }

      bool return_to_wait = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        return_to_wait = current_state_ == SystemState::MAPPING;
        current_map_.clear();
      }
      if (return_to_wait) {
        transitionTo(SystemState::WAIT_MODE);
      }
      RCLCPP_INFO(get_logger(), "[MAPPING] Stopped without saving");
    });
}

void SystemManager::handleSaveMap(
  const std::shared_ptr<srv::SaveMap::Request> request,
  std::shared_ptr<srv::SaveMap::Response> response)
{
  (void)request;
  std::string map_name;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (current_state_ != SystemState::MAPPING) {
      response->accepted = false;
      response->message = "Save map is only valid in MAPPING state";
      return;
    }
    if (operation_in_progress_) {
      response->accepted = false;
      response->message = "Another system operation is in progress";
      return;
    }
    operation_in_progress_ = true;
    map_name = current_map_;
  }

  if (!transitionTo(SystemState::MAP_SAVING)) {
    finishOperation();
    response->accepted = false;
    response->message = "State changed before map saving could start";
    return;
  }

  const auto map_path = map_manager_->getMapPath(map_name);
  response->accepted = true;
  response->message = "Map save accepted";
  launchOperation(
    [this, map_name, map_path]() {
      RCLCPP_INFO(get_logger(), "[MAP] Saving %s to %s", map_name.c_str(), map_path.c_str());
      std::string save_error;
      const bool saved = mapping_adapter_->saveMap(map_path, &save_error);
      if (saved) {
        try {
          map_manager_->markMapSaved(map_name);
        } catch (const std::exception & exception) {
          save_error = exception.what();
        }
      }

      std::string stop_error;
      const bool stopped = mapping_adapter_->stop(&stop_error);
      if (!saved || !save_error.empty()) {
        setSystemError("MAP_SAVE_FAILED", save_error);
        RCLCPP_ERROR(get_logger(), "[MAP] Save failed: %s", save_error.c_str());
      } else if (!stopped) {
        setSystemError("MAPPING_STOP_FAILED", stop_error);
        RCLCPP_ERROR(get_logger(), "[MAPPING] Stop after save failed: %s", stop_error.c_str());
      }

      bool saving_state = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        saving_state = current_state_ == SystemState::MAP_SAVING;
      }
      if (saved && save_error.empty() && stopped && saving_state) {
        transitionTo(SystemState::WAIT_MODE);
        RCLCPP_INFO(get_logger(), "[MAP] Saved %s", map_name.c_str());
      } else if (saving_state) {
        transitionTo(SystemState::ERROR);
      }
    });
}

void SystemManager::handleStartLocalization(
  const std::shared_ptr<srv::StartLocalization::Request> request,
  std::shared_ptr<srv::StartLocalization::Response> response)
{
  if (!MapManager::isValidMapName(request->map_name)) {
    response->accepted = false;
    response->message = "Invalid map name";
    return;
  }
  if (mapping_adapter_->isRunning() || mapping_adapter_->saveServiceAvailable()) {
    response->accepted = false;
    response->message = "Mapping is running or an external mapping service exists";
    return;
  }
  if (localization_adapter_->isRunning()) {
    response->accepted = false;
    response->message = "Localization is already running";
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if ((current_state_ != SystemState::WAIT_MODE &&
      current_state_ != SystemState::SENSOR_READY) || !sensor_status_.sensor_ready)
    {
      std::ostringstream message;
      message << "Localization requires WAIT_MODE and healthy Ouster data"
              << " (state=" << toString(current_state_)
              << ", pointcloud_alive=" << std::boolalpha << sensor_status_.pointcloud_alive
              << ", imu_alive=" << sensor_status_.imu_alive
              << ", pointcloud_hz=" << sensor_status_.pointcloud_hz
              << ", imu_hz=" << sensor_status_.imu_hz << ")";
      response->accepted = false;
      response->message = message.str();
      return;
    }
    if (operation_in_progress_) {
      response->accepted = false;
      response->message = "Another system operation is in progress";
      return;
    }
    operation_in_progress_ = true;
  }

  std::filesystem::path map_path;
  try {
    map_path = map_manager_->loadMap(request->map_name);
  } catch (const std::exception & exception) {
    finishOperation();
    response->accepted = false;
    response->message = exception.what();
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    current_map_ = request->map_name;
    error_code_.clear();
    error_message_.clear();
  }
  relocalization_adapter_->resetSession();
  if (!transitionTo(SystemState::LOCALIZATION_STARTING)) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      current_map_.clear();
    }
    finishOperation();
    response->accepted = false;
    response->message = "State changed before localization could start";
    return;
  }

  response->accepted = true;
  response->message = "Localization start accepted";
  const auto map_name = request->map_name;
  launchOperation(
    [this, map_name, map_path]() {
      RCLCPP_INFO(
        get_logger(), "[LOCALIZATION] Starting map %s from %s",
        map_name.c_str(), map_path.c_str());
      std::string error;
      if (!localization_adapter_->start(map_name, map_path, &error)) {
        setSystemError("LOCALIZATION_START_FAILED", error);
        RCLCPP_ERROR(get_logger(), "[LOCALIZATION] Start failed: %s", error.c_str());
        bool should_transition = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          should_transition = current_state_ == SystemState::LOCALIZATION_STARTING;
          current_map_.clear();
        }
        if (should_transition) {
          transitionTo(SystemState::ERROR);
        }
        return;
      }

      if (!transitionTo(SystemState::RELOCALIZING)) {
        std::string stop_error;
        localization_adapter_->stop(&stop_error);
        RCLCPP_ERROR(
          get_logger(),
          "[LOCALIZATION] State changed while localization started; process stopped");
        return;
      }
      RCLCPP_INFO(
        get_logger(),
        "[LOCALIZATION] Map loaded; preparing initial pose for %s",
        map_name.c_str());

      const auto & relocalization = config_manager_.get().relocalization;
      if (!config_manager_.get().last_pose.enable) {
        RCLCPP_INFO(get_logger(), "[LOCALIZATION] Waiting for manual initial pose");
        return;
      }

      std::string last_pose_error;
      const auto last_pose = relocalization_adapter_->loadLastPose(
        map_path.parent_path(), &last_pose_error);
      if (!last_pose) {
        if (last_pose_error.find("not found") != std::string::npos) {
          RCLCPP_INFO(get_logger(), "[LOCALIZATION] No Last Pose; waiting for manual initial pose");
        } else {
          RCLCPP_WARN(
            get_logger(), "[LOCALIZATION] Last Pose unavailable: %s",
            last_pose_error.c_str());
        }
        return;
      }

      std::this_thread::sleep_for(
        std::chrono::duration<double>(relocalization.initial_pose_settle_sec));
      if (!relocalization_adapter_->waitForInitialPoseConsumer(
          relocalization.initial_pose_wait_timeout_sec))
      {
        RCLCPP_WARN(
          get_logger(),
          "[LOCALIZATION] Initial pose consumer is not ready; waiting for manual initialization");
        return;
      }

      for (std::size_t attempt = 1U; attempt <= relocalization.last_pose_retry_count; ++attempt) {
        std::string publish_error;
        if (!relocalization_adapter_->publishInitialPose(*last_pose, &publish_error)) {
          RCLCPP_WARN(
            get_logger(), "[LOCALIZATION] Unable to apply Last Pose: %s",
            publish_error.c_str());
          return;
        }
        RCLCPP_INFO(
          get_logger(), "[LOCALIZATION] Last Pose published for %s (attempt %zu/%zu)",
          map_name.c_str(), attempt, relocalization.last_pose_retry_count);

        const auto deadline = std::chrono::steady_clock::now() +
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(relocalization.last_pose_retry_interval_sec));
        while (std::chrono::steady_clock::now() < deadline) {
          if (relocalization_adapter_->localizationConfirmed()) {
            return;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
      }
      RCLCPP_WARN(
        get_logger(), "[LOCALIZATION] Last Pose did not localize; waiting for manual initial pose");
    });
}

void SystemManager::handleStopLocalization(
  const std::shared_ptr<srv::StopLocalization::Request> request,
  std::shared_ptr<srv::StopLocalization::Response> response)
{
  (void)request;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!isLocalizationState(current_state_)) {
      response->accepted = false;
      response->message = "Stop localization requires an active localization state";
      return;
    }
    if (operation_in_progress_) {
      response->accepted = false;
      response->message = "Another system operation is in progress";
      return;
    }
    operation_in_progress_ = true;
  }

  response->accepted = true;
  response->message = "Localization stop accepted";
  launchOperation(
    [this]() {
      RCLCPP_INFO(get_logger(), "[LOCALIZATION] Stopping");
      saveLastPose();
      std::string error;
      if (!localization_adapter_->stop(&error)) {
        setSystemError("LOCALIZATION_STOP_FAILED", error);
        RCLCPP_ERROR(get_logger(), "[LOCALIZATION] Stop failed: %s", error.c_str());
        transitionTo(SystemState::ERROR);
        return;
      }

      bool return_to_wait = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        return_to_wait = isLocalizationState(current_state_);
        current_map_.clear();
      }
      if (return_to_wait) {
        transitionTo(SystemState::WAIT_MODE);
      }
      relocalization_adapter_->resetSession();
      RCLCPP_INFO(get_logger(), "[LOCALIZATION] Stopped");
    });
}

void SystemManager::handleSetInitialPose(
  const std::shared_ptr<srv::SetInitialPose::Request> request,
  std::shared_ptr<srv::SetInitialPose::Response> response)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (current_state_ != SystemState::RELOCALIZING &&
      current_state_ != SystemState::LOCALIZED)
    {
      response->accepted = false;
      response->message = "Initial pose requires RELOCALIZING or LOCALIZED state";
      return;
    }
    if (operation_in_progress_) {
      response->accepted = false;
      response->message = "Another system operation is in progress";
      return;
    }
  }

  std::string error;
  if (!relocalization_adapter_->publishInitialPose(request->pose, &error)) {
    response->accepted = false;
    response->message = error;
    return;
  }
  response->accepted = true;
  response->message = "Initial pose published; waiting for localization confirmation";
}

void SystemManager::publishStatus()
{
  status_publisher_->publish(buildStatusMessage());
}

}  // namespace slam_system_manager
