#ifndef SLAM_SYSTEM_MANAGER__SYSTEM_MANAGER_HPP_
#define SLAM_SYSTEM_MANAGER__SYSTEM_MANAGER_HPP_

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "slam_system_manager/config_manager.hpp"
#include "slam_system_manager/health_monitor.hpp"
#include "slam_system_manager/localization_adapter.hpp"
#include "slam_system_manager/map_manager.hpp"
#include "slam_system_manager/mapping_adapter.hpp"
#include "slam_system_manager/msg/system_status.hpp"
#include "slam_system_manager/process_manager.hpp"
#include "slam_system_manager/relocalization_adapter.hpp"
#include "slam_system_manager/srv/create_map.hpp"
#include "slam_system_manager/srv/delete_map.hpp"
#include "slam_system_manager/srv/get_map_list.hpp"
#include "slam_system_manager/srv/get_system_status.hpp"
#include "slam_system_manager/srv/load_map.hpp"
#include "slam_system_manager/srv/save_map.hpp"
#include "slam_system_manager/srv/set_initial_pose.hpp"
#include "slam_system_manager/srv/start_mapping.hpp"
#include "slam_system_manager/srv/stop_mapping.hpp"
#include "slam_system_manager/srv/start_localization.hpp"
#include "slam_system_manager/srv/stop_localization.hpp"
#include "slam_system_manager/system_state.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace slam_system_manager
{

class SystemManager : public rclcpp::Node
{
public:
  explicit SystemManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SystemManager() override;

private:
  bool transitionTo(SystemState next);
  bool transitionToLocked(SystemState next);
  msg::SystemStatus buildStatusMessage();
  void updateSensorState();
  void monitorProcesses();
  void publishStatus();
  void saveLastPose();
  void handleInitialPoseObserved(
    const geometry_msgs::msg::PoseWithCovarianceStamped & pose);
  void handleLocalizationConfirmed();
  void launchOperation(std::function<void()> operation);
  void finishOperation();
  void setSystemError(const std::string & code, const std::string & message);
  void handleGetStatus(
    const std::shared_ptr<srv::GetSystemStatus::Request> request,
    std::shared_ptr<srv::GetSystemStatus::Response> response);
  void handleRecover(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void performRecovery();
  void handleGetMapList(
    const std::shared_ptr<srv::GetMapList::Request> request,
    std::shared_ptr<srv::GetMapList::Response> response);
  void handleCreateMap(
    const std::shared_ptr<srv::CreateMap::Request> request,
    std::shared_ptr<srv::CreateMap::Response> response);
  void handleDeleteMap(
    const std::shared_ptr<srv::DeleteMap::Request> request,
    std::shared_ptr<srv::DeleteMap::Response> response);
  void handleLoadMap(
    const std::shared_ptr<srv::LoadMap::Request> request,
    std::shared_ptr<srv::LoadMap::Response> response);
  void handleStartMapping(
    const std::shared_ptr<srv::StartMapping::Request> request,
    std::shared_ptr<srv::StartMapping::Response> response);
  void handleStopMapping(
    const std::shared_ptr<srv::StopMapping::Request> request,
    std::shared_ptr<srv::StopMapping::Response> response);
  void handleSaveMap(
    const std::shared_ptr<srv::SaveMap::Request> request,
    std::shared_ptr<srv::SaveMap::Response> response);
  void handleStartLocalization(
    const std::shared_ptr<srv::StartLocalization::Request> request,
    std::shared_ptr<srv::StartLocalization::Response> response);
  void handleStopLocalization(
    const std::shared_ptr<srv::StopLocalization::Request> request,
    std::shared_ptr<srv::StopLocalization::Response> response);
  void handleSetInitialPose(
    const std::shared_ptr<srv::SetInitialPose::Request> request,
    std::shared_ptr<srv::SetInitialPose::Response> response);

  mutable std::mutex mutex_;
  SystemState current_state_{SystemState::BOOT};
  std::string current_map_;
  std::string error_code_;
  std::string error_message_;

  ConfigManager config_manager_;
  std::unique_ptr<ProcessManager> process_manager_;
  std::unique_ptr<HealthMonitor> health_monitor_;
  std::unique_ptr<MapManager> map_manager_;
  std::unique_ptr<MappingAdapter> mapping_adapter_;
  std::unique_ptr<LocalizationAdapter> localization_adapter_;
  std::unique_ptr<RelocalizationAdapter> relocalization_adapter_;
  SensorStatus sensor_status_;
  bool operation_in_progress_{false};
  bool previous_pointcloud_alive_{false};
  bool previous_imu_alive_{false};
  bool previous_sensor_ready_{false};
  std::optional<std::chrono::steady_clock::time_point> sensor_failure_started_at_;
  rclcpp::Publisher<msg::SystemStatus>::SharedPtr status_publisher_;
  rclcpp::Service<srv::GetSystemStatus>::SharedPtr get_status_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr recover_service_;
  rclcpp::Service<srv::GetMapList>::SharedPtr get_map_list_service_;
  rclcpp::Service<srv::CreateMap>::SharedPtr create_map_service_;
  rclcpp::Service<srv::DeleteMap>::SharedPtr delete_map_service_;
  rclcpp::Service<srv::LoadMap>::SharedPtr load_map_service_;
  rclcpp::Service<srv::StartMapping>::SharedPtr start_mapping_service_;
  rclcpp::Service<srv::StopMapping>::SharedPtr stop_mapping_service_;
  rclcpp::Service<srv::SaveMap>::SharedPtr save_map_service_;
  rclcpp::Service<srv::StartLocalization>::SharedPtr start_localization_service_;
  rclcpp::Service<srv::StopLocalization>::SharedPtr stop_localization_service_;
  rclcpp::Service<srv::SetInitialPose>::SharedPtr set_initial_pose_service_;
  rclcpp::TimerBase::SharedPtr sensor_timer_;
  rclcpp::TimerBase::SharedPtr process_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr last_pose_timer_;
  std::thread operation_thread_;
  std::atomic<bool> shutting_down_{false};
};

}  // namespace slam_system_manager

#endif  // SLAM_SYSTEM_MANAGER__SYSTEM_MANAGER_HPP_
